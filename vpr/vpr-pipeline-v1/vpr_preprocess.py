#!/usr/bin/env python3
"""Build a VPR feature database from a ROS1 bag (step 1 of the live pipeline).

This script is the first half of the actual (non-evaluation) VPR pipeline.
It merges:
  * ROS1 bag parsing + distance-sampled image extraction from
    ``build_database_3d.py`` (poses are read with
    ``load_poses_from_imgpose_file``).
  * Global descriptor extraction from ``pipeline_3d.py`` (Stage 1).

Output (under ``--output_dir``)::

    database_descriptors.npy      # (N, D) float32 feature database
    database_poses.npy            # (N, 4) float64 -- [x, y, z, ts_ns]
    database_image_paths.txt      # one path per line (relative if the image
                                  # cache lives inside output_dir, absolute
                                  # otherwise)
    database_meta.npz             # method, descriptor_dim, num_database

By default, the extracted JPGs are written under ``<output_dir>/database/``
so the directory is self-contained. For larger deployments where you don't
want the keep the per-frame images next to the (tiny) descriptor files, pass
``--image_cache_dir`` to redirect them elsewhere (e.g. ``~/.vpr/<scene>``).
The downstream detection module only needs the four files above.

Example
-------
    python vpr_preprocess.py \\
        --bag_path data/all_2024.bag \\
        --pose_file data/ImgPose.txt \\
        --image_topic /camera_agent/img_left/compressed \\
        --undistort_fisheye \\
        --db_distance 1.0 \\
        --method salad-indoor \\
        --output_dir datasets/scene_A
"""

import argparse
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime
from glob import glob

import numpy as np
import torch
import torchvision.transforms as transforms
from loguru import logger
from PIL import Image
from torch.utils.data import DataLoader, Dataset
from tqdm import tqdm

from build_database_3d import (
    load_poses_from_imgpose_file,
    save_compressed_bytes,
    undistort_and_save,
)
from model_loader import get_model
from undistort import PolyFisheyeUndistorter


# ---------------------------------------------------------------------------
# Database-only distance sampler (no query split)
# ---------------------------------------------------------------------------

class DatabaseSampler:
    """Keep one frame per ``db_distance`` meters of horizontal motion."""

    def __init__(self, db_distance=1.0):
        self.db_distance = db_distance
        self.last_x = None
        self.last_y = None
        self.kept = 0
        self.skipped = 0

    def keep(self, x, y):
        if self.last_x is None:
            self.last_x, self.last_y = x, y
            self.kept += 1
            return True
        dist = float(np.hypot(x - self.last_x, y - self.last_y))
        if dist >= self.db_distance:
            self.last_x, self.last_y = x, y
            self.kept += 1
            return True
        self.skipped += 1
        return False


# ---------------------------------------------------------------------------
# Step 1a: extract distance-sampled database images from a ROS1 bag
# ---------------------------------------------------------------------------

def extract_database_from_ros1(args):
    """Parse a ROS1 bag + ImgPose.txt and write sampled JPGs to disk."""
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore

    # main() guarantees args.image_cache_dir is set (defaults to
    # <output_dir>/database for backward compatibility)
    db_dir = args.image_cache_dir
    os.makedirs(db_dir, exist_ok=True)

    poses = load_poses_from_imgpose_file(args.pose_file, prefix="left")
    logger.info(f"Loaded {len(poses)} left-frame poses from {args.pose_file}")

    undistorter = None
    if args.undistort_fisheye:
        if args.calibration_file:
            undistorter = PolyFisheyeUndistorter.from_yaml(
                args.calibration_file,
                out_w=args.out_width, out_h=args.out_height, fov_deg=args.fov,
            )
        else:
            undistorter = PolyFisheyeUndistorter.from_defaults(
                out_w=args.out_width, out_h=args.out_height, fov_deg=args.fov,
            )
        logger.info(
            f"Fisheye undistortion enabled "
            f"({args.out_width}x{args.out_height}, {args.fov}° FOV)"
        )

    typestore = get_typestore(Stores.ROS1_NOETIC)
    sampler = DatabaseSampler(args.db_distance)
    raw_count = 0
    unmatched = 0
    futures = []
    t0 = time.time()

    with Reader(args.bag_path) as reader, \
         ProcessPoolExecutor(max_workers=args.num_workers) as pool:

        conns = [c for c in reader.connections if c.topic == args.image_topic]
        if not conns:
            raise ValueError(
                f"Topic '{args.image_topic}' not found in {args.bag_path}"
            )
        logger.info(f"Reading {args.image_topic} from {args.bag_path}")

        for conn, timestamp, rawdata in reader.messages(connections=conns):
            msg = typestore.deserialize_ros1(rawdata, conn.msgtype)
            # 帧号与 ImgPose.txt 中 left_* 的顺序一一对应（与 build_database_3d 保持一致）
            frame_index = raw_count
            raw_count += 1

            if frame_index >= len(poses):
                unmatched += 1
                continue

            x, y, z, _pose_ts = poses[frame_index]
            if not sampler.keep(x, y):
                continue

            fname = f"@{x:.6f}@{y:.6f}@{z:.6f}@{timestamp}@.jpg"
            path = os.path.join(db_dir, fname)
            img_data = bytes(msg.data)

            if undistorter is not None:
                futures.append(
                    pool.submit(undistort_and_save, img_data, undistorter, path)
                )
            else:
                futures.append(
                    pool.submit(save_compressed_bytes, img_data, path)
                )

            if raw_count % 200 == 0:
                rate = raw_count / (time.time() - t0)
                logger.info(
                    f"  Read {raw_count} frames "
                    f"(kept={sampler.kept}, skipped={sampler.skipped}) "
                    f"[{rate:.0f} fr/s]"
                )

        logger.info(f"  Waiting for {len(futures)} image saves ...")
        for f in as_completed(futures):
            f.result()

    if raw_count != len(poses):
        logger.warning(
            f"Frame count mismatch: bag has {raw_count} frames, "
            f"pose file has {len(poses)} left poses."
        )

    elapsed = time.time() - t0
    logger.info(f"Image extraction done in {elapsed:.1f}s.")
    logger.info(f"  Kept (database):  {sampler.kept} -> {db_dir}")
    logger.info(f"  Skipped:          {sampler.skipped}")
    if unmatched:
        logger.info(f"  Unmatched frames: {unmatched}")

    return db_dir


# ---------------------------------------------------------------------------
# Step 1b: extract global descriptors over the saved database images
# ---------------------------------------------------------------------------

class _DatabaseImageFolder(Dataset):
    """Minimal dataset that mirrors the transform used in ``VPRDataset``."""

    def __init__(self, image_paths, image_size=None):
        self.image_paths = image_paths
        ops = [
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
            ),
        ]
        if image_size:
            ops.append(transforms.Resize(size=image_size, antialias=True))
        self.transform = transforms.Compose(ops)

    def __getitem__(self, index):
        img = Image.open(self.image_paths[index]).convert("RGB")
        return self.transform(img), index

    def __len__(self):
        return len(self.image_paths)


def _parse_pose_from_filename(image_path):
    """Recover (x, y, z, timestamp_ns) from an @x@y@z@ts@.jpg filename."""
    parts = os.path.basename(image_path).split("@")
    # parts == ['', x, y, z, ts, '.jpg']
    x = float(parts[1])
    y = float(parts[2])
    z = float(parts[3])
    ts = int(parts[4])
    return x, y, z, ts


def extract_descriptors(args, db_dir):
    """Run the VPR model over ``db_dir`` and persist descriptors + metadata."""
    image_paths = sorted(
        p for p in glob(f"{db_dir}/*")
        if os.path.isfile(p)
        and os.path.splitext(p)[1].lower() in (".jpg", ".jpeg", ".png")
    )
    if not image_paths:
        raise RuntimeError(f"No images found in {db_dir}")
    logger.info(f"Found {len(image_paths)} database images in {db_dir}")

    poses = np.empty((len(image_paths), 4), dtype=np.float64)
    for i, p in enumerate(image_paths):
        x, y, z, ts = _parse_pose_from_filename(p)
        poses[i] = (x, y, z, ts)

    logger.info(f"Loading VPR model: method={args.method}, device={args.device}")
    model, descriptor_dim = get_model(args.method, args.device)

    dataset = _DatabaseImageFolder(image_paths, image_size=args.image_size)
    loader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        num_workers=args.num_workers,
        shuffle=False,
        pin_memory=(args.device == "cuda"),
    )

    descriptors = np.empty((len(image_paths), descriptor_dim), dtype="float32")
    t0 = time.time()
    with torch.inference_mode():
        for images, indices in tqdm(loader, desc="Database descriptors"):
            out = model(images.to(args.device)).cpu().numpy()
            descriptors[indices.numpy(), :] = out
    logger.info(
        f"Descriptor extraction done in {time.time() - t0:.1f}s "
        f"({len(image_paths)} images, dim={descriptor_dim})"
    )

    descriptors_path = os.path.join(args.output_dir, "database_descriptors.npy")
    poses_path = os.path.join(args.output_dir, "database_poses.npy")
    paths_path = os.path.join(args.output_dir, "database_image_paths.txt")
    meta_path = os.path.join(args.output_dir, "database_meta.npz")
    info_path = os.path.join(args.output_dir, "database_build_info.json")

    np.save(descriptors_path, descriptors)
    np.save(poses_path, poses)
    # 写图片路径：缓存若在 output_dir 内则写相对路径（方便整目录搬迁），
    # 否则写绝对路径（缓存可能在 ~/.vpr/... 等独立位置）
    abs_out = os.path.abspath(args.output_dir)
    with open(paths_path, "w") as f:
        for p in image_paths:
            abs_p = os.path.abspath(p)
            if abs_p == abs_out or abs_p.startswith(abs_out + os.sep):
                f.write(os.path.relpath(abs_p, abs_out) + "\n")
            else:
                f.write(abs_p + "\n")
    np.savez(
        meta_path,
        method=np.array(args.method),
        descriptor_dim=np.array(descriptor_dim, dtype=np.int64),
        num_database=np.array(len(image_paths), dtype=np.int64),
        image_size=np.array(args.image_size if args.image_size else [], dtype=np.int64),
    )

    # 人类可读 + 跨语言可读的构造信息（C++ GUI 等下游消费者用）
    build_info = {
        "built_at":          datetime.now().isoformat(timespec="seconds"),
        "method":            args.method,
        "descriptor_dim":    int(descriptor_dim),
        "num_database":      int(len(image_paths)),
        "image_size":        list(args.image_size) if args.image_size else None,
        "output_dir":        args.output_dir,
        "image_cache_dir":   args.image_cache_dir,
        "bag_path":          args.bag_path,
        "pose_file":         args.pose_file,
        "image_topic":       args.image_topic,
        "db_distance":       float(args.db_distance),
        "undistort_fisheye": bool(args.undistort_fisheye),
        "calibration_file":  args.calibration_file,
        "out_width":         int(args.out_width),
        "out_height":        int(args.out_height),
        "fov":               float(args.fov),
        "device":            args.device,
        "batch_size":        int(args.batch_size),
        "num_workers":       int(args.num_workers),
        "xy_min":            poses[:, :2].min(axis=0).tolist(),
        "xy_max":            poses[:, :2].max(axis=0).tolist(),
        "z_min":             float(poses[:, 2].min()),
        "z_max":             float(poses[:, 2].max()),
        "descriptor_extraction_seconds": float(time.time() - t0),
    }
    with open(info_path, "w") as f:
        json.dump(build_info, f, indent=2, ensure_ascii=False)

    logger.info(f"Saved descriptors -> {descriptors_path} (shape={descriptors.shape})")
    logger.info(f"Saved poses       -> {poses_path} (shape={poses.shape})")
    logger.info(f"Saved image paths -> {paths_path}")
    logger.info(f"Saved metadata    -> {meta_path}")
    logger.info(f"Saved build info  -> {info_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description="Build a VPR feature database (.npy) from a ROS1 bag.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # Input
    parser.add_argument("--bag_path", type=str, required=True,
                        help="Path to the ROS1 .bag file")
    parser.add_argument("--pose_file", type=str, required=True,
                        help="Path to ImgPose.txt (parsed with load_poses_from_imgpose_file)")
    parser.add_argument("--image_topic", type=str,
                        default="/camera_agent/img_left/compressed",
                        help="ROS1 image topic to read")

    # Output
    parser.add_argument("--output_dir", type=str, required=True,
                        help="Directory to write the descriptor database "
                             "(.npy/.npz/.txt/.log)")
    parser.add_argument("--image_cache_dir", type=str, default=None,
                        help="Directory for the extracted JPGs. "
                             "Default: <output_dir>/database. "
                             "Pass a path outside output_dir "
                             "(e.g. ~/.vpr/<scene>) to keep the descriptor "
                             "database lean.")

    # Sampling
    parser.add_argument("--db_distance", type=float, default=1.0,
                        help="Min XY distance (m) between database keyframes")

    # Fisheye undistortion (forwarded to PolyFisheyeUndistorter)
    parser.add_argument("--undistort_fisheye", action="store_true",
                        help="Apply POLYFISHEYE -> perspective undistortion")
    parser.add_argument("--calibration_file", type=str, default=None,
                        help="Path to calibration.yaml (uses S20 defaults if unset)")
    parser.add_argument("--out_width", type=int, default=1024)
    parser.add_argument("--out_height", type=int, default=768)
    parser.add_argument("--fov", type=float, default=90,
                        help="Output perspective FOV in degrees")

    # Descriptor extraction
    parser.add_argument("--method", type=str, default="megaloc",
                        choices=["megaloc", "salad-indoor"],
                        help="VPR model used to build the feature database")
    parser.add_argument("--device", type=str, default="cuda",
                        choices=["cuda", "cpu"])
    parser.add_argument("--batch_size", type=int, default=4)
    parser.add_argument("--image_size", type=int, nargs="+", default=None,
                        help="Optional resize (H W) before inference")

    # Performance
    parser.add_argument("--num_workers", type=int, default=4,
                        help="Workers for image saving and DataLoader")

    # Workflow
    parser.add_argument("--skip_extraction", action="store_true",
                        help="Reuse an existing <output_dir>/database/ folder "
                             "and only re-run descriptor extraction")

    return parser.parse_args()


def _configure_logging(output_dir):
    logger.remove()
    logger.add(
        sys.stdout, colorize=True,
        format="<green>{time:%Y-%m-%d %H:%M:%S}</green> {message}",
        level="INFO",
    )
    log_path = os.path.join(output_dir, "vpr_preprocess.log")
    logger.add(log_path, level="DEBUG",
               format="{time:%Y-%m-%d %H:%M:%S} | {level} | {message}")
    logger.info(f"Logging to {log_path}")


def main():
    args = parse_args()
    args.output_dir = os.path.abspath(os.path.expanduser(args.output_dir))
    os.makedirs(args.output_dir, exist_ok=True)
    _configure_logging(args.output_dir)

    # 解析图片缓存目录：未指定时回退到 <output_dir>/database (向后兼容)
    if args.image_cache_dir:
        args.image_cache_dir = os.path.abspath(
            os.path.expanduser(args.image_cache_dir)
        )
    else:
        args.image_cache_dir = os.path.join(args.output_dir, "database")

    logger.info(f"Arguments: {vars(args)}")
    logger.info(f"Image cache dir : {args.image_cache_dir}")
    logger.info(f"Descriptor dir  : {args.output_dir}")

    if args.skip_extraction:
        if not os.path.isdir(args.image_cache_dir):
            raise FileNotFoundError(
                f"--skip_extraction set but {args.image_cache_dir} does not exist"
            )
        logger.info(f"Skipping image extraction; reusing {args.image_cache_dir}")
        db_dir = args.image_cache_dir
    else:
        db_dir = extract_database_from_ros1(args)

    extract_descriptors(args, db_dir)
    logger.info("VPR preprocessing complete.")


if __name__ == "__main__":
    main()
