#!/usr/bin/env python3
"""Build a VPR feature database from a ROS bag (step 1 of the live pipeline).

This script is the first half of the actual (non-evaluation) VPR pipeline.
It currently supports two input sources, selected via ``--source``:

* ``ros1`` (default, SHARE SLAM S20) -- read images from a ROS1 ``.bag``
  and poses from ``ImgPose.txt``; fisheye undistortion is optional and
  uses :class:`PolyFisheyeUndistorter`.
* ``ros2_r1pro`` -- read images and poses from a ROS2 bag (sqlite3 .db3
  directory): images from a ``sensor_msgs/CompressedImage`` topic
  (default ``/hdas/camera_head/left_raw/image_raw_color/compressed``)
  matched in time against ``nav_msgs/Odometry`` poses (default
  ``/Odometry``). Pinhole rational undistortion is always applied with
  :class:`PinHoleUndistorter`, followed by a center crop down to
  ``--crop_width x --crop_height`` (default 1024x768). The crop matches
  ``vpr_bootstrap._VPR_QUERY_TARGET_W/H`` so that live query frames go
  through the *same* geometric pipeline as the database frames.

Both pipelines share Stage 1 (global descriptor extraction); the
offline evaluation counterpart lives in ``eval/pipeline_3d.py``.

Output (under ``--output_dir``)::

    database_descriptors.npy      # (N, D) float32 feature database
    database_poses.npy            # (N, 4) float64 -- [x, y, z, ts_ns]
    database_image_paths.txt      # one path per line (relative if the image
                                  # cache lives inside output_dir, absolute
                                  # otherwise)
    database_meta.npz             # method, descriptor_dim, num_database
    database_build_info.json      # full build provenance (consumed by
                                  # vpr_detect.py to validate undistortion
                                  # consistency across submap databases)

By default the extracted JPGs are written under ``<output_dir>/database/``.
Pass ``--image_cache_dir`` to redirect them elsewhere (e.g. ``~/.vpr/<scene>``)
when you don't want the per-frame images next to the (tiny) descriptor files.
The downstream detection module only needs the files above.

Examples
--------
    # S20 fisheye / ROS1 bag
    python vpr_preprocess.py \\
        --source ros1 \\
        --bag_path data/all_2024.bag \\
        --pose_file data/ImgPose.txt \\
        --image_topic /camera_agent/img_left/compressed \\
        --undistort_fisheye \\
        --db_distance 1.0 \\
        --method salad-indoor \\
        --output_dir datasets/scene_A

    # R1Pro pinhole / ROS2 bag
    python vpr_preprocess.py \\
        --source ros2_r1pro \\
        --bag_path /home/nvidia/vpr_data/2f/rosbag_20260514_093133 \\
        --db_distance 1.0 \\
        --method megaloc \\
        --output_dir ~/Navi/maps/408/vpr/2f
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime
from glob import glob

import numpy as np
import torch
import torchvision.transforms as transforms
from loguru import logger
from PIL import Image
from torch.utils.data import DataLoader, Dataset
from tqdm import tqdm

from model_loader import get_model
from r1pro_utils import extract_database_from_ros2_r1pro
from s20_utils import extract_database_from_ros1


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
# Step 1a / 1a-bis: distance-sampled database image extraction
# ---------------------------------------------------------------------------
# Source-specific bag parsing + undistortion lives in:
#   * s20_utils.extract_database_from_ros1       (ros1 / SHARE SLAM S20)
#   * r1pro_utils.extract_database_from_ros2_r1pro (ros2 / R1Pro)
# Both accept the DB-only :class:`DatabaseSampler` defined above and the
# loguru logger configured here.


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
    # 注意：``undistort_fisheye`` / ``out_width`` / ``out_height`` / ``fov`` /
    # ``calibration_file`` 必须保留，因为 vpr_detect._UNDISTORT_KEYS 会跨 database
    # 校验这几个字段是否一致；R1Pro 路径下我们写 ``undistort_fisheye=false``，
    # 实际去畸变由 vpr_bootstrap 在线侧用 PinHoleUndistorter 完成。
    build_info = {
        "built_at":          datetime.now().isoformat(timespec="seconds"),
        "source":            args.source,
        "method":            args.method,
        "descriptor_dim":    int(descriptor_dim),
        "num_database":      int(len(image_paths)),
        "image_size":        list(args.image_size) if args.image_size else None,
        "output_dir":        args.output_dir,
        "image_cache_dir":   args.image_cache_dir,
        "bag_path":          args.bag_path,
        "pose_file":         getattr(args, "pose_file", None),
        "pose_topic":        getattr(args, "pose_topic", None),
        "image_topic":       args.image_topic,
        "db_distance":       float(args.db_distance),
        # --- fisheye / S20 路径专用（R1Pro 时全部为 false / 默认值） ---
        "undistort_fisheye": bool(args.undistort_fisheye),
        "calibration_file":  args.calibration_file,
        "out_width":         int(args.out_width),
        "out_height":        int(args.out_height),
        "fov":               float(args.fov),
        # --- R1Pro / pinhole 路径专用 ---
        "undistort_pinhole_r1pro": bool(args.source == "ros2_r1pro"),
        "crop_width":        int(getattr(args, "crop_width", 0)) or None,
        "crop_height":       int(getattr(args, "crop_height", 0)) or None,
        # --- 公共 ---
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
        description="Build a VPR feature database (.npy) from a ROS bag.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # Input
    parser.add_argument("--source", type=str, default="ros1",
                        choices=["ros1", "ros2_r1pro"],
                        help="Input pipeline: 'ros1' = SHARE SLAM S20 (.bag + "
                             "ImgPose.txt, optional fisheye undistortion); "
                             "'ros2_r1pro' = R1Pro (.db3 ROS2 bag with "
                             "/Odometry + CompressedImage, always pinhole "
                             "undistorted + center-cropped to match the live "
                             "query pipeline in vpr_bootstrap).")
    parser.add_argument("--bag_path", type=str, required=True,
                        help="Path to the ROS1 .bag file (source=ros1) or "
                             "ROS2 bag directory / .db3 file (source=ros2_r1pro)")
    parser.add_argument("--pose_file", type=str, default=None,
                        help="[source=ros1] Path to ImgPose.txt "
                             "(parsed with load_poses_from_imgpose_file)")
    parser.add_argument("--pose_topic", type=str, default="/Odometry",
                        help="[source=ros2_r1pro] nav_msgs/Odometry topic name")
    parser.add_argument("--image_topic", type=str, default=None,
                        help="Compressed image topic to read. Defaults: "
                             "/camera_agent/img_left/compressed (ros1) or "
                             "/hdas/camera_head/left_raw/image_raw_color/"
                             "compressed (ros2_r1pro)")

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

    # Fisheye undistortion (forwarded to PolyFisheyeUndistorter) -- ros1 only
    parser.add_argument("--undistort_fisheye", action="store_true",
                        help="[source=ros1] Apply POLYFISHEYE -> perspective "
                             "undistortion")
    parser.add_argument("--calibration_file", type=str, default=None,
                        help="[source=ros1] Path to calibration.yaml "
                             "(uses S20 defaults if unset)")
    parser.add_argument("--out_width", type=int, default=1024,
                        help="[source=ros1] PolyFisheyeUndistorter output width")
    parser.add_argument("--out_height", type=int, default=768,
                        help="[source=ros1] PolyFisheyeUndistorter output height")
    parser.add_argument("--fov", type=float, default=90,
                        help="[source=ros1] Output perspective FOV in degrees")

    # R1Pro pinhole crop -- must match vpr_bootstrap._VPR_QUERY_TARGET_W/H
    parser.add_argument("--crop_width", type=int, default=1024,
                        help="[source=ros2_r1pro] Center-crop width after "
                             "pinhole undistortion (must match the live "
                             "query crop in vpr_bootstrap.py)")
    parser.add_argument("--crop_height", type=int, default=768,
                        help="[source=ros2_r1pro] Center-crop height after "
                             "pinhole undistortion (must match the live "
                             "query crop in vpr_bootstrap.py)")

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


_DEFAULT_IMAGE_TOPIC = {
    "ros1": "/camera_agent/img_left/compressed",
    "ros2_r1pro": "/hdas/camera_head/left_raw/image_raw_color/compressed",
}


def _validate_source_args(args):
    """Fill in source-dependent defaults and reject incompatible combos."""
    if args.image_topic is None:
        args.image_topic = _DEFAULT_IMAGE_TOPIC[args.source]

    if args.source == "ros1":
        if not args.pose_file:
            raise SystemExit(
                "source=ros1 requires --pose_file (path to ImgPose.txt)"
            )
    elif args.source == "ros2_r1pro":
        if args.undistort_fisheye:
            raise SystemExit(
                "source=ros2_r1pro applies pinhole undistortion automatically; "
                "do not pass --undistort_fisheye."
            )
        if args.pose_file:
            logger.warning(
                "source=ros2_r1pro reads poses from --pose_topic, "
                "ignoring --pose_file"
            )
        # 强制把 fisheye 字段归零，确保写出的 build_info 不会误导 vpr_detect
        args.undistort_fisheye = False
        args.calibration_file = None


def main():
    args = parse_args()
    args.output_dir = os.path.abspath(os.path.expanduser(args.output_dir))
    args.bag_path = os.path.abspath(os.path.expanduser(args.bag_path))
    os.makedirs(args.output_dir, exist_ok=True)
    _configure_logging(args.output_dir)

    _validate_source_args(args)

    # 解析图片缓存目录：未指定时回退到 <output_dir>/database (向后兼容)
    if args.image_cache_dir:
        args.image_cache_dir = os.path.abspath(
            os.path.expanduser(args.image_cache_dir)
        )
    else:
        args.image_cache_dir = os.path.join(args.output_dir, "database")

    logger.info(f"Source          : {args.source}")
    logger.info(f"Bag path        : {args.bag_path}")
    logger.info(f"Image topic     : {args.image_topic}")
    if args.source == "ros1":
        logger.info(f"Pose file       : {args.pose_file}")
    else:
        logger.info(f"Pose topic      : {args.pose_topic}")
        logger.info(
            f"Pinhole crop    : {args.crop_width}x{args.crop_height} "
            f"(must match vpr_bootstrap._VPR_QUERY_TARGET_W/H)"
        )
    logger.info(f"Image cache dir : {args.image_cache_dir}")
    logger.info(f"Descriptor dir  : {args.output_dir}")
    logger.info(f"Arguments       : {vars(args)}")

    if args.skip_extraction:
        if not os.path.isdir(args.image_cache_dir):
            raise FileNotFoundError(
                f"--skip_extraction set but {args.image_cache_dir} does not exist"
            )
        logger.info(f"Skipping image extraction; reusing {args.image_cache_dir}")
        db_dir = args.image_cache_dir
    elif args.source == "ros1":
        sampler = DatabaseSampler(args.db_distance)
        db_dir = extract_database_from_ros1(
            args, sampler, args.image_cache_dir, logger=logger,
        )
    elif args.source == "ros2_r1pro":
        sampler = DatabaseSampler(args.db_distance)
        db_dir = extract_database_from_ros2_r1pro(
            args, sampler, args.image_cache_dir, logger=logger,
        )
    else:
        raise AssertionError(f"unhandled source: {args.source}")

    extract_descriptors(args, db_dir)
    logger.info("VPR preprocessing complete.")


if __name__ == "__main__":
    main()
