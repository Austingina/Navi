#!/usr/bin/env python3
"""Build VPR image database + query sets from raw data (legacy eval entry).

This is an **evaluation-only** entry point that produces a *paired*
database + query layout suitable for recall@K benchmarks via
``eval/pipeline_3d.py``. The production live pipeline uses
``vpr_preprocess.py`` (database-only output, no query split).

Supports three input sources, dispatched via ``--source``:

* ``ros1``  -- SHARE SLAM S20 ROS1 bag + ImgPose.txt, optional fisheye
  undistortion. Implementation lives in :mod:`utils.utils_s20`.
* ``ros2``  -- R1Pro ROS2 bag with ``nav_msgs/Odometry`` +
  ``sensor_msgs/CompressedImage``, always pinhole-undistorted.
  Implementation lives in :mod:`utils.utils_r1pro`.
* ``folder`` -- plain image folder + ``frame_pose.txt``.

Platform-specific logic has been factored out into :mod:`utils.utils_s20`
and :mod:`utils.utils_r1pro` (which live under ``vpr/utils/`` next to
the live pipeline). This module retains:

* the *generic* ROS2 path (Image + Path) and folder path;
* the CLI entry point.

Images are distance-sampled and saved with ``@x@y[@z]@timestamp@.jpg``
filenames for downstream use by ``eval/pipeline_3d.py``.

Examples:
    # From ROS1 bag (SHARE SLAM S20 with fisheye)
    python eval/build_database_3d.py --source=ros1 \\
        --bag_path=data/all_*.bag --pose_file=data/frame_pose.txt \\
        --image_topic=/camera_agent/img_left/compressed \\
        --undistort_fisheye \\
        --db_distance=1.0 --query_distance=0.5 \\
        --output_dir=datasets/my_scene

    # From ROS2 bag (R1Pro)
    python eval/build_database_3d.py --source=ros2 \\
        --bag_path=data/ --pose_topic=/Odometry \\
        --image_topic=/hdas/camera_head/left_raw/image_raw_color/compressed \\
        --db_distance=1.0 --query_distance=0.5 \\
        --output_dir=datasets/my_scene

    # From plain image folder
    python eval/build_database_3d.py --source=folder \\
        --image_dir=raw_images/ --pose_file=poses.txt \\
        --db_distance=1.0 --query_distance=0.5 \\
        --output_dir=datasets/my_scene
"""

import argparse
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from glob import glob
from pathlib import Path

# Allow ``python eval/build_database_3d.py`` (and ``python build_database_3d.py``
# from eval/) to find the ``utils`` subpackage in the parent ``vpr/``.
_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.insert(0, _PARENT_DIR)

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from utils.undistort import PolyFisheyeUndistorter  # noqa: E402
from utils.utils_common import (  # noqa: E402
    DistanceSampler,
    find_nearest_pose,
    load_poses_from_file,
    make_output_dirs,
    print_progress,
    print_summary,
    save_rgb_array,
    wait_for_futures,
)


# ---------------------------------------------------------------------------
# Source: generic ROS2 bag (Image msg + Path msg). NOT R1Pro --
# R1Pro uses Odometry + CompressedImage and lives in utils.utils_r1pro.
# ---------------------------------------------------------------------------

def extract_from_ros2(args):
    """Extract images from a generic (non-R1Pro) ROS2 bag.

    Expects ``sensor_msgs/Image`` on ``args.image_topic`` and
    ``nav_msgs/Path`` on ``args.pose_topic``; uses the last pose in each
    Path message as the keyframe pose.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image as RosImage
    from nav_msgs.msg import Path as NavPath

    db_dir, q_dir = make_output_dirs(args.output_dir)
    bag_path = args.bag_path
    if bag_path.endswith(".db3"):
        bag_path = os.path.dirname(bag_path) or "."

    def _create_reader(uri):
        reader = rosbag2_py.SequentialReader()
        opts = rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3")
        conv = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )
        reader.open(opts, conv)
        return reader

    print(f"[1/2] Reading poses from {args.pose_topic} ...")
    t0 = time.time()
    reader = _create_reader(bag_path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.pose_topic]))

    poses = []
    while reader.has_next():
        _topic, data, _t = reader.read_next()
        path_msg = deserialize_message(data, NavPath)
        if path_msg.poses:
            lp = path_msg.poses[-1]
            ts = lp.header.stamp.sec * 10**9 + lp.header.stamp.nanosec
            poses.append((ts, lp.pose.position.x, lp.pose.position.y))
    poses.sort(key=lambda p: p[0])
    pose_timestamps = [p[0] for p in poses]
    print(f"  Found {len(poses)} poses in {time.time() - t0:.1f}s")

    if not poses:
        raise RuntimeError(f"No poses found on {args.pose_topic}")

    print(f"[2/2] Extracting images from {args.image_topic} ...")
    reader = _create_reader(bag_path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.image_topic]))

    sampler = DistanceSampler(args.db_distance, args.query_distance)
    raw_count = 0
    futures = []

    def _msg_ts(msg):
        return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

    def _ros_image_to_rgb(msg):
        enc = msg.encoding.lower()
        h, w = msg.height, msg.width
        raw = bytes(msg.data)
        if enc == "rgb8":
            arr = np.frombuffer(raw, np.uint8).reshape(h, w, 3)
        elif enc == "bgr8":
            arr = np.frombuffer(raw, np.uint8).reshape(h, w, 3)[:, :, ::-1]
        elif enc == "bgra8":
            arr = np.frombuffer(raw, np.uint8).reshape(h, w, 4)[:, :, :3][:, :, ::-1]
        elif enc == "rgba8":
            arr = np.frombuffer(raw, np.uint8).reshape(h, w, 4)[:, :, :3]
        elif enc == "mono8":
            arr = np.frombuffer(raw, np.uint8).reshape(h, w)
            arr = np.stack([arr, arr, arr], axis=-1)
        else:
            raise ValueError(f"Unsupported encoding: {msg.encoding}")
        return arr.copy()

    with ProcessPoolExecutor(max_workers=args.num_workers) as pool:
        while reader.has_next():
            _topic, data, _t = reader.read_next()
            img_msg = deserialize_message(data, RosImage)
            img_ts = _msg_ts(img_msg)
            _, x, y = find_nearest_pose(pose_timestamps, poses, img_ts)
            raw_count += 1

            role = sampler.classify(x, y)
            if role == "skip":
                continue

            arr = _ros_image_to_rgb(img_msg)
            fname = f"@{x:.6f}@{y:.6f}@{img_ts}@.jpg"
            out_dir = db_dir if role == "database" else q_dir
            path = os.path.join(out_dir, fname)
            futures.append(pool.submit(save_rgb_array, arr, path))

            if raw_count % 500 == 0:
                print_progress(raw_count, sampler, t0)

        wait_for_futures(futures)

    print_summary(raw_count, sampler, 0, t0, db_dir, q_dir)


# ---------------------------------------------------------------------------
# Source: plain image folder
# ---------------------------------------------------------------------------

def extract_from_folder(args):
    """Extract images from a folder with a pose file."""
    db_dir, q_dir = make_output_dirs(args.output_dir)

    poses = load_poses_from_file(args.pose_file)
    pose_timestamps = [p[0] for p in poses]
    print(f"Loaded {len(poses)} poses from {args.pose_file}")

    images = sorted(glob(f"{args.image_dir}/**/*", recursive=True))
    images = [p for p in images if os.path.isfile(p) and
              os.path.splitext(p)[1].lower() in [".jpg", ".jpeg", ".png"]]
    print(f"Found {len(images)} images in {args.image_dir}")

    undistorter = None
    if args.undistort_fisheye:
        if args.calibration_file:
            undistorter = PolyFisheyeUndistorter.from_yaml(
                args.calibration_file, out_w=args.out_width, out_h=args.out_height, fov_deg=args.fov
            )
        else:
            undistorter = PolyFisheyeUndistorter.from_defaults(
                out_w=args.out_width, out_h=args.out_height, fov_deg=args.fov
            )

    sampler = DistanceSampler(args.db_distance, args.query_distance)
    saved = 0
    t0 = time.time()

    for img_path in images:
        stem = Path(img_path).stem
        try:
            ts = float(stem)
        except ValueError:
            parts = stem.replace("_", ".").split(".")
            try:
                ts = float(f"{parts[0]}.{parts[1]}")
            except (ValueError, IndexError):
                ts = saved * 0.1

        pose = find_nearest_pose(pose_timestamps, poses, ts)
        if pose is None or abs(pose[0] - ts) > 5.0:
            continue

        _, x, y = pose
        role = sampler.classify(x, y)
        if role == "skip":
            continue

        fname = f"@{x:.6f}@{y:.6f}@{int(ts * 1e9)}@.jpg"
        out_dir = db_dir if role == "database" else q_dir
        out_path = os.path.join(out_dir, fname)

        if undistorter:
            img = cv2.imread(img_path)
            undist = undistorter.undistort(img)
            rgb = cv2.cvtColor(undist, cv2.COLOR_BGR2RGB)
            Image.fromarray(rgb).save(out_path, quality=90)
        else:
            img = Image.open(img_path).convert("RGB")
            img.save(out_path, quality=90)
        saved += 1

    print_summary(len(images), sampler, 0, t0, db_dir, q_dir)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Build VPR image database from raw data",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("--source", type=str, required=True,
                        choices=["ros1", "ros2", "folder"],
                        help="Input data source type")
    parser.add_argument("--output_dir", type=str, required=True,
                        help="Output directory (will contain database/ and queries/)")

    parser.add_argument("--bag_path", type=str, default=None,
                        help="Path to ROS bag file (.bag or .db3 directory)")
    parser.add_argument("--image_dir", type=str, default=None,
                        help="Path to image folder (for --source=folder)")
    parser.add_argument("--pose_file", type=str, default=None,
                        help="Path to frame_pose.txt / ImgPose.txt "
                             "(for ros1/folder sources)")
    parser.add_argument("--image_topic", type=str,
                        default="/camera_agent/img_left/compressed",
                        help="ROS image topic")
    parser.add_argument("--pose_topic", type=str,
                        default="/laser_odom_path",
                        help="ROS2 pose topic (for --source=ros2)")

    parser.add_argument("--db_distance", type=float, default=1.0,
                        help="Min distance (m) between database images")
    parser.add_argument("--query_distance", type=float, default=0.5,
                        help="Min distance (m) between query images")

    parser.add_argument("--undistort_fisheye", action="store_true",
                        help="Apply POLYFISHEYE undistortion (ros1/folder)")
    parser.add_argument("--calibration_file", type=str, default=None,
                        help="Path to calibration.yaml (uses S20 defaults if not set)")
    parser.add_argument("--out_width", type=int, default=1024)
    parser.add_argument("--out_height", type=int, default=768)
    parser.add_argument("--fov", type=float, default=90,
                        help="Output perspective FOV in degrees")

    parser.add_argument("--num_workers", type=int, default=16,
                        help="Parallel workers for image saving")

    args = parser.parse_args()

    if args.source == "ros1":
        if not args.bag_path or not args.pose_file:
            parser.error("--source=ros1 requires --bag_path and --pose_file")
        from utils.utils_s20 import extract_from_ros1
        extract_from_ros1(args)

    elif args.source == "ros2":
        if not args.bag_path:
            parser.error("--source=ros2 requires --bag_path")
        from utils.utils_r1pro import extract_from_ros2_of_r1pro
        extract_from_ros2_of_r1pro(args)

    elif args.source == "folder":
        if not args.image_dir or not args.pose_file:
            parser.error("--source=folder requires --image_dir and --pose_file")
        extract_from_folder(args)


if __name__ == "__main__":
    main()
