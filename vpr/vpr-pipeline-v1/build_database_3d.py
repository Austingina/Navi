#!/usr/bin/env python3
"""Build VPR image database and query sets from raw data.

Supports three input sources:
  - ROS1 bags (.bag) with external pose file (frame_pose.txt)
  - ROS2 bags (.db3) with odometry topic
  - Plain image folders with a pose file

Images are distance-sampled and saved with @x@y@timestamp@.jpg filenames
for use with the VPR pipeline.

Examples:
    # From ROS1 bag (SHARE SLAM S20 with fisheye)
    python build_database.py --source=ros1 \\
        --bag_path=data/all_*.bag --pose_file=data/frame_pose.txt \\
        --image_topic=/camera_agent/img_left/compressed \\
        --undistort_fisheye \\
        --db_distance=1.0 --query_distance=0.5 \\
        --output_dir=datasets/my_scene

    # From ROS2 bag (ZED robot)
    python build_database.py --source=ros2 \\
        --bag_path=data/ --pose_topic=/laser_odom_path \\
        --image_topic=/zed/zed_node/rgb/color/rect/image \\
        --db_distance=1.0 --query_distance=0.5 \\
        --output_dir=datasets/my_scene

    # From plain image folder
    python build_database.py --source=folder \\
        --image_dir=raw_images/ --pose_file=poses.txt \\
        --db_distance=1.0 --query_distance=0.5 \\
        --output_dir=datasets/my_scene
"""

import argparse
import bisect
import io
import os
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from glob import glob
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from undistort import PolyFisheyeUndistorter


# ---------------------------------------------------------------------------
# Pose loading
# ---------------------------------------------------------------------------

def load_poses_from_file(path):
    """Load poses from frame_pose.txt format.

    Expected format (one pose per line):
        # timestamp tx ty tz qx qy qz qw
        1772004062.900 -0.000113 -0.000093 0.000036 0 0 0 1

    Returns list of (timestamp_sec, x, y) tuples sorted by timestamp.
    """
    poses = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            ts = float(parts[0])
            tx, ty = float(parts[1]), float(parts[2])
            poses.append((ts, tx, ty))
    poses.sort(key=lambda p: p[0])
    return poses


def find_nearest_pose(pose_timestamps, poses, query_ts):
    """Binary search for the pose closest in time to query_ts."""
    idx = bisect.bisect_left(pose_timestamps, query_ts)
    candidates = []
    if idx > 0:
        candidates.append(idx - 1)
    if idx < len(pose_timestamps):
        candidates.append(idx)
    if not candidates:
        return None
    best = min(candidates, key=lambda i: abs(pose_timestamps[i] - query_ts))
    return poses[best]


def load_poses_from_imgpose_file(path, prefix="left"):
    """Load poses from ImgPose.txt format.

    Expected format (one pose per line):
        index x y z roll pitch yaw qx qy qz qw timestamp
        left_0001 -0.000113 -0.000093 0.000036 0 0 0 0 0 0 1 1772004062.900

    Only reads lines where index starts with the given prefix (default: 'left').
    Returns list of (x, y, z, timestamp) tuples in file order.
    """
    poses = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 12:
                continue
            index = parts[0]
            # 只读取 prefix 开头的行
            if not index.startswith(prefix):
                continue
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            timestamp = float(parts[11])
            poses.append((x, y, z, timestamp))
    return poses


# ---------------------------------------------------------------------------
# Distance sampler
# ---------------------------------------------------------------------------

class DistanceSampler:
    """Decide whether to save an image as database, query, or skip it."""

    def __init__(self, db_distance=1.0, query_distance=0.5, db_query_exclusion=0.1):
        self.db_distance = db_distance
        self.query_distance = query_distance
        self.db_query_exclusion = db_query_exclusion
        self.last_db_x = None
        self.last_db_y = None
        self.last_q_x = None
        self.last_q_y = None
        self.db_positions = []
        self.db_count = 0
        self.q_count = 0
        self.skipped = 0

    def classify(self, x, y):
        """Return 'database', 'query', or 'skip' for a position."""
        if self.last_db_x is None:
            self.last_db_x, self.last_db_y = x, y
            self.db_positions.append((x, y))
            self.db_count += 1
            return "database"

        db_dist = np.sqrt((x - self.last_db_x) ** 2 + (y - self.last_db_y) ** 2)
        if db_dist >= self.db_distance:
            self.last_db_x, self.last_db_y = x, y
            self.db_positions.append((x, y))
            self.db_count += 1
            return "database"

        if self.query_distance > 0 and self.last_q_x is not None:
            q_dist = np.sqrt((x - self.last_q_x) ** 2 + (y - self.last_q_y) ** 2)
            if q_dist < self.query_distance:
                self.skipped += 1
                return "skip"

        for dbx, dby in self.db_positions:
            if np.sqrt((x - dbx) ** 2 + (y - dby) ** 2) < self.db_query_exclusion:
                self.skipped += 1
                return "skip"

        self.last_q_x, self.last_q_y = x, y
        self.q_count += 1
        return "query"


# ---------------------------------------------------------------------------
# Image saving helpers
# ---------------------------------------------------------------------------

def save_rgb_array(arr, path, quality=90):
    """Save a numpy RGB array as JPEG."""
    Image.fromarray(arr).save(path, quality=quality)
    return path


def save_compressed_bytes(img_bytes, path, quality=90):
    """Decode compressed image bytes and save as JPEG."""
    img = Image.open(io.BytesIO(img_bytes)).convert("RGB")
    img.save(path, quality=quality)
    return path


def undistort_and_save(img_bytes, undistorter, path, quality=90):
    """Decode, undistort fisheye, save as JPEG."""
    rgb = undistorter.undistort_bytes(img_bytes)
    Image.fromarray(rgb).save(path, quality=quality)
    return path


# ---------------------------------------------------------------------------
# Source: ROS1 bag
# ---------------------------------------------------------------------------

def extract_from_ros1(args):
    """Extract images from a ROS1 bag with pose file."""
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore

    db_dir, q_dir = _make_output_dirs(args.output_dir)
    
    # 从 ImgPose.txt 读取位姿，只读取 left 开头的行
    poses = load_poses_from_imgpose_file(args.pose_file, prefix="left")
    print(f"Loaded {len(poses)} poses (left frames) from {args.pose_file}")

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
        print(f"Fisheye undistortion enabled ({args.out_width}x{args.out_height}, {args.fov}° FOV)")

    typestore = get_typestore(Stores.ROS1_NOETIC)
    sampler = DistanceSampler(args.db_distance, args.query_distance)
    raw_count = 0
    unmatched = 0
    futures = []
    t0 = time.time()

    with Reader(args.bag_path) as reader, \
         ProcessPoolExecutor(max_workers=args.num_workers) as pool:

        conns = [c for c in reader.connections if c.topic == args.image_topic]
        if not conns:
            raise ValueError(f"Topic '{args.image_topic}' not found in {args.bag_path}")
        print(f"Reading {args.image_topic} from {args.bag_path}")

        for conn, timestamp, rawdata in reader.messages(connections=conns):
            msg = typestore.deserialize_ros1(rawdata, conn.msgtype)
            frame_index = raw_count  # 当前帧索引
            raw_count += 1

            # 按帧序号索引位姿，而不是时间戳匹配
            if frame_index >= len(poses):
                unmatched += 1
                continue

            x, y, z, pose_ts = poses[frame_index]
            role = sampler.classify(x, y)
            if role == "skip":
                continue

            fname = f"@{x:.6f}@{y:.6f}@{z:.6f}@{timestamp}@.jpg"
            out_dir = db_dir if role == "database" else q_dir
            path = os.path.join(out_dir, fname)

            img_data = bytes(msg.data)
            if undistorter:
                futures.append(pool.submit(undistort_and_save, img_data, undistorter, path))
            else:
                futures.append(pool.submit(save_compressed_bytes, img_data, path))

            if raw_count % 200 == 0:
                _print_progress(raw_count, sampler, t0)

        _wait_for_futures(futures)

    # 检查帧数一致性
    if raw_count != len(poses):
        print(f"\nWARNING: Frame count mismatch! Bag has {raw_count} frames, pose file has {len(poses)} left poses.")

    _print_summary(raw_count, sampler, unmatched, t0, db_dir, q_dir)


# ---------------------------------------------------------------------------
# Source: ROS2 bag
# ---------------------------------------------------------------------------

def extract_from_ros2(args):
    """Extract images from a ROS2 bag with odometry topic."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image as RosImage
    from nav_msgs.msg import Path

    db_dir, q_dir = _make_output_dirs(args.output_dir)
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

    # Pass 1: extract poses
    print(f"[1/2] Reading poses from {args.pose_topic} ...")
    t0 = time.time()
    reader = _create_reader(bag_path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.pose_topic]))

    poses = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        path_msg = deserialize_message(data, Path)
        if path_msg.poses:
            lp = path_msg.poses[-1]
            ts = lp.header.stamp.sec * 10**9 + lp.header.stamp.nanosec
            poses.append((ts, lp.pose.position.x, lp.pose.position.y))
    poses.sort(key=lambda p: p[0])
    pose_timestamps = [p[0] for p in poses]
    print(f"  Found {len(poses)} poses in {time.time() - t0:.1f}s")

    if not poses:
        raise RuntimeError(f"No poses found on {args.pose_topic}")

    # Pass 2: extract images
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
            topic, data, t = reader.read_next()
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
                _print_progress(raw_count, sampler, t0)

        _wait_for_futures(futures)

    _print_summary(raw_count, sampler, 0, t0, db_dir, q_dir)


# ---------------------------------------------------------------------------
# Source: ROS2 bag from R1Pro (Odometry + CompressedImage)
# ---------------------------------------------------------------------------

def extract_from_ros2_of_r1pro(args):
    """Extract images from a R1Pro ROS2 bag with Odometry and CompressedImage topics.
    
    Args:
        args: Namespace with attributes:
            - bag_path: Path to the ROS2 bag directory
            - pose_topic: Odometry topic name (nav_msgs/msg/Odometry)
            - image_topic: CompressedImage topic name (sensor_msgs/msg/CompressedImage)
            - output_dir: Output directory for extracted images
            - db_distance: Minimum distance between database images
            - query_distance: Minimum distance between query images
            - num_workers: Number of parallel workers for image saving
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    from nav_msgs.msg import Odometry

    db_dir, q_dir = _make_output_dirs(args.output_dir)
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

    # Pass 1: extract poses from Odometry topic
    print(f"[1/2] Reading poses from {args.pose_topic} ...")
    t0 = time.time()
    reader = _create_reader(bag_path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.pose_topic]))

    poses = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        odom_msg = deserialize_message(data, Odometry)
        ts = odom_msg.header.stamp.sec * 10**9 + odom_msg.header.stamp.nanosec
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        z = odom_msg.pose.pose.position.z
        poses.append((ts, x, y, z))
    poses.sort(key=lambda p: p[0])
    pose_timestamps = [p[0] for p in poses]
    print(f"  Found {len(poses)} poses in {time.time() - t0:.1f}s")

    if not poses:
        raise RuntimeError(f"No poses found on {args.pose_topic}")

    def find_nearest_pose_3d(query_ts):
        """Binary search for the pose closest in time to query_ts."""
        idx = bisect.bisect_left(pose_timestamps, query_ts)
        candidates = []
        if idx > 0:
            candidates.append(idx - 1)
        if idx < len(pose_timestamps):
            candidates.append(idx)
        if not candidates:
            return None
        best = min(candidates, key=lambda i: abs(pose_timestamps[i] - query_ts))
        return poses[best]

    # Pass 2: extract images from CompressedImage topic
    print(f"[2/2] Extracting images from {args.image_topic} ...")
    reader = _create_reader(bag_path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.image_topic]))

    sampler = DistanceSampler(args.db_distance, args.query_distance)
    raw_count = 0
    futures = []

    with ProcessPoolExecutor(max_workers=args.num_workers) as pool:
        while reader.has_next():
            topic, data, t = reader.read_next()
            img_msg = deserialize_message(data, CompressedImage)
            img_ts = img_msg.header.stamp.sec * 10**9 + img_msg.header.stamp.nanosec
            
            pose = find_nearest_pose_3d(img_ts)
            if pose is None:
                continue
                
            _, x, y, z = pose
            raw_count += 1

            role = sampler.classify(x, y)
            if role == "skip":
                continue

            fname = f"@{x:.6f}@{y:.6f}@{z:.6f}@{img_ts}@.jpg"
            out_dir = db_dir if role == "database" else q_dir
            path = os.path.join(out_dir, fname)
            
            img_data = bytes(img_msg.data)
            futures.append(pool.submit(save_compressed_bytes, img_data, path))

            if raw_count % 500 == 0:
                _print_progress(raw_count, sampler, t0)

        _wait_for_futures(futures)

    _print_summary(raw_count, sampler, 0, t0, db_dir, q_dir)


# ---------------------------------------------------------------------------
# Source: plain image folder
# ---------------------------------------------------------------------------

def extract_from_folder(args):
    """Extract images from a folder with a pose file."""
    db_dir, q_dir = _make_output_dirs(args.output_dir)

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

    _print_summary(len(images), sampler, 0, t0, db_dir, q_dir)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_output_dirs(output_dir):
    db_dir = os.path.join(output_dir, "database")
    q_dir = os.path.join(output_dir, "queries")
    os.makedirs(db_dir, exist_ok=True)
    os.makedirs(q_dir, exist_ok=True)
    return db_dir, q_dir


def _wait_for_futures(futures):
    print(f"  Waiting for {len(futures)} image saves ...")
    for f in as_completed(futures):
        f.result()


def _print_progress(raw_count, sampler, t0):
    rate = raw_count / (time.time() - t0)
    print(f"  Read {raw_count} frames "
          f"(db={sampler.db_count}, queries={sampler.q_count}, "
          f"skipped={sampler.skipped}) [{rate:.0f} fr/s]")


def _print_summary(raw_count, sampler, unmatched, t0, db_dir, q_dir):
    elapsed = time.time() - t0
    print(f"\nDone in {elapsed:.1f}s! Read {raw_count} raw frames:")
    print(f"  Database: {sampler.db_count} images in {db_dir}")
    print(f"  Queries:  {sampler.q_count} images in {q_dir}")
    print(f"  Skipped:  {sampler.skipped}")
    if unmatched:
        print(f"  Unmatched: {unmatched} (no pose within threshold)")


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

    # Input paths
    parser.add_argument("--bag_path", type=str, default=None,
                        help="Path to ROS bag file (.bag or .db3 directory)")
    parser.add_argument("--image_dir", type=str, default=None,
                        help="Path to image folder (for --source=folder)")
    parser.add_argument("--pose_file", type=str, default=None,
                        help="Path to frame_pose.txt (for ros1/folder sources)")
    parser.add_argument("--image_topic", type=str,
                        default="/camera_agent/img_left/compressed",
                        help="ROS image topic")
    parser.add_argument("--pose_topic", type=str,
                        default="/laser_odom_path",
                        help="ROS2 odometry topic (for --source=ros2)")

    # Sampling
    parser.add_argument("--db_distance", type=float, default=1.0,
                        help="Min distance (m) between database images")
    parser.add_argument("--query_distance", type=float, default=0.5,
                        help="Min distance (m) between query images")

    # Fisheye undistortion
    parser.add_argument("--undistort_fisheye", action="store_true",
                        help="Apply POLYFISHEYE undistortion")
    parser.add_argument("--calibration_file", type=str, default=None,
                        help="Path to calibration.yaml (uses S20 defaults if not set)")
    parser.add_argument("--out_width", type=int, default=1024)
    parser.add_argument("--out_height", type=int, default=768)
    parser.add_argument("--fov", type=float, default=90,
                        help="Output perspective FOV in degrees")

    # Performance
    parser.add_argument("--num_workers", type=int, default=16,
                        help="Parallel workers for image saving")

    args = parser.parse_args()

    if args.source == "ros1":
        if not args.bag_path or not args.pose_file:
            parser.error("--source=ros1 requires --bag_path and --pose_file")
        extract_from_ros1(args)

    elif args.source == "ros2":
        if not args.bag_path:
            parser.error("--source=ros2 requires --bag_path")
        extract_from_ros2_of_r1pro(args)

    elif args.source == "folder":
        if not args.image_dir or not args.pose_file:
            parser.error("--source=folder requires --image_dir and --pose_file")
        extract_from_folder(args)


if __name__ == "__main__":
    main()
