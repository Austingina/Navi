#!/usr/bin/env python3
"""R1Pro (ROS2 + Odometry + CompressedImage) data-processing helpers.

All R1Pro-specific code that used to be scattered across
``eval/build_database_3d.py`` and ``vpr_preprocess.py`` lives here so that:

* the live pipeline (``vpr_preprocess.py``) and the legacy (evaluation)
  database-builder (``eval/build_database_3d.py``) share one
  implementation;
* worker functions used by :class:`concurrent.futures.ProcessPoolExecutor`
  have a stable, picklable module path (``utils.utils_r1pro.<fn>``).

Geometric pipeline (must match ``NaviSkill/vpr_bootstrap``):
    raw CompressedImage (JPEG)
      -> PinHoleUndistorter.undistort_bytes  (cv2 remap with the
         rational/radial-tangential model derived from the R1Pro head
         left camera calibration)
      -> center-crop to (crop_width, crop_height), default 1024x768
      -> JPEG re-encode at quality=90
"""

import bisect
import os
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

from PIL import Image

from .undistort2 import PinHoleUndistorter
from .utils_common import (
    DistanceSampler,
    make_output_dirs,
    print_progress,
    print_summary,
    undistort_and_save,
)


# ---------------------------------------------------------------------------
# Pinhole undistort + center-crop helpers (used by ProcessPoolExecutor workers)
# ---------------------------------------------------------------------------

_WORKER_PINHOLE_UNDISTORTER = None


def r1pro_worker_init():
    """ProcessPoolExecutor initializer: build the pinhole undistorter once.

    Each worker process pays one-off remap-table construction
    (~1920x1080 = 8 MB of float32 maps) instead of pickling the table
    per task.
    """
    global _WORKER_PINHOLE_UNDISTORTER
    _WORKER_PINHOLE_UNDISTORTER = PinHoleUndistorter.from_defaults()


def r1pro_undistort_crop_save(img_bytes, path, crop_w, crop_h, quality=90):
    """Decode JPEG -> pinhole undistort -> center-crop -> save as JPEG.

    Matches the live-query geometry in
    ``NaviSkill/vpr_bootstrap._undistort_and_center_crop`` so the database
    images and online query images live in identical feature space.

    Designed to be called as a worker task; falls back to building its
    own undistorter when invoked from the main process.
    """
    undistorter = _WORKER_PINHOLE_UNDISTORTER
    if undistorter is None:
        undistorter = PinHoleUndistorter.from_defaults()
    rgb = undistorter.undistort_bytes(img_bytes)
    h, w = rgb.shape[:2]
    if w >= crop_w and h >= crop_h:
        x0 = (w - crop_w) // 2
        y0 = (h - crop_h) // 2
        rgb = rgb[y0:y0 + crop_h, x0:x0 + crop_w]
    Image.fromarray(rgb).save(path, quality=quality)
    return path


# ---------------------------------------------------------------------------
# ROS2 reader / odometry pose helpers
# ---------------------------------------------------------------------------

def _open_ros2_reader(bag_path):
    """Open a SequentialReader for a ROS2 sqlite3 bag.

    Accepts either the .db3 file or the directory that contains it.
    """
    import rosbag2_py
    if bag_path.endswith(".db3"):
        bag_path = os.path.dirname(bag_path) or "."
    reader = rosbag2_py.SequentialReader()
    opts = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    conv = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(opts, conv)
    return reader


def load_odometry_poses(bag_path, pose_topic):
    """Read every ``nav_msgs/Odometry`` message on ``pose_topic``.

    Returns
    -------
    poses : list of (ts_ns, x, y, z)
        Sorted ascending by ``ts_ns``.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry

    reader = _open_ros2_reader(bag_path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[pose_topic]))

    poses = []
    while reader.has_next():
        _topic, data, _t = reader.read_next()
        msg = deserialize_message(data, Odometry)
        ts = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        p = msg.pose.pose.position
        poses.append((ts, p.x, p.y, p.z))
    poses.sort(key=lambda x: x[0])
    return poses


def make_pose_nearest_lookup(poses):
    """Return a closure ``find_nearest(ts_ns) -> (ts, x, y, z) | None``.

    ``poses`` must be sorted ascending by timestamp (as returned by
    :func:`load_odometry_poses`).
    """
    pose_ts = [p[0] for p in poses]

    def find_nearest(query_ts):
        idx = bisect.bisect_left(pose_ts, query_ts)
        candidates = []
        if idx > 0:
            candidates.append(idx - 1)
        if idx < len(pose_ts):
            candidates.append(idx)
        if not candidates:
            return None
        best = min(candidates, key=lambda i: abs(pose_ts[i] - query_ts))
        return poses[best]

    return find_nearest


# ---------------------------------------------------------------------------
# DB-only extraction (called by vpr_preprocess.py)
# ---------------------------------------------------------------------------

def extract_database_from_ros2_r1pro(args, sampler, db_dir, logger=None):
    """Parse an R1Pro ROS2 bag and write undistorted+cropped JPGs to disk.

    Used by the new ``vpr_preprocess.py`` pipeline; the supplied ``sampler``
    is the DB-only :class:`DatabaseSampler` (one keep/skip decision per
    frame, no query split).

    Args:
        args: Namespace with ``bag_path``, ``image_topic``, ``pose_topic``,
            ``crop_width``, ``crop_height``, ``num_workers``.
        sampler: object exposing ``.keep(x, y) -> bool`` plus
            ``.kept`` / ``.skipped`` counters (see
            ``vpr_preprocess.DatabaseSampler``).
        db_dir: output directory for the extracted JPGs (will be created).
        logger: loguru-style logger; defaults to ``loguru.logger``.

    Returns:
        The same ``db_dir`` for caller convenience.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage

    if logger is None:
        from loguru import logger as _logger
        logger = _logger

    os.makedirs(db_dir, exist_ok=True)

    t0 = time.time()
    logger.info(f"[1/2] Reading poses from {args.pose_topic} ...")
    poses = load_odometry_poses(args.bag_path, args.pose_topic)
    if not poses:
        raise RuntimeError(
            f"No poses found on {args.pose_topic} in {args.bag_path}"
        )
    find_nearest_pose = make_pose_nearest_lookup(poses)
    logger.info(f"  Found {len(poses)} poses in {time.time() - t0:.1f}s")

    logger.info(f"[2/2] Extracting images from {args.image_topic} ...")
    reader = _open_ros2_reader(args.bag_path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.image_topic]))

    raw_count = 0
    futures = []
    t1 = time.time()

    with ProcessPoolExecutor(
        max_workers=args.num_workers,
        initializer=r1pro_worker_init,
    ) as pool:
        while reader.has_next():
            _topic, data, _t = reader.read_next()
            msg = deserialize_message(data, CompressedImage)
            img_ts = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            raw_count += 1

            pose = find_nearest_pose(img_ts)
            if pose is None:
                continue
            _, x, y, z = pose

            if not sampler.keep(x, y):
                continue

            fname = f"@{x:.6f}@{y:.6f}@{z:.6f}@{img_ts}@.jpg"
            path = os.path.join(db_dir, fname)
            img_data = bytes(msg.data)
            futures.append(
                pool.submit(
                    r1pro_undistort_crop_save,
                    img_data, path, args.crop_width, args.crop_height,
                )
            )

            if raw_count % 200 == 0:
                rate = raw_count / (time.time() - t1)
                logger.info(
                    f"  Read {raw_count} frames "
                    f"(kept={sampler.kept}, skipped={sampler.skipped}) "
                    f"[{rate:.0f} fr/s]"
                )

        logger.info(f"  Waiting for {len(futures)} image saves ...")
        for f in as_completed(futures):
            f.result()

    elapsed = time.time() - t0
    logger.info(f"Image extraction done in {elapsed:.1f}s.")
    logger.info(f"  Kept (database):  {sampler.kept} -> {db_dir}")
    logger.info(f"  Skipped:          {sampler.skipped}")
    logger.info(f"  Raw image frames: {raw_count}")

    return db_dir


# ---------------------------------------------------------------------------
# DB + query extraction (called by eval/build_database_3d.py legacy main())
# ---------------------------------------------------------------------------

def extract_from_ros2_of_r1pro(args):
    """Extract images from a R1Pro ROS2 bag with Odometry and CompressedImage.

    DB + query variant used by the legacy ``eval/build_database_3d.py`` entry
    point. Self-contained: only depends on :mod:`utils.utils_common` and the
    pinhole undistorter.

    Args:
        args: Namespace with attributes:
            - ``bag_path``: Path to the ROS2 bag directory (or .db3 file)
            - ``pose_topic``: ``nav_msgs/msg/Odometry`` topic name
            - ``image_topic``: ``sensor_msgs/msg/CompressedImage`` topic name
            - ``output_dir``: Output directory for extracted images
            - ``db_distance``: Min distance between database images
            - ``query_distance``: Min distance between query images
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage

    db_dir, q_dir = make_output_dirs(args.output_dir)

    print(f"[1/2] Reading poses from {args.pose_topic} ...")
    t0 = time.time()
    poses = load_odometry_poses(args.bag_path, args.pose_topic)
    if not poses:
        raise RuntimeError(f"No poses found on {args.pose_topic}")
    find_nearest_pose_3d = make_pose_nearest_lookup(poses)
    print(f"  Found {len(poses)} poses in {time.time() - t0:.1f}s")

    print(f"[2/2] Extracting images from {args.image_topic} ...")
    reader = _open_ros2_reader(args.bag_path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.image_topic]))

    sampler = DistanceSampler(args.db_distance, args.query_distance)
    raw_count = 0
    undistorter = PinHoleUndistorter.from_defaults()

    while reader.has_next():
        _topic, data, _t = reader.read_next()
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
        undistort_and_save(img_data, undistorter, path)

        if raw_count % 500 == 0:
            print_progress(raw_count, sampler, t0)

    print_summary(raw_count, sampler, 0, t0, db_dir, q_dir)
