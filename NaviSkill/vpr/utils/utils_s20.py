#!/usr/bin/env python3
"""S20 (SHARE SLAM, ROS1 bag + ImgPose.txt) data-processing helpers.

All S20-specific code that used to be scattered across
``eval/build_database_3d.py`` and ``vpr_preprocess.py`` lives here so that
the live preprocessing pipeline and the legacy (evaluation) DB+query
builder share one implementation.

Geometric pipeline:
    raw CompressedImage (JPEG) from /camera_agent/img_left/compressed
      -> (optional) PolyFisheyeUndistorter.undistort_bytes
         (POLYFISHEYE -> perspective remap using the calibration_file
         shipped with the S20 head, or the in-tree defaults)
      -> JPEG re-encode at quality=90
"""

import os
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

from .undistort import PolyFisheyeUndistorter
from .utils_common import (
    DistanceSampler,
    make_output_dirs,
    print_progress,
    print_summary,
    save_compressed_bytes,
    undistort_and_save,
    wait_for_futures,
)


# ---------------------------------------------------------------------------
# Pose loading (S20 specific format)
# ---------------------------------------------------------------------------

def load_poses_from_imgpose_file(path, prefix="left"):
    """Load poses from ImgPose.txt format (S20 SLAM output).

    Expected format (one pose per line)::

        index x y z roll pitch yaw qx qy qz qw timestamp
        left_0001 -0.000113 -0.000093 0.000036 0 0 0 0 0 0 1 1772004062.900

    Only reads lines where ``index`` starts with ``prefix`` (default: ``left``).
    Returns a list of ``(x, y, z, timestamp)`` tuples in file order; the
    bag-frame index is therefore directly indexable against this list.
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
            if not index.startswith(prefix):
                continue
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            timestamp = float(parts[11])
            poses.append((x, y, z, timestamp))
    return poses


# ---------------------------------------------------------------------------
# Optional fisheye undistorter construction (shared between both pipelines)
# ---------------------------------------------------------------------------

def build_fisheye_undistorter(args):
    """Construct a :class:`PolyFisheyeUndistorter` from ``args`` or return None.

    Honors ``args.undistort_fisheye``, ``args.calibration_file``,
    ``args.out_width`` / ``args.out_height``, and ``args.fov``.
    """
    if not getattr(args, "undistort_fisheye", False):
        return None
    if args.calibration_file:
        return PolyFisheyeUndistorter.from_yaml(
            args.calibration_file,
            out_w=args.out_width, out_h=args.out_height, fov_deg=args.fov,
        )
    return PolyFisheyeUndistorter.from_defaults(
        out_w=args.out_width, out_h=args.out_height, fov_deg=args.fov,
    )


# ---------------------------------------------------------------------------
# DB-only extraction (called by vpr_preprocess.py)
# ---------------------------------------------------------------------------

def extract_database_from_ros1(args, sampler, db_dir, logger=None):
    """Parse a ROS1 bag + ImgPose.txt and write sampled JPGs to disk.

    Used by the new ``vpr_preprocess.py`` pipeline; the supplied
    ``sampler`` is the DB-only :class:`DatabaseSampler` (one keep/skip
    decision per frame, no query split).

    Args:
        args: Namespace with ``bag_path``, ``pose_file``, ``image_topic``,
            ``undistort_fisheye``, ``calibration_file``, ``out_width``,
            ``out_height``, ``fov``, ``num_workers``.
        sampler: object exposing ``.keep(x, y) -> bool`` plus
            ``.kept`` / ``.skipped`` counters.
        db_dir: output directory for the extracted JPGs (will be created).
        logger: loguru-style logger; defaults to ``loguru.logger``.

    Returns:
        The same ``db_dir`` for caller convenience.
    """
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore

    if logger is None:
        from loguru import logger as _logger
        logger = _logger

    os.makedirs(db_dir, exist_ok=True)

    poses = load_poses_from_imgpose_file(args.pose_file, prefix="left")
    logger.info(f"Loaded {len(poses)} left-frame poses from {args.pose_file}")

    undistorter = build_fisheye_undistorter(args)
    if undistorter is not None:
        logger.info(
            f"Fisheye undistortion enabled "
            f"({args.out_width}x{args.out_height}, {args.fov}° FOV)"
        )

    typestore = get_typestore(Stores.ROS1_NOETIC)
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
            # 帧号与 ImgPose.txt 中 left_* 的顺序一一对应
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
# DB + query extraction (called by eval/build_database_3d.py legacy main())
# ---------------------------------------------------------------------------

def extract_from_ros1(args):
    """Extract images from a ROS1 bag with pose file (DB + query split).

    Legacy variant called by ``eval/build_database_3d.py``. Self-contained:
    only depends on :mod:`utils.utils_common` and the fisheye undistorter.
    """
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore

    db_dir, q_dir = make_output_dirs(args.output_dir)

    poses = load_poses_from_imgpose_file(args.pose_file, prefix="left")
    print(f"Loaded {len(poses)} poses (left frames) from {args.pose_file}")

    undistorter = build_fisheye_undistorter(args)
    if undistorter is not None:
        print(
            f"Fisheye undistortion enabled "
            f"({args.out_width}x{args.out_height}, {args.fov}° FOV)"
        )

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
            raise ValueError(
                f"Topic '{args.image_topic}' not found in {args.bag_path}"
            )
        print(f"Reading {args.image_topic} from {args.bag_path}")

        for conn, timestamp, rawdata in reader.messages(connections=conns):
            msg = typestore.deserialize_ros1(rawdata, conn.msgtype)
            frame_index = raw_count
            raw_count += 1

            if frame_index >= len(poses):
                unmatched += 1
                continue

            x, y, z, _pose_ts = poses[frame_index]
            role = sampler.classify(x, y)
            if role == "skip":
                continue

            fname = f"@{x:.6f}@{y:.6f}@{z:.6f}@{timestamp}@.jpg"
            out_dir = db_dir if role == "database" else q_dir
            path = os.path.join(out_dir, fname)

            img_data = bytes(msg.data)
            if undistorter:
                futures.append(
                    pool.submit(undistort_and_save, img_data, undistorter, path)
                )
            else:
                futures.append(
                    pool.submit(save_compressed_bytes, img_data, path)
                )

            if raw_count % 200 == 0:
                print_progress(raw_count, sampler, t0)

        wait_for_futures(futures)

    if raw_count != len(poses):
        print(
            f"\nWARNING: Frame count mismatch! "
            f"Bag has {raw_count} frames, "
            f"pose file has {len(poses)} left poses."
        )

    print_summary(raw_count, sampler, unmatched, t0, db_dir, q_dir)
