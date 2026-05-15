#!/usr/bin/env python3
"""Common helpers shared by the platform-specific extraction modules.

This module hosts the small, platform-agnostic utilities used by
:mod:`utils.utils_s20`, :mod:`utils.utils_r1pro`, and
:mod:`eval.build_database_3d`:

* :class:`DistanceSampler` -- DB+query distance sampler for the legacy
  ``eval/build_database_3d.py`` pipelines.
* JPEG save helpers (``save_rgb_array``, ``save_compressed_bytes``,
  ``undistort_and_save``).
* Output-directory / progress / summary helpers used by the
  ``extract_from_*`` (DB+query) variants.
* Generic ``frame_pose.txt`` pose loader and binary-search nearest-pose
  helper used by the folder and generic-ROS2 paths.

Keeping these here means ``utils_s20`` and ``utils_r1pro`` do not need to
import from ``eval/build_database_3d.py`` (which is an evaluation entry
script with a ``main()`` and ``if __name__ == "__main__"`` block).
"""

import bisect
import io
import os
import time
from concurrent.futures import as_completed

import numpy as np
from PIL import Image


# ---------------------------------------------------------------------------
# Generic pose loading (frame_pose.txt). The S20 ``ImgPose.txt`` loader
# lives in utils.utils_s20.load_poses_from_imgpose_file.
# ---------------------------------------------------------------------------

def load_poses_from_file(path):
    """Load poses from frame_pose.txt format.

    Expected format (one pose per line)::

        # timestamp tx ty tz qx qy qz qw
        1772004062.900 -0.000113 -0.000093 0.000036 0 0 0 1

    Returns a list of ``(timestamp_sec, x, y)`` tuples sorted by timestamp.
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
    """Binary-search for the pose closest in time to ``query_ts``."""
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


# ---------------------------------------------------------------------------
# Distance sampler (DB + query split, used by build_database_3d's
# legacy pipelines and by the platform-specific extract_from_* helpers).
# The DB-only variant lives in vpr_preprocess.py.
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
# JPEG save helpers (used as ProcessPoolExecutor workers and inline)
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
    """Decode, undistort (fisheye or pinhole), save as JPEG.

    ``undistorter`` must expose ``undistort_bytes(jpeg_bytes) -> RGB ndarray``;
    both :class:`undistort.PolyFisheyeUndistorter` and
    :class:`undistort2.PinHoleUndistorter` conform to that contract.
    """
    rgb = undistorter.undistort_bytes(img_bytes)
    Image.fromarray(rgb).save(path, quality=quality)
    return path


# ---------------------------------------------------------------------------
# Output / progress helpers (DB + query layout)
# ---------------------------------------------------------------------------

def make_output_dirs(output_dir):
    """Create ``<output_dir>/{database,queries}`` and return both paths."""
    db_dir = os.path.join(output_dir, "database")
    q_dir = os.path.join(output_dir, "queries")
    os.makedirs(db_dir, exist_ok=True)
    os.makedirs(q_dir, exist_ok=True)
    return db_dir, q_dir


def wait_for_futures(futures):
    """Block on all ProcessPoolExecutor futures (re-raises worker errors)."""
    print(f"  Waiting for {len(futures)} image saves ...")
    for f in as_completed(futures):
        f.result()


def print_progress(raw_count, sampler, t0):
    rate = raw_count / (time.time() - t0)
    print(f"  Read {raw_count} frames "
          f"(db={sampler.db_count}, queries={sampler.q_count}, "
          f"skipped={sampler.skipped}) [{rate:.0f} fr/s]")


def print_summary(raw_count, sampler, unmatched, t0, db_dir, q_dir):
    elapsed = time.time() - t0
    print(f"\nDone in {elapsed:.1f}s! Read {raw_count} raw frames:")
    print(f"  Database: {sampler.db_count} images in {db_dir}")
    print(f"  Queries:  {sampler.q_count} images in {q_dir}")
    print(f"  Skipped:  {sampler.skipped}")
    if unmatched:
        print(f"  Unmatched: {unmatched} (no pose within threshold)")
