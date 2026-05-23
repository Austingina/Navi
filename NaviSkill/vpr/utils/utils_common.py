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
from collections import namedtuple
from concurrent.futures import ProcessPoolExecutor, as_completed

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


# ---------------------------------------------------------------------------
# L1 + L2 low-texture frame filter
# ---------------------------------------------------------------------------
# 用来剔除"白墙/玻璃幕墙/吊顶/纯地板"类没有判别力的关键帧。这种帧的全局
# 描述符容易塌缩到同一片狭窄子空间，跨 submap 互相 KNN 命中，是多楼层
# VPR 的主要误召回来源。
#
# 流程是两级廉价指标，全部在 CPU 上跑，单帧总耗时 < 10ms：
#   L1: cv2.Laplacian(gray).var() —— 抓全局糊（白墙 / 过曝玻璃 / 吊顶）。
#   L2: cv2.ORB FAST 关键点数      —— 抓"局部高频但整体空"的玻璃反光。
#
# 该函数返回值同时带回原始指标，便于线下调阈值（写进
# ``low_texture_filter_report.csv``）。

FrameQuality = namedtuple(
    "FrameQuality", ["kept", "reason", "lap_var", "orb_kpts"]
)


def assess_frame_quality(
    rgb_or_gray,
    min_lap_var: float = 60.0,
    min_orb_kpts: int = 80,
    orb_fast_threshold: int = 15,
    orb_nfeatures: int = 500,
) -> FrameQuality:
    """Score an image's discriminability for VPR (L1 + L2).

    Parameters
    ----------
    rgb_or_gray : np.ndarray
        HxWx3 uint8 RGB or HxW uint8 grayscale.
    min_lap_var : float
        Reject if Laplacian variance below this (L1).
    min_orb_kpts : int
        Reject if fewer than this many ORB keypoints (L2).
    orb_fast_threshold, orb_nfeatures :
        ``cv2.ORB_create`` parameters; defaults work well for 1024x768
        indoor frames.

    Returns
    -------
    FrameQuality(kept, reason, lap_var, orb_kpts)
        ``reason`` is one of ``"kept"``, ``"low_lap_var"``, ``"low_orb_kpts"``.
    """
    import cv2

    arr = rgb_or_gray
    if arr.ndim == 3:
        gray = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)
    else:
        gray = arr

    lap_var = float(cv2.Laplacian(gray, cv2.CV_32F).var())
    if lap_var < min_lap_var:
        return FrameQuality(False, "low_lap_var", lap_var, 0)

    orb = cv2.ORB_create(
        nfeatures=orb_nfeatures, fastThreshold=orb_fast_threshold
    )
    kpts = orb.detect(gray, None)
    n_kpts = int(len(kpts)) if kpts is not None else 0
    if n_kpts < min_orb_kpts:
        return FrameQuality(False, "low_orb_kpts", lap_var, n_kpts)

    return FrameQuality(True, "kept", lap_var, n_kpts)


def _assess_path_worker(args):
    """ProcessPool worker: load JPG -> FrameQuality. Top-level for picklability."""
    path, min_lap_var, min_orb_kpts = args
    import cv2

    bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    if bgr is None:
        return path, FrameQuality(False, "unreadable", 0.0, 0)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return path, assess_frame_quality(
        gray, min_lap_var=min_lap_var, min_orb_kpts=min_orb_kpts
    )


def filter_low_texture_directory(
    db_dir: str,
    *,
    min_lap_var: float = 60.0,
    min_orb_kpts: int = 80,
    num_workers: int = 4,
    delete: bool = True,
    report_path: str = None,
    logger=None,
):
    """Scan a database image directory and drop low-texture frames.

    Runs L1 (Laplacian variance) + L2 (ORB keypoint count) over every
    ``.jpg/.jpeg/.png`` in ``db_dir``. Frames that fail either gate are
    deleted (when ``delete=True``) and a per-frame report is written to
    ``report_path`` so the thresholds can be re-tuned offline.

    Parameters
    ----------
    db_dir : str
        Directory holding extracted database JPGs.
    min_lap_var, min_orb_kpts :
        Pass-through to :func:`assess_frame_quality`.
    num_workers : int
        ProcessPool size; <=1 runs serially in the calling process.
    delete : bool
        If False, only the report is written (dry-run).
    report_path : str, optional
        CSV destination; defaults to ``<db_dir>/../low_texture_filter_report.csv``.
    logger :
        loguru-style logger; defaults to ``loguru.logger``.

    Returns
    -------
    dict
        Summary stats: ``total``, ``kept``, ``filtered``, ``by_reason``,
        ``min_lap_var``, ``min_orb_kpts``, ``elapsed_sec``.
    """
    if logger is None:
        from loguru import logger as _logger
        logger = _logger

    exts = (".jpg", ".jpeg", ".png")
    paths = sorted(
        os.path.join(db_dir, p)
        for p in os.listdir(db_dir)
        if os.path.isfile(os.path.join(db_dir, p))
        and os.path.splitext(p)[1].lower() in exts
    )
    total = len(paths)
    if total == 0:
        logger.warning(f"filter_low_texture_directory: no images in {db_dir}")
        return {
            "total": 0, "kept": 0, "filtered": 0, "by_reason": {},
            "min_lap_var": min_lap_var, "min_orb_kpts": min_orb_kpts,
            "elapsed_sec": 0.0,
        }

    logger.info(
        f"L1+L2 filter: scanning {total} frames "
        f"(min_lap_var={min_lap_var}, min_orb_kpts={min_orb_kpts}, "
        f"workers={num_workers}, delete={delete}) ..."
    )

    t0 = time.time()
    tasks = [(p, min_lap_var, min_orb_kpts) for p in paths]
    results = []

    if num_workers <= 1:
        for t in tasks:
            results.append(_assess_path_worker(t))
    else:
        with ProcessPoolExecutor(max_workers=num_workers) as pool:
            for r in pool.map(_assess_path_worker, tasks, chunksize=16):
                results.append(r)

    if report_path is None:
        report_path = os.path.join(
            os.path.dirname(os.path.abspath(db_dir)),
            "low_texture_filter_report.csv",
        )
    with open(report_path, "w") as f:
        f.write("path,kept,reason,lap_var,orb_kpts\n")
        for p, q in results:
            f.write(
                f"{os.path.basename(p)},{int(q.kept)},{q.reason},"
                f"{q.lap_var:.2f},{q.orb_kpts}\n"
            )

    kept = 0
    filtered = 0
    by_reason = {}
    for p, q in results:
        if q.kept:
            kept += 1
        else:
            filtered += 1
            by_reason[q.reason] = by_reason.get(q.reason, 0) + 1
            if delete:
                try:
                    os.remove(p)
                except OSError as e:
                    logger.warning(f"  could not remove {p}: {e}")

    elapsed = time.time() - t0
    logger.info(
        f"L1+L2 filter done in {elapsed:.1f}s: "
        f"kept={kept}/{total} ({kept / total:.1%}), "
        f"filtered={filtered} ({by_reason})"
    )
    logger.info(f"  per-frame report -> {report_path}")

    return {
        "total": total,
        "kept": kept,
        "filtered": filtered,
        "by_reason": by_reason,
        "min_lap_var": float(min_lap_var),
        "min_orb_kpts": int(min_orb_kpts),
        "elapsed_sec": float(elapsed),
        "report_path": report_path,
    }
