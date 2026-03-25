"""Phase 4: Stairwell reassignment and per-floor PLY export.

Assigns stairwell points (masked in Phase 1) to the appropriate floor
based on Z-range, optionally duplicates boundary points, and exports
per-floor point clouds as PLY files in parallel.
"""

from __future__ import annotations

import os
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import numpy as np
import open3d as o3d
from loguru import logger


def reassign_stairwell_points(
    floor_labels: np.ndarray,
    stairwell_mask_full: np.ndarray,
    z_full: np.ndarray,
    cut_planes: np.ndarray,
    boundary_margin: float = 0.0,
) -> np.ndarray:
    """Assign stairwell points to their containing floor by Z-range.

    Args:
        floor_labels: (N,) current floor labels (stairwell points may be
            assigned but not yet validated).
        stairwell_mask_full: (N,) bool — True for stairwell region points.
        z_full: (N,) Z-coordinates of all points.
        cut_planes: (K,) sorted cut-plane Z-values.
        boundary_margin: If > 0, duplicate stairwell points within this
            distance of a cut plane to both adjacent floors.  Currently
            the duplication is handled by returning a separate array.

    Returns:
        Updated floor_labels (N,) with stairwell points reassigned.
    """
    labels = floor_labels.copy()

    stairwell_indices = np.where(stairwell_mask_full)[0]
    if len(stairwell_indices) == 0:
        return labels

    stairwell_z = z_full[stairwell_indices]
    stairwell_floor = np.digitize(stairwell_z, cut_planes).astype(np.int32)
    labels[stairwell_indices] = stairwell_floor

    logger.info("Reassigned {} stairwell points to floors", len(stairwell_indices))
    return labels


def _write_floor_file(
    positions: np.ndarray,
    colors: np.ndarray | None,
    mask: np.ndarray,
    out_path: Path,
    floor_idx: int,
) -> tuple[int, Path, int]:
    """Write a single floor's point cloud file (PLY or PCD) for parallel execution."""
    n_pts = mask.sum()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(positions[mask].astype(np.float64))
    if colors is not None:
        c = colors[mask]
        if c.dtype == np.uint8:
            c = c.astype(np.float64) / 255.0
        pcd.colors = o3d.utility.Vector3dVector(c)

    # open3d infers format from file extension (.ply, .pcd, etc.)
    o3d.io.write_point_cloud(str(out_path), pcd, write_ascii=False)
    return floor_idx, out_path, n_pts


def export_per_floor_ply(
    positions: np.ndarray,
    colors: np.ndarray | None,
    floor_labels: np.ndarray,
    noise_mask: np.ndarray,
    n_floors: int,
    output_dir: Path,
    floor_z_ranges: np.ndarray | None = None,
    file_format: str = "ply",
) -> list[Path]:
    """Save each floor as a separate point-cloud file (PLY or PCD) in parallel.

    Args:
        positions: (N, 3) full-resolution positions.
        colors: (N, 3) uint8 RGB or None.
        floor_labels: (N,) per-point floor ID.
        noise_mask: (N,) bool — True for noise points to exclude.
        n_floors: Number of floors.
        output_dir: Directory to write PLY files into.
        floor_z_ranges: (n_floors, 2) optional, for filename annotation.

    Returns:
        List of paths to exported PLY files.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    saved: list[Path] = []

    max_workers = min(n_floors, os.cpu_count() or 1)

    tasks: list[tuple[int, np.ndarray, Path]] = []
    for fi in range(n_floors):
        mask = (floor_labels == fi) & (~noise_mask)
        n_pts = mask.sum()
        if n_pts == 0:
            logger.warning("Floor {} has 0 points after filtering, skipping", fi)
            continue

        ext = file_format.lower()
        if ext not in ("ply", "pcd"):
            ext = "ply"

        if floor_z_ranges is not None:
            lo, hi = floor_z_ranges[fi]
            fname = f"floor_{fi:02d}_z{lo:.1f}_{hi:.1f}.{ext}"
        else:
            fname = f"floor_{fi:02d}.{ext}"

        out_path = output_dir / fname
        tasks.append((fi, mask, out_path))

    with ThreadPoolExecutor(max_workers=max_workers) as pool:
        futures = {
            pool.submit(_write_floor_file, positions, colors, mask, out_path, fi): fi
            for fi, mask, out_path in tasks
        }
        for fut in as_completed(futures):
            fi, out_path, n_pts = fut.result()
            logger.info("Exported floor {}: {} pts -> {}", fi, n_pts, out_path)
            saved.append(out_path)

    saved.sort()
    return saved
