"""Phase 3: Per-floor segmentation with connected-component cleanup.

Slices the full-resolution cloud using cut planes, then runs 3D
connected-component labeling on each floor to discard noise clusters
and prevent cross-floor leakage.

Per-floor CCL runs in parallel using ThreadPoolExecutor (scipy's
ndimage.label releases the GIL, enabling true parallel execution).
"""

from __future__ import annotations

import os
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass

import numpy as np
from loguru import logger
from scipy.ndimage import label as ndimage_label


@dataclass
class SegmentationResult:
    """Output of per-floor segmentation."""

    floor_labels: np.ndarray  # (N,) int — floor ID for each full-res point
    n_floors: int
    floor_z_ranges: np.ndarray  # (n_floors, 2) — [lo, hi] per floor
    noise_mask: np.ndarray  # (N,) bool — True for points discarded by CCL


def _build_6conn_structure() -> np.ndarray:
    """3x3x3 structure for 6-connectivity (face-adjacent only)."""
    s = np.zeros((3, 3, 3), dtype=np.int32)
    s[1, 1, 0] = 1
    s[1, 1, 2] = 1
    s[1, 0, 1] = 1
    s[1, 2, 1] = 1
    s[0, 1, 1] = 1
    s[2, 1, 1] = 1
    s[1, 1, 1] = 1
    return s


_STRUCT_6CONN = _build_6conn_structure()


def _ccl_cleanup_floor(
    positions: np.ndarray,
    point_indices: np.ndarray,
    ccl_voxel_size: float,
    min_component_voxels: int,
) -> np.ndarray:
    """Run 3D CCL on a single floor's points and return a noise mask.

    Args:
        positions: (K, 3) positions for points on this floor.
        point_indices: (K,) original indices into the full cloud.
        ccl_voxel_size: Voxel size for the 3D occupancy grid.
        min_component_voxels: Minimum occupied voxels for a component.

    Returns:
        Boolean array (K,) — True for noise points to discard.
    """
    if len(positions) == 0:
        return np.zeros(0, dtype=bool)

    p_min = positions.min(axis=0)
    voxel_idx = ((positions - p_min) / ccl_voxel_size).astype(np.int32)

    grid_shape = voxel_idx.max(axis=0) + 1

    grid_size = int(grid_shape[0]) * int(grid_shape[1]) * int(grid_shape[2])
    if grid_size > 500_000_000:
        logger.warning(
            "CCL grid too large ({:.0f}M voxels), skipping cleanup for this floor",
            grid_size / 1e6,
        )
        return np.zeros(len(positions), dtype=bool)

    voxel_keys = (
        voxel_idx[:, 0].astype(np.int64) * grid_shape[1] * grid_shape[2]
        + voxel_idx[:, 1].astype(np.int64) * grid_shape[2]
        + voxel_idx[:, 2].astype(np.int64)
    )
    unique_keys, counts = np.unique(voxel_keys, return_counts=True)

    min_pts_per_voxel = 3
    valid_keys = unique_keys[counts >= min_pts_per_voxel]

    occupancy = np.zeros(grid_shape, dtype=bool)
    vi = valid_keys // (grid_shape[1] * grid_shape[2])
    remainder = valid_keys % (grid_shape[1] * grid_shape[2])
    vj = remainder // grid_shape[2]
    vk = remainder % grid_shape[2]
    occupancy[vi, vj, vk] = True

    labeled, n_components = ndimage_label(occupancy, structure=_STRUCT_6CONN)

    if n_components == 0:
        return np.zeros(len(positions), dtype=bool)

    component_sizes = np.bincount(labeled.ravel())[1:]
    valid_labels = np.zeros(n_components + 1, dtype=bool)
    valid_labels[1:] = component_sizes >= min_component_voxels

    point_voxel_labels = labeled[voxel_idx[:, 0], voxel_idx[:, 1], voxel_idx[:, 2]]
    noise = ~valid_labels[point_voxel_labels]

    return noise


def _process_floor(
    fi: int,
    floor_mask: np.ndarray,
    positions: np.ndarray,
    lo: float,
    hi: float,
    ccl_voxel_size: float,
    min_component_voxels: int,
) -> tuple[int, np.ndarray, int]:
    """Process a single floor's CCL cleanup (designed for parallel execution).

    Returns:
        (floor_index, noise_indices_in_full_cloud, n_removed)
    """
    floor_indices = np.where(floor_mask)[0]
    floor_pos = positions[floor_indices]

    noise_local = _ccl_cleanup_floor(
        floor_pos, floor_indices, ccl_voxel_size, min_component_voxels,
    )
    noise_global_indices = floor_indices[noise_local]
    return fi, noise_global_indices, int(noise_local.sum())


def segment_floors(
    positions: np.ndarray,
    cut_planes: np.ndarray,
    ccl_voxel_size: float = 0.15,
    min_component_voxels: int = 100,
    enable_ccl: bool = True,
) -> SegmentationResult:
    """Segment the full-resolution cloud into per-floor labels.

    Per-floor CCL runs in parallel threads (scipy releases the GIL).

    Args:
        positions: (N, 3) full-resolution point positions.
        cut_planes: (K,) sorted Z-values from floor detection.
        ccl_voxel_size: Voxel size for connected-component cleanup.
        min_component_voxels: Minimum component size to keep.
        enable_ccl: Whether to run CCL cleanup (disable for speed).

    Returns:
        SegmentationResult with per-point floor labels.
    """
    if len(positions) == 0:
        logger.warning("Empty point cloud, returning single empty floor")
        return SegmentationResult(
            floor_labels=np.array([], dtype=np.int32),
            n_floors=0,
            floor_z_ranges=np.empty((0, 2)),
            noise_mask=np.array([], dtype=bool),
        )
    if ccl_voxel_size <= 0:
        raise ValueError(f"ccl_voxel_size must be positive, got {ccl_voxel_size}")

    z = positions[:, 2]
    z_min, z_max = z.min(), z.max()

    boundaries = np.concatenate([[z_min - 0.01], cut_planes, [z_max + 0.01]])
    n_floors = len(boundaries) - 1

    floor_labels = np.digitize(z, cut_planes).astype(np.int32)
    floor_z_ranges = np.column_stack([boundaries[:-1], boundaries[1:]])
    noise_mask = np.zeros(len(positions), dtype=bool)

    max_workers = min(n_floors, os.cpu_count() or 1)
    logger.info(
        "Segmenting {} points into {} floors (CCL={}, workers={})",
        len(positions), n_floors, enable_ccl, max_workers if enable_ccl else 0,
    )

    for fi in range(n_floors):
        n_pts = (floor_labels == fi).sum()
        lo, hi = floor_z_ranges[fi]
        logger.info("  Floor {}: {} pts, Z=[{:.2f}, {:.2f}]", fi, n_pts, lo, hi)

    if not enable_ccl:
        return SegmentationResult(
            floor_labels=floor_labels,
            n_floors=n_floors,
            floor_z_ranges=floor_z_ranges,
            noise_mask=noise_mask,
        )

    with ThreadPoolExecutor(max_workers=max_workers) as pool:
        futures = {}
        for fi in range(n_floors):
            floor_mask = floor_labels == fi
            n_pts = floor_mask.sum()
            if n_pts < min_component_voxels:
                continue
            lo, hi = floor_z_ranges[fi]
            fut = pool.submit(
                _process_floor,
                fi, floor_mask, positions, lo, hi,
                ccl_voxel_size, min_component_voxels,
            )
            futures[fut] = fi

        for fut in as_completed(futures):
            fi, noise_indices, n_removed = fut.result()
            if n_removed > 0:
                noise_mask[noise_indices] = True
                n_pts = (floor_labels == fi).sum()
                logger.info(
                    "    Floor {} CCL removed {} noise points ({:.1f}%)",
                    fi, n_removed, 100.0 * n_removed / max(n_pts, 1),
                )

    total_noise = noise_mask.sum()
    if total_noise > 0:
        logger.info("Total noise points removed by CCL: {}", total_noise)

    return SegmentationResult(
        floor_labels=floor_labels,
        n_floors=n_floors,
        floor_z_ranges=floor_z_ranges,
        noise_mask=noise_mask,
    )
