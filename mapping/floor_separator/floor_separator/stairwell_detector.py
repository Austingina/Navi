"""Phase 1: Stairwell and elevator shaft detection via voxel-column analysis.

Stairwells create continuous vertical point distributions that corrupt
the Z-histogram.  This module detects them by analysing XY grid columns
for unusually tall, continuously-occupied Z spans, then masks those
regions so the histogram phase operates on clean floor/ceiling data only.

The column analysis is fully vectorized with numpy (no Python loops over
individual columns).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from loguru import logger
from scipy.ndimage import label as ndimage_label


@dataclass
class StairwellMaskResult:
    """Output of stairwell detection."""

    clean_mask: np.ndarray  # (M,) bool — True for non-stairwell points
    stairwell_mask: np.ndarray  # (M,) bool — True for stairwell points
    stairwell_regions: list[dict]  # per-region metadata (centroid, area, etc.)
    grid_cell_size: float


def detect_stairwells(
    positions: np.ndarray,
    expected_floor_height: float = 3.0,
    grid_cell_size: float = 0.5,
    z_bin_size: float = 0.15,
    min_column_points: int = 20,
    occupancy_ratio_thresh: float = 0.5,
    min_area_m2: float = 4.0,
    max_area_m2: float = 100.0,
) -> StairwellMaskResult:
    """Detect stairwell / elevator shaft regions in the point cloud.

    Args:
        positions: (M, 3) analysis-resolution points.
        expected_floor_height: Typical floor-to-floor height in metres.
        grid_cell_size: XY grid cell size in metres.
        z_bin_size: Vertical bin size for occupancy ratio calculation.
        min_column_points: Ignore columns with fewer points.
        occupancy_ratio_thresh: Minimum ratio of occupied Z-bins for a
            column to be flagged as a vertical transition.
        min_area_m2: Minimum connected footprint area to keep.
        max_area_m2: Maximum connected footprint area (filter atriums).

    Returns:
        StairwellMaskResult with boolean masks and region metadata.
    """
    if len(positions) == 0:
        logger.info("Empty point cloud, skipping stairwell detection")
        return StairwellMaskResult(
            clean_mask=np.ones(0, dtype=bool),
            stairwell_mask=np.zeros(0, dtype=bool),
            stairwell_regions=[],
            grid_cell_size=grid_cell_size,
        )
    if grid_cell_size <= 0:
        raise ValueError(f"grid_cell_size must be positive, got {grid_cell_size}")

    x, y, z = positions[:, 0], positions[:, 1], positions[:, 2]
    x_min, x_max = x.min(), x.max()
    y_min, y_max = y.min(), y.max()

    nx = int(np.ceil((x_max - x_min) / grid_cell_size)) + 1
    ny = int(np.ceil((y_max - y_min) / grid_cell_size)) + 1

    ix = np.clip(((x - x_min) / grid_cell_size).astype(np.int32), 0, nx - 1)
    iy = np.clip(((y - y_min) / grid_cell_size).astype(np.int32), 0, ny - 1)

    cell_id = ix * ny + iy
    n_cells = nx * ny

    z_span_threshold = 1.5 * expected_floor_height
    transition_grid = np.zeros((nx, ny), dtype=bool)

    logger.info(
        "Stairwell detection: {} x {} grid ({:.1f}m cells), z_span_thresh={:.1f}m",
        nx, ny, grid_cell_size, z_span_threshold,
    )

    # --- Fully vectorized column analysis (no Python for-loop) ---
    col_counts = np.bincount(cell_id, minlength=n_cells)
    col_z_min = np.full(n_cells, np.inf)
    col_z_max = np.full(n_cells, -np.inf)
    np.minimum.at(col_z_min, cell_id, z)
    np.maximum.at(col_z_max, cell_id, z)

    col_z_span = col_z_max - col_z_min

    candidate_mask = (
        (col_counts >= min_column_points) & (col_z_span > z_span_threshold)
    )
    candidate_ids = np.where(candidate_mask)[0]

    if len(candidate_ids) > 0:
        col_num_bins = np.ceil(col_z_span[candidate_ids] / z_bin_size).astype(np.int32)
        col_num_bins = np.maximum(col_num_bins, 1)

        point_in_candidate = np.isin(cell_id, candidate_ids)
        cand_cell_id = cell_id[point_in_candidate]
        cand_z = z[point_in_candidate]
        cand_z_min = col_z_min[cand_cell_id]
        cand_num_bins = col_num_bins[
            np.searchsorted(candidate_ids, cand_cell_id)
        ]

        cand_bin_idx = np.clip(
            ((cand_z - cand_z_min) / z_bin_size).astype(np.int32),
            0, cand_num_bins - 1,
        )

        max_bins_any = int(col_num_bins.max())
        composite_key = cand_cell_id.astype(np.int64) * max_bins_any + cand_bin_idx

        unique_composite = np.unique(composite_key)
        occupied_cell_ids = unique_composite // max_bins_any
        occupied_per_col = np.bincount(
            np.searchsorted(candidate_ids, occupied_cell_ids),
            minlength=len(candidate_ids),
        )

        ratios = occupied_per_col / col_num_bins
        flagged_candidates = candidate_ids[ratios > occupancy_ratio_thresh]

        for cid in flagged_candidates:
            ci, cj = divmod(int(cid), ny)
            transition_grid[ci, cj] = True

    flagged_count = transition_grid.sum()
    logger.info("Flagged {} / {} columns as vertical transitions", flagged_count, n_cells)

    if flagged_count == 0:
        logger.info("No stairwell regions detected")
        clean = np.ones(len(positions), dtype=bool)
        return StairwellMaskResult(
            clean_mask=clean,
            stairwell_mask=~clean,
            stairwell_regions=[],
            grid_cell_size=grid_cell_size,
        )

    structure_8conn = np.ones((3, 3), dtype=np.int32)
    labeled, n_components = ndimage_label(transition_grid, structure=structure_8conn)

    cell_area = grid_cell_size * grid_cell_size
    comp_sizes = np.bincount(labeled.ravel())
    comp_areas = comp_sizes * cell_area

    regions: list[dict] = []
    keep_mask_grid = np.zeros((nx, ny), dtype=bool)

    for comp_id in range(1, n_components + 1):
        area = comp_areas[comp_id]
        if area < min_area_m2 or area > max_area_m2:
            continue

        comp_cells = labeled == comp_id
        ci_indices, cj_indices = np.where(comp_cells)
        cx = x_min + (ci_indices.mean() + 0.5) * grid_cell_size
        cy = y_min + (cj_indices.mean() + 0.5) * grid_cell_size

        keep_mask_grid[comp_cells] = True
        regions.append(
            {
                "id": comp_id,
                "area_m2": float(area),
                "n_cells": int(comp_sizes[comp_id]),
                "centroid_xy": (float(cx), float(cy)),
            }
        )

    logger.info(
        "Kept {} stairwell/elevator regions (area {:.0f}-{:.0f} m²) from {} components",
        len(regions), min_area_m2, max_area_m2, n_components,
    )
    for r in regions:
        logger.info(
            "  Region {}: {:.1f} m², centroid ({:.1f}, {:.1f})",
            r["id"], r["area_m2"], r["centroid_xy"][0], r["centroid_xy"][1],
        )

    stairwell_mask = keep_mask_grid[ix, iy]
    clean_mask = ~stairwell_mask

    logger.info(
        "Stairwell mask: {} stairwell pts, {} clean pts",
        stairwell_mask.sum(), clean_mask.sum(),
    )

    return StairwellMaskResult(
        clean_mask=clean_mask,
        stairwell_mask=stairwell_mask,
        stairwell_regions=regions,
        grid_cell_size=grid_cell_size,
    )
