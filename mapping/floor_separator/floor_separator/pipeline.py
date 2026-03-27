"""Main pipeline orchestrator for multi-floor point cloud separation.

Chains Phase 0-4 and returns a unified result with per-point floor
labels, cut-plane elevations, and diagnostic data for visualisation.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from loguru import logger

from .export import export_per_floor_ply, reassign_stairwell_points
from .floor_detector import FloorBoundaryResult, detect_floor_boundaries
from .preprocessing import PreprocessedCloud, preprocess
from .segmentation import SegmentationResult, segment_floors
from .stairwell_detector import StairwellMaskResult, detect_stairwells


@dataclass
class FloorSeparatorConfig:
    """All tuneable parameters for the floor separation pipeline."""

    # Phase 0 — preprocessing
    voxel_size: float = 0.10
    normal_knn: int = 30

    # Phase 1 — stairwell detection
    expected_floor_height: float = 3.0
    stairwell_grid_cell: float = 0.5
    stairwell_z_bin: float = 0.15
    stairwell_min_col_pts: int = 20
    stairwell_occupancy_thresh: float = 0.5
    stairwell_min_area: float = 4.0
    stairwell_max_area: float = 100.0
    skip_stairwell_mask: bool = False

    # Phase 2 — floor boundary detection
    num_floors: int | None = None
    histogram_bin_size: float = 0.05
    smooth_sigma_bins: float = 3.0
    normal_filter_angle_deg: float = 15.0
    min_floor_height: float = 2.0
    max_floor_height: float = 5.0
    peak_height_frac: float = 0.10
    peak_prominence_frac: float = 0.10

    # Phase 3 — segmentation
    ccl_voxel_size: float = 0.15
    ccl_min_component: int = 100
    enable_ccl: bool = True

    # Phase 4 — export
    boundary_margin: float = 0.0
    output_dir: Path | None = None
    output_format: str = "ply"


@dataclass
class FloorSeparatorResult:
    """Complete output of the floor separation pipeline."""

    floor_labels: np.ndarray  # (N,) int per full-res point
    cut_planes: np.ndarray  # (K,) sorted Z-values
    n_floors: int
    floor_z_ranges: np.ndarray  # (n_floors, 2)
    noise_mask: np.ndarray  # (N,) bool
    exported_files: list[Path]

    # Intermediate results for debug visualisation
    preprocessed: PreprocessedCloud | None = None
    stairwell_result: StairwellMaskResult | None = None
    boundary_result: FloorBoundaryResult | None = None
    segmentation_result: SegmentationResult | None = None


def _validate_inputs(
    positions: np.ndarray, colors: np.ndarray | None
) -> None:
    """Validate pipeline inputs, raising ValueError on bad data.

    Args:
        positions: Expected (N, 3) float array.
        colors: Expected (N, 3) uint8 array or None.

    Raises:
        ValueError: If inputs have wrong shape, are empty, or contain NaN.
    """
    if not isinstance(positions, np.ndarray):
        raise ValueError(f"positions must be a numpy array, got {type(positions)}")
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError(f"positions must be (N, 3), got shape {positions.shape}")
    if len(positions) == 0:
        raise ValueError("positions is empty (0 points)")
    if np.any(np.isnan(positions)):
        raise ValueError("positions contains NaN values")
    if colors is not None:
        if not isinstance(colors, np.ndarray):
            raise ValueError(f"colors must be a numpy array or None, got {type(colors)}")
        if colors.shape[0] != positions.shape[0]:
            raise ValueError(
                f"colors length {colors.shape[0]} != positions length {positions.shape[0]}"
            )
        if colors.ndim != 2 or colors.shape[1] != 3:
            raise ValueError(f"colors must be (N, 3), got shape {colors.shape}")


def run_pipeline(
    positions: np.ndarray,
    colors: np.ndarray | None = None,
    config: FloorSeparatorConfig | None = None,
) -> FloorSeparatorResult:
    """Run the full floor separation pipeline.

    Args:
        positions: (N, 3) float32/64 point positions (Z-up coordinate system).
        colors: (N, 3) uint8 RGB colours, or None.
        config: Pipeline configuration.  Uses defaults if None.

    Returns:
        FloorSeparatorResult with per-point floor labels and diagnostics.

    Raises:
        ValueError: If positions/colors have invalid shape, are empty, or
            contain NaN values.
    """
    _validate_inputs(positions, colors)

    if config is None:
        config = FloorSeparatorConfig()

    t_total = time.perf_counter()

    # ------------------------------------------------------------------
    # Phase 0: Preprocessing
    # ------------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("PHASE 0: Preprocessing")
    logger.info("=" * 60)
    t0 = time.perf_counter()

    preprocessed = preprocess(
        positions,
        colors,
        voxel_size=config.voxel_size,
        normal_knn=config.normal_knn,
    )

    logger.info("Phase 0 done in {:.1f}s", time.perf_counter() - t0)

    # ------------------------------------------------------------------
    # Phase 1: Stairwell detection
    # ------------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("PHASE 1: Stairwell Detection")
    logger.info("=" * 60)
    t0 = time.perf_counter()

    if config.skip_stairwell_mask:
        logger.info("Stairwell masking disabled by config")
        stairwell_result = StairwellMaskResult(
            clean_mask=np.ones(len(preprocessed.positions), dtype=bool),
            stairwell_mask=np.zeros(len(preprocessed.positions), dtype=bool),
            stairwell_regions=[],
            grid_cell_size=config.stairwell_grid_cell,
        )
    else:
        stairwell_result = detect_stairwells(
            preprocessed.positions,
            expected_floor_height=config.expected_floor_height,
            grid_cell_size=config.stairwell_grid_cell,
            z_bin_size=config.stairwell_z_bin,
            min_column_points=config.stairwell_min_col_pts,
            occupancy_ratio_thresh=config.stairwell_occupancy_thresh,
            min_area_m2=config.stairwell_min_area,
            max_area_m2=config.stairwell_max_area,
        )

    logger.info("Phase 1 done in {:.1f}s", time.perf_counter() - t0)

    # ------------------------------------------------------------------
    # Phase 2: Floor boundary detection
    # ------------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("PHASE 2: Floor Boundary Detection")
    logger.info("=" * 60)
    t0 = time.perf_counter()

    clean_pos = preprocessed.positions[stairwell_result.clean_mask]
    clean_normals = preprocessed.normals[stairwell_result.clean_mask]

    boundary_result = detect_floor_boundaries(
        clean_pos,
        clean_normals,
        num_floors=config.num_floors,
        bin_size=config.histogram_bin_size,
        smooth_sigma_bins=config.smooth_sigma_bins,
        normal_filter_angle_deg=config.normal_filter_angle_deg,
        min_floor_height=config.min_floor_height,
        max_floor_height=config.max_floor_height,
        peak_height_frac=config.peak_height_frac,
        peak_prominence_frac=config.peak_prominence_frac,
    )

    if len(boundary_result.cut_planes) == 0:
        logger.error("No floor boundaries detected — aborting pipeline")
        return FloorSeparatorResult(
            floor_labels=np.zeros(len(positions), dtype=np.int32),
            cut_planes=np.array([]),
            n_floors=1,
            floor_z_ranges=np.array([[positions[:, 2].min(), positions[:, 2].max()]]),
            noise_mask=np.zeros(len(positions), dtype=bool),
            exported_files=[],
            preprocessed=preprocessed,
            stairwell_result=stairwell_result,
            boundary_result=boundary_result,
        )

    logger.info("Phase 2 done in {:.1f}s", time.perf_counter() - t0)

    # ------------------------------------------------------------------
    # Phase 3: Per-floor segmentation with CCL cleanup
    # ------------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("PHASE 3: Per-Floor Segmentation")
    logger.info("=" * 60)
    t0 = time.perf_counter()

    seg_result = segment_floors(
        positions,
        boundary_result.cut_planes,
        ccl_voxel_size=config.ccl_voxel_size,
        min_component_voxels=config.ccl_min_component,
        enable_ccl=config.enable_ccl,
    )

    logger.info("Phase 3 done in {:.1f}s", time.perf_counter() - t0)

    # ------------------------------------------------------------------
    # Phase 4: Stairwell reassignment + export
    # ------------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("PHASE 4: Stairwell Reassignment & Export")
    logger.info("=" * 60)
    t0 = time.perf_counter()

    # Map the analysis-cloud stairwell mask back to full resolution.
    # Since the analysis cloud is a voxel downsample of the full cloud,
    # we re-run the stairwell XY grid test on the full cloud.
    if not config.skip_stairwell_mask and len(stairwell_result.stairwell_regions) > 0:
        full_stairwell_mask = _project_stairwell_mask_to_full(
            positions,
            preprocessed.positions,
            stairwell_result,
            config.stairwell_grid_cell,
        )
        final_labels = reassign_stairwell_points(
            seg_result.floor_labels,
            full_stairwell_mask,
            positions[:, 2],
            boundary_result.cut_planes,
            boundary_margin=config.boundary_margin,
        )
    else:
        final_labels = seg_result.floor_labels
        full_stairwell_mask = np.zeros(len(positions), dtype=bool)

    exported: list[Path] = []
    if config.output_dir is not None:
        exported = export_per_floor_ply(
            positions,
            colors,
            final_labels,
            seg_result.noise_mask,
            seg_result.n_floors,
            config.output_dir,
            floor_z_ranges=seg_result.floor_z_ranges,
            file_format=config.output_format,
        )

    logger.info("Phase 4 done in {:.1f}s", time.perf_counter() - t0)

    elapsed = time.perf_counter() - t_total
    logger.info("=" * 60)
    logger.info("Pipeline complete in {:.1f}s — {} floors detected", elapsed, seg_result.n_floors)
    logger.info("=" * 60)

    return FloorSeparatorResult(
        floor_labels=final_labels,
        cut_planes=boundary_result.cut_planes,
        n_floors=seg_result.n_floors,
        floor_z_ranges=seg_result.floor_z_ranges,
        noise_mask=seg_result.noise_mask,
        exported_files=exported,
        preprocessed=preprocessed,
        stairwell_result=stairwell_result,
        boundary_result=boundary_result,
        segmentation_result=seg_result,
    )


def _project_stairwell_mask_to_full(
    full_positions: np.ndarray,
    analysis_positions: np.ndarray,
    stairwell_result: StairwellMaskResult,
    grid_cell_size: float,
) -> np.ndarray:
    """Re-apply the XY stairwell grid mask to full-resolution points.

    The stairwell detection ran on the analysis (downsampled) cloud.
    We reconstruct the 2D XY grid mask and apply it to the full cloud
    to get a per-point stairwell flag at full resolution.
    """
    ax = analysis_positions[:, 0]
    ay = analysis_positions[:, 1]
    x_min, y_min = ax.min(), ay.min()
    x_max, y_max = ax.max(), ay.max()

    nx = int(np.ceil((x_max - x_min) / grid_cell_size)) + 1
    ny = int(np.ceil((y_max - y_min) / grid_cell_size)) + 1

    stairwell_grid = np.zeros((nx, ny), dtype=bool)
    a_ix = np.clip(((ax - x_min) / grid_cell_size).astype(np.int32), 0, nx - 1)
    a_iy = np.clip(((ay - y_min) / grid_cell_size).astype(np.int32), 0, ny - 1)

    stairwell_cells = stairwell_result.stairwell_mask
    stairwell_grid[a_ix[stairwell_cells], a_iy[stairwell_cells]] = True

    fx = full_positions[:, 0]
    fy = full_positions[:, 1]
    f_ix = np.clip(((fx - x_min) / grid_cell_size).astype(np.int32), 0, nx - 1)
    f_iy = np.clip(((fy - y_min) / grid_cell_size).astype(np.int32), 0, ny - 1)

    return stairwell_grid[f_ix, f_iy]
