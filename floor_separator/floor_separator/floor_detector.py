"""Phase 2: Normal-filtered Z-histogram floor boundary detection.

Pre-filters points to keep only near-horizontal surfaces (floors and
ceilings), builds a Z-density histogram, and extracts cut-plane
elevations from the deepest valleys between peaks.

When ``num_floors`` is ``None`` the number of floors is determined
automatically using three complementary heuristics:

1. **Valley-depth gap analysis** — sort valleys by depth, find the
   largest relative drop between consecutive depths (the "elbow").
   Valleys above the elbow are real floor boundaries.
2. **Height-range validation** — any resulting segment taller than
   ``max_floor_height`` is recursively split at the next-deepest
   valley within that segment.
3. **Minimum depth ratio** — valleys whose depth is less than 5 % of
   the deepest valley are always discarded (noise).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from loguru import logger
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks


@dataclass
class FloorBoundaryResult:
    """Output of floor boundary detection."""

    cut_planes: np.ndarray  # (K,) sorted Z-values of floor boundaries
    peak_z: np.ndarray  # Z-values of detected histogram peaks
    histogram: np.ndarray  # raw histogram counts
    bin_centers: np.ndarray  # histogram bin centres
    smoothed: np.ndarray  # smoothed histogram
    n_horizontal_pts: int  # points passing normal filter
    auto_detected_n_floors: int | None = None
    diagnostics: dict = field(default_factory=dict)


def _filter_horizontal_normals(
    positions: np.ndarray,
    normals: np.ndarray,
    max_angle_deg: float,
) -> np.ndarray:
    """Return mask of points whose normals are near-vertical (floor/ceiling).

    Args:
        positions: (M, 3) point positions (unused, kept for API clarity).
        normals: (M, 3) estimated normals.
        max_angle_deg: Maximum angle from vertical (Z-axis) in degrees.

    Returns:
        Boolean mask (M,) — True for near-horizontal surface points.
    """
    cos_thresh = np.cos(np.deg2rad(max_angle_deg))
    return np.abs(normals[:, 2]) > cos_thresh


def _select_valleys_auto(
    valleys: list[tuple[float, float, int]],
    z_min: float,
    z_max: float,
    min_floor_height: float,
    max_floor_height: float,
) -> list[tuple[float, float, int]]:
    """Automatically decide which valleys are real floor boundaries.

    Strategy:
      1. Discard valleys whose depth < 5 % of the deepest.
      2. Sort remaining by depth (descending), find the "elbow" —
         the index where the ratio depth[i+1]/depth[i] drops most.
      3. Keep valleys above the elbow.
      4. Validate resulting segment heights: if any segment exceeds
         *max_floor_height*, pull in the next-deepest unused valley
         that falls inside it.

    Args:
        valleys: list of (z_value, depth, bin_index) tuples.
        z_min: Lowest Z in the cloud.
        z_max: Highest Z in the cloud.
        min_floor_height: Segments shorter than this are merged.
        max_floor_height: Segments taller than this are split.

    Returns:
        Filtered list of valleys, sorted by Z.
    """
    if not valleys:
        return []

    max_depth = max(v[1] for v in valleys)
    depth_floor = 0.05 * max_depth
    candidates = [v for v in valleys if v[1] >= depth_floor]

    if not candidates:
        return []

    by_depth = sorted(candidates, key=lambda v: -v[1])
    depths = np.array([v[1] for v in by_depth])

    if len(depths) == 1:
        selected = set(range(1))
    else:
        ratios = depths[1:] / np.maximum(depths[:-1], 1e-12)
        elbow_idx = int(np.argmin(ratios))
        n_keep = elbow_idx + 1

        n_keep = max(n_keep, 1)
        selected = set(range(n_keep))

        logger.info(
            "Valley depth gap analysis: depths={}, ratios={}, elbow at index {} -> keep {}",
            [f"{d:.0f}" for d in depths[:8]],
            [f"{r:.2f}" for r in ratios[:7]],
            elbow_idx,
            n_keep,
        )

    kept = sorted([by_depth[i] for i in selected], key=lambda v: v[0])

    boundaries_z = [v[0] for v in kept]
    edges = [z_min] + boundaries_z + [z_max]
    unused = sorted(
        [v for i, v in enumerate(by_depth) if i not in selected],
        key=lambda v: -v[1],
    )

    changed = True
    while changed:
        changed = False
        new_edges = [z_min] + sorted([v[0] for v in kept]) + [z_max]
        for seg_i in range(len(new_edges) - 1):
            seg_height = new_edges[seg_i + 1] - new_edges[seg_i]
            if seg_height > max_floor_height and unused:
                for ui, uv in enumerate(unused):
                    if new_edges[seg_i] < uv[0] < new_edges[seg_i + 1]:
                        kept.append(uv)
                        unused.pop(ui)
                        changed = True
                        logger.info(
                            "Segment [{:.1f}, {:.1f}] ({:.1f}m) too tall, "
                            "inserting valley at z={:.2f} (depth={:.0f})",
                            new_edges[seg_i],
                            new_edges[seg_i + 1],
                            seg_height,
                            uv[0],
                            uv[1],
                        )
                        break

    kept.sort(key=lambda v: v[0])

    final_edges = [z_min] + [v[0] for v in kept] + [z_max]
    merged: list[tuple[float, float, int]] = []
    for v in kept:
        if merged:
            gap = v[0] - merged[-1][0]
            if gap < min_floor_height:
                if v[1] > merged[-1][1]:
                    merged[-1] = v
                continue
        merged.append(v)

    if len(merged) != len(kept):
        logger.info(
            "Merged {} valleys too close together -> {} boundaries",
            len(kept) - len(merged),
            len(merged),
        )

    return merged


def detect_floor_boundaries(
    positions: np.ndarray,
    normals: np.ndarray,
    num_floors: int | None = None,
    bin_size: float = 0.05,
    smooth_sigma_bins: float = 3.0,
    normal_filter_angle_deg: float = 15.0,
    min_floor_height: float = 2.0,
    max_floor_height: float = 5.0,
    peak_height_frac: float = 0.10,
    peak_prominence_frac: float = 0.10,
) -> FloorBoundaryResult:
    """Detect floor boundary elevations from point cloud with normals.

    When *num_floors* is None, the floor count is determined
    automatically via valley-depth gap analysis and height validation.

    Args:
        positions: (M, 3) analysis-cloud positions.
        normals: (M, 3) surface normals.
        num_floors: Expected number of floors.  ``None`` = auto-detect.
        bin_size: Z-histogram bin width in metres.
        smooth_sigma_bins: Gaussian smoothing sigma in bins.
        normal_filter_angle_deg: Max angle from vertical for the
            normal pre-filter (degrees).  Set to 90 to disable.
        min_floor_height: Minimum floor-to-floor height (metres).
        max_floor_height: Maximum allowed segment height before
            forced splitting (metres).  Only used in auto mode.
        peak_height_frac: Minimum peak height as fraction of max.
        peak_prominence_frac: Minimum peak prominence as fraction of max.

    Returns:
        FloorBoundaryResult with cut-plane elevations and diagnostics.
    """
    if len(positions) == 0:
        logger.warning("Empty point cloud, returning no boundaries")
        return FloorBoundaryResult(
            cut_planes=np.array([]),
            peak_z=np.array([]),
            histogram=np.array([]),
            bin_centers=np.array([]),
            smoothed=np.array([]),
            n_horizontal_pts=0,
        )
    if bin_size <= 0:
        raise ValueError(f"bin_size must be positive, got {bin_size}")

    horiz_mask = _filter_horizontal_normals(
        positions, normals, normal_filter_angle_deg
    )
    n_horiz = horiz_mask.sum()
    logger.info(
        "Normal filter (angle<{}°): {} / {} points are horizontal surfaces ({:.1f}%)",
        normal_filter_angle_deg,
        n_horiz,
        len(positions),
        100.0 * n_horiz / max(len(positions), 1),
    )

    z = positions[horiz_mask, 2] if n_horiz > 100 else positions[:, 2]
    if n_horiz <= 100:
        logger.warning(
            "Too few horizontal points ({}), falling back to all points", n_horiz
        )
        n_horiz = len(positions)

    z_min, z_max = z.min(), z.max()
    if z_max - z_min < bin_size:
        logger.warning("Z range ({:.3f} m) smaller than bin_size, returning no boundaries", z_max - z_min)
        return FloorBoundaryResult(
            cut_planes=np.array([]),
            peak_z=np.array([]),
            histogram=np.array([]),
            bin_centers=np.array([]),
            smoothed=np.array([]),
            n_horizontal_pts=n_horiz,
        )
    n_bins = int(np.ceil((z_max - z_min) / bin_size))
    histogram, edges = np.histogram(z, bins=n_bins, range=(z_min, z_max))
    centers = (edges[:-1] + edges[1:]) / 2

    smoothed = gaussian_filter1d(histogram.astype(np.float64), sigma=smooth_sigma_bins)

    min_distance_bins = max(int(min_floor_height / bin_size), 1)
    smax = smoothed.max()

    peaks, peak_props = find_peaks(
        smoothed,
        height=smax * peak_height_frac,
        distance=min_distance_bins,
        prominence=smax * peak_prominence_frac,
    )

    logger.info(
        "Z-histogram: {} bins, {} peaks detected (min_dist={} bins)",
        n_bins,
        len(peaks),
        min_distance_bins,
    )

    if len(peaks) < 2:
        logger.warning("Fewer than 2 peaks detected — cannot determine boundaries")
        return FloorBoundaryResult(
            cut_planes=np.array([]),
            peak_z=centers[peaks] if len(peaks) else np.array([]),
            histogram=histogram,
            bin_centers=centers,
            smoothed=smoothed,
            n_horizontal_pts=n_horiz,
        )

    # -- find all valleys between consecutive peaks -----------------------
    valleys: list[tuple[float, float, int]] = []
    for i in range(len(peaks) - 1):
        segment = smoothed[peaks[i] : peaks[i + 1] + 1]
        valley_idx = peaks[i] + int(np.argmin(segment))
        valley_z = float(centers[valley_idx])
        valley_depth = float(
            smoothed[peaks[i]] + smoothed[peaks[i + 1]] - 2 * smoothed[valley_idx]
        )
        valleys.append((valley_z, valley_depth, valley_idx))

    valleys.sort(key=lambda v: v[0])

    logger.info(
        "All valleys (z, depth): {}",
        [(f"{v[0]:.2f}", f"{v[1]:.0f}") for v in sorted(valleys, key=lambda v: -v[1])],
    )

    # -- select which valleys to keep -------------------------------------
    auto_n_floors: int | None = None

    if num_floors is not None:
        target_cuts = num_floors - 1
        if len(valleys) > target_cuts:
            valleys_by_depth = sorted(valleys, key=lambda v: -v[1])
            valleys = sorted(valleys_by_depth[:target_cuts], key=lambda v: v[0])
            logger.info(
                "Trimmed to {} deepest valleys (target: {} floors)",
                len(valleys),
                num_floors,
            )
        elif len(valleys) < target_cuts:
            logger.warning(
                "Found only {} valleys but expected {} (for {} floors). "
                "Consider relaxing parameters or adjusting num_floors.",
                len(valleys),
                target_cuts,
                num_floors,
            )
    else:
        logger.info("Auto-detecting floor count ...")
        valleys = _select_valleys_auto(
            valleys,
            z_min=z_min,
            z_max=z_max,
            min_floor_height=min_floor_height,
            max_floor_height=max_floor_height,
        )
        auto_n_floors = len(valleys) + 1
        logger.info("Auto-detected {} floors ({} cut planes)", auto_n_floors, len(valleys))

    cut_planes = np.array([v[0] for v in valleys])
    peak_z_vals = centers[peaks]

    logger.info("Cut planes: {}", [f"{c:.2f}" for c in cut_planes])
    logger.info("Peak Z-values: {}", [f"{p:.2f}" for p in peak_z_vals])

    # -- log segment heights for review -----------------------------------
    all_edges = np.concatenate([[z_min], cut_planes, [z_max]])
    for si in range(len(all_edges) - 1):
        logger.info(
            "  Segment {}: [{:.2f}, {:.2f}] = {:.1f} m",
            si,
            all_edges[si],
            all_edges[si + 1],
            all_edges[si + 1] - all_edges[si],
        )

    return FloorBoundaryResult(
        cut_planes=cut_planes,
        peak_z=peak_z_vals,
        histogram=histogram,
        bin_centers=centers,
        smoothed=smoothed,
        n_horizontal_pts=n_horiz,
        auto_detected_n_floors=auto_n_floors,
        diagnostics={
            "valleys": valleys,
            "peak_indices": peaks.tolist(),
            "peak_prominences": peak_props.get("prominences", np.array([])).tolist(),
        },
    )
