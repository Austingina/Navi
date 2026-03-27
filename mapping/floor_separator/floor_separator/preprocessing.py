"""Phase 0: Voxel downsampling and normal estimation.

Converts raw numpy arrays into an Open3D point cloud, downsamples for
analysis, and estimates surface normals needed by later phases.

Uses GPU (Open3D Tensor API + CUDA) when available, falls back to CPU.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
import open3d as o3d
import open3d.core as o3c
from loguru import logger


@dataclass
class PreprocessedCloud:
    """Result of preprocessing: downsampled cloud with normals."""

    positions: np.ndarray  # (M, 3) float64 — analysis-resolution points
    normals: np.ndarray  # (M, 3) float64 — estimated surface normals
    colors: np.ndarray | None  # (M, 3) float64 [0-1] or None
    downsample_indices: np.ndarray | None  # mapping back to original if available


def _cuda_available() -> bool:
    try:
        return o3c.cuda.is_available() and o3c.cuda.device_count() > 0
    except Exception:
        return False


def _preprocess_gpu(
    positions: np.ndarray,
    colors: np.ndarray | None,
    voxel_size: float,
    normal_knn: int,
) -> PreprocessedCloud:
    """GPU-accelerated voxel downsample + CPU normal estimation.

    GPU is used for the voxel downsample step (massive speedup on large
    clouds).  Normal estimation stays on CPU because Open3D's GPU KNN is
    slower than the CPU KDTree implementation for typical cloud sizes.
    """
    device = o3c.Device("CUDA:0")

    t0 = time.perf_counter()
    pcd_t = o3d.t.geometry.PointCloud(device)
    pcd_t.point.positions = o3c.Tensor(
        positions.astype(np.float32), dtype=o3c.float32, device=device
    )
    if colors is not None:
        c = colors.astype(np.float32)
        if c.max() > 1.0:
            c = c / 255.0
        pcd_t.point.colors = o3c.Tensor(c, dtype=o3c.float32, device=device)
    logger.info("  GPU upload: {:.2f}s", time.perf_counter() - t0)

    t0 = time.perf_counter()
    pcd_down_t = pcd_t.voxel_down_sample(voxel_size)
    n_down = len(pcd_down_t.point.positions)
    logger.info(
        "  GPU voxel downsample: {} -> {} points ({:.1f}x) in {:.2f}s",
        len(positions), n_down, len(positions) / max(n_down, 1),
        time.perf_counter() - t0,
    )

    t0 = time.perf_counter()
    out_pos = pcd_down_t.point.positions.cpu().numpy().astype(np.float64)
    out_colors = None
    try:
        out_colors = pcd_down_t.point.colors.cpu().numpy().astype(np.float64)
    except (KeyError, RuntimeError):
        pass
    logger.info("  GPU download: {:.2f}s", time.perf_counter() - t0)

    t0 = time.perf_counter()
    pcd_legacy = o3d.geometry.PointCloud()
    pcd_legacy.points = o3d.utility.Vector3dVector(out_pos)
    pcd_legacy.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=normal_knn)
    )
    out_normals = np.asarray(pcd_legacy.normals)
    logger.info("  CPU normal estimation: {:.2f}s", time.perf_counter() - t0)

    return PreprocessedCloud(
        positions=out_pos,
        normals=out_normals,
        colors=out_colors,
        downsample_indices=None,
    )


def _preprocess_cpu(
    positions: np.ndarray,
    colors: np.ndarray | None,
    voxel_size: float,
    normal_knn: int,
) -> PreprocessedCloud:
    """CPU preprocessing via Open3D legacy API."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(positions.astype(np.float64))
    if colors is not None:
        c = colors.astype(np.float64)
        if c.max() > 1.0:
            c = c / 255.0
        pcd.colors = o3d.utility.Vector3dVector(c)

    t0 = time.perf_counter()
    pcd_down = pcd.voxel_down_sample(voxel_size)
    n_down = len(pcd_down.points)
    logger.info(
        "  CPU voxel downsample: {} -> {} points ({:.1f}x) in {:.2f}s",
        len(positions), n_down, len(positions) / max(n_down, 1),
        time.perf_counter() - t0,
    )

    t0 = time.perf_counter()
    pcd_down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=normal_knn)
    )
    logger.info("  CPU normal estimation: {:.2f}s", time.perf_counter() - t0)

    out_pos = np.asarray(pcd_down.points)
    out_normals = np.asarray(pcd_down.normals)
    out_colors = (
        np.asarray(pcd_down.colors) if pcd_down.has_colors() else None
    )

    return PreprocessedCloud(
        positions=out_pos,
        normals=out_normals,
        colors=out_colors,
        downsample_indices=None,
    )


def preprocess(
    positions: np.ndarray,
    colors: np.ndarray | None = None,
    voxel_size: float = 0.05,
    normal_knn: int = 30,
) -> PreprocessedCloud:
    """Downsample the point cloud and estimate normals.

    Uses GPU (CUDA) when available for ~3-5x speedup on large clouds.

    Args:
        positions: (N, 3) raw point positions.
        colors: (N, 3) uint8 RGB or None.
        voxel_size: Leaf size for voxel-grid downsampling (metres).
        normal_knn: Number of neighbours for normal estimation.

    Returns:
        PreprocessedCloud with downsampled positions, normals, and colours.
    """
    if voxel_size <= 0:
        raise ValueError(f"voxel_size must be positive, got {voxel_size}")
    if normal_knn < 1:
        raise ValueError(f"normal_knn must be >= 1, got {normal_knn}")
    if len(positions) == 0:
        raise ValueError("Cannot preprocess an empty point cloud")

    use_gpu = _cuda_available()
    backend = "GPU (CUDA)" if use_gpu else "CPU"
    logger.info(
        "Preprocessing: {} points, voxel_size={:.3f} m, backend={}",
        len(positions), voxel_size, backend,
    )

    if use_gpu:
        try:
            result = _preprocess_gpu(positions, colors, voxel_size, normal_knn)
        except Exception as e:
            logger.warning("GPU preprocessing failed ({}), falling back to CPU", e)
            result = _preprocess_cpu(positions, colors, voxel_size, normal_knn)
    else:
        result = _preprocess_cpu(positions, colors, voxel_size, normal_knn)

    logger.info("Preprocessing complete: {} analysis points", len(result.positions))
    return result
