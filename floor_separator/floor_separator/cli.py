#!/usr/bin/env python3
"""Standalone CLI for floor separation.

Reads a point cloud file (PLY/PCD), runs the floor separation pipeline,
and exports per-floor PLY files.  No Rerun or Potree dependency.

Usage:
    uv run python floor_separator/cli.py building.ply -o ./output
    uv run python floor_separator/cli.py scan.pcd --num-floors 5 --expected-floor-height 3.5
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import open3d as o3d
from loguru import logger

from floor_separator import FloorSeparatorConfig, run_pipeline


def load_point_cloud(path: Path) -> tuple[np.ndarray, np.ndarray | None]:
    """Load a point cloud file into numpy arrays.

    Args:
        path: Path to PLY, PCD, or other Open3D-supported format.

    Returns:
        (positions (N,3) float64, colors (N,3) uint8 or None)
    """
    logger.info("Loading point cloud: {}", path)
    pcd = o3d.io.read_point_cloud(str(path))
    if len(pcd.points) == 0:
        logger.error("Empty point cloud: {}", path)
        sys.exit(1)

    positions = np.asarray(pcd.points)
    colors = None
    if pcd.has_colors():
        colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)

    logger.info("Loaded {} points", len(positions))
    return positions, colors


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Floor separation CLI — split a multi-floor point cloud into per-floor PLY files"
    )
    parser.add_argument(
        "input", type=Path,
        help="Input point cloud file (PLY, PCD, or any Open3D-supported format)",
    )
    parser.add_argument(
        "-o", "--output-dir", type=Path, default=None,
        help="Output directory for per-floor PLY files (default: <input_dir>/floor_output)",
    )

    parser.add_argument(
        "--output-format", type=str, default="ply", choices=["ply", "pcd"],
        help="Output point cloud file format for exported floors (ply or pcd)",
    )

    grp = parser.add_argument_group("floor detection parameters")
    grp.add_argument("--num-floors", type=int, default=None, help="Expected floor count (default: auto-detect)")
    grp.add_argument("--expected-floor-height", type=float, default=3.0, help="Expected floor-to-floor height in metres")
    grp.add_argument("--max-floor-height", type=float, default=5.0, help="Max segment height before forced split (auto mode)")
    grp.add_argument("--voxel-size", type=float, default=0.10, help="Analysis voxel size in metres")
    grp.add_argument("--normal-filter-angle", type=float, default=15.0, help="Normal pre-filter angle threshold (degrees)")

    grp2 = parser.add_argument_group("optional toggles")
    grp2.add_argument("--skip-stairwell-mask", action="store_true", help="Disable stairwell detection")
    grp2.add_argument("--no-ccl", action="store_true", help="Disable connected-component cleanup")

    args = parser.parse_args()

    if args.output_dir is None:
        args.output_dir = args.input.parent / "floor_output"

    positions, colors = load_point_cloud(args.input)

    config = FloorSeparatorConfig(
        num_floors=args.num_floors,
        expected_floor_height=args.expected_floor_height,
        max_floor_height=args.max_floor_height,
        voxel_size=args.voxel_size,
        normal_filter_angle_deg=args.normal_filter_angle,
        skip_stairwell_mask=args.skip_stairwell_mask,
        enable_ccl=not args.no_ccl,
        output_dir=args.output_dir,
    output_format=args.output_format,
    )

    t0 = time.perf_counter()
    result = run_pipeline(positions, colors, config)
    elapsed = time.perf_counter() - t0

    print(f"\n{'='*60}")
    print(f"Result: {result.n_floors} floors detected in {elapsed:.1f}s")
    print(f"Cut planes: {[f'{z:.2f}' for z in result.cut_planes]}")
    print(f"{'='*60}")
    print(f"\n{'Floor':<8} {'Z range':<25} {'Points':>12} {'%':>6}")
    print("-" * 55)

    total = len(positions)
    for i in range(result.n_floors):
        mask = (result.floor_labels == i) & (~result.noise_mask)
        lo, hi = result.floor_z_ranges[i]
        cnt = mask.sum()
        print(f"Floor {i:<3} [{lo:>7.2f}, {hi:>7.2f}) m   {cnt:>12,} {cnt/total*100:>5.1f}%")

    noise_total = result.noise_mask.sum()
    if noise_total > 0:
        print(f"{'Noise':<8} {'':25} {noise_total:>12,} {noise_total/total*100:>5.1f}%")

    if result.exported_files:
        print(f"\nExported {len(result.exported_files)} files to {args.output_dir}/")
        for f in result.exported_files:
            print(f"  {f.name}")


if __name__ == "__main__":
    main()
