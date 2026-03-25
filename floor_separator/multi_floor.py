#!/usr/bin/env python3
"""
Multi-floor colorized LiDAR visualization with Rerun.

Reads a Potree v2 octree in parallel, runs the robust floor separation
pipeline (stairwell masking, normal-filtered histogram, CCL cleanup),
and logs each floor as a separate toggleable entity in the Rerun viewer.
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from multiprocessing import cpu_count
from pathlib import Path

import numpy as np
import rerun as rr

from floor_separator import FloorSeparatorConfig, run_pipeline

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_PROJECT_ROOT))
from main import (  # noqa: E402
    DEFAULT_DATA_DIR,
    build_point_dtype,
    decode_node,
    load_metadata,
    parse_hierarchy,
)

FLOOR_COLORS = [
    [31, 119, 180],
    [255, 127, 14],
    [44, 160, 44],
    [214, 39, 40],
    [148, 103, 189],
    [140, 86, 75],
    [227, 119, 194],
    [127, 127, 127],
]


def _log_debug_info(result: "FloorSeparatorResult") -> None:  # noqa: F821
    """Log intermediate pipeline results to Rerun for inspection."""
    from floor_separator.pipeline import FloorSeparatorResult

    if not isinstance(result, FloorSeparatorResult):
        return

    # Stairwell overlay on analysis cloud
    if result.stairwell_result is not None and result.preprocessed is not None:
        sw = result.stairwell_result
        pp = result.preprocessed
        if sw.stairwell_mask.any():
            rr.log(
                "world/debug/stairwell_points",
                rr.Points3D(
                    positions=pp.positions[sw.stairwell_mask],
                    colors=np.full((sw.stairwell_mask.sum(), 3), [255, 0, 0], dtype=np.uint8),
                    radii=rr.Radius.ui_points(2.0),
                ),
            )
        if sw.clean_mask.any():
            rr.log(
                "world/debug/clean_points",
                rr.Points3D(
                    positions=pp.positions[sw.clean_mask],
                    colors=np.full((sw.clean_mask.sum(), 3), [0, 200, 0], dtype=np.uint8),
                    radii=rr.Radius.ui_points(1.0),
                ),
            )

    # Z-histogram as a bar chart
    if result.boundary_result is not None:
        br = result.boundary_result
        rr.log(
            "plots/z_histogram",
            rr.BarChart(br.smoothed),
        )

    # Noise points from CCL
    if result.noise_mask is not None and result.noise_mask.any():
        pass  # noise points are already excluded from floor entities


def main() -> None:
    ncpu = cpu_count()

    parser = argparse.ArgumentParser(
        description="Multi-floor colorized LiDAR visualization with Rerun"
    )
    # Data loading
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR)
    parser.add_argument("--workers", type=int, default=ncpu)
    parser.add_argument(
        "--subsample", type=int, default=4,
        help="Keep every Nth point (default: 4 ~ 50M pts)",
    )

    # Rerun output
    parser.add_argument("--save", type=Path, default=None)
    parser.add_argument(
        "--serve", action="store_true",
        help="Web viewer for SSH / headless",
    )
    parser.add_argument("--port", type=int, default=9090)

    # Floor separator — manual override
    parser.add_argument(
        "--floors", type=str, default=None,
        help="Comma-separated Z boundaries (e.g. -18,-6.7,-2.8,0.9) — "
             "bypasses automatic detection",
    )

    # Floor separator — pipeline parameters
    parser.add_argument(
        "--num-floors", type=int, default=None,
        help="Expected floor count (default: auto-detect)",
    )
    parser.add_argument("--expected-floor-height", type=float, default=3.0)
    parser.add_argument("--max-floor-height", type=float, default=5.0,
                        help="Max segment height before forced split (auto mode)")
    parser.add_argument("--voxel-size", type=float, default=0.1)
    parser.add_argument("--normal-filter-angle", type=float, default=15.0)
    parser.add_argument("--stairwell-grid", type=float, default=0.5)
    parser.add_argument("--min-stairwell-area", type=float, default=4.0)
    parser.add_argument(
        "--skip-stairwell-mask", action="store_true",
        help="Disable stairwell detection",
    )
    parser.add_argument(
        "--no-ccl", action="store_true",
        help="Disable connected-component cleanup",
    )
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help="Export per-floor PLY files to this directory",
    )
    parser.add_argument(
        "--debug", action="store_true",
        help="Log intermediate results to Rerun",
    )

    args = parser.parse_args()

    print(f"Machine : {ncpu} CPUs | workers: {args.workers} | subsample: 1/{args.subsample}")

    # ---- metadata --------------------------------------------------------
    meta = load_metadata(args.data_dir)
    scale = np.array(meta["scale"], dtype=np.float32)
    offset = np.array(meta["offset"], dtype=np.float32)
    point_dtype = build_point_dtype(meta["attributes"])
    print(f"Dataset : {meta['points']:,} points | record: {point_dtype.itemsize} B")

    # ---- hierarchy -------------------------------------------------------
    t0 = time.perf_counter()
    nodes = parse_hierarchy(args.data_dir, meta["hierarchy"]["firstChunkSize"])
    print(f"Hierarchy: {len(nodes)} nodes ({time.perf_counter()-t0:.2f}s)")

    # ---- parallel decode -------------------------------------------------
    mmap_data = np.memmap(args.data_dir / "octree.bin", dtype=np.uint8, mode="r")
    nodes.sort(key=lambda n: n["byteOffset"])

    print(f"Decoding with {args.workers} threads ...")
    t0 = time.perf_counter()
    all_pos: list[np.ndarray] = []
    all_rgb: list[np.ndarray] = []

    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {
            pool.submit(
                decode_node, mmap_data,
                n["byteOffset"], n["byteSize"], n["numPoints"],
                point_dtype, scale, offset, args.subsample,
            ): n["name"]
            for n in nodes
        }
        done = 0
        for fut in as_completed(futures):
            pos, rgb = fut.result()
            all_pos.append(pos)
            all_rgb.append(rgb)
            done += 1
            if done % 500 == 0 or done == len(nodes):
                print(f"  decoded {done}/{len(nodes)} nodes")

    print(f"Decode: {time.perf_counter()-t0:.1f}s")

    t0 = time.perf_counter()
    positions = np.concatenate(all_pos)
    colors = np.concatenate(all_rgb)
    del all_pos, all_rgb
    num_pts = len(positions)
    print(f"Total  : {num_pts:,} points ({time.perf_counter()-t0:.1f}s)")

    # ---- floor detection -------------------------------------------------
    if args.floors:
        # Manual override: skip the pipeline entirely
        boundaries = np.array([float(x) for x in args.floors.split(",")])
        print(f"Using manual floor boundaries: {boundaries}")
        floor_ids = np.digitize(positions[:, 2], boundaries)
        n_floors = len(boundaries) + 1
        z = positions[:, 2]
        floor_ranges = np.concatenate([[z.min()], boundaries, [z.max()]])
        noise_mask = np.zeros(num_pts, dtype=bool)
        result = None
    else:
        # Run the full pipeline
        mode = f"target: {args.num_floors} floors" if args.num_floors else "auto-detect"
        print(f"\nRunning floor separation pipeline ({mode}) ...")
        config = FloorSeparatorConfig(
            voxel_size=args.voxel_size,
            num_floors=args.num_floors,
            expected_floor_height=args.expected_floor_height,
            max_floor_height=args.max_floor_height,
            normal_filter_angle_deg=args.normal_filter_angle,
            stairwell_grid_cell=args.stairwell_grid,
            stairwell_min_area=args.min_stairwell_area,
            skip_stairwell_mask=args.skip_stairwell_mask,
            enable_ccl=not args.no_ccl,
            output_dir=args.output_dir,
        )

        result = run_pipeline(positions, colors, config)

        floor_ids = result.floor_labels
        n_floors = result.n_floors
        boundaries = result.cut_planes
        floor_ranges = np.concatenate([
            [result.floor_z_ranges[0, 0]],
            boundaries,
            [result.floor_z_ranges[-1, 1]],
        ])
        noise_mask = result.noise_mask

    # ---- summary table ---------------------------------------------------
    print(f"\n{'Floor':<10} {'Z range':<25} {'Points':>12} {'%':>6}")
    print("-" * 55)
    for i in range(n_floors):
        mask = (floor_ids == i) & (~noise_mask)
        lo, hi = floor_ranges[i], floor_ranges[i + 1]
        cnt = mask.sum()
        print(f"Floor {i+1:<4} [{lo:>7.2f}, {hi:>7.2f}) m   {cnt:>12,} {cnt/num_pts*100:>5.1f}%")

    # ---- Rerun init ------------------------------------------------------
    rr.init("multi_floor_lidar")
    if args.save:
        rr.save(str(args.save))
        print(f"\nRecording -> {args.save}")
    elif args.serve:
        grpc_addr = rr.serve_grpc(server_memory_limit="8GiB")
        rr.serve_web_viewer(
            open_browser=False, web_port=args.port, connect_to=grpc_addr
        )
        print(f"\nRerun web viewer: http://localhost:{args.port}")
        print(f"  Full URL: http://localhost:{args.port}/?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9876%2Fproxy")
        print(f"  Forward ports {args.port} + 9876 in Cursor.")
    else:
        rr.spawn()

    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # ---- log debug info --------------------------------------------------
    if args.debug and result is not None:
        _log_debug_info(result)

    # ---- log each floor --------------------------------------------------
    t0 = time.perf_counter()
    for i in range(n_floors):
        mask = (floor_ids == i) & (~noise_mask)
        cnt = mask.sum()
        if cnt == 0:
            continue

        lo = floor_ranges[i]
        hi = floor_ranges[i + 1]
        entity = f"world/floors/floor_{i+1:02d}_z[{lo:.1f}_{hi:.1f}]"

        rr.log(
            entity,
            rr.Points3D(
                positions=positions[mask],
                colors=colors[mask],
                radii=rr.Radius.ui_points(1.0),
            ),
        )
        print(f"  logged {entity}  ({cnt:,} pts)")

    print(f"Logged {n_floors} floors in {time.perf_counter()-t0:.1f}s  --  done!")

    if args.serve:
        print("\nPress Ctrl+C to stop.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
