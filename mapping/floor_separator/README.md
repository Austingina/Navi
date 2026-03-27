# floor_separator

Automatic multi-floor point cloud segmentation. Splits a unified LiDAR scan of a multi-story building into per-floor PLY files.

## Installation

```bash
cd rerun_colorized_lidar_demo
uv sync
```

## Quick Start

### CLI

```bash
# Auto-detect floors, export per-floor PLY files
uv run floor-separator building.ply -o ./output

# Specify expected floor count and height
uv run floor-separator scan.pcd --num-floors 5 --expected-floor-height 3.5 -o ./output
```

### Python API

```python
from pathlib import Path
import numpy as np
from floor_separator import FloorSeparatorConfig, run_pipeline

positions = np.load("positions.npy")  # (N, 3) float, Z-up
colors = np.load("colors.npy")        # (N, 3) uint8, or None

result = run_pipeline(
    positions, colors,
    FloorSeparatorConfig(output_dir=Path("./output")),
)
print(f"{result.n_floors} floors detected")
print(f"Cut planes: {result.cut_planes}")
```

## How It Works

1. **Phase 0 — Preprocessing:** Voxel downsample (GPU-accelerated when CUDA available) + normal estimation
2. **Phase 1 — Stairwell masking:** XY grid column analysis detects stairwells/elevators and excludes them
3. **Phase 2 — Floor boundary detection:** Normal-filtered Z-histogram with peak/valley analysis
4. **Phase 3 — Per-floor segmentation:** Z-slice + parallel 6-connected CCL cleanup
5. **Phase 4 — Stairwell reassignment:** Re-assign masked stairwell points by Z-range + parallel PLY export

See [floor_seperator_design.md](floor_seperator_design.md) for the full algorithm design document.

## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_floors` | auto | Expected floor count (`None` = auto-detect) |
| `expected_floor_height` | 3.0 | Floor-to-floor height in metres |
| `voxel_size` | 0.10 | Analysis voxel size (metres) |
| `skip_stairwell_mask` | False | Disable stairwell detection |
| `enable_ccl` | True | Enable connected-component cleanup |

## Testing

```bash
uv run pytest floor_separator/tests/ -v
```

## Dependencies

- `open3d` — voxel downsample, normal estimation, PLY export
- `scipy` — connected-component labeling, histogram smoothing, peak detection
- `numpy` — numerical operations
- `loguru` — structured logging
