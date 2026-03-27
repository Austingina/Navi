"""Integration test: synthetic 3-floor building end-to-end."""

from __future__ import annotations

import numpy as np

from floor_separator import FloorSeparatorConfig, run_pipeline


def _make_synthetic_building(
    n_pts_per_floor: int = 5000,
    floor_heights: tuple[float, ...] = (0.0, 5.0, 10.0),
    xy_extent: float = 30.0,
    ceiling_offset: float = 2.8,
    seed: int = 42,
) -> np.ndarray:
    """Generate a synthetic multi-floor point cloud.

    Each floor has a floor slab, a ceiling slab, and wall points.
    Floor-to-floor height should be >= 3.0 m for reliable detection.

    Args:
        n_pts_per_floor: Points per floor slab.
        floor_heights: Z-height of each floor slab.
        xy_extent: Half-width of the floor in XY.
        ceiling_offset: Height from floor to ceiling within one storey.
        seed: Random seed.

    Returns:
        (N, 3) float64 positions.
    """
    rng = np.random.default_rng(seed)
    clouds: list[np.ndarray] = []

    for z_floor in floor_heights:
        floor_pts = np.column_stack([
            rng.uniform(-xy_extent, xy_extent, n_pts_per_floor),
            rng.uniform(-xy_extent, xy_extent, n_pts_per_floor),
            np.full(n_pts_per_floor, z_floor) + rng.normal(0, 0.02, n_pts_per_floor),
        ])
        clouds.append(floor_pts)

        ceiling_z = z_floor + ceiling_offset
        ceiling_pts = np.column_stack([
            rng.uniform(-xy_extent, xy_extent, n_pts_per_floor // 2),
            rng.uniform(-xy_extent, xy_extent, n_pts_per_floor // 2),
            np.full(n_pts_per_floor // 2, ceiling_z) + rng.normal(0, 0.02, n_pts_per_floor // 2),
        ])
        clouds.append(ceiling_pts)

        n_wall = n_pts_per_floor // 4
        wall_pts = np.column_stack([
            np.full(n_wall, xy_extent) + rng.normal(0, 0.02, n_wall),
            rng.uniform(-xy_extent, xy_extent, n_wall),
            rng.uniform(z_floor, ceiling_z, n_wall),
        ])
        clouds.append(wall_pts)

    return np.vstack(clouds)


class TestThreeFloorPipeline:
    """End-to-end test on a synthetic 3-floor building."""

    def test_detects_three_floors(self) -> None:
        positions = _make_synthetic_building(
            n_pts_per_floor=8000,
            floor_heights=(0.0, 10.0, 20.0),
            ceiling_offset=2.5,
        )
        config = FloorSeparatorConfig(
            num_floors=3,
            expected_floor_height=10.0,
            skip_stairwell_mask=True,
            enable_ccl=False,
            voxel_size=0.5,
        )
        result = run_pipeline(positions, config=config)

        assert result.n_floors == 3
        assert len(result.cut_planes) == 2
        assert result.floor_labels.shape == (len(positions),)
        assert result.noise_mask.shape == (len(positions),)

        for cp in result.cut_planes:
            assert 2.0 < cp < 19.0, f"cut plane {cp} out of expected range"

        labels_set = set(result.floor_labels.tolist())
        assert labels_set == {0, 1, 2}

    def test_auto_detect_finds_three(self) -> None:
        positions = _make_synthetic_building()
        config = FloorSeparatorConfig(
            num_floors=None,
            expected_floor_height=5.0,
            max_floor_height=6.0,
            skip_stairwell_mask=True,
            enable_ccl=False,
            voxel_size=0.5,
        )
        result = run_pipeline(positions, config=config)

        assert 2 <= result.n_floors <= 4, f"expected ~3 floors, got {result.n_floors}"

    def test_output_shapes_consistent(self) -> None:
        positions = _make_synthetic_building()
        config = FloorSeparatorConfig(
            skip_stairwell_mask=True,
            enable_ccl=True,
            voxel_size=0.5,
        )
        result = run_pipeline(positions, config=config)

        n = len(positions)
        assert result.floor_labels.shape == (n,)
        assert result.noise_mask.shape == (n,)
        assert result.floor_z_ranges.shape == (result.n_floors, 2)
        assert len(result.cut_planes) == result.n_floors - 1

        for i in range(result.n_floors):
            assert result.floor_z_ranges[i, 0] < result.floor_z_ranges[i, 1]
