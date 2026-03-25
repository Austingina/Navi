"""Tests for edge cases that must not crash the pipeline."""

from __future__ import annotations

import numpy as np

from floor_separator import FloorSeparatorConfig, run_pipeline


class TestSingleFloor:
    """A point cloud with all points at the same Z should return 1 floor."""

    def test_flat_cloud(self) -> None:
        rng = np.random.default_rng(42)
        n = 500
        positions = np.column_stack([
            rng.uniform(-10, 10, n),
            rng.uniform(-10, 10, n),
            np.full(n, 5.0) + rng.normal(0, 0.01, n),
        ])
        config = FloorSeparatorConfig(enable_ccl=False, skip_stairwell_mask=True)
        result = run_pipeline(positions, config=config)

        assert result.n_floors >= 1
        assert result.floor_labels.shape == (n,)
        assert len(result.cut_planes) == result.n_floors - 1


class TestTwoDistinctFloors:
    """Two well-separated horizontal planes should yield 2 floors."""

    def test_two_planes(self) -> None:
        """Two floor slabs + two ceiling slabs separated by a clear gap."""
        rng = np.random.default_rng(42)
        n = 5000

        clouds = []
        for z in [0.0, 2.8, 5.0, 7.8]:
            clouds.append(np.column_stack([
                rng.uniform(-30, 30, n),
                rng.uniform(-30, 30, n),
                np.full(n, z) + rng.normal(0, 0.02, n),
            ]))
        positions = np.vstack(clouds)

        config = FloorSeparatorConfig(
            enable_ccl=False,
            skip_stairwell_mask=True,
            num_floors=2,
            voxel_size=0.5,
            min_floor_height=1.5,
        )
        result = run_pipeline(positions, config=config)

        assert result.n_floors == 2
        assert len(result.cut_planes) == 1
        assert 3.0 < result.cut_planes[0] < 4.5


class TestSmallCloud:
    """Pipeline should handle small but valid point clouds gracefully."""

    def test_minimum_viable(self) -> None:
        rng = np.random.default_rng(42)
        positions = rng.uniform(-1, 1, (50, 3))
        config = FloorSeparatorConfig(
            enable_ccl=False,
            skip_stairwell_mask=True,
            voxel_size=0.5,
        )
        result = run_pipeline(positions, config=config)

        assert result.n_floors >= 1
        assert result.floor_labels.shape == (50,)
