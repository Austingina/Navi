"""Tests for input validation in the pipeline entry point."""

from __future__ import annotations

import numpy as np
import pytest

from floor_separator.pipeline import _validate_inputs


class TestValidateInputs:
    """Input validation should reject bad data with clear errors."""

    def test_wrong_shape_1d(self) -> None:
        with pytest.raises(ValueError, match="must be .N, 3."):
            _validate_inputs(np.zeros(10), None)

    def test_wrong_shape_2d_wrong_cols(self) -> None:
        with pytest.raises(ValueError, match="must be .N, 3."):
            _validate_inputs(np.zeros((10, 4)), None)

    def test_empty_positions(self) -> None:
        with pytest.raises(ValueError, match="empty"):
            _validate_inputs(np.zeros((0, 3)), None)

    def test_nan_positions(self) -> None:
        pts = np.array([[1.0, 2.0, 3.0], [np.nan, 0.0, 0.0]])
        with pytest.raises(ValueError, match="NaN"):
            _validate_inputs(pts, None)

    def test_colors_length_mismatch(self) -> None:
        pts = np.random.rand(10, 3)
        colors = np.random.randint(0, 255, (5, 3), dtype=np.uint8)
        with pytest.raises(ValueError, match="colors length"):
            _validate_inputs(pts, colors)

    def test_colors_wrong_shape(self) -> None:
        pts = np.random.rand(10, 3)
        colors = np.random.randint(0, 255, (10, 4), dtype=np.uint8)
        with pytest.raises(ValueError, match="must be .N, 3."):
            _validate_inputs(pts, colors)

    def test_not_numpy(self) -> None:
        with pytest.raises(ValueError, match="numpy array"):
            _validate_inputs([[1, 2, 3]], None)

    def test_valid_positions_only(self) -> None:
        _validate_inputs(np.random.rand(100, 3), None)

    def test_valid_with_colors(self) -> None:
        pts = np.random.rand(100, 3)
        colors = np.random.randint(0, 255, (100, 3), dtype=np.uint8)
        _validate_inputs(pts, colors)
