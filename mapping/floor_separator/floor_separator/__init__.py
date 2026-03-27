"""Robust multi-floor point cloud separator.

Detects and segments floors from a single unified point cloud scan
(e.g. Share S20 multi-story building) using Z-histogram analysis with
stairwell masking, normal filtering, and connected-component cleanup.
"""

from __future__ import annotations

from .pipeline import FloorSeparatorConfig, FloorSeparatorResult, run_pipeline

__all__ = ["FloorSeparatorConfig", "FloorSeparatorResult", "run_pipeline"]
