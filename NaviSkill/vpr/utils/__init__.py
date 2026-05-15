"""VPR utility package — shared helpers for the live + eval pipelines.

This package collects all library-style modules used by both the
production pipeline (``vpr_preprocess.py`` / ``vpr_detect.py``) and the
offline evaluation pipeline (``eval/pipeline_3d.py``):

* :mod:`utils.utils_common`  -- platform-agnostic extraction helpers
                                (distance sampler, JPEG savers, pose
                                loaders, progress/summary).
* :mod:`utils.utils_s20`     -- SHARE SLAM S20 (ROS1 bag + ImgPose.txt).
* :mod:`utils.utils_r1pro`   -- R1Pro (ROS2 bag + /Odometry).
* :mod:`utils.undistort`     -- PolyFisheye -> perspective undistortion.
* :mod:`utils.undistort2`    -- Pinhole (rational) undistortion.
* :mod:`utils.rerank`        -- SuperPoint + LightGlue geometric re-rank
                                (eval-only).
* :mod:`utils.color_rerank`  -- HSV color histogram re-scoring (eval-only).

Top-level entry scripts (``vpr_preprocess.py`` / ``vpr_detect.py`` and
``eval/*``) import these via the absolute path ``from utils.X import Y``;
modules **inside** this package use intra-package relative imports
(``from .utils_common import …``).
"""
