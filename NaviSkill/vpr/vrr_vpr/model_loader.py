"""Load VPR global descriptor models.

Supports MegaLoc (outdoor-optimized, geometry-focused) and SALAD-indoor
(indoor fine-tuned, appearance-sensitive). Models are downloaded from
torch.hub on first use and cached locally.
"""

from __future__ import annotations

import os

import torch
from loguru import logger


def load_megaloc(device: str = "cuda") -> torch.nn.Module:
    """Load the MegaLoc model (DINOv2 backbone, 8448-d descriptors)."""
    try:
        model = torch.hub.load("gmberton/MegaLoc", "get_trained_model")
    except Exception:
        cache_path = os.path.expanduser("~/.cache/torch/hub/gmberton_MegaLoc_main")
        if os.path.exists(cache_path):
            model = torch.hub.load(cache_path, "get_trained_model", source="local")
        else:
            raise RuntimeError(
                "Cannot download MegaLoc model and no local cache found. "
                "Check internet connection or manually download to ~/.cache/torch/hub/"
            )
    logger.debug("Loaded MegaLoc (DINOv2, 8448-d)")
    return model.eval().to(device)


def load_salad_indoor(device: str = "cuda") -> torch.nn.Module:
    """Load the SALAD-indoor model (DINOv2 backbone, 8448-d descriptors).

    Fine-tuned on indoor datasets. Retains appearance sensitivity for
    cross-floor disambiguation. Handles DINOv2 14px patch alignment internally.
    """
    try:
        model = torch.hub.load(
            "Enrico-Chiavassa/Indoor-VPR", "get_trained_model",
            method="salad", backbone="Dinov2", fc_output_dim=8448,
        )
    except Exception:
        cache_path = os.path.expanduser(
            "~/.cache/torch/hub/Enrico-Chiavassa_Indoor-VPR_main"
        )
        if os.path.exists(cache_path):
            model = torch.hub.load(
                cache_path, "get_trained_model", source="local",
                method="salad", backbone="Dinov2", fc_output_dim=8448,
            )
        else:
            raise RuntimeError(
                "Cannot download SALAD-indoor model and no local cache found. "
                "Check internet connection or manually download to ~/.cache/torch/hub/"
            )
    logger.debug("Loaded SALAD-indoor (DINOv2, 8448-d)")
    return model.eval().to(device)


MODEL_CONFIGS = {
    "megaloc": {
        "backbone": "DINOv2",
        "descriptor_dim": 8448,
        "loader": load_megaloc,
    },
    "salad-indoor": {
        "backbone": "DINOv2",
        "descriptor_dim": 8448,
        "loader": load_salad_indoor,
    },
}


def get_model(method: str = "megaloc", device: str = "cuda") -> tuple[torch.nn.Module, int]:
    """Load a VPR model by name.

    Args:
        method: Model name. Supported: "megaloc", "salad-indoor".
        device: "cuda" or "cpu".

    Returns:
        Tuple of (model in eval mode, descriptor dimensionality).
    """
    if method not in MODEL_CONFIGS:
        raise ValueError(f"Unknown method '{method}'. Supported: {list(MODEL_CONFIGS.keys())}")
    config = MODEL_CONFIGS[method]
    model = config["loader"](device=device)
    return model, config["descriptor_dim"]
