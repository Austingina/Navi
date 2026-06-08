"""Load VPR global descriptor models.

Supports MegaLoc (outdoor-optimized, geometry-focused) and SALAD-indoor
(indoor fine-tuned, appearance-sensitive). Models are downloaded from
torch.hub on first use and cached locally.
"""

from __future__ import annotations

import contextlib
import os
import sys
from urllib.error import HTTPError, URLError

import torch
import torch.hub as _torch_hub
from loguru import logger


def _install_offline_friendly_hub_patch() -> None:
    """让 ``torch.hub`` 在 github 探测分支失败时回退到本地缓存。

    背景：``torch.hub.load("owner/repo", ...)`` 在未显式指定分支时，会先访问
    ``https://github.com/owner/repo/tree/main/`` 来判断默认分支是 ``main`` 还是
    ``master``。torch 自带的逻辑只对 ``URLError``（纯离线、连接被拒等 socket 级
    失败）做缓存兜底；而当网络路径上有网关/代理返回 ``HTTP 504`` 这类 HTTP 错误
    响应时（``HTTPError`` 且 ``code != 404``），它会直接抛出，导致即使本地已有缓存
    也无法离线加载。

    本补丁包住 ``_parse_repo_info``：仅在探测分支抛网络异常时介入，尝试用已缓存
    的分支名兜底；若既连不上又无缓存，则原样抛出，语义与官方离线逻辑一致。
    网络正常时补丁不触发，首次无缓存仍可正常联网下载。
    """
    if getattr(_torch_hub._parse_repo_info, "_offline_friendly", False):
        return

    _orig_parse_repo_info = _torch_hub._parse_repo_info

    def _parse_repo_info_offline_friendly(github):
        try:
            return _orig_parse_repo_info(github)
        except (HTTPError, URLError):
            repo_info = github.split(":")[0]
            repo_owner, repo_name = repo_info.split("/")
            for ref in ("main", "master"):
                cached = os.path.join(
                    _torch_hub.get_dir(), f"{repo_owner}_{repo_name}_{ref}"
                )
                if os.path.exists(cached):
                    logger.warning(
                        "torch.hub 无法访问 github 探测分支，回退到本地缓存: {}",
                        cached,
                    )
                    return repo_owner, repo_name, ref
            raise

    _parse_repo_info_offline_friendly._offline_friendly = True
    _torch_hub._parse_repo_info = _parse_repo_info_offline_friendly


_install_offline_friendly_hub_patch()


@contextlib.contextmanager
def _isolated_utils_namespace():
    """临时把 ``sys.modules['utils']`` 隔离开。

    背景：``vpr_detect.py`` / ``vpr_preprocess.py`` 顶层执行
    ``from utils.undistort import PolyFisheyeUndistorter``，会把本仓库的
    ``NaviSkill/vpr/utils/`` 包注册成 ``sys.modules['utils']``。

    然而 ``serizba/salad`` 上游仓库（salad-indoor 内部依赖）的
    ``vpr_model.py`` 写了 ``import utils`` ，本应解析到 salad 缓存目录里的
    ``utils.py``。但 Python 的导入系统会优先用 ``sys.modules`` 里已存在的同名
    模块 → 拿到我们的 ``utils`` 包 → 调用 ``utils.get_loss(...)`` 直接
    ``AttributeError``。

    本上下文管理器在进入时把所有以 ``utils`` 开头的模块条目暂存并从
    ``sys.modules`` 中剔除，退出时还原；从而让 salad 的 ``import utils`` 能
    找到 salad 自带的 ``utils.py``，而本仓库后续仍可用自己的 ``utils`` 包。
    """
    saved = {
        k: sys.modules[k]
        for k in list(sys.modules)
        if k == "utils" or k.startswith("utils.")
    }
    for k in saved:
        del sys.modules[k]
    try:
        yield
    finally:
        # 还原本仓库 utils 包；salad 自带的 utils 模块顺带在 finally 里
        # 出栈，避免污染后续代码。
        for k in [
            k for k in list(sys.modules)
            if k == "utils" or k.startswith("utils.")
        ]:
            del sys.modules[k]
        sys.modules.update(saved)


def load_megaloc(device: str = "cuda") -> torch.nn.Module:
    """Load the MegaLoc model (DINOv2 backbone, 8448-d descriptors).

    Trained on outdoor street-view data. Color-invariant by design.
    Best for same-floor VPR with geometric re-ranking.

    Returns:
        Model in eval mode on the specified device.
    """
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

    Fine-tuned on indoor datasets with contrastive learning. Retains
    appearance sensitivity (wall colors, floor materials) that outdoor
    models discard, making it effective for cross-floor disambiguation.
    Handles DINOv2 14px patch alignment internally.

    NOTE: salad-indoor 内部会通过 ``torch.hub.load("serizba/salad", ...)`` 间接
    加载 serizba/salad；salad 的 ``vpr_model.py`` 用 ``import utils`` 访问其
    仓库内的 ``utils.py``，会被本仓库的 ``utils`` 包 (sys.modules['utils'])
    屏蔽。使用 ``_isolated_utils_namespace()`` 在 hub.load 前后做隔离。

    Returns:
        Model in eval mode on the specified device.
    """
    with _isolated_utils_namespace():
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
