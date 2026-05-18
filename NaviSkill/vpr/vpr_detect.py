#!/usr/bin/env python3
"""VPR detection: match a query image against one or more VPR databases.

Step 2 of the live VPR pipeline. Given a query image and a list of
pre-built VPR feature databases (each produced by ``vpr_preprocess.py``,
typically one database per submap/floor), this script finds the single
best-matching database image *across all databases* and returns:

  * the matched image's path (the line stored in ``database_image_paths.txt``)
  * its absolute path on disk (for convenience)
  * its ``(x, y, z)`` coordinates
  * the path of the database (submap) it came from
  * the L2 distance in descriptor space

Two ways to use this file:

1. As a CLI (one-shot):

    python vpr_detect.py \\
        --image /path/to/query.jpg \\
        --databases /datasets/2f /datasets/3f /datasets/4f \\
        --top_k 1

2. As a library (recommended for live use; model + databases are loaded
   once, then ``query()`` is called per frame)::

       from vpr_detect import VPRDetector
       det = VPRDetector(["/datasets/2f", "/datasets/3f"])
       result = det.query("/tmp/live_frame.jpg")[0]
       print(result["database"], result["image_rel"],
             result["x"], result["y"], result["z"])
"""

import argparse
import io
import json
import os
import sys
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import faiss
import numpy as np
import torch
import torchvision.transforms as transforms
from loguru import logger
from PIL import Image

from model_loader import get_model
from utils.undistort import PolyFisheyeUndistorter


ImageInput = Union[str, "os.PathLike[str]", Path, Image.Image, np.ndarray, bytes]


# 各 database build_info.json 中影响去畸变的字段；live query 必须保持一致才能
# 让查询描述符落入同一特征空间。
_UNDISTORT_KEYS = (
    "undistort_fisheye",
    "calibration_file",
    "out_width",
    "out_height",
    "fov",
)


# ---------------------------------------------------------------------------
# Database loading
# ---------------------------------------------------------------------------

@dataclass
class DatabaseBundle:
    """One VPR submap database as written by ``vpr_preprocess.py``."""

    path: str
    descriptors: np.ndarray          # (N, D) float32
    poses: np.ndarray                # (N, 4) float64 -- [x, y, z, ts_ns]
    image_paths_rel: List[str]       # exactly the lines in database_image_paths.txt
    image_paths_abs: List[str] = field(default_factory=list)
    method: str = ""
    descriptor_dim: int = 0
    # image_size 来自 database_meta.npz；None == 抽特征时未做 resize
    image_size: Optional[Tuple[int, ...]] = None
    # database_build_info.json 的全部内容（缺失时为 None），用于查询时校验
    # 去畸变配置是否与构造时一致。
    build_info: Optional[Dict[str, Any]] = None

    @property
    def num_images(self) -> int:
        return len(self.image_paths_rel)


def load_database(database_dir: str) -> DatabaseBundle:
    """Load one database directory into a ``DatabaseBundle``."""
    database_dir = os.path.abspath(database_dir)
    if not os.path.isdir(database_dir):
        raise FileNotFoundError(f"Database directory not found: {database_dir}")

    desc_path = os.path.join(database_dir, "database_descriptors.npy")
    poses_path = os.path.join(database_dir, "database_poses.npy")
    paths_path = os.path.join(database_dir, "database_image_paths.txt")
    meta_path = os.path.join(database_dir, "database_meta.npz")
    for p in (desc_path, poses_path, paths_path, meta_path):
        if not os.path.isfile(p):
            raise FileNotFoundError(f"Missing required database file: {p}")

    descriptors = np.load(desc_path).astype("float32", copy=False)
    poses = np.load(poses_path)
    with open(paths_path) as f:
        rel_paths = [line.strip() for line in f if line.strip()]
    abs_paths = [os.path.join(database_dir, p) for p in rel_paths]

    info_path = os.path.join(database_dir, "database_build_info.json")
    build_info: Optional[Dict[str, Any]] = None
    if os.path.isfile(info_path):
        try:
            with open(info_path, "r", encoding="utf-8") as f:
                build_info = json.load(f)
        except Exception as e:
            logger.warning(
                f"{database_dir}: failed to parse database_build_info.json: {e}"
            )

    meta = np.load(meta_path, allow_pickle=False)
    method = str(meta["method"])
    descriptor_dim = int(meta["descriptor_dim"])
    # image_size 在 preprocess 里以 int64 array 形式存：空数组表示 None
    image_size_arr = meta["image_size"] if "image_size" in meta.files else np.array([], dtype=np.int64)
    image_size: Optional[Tuple[int, ...]] = (
        tuple(int(v) for v in image_size_arr.tolist()) if image_size_arr.size > 0 else None
    )

    if not (len(descriptors) == len(poses) == len(rel_paths)):
        raise ValueError(
            f"{database_dir}: inconsistent counts "
            f"(descriptors={len(descriptors)}, poses={len(poses)}, "
            f"image_paths={len(rel_paths)})"
        )
    if descriptors.shape[1] != descriptor_dim:
        raise ValueError(
            f"{database_dir}: descriptor dim mismatch "
            f"(file={descriptors.shape[1]} vs meta={descriptor_dim})"
        )

    return DatabaseBundle(
        path=database_dir,
        descriptors=descriptors,
        poses=poses,
        image_paths_rel=rel_paths,
        image_paths_abs=abs_paths,
        method=method,
        descriptor_dim=descriptor_dim,
        image_size=image_size,
        build_info=build_info,
    )


# ---------------------------------------------------------------------------
# Detector
# ---------------------------------------------------------------------------

def _build_transform(image_size=None):
    """Identical to the transform used in ``vpr_preprocess._DatabaseImageFolder``."""
    ops = [
        transforms.ToTensor(),
        transforms.Normalize(
            mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
        ),
    ]
    if image_size:
        ops.append(transforms.Resize(size=image_size, antialias=True))
    return transforms.Compose(ops)


class VPRDetector:
    """Load model + multiple databases once, then run many queries.

    Parameters
    ----------
    database_dirs : list[str]
        One directory per submap. Each must contain the four files written
        by ``vpr_preprocess.py``.
    device : {"cuda", "cpu"}
        Device for inference.
    image_size : list[int] or None, optional
        Optional ``(H, W)`` resize applied before the model. If not
        provided, the value stored in each database's ``database_meta.npz``
        is used. All databases must share the same ``image_size`` (the
        constructor raises if they disagree, or if the explicit value
        passed here disagrees with what the databases were built with).
    """

    def __init__(self, database_dirs: List[str], device: str = "cuda",
                 image_size=None):
        if not database_dirs:
            raise ValueError("At least one database directory is required.")

        # 几何重排（SuperPoint + LightGlue）是惰性加载的：只有第一次收到
        # rerank=True 的请求时才把模型搬到 GPU，避免不需要 rerank 的部署
        # 多付一份显存 + 启动时间。
        self._reranker = None  # type: Optional[Any]

        self.databases: List[DatabaseBundle] = [
            load_database(d) for d in database_dirs
        ]

        methods = {db.method for db in self.databases}
        dims = {db.descriptor_dim for db in self.databases}
        if len(methods) > 1:
            raise ValueError(
                f"Databases were built with different VPR models {methods}. "
                f"All databases must use the same method."
            )
        if len(dims) > 1:
            raise ValueError(f"Databases have different descriptor dims: {dims}")

        # 所有 database 的 image_size 必须一致
        db_image_sizes = {db.image_size for db in self.databases}
        if len(db_image_sizes) > 1:
            raise ValueError(
                "Databases were preprocessed with different image_size values: "
                + ", ".join(
                    f"{db.path}={db.image_size}" for db in self.databases
                )
                + ". Rebuild them with the same --image_size so the descriptor "
                  "spaces stay comparable."
            )
        db_image_size = next(iter(db_image_sizes))  # 共同的 image_size（可能为 None）

        # 用户显式传入的 image_size 必须与 database 一致；否则一律以 database 为准
        user_image_size = tuple(image_size) if image_size else None
        if user_image_size is not None and user_image_size != db_image_size:
            raise ValueError(
                f"Requested image_size={user_image_size} does not match the "
                f"image_size={db_image_size} used to build the databases. "
                f"They must be identical, otherwise query descriptors will "
                f"not align with database descriptors."
            )
        effective_image_size = user_image_size if user_image_size is not None else db_image_size

        # 所有 database 的去畸变配置必须一致（否则查询时无法决定单一 undistorter）
        self.undistort_config: Optional[Dict[str, Any]] = self._resolve_undistort_config()
        self.undistorter: Optional[PolyFisheyeUndistorter] = (
            self._build_undistorter(self.undistort_config)
            if self.undistort_config
            and self.undistort_config.get("undistort_fisheye")
            else None
        )

        self.method: str = self.databases[0].method
        self.descriptor_dim: int = self.databases[0].descriptor_dim
        self.image_size: Optional[Tuple[int, ...]] = effective_image_size
        self.device = device
        self.transform = _build_transform(effective_image_size)

        # 把所有 submap 的描述符拼到同一个 FAISS index，一次搜索同时覆盖所有库
        all_desc = np.concatenate(
            [db.descriptors for db in self.databases], axis=0
        )
        self.index = faiss.IndexFlatL2(self.descriptor_dim)
        self.index.add(all_desc)

        # _db_starts[k] = sum(N_0..N_{k-1})，用于把全局行号映射回 (db_id, local_idx)
        sizes = [db.num_images for db in self.databases]
        self._db_starts = np.cumsum([0] + sizes)

        logger.info(
            f"Loading VPR model: method={self.method}, device={device}"
        )
        self.model, _ = get_model(self.method, device)

        logger.info(
            f"VPRDetector ready: {len(self.databases)} database(s), "
            f"{int(self._db_starts[-1])} total images, "
            f"dim={self.descriptor_dim}, image_size={self.image_size}, "
            f"undistort={'on' if self.undistorter is not None else 'off'}"
        )
        for db in self.databases:
            logger.info(f"  - {db.path}  ({db.num_images} images)")

    # ------------------------------------------------------------------
    # Undistortion config resolution
    # ------------------------------------------------------------------

    def _resolve_undistort_config(self) -> Optional[Dict[str, Any]]:
        """所有 database 的去畸变配置（来自 build_info.json）必须一致才返回。

        - 若所有 database 都缺 build_info.json：返回 None，且禁用 undistortion。
        - 若部分 database 缺失：直接报错（无法保证查询与构造时的一致）。
        - 若都存在但 _UNDISTORT_KEYS 任一字段不一致：报错。
        """
        infos = [db.build_info for db in self.databases]
        if all(info is None for info in infos):
            return None
        missing = [db.path for db, info in zip(self.databases, infos) if info is None]
        if missing:
            raise ValueError(
                "Some databases are missing database_build_info.json so the "
                "fisheye undistortion config cannot be validated: "
                f"{missing}. Rebuild them with the current vpr_preprocess.py "
                "or remove them from the detector."
            )

        first = infos[0]
        for info, db in zip(infos[1:], self.databases[1:]):
            for k in _UNDISTORT_KEYS:
                if info.get(k) != first.get(k):
                    raise ValueError(
                        f"Inconsistent undistortion config across databases. "
                        f"Field '{k}' is {first.get(k)!r} in "
                        f"{self.databases[0].path} but {info.get(k)!r} in "
                        f"{db.path}. All databases must be built with the same "
                        f"undistortion settings."
                    )
        return {k: first.get(k) for k in _UNDISTORT_KEYS}

    @staticmethod
    def _build_undistorter(cfg: Dict[str, Any]) -> PolyFisheyeUndistorter:
        out_w = int(cfg.get("out_width") or 1024)
        out_h = int(cfg.get("out_height") or 768)
        fov = float(cfg.get("fov") or 90.0)
        calib = cfg.get("calibration_file")
        if calib and os.path.isfile(calib):
            logger.info(
                f"VPRDetector: building undistorter from {calib} "
                f"({out_w}x{out_h}, FOV {fov}°)"
            )
            return PolyFisheyeUndistorter.from_yaml(
                calib, out_w=out_w, out_h=out_h, fov_deg=fov
            )
        if calib:
            logger.warning(
                f"VPRDetector: calibration_file '{calib}' not found, "
                f"falling back to PolyFisheyeUndistorter defaults"
            )
        else:
            logger.info(
                f"VPRDetector: building undistorter with default S20 calibration "
                f"({out_w}x{out_h}, FOV {fov}°)"
            )
        return PolyFisheyeUndistorter.from_defaults(
            out_w=out_w, out_h=out_h, fov_deg=fov
        )

    # ------------------------------------------------------------------
    # Image -> descriptor
    # ------------------------------------------------------------------

    def _to_pil(self, image: ImageInput) -> Image.Image:
        if isinstance(image, (str, os.PathLike, Path)):
            return Image.open(image).convert("RGB")
        if isinstance(image, Image.Image):
            return image.convert("RGB")
        if isinstance(image, np.ndarray):
            return Image.fromarray(image).convert("RGB")
        if isinstance(image, (bytes, bytearray, memoryview)):
            return Image.open(io.BytesIO(bytes(image))).convert("RGB")
        raise TypeError(
            f"Unsupported image input type: {type(image).__name__}. "
            f"Expected path, PIL.Image, numpy.ndarray or bytes."
        )

    def encode(self, image: ImageInput) -> np.ndarray:
        """Compute the (1, D) float32 descriptor for one query image.

        NOTE: This path does **not** apply fisheye undistortion -- it assumes
        the input is already in the same image space as the database (e.g.
        one of the JPGs in ``database_image_paths.txt``). For live raw
        fisheye frames from ROS, use ``encode_raw_bytes`` / ``query_raw_bytes``.
        """
        pil = self._to_pil(image)
        tensor = self.transform(pil).unsqueeze(0).to(self.device)
        with torch.inference_mode():
            desc = self.model(tensor).cpu().numpy()
        return desc.astype("float32", copy=False)

    def encode_raw_bytes(self, jpeg_bytes: bytes) -> np.ndarray:
        """Compute a descriptor from raw camera JPEG bytes.

        Applies the same fisheye undistortion as ``vpr_preprocess.py`` when the
        databases were built with ``--undistort_fisheye``. Without undistortion
        in the databases this just decodes the JPEG and runs the model.
        """
        if self.undistorter is not None:
            rgb = self.undistorter.undistort_bytes(jpeg_bytes)
            pil = Image.fromarray(rgb).convert("RGB")
        else:
            pil = Image.open(io.BytesIO(jpeg_bytes)).convert("RGB")
        tensor = self.transform(pil).unsqueeze(0).to(self.device)
        with torch.inference_mode():
            desc = self.model(tensor).cpu().numpy()
        return desc.astype("float32", copy=False)

    # ------------------------------------------------------------------
    # Retrieval
    # ------------------------------------------------------------------

    def _global_to_local(self, global_idx: int):
        """Map a row in the merged index back to (db_id, local_idx)."""
        # _db_starts is monotonically increasing, length len(databases)+1.
        db_id = int(np.searchsorted(self._db_starts, global_idx, side="right") - 1)
        local_idx = int(global_idx - self._db_starts[db_id])
        return db_id, local_idx

    def _search(self, desc: np.ndarray, top_k: int) -> List[dict]:
        distances, indices = self.index.search(desc, top_k)    # (1, top_k)
        results: List[dict] = []
        for rank, (gidx, dist) in enumerate(zip(indices[0], distances[0])):
            gidx = int(gidx)
            if gidx < 0:
                continue
            db_id, local_idx = self._global_to_local(gidx)
            db = self.databases[db_id]
            x, y, z, ts = db.poses[local_idx]
            results.append({
                "rank": rank,
                "distance": float(dist),
                "database": db.path,
                "image_rel": db.image_paths_rel[local_idx],
                "image_path": db.image_paths_abs[local_idx],
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "timestamp": int(ts),
            })
        return results

    def query(self, image: ImageInput, top_k: int = 1) -> List[dict]:
        """Return the ``top_k`` best matches across all databases.

        ``image`` is treated as already-prepared (same space as the database
        images). Each result is a dict with the following keys:
            rank        int     -- 0 == best
            distance    float   -- L2 distance in descriptor space
            database    str     -- absolute path to the source database dir
            image_rel   str     -- path as stored in database_image_paths.txt
            image_path  str     -- absolute path on disk (database + image_rel)
            x, y, z     float   -- coordinates from database_poses.npy
            timestamp   int     -- bag-message timestamp (ns)
        """
        if top_k < 1:
            raise ValueError("top_k must be >= 1")
        top_k = min(top_k, int(self._db_starts[-1]))
        desc = self.encode(image)                              # (1, D)
        return self._search(desc, top_k)

    def query_raw_bytes(self, jpeg_bytes: bytes, top_k: int = 1) -> List[dict]:
        """Same as ``query`` but takes raw camera JPEG bytes.

        Applies the same fisheye undistortion the databases were built with.
        Primary entry point for live ROS ``CompressedImage`` frames.
        """
        if top_k < 1:
            raise ValueError("top_k must be >= 1")
        top_k = min(top_k, int(self._db_starts[-1]))
        desc = self.encode_raw_bytes(jpeg_bytes)
        return self._search(desc, top_k)

    # ------------------------------------------------------------------
    # Geometric re-ranking (SuperPoint + LightGlue, lazy)
    # ------------------------------------------------------------------

    def _ensure_reranker(self) -> bool:
        """惰性加载 GeometricReranker。失败返回 False（保持 self._reranker=None）。"""
        if self._reranker is not None:
            return True
        try:
            from utils.rerank import GeometricReranker
        except Exception as e:
            logger.error(f"GeometricReranker 不可用（lightglue 未安装？）: {e}")
            return False
        try:
            self._reranker = GeometricReranker(device=self.device)
            logger.info(
                "Geometric reranker (SuperPoint+LightGlue) loaded on %s",
                self.device,
            )
            return True
        except Exception as e:
            logger.error(f"GeometricReranker 加载失败: {e}")
            self._reranker = None
            return False

    def rerank_results(
        self,
        query_image_path: str,
        results: List[dict],
        rerank_top_k: int = 20,
    ) -> List[dict]:
        """对 FAISS 结果做几何重排，按 inlier 数降序返回。

        - 仅对前 ``rerank_top_k`` 个候选做匹配（其余原序追加在尾部）。
        - 每个候选会写回 ``rerank_inliers`` 字段；未参与重排的为 -1。
        - 若 reranker 不可用，原序返回（保持向后兼容，调用方仍能拿到结果）。
        """
        if not results:
            return results
        if not self._ensure_reranker():
            return results

        reranker = self._reranker
        top = results[: max(1, rerank_top_k)]
        tail = results[len(top):]

        try:
            feats_q = reranker._extract(query_image_path)
        except Exception as e:
            logger.warning(f"Rerank: 提取查询特征失败 ({e})，跳过 rerank")
            return results

        inliers: List[int] = []
        for cand in top:
            cand_path = cand.get("image_path")
            if not cand_path or not os.path.isfile(cand_path):
                inliers.append(0)
                continue
            try:
                feats_c = reranker._extract(cand_path)
                inliers.append(reranker.count_inliers(feats_q, feats_c))
            except Exception as e:
                logger.warning(f"Rerank: 候选 {cand_path} 匹配失败 ({e})")
                inliers.append(0)

        order = sorted(range(len(top)), key=lambda i: -inliers[i])
        reranked: List[dict] = []
        for new_rank, old_idx in enumerate(order):
            item = dict(top[old_idx])
            item["rerank_inliers"] = int(inliers[old_idx])
            item["rank"] = new_rank
            reranked.append(item)

        for offset, item in enumerate(tail):
            it = dict(item)
            it["rerank_inliers"] = -1
            it["rank"] = len(reranked) + offset
            reranked.append(it)

        return reranked


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description="Match a query image against one or more VPR databases.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--image", type=str, default=None,
                        help="Path to the query image (one-shot mode)")
    parser.add_argument("--databases", type=str, nargs="+", required=True,
                        help="One or more VPR database directories "
                             "(each produced by vpr_preprocess.py)")
    parser.add_argument("--top_k", type=int, default=1,
                        help="Number of best matches to return")
    parser.add_argument("--device", type=str, default="cuda",
                        choices=["cuda", "cpu"])
    parser.add_argument("--image_size", type=int, nargs="+", default=None,
                        help="Optional resize (H W) before inference "
                             "(use the same value as in vpr_preprocess.py)")
    parser.add_argument("--json", action="store_true",
                        help="Print results as JSON on stdout (and silence logs)")
    parser.add_argument("--serve", action="store_true",
                        help="Run as a long-lived stdin/stdout JSON service. "
                             "Load model+databases once, then process "
                             "newline-delimited JSON requests from stdin "
                             "(see _serve_loop docstring for the protocol).")
    return parser.parse_args()


def _configure_logging(quiet=False):
    logger.remove()
    if quiet:
        return
    logger.add(
        sys.stderr, colorize=True,
        format="<green>{time:%Y-%m-%d %H:%M:%S}</green> {message}",
        level="INFO",
    )


def _print_best(results):
    best = results[0]
    logger.info("Best match:")
    logger.info(f"  database  : {best['database']}")
    logger.info(f"  image_rel : {best['image_rel']}")
    logger.info(f"  image_abs : {best['image_path']}")
    logger.info(f"  xyz       : ({best['x']:.3f}, {best['y']:.3f}, {best['z']:.3f})")
    logger.info(f"  timestamp : {best['timestamp']}")
    logger.info(f"  distance  : {best['distance']:.4f}")


def _print_topk(results):
    logger.info(f"Top-{len(results)} candidates:")
    for r in results:
        db_name = os.path.basename(r["database"].rstrip(os.sep)) or r["database"]
        logger.info(
            f"  [{r['rank']}] dist={r['distance']:.4f}  "
            f"db={db_name}  xyz=({r['x']:.2f},{r['y']:.2f},{r['z']:.2f})  "
            f"{r['image_rel']}"
        )


def _emit(obj: dict) -> None:
    """Write one newline-delimited JSON event to stdout and flush."""
    sys.stdout.write(json.dumps(obj, ensure_ascii=False) + "\n")
    sys.stdout.flush()


def _serve_loop(detector: VPRDetector) -> None:
    """Long-lived stdin/stdout JSON RPC for live VPR queries.

    Protocol (one JSON object per line, in both directions):

    Server -> client (once at startup, after databases+model are ready)::

        {"event": "ready",
         "method": "...",
         "descriptor_dim": ...,
         "image_size": [...],
         "undistort": true/false,
         "databases": [{"path": "...", "num_images": ...}, ...]}

    Client -> server::

        {"id": "<opaque-string>",
         "image_path": "/tmp/live.jpg",       # raw camera JPEG bytes on disk
         "top_k": 1,
         "rerank": false,                     # optional, default false
         "rerank_top_k": 20}                  # optional, only when rerank=true

    Either ``image_path`` (raw JPEG bytes on disk; undistortion is applied
    if the databases were built with it) **or** ``image_processed_path``
    (already in the database's image space; undistortion is NOT applied)
    must be set.

    When ``rerank`` is true, the detector first retrieves
    ``max(top_k, rerank_top_k)`` candidates from FAISS, then re-orders them
    with SuperPoint + LightGlue geometric matching, and returns the top
    ``top_k`` after re-ranking. Each result gains a ``rerank_inliers`` field
    (-1 if the candidate was past the rerank cutoff or rerank was skipped).

    Server -> client (per request)::

        {"id": "<echoed>", "results": [...]}   # success
        {"id": "<echoed>", "error": "..."}     # failure
    """
    _emit({
        "event": "ready",
        "method": detector.method,
        "descriptor_dim": detector.descriptor_dim,
        "image_size": list(detector.image_size) if detector.image_size else None,
        "undistort": detector.undistorter is not None,
        "databases": [
            {"path": db.path, "num_images": db.num_images}
            for db in detector.databases
        ],
    })

    for raw_line in sys.stdin:
        line = raw_line.strip()
        if not line:
            continue
        req_id: Any = None
        try:
            req = json.loads(line)
            req_id = req.get("id")
            top_k = int(req.get("top_k", 1))
            rerank = bool(req.get("rerank", False))
            rerank_top_k = int(req.get("rerank_top_k", 20))
            image_path = req.get("image_path")
            processed_path = req.get("image_processed_path")
            if not image_path and not processed_path:
                raise ValueError(
                    "request needs 'image_path' (raw bytes) or "
                    "'image_processed_path' (already in database space)"
                )

            # 开启 rerank 时先多召回一些候选给 rerank 重排，再截到 top_k
            search_k = max(top_k, rerank_top_k) if rerank else top_k

            t0 = time.time()
            if image_path:
                with open(image_path, "rb") as f:
                    jpeg = f.read()
                results = detector.query_raw_bytes(jpeg, top_k=search_k)
            else:
                results = detector.query(processed_path, top_k=search_k)
            faiss_ms = (time.time() - t0) * 1000.0

            rerank_ms = 0.0
            if rerank and results:
                # query 路径优先使用 image_path（与离线建库相同的"原始字节"语义），
                # 否则用 processed_path（已经在数据库空间里的预处理图）。
                query_path_for_rerank = image_path or processed_path
                tr0 = time.time()
                results = detector.rerank_results(
                    query_path_for_rerank, results, rerank_top_k=rerank_top_k,
                )
                rerank_ms = (time.time() - tr0) * 1000.0
                results = results[:top_k]

            elapsed_ms = faiss_ms + rerank_ms

            _emit({
                "id": req_id,
                "results": results,
                "elapsed_ms": round(elapsed_ms, 2),
                "faiss_ms": round(faiss_ms, 2),
                "rerank_ms": round(rerank_ms, 2),
            })
        except Exception as e:
            _emit({
                "id": req_id,
                "error": str(e),
                "traceback": traceback.format_exc(),
            })


def main():
    args = parse_args()

    if args.serve:
        # 服务模式不能让 loguru 写到 stdout，否则会污染协议帧
        logger.remove()
        logger.add(sys.stderr, level="INFO",
                   format="{time:%Y-%m-%d %H:%M:%S} | {level} | {message}")
        detector = VPRDetector(
            database_dirs=args.databases,
            device=args.device,
            image_size=args.image_size,
        )
        _serve_loop(detector)
        return

    _configure_logging(quiet=args.json)
    if not args.image:
        raise SystemExit("--image is required when not running in --serve mode")
    if not os.path.isfile(args.image):
        raise FileNotFoundError(f"Image not found: {args.image}")

    detector = VPRDetector(
        database_dirs=args.databases,
        device=args.device,
        image_size=args.image_size,
    )

    logger.info(f"Querying with {args.image}")
    results = detector.query(args.image, top_k=args.top_k)

    if not results:
        if args.json:
            print(json.dumps({"results": []}))
        else:
            logger.warning("No match found.")
        sys.exit(1)

    if args.json:
        print(json.dumps({"results": results}, indent=2, ensure_ascii=False))
        return

    _print_best(results)
    if args.top_k > 1:
        _print_topk(results)


if __name__ == "__main__":
    main()
