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
import json
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple, Union

import faiss
import numpy as np
import torch
import torchvision.transforms as transforms
from loguru import logger
from PIL import Image

from model_loader import get_model


ImageInput = Union[str, "os.PathLike[str]", Path, Image.Image, np.ndarray]


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
            f"dim={self.descriptor_dim}, image_size={self.image_size}"
        )
        for db in self.databases:
            logger.info(f"  - {db.path}  ({db.num_images} images)")

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
        raise TypeError(
            f"Unsupported image input type: {type(image).__name__}. "
            f"Expected path, PIL.Image, or numpy.ndarray."
        )

    def encode(self, image: ImageInput) -> np.ndarray:
        """Compute the (1, D) float32 descriptor for one query image."""
        pil = self._to_pil(image)
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

    def query(self, image: ImageInput, top_k: int = 1) -> List[dict]:
        """Return the ``top_k`` best matches across all databases.

        Each result is a dict with the following keys:
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


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description="Match a query image against one or more VPR databases.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--image", type=str, required=True,
                        help="Path to the query image")
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


def main():
    args = parse_args()
    _configure_logging(quiet=args.json)

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
