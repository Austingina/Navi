"""Stateful VPR engine for C++ integration.

Provides a class-based API around the multi-stage pipeline:
  1. Global descriptor extraction (MegaLoc or SALAD-indoor)
  2. FAISS L2 kNN retrieval
  3. SuperPoint + LightGlue geometric re-ranking (optional)
  4. HSV color histogram re-scoring (optional)

Designed for embedding in C++ via pybind11.  All public methods accept
simple types (strings, numpy arrays, ints) that map cleanly to C++.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Optional

import faiss
import numpy as np
import torch
import torchvision.transforms as transforms
from PIL import Image
from torch.utils.data import DataLoader, Dataset

from .model_loader import get_model


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _read_image_paths(folder: str) -> list[str]:
    """Collect image paths from a folder, sorted alphabetically."""
    exts = {".jpg", ".jpeg", ".png"}
    paths = []
    for root, _, files in os.walk(folder):
        for f in sorted(files):
            if os.path.splitext(f)[1].lower() in exts:
                paths.append(os.path.join(root, f))
    paths.sort()
    if not paths:
        raise FileNotFoundError(f"No images found in {folder}")
    return paths


def _build_transform(image_size: Optional[list[int]] = None):
    """ImageNet-normalised tensor transform, with optional resize."""
    ops = [
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225]),
    ]
    if image_size:
        ops.append(transforms.Resize(size=image_size, antialias=True))
    return transforms.Compose(ops)


class _ImageListDataset(Dataset):
    """Minimal dataset that loads images from a list of paths."""

    def __init__(self, paths: list[str], transform):
        self.paths = paths
        self.transform = transform

    def __len__(self):
        return len(self.paths)

    def __getitem__(self, idx):
        img = Image.open(self.paths[idx]).convert("RGB")
        return self.transform(img), idx


# ---------------------------------------------------------------------------
# VPREngine
# ---------------------------------------------------------------------------

class VPREngine:
    """Visual Place Recognition engine.

    Args:
        method: Global descriptor model. ``"megaloc"`` (outdoor/geometry)
            or ``"salad-indoor"`` (indoor/appearance). Both produce 8448-d
            descriptors from a DINOv2 backbone.
        device: ``"cuda"`` or ``"cpu"``.
        rerank: Pre-load SuperPoint + LightGlue for geometric re-ranking.
        max_keypoints: SuperPoint max keypoints (only when ``rerank=True``).
        rerank_resize: Max image dimension for SuperPoint extraction.
        color_rerank: Enable HSV color histogram re-scoring.
        color_weight: Alpha for color scoring:
            ``score = base * (1 + alpha * color_sim)``.
        image_size: Optional ``[H, W]`` resize applied before descriptor
            extraction.
    """

    def __init__(
        self,
        method: str = "megaloc",
        device: str = "cuda",
        rerank: bool = False,
        max_keypoints: int = 1024,
        rerank_resize: int = 512,
        color_rerank: bool = False,
        color_weight: float = 5.0,
        image_size: Optional[list[int]] = None,
    ):
        self.device = device
        self.method = method
        self.image_size = image_size
        self.model, self.descriptor_dim = get_model(method, device)
        self.transform = _build_transform(image_size)

        self.faiss_index: Optional[faiss.IndexFlatL2] = None
        self.database_paths: list[str] = []
        self.database_descriptors: Optional[np.ndarray] = None

        self.reranker = None
        if rerank:
            from .rerank import GeometricReranker
            self.reranker = GeometricReranker(device, max_keypoints, rerank_resize)

        self.color_reranker = None
        if color_rerank:
            from .color_rerank import ColorReranker
            self.color_reranker = ColorReranker(alpha=color_weight)

    # ------------------------------------------------------------------
    # Descriptor extraction
    # ------------------------------------------------------------------

    @torch.inference_mode()
    def _extract_single(self, tensor: torch.Tensor) -> np.ndarray:
        """Extract a single descriptor from a pre-processed image tensor."""
        return self.model(tensor.unsqueeze(0).to(self.device)).cpu().numpy()[0]

    @torch.inference_mode()
    def _extract_batch(self, paths: list[str],
                       batch_size: int = 4,
                       num_workers: int = 4) -> np.ndarray:
        """Extract descriptors for a list of image paths."""
        ds = _ImageListDataset(paths, self.transform)
        loader = DataLoader(ds, batch_size=batch_size,
                            num_workers=num_workers, pin_memory=True)
        out = np.empty((len(paths), self.descriptor_dim), dtype="float32")
        for images, indices in loader:
            desc = self.model(images.to(self.device)).cpu().numpy()
            out[indices.numpy()] = desc
        return out

    # ------------------------------------------------------------------
    # Database management
    # ------------------------------------------------------------------

    def build_database(self, database_folder: str,
                       batch_size: int = 4,
                       num_workers: int = 4) -> int:
        """Build the FAISS index from a folder of database images.

        Returns the number of images indexed.
        """
        self.database_paths = _read_image_paths(database_folder)
        self.database_descriptors = self._extract_batch(
            self.database_paths, batch_size, num_workers
        )
        self.faiss_index = faiss.IndexFlatL2(self.descriptor_dim)
        self.faiss_index.add(self.database_descriptors)
        return len(self.database_paths)

    def build_database_from_descriptors(
        self,
        descriptors: np.ndarray,
        image_paths: list[str],
    ) -> int:
        """Build from pre-computed descriptors (e.g. loaded from .npy)."""
        self.database_descriptors = np.ascontiguousarray(
            descriptors, dtype="float32"
        )
        self.database_paths = list(image_paths)
        self.faiss_index = faiss.IndexFlatL2(self.descriptor_dim)
        self.faiss_index.add(self.database_descriptors)
        return len(self.database_paths)

    def add_to_database(self, image_path: str) -> int:
        """Add a single image (by path) to the live database.

        Returns the new image's database index.
        """
        tensor = self.transform(Image.open(image_path).convert("RGB"))
        descriptor = self._extract_single(tensor).reshape(1, -1)
        return self._append_descriptor(descriptor, image_path)

    def add_image_to_database(self, image_array: np.ndarray,
                              label: str = "") -> int:
        """Add a raw image (H, W, 3 uint8 RGB numpy array) to the database.

        ``label`` is stored as the path/identifier for this entry.
        Returns the new database index.
        """
        tensor = self.transform(Image.fromarray(image_array))
        descriptor = self._extract_single(tensor).reshape(1, -1)
        return self._append_descriptor(descriptor, label)

    def _append_descriptor(self, descriptor: np.ndarray, path: str) -> int:
        if self.faiss_index is None:
            self.faiss_index = faiss.IndexFlatL2(self.descriptor_dim)
            self.database_descriptors = descriptor
        else:
            self.database_descriptors = np.vstack(
                [self.database_descriptors, descriptor]
            )
        self.faiss_index.add(descriptor)
        self.database_paths.append(path)
        return len(self.database_paths) - 1

    def database_size(self) -> int:
        """Number of images currently in the database."""
        return len(self.database_paths)

    # ------------------------------------------------------------------
    # Querying
    # ------------------------------------------------------------------

    def query(self, image_path: str, top_k: int = 5) -> dict:
        """Query with an image file path.

        Returns
        -------
        dict with keys:
            indices   : list[int]   – matched database indices
            distances : list[float] – L2 distances
            paths     : list[str]   – matched database image paths
        """
        self._check_database()
        tensor = self.transform(Image.open(image_path).convert("RGB"))
        descriptor = self._extract_single(tensor).reshape(1, -1)
        return self._search(descriptor, top_k)

    def query_image(self, image_array: np.ndarray, top_k: int = 5) -> dict:
        """Query with a raw image (H, W, 3 uint8 RGB numpy array)."""
        self._check_database()
        tensor = self.transform(Image.fromarray(image_array))
        descriptor = self._extract_single(tensor).reshape(1, -1)
        return self._search(descriptor, top_k)

    def query_descriptor(self, descriptor: np.ndarray,
                         top_k: int = 5) -> dict:
        """Query with a pre-computed descriptor vector."""
        self._check_database()
        d = np.ascontiguousarray(descriptor, dtype="float32").reshape(1, -1)
        return self._search(d, top_k)

    def query_with_rerank(self, image_path: str,
                          top_k: int = 5,
                          rerank_k: int = 20) -> dict:
        """Query + geometric re-ranking via SuperPoint + LightGlue.

        Returns
        -------
        dict with keys:
            indices : list[int]   – matched database indices (re-ranked)
            scores  : list[float] – inlier counts
            paths   : list[str]   – matched database image paths
        """
        if self.reranker is None:
            raise RuntimeError(
                "Reranker not loaded. Pass rerank=True to the constructor."
            )
        self._check_database()

        tensor = self.transform(Image.open(image_path).convert("RGB"))
        descriptor = self._extract_single(tensor).reshape(1, -1)
        distances, indices = self.faiss_index.search(descriptor, rerank_k)
        faiss_indices = indices[0]

        candidate_paths = [self.database_paths[i] for i in faiss_indices]
        sorted_local, sorted_scores = self.reranker.rerank(
            image_path, candidate_paths
        )

        reranked = faiss_indices[sorted_local][:top_k]
        scores = sorted_scores[:top_k]

        return {
            "indices": reranked.tolist(),
            "scores": scores.tolist(),
            "paths": [self.database_paths[i] for i in reranked],
        }

    def query_with_color_rerank(
        self,
        image_path: str,
        top_k: int = 5,
        candidates_k: int = 20,
    ) -> dict:
        """Query + color histogram re-scoring on FAISS candidates.

        Best for cross-floor disambiguation where structural similarity
        is high but color schemes differ. Does NOT use geometric re-ranking.

        Args:
            image_path: Path to the query image.
            top_k: Number of results to return.
            candidates_k: Number of FAISS candidates to re-score.

        Returns:
            dict with keys ``indices``, ``scores``, ``paths``.
        """
        if self.color_reranker is None:
            raise RuntimeError(
                "Color reranker not loaded. Pass color_rerank=True to the constructor."
            )
        self._check_database()

        tensor = self.transform(Image.open(image_path).convert("RGB"))
        descriptor = self._extract_single(tensor).reshape(1, -1)
        distances, indices = self.faiss_index.search(descriptor, candidates_k)

        faiss_indices = indices[0]
        faiss_distances = distances[0]
        max_dist = faiss_distances.max() + 1.0
        base_scores = (max_dist - faiss_distances).astype(np.float64)

        candidate_paths = [self.database_paths[i] for i in faiss_indices]
        sorted_local, sorted_scores = self.color_reranker.rescore(
            image_path, candidate_paths, base_scores,
        )

        reranked = faiss_indices[sorted_local][:top_k]
        scores = sorted_scores[:top_k]

        return {
            "indices": reranked.tolist(),
            "scores": scores.tolist(),
            "paths": [self.database_paths[i] for i in reranked],
        }

    def _search(self, descriptor: np.ndarray, top_k: int) -> dict:
        top_k = min(top_k, len(self.database_paths))
        distances, indices = self.faiss_index.search(descriptor, top_k)
        idx = indices[0]
        return {
            "indices": idx.tolist(),
            "distances": distances[0].tolist(),
            "paths": [self.database_paths[i] for i in idx],
        }

    def _check_database(self):
        if self.faiss_index is None or len(self.database_paths) == 0:
            raise RuntimeError(
                "Database is empty. Call build_database() or add_to_database() first."
            )

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def save_database(self, directory: str) -> None:
        """Save the FAISS index, descriptors, and path list to *directory*."""
        self._check_database()
        base = Path(directory)
        base.mkdir(parents=True, exist_ok=True)

        faiss.write_index(self.faiss_index, str(base / "index.faiss"))
        np.save(str(base / "descriptors.npy"), self.database_descriptors)
        with open(base / "metadata.json", "w") as f:
            json.dump({"paths": self.database_paths,
                       "descriptor_dim": self.descriptor_dim}, f)

    def load_database(self, directory: str) -> int:
        """Load a previously saved database.  Returns number of images."""
        base = Path(directory)
        self.faiss_index = faiss.read_index(str(base / "index.faiss"))
        self.database_descriptors = np.load(str(base / "descriptors.npy"))
        with open(base / "metadata.json") as f:
            meta = json.load(f)
        self.database_paths = meta["paths"]
        return len(self.database_paths)

    # ------------------------------------------------------------------
    # Descriptor access (for advanced use)
    # ------------------------------------------------------------------

    def extract_descriptor(self, image_path: str) -> np.ndarray:
        """Extract a global descriptor for a single image (returned as 1-D array)."""
        tensor = self.transform(Image.open(image_path).convert("RGB"))
        return self._extract_single(tensor)

    def extract_descriptor_from_image(self,
                                      image_array: np.ndarray) -> np.ndarray:
        """Extract a global descriptor from a raw image array."""
        tensor = self.transform(Image.fromarray(image_array))
        return self._extract_single(tensor)
