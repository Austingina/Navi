"""Geometric re-ranking using SuperPoint + LightGlue.

After global descriptor retrieval (Stage 1), this module re-ranks the top-k
candidates by counting geometrically consistent local feature matches (inliers).
This eliminates false positives caused by perceptual aliasing.

Can be toggled on/off without affecting the rest of the pipeline.
"""

import numpy as np
import torch
from lightglue import LightGlue, SuperPoint
from lightglue.utils import load_image
from tqdm import tqdm


class GeometricReranker:
    """Re-rank VPR candidates using SuperPoint + LightGlue local feature matching.

    Parameters
    ----------
    device : str
        "cuda" or "cpu".
    max_keypoints : int
        Maximum keypoints per image. More = better matching but slower.
    resize : int
        Resize images to this max dimension before feature extraction.
    """

    def __init__(self, device="cuda", max_keypoints=1024, resize=512):
        self.device = device
        self.resize = resize
        self.extractor = SuperPoint(max_num_keypoints=max_keypoints).eval().to(device)
        self.matcher = LightGlue(features="superpoint").eval().to(device)

    @torch.inference_mode()
    def _extract(self, image_path):
        """Extract SuperPoint features from an image."""
        image = load_image(image_path, resize=self.resize).to(self.device)
        return self.extractor.extract(image)

    @torch.inference_mode()
    def count_inliers(self, feats_query, feats_candidate):
        """Match two feature sets and return the number of valid matches."""
        try:
            matches = self.matcher({"image0": feats_query, "image1": feats_candidate})
            return int((matches["matches0"][0] > -1).sum().item())
        except Exception:
            return 0

    def rerank(self, query_path, candidate_paths):
        """Re-rank candidates by inlier count against a query.

        Parameters
        ----------
        query_path : str
            Path to the query image.
        candidate_paths : list[str]
            Paths to candidate database images.

        Returns
        -------
        sorted_indices : np.ndarray
            Indices into candidate_paths, sorted by descending inlier count.
        sorted_scores : np.ndarray
            Inlier counts in the same order.
        """
        feats_query = self._extract(query_path)
        scores = []
        for cand_path in candidate_paths:
            feats_cand = self._extract(cand_path)
            inliers = self.count_inliers(feats_query, feats_cand)
            scores.append(inliers)

        scores = np.array(scores)
        sorted_indices = np.argsort(-scores)
        return sorted_indices, scores[sorted_indices]

    def rerank_all(self, predictions, dataset):
        """Re-rank predictions for all queries.

        Parameters
        ----------
        predictions : np.ndarray, shape (num_queries, k)
            Each row contains database indices from FAISS retrieval.
        dataset : VPRDataset
            Dataset with image paths.

        Returns
        -------
        reranked : np.ndarray, same shape as predictions
        confidence_scores : np.ndarray, shape (num_queries, k)
            Inlier counts per prediction.
        """
        num_queries, k = predictions.shape
        reranked = np.empty_like(predictions)
        confidence_scores = np.zeros((num_queries, k), dtype=np.int32)

        for qi in tqdm(range(num_queries), desc="Re-ranking with LightGlue"):
            query_path = dataset.queries_paths[qi]
            candidate_db_indices = predictions[qi]
            candidate_paths = [dataset.database_paths[idx] for idx in candidate_db_indices]

            sorted_indices, sorted_scores = self.rerank(query_path, candidate_paths)
            reranked[qi] = candidate_db_indices[sorted_indices]
            confidence_scores[qi] = sorted_scores

        return reranked, confidence_scores
