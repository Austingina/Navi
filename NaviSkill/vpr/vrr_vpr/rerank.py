"""Geometric re-ranking using SuperPoint + LightGlue.

Re-ranks top-k FAISS candidates by counting geometrically consistent
local feature matches (inliers).
"""

import numpy as np
import torch
from lightglue import LightGlue, SuperPoint
from lightglue.utils import load_image


class GeometricReranker:
    """Re-rank VPR candidates using SuperPoint + LightGlue."""

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

        Returns (sorted_indices, sorted_scores).
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
