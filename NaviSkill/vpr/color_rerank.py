"""Color-aware re-scoring for VPR predictions using HSV histogram similarity.

After geometric re-ranking (SuperPoint + LightGlue), this module re-scores
candidates by combining inlier counts with color histogram similarity.
This addresses cross-floor confusion where geometrically identical corridors
have distinct color schemes that color-invariant models ignore.

Usage:
    color_reranker = ColorReranker(alpha=0.5)
    predictions, scores = color_reranker.rescore_all(
        predictions, rerank_scores, test_ds
    )
"""

from __future__ import annotations

import cv2
import numpy as np
from loguru import logger
from tqdm import tqdm


class ColorReranker:
    """Re-score VPR candidates using HSV color histogram similarity.

    Computes a 2D hue-saturation histogram for each image, ignoring very
    dark and very bright pixels whose hue is unreliable. The final score
    combines the existing geometric score with color similarity:

        final_score = geo_score * (1 + alpha * color_similarity)

    This preserves the geometric signal (zero inliers stays zero) while
    boosting candidates whose color distribution matches the query.

    Args:
        h_bins: Number of hue histogram bins (0-180 in OpenCV).
        s_bins: Number of saturation histogram bins (0-255).
        alpha: Weight for color similarity in the combined score.
        v_low: Minimum V channel value; darker pixels are masked out.
        v_high: Maximum V channel value; brighter pixels are masked out.
    """

    def __init__(
        self,
        h_bins: int = 32,
        s_bins: int = 32,
        alpha: float = 0.5,
        v_low: int = 30,
        v_high: int = 230,
    ):
        self.h_bins = h_bins
        self.s_bins = s_bins
        self.alpha = alpha
        self.v_low = v_low
        self.v_high = v_high

    def _compute_hs_histogram(self, image_path: str) -> np.ndarray:
        """Compute a normalized 2D hue-saturation histogram.

        Args:
            image_path: Path to an image file.

        Returns:
            Normalized histogram of shape (h_bins, s_bins), float32.
        """
        img = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if img is None:
            logger.warning(f"Could not read image: {image_path}")
            return np.zeros((self.h_bins, self.s_bins), dtype=np.float32)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        v_channel = hsv[:, :, 2]
        mask = ((v_channel >= self.v_low) & (v_channel <= self.v_high)).astype(np.uint8) * 255

        hist = cv2.calcHist(
            [hsv], [0, 1], mask,
            [self.h_bins, self.s_bins],
            [0, 180, 0, 256],
        )
        cv2.normalize(hist, hist, norm_type=cv2.NORM_L1)
        return hist.astype(np.float32)

    def color_similarity(self, hist_a: np.ndarray, hist_b: np.ndarray) -> float:
        """Compute color similarity between two HS histograms.

        Uses Bhattacharyya coefficient (1 - Bhattacharyya distance) so that
        1.0 = identical distributions and 0.0 = maximally different.

        Args:
            hist_a: Normalized HS histogram.
            hist_b: Normalized HS histogram.

        Returns:
            Similarity in [0, 1].
        """
        dist = cv2.compareHist(hist_a, hist_b, cv2.HISTCMP_BHATTACHARYYA)
        return float(max(0.0, 1.0 - dist))

    def rescore(
        self,
        query_path: str,
        candidate_paths: list[str],
        geo_scores: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Re-score and re-sort candidates by combining geometry with color.

        Args:
            query_path: Path to the query image.
            candidate_paths: Paths to candidate database images.
            geo_scores: Geometric scores (e.g. inlier counts) for each candidate,
                already sorted in descending order.

        Returns:
            sorted_indices: Indices into the original candidate list, sorted
                by descending combined score.
            combined_scores: The combined scores after sorting.
        """
        query_hist = self._compute_hs_histogram(query_path)

        combined = np.empty(len(candidate_paths), dtype=np.float64)
        for i, cand_path in enumerate(candidate_paths):
            cand_hist = self._compute_hs_histogram(cand_path)
            color_sim = self.color_similarity(query_hist, cand_hist)
            combined[i] = float(geo_scores[i]) * (1.0 + self.alpha * color_sim)

        sorted_indices = np.argsort(-combined)
        return sorted_indices, combined[sorted_indices]

    def rescore_all(
        self,
        predictions: np.ndarray,
        geo_scores: np.ndarray,
        test_ds,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Re-score predictions for all queries.

        Args:
            predictions: Array of shape (num_queries, k) with database indices,
                sorted by the previous ranking stage.
            geo_scores: Array of shape (num_queries, k) with geometric scores
                (inlier counts or inverse L2 distances).
            test_ds: TestDataset with ``queries_paths`` and ``database_paths``.

        Returns:
            rescored_predictions: Same shape as predictions, re-sorted by
                combined score.
            combined_scores: Same shape, with the new combined scores.
        """
        num_queries, k = predictions.shape
        rescored = np.empty_like(predictions)
        combined_scores = np.zeros((num_queries, k), dtype=np.float64)

        for qi in tqdm(range(num_queries), desc="Color re-scoring"):
            query_path = test_ds.queries_paths[qi]
            candidate_db_indices = predictions[qi]
            candidate_paths = [test_ds.database_paths[idx] for idx in candidate_db_indices]

            sorted_indices, sorted_scores = self.rescore(
                query_path, candidate_paths, geo_scores[qi],
            )
            rescored[qi] = candidate_db_indices[sorted_indices]
            combined_scores[qi] = sorted_scores

        return rescored, combined_scores
