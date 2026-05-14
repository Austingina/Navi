"""Color-aware re-scoring using HSV histogram similarity.

Re-scores VPR candidates by combining retrieval scores with color histogram
similarity. Addresses cross-floor confusion where geometrically identical
corridors have distinct color schemes.

    final_score = base_score * (1 + alpha * color_similarity)
"""

from __future__ import annotations

import cv2
import numpy as np
from loguru import logger


class ColorReranker:
    """Re-score VPR candidates using HSV color histogram similarity.

    Computes a 2D hue-saturation histogram for each image, masking very
    dark and very bright pixels whose hue is unreliable. Combines the
    existing retrieval score with color similarity multiplicatively.

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

    def compute_histogram(self, image_path: str) -> np.ndarray:
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
        mask = (
            (v_channel >= self.v_low) & (v_channel <= self.v_high)
        ).astype(np.uint8) * 255

        hist = cv2.calcHist(
            [hsv], [0, 1], mask,
            [self.h_bins, self.s_bins],
            [0, 180, 0, 256],
        )
        cv2.normalize(hist, hist, norm_type=cv2.NORM_L1)
        return hist.astype(np.float32)

    def compute_histogram_from_array(self, image_bgr: np.ndarray) -> np.ndarray:
        """Compute histogram from a BGR numpy array (H, W, 3).

        Args:
            image_bgr: BGR image as numpy array.

        Returns:
            Normalized histogram of shape (h_bins, s_bins), float32.
        """
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        v_channel = hsv[:, :, 2]
        mask = (
            (v_channel >= self.v_low) & (v_channel <= self.v_high)
        ).astype(np.uint8) * 255

        hist = cv2.calcHist(
            [hsv], [0, 1], mask,
            [self.h_bins, self.s_bins],
            [0, 180, 0, 256],
        )
        cv2.normalize(hist, hist, norm_type=cv2.NORM_L1)
        return hist.astype(np.float32)

    def similarity(self, hist_a: np.ndarray, hist_b: np.ndarray) -> float:
        """Bhattacharyya similarity between two HS histograms.

        Args:
            hist_a: Normalized HS histogram.
            hist_b: Normalized HS histogram.

        Returns:
            Similarity in [0, 1] (1.0 = identical, 0.0 = maximally different).
        """
        dist = cv2.compareHist(hist_a, hist_b, cv2.HISTCMP_BHATTACHARYYA)
        return float(max(0.0, 1.0 - dist))

    def rescore(
        self,
        query_path: str,
        candidate_paths: list[str],
        base_scores: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Re-score and re-sort candidates by combining base scores with color.

        Args:
            query_path: Path to the query image.
            candidate_paths: Paths to candidate database images.
            base_scores: Retrieval scores for each candidate (higher = better).

        Returns:
            sorted_indices: Indices into the original candidate list, sorted
                by descending combined score.
            combined_scores: The combined scores after sorting.
        """
        query_hist = self.compute_histogram(query_path)

        combined = np.empty(len(candidate_paths), dtype=np.float64)
        for i, cand_path in enumerate(candidate_paths):
            cand_hist = self.compute_histogram(cand_path)
            color_sim = self.similarity(query_hist, cand_hist)
            combined[i] = float(base_scores[i]) * (1.0 + self.alpha * color_sim)

        sorted_indices = np.argsort(-combined)
        return sorted_indices, combined[sorted_indices]
