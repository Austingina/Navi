"""Regression coverage for instance-to-semantic conversion without model weights."""

import numpy as np
import pytest

from navi_semantic_perception.sam3_engine import merge_instances


def test_grass_keeps_id_two_and_zero_background():
    masks = np.array([[[True, False, True], [False, False, False]]])
    labels, confidence = merge_instances((2, 3), [(2, masks, [0.8])])
    np.testing.assert_array_equal(labels, [[2, 0, 2], [0, 0, 0]])
    np.testing.assert_array_equal(confidence, [[204, 0, 204], [0, 0, 0]])
    assert labels.dtype == confidence.dtype == np.uint8


def test_overlap_uses_highest_score_across_instances_and_classes():
    grass = np.array([[[True, True, False]], [[False, True, True]]])
    sidewalk = np.array([[[True, True, False]]])
    labels, confidence = merge_instances(
        (1, 3), [(2, grass, [0.6, 0.9]), (1, sidewalk, [0.8])])
    np.testing.assert_array_equal(labels, [[1, 2, 2]])
    np.testing.assert_array_equal(confidence, [[204, 230, 230]])


def test_no_detections_returns_background():
    labels, confidence = merge_instances((2, 3), [(2, np.empty((0, 2, 3)), [])])
    assert not labels.any()
    assert not confidence.any()


def test_rejects_transposed_mask_even_with_same_pixel_count():
    with pytest.raises(ValueError, match='original color image grid'):
        merge_instances((2, 3), [(2, np.ones((1, 3, 2)), [0.8])])


@pytest.mark.parametrize('score', [np.nan, np.inf, -0.1, 1.1])
def test_rejects_invalid_confidence(score):
    with pytest.raises(ValueError, match='finite probabilities'):
        merge_instances((2, 3), [(2, np.ones((1, 2, 3)), [score])])
