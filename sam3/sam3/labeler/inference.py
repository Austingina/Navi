# Copyright (c) Meta Platforms, Inc. and affiliates. All Rights Reserved
"""SAM 3 inference wrapper for the browser-based annotation tool.

Exposes two paths against a single cached image embedding:

* ``detect_pcs``: Promptable Concept Segmentation (text + optional
  positive/negative box exemplars) returning every matching instance.
* ``segment_pvs``: Promptable Visual Segmentation (per-object positive/
  negative point clicks and an optional box), returning one mask per
  object slot. This is the SAM 1-task path the Meta notebook
  ``sam3/examples/sam3_for_sam1_task_example.ipynb`` demonstrates.
"""

from __future__ import annotations

import contextlib
import logging
from collections import OrderedDict
from typing import Any, ContextManager, Dict, List, Optional, Sequence, Tuple

import numpy as np
import torch
from PIL import Image

from sam3 import build_sam3_image_model
from sam3.model.sam3_image_processor import Sam3Processor


logger = logging.getLogger(__name__)


# A single PVS object slot: any mix of positive points, negative points,
# and at most one box (xyxy in pixels of the original image).
PvsObject = Dict[str, Any]
# {"pos_points": [(x, y), ...], "neg_points": [(x, y), ...], "box": (x1, y1, x2, y2) | None}


def _xyxy_pix_to_cxcywh_norm(
    box_xyxy_pix: Sequence[float], width: int, height: int
) -> List[float]:
    """Convert pixel xyxy to normalized cxcywh required by ``add_geometric_prompt``."""
    x1, y1, x2, y2 = box_xyxy_pix
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    cx = (x1 + x2) / 2.0 / max(1.0, float(width))
    cy = (y1 + y2) / 2.0 / max(1.0, float(height))
    w = (x2 - x1) / max(1.0, float(width))
    h = (y2 - y1) / max(1.0, float(height))
    return [float(cx), float(cy), float(w), float(h)]


def _mask_to_xyxy(mask: np.ndarray) -> Tuple[float, float, float, float]:
    """Tight xyxy bounding box of a boolean mask, in pixel coords."""
    if mask.ndim != 2:
        mask = mask.squeeze()
    ys, xs = np.where(mask.astype(bool))
    if xs.size == 0:
        return (0.0, 0.0, 0.0, 0.0)
    return (float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max()))


def _ensure_rgb(image: Image.Image) -> Image.Image:
    if image.mode != "RGB":
        return image.convert("RGB")
    return image


class Sam3Annotator:
    """Single point of entry for both SAM 3 image-inference paths.

    The ``inference_state`` returned by ``processor.set_image`` is the
    same object both modes operate on, so a directory walk only pays the
    image-embedding cost once per image even if the user toggles modes.
    """

    def __init__(
        self,
        bpe_path: Optional[str] = None,
        checkpoint_path: Optional[str] = None,
        device: str = "cuda",
        confidence_threshold: float = 0.5,
        max_cached_states: int = 8,
    ) -> None:
        if device.startswith("cuda") and not torch.cuda.is_available():
            logger.warning("CUDA not available, falling back to CPU.")
            device = "cpu"

        # build_sam3_image_model accepts "cuda" or "cpu"; downstream pieces
        # treat "cuda" as "default cuda device".
        self._raw_device = device
        builder_device = "cuda" if device.startswith("cuda") else device
        self.device = torch.device(device if ":" in device else builder_device)

        logger.info("Loading SAM 3 image model on %s ...", self.device)
        self.model = build_sam3_image_model(
            bpe_path=bpe_path,
            checkpoint_path=checkpoint_path,
            device=builder_device,
            eval_mode=True,
            enable_inst_interactivity=True,
        )
        # If the user explicitly asked for cuda:N, move there.
        if ":" in device:
            self.model = self.model.to(self.device)

        processor_device = builder_device if ":" not in device else device
        self.processor = Sam3Processor(
            self.model,
            device=processor_device,
            confidence_threshold=confidence_threshold,
        )

        self.confidence_threshold = confidence_threshold
        self._max_cached_states = int(max_cached_states)
        # path -> inference_state (LRU)
        self._state_cache: "OrderedDict[str, Dict[str, Any]]" = OrderedDict()

    # ------------------------------------------------------------------ #
    # Autocast context: the SAM 3 checkpoint contains a mix of float32
    # and bfloat16 weights, so every forward pass must run under
    # ``torch.autocast("cuda", dtype=torch.bfloat16)`` to keep activation
    # and weight dtypes consistent (the official notebooks do exactly
    # this at the process level). We scope it per-call so we don't leak
    # autocast state into other code in the same Python process.
    # ------------------------------------------------------------------ #
    def _maybe_autocast(self) -> ContextManager[None]:
        if self.device.type == "cuda":
            return torch.autocast("cuda", dtype=torch.bfloat16)
        return contextlib.nullcontext()

    # ------------------------------------------------------------------ #
    # Image embedding cache
    # ------------------------------------------------------------------ #
    def set_image(self, image_key: str, pil: Image.Image) -> Dict[str, Any]:
        """Return a cached inference_state for ``image_key``.

        ``image_key`` is typically the absolute file path. For pure
        uploads it can be a synthetic key (e.g. "__upload__"); the
        caller is responsible for invalidating before reuse.
        """
        pil = _ensure_rgb(pil)
        if image_key in self._state_cache:
            self._state_cache.move_to_end(image_key)
            return self._state_cache[image_key]

        with torch.inference_mode(), self._maybe_autocast():
            state = self.processor.set_image(pil)
        self._state_cache[image_key] = state
        while len(self._state_cache) > self._max_cached_states:
            self._state_cache.popitem(last=False)
        return state

    def invalidate(self, image_key: Optional[str] = None) -> None:
        """Drop a cached embedding (or the whole cache when ``image_key`` is None)."""
        if image_key is None:
            self._state_cache.clear()
        else:
            self._state_cache.pop(image_key, None)

    # ------------------------------------------------------------------ #
    # PCS: text + optional positive/negative box exemplars
    # ------------------------------------------------------------------ #
    @torch.inference_mode()
    def detect_pcs(
        self,
        image_key: str,
        pil: Image.Image,
        noun_phrase: str,
        pos_boxes_xyxy_pix: Sequence[Sequence[float]] = (),
        neg_boxes_xyxy_pix: Sequence[Sequence[float]] = (),
        confidence_threshold: Optional[float] = None,
    ) -> List[Dict[str, Any]]:
        """Return every instance of ``noun_phrase`` in ``pil``.

        Each result is ``{"mask": HxW bool, "box_xyxy_pix": (x1,y1,x2,y2),
        "score": float, "source": "sam3-pcs"}``.
        """
        if not noun_phrase and not pos_boxes_xyxy_pix and not neg_boxes_xyxy_pix:
            raise ValueError(
                "PCS detect requires a noun phrase, a positive box, or a negative box."
            )

        if confidence_threshold is not None:
            self.processor.set_confidence_threshold(float(confidence_threshold))

        pil = _ensure_rgb(pil)
        with self._maybe_autocast():
            state = self.set_image(image_key, pil)
            self.processor.reset_all_prompts(state)

            if noun_phrase:
                self.processor.set_text_prompt(noun_phrase, state)

            W, H = pil.size
            for box in pos_boxes_xyxy_pix:
                self.processor.add_geometric_prompt(
                    _xyxy_pix_to_cxcywh_norm(box, W, H), True, state
                )
            for box in neg_boxes_xyxy_pix:
                self.processor.add_geometric_prompt(
                    _xyxy_pix_to_cxcywh_norm(box, W, H), False, state
                )

            # If boxes were the only prompts, add_geometric_prompt has
            # already forwarded the grounding head. Same if only text
            # was provided. When neither was provided we caught it above.
            return _pcs_state_to_results(state)

    # ------------------------------------------------------------------ #
    # PVS: per-object points + optional box
    # ------------------------------------------------------------------ #
    @torch.inference_mode()
    def segment_pvs(
        self,
        image_key: str,
        pil: Image.Image,
        objects: Sequence[PvsObject],
        multimask_output: bool = True,
    ) -> List[Dict[str, Any]]:
        """Run ``predict_inst`` once per object slot.

        Each result has the same shape as the PCS results but with
        ``"source": "sam3-pvs"`` and the user-supplied ``box`` echoed
        back when present (otherwise we fall back to the mask's tight
        bbox).
        """
        if not objects:
            raise ValueError("PVS segment requires at least one object slot.")

        pil = _ensure_rgb(pil)
        with self._maybe_autocast():
            state = self.set_image(image_key, pil)
            return self._segment_pvs_with_state(state, objects, multimask_output)

    @torch.inference_mode()
    def _segment_pvs_with_state(
        self,
        state: Dict[str, Any],
        objects: Sequence[PvsObject],
        multimask_output: bool,
    ) -> List[Dict[str, Any]]:
        results: List[Dict[str, Any]] = []
        for slot_idx, obj in enumerate(objects):
            pos_pts = list(obj.get("pos_points", []))
            neg_pts = list(obj.get("neg_points", []))
            box = obj.get("box", None)
            if not pos_pts and not neg_pts and box is None:
                # Empty slot, skip silently.
                continue

            point_coords: Optional[np.ndarray] = None
            point_labels: Optional[np.ndarray] = None
            if pos_pts or neg_pts:
                pts = np.asarray(pos_pts + neg_pts, dtype=np.float32)
                lbl = np.asarray(
                    [1] * len(pos_pts) + [0] * len(neg_pts), dtype=np.int64
                )
                point_coords, point_labels = pts, lbl

            box_arr: Optional[np.ndarray] = None
            if box is not None:
                bx1, by1, bx2, by2 = box
                if bx2 < bx1:
                    bx1, bx2 = bx2, bx1
                if by2 < by1:
                    by1, by2 = by2, by1
                box_arr = np.asarray([bx1, by1, bx2, by2], dtype=np.float32)

            masks, scores, _logits = self.model.predict_inst(
                state,
                point_coords=point_coords,
                point_labels=point_labels,
                box=box_arr,
                multimask_output=multimask_output,
            )

            # masks shape is (num_masks, H, W); pick the highest-scoring
            # one when multimask_output=True, else there's a single mask.
            if scores.size == 0:
                continue
            best = int(np.argmax(scores))
            mask = masks[best].astype(bool)
            if box_arr is not None:
                box_xyxy_pix: Tuple[float, float, float, float] = (
                    float(box_arr[0]),
                    float(box_arr[1]),
                    float(box_arr[2]),
                    float(box_arr[3]),
                )
            else:
                box_xyxy_pix = _mask_to_xyxy(mask)
            results.append(
                {
                    "mask": mask,
                    "box_xyxy_pix": box_xyxy_pix,
                    "score": float(scores[best]),
                    "source": "sam3-pvs",
                    "slot_index": slot_idx,
                }
            )

        return results

    def set_confidence_threshold(self, threshold: float) -> None:
        self.confidence_threshold = float(threshold)
        self.processor.set_confidence_threshold(float(threshold))


# ---------------------------------------------------------------------- #
# State-to-results conversion for the PCS path
# ---------------------------------------------------------------------- #
def _pcs_state_to_results(state: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Pull ``masks``, ``boxes``, ``scores`` out of a forwarded state."""
    if "masks" not in state:
        return []

    masks_t: torch.Tensor = state["masks"]
    boxes_t: torch.Tensor = state["boxes"]
    scores_t: torch.Tensor = state["scores"]

    # masks_t is (N, 1, H, W) boolean at original resolution.
    if masks_t.ndim == 4:
        masks_np = masks_t.squeeze(1).detach().cpu().numpy().astype(bool)
    else:
        masks_np = masks_t.detach().cpu().numpy().astype(bool)
    # boxes / scores can be bfloat16 (the model is typically run under
    # autocast). NumPy cannot decode bfloat16, so cast to float32 first.
    boxes_np = boxes_t.detach().float().cpu().numpy().astype(float)
    scores_np = scores_t.detach().float().cpu().numpy().astype(float)

    results: List[Dict[str, Any]] = []
    for i in range(masks_np.shape[0]):
        results.append(
            {
                "mask": masks_np[i],
                "box_xyxy_pix": (
                    float(boxes_np[i, 0]),
                    float(boxes_np[i, 1]),
                    float(boxes_np[i, 2]),
                    float(boxes_np[i, 3]),
                ),
                "score": float(scores_np[i]),
                "source": "sam3-pcs",
                "slot_index": i,
            }
        )
    return results
