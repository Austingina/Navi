# Copyright (c) Meta Platforms, Inc. and affiliates. All Rights Reserved
"""Gradio-based SAM 3 browser annotation tool.

Run::

    python -m sam3.labeler.app \
        --output_dir /tmp/sam3-labels \
        --device cuda:0

Open the printed URL in a browser. The page supports two modes:

* **Find all (PCS)** - type a noun phrase, optionally drop positive /
  negative box exemplars, hit **Detect** to get every matching instance.
* **Click and segment (PVS)** - drop positive / negative point clicks
  on each object, optionally add a single box around it (two clicks for
  the box's corners), click **Add object** to commit the slot and start
  a new one. **Detect** runs SAM 3's ``predict_inst`` once per
  committed slot (plus the current uncommitted one if it has prompts).

Prompt capture uses native Gradio components only (no
``gradio-image-prompter`` dependency), so the app works with the latest
Gradio + ``huggingface_hub`` and stays compatible with ``transformers``
in the same environment. The trade-off vs the prompter is that boxes
are placed with **two clicks** (top-left then bottom-right) instead of
a click-drag.

**Save** writes one self-contained shard at
``<output_dir>/pairs/<stem>__<np>.json`` plus the matching
``vis/<stem>__<np>.jpg`` and ``prompts/<stem>__<np>.json`` sidecars.
**Mark as negative** writes a shard whose ``annotations`` list is
empty, which is how SAM 3 represents an NP that is absent from the
image. The canonical ``<output_dir>/annotations.json`` consumed by
SAM 3 eval / training is a derived artifact; build it from the shards
with ``python -m sam3.labeler.compact <output_dir>`` whenever you are
ready. Deleting a single mislabelled pair is as simple as removing the
matching ``pairs/<stem>__<np>.json`` (plus its sidecars) and re-running
``compact``.
"""

from __future__ import annotations

import argparse
import copy
import logging
import os
import sys
import threading
import traceback
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from PIL import Image

import gradio as gr


def _patch_gradio_client_bool_schema_bug() -> None:
    """Make ``gradio_client.utils.get_type`` accept boolean JSON schemas.

    See https://github.com/gradio-app/gradio/issues/11722 ,
    https://github.com/gradio-app/gradio/issues/11084 ,
    https://github.com/gradio-app/gradio/issues/11116 .

    JSON Schema allows ``additionalProperties: true`` (and other places
    where a sub-schema can be a literal ``True``/``False``). Several
    ``gradio_client`` builds assume those sub-schemas are always dicts,
    which crashes with ``TypeError: argument of type 'bool' is not
    iterable`` while serving the main page. We coerce bool schemas to
    ``"Any"`` here so the schema walker keeps going.
    """
    try:
        from gradio_client import utils as _gc_utils
    except ImportError:
        return

    orig_get_type = getattr(_gc_utils, "get_type", None)
    orig_walk = getattr(_gc_utils, "_json_schema_to_python_type", None)
    if orig_get_type is None or orig_walk is None:
        return

    def get_type_safe(schema):
        if isinstance(schema, bool):
            return "Any"
        if not isinstance(schema, dict):
            return "Any"
        return orig_get_type(schema)

    def walk_safe(schema, defs=None):
        if isinstance(schema, bool):
            return "Any"
        if not isinstance(schema, dict):
            return "Any"
        return orig_walk(schema, defs)

    _gc_utils.get_type = get_type_safe
    _gc_utils._json_schema_to_python_type = walk_safe


_patch_gradio_client_bool_schema_bug()


from sam3.labeler.annotations import AnnotationStore
from sam3.labeler.inference import Sam3Annotator
from sam3.labeler.visualization import (
    render_existing_detections,
    render_prompts,
    render_results,
    save_rgb_jpeg,
)


logger = logging.getLogger(__name__)


_SUPPORTED_IMAGE_EXT = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


# ---------------------------------------------------------------------- #
# Lazy model singleton
# ---------------------------------------------------------------------- #
_annotator_lock = threading.Lock()
_annotator_instance: Optional[Sam3Annotator] = None
_annotator_kwargs: Dict[str, Any] = {}


def _get_annotator(
    bpe_path: Optional[str],
    checkpoint_path: Optional[str],
    device: str,
    confidence_threshold: float,
) -> Sam3Annotator:
    """Build the SAM 3 model once and share it across requests."""
    global _annotator_instance, _annotator_kwargs
    with _annotator_lock:
        new_kwargs = {
            "bpe_path": bpe_path,
            "checkpoint_path": checkpoint_path,
            "device": device,
        }
        if (_annotator_instance is None) or (new_kwargs != _annotator_kwargs):
            _annotator_instance = Sam3Annotator(
                bpe_path=bpe_path,
                checkpoint_path=checkpoint_path,
                device=device,
                confidence_threshold=confidence_threshold,
            )
            _annotator_kwargs = new_kwargs
        else:
            _annotator_instance.set_confidence_threshold(confidence_threshold)
    return _annotator_instance


# ---------------------------------------------------------------------- #
# Session state
# ---------------------------------------------------------------------- #
def _empty_pvs_slot() -> Dict[str, Any]:
    return {"pos_points": [], "neg_points": [], "box": None}


def _new_state() -> Dict[str, Any]:
    return {
        "mode": "pcs",
        "image_key": None,
        "image_path": None,
        "image_size": (0, 0),  # (W, H)
        # PCS prompts: lists of [x1, y1, x2, y2] in pixel coords.
        "pcs_prompts": {"pos_boxes": [], "neg_boxes": []},
        # PVS prompts: a list of committed object slots + the current uncommitted slot.
        "pvs_prompts": {"committed": [], "current": _empty_pvs_slot()},
        # 2-click box state machine: when the user is mid-box-drawing, this
        # holds the first corner as (x, y). None otherwise. Used by both
        # PCS/PVS prompt placement and the edit-mode batch select tool.
        "box_in_progress": None,
        "last_detection": [],
        "saved_nps": [],
        "directory": {"files": [], "index": 0},
        # Edit mode: detections loaded from <output_dir>/pairs/<stem>__*.json
        # for the current image. Keyed by noun phrase; each value is a list
        # of {ann_id, box_xyxy_pix, mask, score, source}.
        "loaded_detections": {},
        # (np_text, ann_id) entries the user has clicked / box-selected
        # but not yet marked for deletion. Highlighted in red on the
        # canvas; cleared by "Clear selection" or by switching modes.
        "selected_for_delete": [],
        # (np_text, ann_id) entries the user has staged for deletion via
        # "Delete selected". Hidden from the canvas to preview the
        # post-save state, but not yet written to disk. Committed by
        # "Save edits to disk"; reverted by "Discard edits" or "Reload
        # from disk".
        "pending_delete": [],
    }


def _state_for_new_image(state: Dict[str, Any]) -> Dict[str, Any]:
    state = copy.deepcopy(state)
    state["pcs_prompts"] = {"pos_boxes": [], "neg_boxes": []}
    state["pvs_prompts"] = {"committed": [], "current": _empty_pvs_slot()}
    state["box_in_progress"] = None
    state["last_detection"] = []
    state["saved_nps"] = []
    state["loaded_detections"] = {}
    state["selected_for_delete"] = []
    state["pending_delete"] = []
    return state


def _list_images(directory: str) -> List[str]:
    if not directory or not os.path.isdir(directory):
        return []
    out: List[str] = []
    for name in sorted(os.listdir(directory)):
        if os.path.splitext(name)[1].lower() in _SUPPORTED_IMAGE_EXT:
            out.append(os.path.join(directory, name))
    return out


def _load_pil(path: str) -> Image.Image:
    pil = Image.open(path)
    pil.load()
    if pil.mode != "RGB":
        pil = pil.convert("RGB")
    return pil


def _load_existing_for_image(
    output_dir: str, image_path: str,
) -> Dict[str, List[Dict[str, Any]]]:
    """Scan ``<output_dir>/pairs/<stem>__*.json`` and return per-NP detections.

    Empty dict on any error (unset output_dir, missing pairs/, malformed
    shards). Negative shards (NPs with empty annotations) are still
    surfaced as an empty detection list under their noun phrase so the
    saved-NP badge in the status bar stays in sync with disk.
    """
    if not output_dir or not image_path:
        return {}
    if not os.path.isdir(output_dir):
        return {}
    try:
        store = AnnotationStore(output_dir, fsync=False)
        loaded = store.load_pairs_for_image(image_path)
    except Exception:
        logger.warning(
            "Failed to load existing detections from %s", output_dir,
            exc_info=True,
        )
        return {}
    out: Dict[str, List[Dict[str, Any]]] = {}
    for np_text, payload in loaded.items():
        out[np_text] = list(payload.get("detections") or [])
    return out


def _refresh_loaded_detections(
    state: Dict[str, Any], output_dir: str,
) -> Dict[str, Any]:
    """Re-read ``<output_dir>/pairs/`` and update state in-place.

    Clears both ``selected_for_delete`` and ``pending_delete`` because
    after a reload the in-memory deletion queue may refer to ann_ids
    that no longer exist on disk (e.g., the user deleted them already
    or re-ran inference).
    """
    image_path = state.get("image_path")
    if image_path is None:
        state["loaded_detections"] = {}
        state["selected_for_delete"] = []
        state["pending_delete"] = []
        return state
    detections = _load_existing_for_image(output_dir, image_path)
    state["loaded_detections"] = detections
    saved_nps = list(detections.keys())
    if saved_nps:
        state["saved_nps"] = saved_nps
    state["selected_for_delete"] = []
    state["pending_delete"] = []
    return state


# ---------------------------------------------------------------------- #
# Overlay rendering
# ---------------------------------------------------------------------- #
def _render_overlay(state: Dict[str, Any]) -> np.ndarray:
    """Render the prompts (or edit) overlay for the input image panel."""
    if state.get("image_path") is None:
        return np.zeros((1, 1, 3), dtype=np.uint8)
    try:
        pil = _load_pil(state["image_path"])
    except OSError:
        return np.zeros((1, 1, 3), dtype=np.uint8)

    mode = state.get("mode", "pcs")
    if mode == "edit":
        overlay = render_existing_detections(
            pil,
            _visible_detections(state),
            state.get("selected_for_delete") or [],
        )
    else:
        # In PCS/PVS, draw existing detections (minus pending-deletes) as
        # a faint cyan background so the user can place new prompts with
        # full awareness of what is already detected. The model never
        # sees this layer; it is purely visual scaffolding.
        visible = _visible_detections(state)
        if visible:
            base = render_existing_detections(
                pil,
                visible,
                [],
                alpha=0.12,
                outline_thickness=1,
                draw_label=False,
            )
        else:
            base = pil
        if mode == "pcs":
            overlay = render_prompts(
                base,
                mode="pcs",
                pcs_pos_boxes=state["pcs_prompts"]["pos_boxes"],
                pcs_neg_boxes=state["pcs_prompts"]["neg_boxes"],
            )
        else:
            overlay = render_prompts(
                base,
                mode="pvs",
                pvs_committed=state["pvs_prompts"]["committed"],
                pvs_current=state["pvs_prompts"]["current"],
            )

    # Mark the first corner of a half-drawn box so the user knows where
    # to aim the next click.
    if state.get("box_in_progress") is not None:
        x, y = state["box_in_progress"]
        import cv2

        cx, cy = int(round(x)), int(round(y))
        cv2.drawMarker(
            overlay,
            (cx, cy),
            color=(255, 200, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=20,
            thickness=2,
            line_type=cv2.LINE_AA,
        )
    return overlay


def _status_text(state: Dict[str, Any], extra: str = "") -> str:
    parts: List[str] = []
    if state.get("image_path"):
        path = state["image_path"]
        W, H = state.get("image_size", (0, 0))
        files = state.get("directory", {}).get("files", [])
        if files:
            idx = state["directory"]["index"]
            parts.append(f"[{idx + 1}/{len(files)}] {os.path.basename(path)} ({W}x{H})")
        else:
            parts.append(f"{os.path.basename(path)} ({W}x{H})")
    if state.get("box_in_progress") is not None:
        parts.append("Box tool: place 2nd corner.")
    if state.get("mode") == "edit":
        loaded = state.get("loaded_detections") or {}
        n_total = sum(len(v) for v in loaded.values())
        n_sel = len(state.get("selected_for_delete") or [])
        n_pending = len(state.get("pending_delete") or [])
        edit_parts = [
            f"Edit mode: {n_total} loaded across {len(loaded)} NP(s); "
            f"{n_sel} selected"
        ]
        if n_pending:
            edit_parts.append(f"{n_pending} pending delete (unsaved)")
        parts.append("; ".join(edit_parts) + ".")
    elif state.get("saved_nps"):
        parts.append("Saved NPs: " + ", ".join(state["saved_nps"]))
    if extra:
        parts.append(extra)
    return " | ".join(parts) if parts else "Ready."


# ---------------------------------------------------------------------- #
# NMS for PCS detections
#
# Mirrors the two-stage pipeline used by ``sam3/export_hf/trt_infer.py``:
#
#   1. ``_filter_intra_nms_results`` -- greedy box-IoU NMS, removes
#      near-duplicate detections of the same noun phrase.
#   2. ``_filter_cross_nms_results`` -- greedy mask-IoU + box-proximity
#      NMS, removes overlapping detections whose centers are also close.
#      Also drops tiny masks below ``min_mask_size_ratio`` of the image.
#
# The TRT version runs on GPU tensors; here results are already on CPU
# as numpy arrays (see ``_pcs_state_to_results``), so this implementation
# is fully numpy-based.
# ---------------------------------------------------------------------- #
def _box_iou_numpy(boxes: np.ndarray, query: np.ndarray) -> np.ndarray:
    """Per-row IoU between ``boxes`` [N, 4] xyxy and ``query`` [4] xyxy."""
    x1 = np.maximum(boxes[:, 0], query[0])
    y1 = np.maximum(boxes[:, 1], query[1])
    x2 = np.minimum(boxes[:, 2], query[2])
    y2 = np.minimum(boxes[:, 3], query[3])
    inter = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    area_a = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    area_b = (query[2] - query[0]) * (query[3] - query[1])
    union = area_a + area_b - inter
    return inter / np.maximum(union, 1e-6)


def _filter_intra_nms_results(
    results: List[Dict[str, Any]], iou_threshold: float,
) -> List[Dict[str, Any]]:
    """Greedy box-IoU NMS over a score-sorted result list."""
    if not results or iou_threshold <= 0:
        return results
    sorted_results = sorted(results, key=lambda r: r["score"], reverse=True)
    boxes = np.asarray(
        [r["box_xyxy_pix"] for r in sorted_results], dtype=np.float32
    )

    kept: List[Dict[str, Any]] = []
    kept_idx: List[int] = []
    for i, r in enumerate(sorted_results):
        if kept_idx:
            ious = _box_iou_numpy(boxes[kept_idx], boxes[i])
            if ious.max() > iou_threshold:
                continue
        kept.append(r)
        kept_idx.append(i)
    return kept


def _filter_cross_nms_results(
    results: List[Dict[str, Any]],
    image_hw: Tuple[int, int],
    iou_threshold: float,
    proximity_threshold: float,
    min_mask_size_ratio: float,
    keep_larger: bool = True,
) -> List[Dict[str, Any]]:
    """Greedy mask-IoU + box-center-proximity NMS over a result list.

    Two detections collide when either their masks overlap more than
    ``iou_threshold`` or their box centers are within
    ``proximity_threshold`` pixels of each other. In the overlap case
    the higher-scoring one wins; in the proximity case the box with
    the larger (or smaller, depending on ``keep_larger``) area wins.
    Detections whose masks cover less than ``min_mask_size_ratio`` of
    the image area are dropped up front.
    """
    if not results or iou_threshold <= 0:
        return results
    sorted_results = sorted(results, key=lambda r: r["score"], reverse=True)
    K = len(sorted_results)

    boxes = np.asarray(
        [r["box_xyxy_pix"] for r in sorted_results], dtype=np.float32
    )
    box_cx = (boxes[:, 0] + boxes[:, 2]) * 0.5
    box_cy = (boxes[:, 1] + boxes[:, 3]) * 0.5
    box_areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])

    # Mask-IoU via boolean → float32 matmul ([K, H*W] @ [H*W] → [K]).
    masks_flat = np.stack(
        [np.asarray(r["mask"], dtype=bool).reshape(-1).astype(np.float32)
         for r in sorted_results]
    )
    mask_areas = masks_flat.sum(axis=1)

    total_pixels = float(image_hw[0]) * float(image_hw[1])
    min_mask_px = total_pixels * float(min_mask_size_ratio)

    kept_indices: List[int] = []
    for i in range(K):
        if mask_areas[i] < min_mask_px:
            continue
        if not kept_indices:
            kept_indices.append(i)
            continue

        ki = np.asarray(kept_indices, dtype=np.int64)
        inter = masks_flat[ki] @ masks_flat[i]
        unions = mask_areas[i] + mask_areas[ki] - inter
        mask_ious = inter / np.maximum(unions, 1.0)
        dists = np.sqrt(
            (box_cx[i] - box_cx[ki]) ** 2 + (box_cy[i] - box_cy[ki]) ** 2
        )

        keep = True
        to_remove: List[int] = []
        cur_area = float(box_areas[i])
        for j_local, j_global in enumerate(kept_indices):
            if mask_ious[j_local] > iou_threshold:
                keep = False
                break
            if dists[j_local] < proximity_threshold:
                k_area = float(box_areas[j_global])
                if (keep_larger and cur_area > k_area) or (
                    not keep_larger and cur_area < k_area
                ):
                    to_remove.append(j_local)
                else:
                    keep = False
                    break
        if keep:
            for idx in sorted(to_remove, reverse=True):
                kept_indices.pop(idx)
            kept_indices.append(i)

    return [sorted_results[i] for i in kept_indices]


def _apply_detection_nms(
    results: List[Dict[str, Any]],
    image_hw: Tuple[int, int],
    intra_iou_threshold: float,
    cross_iou_threshold: float,
    proximity_threshold: float,
    min_mask_size_ratio: float,
) -> Tuple[List[Dict[str, Any]], int, int]:
    """Run the two-stage NMS pipeline.

    Returns ``(kept, intra_removed, cross_removed)``.
    """
    intra_removed = 0
    if intra_iou_threshold > 0 and results:
        before = len(results)
        results = _filter_intra_nms_results(results, intra_iou_threshold)
        intra_removed = before - len(results)

    cross_removed = 0
    if cross_iou_threshold > 0 and results:
        before = len(results)
        results = _filter_cross_nms_results(
            results,
            image_hw=image_hw,
            iou_threshold=cross_iou_threshold,
            proximity_threshold=proximity_threshold,
            min_mask_size_ratio=min_mask_size_ratio,
        )
        cross_removed = before - len(results)

    return results, intra_removed, cross_removed


# ---------------------------------------------------------------------- #
# Click dispatch
# ---------------------------------------------------------------------- #
def _on_image_click(
    image: Any,
    sel_data: gr.SelectData,
    mode: str,
    polarity: str,
    tool: str,
    state: Dict[str, Any],
):
    """Translate one image click into a point, box-corner, or edit-select.

    ``sel_data.index`` is ``(x, y)`` in pixel coordinates of the
    underlying image, regardless of the on-screen display size.
    """
    if state is None:
        state = _new_state()
    state = copy.deepcopy(state)

    if state.get("image_path") is None:
        return state, _render_overlay(state), _status_text(state, "Load an image first.")
    if sel_data is None or sel_data.index is None:
        return state, _render_overlay(state), _status_text(state)

    x, y = sel_data.index[0], sel_data.index[1]
    state["mode"] = mode  # latest UI value wins (handles initial-render races)
    is_positive = (polarity or "Positive").lower().startswith("pos")
    tool_lc = (tool or "Point").lower()

    msg = ""

    if mode == "edit":
        return _handle_edit_click(state, float(x), float(y), tool_lc)

    if tool_lc == "box":
        first = state.get("box_in_progress")
        if first is None:
            state["box_in_progress"] = [float(x), float(y)]
            msg = "Box corner 1 set. Click again to place the opposite corner."
        else:
            x1, y1 = first
            x2, y2 = float(x), float(y)
            if x2 < x1:
                x1, x2 = x2, x1
            if y2 < y1:
                y1, y2 = y2, y1
            if x2 - x1 < 1 or y2 - y1 < 1:
                # Degenerate, treat the second click as a fresh corner.
                state["box_in_progress"] = [float(x), float(y)]
                msg = "Box was zero-sized; using this click as a new corner 1."
            else:
                box = [x1, y1, x2, y2]
                state["box_in_progress"] = None
                if mode == "pcs":
                    bucket = "pos_boxes" if is_positive else "neg_boxes"
                    state["pcs_prompts"][bucket].append(box)
                else:
                    state["pvs_prompts"]["current"]["box"] = box
                msg = f"Box added ({'positive' if is_positive else 'negative'})."
    else:
        # Point tool. Clicks are points; any half-drawn box is silently
        # discarded so the user is never left with a stuck state.
        state["box_in_progress"] = None
        if mode == "pcs":
            # PCS doesn't accept points; turn a click into a tiny box.
            bucket = "pos_boxes" if is_positive else "neg_boxes"
            state["pcs_prompts"][bucket].append(
                [float(x) - 12.0, float(y) - 12.0, float(x) + 12.0, float(y) + 12.0]
            )
            msg = (
                "PCS: point click converted to a small box. "
                "Use the Box tool for precise box exemplars."
            )
        else:
            slot = state["pvs_prompts"]["current"]
            bucket = "pos_points" if is_positive else "neg_points"
            slot[bucket].append([float(x), float(y)])
            msg = f"Point added ({'positive' if is_positive else 'negative'})."

    return state, _render_overlay(state), _status_text(state, msg)


def _set_mode(mode: str, state: Dict[str, Any]):
    state = copy.deepcopy(state or _new_state())
    state["mode"] = mode
    # Any half-drawn box is mode-agnostic, but the user typically wants a
    # clean slate when toggling modes.
    state["box_in_progress"] = None
    if mode != "edit":
        # Drop any pending edit-mode selection so it doesn't silently
        # affect a later switch back to edit mode.
        state["selected_for_delete"] = []
    is_pcs = mode == "pcs"
    is_pvs = mode == "pvs"
    is_edit = mode == "edit"
    is_prompt = is_pcs or is_pvs
    return (
        state,
        _render_overlay(state),
        gr.update(visible=is_pvs),       # reset_all_btn
        gr.update(visible=is_pvs),       # add_obj_btn
        gr.update(visible=is_pcs),       # confidence slider
        gr.update(visible=is_pvs),       # multimask checkbox
        gr.update(visible=is_pcs),       # nms accordion
        gr.update(visible=is_prompt),    # np_in
        gr.update(visible=is_prompt),    # detect_btn
        gr.update(visible=is_prompt),    # save_btn
        gr.update(visible=is_prompt),    # mark_neg_btn
        gr.update(visible=is_prompt),    # reset_btn
        gr.update(visible=is_edit),      # delete_btn
        gr.update(visible=is_edit),      # clear_sel_btn
        gr.update(visible=is_edit),      # save_edits_btn
        gr.update(visible=is_edit),      # discard_edits_btn
        gr.update(visible=is_edit),      # reload_existing_btn
        _status_text(state),
    )


# ---------------------------------------------------------------------- #
# Reset / Add object
# ---------------------------------------------------------------------- #
def _handle_edit_click(
    state: Dict[str, Any], x: float, y: float, tool_lc: str,
):
    """Edit-mode click: toggle selection (Point) or batch-select (Box).

    Point tool: pick the smallest existing mask whose pixel at ``(x, y)``
    is True; toggle ``(np_text, ann_id)`` in ``selected_for_delete``.
    Tie-break by score (higher wins).

    Box tool: 2-click rectangle, identical machinery to the prompt-mode
    box. On the second click, every loaded detection whose bbox center
    falls inside the rectangle is added to the selection (additive).
    """
    if tool_lc == "box":
        first = state.get("box_in_progress")
        if first is None:
            state["box_in_progress"] = [float(x), float(y)]
            msg = "Edit-box corner 1 set. Click again to mark the opposite corner."
        else:
            x1, y1 = first
            x2, y2 = float(x), float(y)
            if x2 < x1:
                x1, x2 = x2, x1
            if y2 < y1:
                y1, y2 = y2, y1
            state["box_in_progress"] = None
            if x2 - x1 < 1 or y2 - y1 < 1:
                state["box_in_progress"] = [float(x), float(y)]
                msg = "Edit-box was zero-sized; using this click as a new corner 1."
            else:
                added = _select_in_rect(state, (x1, y1, x2, y2))
                msg = (
                    f"Edit-box: added {added} detection(s) to selection."
                    if added
                    else "Edit-box: no existing detection's bbox center fell inside."
                )
        return state, _render_overlay(state), _status_text(state, msg)

    state["box_in_progress"] = None
    hit = _pick_smallest_at(state, x, y)
    if hit is None:
        visible = _visible_detections(state)
        n_total = sum(len(v) for v in visible.values())
        if n_total == 0:
            extra = (
                "No detections loaded for this image. Check the Output "
                "directory below, or click 'Reload from disk'."
            )
        else:
            extra = (
                f"No detection under or near that click "
                f"(searched {n_total} visible detection(s))."
            )
        return state, _render_overlay(state), _status_text(state, extra)
    np_text, ann_id = hit
    sel = list(state.get("selected_for_delete") or [])
    key = (np_text, ann_id)
    if key in sel:
        sel.remove(key)
        msg = f"Unselected {np_text!r} #{ann_id}."
    else:
        sel.append(key)
        msg = f"Selected {np_text!r} #{ann_id}."
    state["selected_for_delete"] = sel
    return state, _render_overlay(state), _status_text(state, msg)


# Pixel tolerance for the "near-bbox" fallback in the edit-mode picker.
# A click within this many pixels of any side of a bbox still counts as
# a hit. Generous enough to cover label-tag clicks (the label is drawn
# above the box top edge by ~14 px), bbox-outline clicks, etc.
_EDIT_PICK_TOL_PX: float = 16.0


def _visible_detections(
    state: Dict[str, Any],
) -> Dict[str, List[Dict[str, Any]]]:
    """``loaded_detections`` minus anything in ``pending_delete``.

    Pending-delete items are queued for removal but not yet written to
    disk; the labeler should hide them from the canvas, the click
    picker, and the box-select tool so the user perceives them as
    already-gone (and can undo by pressing **Discard edits**).
    """
    pending = {
        (str(np_text), int(ann_id))
        for np_text, ann_id in (state.get("pending_delete") or [])
    }
    out: Dict[str, List[Dict[str, Any]]] = {}
    for np_text, dets in (state.get("loaded_detections") or {}).items():
        kept = [
            d
            for d in dets
            if (np_text, int(d.get("ann_id", 0) or 0)) not in pending
        ]
        if kept:
            out[np_text] = kept
    return out


def _pick_smallest_at(
    state: Dict[str, Any], x: float, y: float,
) -> Optional[Tuple[str, int]]:
    """Find the smallest existing detection "under" pixel ``(x, y)``.

    Lookup is forgiving on purpose: clicks frequently land on the label
    tag (drawn above the bbox top edge) or on the bbox outline itself,
    which would otherwise miss the underlying mask. Search order:

    1. Detections whose **mask** contains the click pixel.
       Smaller mask area wins; ties broken by higher score, then
       smaller bbox area.
    2. Detections whose **bbox** contains the click. Smaller bbox wins.
    3. Detections whose bbox is within ``_EDIT_PICK_TOL_PX`` pixels of
       the click. Closest box wins. Catches label-tag and edge clicks.

    Pending-delete detections are skipped so they cannot be re-selected
    while queued for deletion.
    """
    cx, cy = int(round(x)), int(round(y))
    mask_hits: List[Tuple[float, float, float, str, int]] = []
    box_hits: List[Tuple[float, float, str, int]] = []
    near_hits: List[Tuple[float, float, str, int]] = []
    for np_text, dets in _visible_detections(state).items():
        for det in dets:
            ann_id = int(det.get("ann_id", 0) or 0)
            box = det.get("box_xyxy_pix") or [0.0, 0.0, 0.0, 0.0]
            x1, y1, x2, y2 = (float(v) for v in box[:4])
            score = float(det.get("score", 0.0) or 0.0)
            box_area = max(1.0, (x2 - x1) * (y2 - y1))

            in_mask = False
            mask_area = box_area
            mask = det.get("mask")
            if mask is not None:
                arr = np.asarray(mask)
                if arr.ndim == 2 and 0 <= cy < arr.shape[0] and 0 <= cx < arr.shape[1]:
                    if bool(arr[cy, cx]):
                        in_mask = True
                        mask_area = float(arr.sum())

            in_box = (x1 <= x <= x2 and y1 <= y <= y2)
            if in_mask:
                mask_hits.append((mask_area, -score, box_area, np_text, ann_id))
            elif in_box:
                box_hits.append((box_area, -score, np_text, ann_id))
            else:
                dx = max(x1 - x, x - x2, 0.0)
                dy = max(y1 - y, y - y2, 0.0)
                dist = float((dx * dx + dy * dy) ** 0.5)
                if dist <= _EDIT_PICK_TOL_PX:
                    near_hits.append((dist, box_area, np_text, ann_id))

    if mask_hits:
        mask_hits.sort()
        return (mask_hits[0][3], mask_hits[0][4])
    if box_hits:
        box_hits.sort()
        return (box_hits[0][2], box_hits[0][3])
    if near_hits:
        near_hits.sort()
        return (near_hits[0][2], near_hits[0][3])
    return None


def _select_in_rect(
    state: Dict[str, Any], rect_xyxy: Tuple[float, float, float, float],
) -> int:
    """Add to ``selected_for_delete`` every visible detection whose bbox
    center falls inside ``rect_xyxy``. Returns the number added."""
    rx1, ry1, rx2, ry2 = rect_xyxy
    sel = list(state.get("selected_for_delete") or [])
    sel_set = set(sel)
    added = 0
    for np_text, dets in _visible_detections(state).items():
        for det in dets:
            box = det.get("box_xyxy_pix") or [0, 0, 0, 0]
            x1, y1, x2, y2 = box[:4]
            cx = (x1 + x2) * 0.5
            cy = (y1 + y2) * 0.5
            if rx1 <= cx <= rx2 and ry1 <= cy <= ry2:
                key = (np_text, int(det.get("ann_id", 0) or 0))
                if key not in sel_set:
                    sel.append(key)
                    sel_set.add(key)
                    added += 1
    state["selected_for_delete"] = sel
    return added


def _on_clear_selection(state: Dict[str, Any]):
    state = copy.deepcopy(state or _new_state())
    state["selected_for_delete"] = []
    state["box_in_progress"] = None
    return state, _render_overlay(state), _status_text(state, "Selection cleared.")


def _on_reload_existing(state: Dict[str, Any], output_dir: str):
    state = copy.deepcopy(state or _new_state())
    if state.get("image_path") is None:
        return state, _render_overlay(state), _status_text(state, "Load an image first.")
    if not output_dir:
        return state, _render_overlay(state), _status_text(
            state, "Set an output directory before reloading existing detections.",
        )
    state = _refresh_loaded_detections(state, output_dir)
    n_total = sum(len(v) for v in state["loaded_detections"].values())
    return (
        state,
        _render_overlay(state),
        _status_text(state, f"Reloaded {n_total} detection(s) from disk."),
    )


def _on_delete_selected(state: Dict[str, Any]):
    """Stage the currently-selected detections for deletion.

    Moves ``selected_for_delete`` into ``pending_delete`` (deduplicated)
    and clears the selection. **Does not touch disk** -- the user must
    click "Save edits to disk" to commit. "Discard edits" or "Reload
    from disk" abandons the queue without writing.
    """
    state = copy.deepcopy(state or _new_state())
    if state.get("image_path") is None:
        return state, _render_overlay(state), _status_text(state, "Load an image first.")

    selected: List[Tuple[str, int]] = list(state.get("selected_for_delete") or [])
    if not selected:
        return state, _render_overlay(state), _status_text(state, "Nothing selected.")

    pending = list(state.get("pending_delete") or [])
    pending_set = {(str(np_text), int(ann_id)) for np_text, ann_id in pending}
    added = 0
    for np_text, ann_id in selected:
        key = (str(np_text), int(ann_id))
        if key not in pending_set:
            pending.append(key)
            pending_set.add(key)
            added += 1

    state["pending_delete"] = pending
    state["selected_for_delete"] = []
    state["box_in_progress"] = None

    n_pending = len(pending)
    msg = (
        f"Marked {added} detection(s) for delete "
        f"({n_pending} pending total). Click 'Save edits to disk' to "
        f"commit, or 'Discard edits' to revert."
    )
    return state, _render_overlay(state), _status_text(state, msg)


def _on_save_edits(state: Dict[str, Any], output_dir: str):
    """Commit ``pending_delete`` to disk via ``write_pair_overrides``.

    For each affected noun phrase, rewrites the shard at
    ``<output_dir>/pairs/<stem>__<np>.json`` keeping only the
    annotations whose local id is *not* pending. Then reloads from disk
    so the in-memory state matches what compact will read next.
    """
    state = copy.deepcopy(state or _new_state())
    if state.get("image_path") is None:
        return state, _render_overlay(state), _status_text(state, "Load an image first.")
    if not output_dir:
        return state, _render_overlay(state), _status_text(
            state, "Set an output directory before saving edits.",
        )

    pending: List[Tuple[str, int]] = list(state.get("pending_delete") or [])
    if not pending:
        return state, _render_overlay(state), _status_text(
            state, "No pending edits to save.",
        )

    by_np: Dict[str, set] = {}
    for np_text, ann_id in pending:
        by_np.setdefault(str(np_text), set()).add(int(ann_id))

    try:
        store = AnnotationStore(output_dir, fsync=False)
    except Exception:
        return (
            state,
            _render_overlay(state),
            _status_text(state, f"Could not open output dir: {traceback.format_exc()}"),
        )

    image_path = state["image_path"]
    loaded = state.get("loaded_detections") or {}
    n_deleted = 0
    n_concepts = 0
    for np_text, drop_ids in by_np.items():
        all_ids = {int(d.get("ann_id", 0) or 0) for d in loaded.get(np_text, [])}
        kept = sorted(all_ids - drop_ids)
        try:
            store.write_pair_overrides(image_path, np_text, kept)
        except FileNotFoundError:
            logger.warning(
                "Skipping delete for missing shard (np=%r, image=%s)",
                np_text, image_path,
            )
            continue
        except Exception:
            return (
                state,
                _render_overlay(state),
                _status_text(state, f"Save edits failed for {np_text!r}: {traceback.format_exc()}"),
            )
        n_deleted += len(drop_ids & all_ids)
        n_concepts += 1

    state = _refresh_loaded_detections(state, output_dir)
    msg = (
        f"Saved {n_deleted} deletion(s) across {n_concepts} concept(s) "
        f"to disk. Run `python -m sam3.labeler.compact <output_dir>` "
        f"when ready to refresh annotations.json."
    )
    return state, _render_overlay(state), _status_text(state, msg)


def _on_discard_edits(state: Dict[str, Any]):
    """Drop ``pending_delete`` so queued items reappear on the canvas."""
    state = copy.deepcopy(state or _new_state())
    n = len(state.get("pending_delete") or [])
    state["pending_delete"] = []
    state["selected_for_delete"] = []
    state["box_in_progress"] = None
    msg = (
        f"Discarded {n} pending edit(s); on-disk state unchanged."
        if n
        else "No pending edits to discard."
    )
    return state, _render_overlay(state), _status_text(state, msg)


def _reset_prompts(state: Dict[str, Any]):
    state = copy.deepcopy(state or _new_state())
    if state.get("mode") == "pvs":
        state["pvs_prompts"]["current"] = _empty_pvs_slot()
    else:
        state["pcs_prompts"] = {"pos_boxes": [], "neg_boxes": []}
    state["box_in_progress"] = None
    return state, _render_overlay(state), _status_text(state, "Prompts cleared.")


def _reset_all_pvs(state: Dict[str, Any]):
    state = copy.deepcopy(state or _new_state())
    state["pvs_prompts"] = {"committed": [], "current": _empty_pvs_slot()}
    state["box_in_progress"] = None
    return state, _render_overlay(state), _status_text(state, "All PVS slots cleared.")


def _add_object(state: Dict[str, Any]):
    state = copy.deepcopy(state or _new_state())
    current = state["pvs_prompts"]["current"]
    has_any = bool(
        current["pos_points"] or current["neg_points"] or current["box"]
    )
    msg = ""
    if has_any:
        state["pvs_prompts"]["committed"].append(current)
        state["pvs_prompts"]["current"] = _empty_pvs_slot()
        state["box_in_progress"] = None
        msg = (
            f"Committed object slot #{len(state['pvs_prompts']['committed'])}. "
            "Start the next object."
        )
    else:
        msg = "Current slot is empty; nothing to commit."
    return state, _render_overlay(state), _status_text(state, msg)


# ---------------------------------------------------------------------- #
# Image loading
# ---------------------------------------------------------------------- #
def _on_upload(file_obj: Any, state: Dict[str, Any], output_dir: str = ""):
    state = _state_for_new_image(state or _new_state())
    if file_obj is None:
        return (
            state,
            None,
            None,
            _status_text(state, "No image loaded."),
        )
    path = file_obj if isinstance(file_obj, str) else getattr(file_obj, "name", None)
    if path is None:
        return (
            state,
            None,
            None,
            _status_text(state, "Could not read uploaded file."),
        )
    pil = _load_pil(path)
    state["image_key"] = f"upload::{os.path.basename(path)}"
    state["image_path"] = path
    state["image_size"] = pil.size
    state = _refresh_loaded_detections(state, output_dir)
    return (
        state,
        _render_overlay(state),  # bare original until prompts are placed
        None,                     # result image cleared
        _status_text(state),
    )


def _counter_update(index_zero_based: Optional[int], total: int):
    """Build the ``gr.update`` payload for the editable Index widget.

    The widget is a ``gr.Number`` whose label shows the directory size
    (``Index (of 297)``) and whose value is the 1-based current index.
    When the directory is empty both are nulled out.
    """
    if total <= 0:
        return gr.update(value=None, label="Index", maximum=None)
    one_based = (
        max(1, min(int(index_zero_based) + 1, total))
        if index_zero_based is not None
        else None
    )
    return gr.update(
        value=one_based, label=f"Index (of {total})", maximum=total,
    )


def _on_dir_load(directory: str, state: Dict[str, Any], output_dir: str = ""):
    state = copy.deepcopy(state or _new_state())
    files = _list_images(directory)
    state["directory"] = {"files": files, "index": 0}
    if not files:
        return (
            state,
            None,
            None,
            f"No images found in {directory!r}.",
            _counter_update(None, 0),
        )
    return _load_directory_index(state, 0, output_dir=output_dir)


def _load_directory_index(
    state: Dict[str, Any], index: int, output_dir: str = "",
):
    state = _state_for_new_image(state)
    files = state["directory"]["files"]
    if not files:
        return (
            state,
            None,
            None,
            "Empty directory.",
            _counter_update(None, 0),
        )
    index = max(0, min(index, len(files) - 1))
    state["directory"]["index"] = index
    path = files[index]
    pil = _load_pil(path)
    state["image_key"] = path
    state["image_path"] = path
    state["image_size"] = pil.size
    state = _refresh_loaded_detections(state, output_dir)
    return (
        state,
        _render_overlay(state),
        None,
        _status_text(state),
        _counter_update(index, len(files)),
    )


def _on_prev(state: Dict[str, Any], output_dir: str = ""):
    state = copy.deepcopy(state or _new_state())
    files = state.get("directory", {}).get("files", [])
    if not files:
        return _load_directory_index(state, 0, output_dir=output_dir)
    return _load_directory_index(
        state, state["directory"]["index"] - 1, output_dir=output_dir,
    )


def _on_next(state: Dict[str, Any], output_dir: str = ""):
    state = copy.deepcopy(state or _new_state())
    files = state.get("directory", {}).get("files", [])
    if not files:
        return _load_directory_index(state, 0, output_dir=output_dir)
    return _load_directory_index(
        state, state["directory"]["index"] + 1, output_dir=output_dir,
    )


def _on_goto(
    target_one_based: Any, state: Dict[str, Any], output_dir: str = "",
):
    """Jump to a 1-based image index supplied by the user.

    Triggered by ``submit`` (Enter) on the editable Index widget.
    ``None`` / blank input is a no-op (keeps the current image and
    surfaces a brief status hint). Out-of-range values are clamped by
    :func:`_load_directory_index`, so typing ``9999`` lands on the
    last image rather than raising.
    """
    state = copy.deepcopy(state or _new_state())
    files = state.get("directory", {}).get("files", [])
    current = state.get("directory", {}).get("index", 0)
    if target_one_based is None or target_one_based == "":
        return (
            state,
            _render_overlay(state),
            None,
            _status_text(state, "Enter an index and press Enter to jump."),
            _counter_update(current, len(files)),
        )
    if not files:
        return (
            state,
            None,
            None,
            _status_text(state, "Load a directory first."),
            _counter_update(None, 0),
        )
    try:
        idx = int(target_one_based) - 1
    except (TypeError, ValueError):
        return (
            state,
            _render_overlay(state),
            None,
            _status_text(state, f"Invalid index: {target_one_based!r}."),
            _counter_update(current, len(files)),
        )
    return _load_directory_index(state, idx, output_dir=output_dir)


# ---------------------------------------------------------------------- #
# Detect / Save / Mark-as-negative
# ---------------------------------------------------------------------- #
def _collect_pvs_slots(state: Dict[str, Any]) -> List[Dict[str, Any]]:
    slots: List[Dict[str, Any]] = list(state["pvs_prompts"]["committed"])
    current = state["pvs_prompts"]["current"]
    if (
        current["pos_points"]
        or current["neg_points"]
        or current["box"] is not None
    ):
        slots = slots + [current]
    return slots


def _on_detect(
    state: Dict[str, Any],
    noun_phrase: str,
    confidence: float,
    bpe_path: Optional[str],
    checkpoint_path: Optional[str],
    device: str,
    multimask_output: bool,
    intra_nms_iou: float,
    cross_nms_iou: float,
    nms_proximity: float,
    nms_min_mask_ratio: float,
):
    state = copy.deepcopy(state or _new_state())
    if state.get("image_path") is None:
        return state, np.zeros((1, 1, 3), dtype=np.uint8), _status_text(state, "Load an image first.")

    pil = _load_pil(state["image_path"])
    try:
        annotator = _get_annotator(
            bpe_path=bpe_path or None,
            checkpoint_path=checkpoint_path or None,
            device=device,
            confidence_threshold=float(confidence),
        )
    except Exception:  # pragma: no cover - surfaced to user
        return (
            state,
            np.zeros((1, 1, 3), dtype=np.uint8),
            _status_text(state, f"Could not load SAM 3 model: {traceback.format_exc()}"),
        )

    mode = state.get("mode", "pcs")
    try:
        if mode == "pcs":
            results = annotator.detect_pcs(
                image_key=state["image_key"],
                pil=pil,
                noun_phrase=(noun_phrase or "").strip(),
                pos_boxes_xyxy_pix=state["pcs_prompts"]["pos_boxes"],
                neg_boxes_xyxy_pix=state["pcs_prompts"]["neg_boxes"],
                confidence_threshold=float(confidence),
            )
            label_for_results = (noun_phrase or "").strip() or "match"
        else:
            slots = _collect_pvs_slots(state)
            results = annotator.segment_pvs(
                image_key=state["image_key"],
                pil=pil,
                objects=slots,
                multimask_output=bool(multimask_output),
            )
            label_for_results = (noun_phrase or "").strip() or "obj"
    except ValueError as e:
        return state, np.zeros((1, 1, 3), dtype=np.uint8), _status_text(state, str(e))
    except Exception:  # pragma: no cover - surfaced to user
        return (
            state,
            np.zeros((1, 1, 3), dtype=np.uint8),
            _status_text(state, f"Inference error: {traceback.format_exc()}"),
        )

    # PCS produces multiple instances for a single noun phrase, so the
    # same two-stage NMS pipeline used by ``trt_infer.py`` applies here.
    # PVS results come from independent user-defined object slots and
    # are not deduplicated.
    intra_removed = 0
    cross_removed = 0
    if mode == "pcs" and results:
        W, H = state.get("image_size", (pil.size[0], pil.size[1]))
        results, intra_removed, cross_removed = _apply_detection_nms(
            results,
            image_hw=(int(H), int(W)),
            intra_iou_threshold=float(intra_nms_iou),
            cross_iou_threshold=float(cross_nms_iou),
            proximity_threshold=float(nms_proximity),
            min_mask_size_ratio=float(nms_min_mask_ratio),
        )

    state["last_detection"] = results
    result_img = render_results(pil, results, noun_phrase=label_for_results)
    msg = f"Detected {len(results)} instance(s) in {mode.upper()} mode."
    if intra_removed or cross_removed:
        msg += (
            f" (NMS removed {intra_removed} intra-class duplicate(s), "
            f"{cross_removed} cross-class duplicate(s).)"
        )
    return state, result_img, _status_text(state, msg)


def _build_raw_prompts_sidecar(
    state: Dict[str, Any], noun_phrase: str, confidence: float
) -> Dict[str, Any]:
    return {
        "image_path": state.get("image_path"),
        "image_size": state.get("image_size"),
        "mode": state.get("mode"),
        "noun_phrase": noun_phrase,
        "confidence_threshold": float(confidence),
        "pcs_prompts": state.get("pcs_prompts"),
        "pvs_prompts": state.get("pvs_prompts"),
    }


def _drop_pending_for_np(state: Dict[str, Any], np_text: str) -> int:
    """Remove ``pending_delete`` entries scoped to ``np_text``.

    Called after any path that rewrites the shard for that NP, so the
    in-memory queue cannot point at ann_ids that no longer exist on
    disk. Returns the number of entries dropped.
    """
    pending = list(state.get("pending_delete") or [])
    keep = [(n, i) for (n, i) in pending if str(n) != str(np_text)]
    state["pending_delete"] = keep
    return len(pending) - len(keep)


def _on_save(
    state: Dict[str, Any],
    noun_phrase: str,
    output_dir: str,
    is_instance_exhaustive: bool,
    is_pixel_exhaustive: bool,
    confidence: float,
):
    state = copy.deepcopy(state or _new_state())
    if state.get("image_path") is None:
        return state, _status_text(state, "Load an image first.")
    if not output_dir:
        return state, _status_text(state, "Set an output directory before saving.")
    np_text = (noun_phrase or "").strip()
    if not np_text:
        return state, _status_text(
            state, "A noun phrase is required to save (used as the NP for the image-NP pair)."
        )

    results = state.get("last_detection") or []
    if not results:
        return state, _status_text(
            state, "Nothing to save - run Detect first, or use 'Mark as negative'."
        )

    mode = state.get("mode", "pcs")

    try:
        store = AnnotationStore(output_dir)
        pil = _load_pil(state["image_path"])
        W, H = pil.size
        raw_prompts = _build_raw_prompts_sidecar(state, np_text, confidence)
        pairs_path = store.pairs_path(state["image_path"], np_text)
        vis_path = store.vis_path(state["image_path"], np_text)

        if mode == "pvs":
            # Append: new PVS detections land alongside whatever the
            # shard already contains, minus anything queued for delete
            # in edit mode (so a single Save click can both delete and
            # add atomically).
            pending_for_np = {
                int(ann_id)
                for (n, ann_id) in (state.get("pending_delete") or [])
                if str(n) == np_text
            }
            existing_dets = (state.get("loaded_detections") or {}).get(np_text, [])
            kept_existing = [
                {
                    "box_xyxy_pix": d.get("box_xyxy_pix"),
                    "mask": d.get("mask"),
                    "score": float(d.get("score", 0.0) or 0.0),
                    "source": d.get("source") or "sam3-pvs",
                }
                for d in existing_dets
                if int(d.get("ann_id", 0) or 0) not in pending_for_np
            ]
            slots = _collect_pvs_slots(state)
            store.append_pair(
                file_name=state["image_path"],
                width=W,
                height=H,
                noun_phrase=np_text,
                results=results,
                drop_ann_ids=pending_for_np,
                is_instance_exhaustive=int(bool(is_instance_exhaustive)),
                is_pixel_exhaustive=int(bool(is_pixel_exhaustive)),
                pvs_objects=slots,
                mode=mode,
                raw_prompts=raw_prompts,
            )
            merged_results = kept_existing + list(results)
            save_rgb_jpeg(
                render_results(pil, merged_results, noun_phrase=np_text), vis_path,
            )
            n_dropped = _drop_pending_for_np(state, np_text)
            state = _refresh_loaded_detections(state, output_dir)
            kept_n = len(kept_existing)
            new_n = len(results)
            if n_dropped:
                msg = (
                    f"Appended {new_n} PVS detection(s) and dropped "
                    f"{n_dropped} pending delete(s) for NP {np_text!r} "
                    f"({kept_n + new_n} total now). Wrote shard "
                    f"{os.path.basename(pairs_path)} and "
                    f"{os.path.basename(vis_path)}."
                )
            else:
                msg = (
                    f"Appended {new_n} PVS detection(s) to NP {np_text!r} "
                    f"({kept_n + new_n} total now). Wrote shard "
                    f"{os.path.basename(pairs_path)} and "
                    f"{os.path.basename(vis_path)}."
                )
        else:
            # PCS: full shard rewrite. Re-running Detect is the user's
            # cue that they want a fresh enumeration of the NP, so
            # replacing the shard is the right semantic. Any
            # pending_delete entries for this NP are now moot (the
            # whole shard was just rewritten).
            store.save_pair(
                file_name=state["image_path"],
                width=W,
                height=H,
                noun_phrase=np_text,
                results=results,
                is_instance_exhaustive=int(bool(is_instance_exhaustive)),
                is_pixel_exhaustive=int(bool(is_pixel_exhaustive)),
                pcs_pos_boxes_xyxy_pix=state["pcs_prompts"]["pos_boxes"],
                pcs_neg_boxes_xyxy_pix=state["pcs_prompts"]["neg_boxes"],
                pvs_objects=(),
                mode=mode,
                raw_prompts=raw_prompts,
            )
            save_rgb_jpeg(render_results(pil, results, noun_phrase=np_text), vis_path)
            _drop_pending_for_np(state, np_text)
            state = _refresh_loaded_detections(state, output_dir)
            msg = (
                f"Saved {len(results)} mask(s) for NP {np_text!r} "
                f"(replaced shard {os.path.basename(pairs_path)}). "
                f"Run `python -m sam3.labeler.compact <output_dir>` "
                f"when ready to build annotations.json."
            )

        if np_text not in state["saved_nps"]:
            state["saved_nps"].append(np_text)
        return state, _status_text(state, msg)
    except Exception:  # pragma: no cover - surfaced to user
        return state, _status_text(state, f"Save failed: {traceback.format_exc()}")


def _on_mark_negative(
    state: Dict[str, Any],
    noun_phrase: str,
    output_dir: str,
    confidence: float,
):
    state = copy.deepcopy(state or _new_state())
    if state.get("image_path") is None:
        return state, _status_text(state, "Load an image first.")
    if not output_dir:
        return state, _status_text(state, "Set an output directory before saving.")
    np_text = (noun_phrase or "").strip()
    if not np_text:
        return state, _status_text(state, "A noun phrase is required to mark as negative.")

    try:
        store = AnnotationStore(output_dir)
        pil = _load_pil(state["image_path"])
        W, H = pil.size
        store.save_pair(
            file_name=state["image_path"],
            width=W,
            height=H,
            noun_phrase=np_text,
            results=[],
            is_instance_exhaustive=1,
            is_pixel_exhaustive=1,
            mode=state.get("mode", "pcs"),
            raw_prompts=_build_raw_prompts_sidecar(state, np_text, confidence),
        )
        pairs_path = store.pairs_path(state["image_path"], np_text)
        # The shard was rewritten as empty, so any pending deletes for
        # this NP are stale ann_ids - drop them.
        n_dropped = _drop_pending_for_np(state, np_text)
        state = _refresh_loaded_detections(state, output_dir)
        if np_text not in state["saved_nps"]:
            state["saved_nps"].append(np_text)
        suffix = (
            f" Dropped {n_dropped} stale pending delete(s)."
            if n_dropped
            else ""
        )
        msg = (
            f"Marked {os.path.basename(state['image_path'])!r} as NEGATIVE "
            f"for NP {np_text!r}. Wrote shard "
            f"{os.path.basename(pairs_path)}.{suffix}"
        )
        return state, _status_text(state, msg)
    except Exception:  # pragma: no cover - surfaced to user
        return state, _status_text(state, f"Save failed: {traceback.format_exc()}")


# ---------------------------------------------------------------------- #
# UI
# ---------------------------------------------------------------------- #
# Injected into the page's <head>. `gr.HTML("<script>...")` blocks in the
# body get sanitised in Gradio 5 (script tags are stripped before render),
# so the keyboard navigation hook has to live in the document head where
# Gradio passes it through verbatim via the Blocks(head=...) parameter.
_KEYBOARD_NAV_JS = """
<script>
(function () {
    if (window.__sam3LabelerKeysBound) { return; }
    window.__sam3LabelerKeysBound = true;

    function isTypingContext(el) {
        if (!el) return false;
        if (el.isContentEditable) return true;
        var tag = (el.tagName || '').toUpperCase();
        if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return true;
        return false;
    }

    function clickBtn(elemId) {
        var wrapper = document.getElementById(elemId);
        if (!wrapper) return false;
        var btn = (wrapper.tagName && wrapper.tagName.toUpperCase() === 'BUTTON')
            ? wrapper
            : wrapper.querySelector('button');
        if (!btn || btn.disabled) return false;
        btn.click();
        return true;
    }

    function handler(e) {
        if (e.defaultPrevented) return;
        if (e.ctrlKey || e.metaKey || e.altKey) return;
        if (isTypingContext(e.target)) return;
        if (e.key === 'ArrowLeft') {
            if (clickBtn('labeler-prev-btn')) e.preventDefault();
        } else if (e.key === 'ArrowRight') {
            if (clickBtn('labeler-next-btn')) e.preventDefault();
        }
    }

    // Bind on both document and window to maximise coverage across
    // Gradio host configurations (top-level pages, hosted Spaces, etc.).
    document.addEventListener('keydown', handler, true);
    window.addEventListener('keydown', handler, true);
})();
</script>
"""


def build_ui(default_output_dir: str = "", default_device: str = "cuda:0") -> gr.Blocks:
    with gr.Blocks(title="SAM 3 Labeler", head=_KEYBOARD_NAV_JS) as demo:
        gr.Markdown(
            "## SAM 3 Browser Annotation Tool\n"
            "Two inference paths against Meta's official `sam3` package: "
            "**Find all (PCS)** for text-driven exhaustive detection and "
            "**Click and segment (PVS)** for per-object interactive masking. "
            "Prompts are placed by left-clicking on the input image; the "
            "image redraws with your prompts on top of it after every click. "
            "Boxes are drawn with **two clicks** (top-left then bottom-right).\n\n"
            "### How clicks work\n"
            "Pick a **Mode** (what *Detect* does) plus a **Tool** + "
            "**Polarity** (what each click adds). The four useful "
            "combinations are:\n\n"
            "- **Find all (PCS) + Box** \u2014 place positive (green) or "
            "negative (red) box exemplars to disambiguate the noun phrase. "
            "Optional when the NP alone is unambiguous; required when it "
            "is not. Two clicks per box.\n"
            "- **Find all (PCS) + Point** \u2014 SAM 3's text path does "
            "not accept points, so a click is auto-converted to a small "
            "\u00b112 px box. Handy as a quick exemplar; prefer Box for "
            "precision.\n"
            "- **Click and segment (PVS) + Point** \u2014 positive points "
            "go inside the object, negative points stay off it. Click "
            "**Add object (PVS)** to commit the current slot and start "
            "the next object.\n"
            "- **Click and segment (PVS) + Box** \u2014 optional single "
            "bounding box around the current PVS object slot (two clicks). "
            "Combines with the slot's points.\n"
            "- **Edit existing detections** \u2014 review and delete "
            "previously-saved (or `trt_infer.py --save_for_labeler`) "
            "detections. Tool=Point clicks toggle a single detection's "
            "selection (clicks on the label or near the bbox edge also "
            "count); Tool=Box draws a 2-click rectangle and adds every "
            "detection whose bbox center is inside. Click "
            "**Delete selected** to queue them for removal (they "
            "disappear from the canvas). The queue is **not written to "
            "disk** until you click **Save edits to disk**; click "
            "**Discard edits** to bring them back, or **Reload from "
            "disk** to throw away all in-memory changes for this "
            "image. Set the **Output directory** below to the same "
            "folder used by the inference run.\n\n"
            "### Where saves go\n"
            "Each **Save** writes one self-contained shard at "
            "`<output_dir>/pairs/<image_stem>__<np_slug>.json` plus a "
            "matching `vis/...jpg` thumbnail and `prompts/...json` sidecar. "
            "**Re-saving the same image-NP pair overwrites that shard**, "
            "so you cannot accumulate duplicates. To undo a mistake, "
            "delete the offending `pairs/<file>.json` (plus its sidecars) "
            "from disk. When you are ready to produce the canonical "
            "`annotations.json` consumed by SAM 3 eval / training, run "
            "`python -m sam3.labeler.compact <output_dir>` from a shell."
        )

        # ---- Top settings ----
        with gr.Accordion("Model settings", open=False):
            bpe_path_in = gr.Textbox(
                label="BPE vocab path (optional)",
                value="",
                placeholder="Leave empty to use sam3's bundled assets/bpe_simple_vocab_16e6.txt.gz",
            )
            checkpoint_path_in = gr.Textbox(
                label="Checkpoint path (optional)",
                value="",
                placeholder="Specify the directory of the checkpoint file",
            )
            device_in = gr.Textbox(label="Device", value=default_device)

        # ---- Source ----
        with gr.Row():
            source_radio = gr.Radio(
                choices=["Single upload", "Directory"],
                value="Single upload",
                label="Source",
            )

        with gr.Group(visible=True) as upload_group:
            upload_in = gr.File(
                label="Upload an image",
                file_types=["image"],
                file_count="single",
                type="filepath",
            )

        with gr.Group(visible=False) as dir_group:
            with gr.Row():
                dir_in = gr.Textbox(label="Directory path", value="")
                dir_load_btn = gr.Button("Load directory")
            with gr.Row():
                prev_btn = gr.Button("Prev", elem_id="labeler-prev-btn")
                # Single editable Index widget: shows the current 1-based
                # index, total appears in the label as "Index (of N)",
                # and pressing Enter on a new value jumps to it.
                # Arrow-key Prev/Next still work because the hook in
                # _KEYBOARD_NAV_JS skips when an <input> has focus.
                dir_counter = gr.Number(
                    label="Index",
                    value=None,
                    precision=0,
                    minimum=1,
                    step=1,
                    interactive=True,
                )
                next_btn = gr.Button("Next", elem_id="labeler-next-btn")

        # ---- Inference mode ----
        with gr.Row():
            mode_radio = gr.Radio(
                choices=[
                    ("Find all (PCS)", "pcs"),
                    ("Click and segment (PVS)", "pvs"),
                    ("Edit existing detections", "edit"),
                ],
                value="pcs",
                label="Mode",
                type="value",
            )
            tool_radio = gr.Radio(
                choices=["Point", "Box"],
                value="Point",
                label="Tool (next click)",
            )
            polarity_radio = gr.Radio(
                choices=["Positive", "Negative"],
                value="Positive",
                label="Polarity (next prompt)",
            )
        np_in = gr.Textbox(
            label="Noun phrase",
            placeholder="e.g. the red mug (required in PCS, also used as the saved NP in PVS)",
            value="",
        )
        confidence_in = gr.Slider(
            minimum=0.05, maximum=0.95, value=0.5, step=0.01,
            label="Confidence threshold (PCS)",
            visible=True,
        )
        multimask_in = gr.Checkbox(
            label="PVS multimask output (pick best of 3)",
            value=True,
            visible=False,
        )

        # ---- NMS settings (PCS-only, mirrors ``trt_infer.py``) ----
        with gr.Accordion(
            "NMS settings (PCS)", open=False, visible=True,
        ) as nms_accordion:
            gr.Markdown(
                "Same two-stage NMS as `sam3/export_hf/trt_infer.py`. Set a "
                "threshold to 0 to disable that stage."
            )
            with gr.Row():
                intra_nms_in = gr.Slider(
                    minimum=0.0, maximum=1.0, value=0.5, step=0.01,
                    label="Intra-class box IoU threshold",
                )
                cross_nms_in = gr.Slider(
                    minimum=0.0, maximum=1.0, value=0.8, step=0.01,
                    label="Cross-class mask IoU threshold",
                )
            with gr.Row():
                nms_proximity_in = gr.Slider(
                    minimum=0.0, maximum=200.0, value=20.0, step=1.0,
                    label="Cross-class proximity threshold (px)",
                )
                nms_min_mask_ratio_in = gr.Slider(
                    minimum=0.0, maximum=0.05, value=0.0001, step=0.0001,
                    label="Min mask size ratio",
                )

        # ---- Status (sits directly above the visualisations so the
        # current image / mode / selection counts are always within
        # eyeshot of the canvases the user is interacting with) ----
        status = gr.Markdown("Ready.")

        # ---- Input + result ----
        # ``image_in`` doubles as the prompt overlay: every state-changing
        # callback pushes ``_render_overlay(state)`` (the original image with
        # the user's prompts drawn on top) back into it. ``sources=[]``
        # disables the Upload / Webcam / Clipboard tabs that Gradio adds by
        # default; uploads go through the dedicated ``gr.File`` widget above.
        with gr.Row():
            image_in = gr.Image(
                label="Input - click to drop prompts (overlay drawn here)",
                type="numpy",
                interactive=True,
                sources=[],
                show_download_button=False,
            )
            result_img = gr.Image(
                label="Detection / segmentation result",
                interactive=False,
            )

        # ---- Actions ----
        with gr.Row():
            reset_btn = gr.Button("Reset prompts")
            reset_all_btn = gr.Button("Reset all (PVS)", visible=False)
            add_obj_btn = gr.Button("Add object (PVS)", visible=False)
            detect_btn = gr.Button("Detect", variant="primary")
            save_btn = gr.Button("Save")
            mark_neg_btn = gr.Button("Mark as negative")
        # Edit-mode actions live on a separate row so the workflow is
        # visually distinct from prompt placement.
        with gr.Row():
            delete_btn = gr.Button(
                "Delete selected", variant="stop", visible=False,
            )
            clear_sel_btn = gr.Button("Clear selection", visible=False)
            save_edits_btn = gr.Button(
                "Save edits to disk", variant="primary", visible=False,
            )
            discard_edits_btn = gr.Button("Discard edits", visible=False)
            reload_existing_btn = gr.Button("Reload from disk", visible=False)

        # ---- Save options ----
        with gr.Accordion("Save options", open=True):
            output_dir_in = gr.Textbox(
                label="Output directory",
                value=default_output_dir,
                placeholder="e.g. /tmp/sam3-labels",
            )
            with gr.Row():
                inst_exh_in = gr.Checkbox(label="is_instance_exhaustive", value=True)
                pix_exh_in = gr.Checkbox(label="is_pixel_exhaustive", value=True)

        state = gr.State(value=_new_state())

        # ------------------------------------------------------------------
        # Wiring
        # ------------------------------------------------------------------
        def _toggle_source(choice: str):
            return (
                gr.update(visible=(choice == "Single upload")),
                gr.update(visible=(choice == "Directory")),
            )

        source_radio.change(
            _toggle_source, inputs=[source_radio], outputs=[upload_group, dir_group]
        )

        mode_radio.change(
            _set_mode,
            inputs=[mode_radio, state],
            outputs=[
                state,
                image_in,
                reset_all_btn,
                add_obj_btn,
                confidence_in,
                multimask_in,
                nms_accordion,
                np_in,
                detect_btn,
                save_btn,
                mark_neg_btn,
                reset_btn,
                delete_btn,
                clear_sel_btn,
                save_edits_btn,
                discard_edits_btn,
                reload_existing_btn,
                status,
            ],
        )

        upload_in.change(
            _on_upload,
            inputs=[upload_in, state, output_dir_in],
            outputs=[state, image_in, result_img, status],
        )
        dir_load_btn.click(
            _on_dir_load,
            inputs=[dir_in, state, output_dir_in],
            outputs=[state, image_in, result_img, status, dir_counter],
        )
        prev_btn.click(
            _on_prev,
            inputs=[state, output_dir_in],
            outputs=[state, image_in, result_img, status, dir_counter],
        )
        next_btn.click(
            _on_next,
            inputs=[state, output_dir_in],
            outputs=[state, image_in, result_img, status, dir_counter],
        )
        # Editable Index: pressing Enter on a new value jumps to it.
        dir_counter.submit(
            _on_goto,
            inputs=[dir_counter, state, output_dir_in],
            outputs=[state, image_in, result_img, status, dir_counter],
        )

        image_in.select(
            _on_image_click,
            inputs=[image_in, mode_radio, polarity_radio, tool_radio, state],
            outputs=[state, image_in, status],
        )

        reset_btn.click(
            _reset_prompts,
            inputs=[state],
            outputs=[state, image_in, status],
        )
        reset_all_btn.click(
            _reset_all_pvs,
            inputs=[state],
            outputs=[state, image_in, status],
        )
        add_obj_btn.click(
            _add_object,
            inputs=[state],
            outputs=[state, image_in, status],
        )

        detect_btn.click(
            _on_detect,
            inputs=[
                state,
                np_in,
                confidence_in,
                bpe_path_in,
                checkpoint_path_in,
                device_in,
                multimask_in,
                intra_nms_in,
                cross_nms_in,
                nms_proximity_in,
                nms_min_mask_ratio_in,
            ],
            outputs=[state, result_img, status],
        )

        save_btn.click(
            _on_save,
            inputs=[
                state,
                np_in,
                output_dir_in,
                inst_exh_in,
                pix_exh_in,
                confidence_in,
            ],
            outputs=[state, status],
        )

        mark_neg_btn.click(
            _on_mark_negative,
            inputs=[state, np_in, output_dir_in, confidence_in],
            outputs=[state, status],
        )

        delete_btn.click(
            _on_delete_selected,
            inputs=[state],
            outputs=[state, image_in, status],
        )
        clear_sel_btn.click(
            _on_clear_selection,
            inputs=[state],
            outputs=[state, image_in, status],
        )
        save_edits_btn.click(
            _on_save_edits,
            inputs=[state, output_dir_in],
            outputs=[state, image_in, status],
        )
        discard_edits_btn.click(
            _on_discard_edits,
            inputs=[state],
            outputs=[state, image_in, status],
        )
        reload_existing_btn.click(
            _on_reload_existing,
            inputs=[state, output_dir_in],
            outputs=[state, image_in, status],
        )

    return demo


# ---------------------------------------------------------------------- #
# Entry point
# ---------------------------------------------------------------------- #
def _parse_args(argv: List[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="SAM 3 browser-based annotation tool.")
    p.add_argument("--output_dir", default="", help="Default output directory for saved annotations.")
    p.add_argument("--device", default="cuda:0", help='Default device, e.g. "cuda", "cuda:0", "cpu".')
    p.add_argument("--bpe_path", default=None, help="Optional override for the BPE vocab path.")
    p.add_argument("--checkpoint_path", default=None, help="Optional override for the SAM 3 checkpoint path.")
    p.add_argument("--share", action="store_true", help="Launch a public Gradio link.")
    p.add_argument(
        "--server_name",
        default="0.0.0.0",
        help=(
            "Interface to bind to. Use 0.0.0.0 to expose to your LAN / SSH host "
            '(default), or 127.0.0.1 for loopback-only.'
        ),
    )
    p.add_argument("--server_port", type=int, default=7860)
    p.add_argument(
        "--show_api",
        action="store_true",
        help=(
            "Enable Gradio's /info API schema endpoint. Disabled by default "
            "because older gradio_client builds crash with "
            "'argument of type bool is not iterable' on schemas containing "
            "additionalProperties=true."
        ),
    )
    p.add_argument("--max_cached_states", type=int, default=8)
    p.add_argument("--preload", action="store_true", help="Load the SAM 3 model at startup instead of on first Detect.")
    return p.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    args = _parse_args(argv if argv is not None else sys.argv[1:])

    if args.preload:
        _get_annotator(
            bpe_path=args.bpe_path,
            checkpoint_path=args.checkpoint_path,
            device=args.device,
            confidence_threshold=0.5,
        )

    demo = build_ui(default_output_dir=args.output_dir, default_device=args.device)
    demo.queue().launch(
        server_name=args.server_name,
        server_port=args.server_port,
        share=args.share,
        show_api=args.show_api,
    )


if __name__ == "__main__":
    main()
