# Copyright (c) Meta Platforms, Inc. and affiliates. All Rights Reserved
"""Overlay renderers for the SAM 3 browser labeler.

Two functions are exposed:

* :func:`render_prompts` - draws the user's prompts (positive/negative
  points and boxes, per-PVS-slot tinted) on top of the original image
  for the left panel.
* :func:`render_results` - draws the predicted masks, bboxes, and score
  labels on top of the original image for the right (result) panel.

Both return an ``np.ndarray`` (H, W, 3) in RGB at the original image
resolution, which Gradio's :class:`gradio.Image` component renders
natively. The helpers reuse the colour palette from
:mod:`sam3.export_hf.trt_infer`.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
from PIL import Image


# Curated colour palette (RGB in [0, 1]) reused from
# ``sam3/export_hf/trt_infer.py`` so visual identity stays consistent
# across the codebase. Inlined because ``sam3/export_hf`` is a sibling
# directory of the ``sam3`` python package, not part of it.
COLORS: List[Tuple[float, float, float]] = [
    (0.122, 0.467, 0.706),
    (1.0, 0.498, 0.055),
    (0.173, 0.627, 0.173),
    (0.839, 0.153, 0.157),
    (0.580, 0.404, 0.741),
    (0.549, 0.337, 0.294),
    (0.890, 0.467, 0.761),
    (0.498, 0.498, 0.498),
    (0.737, 0.741, 0.133),
    (0.090, 0.745, 0.812),
]


def color_to_bgr(color) -> Tuple[int, int, int]:
    r = int(color[0] * 255) if color[0] <= 1.0 else int(color[0])
    g = int(color[1] * 255) if color[1] <= 1.0 else int(color[1])
    b = int(color[2] * 255) if color[2] <= 1.0 else int(color[2])
    return (b, g, r)


def draw_mask_cv2(img: np.ndarray, mask: np.ndarray, color_bgr, alpha: float = 0.5) -> None:
    """Alpha-blend a coloured overlay where ``mask`` is True."""
    import torch

    if isinstance(mask, torch.Tensor):
        mask = mask.cpu().numpy()
    mask = np.asarray(mask).astype(bool)
    if mask.ndim != 2:
        mask = mask.squeeze()
    overlay = img.copy()
    overlay[mask] = color_bgr
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)


def draw_bbox_cv2(
    img: np.ndarray,
    box: Sequence[float],
    text: str,
    color_bgr,
    thickness: int = 4,
) -> None:
    x1, y1, x2, y2 = map(int, box[:4])
    cv2.rectangle(img, (x1, y1), (x2, y2), color_bgr, thickness)
    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale, font_thickness = 0.5, 1
    (tw, th), _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    pad = 8
    cv2.rectangle(img, (x1, y1 - th - pad * 2), (x1 + tw + pad * 2, y1), color_bgr, -1)
    cv2.putText(img, text, (x1 + pad, y1 - pad), font, font_scale, (0, 0, 0), font_thickness + 2, cv2.LINE_AA)
    cv2.putText(img, text, (x1 + pad, y1 - pad), font, font_scale, (255, 255, 255), font_thickness, cv2.LINE_AA)


# Bright, saturated colours for the user-prompt overlay so they read
# against any image. BGR for cv2.
_POS_COLOR_BGR: Tuple[int, int, int] = (0, 200, 0)
_NEG_COLOR_BGR: Tuple[int, int, int] = (0, 0, 220)
_NEG_TINT_ALPHA: float = 0.35


def _pil_or_array_to_rgb_ndarray(image) -> np.ndarray:
    if isinstance(image, np.ndarray):
        arr = image
        if arr.ndim == 2:
            arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2RGB)
        elif arr.shape[2] == 4:
            arr = cv2.cvtColor(arr, cv2.COLOR_RGBA2RGB)
        return arr.copy()

    if not isinstance(image, Image.Image):
        raise TypeError(f"Unsupported image type: {type(image)!r}")

    if image.mode != "RGB":
        image = image.convert("RGB")
    return np.array(image)


def _slot_color_bgr(slot_index: int) -> Tuple[int, int, int]:
    return color_to_bgr(COLORS[slot_index % len(COLORS)])


def _draw_point(
    img_bgr: np.ndarray,
    x: float,
    y: float,
    color_bgr: Tuple[int, int, int],
    radius: int = 8,
) -> None:
    """Draw a filled circle with a thin white outline at (x, y)."""
    cx, cy = int(round(x)), int(round(y))
    cv2.circle(img_bgr, (cx, cy), radius + 2, (255, 255, 255), thickness=2, lineType=cv2.LINE_AA)
    cv2.circle(img_bgr, (cx, cy), radius, color_bgr, thickness=-1, lineType=cv2.LINE_AA)


def render_prompts(
    image,
    *,
    mode: str = "pcs",
    pcs_pos_boxes: Sequence[Sequence[float]] = (),
    pcs_neg_boxes: Sequence[Sequence[float]] = (),
    pvs_committed: Sequence[Dict[str, Any]] = (),
    pvs_current: Optional[Dict[str, Any]] = None,
) -> np.ndarray:
    """Render an overlay of the user prompts on top of ``image``.

    All coordinates are in pixels of the original image. Returns an RGB
    ``np.ndarray``.
    """
    rgb = _pil_or_array_to_rgb_ndarray(image)
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    if mode == "pcs":
        for box in pcs_pos_boxes:
            _draw_prompt_box(bgr, box, _POS_COLOR_BGR, label="+", filled=False)
        for box in pcs_neg_boxes:
            _draw_prompt_box(bgr, box, _NEG_COLOR_BGR, label="-", filled=True)
    elif mode == "pvs":
        # Committed slots use the per-slot palette to make it easy for the
        # user to keep track of which prompts belong together; the current
        # uncommitted slot uses pure red/green polarity colours.
        for slot_idx, obj in enumerate(pvs_committed):
            slot_color = _slot_color_bgr(slot_idx)
            _draw_pvs_slot(bgr, obj, slot_color=slot_color, label=f"obj {slot_idx + 1}")
        if pvs_current is not None and _slot_has_prompts(pvs_current):
            _draw_pvs_slot(
                bgr,
                pvs_current,
                slot_color=None,
                label=f"obj {len(pvs_committed) + 1} (current)",
            )
    else:
        raise ValueError(f"Unknown mode: {mode!r}")

    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def render_results(
    image,
    results: Sequence[Dict[str, Any]],
    *,
    noun_phrase: str = "",
    alpha: float = 0.5,
) -> np.ndarray:
    """Render predicted masks + bboxes + scores on top of ``image``.

    ``results`` is the list returned by ``Sam3Annotator.detect_pcs`` /
    ``Sam3Annotator.segment_pvs`` (or anything with the same shape).
    """
    rgb = _pil_or_array_to_rgb_ndarray(image)
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    if not results:
        cv2.putText(
            bgr,
            "No detections",
            (16, 32),
            cv2.FONT_HERSHEY_DUPLEX,
            0.7,
            (40, 40, 220),
            2,
            cv2.LINE_AA,
        )
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    for i, res in enumerate(results):
        color = _slot_color_bgr(i)
        mask = res.get("mask")
        if mask is not None:
            draw_mask_cv2(bgr, mask, color, alpha=alpha)
        box = res.get("box_xyxy_pix")
        score = float(res.get("score", 0.0))
        if box is not None:
            label = noun_phrase if noun_phrase else f"obj {i + 1}"
            draw_bbox_cv2(bgr, box, f"{label}: {score:.2f}", color)

    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


# ---------------------------------------------------------------------- #
# Helpers
# ---------------------------------------------------------------------- #
def _slot_has_prompts(obj: Dict[str, Any]) -> bool:
    return bool(
        obj.get("pos_points")
        or obj.get("neg_points")
        or obj.get("box") is not None
    )


def _draw_pvs_slot(
    bgr: np.ndarray,
    obj: Dict[str, Any],
    *,
    slot_color: Optional[Tuple[int, int, int]],
    label: str,
) -> None:
    """Draw one PVS object slot.

    Positive points are green dots, negative points are red dots. A box
    (if any) gets the slot palette colour when ``slot_color`` is set,
    otherwise the standard positive-prompt green. The label is anchored
    to the top-left of the slot's bounding region.
    """
    pos_pts = obj.get("pos_points", []) or []
    neg_pts = obj.get("neg_points", []) or []
    box = obj.get("box", None)

    if box is not None:
        box_color = slot_color if slot_color is not None else _POS_COLOR_BGR
        _draw_prompt_box(bgr, box, box_color, label=label, filled=False)
    elif pos_pts or neg_pts:
        # Anchor a small text tag near the first point.
        x, y = pos_pts[0] if pos_pts else neg_pts[0]
        tag_color = slot_color if slot_color is not None else _POS_COLOR_BGR
        _draw_text_tag(bgr, int(x) + 12, int(y) - 12, label, tag_color)

    for (x, y) in pos_pts:
        _draw_point(bgr, x, y, _POS_COLOR_BGR)
    for (x, y) in neg_pts:
        _draw_point(bgr, x, y, _NEG_COLOR_BGR)


def _draw_prompt_box(
    bgr: np.ndarray,
    box: Sequence[float],
    color_bgr: Tuple[int, int, int],
    *,
    label: str,
    filled: bool,
) -> None:
    x1, y1, x2, y2 = box[:4]
    x1i, y1i, x2i, y2i = (
        int(round(min(x1, x2))),
        int(round(min(y1, y2))),
        int(round(max(x1, x2))),
        int(round(max(y1, y2))),
    )

    if filled:
        overlay = bgr.copy()
        cv2.rectangle(overlay, (x1i, y1i), (x2i, y2i), color_bgr, thickness=-1)
        cv2.addWeighted(overlay, _NEG_TINT_ALPHA, bgr, 1 - _NEG_TINT_ALPHA, 0, bgr)

    cv2.rectangle(bgr, (x1i, y1i), (x2i, y2i), color_bgr, thickness=2, lineType=cv2.LINE_AA)
    _draw_text_tag(bgr, x1i, y1i - 4, label, color_bgr)


def _draw_text_tag(
    bgr: np.ndarray,
    x: int,
    y: int,
    text: str,
    color_bgr: Tuple[int, int, int],
) -> None:
    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale, font_thickness = 0.5, 1
    (tw, th), _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    pad = 4
    y_top = max(0, y - th - pad * 2)
    cv2.rectangle(
        bgr,
        (x, y_top),
        (x + tw + pad * 2, y_top + th + pad * 2),
        color_bgr,
        thickness=-1,
    )
    cv2.putText(
        bgr,
        text,
        (x + pad, y_top + th + pad),
        font,
        font_scale,
        (0, 0, 0),
        font_thickness + 2,
        cv2.LINE_AA,
    )
    cv2.putText(
        bgr,
        text,
        (x + pad, y_top + th + pad),
        font,
        font_scale,
        (255, 255, 255),
        font_thickness,
        cv2.LINE_AA,
    )


_SELECTED_OUTLINE_BGR: Tuple[int, int, int] = (40, 40, 240)

# Palette for unselected "existing" detections. Built from the standard
# matplotlib 10-colour scheme used elsewhere in the labeler, with the
# red entry stripped out: red is reserved exclusively for the
# selected-for-delete highlight so the user can always distinguish
# "click I made just now" from "object that already exists".
_EXISTING_PALETTE_BGR: List[Tuple[int, int, int]] = [
    color_to_bgr(c) for i, c in enumerate(COLORS) if i != 3
]

# Backwards-compatible single-colour default for callers that don't pass
# a noun-phrase-keyed dict (currently unused internally but kept for
# external imports).
_EXISTING_OUTLINE_BGR: Tuple[int, int, int] = _EXISTING_PALETTE_BGR[0]


def _np_color_map(np_keys: Sequence[str]) -> Dict[str, Tuple[int, int, int]]:
    """Stable noun-phrase -> BGR colour assignment.

    Keys are sorted before indexing so the same NP always gets the
    same colour across renders, even if the dict iteration order
    changes between Gradio invocations.
    """
    palette = _EXISTING_PALETTE_BGR
    return {
        np_text: palette[i % len(palette)]
        for i, np_text in enumerate(sorted(np_keys, key=str))
    }


def render_existing_detections(
    image,
    detections_by_np: Dict[str, List[Dict[str, Any]]],
    selected: Sequence[Tuple[str, int]] = (),
    *,
    alpha: float = 0.35,
    outline_thickness: int = 2,
    draw_label: bool = True,
) -> np.ndarray:
    """Overlay previously-saved detections on top of ``image``.

    ``detections_by_np`` is the loader output (keyed by noun phrase),
    each entry being a list of ``{ann_id, box_xyxy_pix, mask, score,
    source}``. Each noun phrase gets a distinct colour from a stable
    palette so overlapping NPs are easy to tell apart; detections in
    ``selected`` are highlighted in red (which is explicitly absent
    from the per-NP palette to keep the two cues unambiguous).

    ``outline_thickness`` is the cv2 line width for unselected boxes;
    selected boxes are drawn one pixel thicker. Set to ``1`` together
    with a low ``alpha`` and ``draw_label=False`` to produce a quiet
    background layer (used by PCS/PVS overlays so the user can place
    new prompts with the existing detections still visible).
    """
    rgb = _pil_or_array_to_rgb_ndarray(image)
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    selected_set = {(str(np_text), int(ann_id)) for np_text, ann_id in selected}
    color_map = _np_color_map(detections_by_np.keys())

    for np_text, dets in detections_by_np.items():
        np_color = color_map.get(np_text, _EXISTING_OUTLINE_BGR)
        for det in dets:
            ann_id = int(det.get("ann_id", 0))
            is_selected = (np_text, ann_id) in selected_set
            color = _SELECTED_OUTLINE_BGR if is_selected else np_color
            mask = det.get("mask")
            if mask is not None:
                draw_mask_cv2(bgr, mask, color, alpha=alpha)
            box = det.get("box_xyxy_pix")
            if box is not None:
                thickness = outline_thickness + 1 if is_selected else outline_thickness
                if draw_label:
                    score = float(det.get("score", 0.0))
                    marker = "*" if is_selected else ""
                    label = f"{marker}{np_text or 'obj'}: {score:.2f}"
                else:
                    label = None
                _draw_existing_box(
                    bgr, box, color, label, thickness=thickness,
                )

    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def _draw_existing_box(
    bgr: np.ndarray,
    box: Sequence[float],
    color_bgr: Tuple[int, int, int],
    label: Optional[str],
    *,
    thickness: int,
) -> None:
    x1, y1, x2, y2 = box[:4]
    x1i, y1i, x2i, y2i = (
        int(round(min(x1, x2))),
        int(round(min(y1, y2))),
        int(round(max(x1, x2))),
        int(round(max(y1, y2))),
    )
    cv2.rectangle(
        bgr,
        (x1i, y1i),
        (x2i, y2i),
        color_bgr,
        thickness=max(1, int(thickness)),
        lineType=cv2.LINE_AA,
    )
    if label:
        _draw_text_tag(bgr, x1i, y1i - 4, label, color_bgr)


def save_rgb_jpeg(arr_rgb: np.ndarray, path: str, quality: int = 92) -> None:
    """Write an RGB ndarray to ``path`` as a JPEG, creating parents."""
    import os

    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    bgr = cv2.cvtColor(arr_rgb, cv2.COLOR_RGB2BGR)
    cv2.imwrite(path, bgr, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
