# Copyright (c) Meta Platforms, Inc. and affiliates. All Rights Reserved
"""Per-pair JSON shard writer for the SAM 3 browser labeler.

Every ``Save`` click writes ONE self-contained JSON shard at
``<output_dir>/pairs/<image_stem>__<np_slug>.json`` describing a
single image-NP pair. A shard is a minimal COCO fragment with local
IDs (``images[0].id == 1``; ``annotations[i].id == i + 1``;
``annotations[i].image_id == 1``); the canonical, monotonic IDs are
only assigned when shards are merged back into
``<output_dir>/annotations.json`` by :mod:`sam3.labeler.compact`.

This design lets the user delete a single mislabelled pair by
removing one file (``rm pairs/<stem>__<np>.json`` plus its
``vis/`` and ``prompts/`` sidecars); the next ``compact`` run will
omit it cleanly. The canonical ``annotations.json`` is a derived
artifact and is never written by this module.

Each shard's schema matches the SA-Co Gold/Silver layout consumed by
``sam3/sam3/train/data/sam3_image_dataset.py``:

* ``images[].id``       - local id, always ``1`` in a shard
* ``images[].text_input`` - the noun phrase
* ``images[].file_name``  - basename only
* ``images[].width / .height`` - in pixels
* ``images[].is_instance_exhaustive`` / ``is_pixel_exhaustive`` - 0/1
* ``images[].input_box``         - normalized xywh list of user box exemplars (PCS)
* ``images[].input_box_label``   - 1=positive, 0=negative
* ``images[].input_points``      - normalized [x, y, label] (PVS clicks)
* ``annotations[].image_id``     - local id, always ``1`` in a shard
* ``annotations[].bbox``         - normalized xywh
* ``annotations[].segmentation`` - COCO RLE
* ``annotations[].area``         - normalized area in [0, 1]
* ``annotations[].category_id``  - always 1 (unused by SAM 3 eval)
* ``annotations[].iscrowd``      - 0 by default

Negative pairs (``Mark as negative``) write a shard whose
``annotations`` list is empty.
"""

from __future__ import annotations

import json
import logging
import os
import re
import tempfile
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

logger = logging.getLogger(__name__)


_DEFAULT_INFO = {"version": "0.1", "tool": "sam3-labeler"}
_DEFAULT_CATEGORIES = [{"id": 1, "name": "object"}]


def _slugify(s: str, max_len: int = 64) -> str:
    s = s.strip().lower()
    s = re.sub(r"[^a-z0-9]+", "-", s)
    s = s.strip("-")
    if not s:
        s = "np"
    return s[:max_len]


def _encode_rle(mask: np.ndarray) -> Dict[str, Any]:
    """COCO RLE-encode a HxW boolean mask.

    The ``counts`` value is decoded to a ``str`` so the JSON file stays
    pure text and round-trips through ``json.load`` / ``json.dump``.
    """
    try:
        from pycocotools import mask as mask_utils
    except ImportError as e:  # pragma: no cover - import-time dep
        raise RuntimeError(
            "pycocotools is required to save SAM 3 labeler annotations. "
            "Install it via `pip install pycocotools`."
        ) from e

    if mask.ndim != 2:
        mask = mask.squeeze()
    fortran = np.asfortranarray(mask.astype(np.uint8))
    rle = mask_utils.encode(fortran)
    counts = rle["counts"]
    if isinstance(counts, bytes):
        counts = counts.decode("ascii")
    return {"counts": counts, "size": [int(rle["size"][0]), int(rle["size"][1])]}


def _decode_rle(seg: Dict[str, Any]) -> np.ndarray:
    """Decode a COCO RLE dict back to a HxW boolean mask.

    Inverse of :func:`_encode_rle`; tolerates a ``str`` ``counts`` (as
    written by the labeler) by re-encoding to bytes for pycocotools.
    """
    try:
        from pycocotools import mask as mask_utils
    except ImportError as e:  # pragma: no cover - import-time dep
        raise RuntimeError(
            "pycocotools is required to load SAM 3 labeler annotations. "
            "Install it via `pip install pycocotools`."
        ) from e

    counts = seg.get("counts")
    if isinstance(counts, str):
        counts = counts.encode("ascii")
    payload = {"counts": counts, "size": list(seg.get("size", []))}
    decoded = mask_utils.decode(payload)
    if decoded.ndim == 3:
        decoded = decoded[..., 0]
    return decoded.astype(bool)


def _xywh_norm_to_xyxy_pix(
    bbox_norm: Sequence[float], width: int, height: int,
) -> List[float]:
    """Convert a normalized xywh bbox to a pixel-space xyxy bbox."""
    if len(bbox_norm) < 4:
        return [0.0, 0.0, 0.0, 0.0]
    x, y, w, h = (float(v) for v in bbox_norm[:4])
    W = float(width)
    H = float(height)
    return [x * W, y * H, (x + w) * W, (y + h) * H]


def _norm_bbox_xywh_from_xyxy_pix(
    box_xyxy_pix: Sequence[float], width: int, height: int
) -> List[float]:
    x1, y1, x2, y2 = box_xyxy_pix
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    w = float(width)
    h = float(height)
    return [
        float(x1) / w,
        float(y1) / h,
        float(x2 - x1) / w,
        float(y2 - y1) / h,
    ]


def _xyxy_pix_to_xywh_norm(
    box_xyxy_pix: Sequence[float], width: int, height: int
) -> List[float]:
    return _norm_bbox_xywh_from_xyxy_pix(box_xyxy_pix, width, height)


def _normalize_points(
    points_xy_pix: Sequence[Sequence[float]],
    labels: Sequence[int],
    width: int,
    height: int,
) -> List[List[float]]:
    out: List[List[float]] = []
    w = float(width)
    h = float(height)
    for (x, y), lab in zip(points_xy_pix, labels):
        out.append([float(x) / w, float(y) / h, int(lab)])
    return out


class AnnotationStore:
    """Write self-contained per-pair JSON shards under ``<output_dir>/pairs/``.

    Each shard is written atomically (``.tmp`` + ``os.replace``) so a
    crash mid-write cannot corrupt an existing shard. There is no
    in-memory accumulator and no monolithic ``annotations.json``: the
    canonical file is produced on demand by :mod:`sam3.labeler.compact`.

    ``fsync`` defaults to True (the labeler's safety guarantee). Inference
    pipelines that write thousands of shards can flip it off to skip the
    per-write ``os.fsync`` (~1-3 ms each on a typical SSD); a partial
    crash then leaves a ``.pair.*.json.tmp`` file that ``compact`` will
    silently skip on the next merge.
    """

    def __init__(self, output_dir: str, *, fsync: bool = True) -> None:
        self.output_dir = os.path.abspath(output_dir)
        self.fsync = bool(fsync)
        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, "vis"), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, "prompts"), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, "pairs"), exist_ok=True)

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #
    def vis_path(self, file_name: str, noun_phrase: str) -> str:
        stem = os.path.splitext(os.path.basename(file_name))[0]
        slug = _slugify(noun_phrase or "np")
        return os.path.join(self.output_dir, "vis", f"{stem}__{slug}.jpg")

    def prompts_path(self, file_name: str, noun_phrase: str) -> str:
        stem = os.path.splitext(os.path.basename(file_name))[0]
        slug = _slugify(noun_phrase or "np")
        return os.path.join(self.output_dir, "prompts", f"{stem}__{slug}.json")

    def pairs_path(self, file_name: str, noun_phrase: str) -> str:
        stem = os.path.splitext(os.path.basename(file_name))[0]
        slug = _slugify(noun_phrase or "np")
        return os.path.join(self.output_dir, "pairs", f"{stem}__{slug}.json")

    def save_pair(
        self,
        file_name: str,
        width: int,
        height: int,
        noun_phrase: str,
        results: Sequence[Dict[str, Any]],
        *,
        is_instance_exhaustive: int = 1,
        is_pixel_exhaustive: int = 1,
        pcs_pos_boxes_xyxy_pix: Sequence[Sequence[float]] = (),
        pcs_neg_boxes_xyxy_pix: Sequence[Sequence[float]] = (),
        pvs_objects: Sequence[Dict[str, Any]] = (),
        mode: str = "pcs",
        raw_prompts: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """Write the shard for a single image-NP pair.

        Re-saving the same ``(file_name, noun_phrase)`` overwrites the
        shard rather than accumulating duplicates. Returns the
        ``images`` entry that was written (its local ``id`` is always
        ``1`` within a shard; canonical IDs are assigned by compact).
        """
        input_box_norm: List[List[float]] = []
        input_box_label: List[int] = []
        for box in pcs_pos_boxes_xyxy_pix:
            input_box_norm.append(_xyxy_pix_to_xywh_norm(box, width, height))
            input_box_label.append(1)
        for box in pcs_neg_boxes_xyxy_pix:
            input_box_norm.append(_xyxy_pix_to_xywh_norm(box, width, height))
            input_box_label.append(0)

        input_points_norm: List[List[float]] = []
        # For PVS we also surface the per-slot box so consumers know it
        # was part of the prompt; SAM 3 train loaders read ``input_box``
        # only at the image level, but adding entries here is harmless.
        for obj in pvs_objects:
            input_points_norm.extend(
                _normalize_points(
                    obj.get("pos_points", []),
                    [1] * len(obj.get("pos_points", [])),
                    width,
                    height,
                )
            )
            input_points_norm.extend(
                _normalize_points(
                    obj.get("neg_points", []),
                    [0] * len(obj.get("neg_points", [])),
                    width,
                    height,
                )
            )
            if obj.get("box") is not None:
                input_box_norm.append(
                    _xyxy_pix_to_xywh_norm(obj["box"], width, height)
                )
                input_box_label.append(1)

        image_entry: Dict[str, Any] = {
            "id": 1,
            "file_name": os.path.basename(file_name),
            "width": int(width),
            "height": int(height),
            "text_input": noun_phrase or "",
            "queried_category": "1",
            "is_instance_exhaustive": int(bool(is_instance_exhaustive)),
            "is_pixel_exhaustive": int(bool(is_pixel_exhaustive)),
            "mode": mode,
        }
        if input_box_norm:
            image_entry["input_box"] = input_box_norm
            image_entry["input_box_label"] = input_box_label
        if input_points_norm:
            image_entry["input_points"] = input_points_norm

        ann_list: List[Dict[str, Any]] = []
        next_ann_id = 1
        source = "sam3-pcs" if mode == "pcs" else "sam3-pvs"
        for res in results:
            mask = res["mask"]
            if mask is None or not np.any(mask):
                continue
            segmentation = _encode_rle(mask)
            bbox_norm = _norm_bbox_xywh_from_xyxy_pix(
                res["box_xyxy_pix"], width, height
            )
            area_norm = float(bbox_norm[2] * bbox_norm[3])
            ann_list.append(
                {
                    "id": next_ann_id,
                    "image_id": 1,
                    "category_id": 1,
                    "iscrowd": 0,
                    "bbox": bbox_norm,
                    "area": area_norm,
                    "segmentation": segmentation,
                    "score": float(res.get("score", 0.0)),
                    "source": res.get("source", source),
                }
            )
            next_ann_id += 1

        shard: Dict[str, Any] = {
            "info": dict(_DEFAULT_INFO),
            "categories": list(_DEFAULT_CATEGORIES),
            "images": [image_entry],
            "annotations": ann_list,
        }
        self._atomic_dump_shard(
            self.pairs_path(file_name, noun_phrase), shard, fsync=self.fsync,
        )

        if raw_prompts is not None:
            self._write_prompts_sidecar(file_name, noun_phrase, raw_prompts)

        return image_entry

    def has_pair(self, file_name: str, noun_phrase: str) -> bool:
        """Cheap check used by the UI to render saved-NP badges."""
        return os.path.exists(self.pairs_path(file_name, noun_phrase))

    def load_pairs_for_image(
        self, file_name: str,
    ) -> Dict[str, Dict[str, Any]]:
        """Read every shard ``pairs/<stem>__*.json`` for ``file_name``.

        Returns a dict keyed by the shard's ``text_input`` (the noun
        phrase) whose value is::

            {
                "shard_path": str,
                "image_size": (W, H),
                "detections": [
                    {
                        "ann_id":      int,   # local id within the shard
                        "box_xyxy_pix": [float, float, float, float],
                        "mask":        np.ndarray[H, W] bool | None,
                        "score":       float,
                        "source":      str,
                    },
                    ...
                ],
                "is_negative": bool,  # True iff the shard has empty annotations[]
            }

        Skips malformed shards (logged + skipped) so a single corrupt
        file cannot break the labeler's image-load path. Decoded masks
        are at the shard's stored resolution; if that disagrees with the
        on-disk image (e.g. someone re-resized the source), the caller
        is responsible for rescaling.
        """
        stem = os.path.splitext(os.path.basename(file_name))[0]
        pairs_dir = os.path.join(self.output_dir, "pairs")
        if not os.path.isdir(pairs_dir):
            return {}

        prefix = f"{stem}__"
        out: Dict[str, Dict[str, Any]] = {}
        for name in sorted(os.listdir(pairs_dir)):
            if not name.endswith(".json") or not name.startswith(prefix):
                continue
            shard_path = os.path.join(pairs_dir, name)
            try:
                with open(shard_path, "r", encoding="utf-8") as f:
                    shard = json.load(f)
            except (OSError, json.JSONDecodeError) as e:
                logger.warning("Skipping unreadable shard %s: %s", shard_path, e)
                continue

            images = shard.get("images") or []
            if not images:
                continue
            img_entry = images[0]
            np_text = img_entry.get("text_input", "") or ""
            W = int(img_entry.get("width", 0) or 0)
            H = int(img_entry.get("height", 0) or 0)
            if W <= 0 or H <= 0:
                logger.warning(
                    "Shard %s has non-positive image_size; skipping.", shard_path,
                )
                continue

            anns = shard.get("annotations") or []
            detections: List[Dict[str, Any]] = []
            for ann in anns:
                ann_id = int(ann.get("id", 0) or 0)
                bbox_norm = ann.get("bbox") or [0.0, 0.0, 0.0, 0.0]
                box_xyxy_pix = _xywh_norm_to_xyxy_pix(bbox_norm, W, H)
                mask = None
                seg = ann.get("segmentation")
                if isinstance(seg, dict) and "counts" in seg and "size" in seg:
                    mask = _decode_rle(seg)
                detections.append(
                    {
                        "ann_id": ann_id,
                        "box_xyxy_pix": box_xyxy_pix,
                        "mask": mask,
                        "score": float(ann.get("score", 0.0) or 0.0),
                        "source": str(ann.get("source", "") or ""),
                    }
                )

            out[np_text] = {
                "shard_path": shard_path,
                "image_size": (W, H),
                "detections": detections,
                "is_negative": len(anns) == 0,
            }
        return out

    def append_pair(
        self,
        file_name: str,
        width: int,
        height: int,
        noun_phrase: str,
        results: Sequence[Dict[str, Any]],
        *,
        drop_ann_ids: Sequence[int] = (),
        is_instance_exhaustive: int = 1,
        is_pixel_exhaustive: int = 1,
        pcs_pos_boxes_xyxy_pix: Sequence[Sequence[float]] = (),
        pcs_neg_boxes_xyxy_pix: Sequence[Sequence[float]] = (),
        pvs_objects: Sequence[Dict[str, Any]] = (),
        mode: str = "pvs",
        raw_prompts: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """Append ``results`` to the existing ``(file_name, noun_phrase)``
        shard, optionally dropping a subset of its existing annotations
        first.

        Used by the labeler's PVS Save flow when the user wants to add
        a new mask to an image-NP pair that already has detections on
        disk (typically because the shard was produced by inference, or
        by a prior PCS save). Compared to :meth:`save_pair`, the key
        difference is that the existing ``images[0]`` row and the kept
        annotations are preserved verbatim - this keeps ``text_input``,
        ``is_instance_exhaustive``, ``is_pixel_exhaustive``, and the
        original ``mode`` flag stable across edits.

        ``drop_ann_ids`` is the set of local annotation ids to remove
        before appending. The labeler passes the subset of
        ``state["pending_delete"]`` scoped to this NP, so a single Save
        click can both delete-and-add atomically.

        If no shard exists yet this method delegates to :meth:`save_pair`
        (so the call site can use ``append_pair`` unconditionally for
        PVS saves).

        Returns the merged shard dict so the caller can refresh its
        in-memory state without a re-read.
        """
        path = self.pairs_path(file_name, noun_phrase)

        if not os.path.exists(path):
            self.save_pair(
                file_name=file_name,
                width=width,
                height=height,
                noun_phrase=noun_phrase,
                results=results,
                is_instance_exhaustive=is_instance_exhaustive,
                is_pixel_exhaustive=is_pixel_exhaustive,
                pcs_pos_boxes_xyxy_pix=pcs_pos_boxes_xyxy_pix,
                pcs_neg_boxes_xyxy_pix=pcs_neg_boxes_xyxy_pix,
                pvs_objects=pvs_objects,
                mode=mode,
                raw_prompts=raw_prompts,
            )
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)

        with open(path, "r", encoding="utf-8") as f:
            shard = json.load(f)

        drop_set = {int(i) for i in drop_ann_ids}
        old_anns = shard.get("annotations") or []
        kept_anns: List[Dict[str, Any]] = []
        for ann in old_anns:
            if int(ann.get("id", 0) or 0) in drop_set:
                continue
            kept_anns.append(dict(ann))
        for new_id, ann in enumerate(kept_anns, start=1):
            ann["id"] = new_id
            ann["image_id"] = 1

        new_anns: List[Dict[str, Any]] = []
        next_ann_id = len(kept_anns) + 1
        default_source = "sam3-pcs" if mode == "pcs" else "sam3-pvs"
        for res in results:
            mask = res.get("mask")
            if mask is None or not np.any(mask):
                continue
            segmentation = _encode_rle(mask)
            bbox_norm = _norm_bbox_xywh_from_xyxy_pix(
                res["box_xyxy_pix"], width, height
            )
            area_norm = float(bbox_norm[2] * bbox_norm[3])
            new_anns.append(
                {
                    "id": next_ann_id,
                    "image_id": 1,
                    "category_id": 1,
                    "iscrowd": 0,
                    "bbox": bbox_norm,
                    "area": area_norm,
                    "segmentation": segmentation,
                    "score": float(res.get("score", 0.0)),
                    "source": res.get("source", default_source),
                }
            )
            next_ann_id += 1

        shard["annotations"] = kept_anns + new_anns
        # Preserve the existing images[0] verbatim. Other top-level
        # fields ("info", "categories") are also left untouched so a
        # shard authored by trt_infer's exporter keeps its provenance
        # info even after a labeler edit.
        self._atomic_dump_shard(path, shard, fsync=self.fsync)

        if raw_prompts is not None:
            self._write_prompts_sidecar(file_name, noun_phrase, raw_prompts)

        return shard

    def write_pair_overrides(
        self,
        file_name: str,
        noun_phrase: str,
        kept_ann_ids: Sequence[int],
    ) -> Dict[str, Any]:
        """Rewrite the ``(file_name, noun_phrase)`` shard, keeping only
        annotations whose local ``id`` is in ``kept_ann_ids``.

        Annotations are renumbered ``1..N`` after filtering so the local
        id space stays contiguous. ``kept_ann_ids`` may be empty: that
        produces a shard with ``annotations: []`` (a "negative" pair),
        rather than removing the file. Callers that want full deletion
        should ``os.remove`` the shard themselves and re-run compact.

        Returns the updated shard dict (useful for the labeler's reload
        path so it can avoid a re-read round-trip).
        """
        path = self.pairs_path(file_name, noun_phrase)
        if not os.path.exists(path):
            raise FileNotFoundError(f"No shard at {path}")

        with open(path, "r", encoding="utf-8") as f:
            shard = json.load(f)

        kept_set = {int(i) for i in kept_ann_ids}
        old_anns = shard.get("annotations") or []
        new_anns: List[Dict[str, Any]] = []
        for ann in old_anns:
            ann_id = int(ann.get("id", 0) or 0)
            if ann_id in kept_set:
                new_anns.append(dict(ann))

        for new_id, ann in enumerate(new_anns, start=1):
            ann["id"] = new_id
            ann["image_id"] = 1

        shard["annotations"] = new_anns
        self._atomic_dump_shard(path, shard, fsync=self.fsync)
        return shard

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
    @staticmethod
    def _atomic_dump_shard(
        path: str, data: Dict[str, Any], *, fsync: bool = True,
    ) -> None:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        fd, tmp = tempfile.mkstemp(
            prefix=".pair.",
            suffix=".json.tmp",
            dir=os.path.dirname(path) or ".",
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
                f.flush()
                if fsync:
                    os.fsync(f.fileno())
            os.replace(tmp, path)
        except Exception:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise

    def _write_prompts_sidecar(
        self, file_name: str, noun_phrase: str, raw_prompts: Dict[str, Any]
    ) -> None:
        path = self.prompts_path(file_name, noun_phrase)
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(raw_prompts, f, indent=2, ensure_ascii=False)
        except OSError as e:
            logger.warning("Could not write prompts sidecar %s: %s", path, e)


class InferenceAnnotationBuilder:
    """In-memory builder for labeler-format ``annotations.json`` used by
    offline inference scripts (``trt_infer``, ``hf_infer``,
    ``parallel_infer``).

    Unlike :class:`AnnotationStore`, this builder does **not** write to
    disk on every pair: callers accumulate in memory and ``dump(path)``
    once at the end of the run. RLE masks are not produced (this is the
    bbox-only sibling intended for offline detection-style precision /
    recall against labeler outputs); extend ``add_pair`` with a ``masks``
    argument if that changes later.

    The output JSON is wire-compatible with
    :class:`AnnotationStore` (same schema as ``annotations.json``), so a
    file produced here can be diffed pairwise against labeler output by
    joining ``images[]`` rows on ``(file_name, text_input)``.
    """

    def __init__(self, tool: str = "sam3-infer", source: str = "sam3-pcs") -> None:
        self.source = source
        self.data: Dict[str, Any] = {
            "info": {**_DEFAULT_INFO, "tool": tool},
            "images": [],
            "annotations": [],
            "categories": list(_DEFAULT_CATEGORIES),
        }
        self._next_img_id = 1
        self._next_ann_id = 1

    def add_pair(
        self,
        file_name: str,
        width: int,
        height: int,
        noun_phrase: str,
        boxes_xyxy_pix: Sequence[Sequence[float]] = (),
        scores: Sequence[float] = (),
        *,
        mode: str = "pcs",
        is_instance_exhaustive: int = 1,
        is_pixel_exhaustive: int = 1,
    ) -> int:
        """Append one (image, NP) pair plus zero-or-more detections.

        ``boxes_xyxy_pix`` is a sequence of length-4 sequences in pixel
        xyxy coordinates of the original image. ``scores`` is a parallel
        sequence of floats. Pass empty sequences for a "negative" pair
        (NP queried but no instances detected); the ``images[]`` row is
        still appended so downstream evaluators can compute false
        negatives.

        Returns the allocated ``image_id``.
        """
        image_id = self._next_img_id
        self._next_img_id += 1

        self.data["images"].append(
            {
                "id": image_id,
                "file_name": os.path.basename(file_name),
                "width": int(width),
                "height": int(height),
                "text_input": noun_phrase,
                "queried_category": "1",
                "is_instance_exhaustive": int(bool(is_instance_exhaustive)),
                "is_pixel_exhaustive": int(bool(is_pixel_exhaustive)),
                "mode": mode,
            }
        )

        for box, score in zip(boxes_xyxy_pix, scores):
            bbox_norm = _norm_bbox_xywh_from_xyxy_pix(box, width, height)
            self.data["annotations"].append(
                {
                    "id": self._next_ann_id,
                    "image_id": image_id,
                    "category_id": 1,
                    "iscrowd": 0,
                    "bbox": bbox_norm,
                    "area": float(bbox_norm[2] * bbox_norm[3]),
                    "score": float(score),
                    "source": self.source,
                }
            )
            self._next_ann_id += 1

        return image_id

    def dump(self, path: str) -> None:
        """Write the accumulated annotations.json atomically to ``path``."""
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        fd, tmp = tempfile.mkstemp(
            prefix=".annotations.", suffix=".json.tmp",
            dir=os.path.dirname(path) or ".",
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(self.data, f, indent=2, ensure_ascii=False)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, path)
        except Exception:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise
