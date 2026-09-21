#!/usr/bin/env python3
"""
TensorRT benchmark for HuggingFace SAM3.

Same interface and NMS pipeline as hf_infer_helper.py, but runs the 2-engine
TRT pipeline (VisionEncoder + DetectorBody) instead of PyTorch.

Usage:
    python export_hf/trt_infer.py --engine_dir export_hf/trt_engines [options]
"""

import argparse
import glob
import json
import os
import struct
import sys
import time
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import torch
from PIL import Image
from tqdm import tqdm

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from export_hf.trt_infer_helper import HFSam3TRTInference, _postprocess_single
from sam3.labeler.annotations import AnnotationStore, InferenceAnnotationBuilder


# ─────────────────────────────────────────────────────────────────────
# Timing
# ─────────────────────────────────────────────────────────────────────

class TimeTracker:
    def __init__(self):
        self.times = defaultdict(float)
        self.counts = defaultdict(int)
        self._start_times = {}

    def start(self, name: str):
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        self._start_times[name] = time.perf_counter()

    def stop(self, name: str) -> float:
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        if name in self._start_times:
            elapsed = time.perf_counter() - self._start_times[name]
            self.times[name] += elapsed
            self.counts[name] += 1
            del self._start_times[name]
            return elapsed
        return 0.0

    def reset(self):
        self.times.clear()
        self.counts.clear()
        self._start_times.clear()

    def get_summary(self) -> str:
        if not self.times:
            return "No timing data collected."
        lines = ["\n" + "=" * 70, "TIMING SUMMARY", "=" * 70]
        total_time = sum(self.times.values())
        for name, total in sorted(self.times.items(), key=lambda x: -x[1]):
            count = self.counts[name]
            avg = total / count if count > 0 else 0
            pct = (total / total_time * 100) if total_time > 0 else 0
            lines.append(
                f"  {name:40s}: {total:8.2f}s total | {avg:7.4f}s avg | {count:4d} calls | {pct:5.1f}%"
            )
        lines.append("-" * 70)
        lines.append(f"  {'TOTAL':40s}: {total_time:8.2f}s")
        lines.append("=" * 70)
        return "\n".join(lines)


timer = TimeTracker()


# ─────────────────────────────────────────────────────────────────────
# Default classes / NMS config (matching hf_infer.py)
# ─────────────────────────────────────────────────────────────────────

INDOOR_CLASSES = [
    # "elevator",
    # "red fire extinguisher box",
    # "red fire hydrant identification sign",
    # "emergency exit sign",
    "floor",
    "grey carpet",
    "door",
    "person",
]

OUTDOOR_CLASSES = [
    "road",
    "sidewalk",
    "person",
    "car",
    "truck",
    "van",
    "bus",
    "grass",
]


# ─────────────────────────────────────────────────────────────────────
# CUDA helpers
# ─────────────────────────────────────────────────────────────────────

def _is_jetson() -> bool:
    """Detect Jetson (Tegra) platforms where CPU and GPU share unified memory."""
    try:
        with open("/etc/nv_tegra_release") as f:
            return True
    except FileNotFoundError:
        pass
    try:
        with open("/proc/device-tree/compatible", "rb") as f:
            return b"tegra" in f.read()
    except FileNotFoundError:
        return False


def _get_process_rss_mib() -> float:
    """Return current process RSS in MiB from /proc/self/status."""
    try:
        with open("/proc/self/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1]) / 1024.0  # kB → MiB
    except Exception:
        pass
    return -1.0


def _get_system_memory_mib():
    """Parse /proc/meminfo → (total, available, used) in MiB, or None."""
    try:
        info = {}
        with open("/proc/meminfo") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 2:
                    info[parts[0].rstrip(":")] = int(parts[1])  # kB
        total = info.get("MemTotal", 0) / 1024.0
        avail = info.get("MemAvailable", 0) / 1024.0
        return total, avail, total - avail
    except Exception:
        return None


def log_cuda_memory(device: torch.device, heading: str):
    if not torch.cuda.is_available():
        return
    torch.cuda.synchronize(device)

    gpu_index = device.index if device.index is not None else 0
    to_mib = 1.0 / (1024 ** 2)
    is_jetson = _is_jetson()

    pt_alloc = torch.cuda.memory_allocated(device) * to_mib
    pt_reserved = torch.cuda.memory_reserved(device) * to_mib
    pt_peak_alloc = torch.cuda.max_memory_allocated(device) * to_mib
    pt_peak_reserved = torch.cuda.max_memory_reserved(device) * to_mib

    # Driver-level stats via pynvml (covers everything: TRT engines, CUDA
    # context, cuDNN workspaces, PyTorch, and any other native allocations).
    # NVML is unavailable on Jetson (Tegra) — fall back to /proc/meminfo.
    nvml_used = nvml_total = nvml_free = None
    try:
        import pynvml
        pynvml.nvmlInit()
        handle = pynvml.nvmlDeviceGetHandleByIndex(gpu_index)
        mem_info = pynvml.nvmlDeviceGetMemoryInfo(handle)
        nvml_total = mem_info.total * to_mib
        nvml_used = mem_info.used * to_mib
        nvml_free = mem_info.free * to_mib
        pynvml.nvmlShutdown()
    except Exception:
        pass

    print(f"\n{heading}")
    print(f"  [PyTorch caching allocator]")
    print(f"    allocated (now):           {pt_alloc:10.1f} MiB")
    print(f"    reserved  (now):           {pt_reserved:10.1f} MiB")
    print(f"    max_allocated (peak):      {pt_peak_alloc:10.1f} MiB")
    print(f"    max_reserved  (peak):      {pt_peak_reserved:10.1f} MiB")
    non_pt = pt_reserved - pt_alloc
    print(f"    fragmentation (res-alloc): {non_pt:10.1f} MiB")
    if nvml_used is not None:
        if is_jetson:
            print(f"  [Unified memory (pynvml) — Jetson GPU {gpu_index}]")
            print(f"    unified pool total:        {nvml_total:10.1f} MiB")
            print(f"    unified pool used:         {nvml_used:10.1f} MiB")
            print(f"    unified pool free:         {nvml_free:10.1f} MiB")
        else:
            print(f"  [Driver-level (pynvml) — GPU {gpu_index}]")
            print(f"    total:                     {nvml_total:10.1f} MiB")
            print(f"    used:                      {nvml_used:10.1f} MiB")
            print(f"    free:                      {nvml_free:10.1f} MiB")
            outside_pt = nvml_used - pt_reserved
            print(f"    non-PyTorch (TRT, ctx...): {outside_pt:10.1f} MiB")
    elif is_jetson:
        sys_mem = _get_system_memory_mib()
        if sys_mem:
            total, avail, used = sys_mem
            print(f"  [Unified memory (/proc/meminfo) — Jetson]")
            print(f"    system total:              {total:10.1f} MiB")
            print(f"    system used:               {used:10.1f} MiB")
            print(f"    system available:           {avail:10.1f} MiB")
    else:
        print(f"  [pynvml not available — install nvidia-ml-py for full GPU memory stats]")
    if is_jetson:
        rss = _get_process_rss_mib()
        if rss >= 0:
            print(f"  [Process RSS (this process)]")
            print(f"    RSS:                       {rss:10.1f} MiB")


# ─────────────────────────────────────────────────────────────────────
# Visualization  (same as hf_infer.py)
# ─────────────────────────────────────────────────────────────────────

COLORS = [
    (0.122, 0.467, 0.706), (1.0, 0.498, 0.055), (0.173, 0.627, 0.173),
    (0.839, 0.153, 0.157), (0.580, 0.404, 0.741), (0.549, 0.337, 0.294),
    (0.890, 0.467, 0.761), (0.894, 0.102, 0.329), (0.737, 0.741, 0.133),
    (0.090, 0.745, 0.812),
]


def color_to_bgr(color) -> Tuple[int, int, int]:
    r = int(color[0] * 255) if color[0] <= 1.0 else int(color[0])
    g = int(color[1] * 255) if color[1] <= 1.0 else int(color[1])
    b = int(color[2] * 255) if color[2] <= 1.0 else int(color[2])
    return (b, g, r)


def draw_mask_cv2(img, mask, color_bgr, alpha=0.5):
    if isinstance(mask, torch.Tensor):
        mask = mask.cpu().numpy()
    mask = mask.astype(bool)
    overlay = img.copy()
    overlay[mask] = color_bgr
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)


def draw_bbox_cv2(img, box, text, color_bgr, thickness=4):
    x1, y1, x2, y2 = map(int, box[:4])
    cv2.rectangle(img, (x1, y1), (x2, y2), color_bgr, thickness)
    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale, font_thickness = 0.5, 1
    (tw, th), _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    pad = 8
    cv2.rectangle(img, (x1, y1 - th - pad * 2), (x1 + tw + pad * 2, y1), color_bgr, -1)
    cv2.putText(img, text, (x1 + pad, y1 - pad), font, font_scale, (0, 0, 0), font_thickness + 2, cv2.LINE_AA)
    cv2.putText(img, text, (x1 + pad, y1 - pad), font, font_scale, (255, 255, 255), font_thickness, cv2.LINE_AA)


def save_visualization(
    image, result: Dict, class_list: List[str], output_path: str,
    *,
    cpu_payload: Optional[Dict[str, Dict[str, Any]]] = None,
):
    """Draw masks + boxes on image and save.

    Args:
        image: PIL Image (RGB) or numpy array (BGR, from cv2.imdecode).
        cpu_payload: Optional output of ``_extract_per_np_cpu_payload`` so
            visualization re-uses the same CPU mask buffers the shard
            saver consumes (avoids a duplicate GPU->CPU transfer when
            both ``--save_for_labeler`` and visualization are on).
    """
    all_results = result["results_by_class"]
    total_objects = result["total_objects"]

    if isinstance(image, np.ndarray):
        img = image if image.ndim == 3 and image.shape[2] == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    else:
        img = np.array(image)
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        elif img.shape[2] == 4:
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        else:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    vis_img = img.copy()

    if total_objects > 0:
        obj_idx = 0
        legend = []
        for class_name, data in all_results.items():
            count = data["count"]
            cpu_data = cpu_payload.get(class_name) if cpu_payload else None
            if cpu_data is not None:
                masks_iter = cpu_data["masks"]
                boxes_iter = cpu_data["boxes"]
                scores_iter = cpu_data["scores"]
                for i in range(count):
                    c_bgr = color_to_bgr(COLORS[obj_idx % len(COLORS)])
                    obj_idx += 1
                    mask_2d = masks_iter[i]
                    while mask_2d.ndim > 2:
                        mask_2d = mask_2d[0]
                    draw_mask_cv2(vis_img, mask_2d, c_bgr)
                    box_flat = np.asarray(boxes_iter[i]).flatten()
                    prob = float(scores_iter[i])
                    draw_bbox_cv2(
                        vis_img, box_flat, f"{class_name}: {prob:.2f}", c_bgr,
                    )
            else:
                masks, boxes, scores = data["masks"], data["boxes"], data["scores"]
                for i in range(count):
                    c_bgr = color_to_bgr(COLORS[obj_idx % len(COLORS)])
                    obj_idx += 1
                    mask = masks[i] if count > 1 else masks
                    box = boxes[i] if count > 1 else boxes
                    score = scores[i] if count > 1 else scores
                    mask_2d = mask.squeeze().cpu().numpy() if isinstance(mask, torch.Tensor) else np.array(mask).squeeze()
                    while mask_2d.ndim > 2:
                        mask_2d = mask_2d[0]
                    draw_mask_cv2(vis_img, mask_2d, c_bgr)
                    prob = score.item() if hasattr(score, "item") else float(score)
                    box_flat = box.cpu().numpy().squeeze().flatten() if isinstance(box, torch.Tensor) else np.array(box).squeeze().flatten()
                    draw_bbox_cv2(vis_img, box_flat, f"{class_name}: {prob:.2f}", c_bgr)
            legend.append(f"{class_name}: {count}")
        title = f"Total: {total_objects} | {', '.join(legend)}"
    else:
        title = "No objects detected"

    font = cv2.FONT_HERSHEY_DUPLEX
    (tw, th), _ = cv2.getTextSize(title, font, 1.0, 1)
    pad = 15
    cv2.rectangle(vis_img, (0, 0), (tw + pad * 2, th + pad * 2), (0, 0, 0), -1)
    cv2.putText(vis_img, title, (pad, th + pad), font, 1.0, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(output_path, vis_img)


# ─────────────────────────────────────────────────────────────────────
# GPU-accelerated NMS  (avoids CPU transfers; masks stay on GPU)
# ─────────────────────────────────────────────────────────────────────

NMS_EXCLUDE_PAIRS: set = set()

def _to_float_score(s):
    return s.item() if hasattr(s, "item") else float(s)


def _gather_class_tensors(
    data: Dict,
) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Normalize a per-class result dict so outputs are [K, ...] tensors,
    handling the count==1 unwrapped convention."""
    masks, boxes, scores = data["masks"], data["boxes"], data["scores"]
    if data["count"] == 1:
        if isinstance(masks, torch.Tensor) and masks.dim() == 2:
            masks = masks.unsqueeze(0)
        if isinstance(boxes, torch.Tensor) and boxes.dim() == 1:
            boxes = boxes.unsqueeze(0)
        if isinstance(scores, torch.Tensor) and scores.dim() == 0:
            scores = scores.unsqueeze(0)
    return masks, boxes, scores


def _box_iou_gpu(
    a: torch.Tensor, b: torch.Tensor,
) -> torch.Tensor:
    """Pairwise box IoU on GPU.  a: [M, 4] xyxy, b: [N, 4] xyxy → [M, N]."""
    x1 = torch.max(a[:, 0].unsqueeze(1), b[:, 0].unsqueeze(0))
    y1 = torch.max(a[:, 1].unsqueeze(1), b[:, 1].unsqueeze(0))
    x2 = torch.min(a[:, 2].unsqueeze(1), b[:, 2].unsqueeze(0))
    y2 = torch.min(a[:, 3].unsqueeze(1), b[:, 3].unsqueeze(0))
    inter = (x2 - x1).clamp(min=0) * (y2 - y1).clamp(min=0)
    area_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    union = area_a.unsqueeze(1) + area_b.unsqueeze(0) - inter
    return inter / union.clamp(min=1e-6)


def filter_intra_class_nms(
    result: Dict[str, Any], iou_threshold: float = 0.5,
) -> Dict[str, Any]:
    """Per-class greedy box NMS on GPU (no CPU transfers)."""
    if result["total_objects"] == 0:
        return result

    new_by_class: Dict[str, Dict] = {}
    total_kept = 0

    for class_name, data in result["results_by_class"].items():
        masks, boxes, scores = _gather_class_tensors(data)
        K = scores.shape[0]

        order = scores.argsort(descending=True)
        masks, boxes, scores = masks[order], boxes[order], scores[order]

        keep_idx: list = []
        for i in range(K):
            if keep_idx:
                kept_boxes = boxes[keep_idx]             # [N_kept, 4]
                ious = _box_iou_gpu(boxes[i : i + 1], kept_boxes)  # [1, N_kept]
                if ious.max().item() > iou_threshold:
                    continue
            keep_idx.append(i)

        if keep_idx:
            idx = torch.tensor(keep_idx, device=masks.device, dtype=torch.long)
            m, b, s = masks[idx], boxes[idx], scores[idx]
            c = len(keep_idx)
            if c == 1:
                m, b, s = m.squeeze(0), b.squeeze(0), s.squeeze(0)
            new_by_class[class_name] = {
                "masks": m, "boxes": b, "scores": s, "count": c,
            }
            total_kept += c

    removed = result["total_objects"] - total_kept
    return {
        "results_by_class": new_by_class,
        "total_objects": total_kept,
        "classes_detected": list(new_by_class.keys()),
        "intra_nms_removed": removed,
        "nms_removed": result.get("nms_removed", 0),
    }


def filter_cross_class_nms(
    result: Dict[str, Any],
    image_hw: Tuple[int, int],
    iou_threshold: float = 0.8,
    proximity_threshold: float = 20.0,
    keep_larger: bool = True,
    min_mask_size_ratio: float = 0.0001,
    exclude_pairs: set = NMS_EXCLUDE_PAIRS,
    image_name: str = "",
    debug: bool = False,
) -> Dict[str, Any]:
    """Cross-class greedy mask+proximity NMS on GPU.

    Mask IoU is computed via a GPU matmul over flattened boolean masks,
    avoiding costly GPU→CPU transfers.  The greedy suppression loop stays
    in Python (trivial for ≤50 detections) but every IoU / distance
    comparison reads from pre-computed GPU tensors.
    """
    if result["total_objects"] == 0:
        return result

    # ── 1. Gather all detections into GPU tensors ─────────────────
    all_scores: list = []
    all_classes: list = []
    all_masks_list: list = []
    all_boxes_list: list = []

    for class_name, data in result["results_by_class"].items():
        masks, boxes, scores = _gather_class_tensors(data)
        for i in range(data["count"]):
            all_scores.append(scores[i])
            all_classes.append(class_name)
            all_masks_list.append(masks[i])
            all_boxes_list.append(boxes[i])

    if not all_scores:
        return result

    K = len(all_scores)
    device = all_masks_list[0].device

    score_tensor = torch.stack(all_scores)          # [K]
    order = score_tensor.argsort(descending=True)
    order_list = order.tolist()

    all_classes = [all_classes[i] for i in order_list]
    all_masks = torch.stack([all_masks_list[i] for i in order_list])   # [K, H, W]
    all_boxes = torch.stack([all_boxes_list[i] for i in order_list])   # [K, 4]
    all_scores_sorted = score_tensor[order]                            # [K]

    # Pre-compute flattened masks and areas (one GPU op each)
    masks_flat = all_masks.reshape(K, -1).float()   # [K, H*W]
    mask_areas = masks_flat.sum(dim=1)               # [K]

    # Pre-compute box geometry
    box_cx = (all_boxes[:, 0] + all_boxes[:, 2]) * 0.5
    box_cy = (all_boxes[:, 1] + all_boxes[:, 3]) * 0.5
    box_areas = (all_boxes[:, 2] - all_boxes[:, 0]) * (all_boxes[:, 3] - all_boxes[:, 1])

    total_pixels = image_hw[0] * image_hw[1]
    min_mask_px = total_pixels * min_mask_size_ratio

    # ── 2. Greedy suppression loop ────────────────────────────────
    kept_indices: list = []

    for i in range(K):
        if mask_areas[i].item() < min_mask_px:
            continue

        cn = all_classes[i]

        if not kept_indices:
            kept_indices.append(i)
            continue

        ki = torch.tensor(kept_indices, device=device, dtype=torch.long)

        # Vectorised mask IoU: [N_kept] via matmul
        inter = masks_flat[ki] @ masks_flat[i]                 # [N_kept]
        unions = mask_areas[i] + mask_areas[ki] - inter
        mask_ious = inter / unions.clamp(min=1)

        # Vectorised Euclidean distances: [N_kept]
        dists = torch.sqrt(
            (box_cx[i] - box_cx[ki]) ** 2
            + (box_cy[i] - box_cy[ki]) ** 2
        )

        keep = True
        to_remove: list = []
        cur_area = box_areas[i].item()

        for j_local, j_global in enumerate(kept_indices):
            k_cn = all_classes[j_global]

            if exclude_pairs and frozenset({cn, k_cn}) in exclude_pairs:
                continue

            if mask_ious[j_local].item() > iou_threshold:
                keep = False
                break

            if dists[j_local].item() < proximity_threshold:
                k_area = box_areas[j_global].item()
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

    # ── 3. Rebuild per-class output dict ──────────────────────────
    new_by_class: Dict[str, Dict] = {}
    for i in kept_indices:
        cn = all_classes[i]
        if cn not in new_by_class:
            new_by_class[cn] = {"masks": [], "boxes": [], "scores": []}
        new_by_class[cn]["masks"].append(all_masks[i])
        new_by_class[cn]["boxes"].append(all_boxes[i])
        new_by_class[cn]["scores"].append(all_scores_sorted[i])

    for data in new_by_class.values():
        count = len(data["masks"])
        data["count"] = count
        if count == 1:
            data["masks"] = data["masks"][0]
            data["boxes"] = data["boxes"][0]
            data["scores"] = data["scores"][0]
        else:
            data["masks"] = torch.stack(data["masks"])
            data["boxes"] = torch.stack(data["boxes"])
            data["scores"] = torch.stack(data["scores"])

    total_kept = sum(d["count"] for d in new_by_class.values())
    removed = result["total_objects"] - total_kept
    return {
        "results_by_class": new_by_class,
        "total_objects": total_kept,
        "classes_detected": list(new_by_class.keys()),
        "nms_removed": removed,
    }


# ─────────────────────────────────────────────────────────────────────
# Per-class boxes/scores extraction for the predictions.json writer
# ─────────────────────────────────────────────────────────────────────

def _collect_boxes_scores(
    result: Dict[str, Any], class_name: str,
) -> Tuple[List[List[float]], List[float]]:
    """Return ``([xyxy_pix, ...], [score, ...])`` for one queried class.

    Handles the ``count == 1`` collapse convention used by
    ``results_by_class`` (where ``boxes`` / ``scores`` are single
    tensors rather than length-1 sequences). Returns empty lists when
    the class was queried but produced no detections, so the caller can
    still record a negative ``images[]`` row.
    """
    data = result.get("results_by_class", {}).get(class_name)
    if data is None:
        return [], []

    count = data["count"]
    boxes_out: List[List[float]] = []
    scores_out: List[float] = []
    for i in range(count):
        box = data["boxes"][i] if count > 1 else data["boxes"]
        score = data["scores"][i] if count > 1 else data["scores"]
        if isinstance(box, torch.Tensor):
            box_list = box.detach().float().cpu().numpy().squeeze().flatten().tolist()
        else:
            box_list = np.asarray(box).squeeze().flatten().tolist()
        boxes_out.append([float(v) for v in box_list[:4]])
        scores_out.append(_to_float_score(score))
    return boxes_out, scores_out


def extract_detections(result: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Backward-compatible per-detection dump (class / score / pixel xyxy bbox).

    Retained for external callers like ``sam3/export_hf/ros2_trt_node.py``
    that pre-date the labeler-format annotations writer. Internally
    delegates to :func:`_collect_boxes_scores`. Prefer
    :class:`sam3.labeler.annotations.InferenceAnnotationBuilder` for
    new code paths that need to be diffed against labeler outputs.
    """
    detections: List[Dict[str, Any]] = []
    for class_name in result.get("results_by_class", {}):
        boxes_pix, scores = _collect_boxes_scores(result, class_name)
        for box, score in zip(boxes_pix, scores):
            detections.append({
                "class": class_name,
                "score": round(score, 4),
                "bbox": [round(v, 2) for v in box],
            })
    return detections


# ─────────────────────────────────────────────────────────────────────
# Async I/O prefetcher
# ─────────────────────────────────────────────────────────────────────

def _read_file_and_size(
    path: str, decode_bgr: bool = False,
) -> Tuple[bytes, Tuple[int, int], Optional[np.ndarray]]:
    """Read raw image bytes and extract (H, W).

    For JPEG, reads the SOF0 marker to get dimensions (~0.01ms).
    Falls back to cv2 full decode for other formats.

    Args:
        decode_bgr: If True, also return the cv2 BGR decoded image
                    (useful for visualization — decoding happens in
                    the background thread, overlapped with GPU compute).

    Returns:
        (raw_bytes, (height, width), bgr_or_None)
    """
    with open(path, "rb") as f:
        data = f.read()

    bgr = None
    hw = _jpeg_dimensions(data)

    if hw is None or decode_bgr:
        arr = np.frombuffer(data, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is not None and hw is None:
            hw = (bgr.shape[0], bgr.shape[1])

    if hw is None:
        img = Image.open(__import__("io").BytesIO(data))
        hw = (img.height, img.width)

    return data, hw, bgr


def _jpeg_dimensions(data: bytes) -> Optional[Tuple[int, int]]:
    """Extract (height, width) from JPEG SOFn marker without decoding."""
    if len(data) < 4 or data[0] != 0xFF or data[1] != 0xD8:
        return None
    i = 2
    while i < len(data) - 9:
        if data[i] != 0xFF:
            return None
        marker = data[i + 1]
        if marker in (0xC0, 0xC1, 0xC2):
            h = struct.unpack(">H", data[i + 5 : i + 7])[0]
            w = struct.unpack(">H", data[i + 7 : i + 9])[0]
            return (h, w)
        length = struct.unpack(">H", data[i + 2 : i + 4])[0]
        i += 2 + length
    return None


class PrefetchLoader:
    """Reads the next image's raw bytes in a background thread.

    While the GPU processes image N, the I/O thread reads image N+1
    from disk, hiding file I/O (and optional CPU decode for
    visualization) behind GPU compute.
    """

    def __init__(
        self, file_paths: List[str], num_workers: int = 1,
        decode_bgr: bool = False,
    ):
        self.paths = file_paths
        self.decode_bgr = decode_bgr
        self.executor = ThreadPoolExecutor(max_workers=num_workers)
        self._future = None

    def kick(self, idx: int):
        """Submit a read for paths[idx] in the background."""
        if 0 <= idx < len(self.paths):
            self._future = self.executor.submit(
                _read_file_and_size, self.paths[idx], self.decode_bgr,
            )

    def get(self) -> Optional[Tuple[bytes, Tuple[int, int], Optional[np.ndarray]]]:
        """Block until the prefetched result is ready and return it."""
        if self._future is not None:
            result = self._future.result()
            self._future = None
            return result
        return None

    def shutdown(self):
        self.executor.shutdown(wait=False)


# ─────────────────────────────────────────────────────────────────────
# Background shard saver (writes labeler-format per-pair JSON shards)
#
# The labeler at ``sam3/sam3/labeler/app.py`` consumes one
# ``pairs/<stem>__<np_slug>.json`` per (image, noun-phrase) pair.
# When the user passes ``--save_for_labeler``, we want to emit those
# shards from inference *without* slowing the GPU loop down.
#
# Design:
#  - Main thread does the *only* unavoidable GPU work: cast to uint8 +
#    ``.cpu().numpy()`` once per image. This is the same transfer cost
#    that ``save_visualization`` already pays (it calls ``mask.cpu()``
#    inside ``draw_mask_cv2``); when both paths are on we share the
#    result.
#  - Worker threads do RLE encoding + JSON serialization + atomic
#    rename, all of which are pure CPU and trivially parallel across
#    images.
#  - ``drain()`` is called once before the timing summary so we surface
#    any worker exceptions and the timing summary doesn't include
#    exit-shutdown wait.
# ─────────────────────────────────────────────────────────────────────


def _extract_per_np_cpu_payload(
    result: Dict[str, Any],
) -> Dict[str, Dict[str, Any]]:
    """Move post-NMS GPU tensors to CPU once per image, structured per NP.

    Mirrors the ``count == 1`` collapse convention used by
    ``results_by_class`` (see :func:`_gather_class_tensors`); the output
    always has ``masks`` as ``np.ndarray[K, H, W]`` ``uint8``, ``boxes``
    as ``List[List[float]]`` xyxy_pix, and ``scores`` as
    ``List[float]``.
    """
    out: Dict[str, Dict[str, Any]] = {}
    for class_name, data in result.get("results_by_class", {}).items():
        masks, boxes, scores = _gather_class_tensors(data)
        if isinstance(masks, torch.Tensor):
            masks_cpu = masks.to(torch.uint8).cpu().numpy()
        else:
            masks_cpu = np.asarray(masks).astype(np.uint8)
        if masks_cpu.ndim == 2:
            masks_cpu = masks_cpu[None, ...]

        if isinstance(boxes, torch.Tensor):
            boxes_cpu = boxes.detach().float().cpu().numpy()
        else:
            boxes_cpu = np.asarray(boxes, dtype=np.float32)
        if boxes_cpu.ndim == 1:
            boxes_cpu = boxes_cpu[None, ...]
        boxes_list = [
            [float(v) for v in boxes_cpu[i, :4].tolist()]
            for i in range(boxes_cpu.shape[0])
        ]

        if isinstance(scores, torch.Tensor):
            scores_list = [float(v) for v in scores.detach().cpu().tolist()]
        else:
            scores_list = [_to_float_score(s) for s in (scores if hasattr(scores, "__iter__") else [scores])]

        out[class_name] = {
            "masks": masks_cpu,
            "boxes": boxes_list,
            "scores": scores_list,
        }
    return out


class ShardSaver:
    """Background writer for labeler-format per-pair JSON shards.

    Submit ``(image_path, height, width, per_np_data, queried_classes)``
    after each image's post-NMS step; workers RLE-encode masks and write
    one shard per (image, NP) pair to ``<output_dir>/pairs/`` plus the
    ``<output_dir>/prompts/`` sidecar describing the queried class list.
    """

    def __init__(
        self,
        output_dir: str,
        *,
        num_workers: int = 2,
        fsync: bool = False,
        tool_name: str = "sam3-trt-infer",
        source: str = "trt-pcs",
    ) -> None:
        self.store = AnnotationStore(output_dir, fsync=fsync)
        self.tool_name = tool_name
        self.source = source
        self.executor = ThreadPoolExecutor(
            max_workers=max(1, int(num_workers)),
            thread_name_prefix="sam3-shard-saver",
        )
        self._futures: list = []
        self._submit_count = 0
        self._error_count = 0

    def submit(
        self,
        image_path: str,
        width: int,
        height: int,
        per_np_data: Dict[str, Dict[str, Any]],
        queried_classes: List[str],
        *,
        mode: str = "pcs",
    ) -> None:
        """Submit one image's worth of work to the worker pool.

        Negative pairs (queried but ``count == 0``) still produce a shard
        with ``annotations: []`` so downstream evaluators can compute
        false negatives.
        """
        self._submit_count += 1
        f = self.executor.submit(
            self._write_one,
            image_path, int(width), int(height),
            per_np_data, list(queried_classes), mode,
        )
        self._futures.append(f)

    def drain(self) -> None:
        """Wait for all submitted work to finish; surface any exceptions."""
        for f in self._futures:
            try:
                f.result()
            except Exception as e:
                self._error_count += 1
                print(f"[ShardSaver] worker error: {e}")
        self._futures.clear()
        self.executor.shutdown(wait=True)

    def stats(self) -> str:
        return (
            f"submitted={self._submit_count}, errors={self._error_count}"
        )

    def _write_one(
        self,
        image_path: str,
        width: int,
        height: int,
        per_np_data: Dict[str, Dict[str, Any]],
        queried_classes: List[str],
        mode: str,
    ) -> None:
        for cn in queried_classes:
            data = per_np_data.get(cn)
            if data is None:
                results: List[Dict[str, Any]] = []
            else:
                masks = data["masks"]
                boxes = data["boxes"]
                scores = data["scores"]
                results = []
                for i in range(masks.shape[0]):
                    results.append(
                        {
                            "mask": masks[i].astype(bool),
                            "box_xyxy_pix": boxes[i],
                            "score": scores[i],
                            "source": self.source,
                        }
                    )

            self.store.save_pair(
                file_name=image_path,
                width=width,
                height=height,
                noun_phrase=cn,
                results=results,
                is_instance_exhaustive=1,
                is_pixel_exhaustive=1,
                mode=mode,
                raw_prompts={
                    "image_path": image_path,
                    "image_size": (int(width), int(height)),
                    "mode": mode,
                    "noun_phrase": cn,
                    "tool": self.tool_name,
                    "source": self.source,
                    "queried_classes": list(queried_classes),
                },
            )


# ─────────────────────────────────────────────────────────────────────
# TRT benchmark processor (wraps HFSam3TRTInference with timing)
# ─────────────────────────────────────────────────────────────────────

class TRTBenchmarkProcessor:
    """Wraps HFSam3TRTInference with per-stage timing."""

    def __init__(self, engine: HFSam3TRTInference):
        self.engine = engine

    @torch.inference_mode()
    def process_image(
        self, pil_image: Image.Image, class_list: List[str],
    ) -> Dict[str, Any]:
        """Original PIL-based path (used for warmup and fallback)."""
        h, w = pil_image.height, pil_image.width
        N = len(class_list)

        timer.start("trt_preprocess")
        if self.engine.gpu_preprocessor is not None:
            pixel_values = self.engine.gpu_preprocessor.preprocess_from_pil(pil_image)
        else:
            img_inputs = self.engine.processor(
                images=pil_image, return_tensors="pt",
            ).to(self.engine.device)
            pixel_values = img_inputs.pixel_values
        timer.stop("trt_preprocess")

        return self._run_engines(pixel_values, class_list, (h, w), N)

    @torch.inference_mode()
    def process_image_bytes(
        self, raw_bytes: bytes, original_hw: Tuple[int, int], class_list: List[str],
    ) -> Dict[str, Any]:
        """nvJPEG + async path: decode from raw bytes on GPU."""
        h, w = original_hw
        N = len(class_list)

        timer.start("trt_gpu_decode")
        if self.engine.gpu_preprocessor is not None:
            pixel_values = self.engine.gpu_preprocessor.preprocess_from_bytes(raw_bytes)
        else:
            img = Image.open(__import__("io").BytesIO(raw_bytes)).convert("RGB")
            img_inputs = self.engine.processor(
                images=img, return_tensors="pt",
            ).to(self.engine.device)
            pixel_values = img_inputs.pixel_values
        timer.stop("trt_gpu_decode")

        return self._run_engines(pixel_values, class_list, (h, w), N)

    def _run_engines(
        self, pixel_values: torch.Tensor, class_list: List[str],
        original_hw: Tuple[int, int], N: int,
    ) -> Dict[str, Any]:
        timer.start("trt_vision_encode")
        vision_out = self.engine.vision_engine.infer({"pixel_values": pixel_values})
        timer.stop("trt_vision_encode")

        timer.start("trt_vision_repeat")
        fpn0 = vision_out["fpn_feat_0"].repeat(N, 1, 1, 1)
        fpn1 = vision_out["fpn_feat_1"].repeat(N, 1, 1, 1)
        fpn2 = vision_out["fpn_feat_2"].repeat(N, 1, 1, 1)
        pos2 = vision_out["fpn_pos_2"].repeat(N, 1, 1, 1)
        timer.stop("trt_vision_repeat")

        text = self.engine.cache_text_inputs(class_list)

        timer.start("trt_detector_forward")
        det_out = self.engine.detector_engine.infer({
            "fpn_feat_0": fpn0, "fpn_feat_1": fpn1,
            "fpn_feat_2": fpn2, "fpn_pos_2": pos2,
            "input_ids": text["input_ids"],
            "attention_mask": text["attention_mask"],
        })
        timer.stop("trt_detector_forward")

        timer.start("trt_postprocess")
        results_by_class = {}
        total_objects = 0
        for i, cn in enumerate(class_list):
            res = _postprocess_single(
                det_out["pred_masks"][i], det_out["pred_boxes"][i],
                det_out["pred_logits"][i], det_out["presence_logits"][i],
                original_hw, self.engine.confidence_threshold,
            )
            n = len(res["scores"]) if isinstance(res["scores"], (list, torch.Tensor)) else 0
            if isinstance(res["scores"], torch.Tensor):
                n = res["scores"].numel()
            if n > 0:
                results_by_class[cn] = {
                    "masks": res["masks"], "boxes": res["boxes"],
                    "scores": res["scores"], "count": n,
                }
                total_objects += n
        timer.stop("trt_postprocess")

        return {
            "results_by_class": results_by_class,
            "total_objects": total_objects,
            "classes_detected": list(results_by_class.keys()),
        }


# ─────────────────────────────────────────────────────────────────────
# Main processing loop
# ─────────────────────────────────────────────────────────────────────

def process_all_images(
    input_dir: str,
    output_dir: str,
    class_list: List[str],
    engine_dir: str,
    model_name: str,
    confidence_threshold: float = 0.5,
    gpu_id: int = 0,
    stride: int = 1,
    offset: int = 0,
    intra_nms_threshold: float = 0.0,
    nms_iou_threshold: float = 0.0,
    nms_proximity_threshold: float = 20.0,
    nms_min_mask_ratio: float = 0.0001,
    debug_nms: bool = False,
    save_vis: bool = True,
    max_images: int = 500,
    save_for_labeler: bool = False,
    labeler_output_dir: Optional[str] = None,
    saver_workers: int = 2,
    saver_no_fsync: bool = True,
):
    os.makedirs(output_dir, exist_ok=True)
    device = torch.device(f"cuda:{gpu_id}")

    print(f"Loading TRT engines from {engine_dir}")
    trt_engine = HFSam3TRTInference(
        engine_dir=engine_dir,
        model_name=model_name,
        confidence_threshold=confidence_threshold,
        device=str(device),
        use_gpu_preprocess=True,
    )
    proc = TRTBenchmarkProcessor(trt_engine)

    if torch.cuda.is_available():
        log_cuda_memory(device, "CUDA memory after engine load")
        torch.cuda.reset_peak_memory_stats(device)

    image_extensions = ["*.jpg", "*.jpeg", "*.png", "*.bmp"]
    image_files = []
    for ext in image_extensions:
        image_files.extend(glob.glob(os.path.join(input_dir, ext)))
    image_files = sorted(image_files)

    if not image_files:
        print(f"No images found in {input_dir}")
        return

    total_images = len(image_files)
    image_files = image_files[offset::stride]
    image_files = image_files[:max_images]

    print(f"\nFound {total_images} images, processing {len(image_files)} (offset={offset}, stride={stride})")
    print(f"Classes: {class_list}")
    print(f"Confidence: {confidence_threshold}")
    print(f"GPU: {gpu_id}")
    if intra_nms_threshold > 0:
        print(f"Intra-class NMS: box IoU > {intra_nms_threshold}")
    if nms_iou_threshold > 0:
        print(f"Cross-class NMS: IoU > {nms_iou_threshold}, proximity < {nms_proximity_threshold}px, "
              f"min_mask_ratio = {nms_min_mask_ratio}")
    print("-" * 60)

    # Pre-cache text tokens (they don't change between images)
    trt_engine.cache_text_inputs(class_list)

    # Warmup
    if image_files:
        print("Warmup forward pass...")
        warmup_img = Image.open(image_files[0]).convert("RGB")
        proc.process_image(warmup_img, class_list)
        if torch.cuda.is_available():
            torch.cuda.synchronize(device)
        timer.reset()
        if torch.cuda.is_available():
            torch.cuda.reset_peak_memory_stats(device)
        print("Warmup done.\n")

    total_objects = 0
    total_intra_nms_removed = 0
    total_nms_removed = 0
    class_counts = {c: 0 for c in class_list}
    n_images_processed = 0
    ann_builder = InferenceAnnotationBuilder(
        tool="sam3-trt-infer", source="trt-pcs",
    )

    shard_saver: Optional[ShardSaver] = None
    if save_for_labeler:
        labeler_dir = labeler_output_dir or output_dir
        os.makedirs(labeler_dir, exist_ok=True)
        shard_saver = ShardSaver(
            labeler_dir,
            num_workers=saver_workers,
            fsync=not saver_no_fsync,
            tool_name="sam3-trt-infer",
            source="trt-pcs",
        )
        print(
            f"[ShardSaver] Writing labeler-format pairs to "
            f"{os.path.abspath(labeler_dir)}/pairs/ "
            f"(workers={saver_workers}, fsync={not saver_no_fsync})."
        )

    prefetcher = PrefetchLoader(image_files, decode_bgr=save_vis)

    # Kick off prefetch for the first image
    prefetcher.kick(0)

    for idx, image_path in enumerate(tqdm(image_files, desc="Processing images")):
        try:
            # Get the prefetched raw bytes + dimensions (+ optional BGR for vis)
            timer.start("image_load")
            prefetched = prefetcher.get()
            timer.stop("image_load")

            # Immediately kick off prefetch of the NEXT image
            # (overlaps with GPU processing below)
            prefetcher.kick(idx + 1)

            if prefetched is not None:
                raw_bytes, (img_h, img_w), bgr_vis = prefetched
                result = proc.process_image_bytes(raw_bytes, (img_h, img_w), class_list)
            else:
                pil_image = Image.open(image_path).convert("RGB")
                img_h, img_w = pil_image.height, pil_image.width
                bgr_vis = None
                result = proc.process_image(pil_image, class_list)

            if intra_nms_threshold > 0:
                timer.start("intra_class_nms")
                result = filter_intra_class_nms(result, iou_threshold=intra_nms_threshold)
                timer.stop("intra_class_nms")

            if nms_iou_threshold > 0:
                timer.start("nms_filter")
                result = filter_cross_class_nms(
                    result, image_hw=(img_h, img_w),
                    iou_threshold=nms_iou_threshold,
                    proximity_threshold=nms_proximity_threshold,
                    min_mask_size_ratio=nms_min_mask_ratio,
                    image_name=os.path.basename(image_path), debug=debug_nms,
                )
                timer.stop("nms_filter")

            # When --save_for_labeler is on, perform the (single)
            # GPU->CPU mask transfer here so visualization can re-use the
            # buffers and the per-image RLE+JSON cost moves to the
            # background ShardSaver pool.
            cpu_payload: Optional[Dict[str, Dict[str, Any]]] = None
            if shard_saver is not None:
                timer.start("shard_save_main")
                cpu_payload = _extract_per_np_cpu_payload(result)
                shard_saver.submit(
                    image_path=image_path,
                    width=img_w,
                    height=img_h,
                    per_np_data=cpu_payload,
                    queried_classes=class_list,
                    mode="pcs",
                )
                timer.stop("shard_save_main")

            if save_vis:
                timer.start("visualization")
                vis_input = bgr_vis if bgr_vis is not None else pil_image
                out_path = os.path.join(output_dir, f"vis_{os.path.basename(image_path)}")
                save_visualization(
                    vis_input, result, class_list, out_path,
                    cpu_payload=cpu_payload,
                )
                timer.stop("visualization")

            total_objects += result["total_objects"]
            total_intra_nms_removed += result.get("intra_nms_removed", 0)
            total_nms_removed += result.get("nms_removed", 0)
            for cn, data in result["results_by_class"].items():
                class_counts[cn] = class_counts.get(cn, 0) + data["count"]

            # Emit one labeler-format images[] row per (image, queried NP),
            # including NPs with zero detections (negative pair). Bbox-only,
            # so segmentation is omitted; bboxes are normalized inside
            # ann_builder.add_pair().
            for cn in class_list:
                boxes_pix, scores = _collect_boxes_scores(result, cn)
                ann_builder.add_pair(
                    file_name=image_path,
                    width=img_w,
                    height=img_h,
                    noun_phrase=cn,
                    boxes_xyxy_pix=boxes_pix,
                    scores=scores,
                )
            n_images_processed += 1
        except Exception as e:
            print(f"\nError processing {image_path}: {e}")
            import traceback
            traceback.print_exc()

    prefetcher.shutdown()

    # Drain the background shard saver (writes any pending labeler shards
    # before we report timing). This wait is reported separately and does
    # not contaminate the per-image trt_* / nms_* timers.
    if shard_saver is not None:
        timer.start("shard_save_drain")
        shard_saver.drain()
        timer.stop("shard_save_drain")

    # Summary
    n = n_images_processed
    print("\n" + "-" * 60)
    print(f"Images processed: {n}")
    print(f"Total objects: {total_objects}")
    if n > 0:
        print(f"Avg objects/image: {total_objects / n:.2f}")
        infer_time = (
            timer.times.get("trt_gpu_decode", 0)
            + timer.times.get("trt_preprocess", 0)
            + timer.times.get("trt_vision_encode", 0)
            + timer.times.get("trt_vision_repeat", 0)
            + timer.times.get("trt_detector_forward", 0)
        )
        if infer_time > 0:
            fps = n / infer_time
            latency = infer_time / n * 1000
            print(f"\nInference-only (excl. I/O & vis):")
            print(f"  FPS:     {fps:.2f}")
            print(f"  Latency: {latency:.1f} ms / image")
    if intra_nms_threshold > 0 and total_intra_nms_removed > 0:
        print(f"Intra-class NMS removed: {total_intra_nms_removed} same-class duplicates")
    if nms_iou_threshold > 0 and total_nms_removed > 0:
        print(f"Cross-class NMS removed: {total_nms_removed} overlapping detections")
    print("\nPer-class counts:")
    for cn, cnt in class_counts.items():
        print(f"  {cn}: {cnt}")

    json_path = os.path.join(output_dir, "predictions.json")
    ann_builder.dump(json_path)
    print(f"\nResults saved to: {json_path}")

    if shard_saver is not None:
        labeler_dir = labeler_output_dir or output_dir
        print(
            f"Labeler-format shards saved under "
            f"{os.path.abspath(labeler_dir)}/pairs/ ({shard_saver.stats()})."
        )

    if torch.cuda.is_available():
        log_cuda_memory(device, "CUDA memory after processing (peak since engine load)")

    print(timer.get_summary())


# ─────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description="SAM3 HF TensorRT Benchmark — multi-text-prompt latency test"
    )
    p.add_argument("--input_dir", type=str,
                    required=True)
    p.add_argument("--output_dir", type=str,
                    default="outputs")
    p.add_argument("--engine_dir", type=str, default="export_hf/trt_engines")
    p.add_argument("--model_name", type=str,
                    required=True,
                    help="HuggingFace model name or local path (for processor loading)")
    p.add_argument("--classes", type=str, nargs="+", default=None)
    p.add_argument("--outdoor", action="store_true")
    p.add_argument("--confidence_threshold", type=float, default=0.6)
    p.add_argument("--gpu", type=int, default=0)
    p.add_argument("--stride", type=int, default=1)
    p.add_argument("--offset", type=int, default=0)
    p.add_argument("--max_images", type=int, default=5000)
    p.add_argument("--intra_nms_threshold", type=float, default=0.5)
    p.add_argument("--nms_iou_threshold", type=float, default=0.8)
    p.add_argument("--nms_proximity_threshold", type=float, default=20.0)
    p.add_argument("--nms_min_mask_ratio", type=float, default=0.0001)
    p.add_argument("--debug_nms", action="store_true")
    p.add_argument("--no_vis", action="store_true")
    p.add_argument(
        "--save_for_labeler", action="store_true",
        help=(
            "Also write labeler-format per-pair shards "
            "(pairs/<stem>__<np>.json with RLE masks) to "
            "--labeler_output_dir, suitable for editing in the "
            "sam3.labeler.app browser tool. Runs in a background thread "
            "pool to avoid affecting inference latency."
        ),
    )
    p.add_argument(
        "--labeler_output_dir", type=str, default=None,
        help=(
            "Directory for labeler-format shards. Defaults to "
            "--output_dir when --save_for_labeler is on."
        ),
    )
    p.add_argument(
        "--saver_workers", type=int, default=2,
        help="Number of background ShardSaver worker threads.",
    )
    p.add_argument(
        "--saver_no_fsync", action="store_true", default=True,
        help=(
            "Skip os.fsync on every shard write (default: skipped). "
            "Trades crash-durability for ~1-3 ms per shard. The labeler "
            "writes always use fsync; only inference-time writes are "
            "affected."
        ),
    )
    p.add_argument(
        "--saver_fsync", dest="saver_no_fsync", action="store_false",
        help="Force fsync on every shard write (slower, safer).",
    )

    args = p.parse_args()
    
    if args.classes:
        class_list = args.classes
    elif args.outdoor:
        class_list = OUTDOOR_CLASSES
    else:
        class_list = INDOOR_CLASSES

    process_all_images(
        input_dir=args.input_dir,
        output_dir=args.output_dir,
        class_list=class_list,
        engine_dir=args.engine_dir,
        model_name=args.model_name,
        confidence_threshold=args.confidence_threshold,
        gpu_id=args.gpu,
        stride=args.stride,
        offset=args.offset,
        intra_nms_threshold=args.intra_nms_threshold,
        nms_iou_threshold=args.nms_iou_threshold,
        nms_proximity_threshold=args.nms_proximity_threshold,
        nms_min_mask_ratio=args.nms_min_mask_ratio,
        debug_nms=args.debug_nms,
        save_vis=not args.no_vis,
        max_images=args.max_images,
        save_for_labeler=args.save_for_labeler,
        labeler_output_dir=args.labeler_output_dir,
        saver_workers=args.saver_workers,
        saver_no_fsync=args.saver_no_fsync,
    )


if __name__ == "__main__":
    main()
