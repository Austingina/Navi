"""
SAM3 HuggingFace Inference Benchmark Script

Mirrors the structure of parallel_infer.py but uses the HuggingFace
transformers implementation of SAM3. Designed for side-by-side latency
comparison, especially for multi-text-prompt scenarios.

Key feature: vision_embeds caching — the vision backbone runs once per
image, and its output is reused across all text prompts.
"""

import os
import glob
import json
import argparse
import time
from tqdm import tqdm
from typing import List, Dict, Any, Tuple
from collections import defaultdict

import torch
import numpy as np
import cv2
from PIL import Image

from sam3.labeler.annotations import InferenceAnnotationBuilder


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
# Default classes (same as parallel_infer.py)
# ─────────────────────────────────────────────────────────────────────

FOREGROUND_CLASSES = [
    "elevator",
    "red fire extinguisher box",
    "red fire hydrant identification sign",
    "emergency exit sign",
]

NMS_EXCLUDE_PAIRS: set = {
    frozenset({"door", "door handle"}),
    frozenset({"door", "door knob"}),
}


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
            rss = _get_process_rss_mib()
            if rss >= 0:
                print(f"  [Process memory (this process)]")
                print(f"    RSS:                       {rss:10.1f} MiB")
        else:
            print(f"  [Driver-level (pynvml) — GPU {gpu_index}]")
            print(f"    total:                     {nvml_total:10.1f} MiB")
            print(f"    used:                      {nvml_used:10.1f} MiB")
            print(f"    free:                      {nvml_free:10.1f} MiB")
            outside_pt = nvml_used - pt_reserved
            print(f"    non-PyTorch (TRT, ctx...): {outside_pt:10.1f} MiB")
    else:
        print(f"  [pynvml not available — install nvidia-ml-py3 for full GPU memory stats]")


def setup_cuda(gpu_id: int = 0):
    torch.cuda.set_device(gpu_id)
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True
    torch.autocast("cuda", dtype=torch.bfloat16).__enter__()


# ─────────────────────────────────────────────────────────────────────
# Model loading
# ─────────────────────────────────────────────────────────────────────

def load_model(model_name: str, gpu_id: int = 0):
    from transformers import AutoModel, AutoProcessor, AutoTokenizer, AutoImageProcessor, Sam3Model

    device = torch.device(f"cuda:{gpu_id}")

    timer.start("model_loading")
    print(f"Loading HuggingFace SAM3 model: {model_name}")

    raw_model = AutoModel.from_pretrained(model_name, torch_dtype=torch.float32)
    raw_model = raw_model.to(device).eval()

    # The local checkpoint may be Sam3VideoModel (video wrapper) instead of
    # Sam3Model (image detector).  Extract the inner detector if needed.
    if isinstance(raw_model, Sam3Model):
        model = raw_model
    elif hasattr(raw_model, "detector_model"):
        print(f"  Loaded {type(raw_model).__name__}, extracting inner detector_model (Sam3Model)")
        model = raw_model.detector_model
    else:
        raise RuntimeError(
            f"Loaded model is {type(raw_model).__name__} and has no detector_model attribute. "
            f"Please provide a Sam3Model checkpoint or a Sam3VideoModel checkpoint."
        )

    # Build the image-model processor (Sam3Processor).
    # AutoProcessor may return Sam3VideoProcessor when the checkpoint is a
    # video model, which lacks text handling.  Fall back to building
    # Sam3Processor manually from image_processor + tokenizer.
    try:
        from transformers import Sam3Processor as _Sam3Proc
        processor = AutoProcessor.from_pretrained(model_name)
        if not isinstance(processor, _Sam3Proc):
            raise TypeError("need Sam3Processor")
    except (TypeError, Exception):
        from transformers import Sam3Processor as _Sam3Proc
        image_processor = AutoImageProcessor.from_pretrained(model_name)
        tokenizer = AutoTokenizer.from_pretrained(model_name)
        processor = _Sam3Proc(image_processor=image_processor, tokenizer=tokenizer, target_size=1008)
        print("  Built Sam3Processor manually from image_processor + tokenizer")

    timer.stop("model_loading")
    print(f"Model loaded on GPU {gpu_id} ({type(raw_model).__name__} → {type(model).__name__})")
    return model, processor, device


# ─────────────────────────────────────────────────────────────────────
# HuggingFace batch processor
# ─────────────────────────────────────────────────────────────────────

class HFBatchProcessor:
    """
    Processes one image with N text prompts using the HuggingFace SAM3
    implementation.  Caches vision_embeds so the vision backbone runs
    only once per image.

    Two modes are supported (controlled by --strategy):
      - "batch":    repeat vision_embeds N times, send all N texts in
                    a single forward pass  (batch=N)
      - "loop":     loop over N texts, each with batch=1 and cached
                    vision_embeds  (baseline comparison)
    """

    def __init__(
        self,
        model,
        processor,
        device: torch.device,
        confidence_threshold: float = 0.5,
        strategy: str = "batch",
    ):
        self.model = model
        self.processor = processor
        self.device = device
        self.confidence_threshold = confidence_threshold
        self.strategy = strategy

    # ── vision_embeds repeat ──────────────────────────────────────

    @staticmethod
    def _repeat_vision_embeds(embeds, n: int):
        """Repeat every tensor inside a vision-encoder output along the batch
        dimension to turn batch=1 into batch=n.  Handles plain tensors,
        tuples-of-tensors, and None values."""
        def _rep(v):
            if v is None:
                return None
            if isinstance(v, torch.Tensor):
                return v.repeat(n, *([1] * (v.ndim - 1)))
            if isinstance(v, tuple):
                return tuple(_rep(t) for t in v)
            if isinstance(v, list):
                return [_rep(t) for t in v]
            return v

        from dataclasses import fields as dc_fields
        kwargs = {}
        for f in dc_fields(embeds):
            kwargs[f.name] = _rep(getattr(embeds, f.name))
        return type(embeds)(**kwargs)

    # ── post-processing ─────────────────────────────────────────────

    @staticmethod
    def _postprocess_single(
        pred_masks: torch.Tensor,
        pred_boxes: torch.Tensor,
        pred_logits: torch.Tensor,
        presence_logits: torch.Tensor,
        original_hw: Tuple[int, int],
        confidence_threshold: float,
    ) -> Dict[str, Any]:
        """Filter predictions for a single (image, text) pair and
        rescale masks / boxes to the original image size."""
        h, w = original_hw

        # pred_logits: (num_queries,) or (num_queries, num_text_tokens)
        # presence_logits: (1,) — single presence score per (image, text) pair
        probs = pred_logits.sigmoid()
        if presence_logits is not None:
            presence = presence_logits.sigmoid().view(1)  # scalar
            probs = probs * presence
        # Collapse to (num_queries,) — max-pool over text tokens if multi-token
        while probs.ndim > 1:
            probs = probs.max(dim=-1).values

        keep = probs > confidence_threshold
        if keep.sum() == 0:
            return {"masks": [], "boxes": [], "scores": []}

        scores = probs[keep]
        boxes = pred_boxes[keep]  # already xyxy normalised [0,1] (converted inside model forward)
        masks = pred_masks[keep]  # (K, mask_h, mask_w)

        # boxes: xyxy normalised → xyxy in pixel coords
        scale = torch.tensor([w, h, w, h], device=boxes.device, dtype=boxes.dtype)
        boxes_xyxy = boxes * scale

        # masks → original resolution
        masks = torch.nn.functional.interpolate(
            masks.unsqueeze(1).float(), size=(h, w), mode="bilinear", align_corners=False
        ).sigmoid().squeeze(1)
        masks = masks > 0.5

        return {
            "masks": masks,
            "boxes": boxes_xyxy,
            "scores": scores,
        }

    # ── core forward ────────────────────────────────────────────────

    @torch.inference_mode()
    def process_image(
        self,
        pil_image: Image.Image,
        class_list: List[str],
    ) -> Dict[str, Any]:
        h, w = pil_image.height, pil_image.width

        # 1. Encode the image ONCE
        timer.start("hf_vision_encode")
        img_inputs = self.processor(images=pil_image, return_tensors="pt").to(self.device)
        vision_embeds = self.model.get_vision_features(pixel_values=img_inputs.pixel_values)
        timer.stop("hf_vision_encode")

        results_by_class: Dict[str, Dict] = {}
        total_objects = 0

        if self.strategy == "batch":
            results_by_class, total_objects = self._forward_batch(
                vision_embeds, pil_image, class_list, (h, w)
            )
        else:
            results_by_class, total_objects = self._forward_loop(
                vision_embeds, pil_image, class_list, (h, w)
            )

        return {
            "results_by_class": results_by_class,
            "total_objects": total_objects,
            "classes_detected": list(results_by_class.keys()),
        }

    # ── strategy: batch ─────────────────────────────────────────────

    def _forward_batch(
        self,
        vision_embeds,
        pil_image: Image.Image,
        class_list: List[str],
        original_hw: Tuple[int, int],
    ) -> Tuple[Dict[str, Dict], int]:
        """All N texts in a single forward pass (batch=N)."""
        N = len(class_list)

        timer.start("hf_text_encode")
        text_inputs = self.processor(text=class_list, return_tensors="pt", padding=True).to(self.device)
        timer.stop("hf_text_encode")

        # Repeat vision_embeds to match batch=N
        timer.start("hf_vision_repeat")
        repeated_embeds = self._repeat_vision_embeds(vision_embeds, N)
        timer.stop("hf_vision_repeat")

        timer.start("hf_detr_forward")
        outputs = self.model(
            vision_embeds=repeated_embeds,
            input_ids=text_inputs.input_ids,
            attention_mask=text_inputs.attention_mask,
        )
        timer.stop("hf_detr_forward")

        timer.start("hf_postprocess")
        results_by_class = {}
        total_objects = 0
        for i, class_name in enumerate(class_list):
            res = self._postprocess_single(
                pred_masks=outputs.pred_masks[i],
                pred_boxes=outputs.pred_boxes[i],
                pred_logits=outputs.pred_logits[i],
                presence_logits=outputs.presence_logits[i] if outputs.presence_logits is not None else None,
                original_hw=original_hw,
                confidence_threshold=self.confidence_threshold,
            )
            n = len(res["scores"]) if isinstance(res["scores"], (list, torch.Tensor)) else 0
            if isinstance(res["scores"], torch.Tensor):
                n = res["scores"].numel()
            if n > 0:
                results_by_class[class_name] = {
                    "masks": res["masks"],
                    "boxes": res["boxes"],
                    "scores": res["scores"],
                    "count": n,
                }
                total_objects += n
        timer.stop("hf_postprocess")

        return results_by_class, total_objects

    # ── strategy: loop ──────────────────────────────────────────────

    def _forward_loop(
        self,
        vision_embeds,
        pil_image: Image.Image,
        class_list: List[str],
        original_hw: Tuple[int, int],
    ) -> Tuple[Dict[str, Dict], int]:
        """One forward pass per text prompt (batch=1), reusing
        vision_embeds."""
        results_by_class = {}
        total_objects = 0

        for class_name in class_list:
            timer.start("hf_text_encode")
            text_inputs = self.processor(text=class_name, return_tensors="pt").to(self.device)
            timer.stop("hf_text_encode")

            timer.start("hf_detr_forward")
            outputs = self.model(
                vision_embeds=vision_embeds,
                input_ids=text_inputs.input_ids,
                attention_mask=text_inputs.attention_mask,
            )
            timer.stop("hf_detr_forward")

            timer.start("hf_postprocess")
            res = self._postprocess_single(
                pred_masks=outputs.pred_masks[0],
                pred_boxes=outputs.pred_boxes[0],
                pred_logits=outputs.pred_logits[0],
                presence_logits=outputs.presence_logits[0] if outputs.presence_logits is not None else None,
                original_hw=original_hw,
                confidence_threshold=self.confidence_threshold,
            )
            n = len(res["scores"]) if isinstance(res["scores"], (list, torch.Tensor)) else 0
            if isinstance(res["scores"], torch.Tensor):
                n = res["scores"].numel()
            if n > 0:
                results_by_class[class_name] = {
                    "masks": res["masks"],
                    "boxes": res["boxes"],
                    "scores": res["scores"],
                    "count": n,
                }
                total_objects += n
            timer.stop("hf_postprocess")

        return results_by_class, total_objects


# ─────────────────────────────────────────────────────────────────────
# Visualization  (reused from parallel_infer.py)
# ─────────────────────────────────────────────────────────────────────

COLORS = [
    (0.122, 0.467, 0.706), (1.0, 0.498, 0.055), (0.173, 0.627, 0.173),
    (0.839, 0.153, 0.157), (0.580, 0.404, 0.741), (0.549, 0.337, 0.294),
    (0.890, 0.467, 0.761), (0.894, 0.102, 0.329), (0.737, 0.741, 0.133),
    (0.090, 0.745, 0.812),
]


def color_to_bgr(color) -> Tuple[int, int, int]:
    if isinstance(color, (tuple, list, np.ndarray)) and len(color) >= 3:
        r = int(color[0] * 255) if color[0] <= 1.0 else int(color[0])
        g = int(color[1] * 255) if color[1] <= 1.0 else int(color[1])
        b = int(color[2] * 255) if color[2] <= 1.0 else int(color[2])
        return (b, g, r)
    return (0, 0, 255)


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


def save_visualization(image: Image.Image, result: Dict, class_list: List[str], output_path: str):
    all_results = result["results_by_class"]
    total_objects = result["total_objects"]

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
            masks = data["masks"]
            boxes = data["boxes"]
            scores = data["scores"]
            count = data["count"]
            for i in range(count):
                c_bgr = color_to_bgr(COLORS[obj_idx % len(COLORS)])
                obj_idx += 1

                mask = masks[i] if count > 1 else masks
                box = boxes[i] if count > 1 else boxes
                score = scores[i] if count > 1 else scores

                if isinstance(mask, torch.Tensor):
                    mask_2d = mask.squeeze().cpu().numpy()
                else:
                    mask_2d = np.array(mask).squeeze()
                while mask_2d.ndim > 2:
                    mask_2d = mask_2d[0]
                draw_mask_cv2(vis_img, mask_2d, c_bgr)

                prob = score.item() if hasattr(score, "item") else float(score)
                if isinstance(box, torch.Tensor):
                    box_flat = box.cpu().numpy().squeeze().flatten()
                else:
                    box_flat = np.array(box).squeeze().flatten()
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
# NMS utilities  (same logic as parallel_infer.py)
# ─────────────────────────────────────────────────────────────────────

def _box_iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def _mask_iou_np(mask_a: np.ndarray, mask_b: np.ndarray) -> float:
    intersection = np.count_nonzero(mask_a & mask_b)
    union = np.count_nonzero(mask_a | mask_b)
    return intersection / union if union > 0 else 0.0


def _to_np_box(box):
    if isinstance(box, torch.Tensor):
        return box.cpu().numpy().squeeze().flatten()
    return np.asarray(box).squeeze().flatten()


def _to_float_score(s):
    return s.item() if hasattr(s, "item") else float(s)


def filter_intra_class_nms(
    result: Dict[str, Any],
    iou_threshold: float = 0.5,
) -> Dict[str, Any]:
    if result["total_objects"] == 0:
        return result

    new_by_class: Dict[str, Dict] = {}
    total_kept = 0

    for class_name, data in result["results_by_class"].items():
        count = data["count"]
        dets = []
        for i in range(count):
            mask = data["masks"][i] if count > 1 else data["masks"]
            box = data["boxes"][i] if count > 1 else data["boxes"]
            score = data["scores"][i] if count > 1 else data["scores"]
            b_np = _to_np_box(box)
            s_f = _to_float_score(score)
            dets.append((s_f, b_np, mask, box, score))

        dets.sort(key=lambda d: d[0], reverse=True)

        kept: list = []
        for det in dets:
            s_f, b_np, orig_mask, orig_box, orig_score = det
            suppress = False
            for _, k_b, _, _, _ in kept:
                if _box_iou(b_np, k_b) > iou_threshold:
                    suppress = True
                    break
            if not suppress:
                kept.append(det)

        if kept:
            masks_out = [k[2] for k in kept]
            boxes_out = [k[3] for k in kept]
            scores_out = [k[4] for k in kept]
            c = len(kept)
            if c == 1:
                masks_out = masks_out[0]
                boxes_out = boxes_out[0]
                scores_out = scores_out[0]
            new_by_class[class_name] = {
                "masks": masks_out,
                "boxes": boxes_out,
                "scores": scores_out,
                "count": c,
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
    if result["total_objects"] == 0:
        return result

    tag = f"[cross-NMS {image_name}]" if image_name else "[cross-NMS]"

    all_dets = []
    for class_name, data in result["results_by_class"].items():
        count = data["count"]
        for i in range(count):
            mask = data["masks"][i] if count > 1 else data["masks"]
            box = data["boxes"][i] if count > 1 else data["boxes"]
            score = data["scores"][i] if count > 1 else data["scores"]

            if isinstance(mask, torch.Tensor):
                m2d = mask.squeeze().cpu().numpy().astype(bool)
            else:
                m2d = np.asarray(mask).squeeze().astype(bool)
            while m2d.ndim > 2:
                m2d = m2d[0]

            b_np = _to_np_box(box)
            s_f = _to_float_score(score)
            all_dets.append((s_f, class_name, m2d, b_np, mask, box, score))

    if not all_dets:
        return result

    all_dets.sort(key=lambda d: d[0], reverse=True)

    total_pixels = image_hw[0] * image_hw[1]
    min_mask_px = total_pixels * min_mask_size_ratio

    kept: list = []
    for det in all_dets:
        s_f, cn, m2d, b_np, orig_mask, orig_box, orig_score = det
        mask_px = np.count_nonzero(m2d)

        if mask_px < min_mask_px:
            continue

        cx, cy = (b_np[0] + b_np[2]) / 2, (b_np[1] + b_np[3]) / 2
        area = (b_np[2] - b_np[0]) * (b_np[3] - b_np[1])
        keep = True
        to_remove: list = []

        for k_idx, k in enumerate(kept):
            k_s, k_cn, k_m2d, k_b, _, _, _ = k

            if exclude_pairs and frozenset({cn, k_cn}) in exclude_pairs:
                continue

            miou = _mask_iou_np(m2d, k_m2d)
            if miou > iou_threshold:
                keep = False
                break

            k_cx = (k_b[0] + k_b[2]) / 2
            k_cy = (k_b[1] + k_b[3]) / 2
            dist = np.sqrt((cx - k_cx) ** 2 + (cy - k_cy) ** 2)
            if dist < proximity_threshold:
                k_area = (k_b[2] - k_b[0]) * (k_b[3] - k_b[1])
                if (keep_larger and area > k_area) or (not keep_larger and area < k_area):
                    to_remove.append(k_idx)
                else:
                    keep = False
                    break

        if keep:
            for idx in sorted(to_remove, reverse=True):
                kept.pop(idx)
            kept.append(det)

    new_by_class: Dict[str, Dict] = {}
    for det in kept:
        _, cn, _, _, orig_mask, orig_box, orig_score = det
        if cn not in new_by_class:
            new_by_class[cn] = {"masks": [], "boxes": [], "scores": []}
        new_by_class[cn]["masks"].append(orig_mask)
        new_by_class[cn]["boxes"].append(orig_box)
        new_by_class[cn]["scores"].append(orig_score)

    for data in new_by_class.values():
        count = len(data["masks"])
        data["count"] = count
        if count == 1:
            data["masks"] = data["masks"][0]
            data["boxes"] = data["boxes"][0]
            data["scores"] = data["scores"][0]

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


# ─────────────────────────────────────────────────────────────────────
# Main processing loop
# ─────────────────────────────────────────────────────────────────────

def process_all_images(
    input_dir: str,
    output_dir: str,
    class_list: List[str],
    model_name: str,
    confidence_threshold: float = 0.5,
    gpu_id: int = 0,
    stride: int = 1,
    offset: int = 0,
    strategy: str = "batch",
    intra_nms_threshold: float = 0.0,
    nms_iou_threshold: float = 0.0,
    nms_proximity_threshold: float = 20.0,
    nms_min_mask_ratio: float = 0.0001,
    debug_nms: bool = False,
    save_vis: bool = True,
):
    setup_cuda(gpu_id)
    os.makedirs(output_dir, exist_ok=True)

    model, processor, device = load_model(model_name, gpu_id)
    hf_proc = HFBatchProcessor(model, processor, device, confidence_threshold, strategy)

    if torch.cuda.is_available():
        log_cuda_memory(device, "CUDA memory after model load")
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
    image_files = image_files[:500]

    print(f"\nFound {total_images} images, processing {len(image_files)} (offset={offset}, stride={stride})")
    print(f"Classes: {class_list}")
    print(f"Strategy: {strategy}")
    print(f"Confidence: {confidence_threshold}")
    print(f"GPU: {gpu_id}")
    if intra_nms_threshold > 0:
        print(f"Intra-class NMS: box IoU > {intra_nms_threshold}")
    else:
        print("Intra-class NMS: disabled")
    if nms_iou_threshold > 0:
        print(f"Cross-class NMS: IoU > {nms_iou_threshold}, proximity < {nms_proximity_threshold}px, "
              f"min_mask_ratio = {nms_min_mask_ratio}")
    else:
        print("Cross-class NMS: disabled")
    print("-" * 60)

    total_objects = 0
    total_intra_nms_removed = 0
    total_nms_removed = 0
    class_counts = {c: 0 for c in class_list}
    n_images_processed = 0
    ann_builder = InferenceAnnotationBuilder(
        tool="sam3-hf-infer", source="hf-pcs",
    )

    # Warmup: run one image to trigger JIT / CUDA kernel caching
    if image_files:
        print("Warmup forward pass...")
        warmup_img = Image.open(image_files[0]).convert("RGB")
        with torch.inference_mode():
            hf_proc.process_image(warmup_img, class_list)
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        timer.reset()
        if torch.cuda.is_available():
            torch.cuda.reset_peak_memory_stats(device)
        print("Warmup done.\n")

    for image_path in tqdm(image_files, desc="Processing images"):
        try:
            timer.start("image_load")
            pil_image = Image.open(image_path).convert("RGB")
            timer.stop("image_load")

            result = hf_proc.process_image(pil_image, class_list)

            if intra_nms_threshold > 0:
                timer.start("intra_class_nms")
                result = filter_intra_class_nms(result, iou_threshold=intra_nms_threshold)
                timer.stop("intra_class_nms")

            if nms_iou_threshold > 0:
                timer.start("nms_filter")
                result = filter_cross_class_nms(
                    result,
                    image_hw=(pil_image.height, pil_image.width),
                    iou_threshold=nms_iou_threshold,
                    proximity_threshold=nms_proximity_threshold,
                    min_mask_size_ratio=nms_min_mask_ratio,
                    image_name=os.path.basename(image_path),
                    debug=debug_nms,
                )
                timer.stop("nms_filter")

            if save_vis:
                timer.start("visualization")
                out_path = os.path.join(output_dir, f"vis_{os.path.basename(image_path)}")
                save_visualization(pil_image, result, class_list, out_path)
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
                    width=pil_image.width,
                    height=pil_image.height,
                    noun_phrase=cn,
                    boxes_xyxy_pix=boxes_pix,
                    scores=scores,
                )
            n_images_processed += 1
        except Exception as e:
            print(f"\nError processing {image_path}: {e}")
            import traceback
            traceback.print_exc()

    # Summary
    n = n_images_processed
    print("\n" + "-" * 60)
    print(f"Images processed: {n}")
    print(f"Total objects: {total_objects}")
    if n > 0:
        print(f"Avg objects/image: {total_objects / n:.2f}")
        total_inference_time = (
            timer.times.get("hf_vision_encode", 0)
            + timer.times.get("hf_text_encode", 0)
            + timer.times.get("hf_vision_repeat", 0)
            + timer.times.get("hf_detr_forward", 0)
        )
        if total_inference_time > 0:
            fps = n / total_inference_time
            latency = total_inference_time / n * 1000
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

    if torch.cuda.is_available():
        log_cuda_memory(device, "CUDA memory after processing (peak since model load)")

    print(timer.get_summary())


# ─────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="SAM3 HuggingFace Benchmark — multi-text-prompt latency test"
    )
    parser.add_argument("--input_dir", type=str,
                        required=True)
    parser.add_argument("--output_dir", type=str,
                        default="outputs")
    parser.add_argument("--classes", type=str, nargs="+", default=None,
                        help="Class names to detect. Defaults to FOREGROUND_CLASSES.")
    parser.add_argument("--model_name", type=str, required=True,
                        help="HuggingFace model name or local path")
    parser.add_argument("--confidence_threshold", type=float, default=0.6)
    parser.add_argument("--gpu", type=int, default=0)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--offset", type=int, default=0)
    parser.add_argument("--strategy", type=str, default="batch", choices=["batch", "loop"],
                        help="'batch' = all texts in one forward (batch=N, cached vision), "
                             "'loop' = one forward per text (batch=1, cached vision)")
    parser.add_argument("--intra_nms_threshold", type=float, default=0.5,
                        help="Intra-class box NMS IoU threshold (0 to disable)")
    parser.add_argument("--nms_iou_threshold", type=float, default=0.8,
                        help="Cross-class mask NMS IoU threshold (0 to disable)")
    parser.add_argument("--nms_proximity_threshold", type=float, default=20.0,
                        help="Suppress detections whose box centres are within this many pixels")
    parser.add_argument("--nms_min_mask_ratio", type=float, default=0.0001,
                        help="Discard masks smaller than this fraction of total pixels")
    parser.add_argument("--debug_nms", action="store_true", default=False,
                        help="Print detailed cross-class NMS decisions")
    parser.add_argument("--no_vis", action="store_true", help="Skip saving visualizations")

    args = parser.parse_args()
    class_list = args.classes if args.classes else FOREGROUND_CLASSES

    process_all_images(
        input_dir=args.input_dir,
        output_dir=args.output_dir,
        class_list=class_list,
        model_name=args.model_name,
        confidence_threshold=args.confidence_threshold,
        gpu_id=args.gpu,
        stride=args.stride,
        offset=args.offset,
        strategy=args.strategy,
        intra_nms_threshold=args.intra_nms_threshold,
        nms_iou_threshold=args.nms_iou_threshold,
        nms_proximity_threshold=args.nms_proximity_threshold,
        nms_min_mask_ratio=args.nms_min_mask_ratio,
        debug_nms=args.debug_nms,
        save_vis=not args.no_vis,
    )


if __name__ == "__main__":
    main()
