"""
SAM3 Image Segmentation Test Script with Text Prompts

This script processes all images in the test_images directory using SAM3
with text prompts and saves the visualization results.

Supports true batch processing using SAM3's official batching API for maximum throughput.
Includes detailed timing instrumentation to identify performance bottlenecks.

Supports multi-GPU processing with the use of --gpu and --offset arguments.
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


class TimeTracker:
    """
    Tracks cumulative time spent in different stages of processing.
    Thread-safe accumulator for timing measurements.
    """
    
    def __init__(self):
        self.times = defaultdict(float)
        self.counts = defaultdict(int)
        self._start_times = {}
    
    def start(self, name: str):
        """Start timing a named stage."""
        if torch.cuda.is_available():
            torch.cuda.synchronize()  # Ensure GPU ops are complete
        self._start_times[name] = time.perf_counter()
    
    def stop(self, name: str):
        """Stop timing a named stage and accumulate the time."""
        if torch.cuda.is_available():
            torch.cuda.synchronize()  # Ensure GPU ops are complete
        if name in self._start_times:
            elapsed = time.perf_counter() - self._start_times[name]
            self.times[name] += elapsed
            self.counts[name] += 1
            del self._start_times[name]
            return elapsed
        return 0.0
    
    def reset(self):
        """Clear all accumulated timing data."""
        self.times.clear()
        self.counts.clear()
        self._start_times.clear()

    def get_summary(self) -> str:
        """Generate a formatted summary of all timing data."""
        if not self.times:
            return "No timing data collected."
        
        lines = ["\n" + "=" * 70, "TIMING SUMMARY", "=" * 70]
        
        total_time = sum(self.times.values())
        
        # Sort by total time descending
        sorted_times = sorted(self.times.items(), key=lambda x: -x[1])
        
        for name, total in sorted_times:
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


# Global time tracker instance
timer = TimeTracker()

from sam3.model_builder import build_sam3_image_model
from sam3.model.sam3_image_processor import Sam3Processor
# Fixed palette (no gray — blends with tarmac); same as hf_infer.py / trt_infer.py
COLORS = [
    (0.122, 0.467, 0.706), (1.0, 0.498, 0.055), (0.173, 0.627, 0.173),
    (0.839, 0.153, 0.157), (0.580, 0.404, 0.741), (0.549, 0.337, 0.294),
    (0.890, 0.467, 0.761), (0.894, 0.102, 0.329), (0.737, 0.741, 0.133),
    (0.090, 0.745, 0.812),
]

# Official batch processing imports
from sam3.train.data.sam3_image_dataset import InferenceMetadata, FindQueryLoaded, Image as SAMImage, Datapoint
from sam3.train.data.collator import collate_fn_api as collate
from sam3.train.transforms.basic_for_api import ComposeAPI, RandomResizeAPI, ToTensorAPI, NormalizeAPI
from sam3.model.utils.misc import copy_data_to_device
from sam3.eval.postprocessors import PostProcessImage
from sam3.labeler.annotations import InferenceAnnotationBuilder


# Define the object classes to detect
FOREGROUND_CLASSES = [
    "red fire hydrant identification sign",
    "red fire extinguisher box", 
    "emergency exit sign", 
    "elevator",
]

# Class pairs that should never suppress each other in cross-class NMS.
# Use this for part-whole relationships where both detections are valid
# (e.g. a door handle is *inside* a door, but they are distinct objects).
NMS_EXCLUDE_PAIRS: set = {
    frozenset({"door", "door handle"}),
    frozenset({"door", "door knob"}),
}

common_indoor_classes = [
    "desk", "chair", "laptop computer", "can", "plastic bag", "bottle", "computer keyboard",
    "desktop monitor", "mouse", "logo",
]

common_outdoor_classes = [
    "ground", "wall", "water", "grass"
]

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


def log_cuda_memory(device: torch.device, heading: str) -> None:
    """Print current and peak CUDA memory for `device` (syncs first)."""
    if not torch.cuda.is_available():
        return
    torch.cuda.synchronize(device)

    gpu_index = device.index if device.index is not None else 0
    to_mib = 1.0 / (1024**2)
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
    """Setup CUDA optimizations for Ampere GPUs.
    
    Args:
        gpu_id: GPU device ID to use. Default is 0.
    """
    torch.cuda.set_device(gpu_id)
    
    # Turn on tfloat32 for Ampere GPUs
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True
    # Use bfloat16 for inference
    torch.autocast("cuda", dtype=torch.bfloat16).__enter__()


def load_model(
    bpe_path: str,
    confidence_threshold: float = 0.5,
    checkpoint_path: str = None,
    gpu_id: int = 0,
):
    """Load SAM3 model and create processor.
    
    Args:
        bpe_path: Path to BPE vocabulary file
        confidence_threshold: Confidence threshold for detections
        checkpoint_path: Path to local model checkpoint (.pt file)
                        If None, downloads from HuggingFace (requires login)
        gpu_id: GPU device ID to use.
    
    Note:
        The checkpoint must be in the raw Meta format (sam3.pt), NOT the 
        HuggingFace Transformers format (model.safetensors).
        
        DataParallel is NOT supported because SAM3's architecture uses
        precomputed rotary position embeddings (RoPE) that expect fixed
        input shapes. For multi-GPU processing, run multiple script
        instances with different --gpu and --offset values.
    """
    device = torch.device(f"cuda:{gpu_id}")
    
    # Determine if we should load from HuggingFace or local checkpoint
    load_from_HF = checkpoint_path is None
    
    if load_from_HF:
        print("Downloading model from HuggingFace (requires login)...")
    else:
        print(f"Loading model from local checkpoint: {checkpoint_path}")
    
    timer.start("model_loading")
    model = build_sam3_image_model(
        bpe_path=bpe_path,
        load_from_HF=load_from_HF,
        checkpoint_path=checkpoint_path,
    )
    
    # Move model to specified device
    model = model.to(device)
    print(f"Using GPU: {gpu_id}")
    
    processor = Sam3Processor(model, confidence_threshold=confidence_threshold)
    timer.stop("model_loading")
    return model, processor, gpu_id


class BatchProcessor:
    """
    True batch processor using SAM3's official batching API.
    
    Processes ALL images with ALL their text queries in a SINGLE forward pass.
    This is the most efficient approach: N images × C classes → 1 forward pass.
    """
    
    # Global counter for unique query IDs
    _query_counter = 0
    
    def __init__(self, model, confidence_threshold: float = 0.5, gpu_id: int = 0):
        self.model = model
        self.device = torch.device(f"cuda:{gpu_id}")
        self.confidence_threshold = confidence_threshold
        
        # Official transform pipeline
        self.transform = ComposeAPI(
            transforms=[
                RandomResizeAPI(sizes=1008, max_size=1008, square=True, consistent_transform=False),
                ToTensorAPI(),
                NormalizeAPI(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5]),
            ]
        )
        
        # Official post-processor
        self.postprocessor = PostProcessImage(
            max_dets_per_img=-1,
            iou_type="segm",
            use_original_sizes_box=True,
            use_original_sizes_mask=True,
            convert_mask_to_rle=False,
            detection_threshold=confidence_threshold,
            to_cpu=False,
        )
    
    def _create_datapoint(self, pil_image: Image.Image, class_list: List[str]) -> Tuple[Datapoint, Dict[str, int]]:
        """
        Create a datapoint for one image with all class queries.
        
        Returns:
            Tuple of (datapoint, query_id_map) where query_id_map maps class_name -> query_id
        """
        w, h = pil_image.size
        
        # Build the list of queries
        find_queries = []
        query_id_map = {}
        
        for class_name in class_list:
            BatchProcessor._query_counter += 1
            query_id = BatchProcessor._query_counter
            
            find_queries.append(
                FindQueryLoaded(
                    query_text=class_name,
                    image_id=0,
                    object_ids_output=[],  # unused for inference
                    is_exhaustive=True,    # unused for inference
                    query_processing_order=0,
                    inference_metadata=InferenceMetadata(
                        coco_image_id=query_id,
                        original_image_id=query_id,
                        original_category_id=1,
                        original_size=[h, w],
                        object_id=0,
                        frame_index=0,
                    ),
                )
            )
            query_id_map[class_name] = query_id
        
        # Create datapoint with required arguments
        datapoint = Datapoint(
            find_queries=find_queries,
            images=[SAMImage(data=pil_image, objects=[], size=[h, w])],
        )
        
        return datapoint, query_id_map
    
    @torch.inference_mode()
    def process_batch(
        self,
        images: List[Image.Image],
        class_list: List[str],
    ) -> List[Dict[str, Any]]:
        """
        Process a batch of images with multiple class prompts.
        
        ALL images and ALL queries are processed in a SINGLE forward pass.
        
        Args:
            images: List of PIL images
            class_list: List of class names to detect
            
        Returns:
            List of result dicts, one per image
        """
        if len(images) == 0:
            return []
        
        # Create datapoints for all images, each with all class queries
        timer.start("meta_preprocess")
        datapoints = []
        query_maps = []  # Maps query_id -> (image_idx, class_name)
        
        for img_idx, pil_image in enumerate(images):
            datapoint, query_id_map = self._create_datapoint(pil_image, class_list)
            datapoint = self.transform(datapoint)
            datapoints.append(datapoint)
            
            for class_name, query_id in query_id_map.items():
                query_maps.append((query_id, img_idx, class_name))

        batch = collate(datapoints, dict_key="dummy")["dummy"]
        batch = copy_data_to_device(batch, self.device, non_blocking=True)
        timer.stop("meta_preprocess")
        
        # SINGLE forward pass for ALL images and ALL queries
        timer.start("meta_detr_forward")
        output = self.model(batch)
        timer.stop("meta_detr_forward")
        
        # Post-process results
        timer.start("meta_postprocess")
        processed_results = self.postprocessor.process_results(output, batch.find_metadatas)
        
        # Initialize results for each image
        results = [{
            "results_by_class": {},
            "total_objects": 0,
            "classes_detected": []
        } for _ in range(len(images))]
        
        # Distribute results back to images by query_id
        for query_id, img_idx, class_name in query_maps:
            if query_id in processed_results:
                query_result = processed_results[query_id]
                
                masks = query_result.get("masks", [])
                boxes = query_result.get("boxes", [])
                scores = query_result.get("scores", [])
                
                # Handle tensor types
                if isinstance(scores, torch.Tensor):
                    num_objects = scores.numel()
                elif hasattr(scores, '__len__'):
                    num_objects = len(scores)
                else:
                    num_objects = 1 if scores is not None else 0
                
                if num_objects > 0:
                    results[img_idx]["results_by_class"][class_name] = {
                        "masks": masks,
                        "boxes": boxes,
                        "scores": scores,
                        "count": num_objects
                    }
                    results[img_idx]["total_objects"] += num_objects
                    if class_name not in results[img_idx]["classes_detected"]:
                        results[img_idx]["classes_detected"].append(class_name)
        timer.stop("meta_postprocess")
        
        return results


def color_to_bgr(color) -> Tuple[int, int, int]:
    """Convert color (numpy array, tuple, list, or string) to BGR tuple for OpenCV."""
    # Handle numpy arrays (COLORS is numpy array with RGB floats 0-1)
    if isinstance(color, np.ndarray):
        r = int(color[0] * 255) if color[0] <= 1.0 else int(color[0])
        g = int(color[1] * 255) if color[1] <= 1.0 else int(color[1])
        b = int(color[2] * 255) if color[2] <= 1.0 else int(color[2])
        return (b, g, r)  # BGR for OpenCV
    
    # Handle tuples/lists with RGB values
    if isinstance(color, (tuple, list)) and len(color) >= 3:
        r = int(color[0] * 255) if color[0] <= 1.0 else int(color[0])
        g = int(color[1] * 255) if color[1] <= 1.0 else int(color[1])
        b = int(color[2] * 255) if color[2] <= 1.0 else int(color[2])
        return (b, g, r)  # BGR for OpenCV
    
    # Map common color names to BGR values
    color_map = {
        'r': (0, 0, 255), 'red': (0, 0, 255),
        'g': (0, 255, 0), 'green': (0, 255, 0),
        'b': (255, 0, 0), 'blue': (255, 0, 0),
        'y': (0, 255, 255), 'yellow': (0, 255, 255),
        'c': (255, 255, 0), 'cyan': (255, 255, 0),
        'm': (255, 0, 255), 'magenta': (255, 0, 255),
        'w': (255, 255, 255), 'white': (255, 255, 255),
        'k': (0, 0, 0), 'black': (0, 0, 0),
        'orange': (0, 165, 255),
        'purple': (128, 0, 128),
        'pink': (203, 192, 255),
        'lime': (0, 255, 0),
    }
    
    if isinstance(color, str):
        color_lower = color.lower()
        if color_lower in color_map:
            return color_map[color_lower]
        # Try to parse hex color
        if color.startswith('#'):
            hex_color = color.lstrip('#')
            if len(hex_color) == 6:
                r = int(hex_color[0:2], 16)
                g = int(hex_color[2:4], 16)
                b = int(hex_color[4:6], 16)
                return (b, g, r)  # BGR for OpenCV
    
    # Default to red
    return (0, 0, 255)


def draw_mask_cv2(img: np.ndarray, mask: np.ndarray, color_bgr: Tuple[int, int, int], alpha: float = 0.5):
    """Draw a semi-transparent mask overlay on the image using OpenCV."""
    # Ensure mask is 2D boolean/uint8
    if isinstance(mask, torch.Tensor):
        mask = mask.cpu().numpy()
    mask = mask.astype(bool)
    
    # Create colored overlay
    overlay = img.copy()
    overlay[mask] = color_bgr
    
    # Blend with original
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)


def draw_bbox_cv2(
    img: np.ndarray,
    box: np.ndarray,
    text: str,
    color_bgr: Tuple[int, int, int],
    thickness: int = 4,
):
    """Draw bounding box with label using OpenCV."""
    x1, y1, x2, y2 = map(int, box[:4])
    
    # Draw rectangle
    cv2.rectangle(img, (x1, y1), (x2, y2), color_bgr, thickness)
    
    # Draw text with background (large font for readability)
    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale = 0.5
    font_thickness = 1
    
    (text_w, text_h), baseline = cv2.getTextSize(text, font, font_scale, font_thickness)
    
    # Text background (solid color matching bbox)
    padding = 8
    cv2.rectangle(img, (x1, y1 - text_h - padding * 2), (x1 + text_w + padding * 2, y1), color_bgr, -1)
    
    # Text (white for contrast, with black outline for extra readability)
    text_x = x1 + padding
    text_y = y1 - padding
    # Black outline
    cv2.putText(img, text, (text_x, text_y), font, font_scale, (0, 0, 0), font_thickness + 2, cv2.LINE_AA)
    # White fill
    cv2.putText(img, text, (text_x, text_y), font, font_scale, (255, 255, 255), font_thickness, cv2.LINE_AA)


def save_visualization(
    image: Image.Image,
    result: Dict[str, Any],
    class_list: List[str],
    output_path: str,
):
    """Save visualization for a single image result using OpenCV (fast)."""
    all_results = result["results_by_class"]
    total_objects = result["total_objects"]
    
    # Convert PIL image to OpenCV format (BGR)
    img = np.array(image)
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    elif img.shape[2] == 4:
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    else:
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    
    # Make a copy to draw on
    vis_img = img.copy()
    
    if total_objects > 0:
        # Use a distinct color for each object instance
        object_idx = 0
        legend_entries = []
        
        for class_name, results in all_results.items():
            masks = results["masks"]
            boxes = results["boxes"]
            scores = results["scores"]
            count = results["count"]
            
            for i in range(count):
                # Each object gets a unique color
                color_bgr = color_to_bgr(COLORS[object_idx % len(COLORS)])
                object_idx += 1
                
                mask = masks[i] if count > 1 else masks
                box = boxes[i] if count > 1 else boxes
                score = scores[i] if count > 1 else scores
                
                # Process mask
                if isinstance(mask, torch.Tensor):
                    mask_2d = mask.squeeze().cpu().numpy()
                else:
                    mask_2d = np.array(mask).squeeze()
                while mask_2d.ndim > 2:
                    mask_2d = mask_2d[0]
                
                draw_mask_cv2(vis_img, mask_2d, color_bgr, alpha=0.5)
                
                # Process box
                prob = score.item() if hasattr(score, 'item') else float(score)
                if isinstance(box, torch.Tensor):
                    box_flat = box.cpu().numpy().squeeze().flatten()
                else:
                    box_flat = np.array(box).squeeze().flatten()
                
                draw_bbox_cv2(vis_img, box_flat, f"{class_name}: {prob:.2f}", color_bgr)
            
            legend_entries.append(f"{class_name}: {count}")
        
        # Add title at the top with solid background for readability
        detected_classes = ", ".join(legend_entries)
        title = f"Total: {total_objects} | Detected: {detected_classes}"
        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = 1.0
        font_thickness = 1
        
        (text_w, text_h), baseline = cv2.getTextSize(title, font, font_scale, font_thickness)
        
        # Solid black background rectangle
        padding = 15
        cv2.rectangle(vis_img, (0, 0), (text_w + padding * 2, text_h + padding * 2), (0, 0, 0), -1)
        
        # White text
        cv2.putText(vis_img, title, (padding, text_h + padding), font, font_scale, (255, 255, 255), font_thickness, cv2.LINE_AA)
    else:
        # Add "No objects detected" text with solid background
        title = "No objects detected"
        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = 1.0
        font_thickness = 1
        
        (text_w, text_h), baseline = cv2.getTextSize(title, font, font_scale, font_thickness)
        
        # Solid black background rectangle
        padding = 15
        cv2.rectangle(vis_img, (0, 0), (text_w + padding * 2, text_h + padding * 2), (0, 0, 0), -1)
        
        # White text
        cv2.putText(vis_img, title, (padding, text_h + padding), font, font_scale, (255, 255, 255), font_thickness, cv2.LINE_AA)
    
    # Save with OpenCV (much faster than matplotlib)
    cv2.imwrite(output_path, vis_img)


# ---------------------------------------------------------------------------
# Intra-class (same-class) box NMS
# ---------------------------------------------------------------------------

def _box_iou(box_a: np.ndarray, box_b: np.ndarray) -> float:
    """IoU between two [x1, y1, x2, y2] boxes."""
    x1 = max(box_a[0], box_b[0])
    y1 = max(box_a[1], box_b[1])
    x2 = min(box_a[2], box_b[2])
    y2 = min(box_a[3], box_b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
    area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def filter_intra_class_nms(
    result: Dict[str, Any],
    iou_threshold: float = 0.5,
) -> Dict[str, Any]:
    """Per-class greedy box NMS: within each class, suppress lower-scoring
    detections whose box IoU with a higher-scoring kept detection exceeds
    *iou_threshold*."""
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

            if isinstance(box, torch.Tensor):
                b_np = box.cpu().numpy().squeeze().flatten()
            else:
                b_np = np.asarray(box).squeeze().flatten()
            s_f = score.item() if hasattr(score, "item") else float(score)
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


# ---------------------------------------------------------------------------
# Cross-class mask NMS
# ---------------------------------------------------------------------------

def _mask_iou_np(mask_a: np.ndarray, mask_b: np.ndarray) -> float:
    """IoU between two 2-D boolean masks."""
    intersection = np.count_nonzero(mask_a & mask_b)
    union = np.count_nonzero(mask_a | mask_b)
    return intersection / union if union > 0 else 0.0


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
    """Cross-class greedy NMS: suppress overlapping or proximate detections
    regardless of class.  Sorted by confidence; lower-scoring duplicates are
    removed when mask IoU > *iou_threshold* or box centres are within
    *proximity_threshold* pixels.  Masks smaller than *min_mask_size_ratio*
    of total image area are dropped unconditionally.

    *exclude_pairs* is a set of frozensets; class pairs listed here will
    never suppress each other (useful for part-whole relationships like
    door / door handle)."""
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

            if isinstance(box, torch.Tensor):
                b_np = box.cpu().numpy().squeeze().flatten()
            else:
                b_np = np.asarray(box).squeeze().flatten()

            s_f = score.item() if hasattr(score, "item") else float(score)
            all_dets.append((s_f, class_name, m2d, b_np, mask, box, score))

    if not all_dets:
        return result

    all_dets.sort(key=lambda d: d[0], reverse=True)

    total_pixels = image_hw[0] * image_hw[1]
    min_mask_px = total_pixels * min_mask_size_ratio

    if debug:
        print(f"{tag} image_hw={image_hw}, total_pixels={total_pixels}, "
              f"min_mask_px={min_mask_px:.0f} ({min_mask_size_ratio*100:.4f}%)")
        print(f"{tag} {len(all_dets)} pre-NMS detections (sorted by confidence):")
        for idx, (s, cn, m, b, *_) in enumerate(all_dets):
            mask_px = np.count_nonzero(m)
            cx, cy = (b[0]+b[2])/2, (b[1]+b[3])/2
            box_area = (b[2]-b[0]) * (b[3]-b[1])
            print(f"{tag}   [{idx}] {cn:20s}  conf={s:.4f}  mask_px={mask_px:>8d} "
                  f"({mask_px/total_pixels*100:5.2f}%)  box=[{b[0]:.0f},{b[1]:.0f},"
                  f"{b[2]:.0f},{b[3]:.0f}]  center=({cx:.0f},{cy:.0f})  box_area={box_area:.0f}")

    kept: list = []
    for det in all_dets:
        s_f, cn, m2d, b_np, orig_mask, orig_box, orig_score = det
        mask_px = np.count_nonzero(m2d)

        if mask_px < min_mask_px:
            if debug:
                print(f"{tag}   DROP '{cn}' conf={s_f:.4f}: mask too small "
                      f"({mask_px} < {min_mask_px:.0f} px)")
            continue

        cx, cy = (b_np[0] + b_np[2]) / 2, (b_np[1] + b_np[3]) / 2
        area = (b_np[2] - b_np[0]) * (b_np[3] - b_np[1])
        keep = True
        to_remove: list = []
        suppress_reason = ""

        for k_idx, k in enumerate(kept):
            k_s, k_cn, k_m2d, k_b, _, _, _ = k

            if exclude_pairs and frozenset({cn, k_cn}) in exclude_pairs:
                if debug:
                    print(f"{tag}   SKIP pair ('{cn}', '{k_cn}'): excluded")
                continue

            miou = _mask_iou_np(m2d, k_m2d)
            if miou > iou_threshold:
                suppress_reason = (f"mask IoU={miou:.4f} > {iou_threshold} "
                                   f"with kept[{k_idx}] '{k_cn}' conf={k_s:.4f}")
                keep = False
                break

            k_cx = (k_b[0] + k_b[2]) / 2
            k_cy = (k_b[1] + k_b[3]) / 2
            dist = np.sqrt((cx - k_cx) ** 2 + (cy - k_cy) ** 2)
            if dist < proximity_threshold:
                k_area = (k_b[2] - k_b[0]) * (k_b[3] - k_b[1])
                if (keep_larger and area > k_area) or (not keep_larger and area < k_area):
                    to_remove.append(k_idx)
                    if debug:
                        print(f"{tag}   EVICT kept[{k_idx}] '{k_cn}' conf={k_s:.4f}: "
                              f"proximity={dist:.1f}px < {proximity_threshold}px, "
                              f"current box_area={area:.0f} > kept box_area={k_area:.0f}")
                else:
                    suppress_reason = (f"proximity={dist:.1f}px < {proximity_threshold}px "
                                       f"with kept[{k_idx}] '{k_cn}' conf={k_s:.4f}, "
                                       f"current box_area={area:.0f} <= kept box_area={k_area:.0f}")
                    keep = False
                    break

        if keep:
            for idx in sorted(to_remove, reverse=True):
                kept.pop(idx)
            kept.append(det)
            if debug:
                print(f"{tag}   KEEP '{cn}' conf={s_f:.4f}  (kept count: {len(kept)})")
        else:
            if debug:
                print(f"{tag}   DROP '{cn}' conf={s_f:.4f}: {suppress_reason}")

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
    if debug:
        kept_summary = ", ".join(f"{cn}: {d['count']}" for cn, d in new_by_class.items())
        print(f"{tag} RESULT: kept {total_kept}, removed {removed}  [{kept_summary}]")
    return {
        "results_by_class": new_by_class,
        "total_objects": total_kept,
        "classes_detected": list(new_by_class.keys()),
        "nms_removed": removed,
    }


# ---------------------------------------------------------------------------
# JSON-serializable detection extraction
# ---------------------------------------------------------------------------

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
        score_val = score.item() if hasattr(score, "item") else float(score)
        boxes_out.append([float(v) for v in box_list[:4]])
        scores_out.append(score_val)
    return boxes_out, scores_out


# ---------------------------------------------------------------------------
# Single-image sequential processing
# ---------------------------------------------------------------------------

def process_single_image_multi_class(
    image_path: str,
    processor: Sam3Processor,
    class_list: List[str],
    output_dir: str,
    save_vis: bool = True,
    intra_nms_threshold: float = 0.0,
    nms_iou_threshold: float = 0.0,
    nms_proximity_threshold: float = 20.0,
    nms_min_mask_ratio: float = 0.0001,
    debug_nms: bool = False,
) -> Dict[str, Any]:
    """
    Process a single image with SAM3 using multiple class prompts.
    
    For each class, we run the model separately and aggregate all results.
    Note: SAM3 requires sequential processing - it doesn't support multi-concept
    prompts with separators like Grounding DINO does.
    """
    # Load image
    timer.start("image_load")
    image = Image.open(image_path).convert("RGB")
    timer.stop("image_load")
    
    # Set image once (backbone features can be reused)
    timer.start("meta_vision_encode")
    inference_state = processor.set_image(image)
    timer.stop("meta_vision_encode")
    
    # Aggregate results for all classes
    all_results = {}
    total_objects = 0
    
    for class_name in class_list:
        # Reset prompts and apply new text prompt
        timer.start("meta_detr_forward")
        processor.reset_all_prompts(inference_state)
        output = processor.set_text_prompt(state=inference_state, prompt=class_name)
        timer.stop("meta_detr_forward")
        
        # Get results for this class
        masks = output.get("masks", [])
        boxes = output.get("boxes", [])
        scores = output.get("scores", [])
        
        num_objects = len(scores) if hasattr(scores, '__len__') else (1 if scores is not None else 0)
        
        if num_objects > 0:
            all_results[class_name] = {
                "masks": masks,
                "boxes": boxes,
                "scores": scores,
                "count": num_objects
            }
            total_objects += num_objects
    
    result = {
        "results_by_class": all_results,
        "total_objects": total_objects,
        "classes_detected": list(all_results.keys())
    }
    
    if intra_nms_threshold > 0:
        timer.start("intra_class_nms")
        result = filter_intra_class_nms(result, iou_threshold=intra_nms_threshold)
        timer.stop("intra_class_nms")

    if nms_iou_threshold > 0:
        timer.start("nms_filter")
        result = filter_cross_class_nms(
            result,
            image_hw=(image.height, image.width),
            iou_threshold=nms_iou_threshold,
            proximity_threshold=nms_proximity_threshold,
            min_mask_size_ratio=nms_min_mask_ratio,
            image_name=os.path.basename(image_path),
            debug=debug_nms,
        )
        timer.stop("nms_filter")
    
    # Save visualization
    if save_vis:
        timer.start("visualization")
        image_name = os.path.basename(image_path)
        output_path = os.path.join(output_dir, f"vis_{image_name}")
        save_visualization(image, result, class_list, output_path)
        timer.stop("visualization")
    
    return result


def process_image_batch(
    image_paths: List[str],
    batch_processor: BatchProcessor,
    class_list: List[str],
    output_dir: str,
    save_vis: bool = True,
    intra_nms_threshold: float = 0.0,
    nms_iou_threshold: float = 0.0,
    nms_proximity_threshold: float = 20.0,
    nms_min_mask_ratio: float = 0.0001,
    debug_nms: bool = False,
) -> List[Dict[str, Any]]:
    """
    Process a batch of images with SAM3 using batched backbone processing.
    """
    # Load all images in the batch
    timer.start("image_load")
    images = [Image.open(path).convert("RGB") for path in image_paths]
    timer.stop("image_load")
    
    # Process batch (backbone is batched, grounding is per-image)
    results = batch_processor.process_batch(images, class_list)
    
    if intra_nms_threshold > 0:
        timer.start("intra_class_nms")
        for idx, result in enumerate(results):
            results[idx] = filter_intra_class_nms(result, iou_threshold=intra_nms_threshold)
        timer.stop("intra_class_nms")

    if nms_iou_threshold > 0:
        timer.start("nms_filter")
        for idx, (image, image_path, result) in enumerate(zip(images, image_paths, results)):
            results[idx] = filter_cross_class_nms(
                result,
                image_hw=(image.height, image.width),
                iou_threshold=nms_iou_threshold,
                proximity_threshold=nms_proximity_threshold,
                min_mask_size_ratio=nms_min_mask_ratio,
                image_name=os.path.basename(image_path),
                debug=debug_nms,
            )
        timer.stop("nms_filter")
    
    # Save visualizations
    if save_vis:
        timer.start("visualization")
        for image, image_path, result in zip(images, image_paths, results):
            image_name = os.path.basename(image_path)
            output_path = os.path.join(output_dir, f"vis_{image_name}")
            save_visualization(image, result, class_list, output_path)
        timer.stop("visualization")

    # Carry image dimensions on the result dict so the caller can
    # write labeler-format annotations without re-reading each file.
    for image, result in zip(images, results):
        result["image_size"] = (image.width, image.height)

    return results


def process_all_images(
    input_dir: str,
    output_dir: str,
    class_list: List[str],
    bpe_path: str,
    confidence_threshold: float = 0.5,
    checkpoint_path: str = None,
    stride: int = 1,
    batch_size: int = 1,
    gpu_id: int = 0,
    offset: int = 0,
    intra_nms_threshold: float = 0.0,
    nms_iou_threshold: float = 0.0,
    nms_proximity_threshold: float = 20.0,
    nms_min_mask_ratio: float = 0.0001,
    debug_nms: bool = False,
):
    # Setup CUDA optimizations
    setup_cuda(gpu_id)
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Load model
    print("Loading SAM3 model...")
    model, processor, gpu_id = load_model(bpe_path, confidence_threshold, checkpoint_path, gpu_id)
    print("Model loaded successfully!")
    
    # Create batch processor if using batching
    batch_processor = BatchProcessor(model, confidence_threshold, gpu_id)

    device = torch.device(f"cuda:{gpu_id}")
    if torch.cuda.is_available():
        log_cuda_memory(device, "CUDA memory after model load (peak since first CUDA alloc in process)")
        torch.cuda.reset_peak_memory_stats(device)
    
    # Get all image files
    image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
    image_files = []
    for ext in image_extensions:
        image_files.extend(glob.glob(os.path.join(input_dir, ext)))
    image_files = sorted(image_files)
    
    if len(image_files) == 0:
        print(f"No images found in {input_dir}")
        return
    
    # Apply offset and stride to sample images (for multi-GPU parallelism)
    total_images = len(image_files)
    image_files = image_files[offset::stride]
    image_files = image_files[:500]
    
    print(f"Found {total_images} images, processing {len(image_files)} (offset={offset}, stride={stride})")
    print(f"Classes to detect: {class_list}")
    print(f"Confidence threshold: {confidence_threshold}")
    print(f"Batch size: {batch_size}")
    print(f"GPU: {gpu_id}")
    print(f"Saving visualizations to: {output_dir}")
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

    # Warmup pass
    print("Warmup forward pass...")
    warmup_img = image_files[0]
    process_image_batch(
        image_paths=[warmup_img],
        batch_processor=batch_processor,
        class_list=class_list,
        output_dir=output_dir,
        save_vis=False,
    )
    timer.reset()
    if torch.cuda.is_available():
        torch.cuda.reset_peak_memory_stats(device)
    print("Warmup done, timer reset.")
    print("-" * 60)

    # Process images
    total_objects = 0
    total_intra_nms_removed = 0
    total_nms_removed = 0
    class_counts = {cls: 0 for cls in class_list}
    n_images_processed = 0
    ann_builder = InferenceAnnotationBuilder(
        tool="sam3-parallel-infer", source="parallel-pcs",
    )

    num_batches = (len(image_files) + batch_size - 1) // batch_size

    for batch_idx in tqdm(range(num_batches), desc="Processing batches"):
        start_idx = batch_idx * batch_size
        end_idx = min(start_idx + batch_size, len(image_files))
        batch_paths = image_files[start_idx:end_idx]

        try:
            batch_results = process_image_batch(
                image_paths=batch_paths,
                batch_processor=batch_processor,
                class_list=class_list,
                output_dir=output_dir,
                save_vis=True,
                intra_nms_threshold=intra_nms_threshold,
                nms_iou_threshold=nms_iou_threshold,
                nms_proximity_threshold=nms_proximity_threshold,
                nms_min_mask_ratio=nms_min_mask_ratio,
                debug_nms=debug_nms,
            )

            for image_path, result in zip(batch_paths, batch_results):
                total_objects += result["total_objects"]
                total_intra_nms_removed += result.get("intra_nms_removed", 0)
                total_nms_removed += result.get("nms_removed", 0)
                for cls, data in result["results_by_class"].items():
                    class_counts[cls] += data["count"]

                # Emit one labeler-format images[] row per (image, queried NP),
                # including NPs with zero detections (negative pair). Bbox-only,
                # so segmentation is omitted; bboxes are normalized inside
                # ann_builder.add_pair(). Image dims were attached to the
                # result dict inside process_image_batch.
                img_w, img_h = result.get("image_size", (0, 0))
                for cn in class_list:
                    boxes_pix, scores = _collect_boxes_scores(result, cn)
                    ann_builder.add_pair(
                        file_name=image_path,
                        width=int(img_w),
                        height=int(img_h),
                        noun_phrase=cn,
                        boxes_xyxy_pix=boxes_pix,
                        scores=scores,
                    )
                n_images_processed += 1
        except Exception as e:
            print(f"\nError processing batch {batch_idx}: {e}")
            import traceback
            traceback.print_exc()
            continue
    
    # Print summary
    n = n_images_processed
    print("\n" + "-" * 60)
    print(f"Images processed: {n}")
    print(f"Total objects: {total_objects}")
    if n > 0:
        print(f"Avg objects/image: {total_objects / n:.2f}")
        total_inference_time = (
            timer.times.get("meta_preprocess", 0)
            + timer.times.get("meta_detr_forward", 0)
            + timer.times.get("meta_postprocess", 0)
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
    for cls, count in class_counts.items():
        print(f"  {cls}: {count}")
    print(f"\nVisualizations saved to: {output_dir}")

    # Write labeler-format predictions.json
    json_path = os.path.join(output_dir, "predictions.json")
    ann_builder.dump(json_path)
    print(f"Detection results saved to: {json_path}")

    if torch.cuda.is_available():
        log_cuda_memory(
            device,
            "CUDA memory after image loop (peak stats = since reset after model load)",
        )

    # Print timing summary
    print(timer.get_summary())


def main():
    parser = argparse.ArgumentParser(
        description="SAM3 Image Segmentation with Multiple Class Text Prompts"
    )
    parser.add_argument(
        "--input_dir",
        type=str,
        required=True,
        help="Directory containing input images"
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default="outputs",
        help="Directory to save visualizations"
    )
    parser.add_argument(
        "--classes",
        type=str,
        nargs='+',
        default=None,
        help="List of class names to detect (space-separated). If not provided, uses default FOREGROUND_CLASSES."
    )
    parser.add_argument(
        "--confidence_threshold",
        type=float,
        default=0.6,
        help="Confidence threshold for detections (default: 0.5)"
    )
    parser.add_argument(
        "--bpe_path",
        type=str,
        default=os.path.join(os.path.dirname(__file__), "..", "sam3", "assets", "bpe_simple_vocab_16e6.txt.gz"),
        help="Path to BPE vocabulary file"
    )
    parser.add_argument(
        "--checkpoint_path",
        type=str,
        required=True,
        help="Path to model checkpoint (.pt file)."
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=1,
        help="Sample every Nth image from the dataset (default: 1, process all images)"
    )
    parser.add_argument(
        "--batch_size",
        type=int,
        default=1,
        help="Number of images to process in each batch (default: 4). "
             "Larger batches improve throughput by batching the backbone forward pass. "
             "Adjust based on GPU memory."
    )
    parser.add_argument(
        "--gpu",
        type=int,
        default=0,
        help="GPU device ID to use (default: 0). "
             "For multi-GPU parallelism, run multiple instances with different "
             "--gpu and --offset values."
    )
    parser.add_argument(
        "--offset",
        type=int,
        default=0,
        help="Starting offset for image selection (default: 0). "
             "Use with --stride for multi-GPU parallelism. "
             "Example: 4 GPUs with stride=4, use offset 0,1,2,3 on each GPU."
    )
    
    # --- NMS ---
    parser.add_argument(
        "--intra_nms_threshold",
        type=float,
        default=0.5,
        help="Intra-class (same-class) box NMS IoU threshold (default: 0.5). "
             "Suppresses duplicate detections of the same class. Set to 0 to disable.",
    )
    parser.add_argument(
        "--nms_iou_threshold",
        type=float,
        default=0.8,
        help="Cross-class mask NMS IoU threshold (default: 0.8). Set to 0 to disable.",
    )
    parser.add_argument(
        "--nms_proximity_threshold",
        type=float,
        default=20.0,
        help="Suppress detections whose box centres are within this many pixels (default: 20).",
    )
    parser.add_argument(
        "--nms_min_mask_ratio",
        type=float,
        default=0.0001,
        help="Discard masks smaller than this fraction of total pixels (default: 0.0001).",
    )
    parser.add_argument(
        "--debug_nms",
        action="store_true",
        default=False,
        help="Print detailed cross-class NMS decisions for every image.",
    )
    
    args = parser.parse_args()
    
    # Use provided classes or default to FOREGROUND_CLASSES
    class_list = args.classes if args.classes else FOREGROUND_CLASSES
    
    process_all_images(
        input_dir=args.input_dir,
        output_dir=args.output_dir,
        class_list=class_list,
        bpe_path=args.bpe_path,
        confidence_threshold=args.confidence_threshold,
        checkpoint_path=args.checkpoint_path,
        stride=args.stride,
        batch_size=args.batch_size,
        gpu_id=args.gpu,
        offset=args.offset,
        intra_nms_threshold=args.intra_nms_threshold,
        nms_iou_threshold=args.nms_iou_threshold,
        nms_proximity_threshold=args.nms_proximity_threshold,
        nms_min_mask_ratio=args.nms_min_mask_ratio,
        debug_nms=args.debug_nms,
    )


if __name__ == "__main__":
    main()
