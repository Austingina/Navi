#!/usr/bin/env python3
"""
TensorRT inference pipeline for HuggingFace SAM3.

Orchestrates 2 TRT engines (VisionEncoder, DetectorBody) with the
HF Batch strategy: vision features are computed once per image, repeated
to batch=N for N text prompts, then processed in a single DetectorBody pass.

Usage:
    from export_hf.trt_infer import HFSam3TRTInference
    engine = HFSam3TRTInference("export_hf/trt_engines")
    results = engine.predict(pil_image, ["car", "person", "tree"])
"""

import os
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image

try:
    import tensorrt as trt
except ImportError:
    trt = None

try:
    import torchvision.io
    _HAS_TORCHVISION_IO = True
except ImportError:
    _HAS_TORCHVISION_IO = False


# ─────────────────────────────────────────────────────────────────────
# GPU-accelerated image preprocessing (nvJPEG decode + GPU resize/norm)
# ─────────────────────────────────────────────────────────────────────

# Sam3ImageProcessor config: resize to 1008x1008, rescale 1/255, norm mean=0.5 std=0.5
_TARGET_SIZE = 1008
_IMG_MEAN = 0.5
_IMG_STD = 0.5


class GpuPreprocessor:
    """Replace Sam3Processor's image pipeline with GPU-accelerated ops.

    Pipeline:
      1. Decode JPEG on GPU via torchvision.io.decode_jpeg (nvJPEG backend)
      2. Resize to 1008x1008 on GPU via F.interpolate (bilinear)
      3. Normalize: (pixel / 255 - 0.5) / 0.5 on GPU

    Falls back to cv2 CPU decode if nvJPEG is unavailable or the file
    is not JPEG.
    """

    def __init__(self, device: torch.device):
        self.device = device
        self._nvjpeg_ok = _HAS_TORCHVISION_IO
        if self._nvjpeg_ok:
            try:
                dummy = torch.tensor([0xFF, 0xD8], dtype=torch.uint8)
                torchvision.io.decode_jpeg(dummy, device=str(device))
            except Exception:
                pass

    def preprocess_from_bytes(self, raw_bytes: bytes) -> torch.Tensor:
        """Decode JPEG bytes and preprocess entirely on GPU.

        Args:
            raw_bytes: Raw JPEG file content.

        Returns:
            pixel_values: [1, 3, 1008, 1008] float32 tensor on self.device.
        """
        buf = torch.frombuffer(bytearray(raw_bytes), dtype=torch.uint8)

        if self._nvjpeg_ok and _is_jpeg(raw_bytes):
            try:
                img = torchvision.io.decode_jpeg(buf, device=str(self.device))
                return self._resize_and_normalize(img)
            except Exception:
                pass

        return self._fallback_decode(buf, raw_bytes)

    def preprocess_from_pil(self, image: Image.Image) -> torch.Tensor:
        """Preprocess a PIL image (GPU resize + normalize, no nvJPEG)."""
        arr = np.array(image)
        img = torch.from_numpy(arr).permute(2, 0, 1).to(self.device)
        return self._resize_and_normalize(img)

    def preprocess_from_ndarray(
        self, arr: np.ndarray, channel_order: str = "rgb",
    ) -> torch.Tensor:
        """Preprocess an HxWx3 uint8 ndarray (e.g. from ``cv_bridge``)
        with GPU-side resize and normalize.

        Skips PIL and (for ``channel_order='bgr'``) the CPU
        ``cv2.cvtColor`` swap, by doing the channel flip on the GPU
        after the H2D copy.

        Args:
            arr: HxWx3 uint8 ndarray in either RGB or BGR layout.
            channel_order: ``'rgb'`` (default) or ``'bgr'``.
        """
        order = channel_order.lower()
        if order not in ("rgb", "bgr"):
            raise ValueError(f"channel_order must be 'rgb' or 'bgr', got {channel_order!r}")
        if arr.ndim != 3 or arr.shape[2] != 3:
            raise ValueError(f"expected HxWx3 ndarray, got shape {arr.shape}")

        img = torch.from_numpy(np.ascontiguousarray(arr)).permute(2, 0, 1).to(self.device)
        if order == "bgr":
            img = img.flip(0)
        return self._resize_and_normalize(img)

    def _resize_and_normalize(self, img: torch.Tensor) -> torch.Tensor:
        """img: [3, H, W] uint8 or float on GPU → [1, 3, 1008, 1008] float32."""
        x = img.unsqueeze(0).float()
        if x.shape[2] != _TARGET_SIZE or x.shape[3] != _TARGET_SIZE:
            x = F.interpolate(x, size=(_TARGET_SIZE, _TARGET_SIZE),
                              mode="bilinear", align_corners=False)
        x = (x / 255.0 - _IMG_MEAN) / _IMG_STD
        return x

    def _fallback_decode(self, buf: torch.Tensor, raw_bytes: bytes) -> torch.Tensor:
        """CPU decode via cv2 or PIL, then GPU resize + normalize."""
        import cv2
        arr = np.frombuffer(raw_bytes, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError("Failed to decode image")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        img = torch.from_numpy(rgb).permute(2, 0, 1).to(self.device)
        return self._resize_and_normalize(img)


def _is_jpeg(data: bytes) -> bool:
    return len(data) >= 2 and data[0] == 0xFF and data[1] == 0xD8


# ─────────────────────────────────────────────────────────────────────
# TRT Engine wrapper
# ─────────────────────────────────────────────────────────────────────

def _trt_dtype_to_torch(trt_dtype):
    mapping = {
        trt.float32: torch.float32,
        trt.float16: torch.float16,
        trt.int32: torch.int32,
        trt.int8: torch.int8,
        trt.bool: torch.bool,
    }
    if hasattr(trt, "int64"):
        mapping[trt.int64] = torch.int64
    return mapping.get(trt_dtype, torch.float32)


class TRTEngine:
    """Thin wrapper around a single TensorRT engine."""

    def __init__(self, engine_path: str, device: torch.device):
        if trt is None:
            raise RuntimeError("tensorrt package is required")
        self.device = device
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)

        with open(engine_path, "rb") as f:
            self.engine = self.runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

        self.input_names = []
        self.output_names = []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                self.input_names.append(name)
            else:
                self.output_names.append(name)

        self.stream = torch.cuda.Stream(device=device)

    def infer(self, inputs: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        for name in self.input_names:
            tensor = inputs[name].contiguous()
            self.context.set_input_shape(name, tuple(tensor.shape))
            self.context.set_tensor_address(name, tensor.data_ptr())

        outputs = {}
        for name in self.output_names:
            shape = self.context.get_tensor_shape(name)
            dtype_trt = self.engine.get_tensor_dtype(name)
            dtype_torch = _trt_dtype_to_torch(dtype_trt)
            out = torch.empty(tuple(shape), dtype=dtype_torch, device=self.device)
            self.context.set_tensor_address(name, out.data_ptr())
            outputs[name] = out

        # Ensure input tensors produced on other streams (e.g. the default
        # stream used by the GPU preprocessor) are ready before TRT reads them.
        event = torch.cuda.current_stream(self.device).record_event()
        self.stream.wait_event(event)

        with torch.cuda.stream(self.stream):
            self.context.execute_async_v3(self.stream.cuda_stream)
        self.stream.synchronize()
        return outputs


# ─────────────────────────────────────────────────────────────────────
# Postprocessing helpers
# ─────────────────────────────────────────────────────────────────────

def _postprocess_single(
    pred_masks: torch.Tensor,
    pred_boxes: torch.Tensor,
    pred_logits: torch.Tensor,
    presence_logits: torch.Tensor,
    original_hw: Tuple[int, int],
    confidence_threshold: float,
) -> Dict[str, Any]:
    """Filter predictions for a single (image, text) pair, rescale to original size."""
    h, w = original_hw

    probs = pred_logits.sigmoid()
    if presence_logits is not None:
        presence = presence_logits.sigmoid().view(1)
        probs = probs * presence
    while probs.ndim > 1:
        probs = probs.max(dim=-1).values

    keep = probs > confidence_threshold
    if keep.sum() == 0:
        return {"masks": [], "boxes": [], "scores": []}

    scores = probs[keep]
    boxes = pred_boxes[keep]

    scale = torch.tensor([w, h, w, h], device=boxes.device, dtype=boxes.dtype)
    boxes_xyxy = boxes * scale

    masks = torch.nn.functional.interpolate(
        pred_masks[keep].unsqueeze(1).float(),
        size=(h, w), mode="bilinear", align_corners=False,
    ).sigmoid().squeeze(1)
    masks = masks > 0.5

    return {"masks": masks, "boxes": boxes_xyxy, "scores": scores}


# ─────────────────────────────────────────────────────────────────────
# Main inference class
# ─────────────────────────────────────────────────────────────────────

class HFSam3TRTInference:
    """
    Full HF SAM3 TensorRT inference pipeline.

    Two TRT engines:
      1. VisionEncoder — static [1, 3, 1008, 1008], runs once per image
      2. DetectorBody  — dynamic batch N (one per text prompt), runs once per image

    Vision features from engine 1 are repeated to batch=N before engine 2.
    """

    def __init__(
        self,
        engine_dir: str,
        model_name: str = "sam3/sam3",
        confidence_threshold: float = 0.5,
        device: str = "cuda:0",
        use_gpu_preprocess: bool = True,
    ):
        self.device = torch.device(device)
        self.confidence_threshold = confidence_threshold

        self.vision_engine = TRTEngine(
            os.path.join(engine_dir, "vision_encoder.plan"), self.device,
        )
        self.detector_engine = TRTEngine(
            os.path.join(engine_dir, "detector_body.plan"), self.device,
        )

        self.processor = self._load_processor(model_name)
        self.gpu_preprocessor = GpuPreprocessor(self.device) if use_gpu_preprocess else None

        self._cached_text_inputs: Optional[Dict[str, torch.Tensor]] = None
        self._cached_class_list: Optional[List[str]] = None

    @staticmethod
    def _load_processor(model_name: str):
        from transformers import AutoProcessor, AutoTokenizer, AutoImageProcessor

        try:
            from transformers import Sam3Processor
            processor = AutoProcessor.from_pretrained(model_name)
            if not isinstance(processor, Sam3Processor):
                raise TypeError("need Sam3Processor")
        except (TypeError, Exception):
            from transformers import Sam3Processor
            image_processor = AutoImageProcessor.from_pretrained(model_name)
            tokenizer = AutoTokenizer.from_pretrained(model_name)
            processor = Sam3Processor(
                image_processor=image_processor, tokenizer=tokenizer, target_size=1008,
            )
        return processor

    def cache_text_inputs(self, class_list: List[str]) -> Dict[str, torch.Tensor]:
        """Tokenize text prompts once and cache on GPU for reuse."""
        if self._cached_class_list == class_list and self._cached_text_inputs is not None:
            return self._cached_text_inputs
        text_inputs = self.processor(
            text=class_list, return_tensors="pt", padding=True,
        ).to(self.device)
        self._cached_text_inputs = {
            "input_ids": text_inputs.input_ids,
            "attention_mask": text_inputs.attention_mask,
        }
        self._cached_class_list = list(class_list)
        return self._cached_text_inputs

    @torch.inference_mode()
    def predict(
        self,
        pil_image: Image.Image,
        class_list: List[str],
    ) -> Dict[str, Any]:
        """Run inference on one image with N text prompts.

        Returns dict with:
            results_by_class: {class_name: {masks, boxes, scores, count}}
            total_objects: int
        """
        h, w = pil_image.height, pil_image.width
        N = len(class_list)

        # ── 1. Preprocess image & run VisionEncoder ─────────────────
        if self.gpu_preprocessor is not None:
            pixel_values = self.gpu_preprocessor.preprocess_from_pil(pil_image)
        else:
            img_inputs = self.processor(
                images=pil_image, return_tensors="pt",
            ).to(self.device)
            pixel_values = img_inputs.pixel_values

        vision_out = self.vision_engine.infer({"pixel_values": pixel_values})

        # ── 2. Repeat vision features to batch=N ────────────────────
        fpn0 = vision_out["fpn_feat_0"].repeat(N, 1, 1, 1)
        fpn1 = vision_out["fpn_feat_1"].repeat(N, 1, 1, 1)
        fpn2 = vision_out["fpn_feat_2"].repeat(N, 1, 1, 1)
        pos2 = vision_out["fpn_pos_2"].repeat(N, 1, 1, 1)

        # ── 3. Get text tokens (cached) ─────────────────────────────
        text = self.cache_text_inputs(class_list)

        # ── 4. Run DetectorBody ─────────────────────────────────────
        detector_out = self.detector_engine.infer({
            "fpn_feat_0": fpn0, "fpn_feat_1": fpn1,
            "fpn_feat_2": fpn2, "fpn_pos_2": pos2,
            "input_ids": text["input_ids"],
            "attention_mask": text["attention_mask"],
        })

        # ── 5. Postprocess per class ────────────────────────────────
        return self._postprocess_all(detector_out, class_list, (h, w))

    @torch.inference_mode()
    def predict_from_bytes(
        self,
        raw_bytes: bytes,
        original_hw: Tuple[int, int],
        class_list: List[str],
    ) -> Dict[str, Any]:
        """Run inference from raw JPEG bytes (nvJPEG GPU decode path).

        Args:
            raw_bytes: Raw image file content.
            original_hw: (height, width) of the original image (for postprocessing).
            class_list: Text prompts.
        """
        h, w = original_hw
        N = len(class_list)

        # ── 1. GPU decode + preprocess + VisionEncoder ──────────────
        if self.gpu_preprocessor is not None:
            pixel_values = self.gpu_preprocessor.preprocess_from_bytes(raw_bytes)
        else:
            img = Image.open(__import__("io").BytesIO(raw_bytes)).convert("RGB")
            img_inputs = self.processor(images=img, return_tensors="pt").to(self.device)
            pixel_values = img_inputs.pixel_values

        vision_out = self.vision_engine.infer({"pixel_values": pixel_values})

        # ── 2. Repeat + text + detector ─────────────────────────────
        fpn0 = vision_out["fpn_feat_0"].repeat(N, 1, 1, 1)
        fpn1 = vision_out["fpn_feat_1"].repeat(N, 1, 1, 1)
        fpn2 = vision_out["fpn_feat_2"].repeat(N, 1, 1, 1)
        pos2 = vision_out["fpn_pos_2"].repeat(N, 1, 1, 1)

        text = self.cache_text_inputs(class_list)

        detector_out = self.detector_engine.infer({
            "fpn_feat_0": fpn0, "fpn_feat_1": fpn1,
            "fpn_feat_2": fpn2, "fpn_pos_2": pos2,
            "input_ids": text["input_ids"],
            "attention_mask": text["attention_mask"],
        })

        return self._postprocess_all(detector_out, class_list, (h, w))

    @torch.inference_mode()
    def predict_from_ndarray(
        self,
        arr: np.ndarray,
        original_hw: Tuple[int, int],
        class_list: List[str],
        channel_order: str = "rgb",
    ) -> Dict[str, Any]:
        """Run inference on an HxWx3 uint8 ndarray (e.g. from ``cv_bridge``).

        Skips PIL and (for BGR inputs) the CPU ``cv2.cvtColor`` swap;
        the channel flip happens on the GPU.  This is the closest
        analogue to :meth:`predict_from_bytes` for already-decoded
        frames coming off a ROS topic or capture device.

        Args:
            arr: HxWx3 uint8 ndarray.
            original_hw: ``(height, width)`` of the original image
                (used for postprocessing rescale).
            class_list: Text prompts.
            channel_order: ``'rgb'`` or ``'bgr'``.
        """
        h, w = original_hw
        N = len(class_list)

        if self.gpu_preprocessor is not None:
            pixel_values = self.gpu_preprocessor.preprocess_from_ndarray(
                arr, channel_order=channel_order,
            )
        else:
            arr_rgb = arr[:, :, ::-1] if channel_order.lower() == "bgr" else arr
            pil_image = Image.fromarray(np.ascontiguousarray(arr_rgb))
            img_inputs = self.processor(images=pil_image, return_tensors="pt").to(self.device)
            pixel_values = img_inputs.pixel_values

        vision_out = self.vision_engine.infer({"pixel_values": pixel_values})

        fpn0 = vision_out["fpn_feat_0"].repeat(N, 1, 1, 1)
        fpn1 = vision_out["fpn_feat_1"].repeat(N, 1, 1, 1)
        fpn2 = vision_out["fpn_feat_2"].repeat(N, 1, 1, 1)
        pos2 = vision_out["fpn_pos_2"].repeat(N, 1, 1, 1)

        text = self.cache_text_inputs(class_list)

        detector_out = self.detector_engine.infer({
            "fpn_feat_0": fpn0, "fpn_feat_1": fpn1,
            "fpn_feat_2": fpn2, "fpn_pos_2": pos2,
            "input_ids": text["input_ids"],
            "attention_mask": text["attention_mask"],
        })

        return self._postprocess_all(detector_out, class_list, (h, w))

    def _postprocess_all(
        self, detector_out: Dict[str, torch.Tensor],
        class_list: List[str], original_hw: Tuple[int, int],
    ) -> Dict[str, Any]:
        results_by_class = {}
        total_objects = 0
        for i, class_name in enumerate(class_list):
            res = _postprocess_single(
                detector_out["pred_masks"][i], detector_out["pred_boxes"][i],
                detector_out["pred_logits"][i], detector_out["presence_logits"][i],
                original_hw, self.confidence_threshold,
            )
            n = len(res["scores"]) if isinstance(res["scores"], (list, torch.Tensor)) else 0
            if isinstance(res["scores"], torch.Tensor):
                n = res["scores"].numel()
            if n > 0:
                results_by_class[class_name] = {
                    "masks": res["masks"], "boxes": res["boxes"],
                    "scores": res["scores"], "count": n,
                }
                total_objects += n
        return {
            "results_by_class": results_by_class,
            "total_objects": total_objects,
            "classes_detected": list(results_by_class.keys()),
        }
