#!/usr/bin/env python3
"""
ROS 2 node for real-time SAM 3 TensorRT inference on streaming camera images.

Subscribes to a color image topic, runs the 2-engine TRT pipeline
(VisionEncoder + DetectorBody), applies NMS, and publishes:
  - /sam3/detections/image  — annotated image (masks + boxes + labels)
  - /sam3/detections/json   — JSON string with per-frame detection results

Usage:
    source .venv/bin/activate
    source /opt/ros/humble/setup.bash
    python export_hf/ros2_trt_node.py --engine_dir export_hf/trt_engines
"""

import argparse
import json
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from export_hf.trt_infer_helper import HFSam3TRTInference
from export_hf.trt_infer import (
    FOREGROUND_CLASSES,
    NMS_EXCLUDE_PAIRS,
    filter_intra_class_nms,
    filter_cross_class_nms,
    extract_detections,
    COLORS,
    color_to_bgr,
    draw_mask_cv2,
    draw_bbox_cv2,
)


class Sam3TRTDetector(Node):
    """ROS 2 node that runs TensorRT SAM 3 on each incoming camera frame."""

    def __init__(
        self,
        engine_dir: str,
        model_name: str,
        class_list: List[str],
        confidence_threshold: float = 0.6,
        gpu_id: int = 0,
        image_topic: str = "/camera/camera/color/image_raw",
        intra_nms_threshold: float = 0.5,
        nms_iou_threshold: float = 0.8,
        nms_proximity_threshold: float = 20.0,
        nms_min_mask_ratio: float = 0.0001,
    ):
        super().__init__("sam3_trt_detector")
        self.bridge = CvBridge()
        self.class_list = class_list
        self.confidence_threshold = confidence_threshold
        self.intra_nms_threshold = intra_nms_threshold
        self.nms_iou_threshold = nms_iou_threshold
        self.nms_proximity_threshold = nms_proximity_threshold
        self.nms_min_mask_ratio = nms_min_mask_ratio

        self._latest_msg: Optional[Image] = None
        self._lock = threading.Lock()
        self._processing = False
        self._frame_count = 0
        self._total_infer_time = 0.0

        device = torch.device(f"cuda:{gpu_id}")
        self.get_logger().info(f"Loading TRT engines from {engine_dir} ...")
        self.engine = HFSam3TRTInference(
            engine_dir=engine_dir,
            model_name=model_name,
            confidence_threshold=confidence_threshold,
            device=str(device),
            use_gpu_preprocess=True,
        )
        self.engine.cache_text_inputs(class_list)
        self._warmup(device)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub_image = self.create_subscription(
            Image, image_topic, self._image_cb, qos,
        )

        self.pub_vis = self.create_publisher(Image, "/sam3/detections/image", 10)
        self.pub_json = self.create_publisher(String, "/sam3/detections/json", 10)

        self._timer = self.create_timer(0.01, self._process_latest)

        self.get_logger().info(
            f"Ready — subscribed to {image_topic}, "
            f"classes={class_list}, confidence={confidence_threshold}"
        )

    def _warmup(self, device: torch.device):
        """Run a dummy forward pass to pre-allocate CUDA memory.

        Uses the same ndarray entry point as the live path so JIT and
        kernel caches are warm for the steady-state code path.
        """
        self.get_logger().info("Running warmup forward pass ...")
        dummy = np.zeros((720, 1280, 3), dtype=np.uint8)
        self.engine.predict_from_ndarray(
            dummy, (720, 1280), self.class_list, channel_order="bgr",
        )
        if torch.cuda.is_available():
            torch.cuda.synchronize(device)
        self.get_logger().info("Warmup complete.")

    def _image_cb(self, msg: Image):
        with self._lock:
            self._latest_msg = msg

    def _process_latest(self):
        """Timer callback: grab the latest frame and run inference if idle."""
        with self._lock:
            if self._latest_msg is None or self._processing:
                return
            msg = self._latest_msg
            self._latest_msg = None
            self._processing = True

        try:
            self._run_inference(msg)
        finally:
            with self._lock:
                self._processing = False

    @torch.inference_mode()
    def _run_inference(self, msg: Image):
        t0 = time.perf_counter()

        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img_h, img_w = bgr.shape[:2]

        result = self.engine.predict_from_ndarray(
            bgr, (img_h, img_w), self.class_list, channel_order="bgr",
        )

        if self.intra_nms_threshold > 0:
            result = filter_intra_class_nms(
                result, iou_threshold=self.intra_nms_threshold,
            )
        if self.nms_iou_threshold > 0:
            result = filter_cross_class_nms(
                result,
                image_hw=(img_h, img_w),
                iou_threshold=self.nms_iou_threshold,
                proximity_threshold=self.nms_proximity_threshold,
                min_mask_size_ratio=self.nms_min_mask_ratio,
            )

        infer_ms = (time.perf_counter() - t0) * 1000
        self._frame_count += 1
        self._total_infer_time += infer_ms
        avg_ms = self._total_infer_time / self._frame_count

        vis_img = self._draw_detections(bgr, result, infer_ms)
        vis_msg = self.bridge.cv2_to_imgmsg(vis_img, encoding="bgr8")
        vis_msg.header = msg.header
        self.pub_vis.publish(vis_msg)

        det_json = {
            "stamp": {
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec,
            },
            "frame_id": msg.header.frame_id,
            "total_objects": result["total_objects"],
            "inference_ms": round(infer_ms, 1),
            "detections": extract_detections(result),
        }
        json_msg = String()
        json_msg.data = json.dumps(det_json)
        self.pub_json.publish(json_msg)

        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f"Frame {self._frame_count}: {result['total_objects']} objects, "
                f"{infer_ms:.0f}ms (avg {avg_ms:.0f}ms)"
            )

    def _draw_detections(
        self, bgr: np.ndarray, result: Dict[str, Any], infer_ms: float,
    ) -> np.ndarray:
        vis = bgr.copy()
        all_results = result["results_by_class"]
        total_objects = result["total_objects"]

        if total_objects > 0:
            obj_idx = 0
            legend = []
            for class_name, data in all_results.items():
                masks, boxes, scores = data["masks"], data["boxes"], data["scores"]
                count = data["count"]
                for i in range(count):
                    c_bgr = color_to_bgr(COLORS[obj_idx % len(COLORS)])
                    obj_idx += 1
                    mask = masks[i] if count > 1 else masks
                    box = boxes[i] if count > 1 else boxes
                    score = scores[i] if count > 1 else scores
                    mask_2d = (
                        mask.squeeze().cpu().numpy()
                        if isinstance(mask, torch.Tensor)
                        else np.array(mask).squeeze()
                    )
                    while mask_2d.ndim > 2:
                        mask_2d = mask_2d[0]
                    draw_mask_cv2(vis, mask_2d, c_bgr)
                    prob = score.item() if hasattr(score, "item") else float(score)
                    box_flat = (
                        box.cpu().numpy().squeeze().flatten()
                        if isinstance(box, torch.Tensor)
                        else np.array(box).squeeze().flatten()
                    )
                    draw_bbox_cv2(vis, box_flat, f"{class_name}: {prob:.2f}", c_bgr)
                legend.append(f"{class_name}: {count}")
            title = f"{total_objects} obj | {', '.join(legend)} | {infer_ms:.0f}ms"
        else:
            title = f"No detections | {infer_ms:.0f}ms"

        font = cv2.FONT_HERSHEY_DUPLEX
        (tw, th), _ = cv2.getTextSize(title, font, 0.7, 1)
        pad = 10
        cv2.rectangle(vis, (0, 0), (tw + pad * 2, th + pad * 2), (0, 0, 0), -1)
        cv2.putText(
            vis, title, (pad, th + pad), font, 0.7, (255, 255, 255), 1, cv2.LINE_AA,
        )
        return vis


def main():
    parser = argparse.ArgumentParser(description="ROS 2 SAM 3 TensorRT detector node")
    parser.add_argument(
        "--engine_dir", type=str, default="export_hf/trt_engines",
        help="Directory containing vision_encoder.plan and detector_body.plan",
    )
    parser.add_argument(
        "--model_name", type=str, default="sam3/sam3",
        help="HuggingFace model name or local path (for processor/tokenizer)",
    )
    parser.add_argument("--classes", type=str, nargs="+", default=None)
    parser.add_argument("--confidence_threshold", type=float, default=0.5)
    parser.add_argument("--gpu", type=int, default=0)
    parser.add_argument(
        "--image_topic", type=str, default="/camera/camera/color/image_raw",
    )
    parser.add_argument("--intra_nms_threshold", type=float, default=0.5)
    parser.add_argument("--nms_iou_threshold", type=float, default=0.8)
    parser.add_argument("--nms_proximity_threshold", type=float, default=20.0)
    parser.add_argument("--nms_min_mask_ratio", type=float, default=0.0001)

    args, ros_args = parser.parse_known_args()
    class_list = args.classes if args.classes else FOREGROUND_CLASSES

    rclpy.init(args=ros_args)
    node = Sam3TRTDetector(
        engine_dir=args.engine_dir,
        model_name=args.model_name,
        class_list=class_list,
        confidence_threshold=args.confidence_threshold,
        gpu_id=args.gpu,
        image_topic=args.image_topic,
        intra_nms_threshold=args.intra_nms_threshold,
        nms_iou_threshold=args.nms_iou_threshold,
        nms_proximity_threshold=args.nms_proximity_threshold,
        nms_min_mask_ratio=args.nms_min_mask_ratio,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f"Shutting down — processed {node._frame_count} frames"
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
