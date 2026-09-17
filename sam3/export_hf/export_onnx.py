#!/usr/bin/env python3
"""
Export HuggingFace SAM3 to ONNX in 2 graphs:
  1. VisionEncoder  — ViT backbone + FPN neck  (static shape)
  2. DetectorBody   — Text encoder + DETR + Mask decoder  (dynamic batch N)

Usage:
    python -m export_hf.export_onnx [--model PATH] [--output-dir DIR] [--device DEVICE] [--opset INT]
"""

import argparse
import os
import sys
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from export_hf.hf_vision_wrapper import HFVisionEncoderONNX
from export_hf.hf_detector_wrapper import HFDetectorBodyONNX


def _force_eager_attention(model):
    """Force eager attention implementation on all sub-configs.

    SDPA / FlashAttention cannot be reliably traced to ONNX.  Switching to
    the explicit eager path (matmul + softmax) makes the graph portable.
    """
    for module in model.modules():
        cfg = getattr(module, "config", None)
        if cfg is not None and hasattr(cfg, "_attn_implementation"):
            cfg._attn_implementation = "eager"


def _patch_masking_utils_for_onnx():
    """Replace create_bidirectional_mask with a JIT-trace-safe version.

    The stock HF implementation dispatches through ``sdpa_mask`` which
    interprets traced-tensor shapes via Python indexing that fails during
    ``torch.jit.trace``.  We replace it with a simple padding-aware
    float mask built directly from the 2-D ``attention_mask``.
    """
    import transformers.masking_utils as mu
    import transformers.models.sam3.modeling_sam3 as sam3_mod

    def _bidirectional_mask_onnx(config, inputs_embeds, attention_mask,
                                 encoder_hidden_states=None, **kwargs):
        # Already a 4-D mask — pass through
        if attention_mask is not None and attention_mask.dim() == 4:
            return attention_mask

        embeds = encoder_hidden_states if encoder_hidden_states is not None else inputs_embeds
        bsz = embeds.shape[0]
        q_len = inputs_embeds.shape[1]
        kv_len = embeds.shape[1]

        if attention_mask is None:
            return None

        # attention_mask: [bsz, kv_len]  True=valid / 1=valid
        mask_float = attention_mask.to(dtype=embeds.dtype)
        # [bsz, 1, 1, kv_len] — broadcast over heads and query positions
        mask_4d = mask_float.unsqueeze(1).unsqueeze(2)
        # Convert: 0 → -inf, 1 → 0
        mask_4d = (1.0 - mask_4d) * torch.finfo(embeds.dtype).min
        return mask_4d.expand(bsz, 1, q_len, kv_len)

    mu.create_bidirectional_mask = _bidirectional_mask_onnx
    sam3_mod.create_bidirectional_mask = _bidirectional_mask_onnx


def load_hf_model(model_name: str, device: str = "cpu", config_path=None):
    """Load Sam3Model from a local or remote HuggingFace checkpoint."""
    from transformers import AutoModel, Sam3Config, Sam3Model

    _patch_masking_utils_for_onnx()

    print(f"Loading HuggingFace SAM3 model: {model_name}")
    if config_path:
        raw_model = Sam3Model.from_pretrained(
            model_name, config=Sam3Config.from_pretrained(config_path, local_files_only=True),
            dtype=torch.float32, attn_implementation="eager", local_files_only=True,
        )
    else:
        raw_model = AutoModel.from_pretrained(
            model_name, dtype=torch.float32, attn_implementation="eager",
        )
    raw_model = raw_model.to(device).eval()

    if isinstance(raw_model, Sam3Model):
        model = raw_model
    elif hasattr(raw_model, "detector_model"):
        print(f"  Loaded {type(raw_model).__name__}, extracting detector_model")
        model = raw_model.detector_model
    else:
        raise RuntimeError(
            f"Loaded model is {type(raw_model).__name__} with no detector_model attribute."
        )

    _force_eager_attention(model)
    model.eval()
    print(f"  Model on {device} ({type(model).__name__})")
    return model


def export_vision_encoder(model, output_dir, device, opset):
    print("=" * 60)
    print("Exporting VisionEncoder ...")
    print("=" * 60)

    wrapper = HFVisionEncoderONNX(model.vision_encoder).to(device).eval()

    dummy = torch.randn(1, 3, 1008, 1008, device=device)

    with torch.no_grad():
        fpn0, fpn1, fpn2, pos2 = wrapper(dummy)
    print(f"  fpn_feat_0 : {fpn0.shape}")
    print(f"  fpn_feat_1 : {fpn1.shape}")
    print(f"  fpn_feat_2 : {fpn2.shape}")
    print(f"  fpn_pos_2  : {pos2.shape}")

    onnx_path = os.path.join(output_dir, "vision_encoder.onnx")
    torch.onnx.export(
        wrapper,
        (dummy,),
        onnx_path,
        input_names=["pixel_values"],
        output_names=["fpn_feat_0", "fpn_feat_1", "fpn_feat_2", "fpn_pos_2"],
        opset_version=opset,
        dynamo=False,
    )
    print(f"  Saved to {onnx_path}")


def export_detector_body(model, output_dir, device, opset):
    print("=" * 60)
    print("Exporting DetectorBody ...")
    print("=" * 60)

    wrapper = HFDetectorBodyONNX(model).to(device).eval()

    N = 2  # dummy batch = 2 text prompts
    D = 256
    H0, W0 = 288, 288
    H1, W1 = 144, 144
    H2, W2 = 72, 72
    L = 32  # tokenizer context length

    dummy_fpn0 = torch.randn(N, D, H0, W0, device=device)
    dummy_fpn1 = torch.randn(N, D, H1, W1, device=device)
    dummy_fpn2 = torch.randn(N, D, H2, W2, device=device)
    dummy_pos2 = torch.randn(N, D, H2, W2, device=device)
    dummy_ids = torch.randint(0, 49408, (N, L), device=device)
    dummy_mask = torch.ones(N, L, dtype=torch.long, device=device)

    with torch.no_grad():
        pred_masks, pred_boxes, pred_logits, presence_logits = wrapper(
            dummy_fpn0, dummy_fpn1, dummy_fpn2, dummy_pos2,
            dummy_ids, dummy_mask,
        )
    print(f"  pred_masks      : {pred_masks.shape}")
    print(f"  pred_boxes      : {pred_boxes.shape}")
    print(f"  pred_logits     : {pred_logits.shape}")
    print(f"  presence_logits : {presence_logits.shape}")

    onnx_path = os.path.join(output_dir, "detector_body.onnx")
    torch.onnx.export(
        wrapper,
        (dummy_fpn0, dummy_fpn1, dummy_fpn2, dummy_pos2, dummy_ids, dummy_mask),
        onnx_path,
        input_names=[
            "fpn_feat_0", "fpn_feat_1", "fpn_feat_2", "fpn_pos_2",
            "input_ids", "attention_mask",
        ],
        output_names=["pred_masks", "pred_boxes", "pred_logits", "presence_logits"],
        dynamic_axes={
            "fpn_feat_0":      {0: "num_prompts"},
            "fpn_feat_1":      {0: "num_prompts"},
            "fpn_feat_2":      {0: "num_prompts"},
            "fpn_pos_2":       {0: "num_prompts"},
            "input_ids":       {0: "num_prompts"},
            "attention_mask":  {0: "num_prompts"},
            "pred_masks":      {0: "num_prompts"},
            "pred_boxes":      {0: "num_prompts"},
            "pred_logits":     {0: "num_prompts"},
            "presence_logits": {0: "num_prompts"},
        },
        opset_version=opset,
        dynamo=False,
    )
    print(f"  Saved to {onnx_path}")


def parse_args():
    p = argparse.ArgumentParser(description="Export HF SAM3 to ONNX (2 graphs)")
    p.add_argument(
        "--model", type=str,
        required=True,
        help="HuggingFace model name or local path",
    )
    p.add_argument("--config", help="Separate local model config directory (Navi layout)")
    p.add_argument("--output-dir", type=str, default="export_hf/onnx_weights")
    p.add_argument("--device", type=str, default="cpu",
                    help="Device for tracing (cpu recommended)")
    p.add_argument("--opset", type=int, default=17)
    return p.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    model = load_hf_model(args.model, args.device, args.config)

    export_vision_encoder(model, args.output_dir, args.device, args.opset)
    export_detector_body(model, args.output_dir, args.device, args.opset)

    print()
    print("=" * 60)
    print("Export complete.  ONNX files:")
    for f in sorted(os.listdir(args.output_dir)):
        fpath = os.path.join(args.output_dir, f)
        if os.path.isfile(fpath):
            size_mb = os.path.getsize(fpath) / (1024 * 1024)
            print(f"  {f}: {size_mb:.1f} MB")
    print()
    print("Next step: build TensorRT engines with export_hf/build_trt.py")


if __name__ == "__main__":
    main()
