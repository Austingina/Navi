#!/usr/bin/env python3
"""
Build TensorRT engines from the 2 exported HF SAM3 ONNX files.

Usage:
    python -m export_hf.build_trt [--onnx-dir DIR] [--engine-dir DIR] [--verbose]

Prerequisites:
    - pip install tensorrt-cu12-bindings tensorrt-cu12-libs tensorrt-cu12
    - ONNX files exported via: python -m export_hf.export_onnx
"""

import argparse
import os
import time

import tensorrt as trt

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)


def build_engine(
    onnx_path: str,
    engine_path: str,
    verbose: bool = False,
    dynamic_shapes: dict | None = None,
    workspace_gb: int = 10,
):
    """Build an FP16 TensorRT engine from an ONNX file.

    Args:
        dynamic_shapes: dict mapping input name -> (min_shape, opt_shape, max_shape).
    """
    if verbose:
        TRT_LOGGER.min_severity = trt.Logger.VERBOSE

    builder = trt.Builder(TRT_LOGGER)
    network = builder.create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    )
    parser = trt.OnnxParser(network, TRT_LOGGER)

    print(f"  Parsing {onnx_path} ...")
    with open(onnx_path, "rb") as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                print(f"    ONNX parse error: {parser.get_error(i)}")
            raise RuntimeError(f"Failed to parse {onnx_path}")

    config = builder.create_builder_config()
    config.set_memory_pool_limit(
        trt.MemoryPoolType.WORKSPACE, int(workspace_gb * (1 << 30))
    )

    if not builder.platform_has_fast_fp16:
        print("  WARNING: FP16 not natively supported on this GPU")
    config.set_flag(trt.BuilderFlag.FP16)

    if dynamic_shapes:
        profile = builder.create_optimization_profile()
        for input_name, (min_s, opt_s, max_s) in dynamic_shapes.items():
            profile.set_shape(input_name, min_s, opt_s, max_s)
        config.add_optimization_profile(profile)

    print("  Building engine (this may take several minutes) ...")
    t0 = time.time()
    serialized = builder.build_serialized_network(network, config)
    elapsed = time.time() - t0

    if serialized is None:
        raise RuntimeError(f"Engine build failed for {onnx_path}")

    with open(engine_path, "wb") as f:
        f.write(serialized)

    size_mb = os.path.getsize(engine_path) / (1024 * 1024)
    print(f"  Saved {engine_path} ({size_mb:.1f} MB) in {elapsed:.1f}s")


def main():
    p = argparse.ArgumentParser(description="Build TRT engines from HF ONNX")
    p.add_argument("--onnx-dir", default="export_hf/onnx_weights")
    p.add_argument("--engine-dir", default="export_hf/trt_engines")
    p.add_argument(
        "--max-prompts", type=int, default=8,
        help="Max number of text prompts (dynamic batch dim upper bound)",
    )
    p.add_argument("--verbose", action="store_true")
    args = p.parse_args()

    if args.max_prompts < 1:
        p.error("--max-prompts must be positive")
    os.makedirs(args.engine_dir, exist_ok=True)

    print("=" * 60)
    print("Building TensorRT engines (HF SAM3, FP16)")
    print(f"  TensorRT version : {trt.__version__}")
    print(f"  ONNX dir         : {args.onnx_dir}")
    print(f"  Engine dir       : {args.engine_dir}")
    print(f"  Max prompts      : {args.max_prompts}")
    print("=" * 60)

    # ── 1. VisionEncoder (static shape) ─────────────────────────────
    print("\n[1/2] VisionEncoder (static shape, FP16)")
    build_engine(
        onnx_path=os.path.join(args.onnx_dir, "vision_encoder.onnx"),
        engine_path=os.path.join(args.engine_dir, "vision_encoder.plan"),
        verbose=args.verbose,
    )

    # ── 2. DetectorBody (dynamic batch = num_prompts) ───────────────
    MP = args.max_prompts
    optimum = min(4, MP)
    D = 256
    H0, W0 = 288, 288
    H1, W1 = 144, 144
    H2, W2 = 72, 72
    L = 32  # tokenizer context length

    print(f"\n[2/2] DetectorBody (dynamic batch 1..{MP}, FP16)")
    build_engine(
        onnx_path=os.path.join(args.onnx_dir, "detector_body.onnx"),
        engine_path=os.path.join(args.engine_dir, "detector_body.plan"),
        verbose=args.verbose,
        workspace_gb=16,
        dynamic_shapes={
            "fpn_feat_0":     ((1, D, H0, W0), (optimum, D, H0, W0), (MP, D, H0, W0)),
            "fpn_feat_1":     ((1, D, H1, W1), (optimum, D, H1, W1), (MP, D, H1, W1)),
            "fpn_feat_2":     ((1, D, H2, W2), (optimum, D, H2, W2), (MP, D, H2, W2)),
            "fpn_pos_2":      ((1, D, H2, W2), (optimum, D, H2, W2), (MP, D, H2, W2)),
            "input_ids":      ((1, L),         (optimum, L),          (MP, L)),
            "attention_mask": ((1, L),         (optimum, L),          (MP, L)),
        },
    )

    print("\n" + "=" * 60)
    print("TensorRT engines built:")
    for f in sorted(os.listdir(args.engine_dir)):
        if f.endswith(".plan"):
            size_mb = os.path.getsize(os.path.join(args.engine_dir, f)) / (1024 * 1024)
            print(f"  {f}: {size_mb:.1f} MB")
    print("=" * 60)


if __name__ == "__main__":
    main()
