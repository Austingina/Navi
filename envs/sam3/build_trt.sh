#!/usr/bin/env bash
# Build machine-specific SAM3 engines from the repository-local checkpoint.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAVI_REPO="$(cd "$HERE/../.." && pwd)"
SAM3_CHECKPOINT_DIR="${SAM3_CHECKPOINT_DIR:-$NAVI_REPO/checkpoints/sam3}"
SAM3_ENGINE_DIR="${SAM3_ENGINE_DIR:-$SAM3_CHECKPOINT_DIR/trt_engines}"
SAM3_ONNX_DIR="${SAM3_ONNX_DIR:-$SAM3_CHECKPOINT_DIR/onnx}"
SAM3_RUNTIME="$HERE/.venv/bin/python"
if [[ "${1:-}" == '--help' ]]; then
    echo 'Usage: bash envs/sam3/build_trt.sh [--skip-export]'
    echo 'Environment: SAM3_CHECKPOINT_DIR, SAM3_ENGINE_DIR, SAM3_ONNX_DIR.'
    echo 'Requires the repository SAM3 venv, onnx, and matching system TensorRT.'
    exit 0
fi
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != '--skip-export' ) ]]; then
    echo 'Unknown argument; use --help' >&2
    exit 2
fi
"$SAM3_RUNTIME" -c 'import tensorrt, torch; assert torch.cuda.is_available(); print("TensorRT", tensorrt.__version__)'
if [[ "${1:-}" != '--skip-export' ]]; then
    "$SAM3_RUNTIME" -c 'import onnx' || {
        echo "Install export dependency: $SAM3_RUNTIME -m pip install -r $HERE/requirements-export.txt" >&2
        exit 1
    }
    "$SAM3_RUNTIME" "$NAVI_REPO/sam3/export_hf/export_onnx.py" \
        --model "$SAM3_CHECKPOINT_DIR" \
        --config "$NAVI_REPO/ros2_ws/src/navi_semantic_perception/models/sam3" \
        --output-dir "$SAM3_ONNX_DIR" --device cpu
fi
"$SAM3_RUNTIME" "$NAVI_REPO/sam3/export_hf/build_trt.py" \
    --onnx-dir "$SAM3_ONNX_DIR" --engine-dir "$SAM3_ENGINE_DIR" --max-prompts 1
