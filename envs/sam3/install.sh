#!/usr/bin/env bash
# Create the portable SAM3 runtime and fetch assets; see README.md in this directory.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAVI_REPO="$(cd "$HERE/../.." && pwd)"
SAM3_CHECKPOINT_DIR="${SAM3_CHECKPOINT_DIR:-$NAVI_REPO/checkpoints/sam3}"
SAM3_PYTHON="${SAM3_PYTHON:-/usr/bin/python3}"
SAM3_TORCH_INDEX_URL="${SAM3_TORCH_INDEX_URL:-https://download.pytorch.org/whl/cu130}"
SAM3_TORCH_VERSION="${SAM3_TORCH_VERSION:-2.13.0}"
SAM3_TORCHVISION_VERSION="${SAM3_TORCHVISION_VERSION:-0.28.0}"
env_only=false
local_source=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --env-only) env_only=true; shift ;;
        --from-local)
            [[ $# -ge 2 ]] || { echo '--from-local requires a SAM3 asset directory' >&2; exit 2; }
            local_source="$2"; shift 2 ;;
        -h|--help)
            cat <<'HELP'
Usage: bash envs/sam3/install.sh [--env-only | --from-local /path/to/sam3-assets]

Default: create <Navi>/envs/sam3/.venv; keep weights in <Navi>/checkpoints/sam3.
--env-only    Install/check Python dependencies without downloading model weights.
--from-local  Import an existing model.safetensors file or deployment directory.

Environment:
  SAM3_CHECKPOINT_DIR   Local weight directory (default: <Navi>/checkpoints/sam3)
  SAM3_PYTHON           ROS-compatible Python (default: /usr/bin/python3)
  SAM3_TORCH_INDEX_URL  PyTorch wheel index (default: official cu130 for Thor)
  SAM3_TORCH_VERSION    Version installed when CUDA torch is missing (default: 2.13.0)
  SAM3_TORCHVISION_VERSION Matching torchvision version (default: 0.28.0)
  HF_TOKEN              Optional HF token; otherwise uses existing `hf auth login`

Existing compatible CUDA torch/torchvision are reused. For another GPU/driver,
select its PyTorch index or install the matching vendor wheels before this script.
HELP
            exit 0 ;;
        *) echo "Unknown option: $1 (use --help)" >&2; exit 2 ;;
    esac
done
if [[ "$env_only" == true && -n "$local_source" ]]; then
    echo '--env-only and --from-local are mutually exclusive' >&2
    exit 2
fi

if [[ ! -x "$HERE/.venv/bin/python" ]]; then
    "$SAM3_PYTHON" -m venv --system-site-packages "$HERE/.venv"
fi
SAM3_RUNTIME="$HERE/.venv/bin/python"
# Use the same minor version as ROS, never a Conda Python with a different ABI.
expected_abi="$("$SAM3_PYTHON" -c 'import sys; print(sys.version_info[:2])')"
runtime_abi="$("$SAM3_RUNTIME" -c 'import sys; print(sys.version_info[:2])')"
if [[ "$expected_abi" != "$runtime_abi" ]]; then
    echo "SAM3 Python ABI $runtime_abi != ROS Python ABI $expected_abi" >&2
    exit 1
fi

install_packages() {
    if command -v uv >/dev/null 2>&1; then
        uv pip install --python "$SAM3_RUNTIME" "$@"
    else
        "$SAM3_RUNTIME" -m pip install "$@"
    fi
}

if ! "$SAM3_RUNTIME" - <<'PY'
import torch
import torchvision
assert tuple(int(n) for n in torch.__version__.split('.')[:2]) >= (2, 7), 'torch >= 2.7 required'
assert torch.cuda.is_available(), 'CUDA is unavailable'
assert hasattr(torchvision.ops, 'nms'), 'torchvision is incomplete'
print('Reusing torch', torch.__version__, 'torchvision', torchvision.__version__)
PY
then
    install_packages --force-reinstall \
        "torch==$SAM3_TORCH_VERSION" "torchvision==$SAM3_TORCHVISION_VERSION" \
        --index-url "$SAM3_TORCH_INDEX_URL"
fi
install_packages -r "$HERE/requirements.txt"
"$SAM3_RUNTIME" - <<'PY'
import torch
import torchvision
from transformers import Sam3Model, Sam3Processor
assert torch.cuda.is_available(), 'Install CUDA torch/torchvision matching the target GPU/driver'
print('SAM3 imports OK; GPU:', torch.cuda.get_device_name(0))
PY

if [[ "$env_only" == false ]]; then
    args=(--model-dir "$SAM3_CHECKPOINT_DIR")
    if [[ -n "$local_source" ]]; then
        args+=(--from-local "$local_source")
    fi
    "$SAM3_RUNTIME" "$HERE/prepare_assets.py" "${args[@]}"
fi
printf 'SAM3 runtime: %s\n' "$SAM3_RUNTIME"
printf 'Checkpoint directory: %s\n' "$SAM3_CHECKPOINT_DIR"
