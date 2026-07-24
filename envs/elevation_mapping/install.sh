#!/bin/bash
# elevation_mapping 专用虚拟环境安装脚本 (Jetson Orin / JetPack 6.x / Python 3.10)
#
# 背景：elevation_mapping_cupy 的 elevation_mapping_node.py 用系统 python3 运行，
# launch (ros2_ws/src/FAST_LIO/launch/elevation_mapping_cupy_fastlio.launch.py)
# 只是把本 venv 的 site-packages 前置到该节点的 PYTHONPATH，用来提供 torch。
# cupy / numpy / scipy / shapely / cv2 等系统或用户 site 已有，无需在此重复安装。
# 因此本环境只需装 torch（含其纯 python 依赖 filelock/fsspec/networkx/sympy/jinja2）。
#
# 关键：必须是 python 3.10（与系统 python3 ABI 一致），否则注入 PYTHONPATH 会 ABI 不兼容。
#
# 用法: ./install.sh
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

# NVIDIA Jetson wheel 文件名与内部版本不完全匹配，跳过 uv 的文件名校验
export UV_SKIP_WHEEL_FILENAME_CHECK=1

# JetPack 6.x 的 CUDA-enabled PyTorch 轮子（与 vpr 用的同一颗）
TORCH_WHEEL="https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torch-2.4.0a0+07cecf4168.nv24.05.14710581-cp310-cp310-linux_aarch64.whl"

if ! command -v uv &> /dev/null; then
    echo "错误: 未找到 uv，请先安装: curl -LsSf https://astral.sh/uv/install.sh | sh"
    exit 1
fi

echo "=== Step 1: 创建 python3.10 虚拟环境 ==="
uv venv --python 3.10 --seed .venv

echo "=== Step 2: 安装 PyTorch (CUDA enabled, Jetson wheel) ==="
uv pip install --python .venv/bin/python --no-cache-dir "$TORCH_WHEEL"

echo ""
echo "=== 验证（在系统 python3 上按 launch 的注入方式检查）==="
NEW=".venv/lib/python3.10/site-packages"
PYTHONPATH="$(pwd)/$NEW${PYTHONPATH:+:$PYTHONPATH}" /usr/bin/python3 - <<'PY'
import torch, torch.nn  # noqa: F401
import cupy             # noqa: F401  # 来自 ~/.local，验证不被本 venv 挡住
print("torch:", torch.__version__, "cuda:", torch.cuda.is_available())
print("cupy :", cupy.__version__)
print("OK")
PY

echo ""
echo "=== 完成！launch 会自动使用 $HERE/.venv ==="
echo "如需自定义路径: export ELEVATION_VENV_SITE_PACKAGES=/abs/path/to/site-packages"
