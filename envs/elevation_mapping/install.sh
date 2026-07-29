#!/bin/bash
# elevation_mapping 专用虚拟环境安装脚本 (Jetson Thor / JetPack 7 / Python 3.12)
#
# 背景：elevation_mapping_cupy 的 elevation_mapping_node.py 用系统 python3 运行，
# launch (ros2_ws/src/FAST_LIO/launch/elevation_mapping_cupy_fastlio.launch.py)
# 只是把本 venv 的 site-packages 前置到该节点的 PYTHONPATH，用来提供 torch。
# cupy / numpy / scipy / shapely / cv2 等系统或用户 site 已有，无需在此重复安装。
# 因此本环境只需装 torch（含其纯 python 依赖 filelock/fsspec/networkx/sympy/jinja2）。
#
# 关键：venv 的 python 小版本必须与系统 python3 一致（Ubuntu 24.04 = 3.12），
# 否则注入 PYTHONPATH 会 ABI 不兼容。这里直接取系统 python3 的版本，不写死。
#
# 用法: ./install.sh
#   可用 TORCH_WHEEL 覆盖要安装的 torch（例如换一颗 JetPack 轮子）。
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

# NVIDIA Jetson wheel 文件名与内部版本不完全匹配，跳过 uv 的文件名校验
export UV_SKIP_WHEEL_FILENAME_CHECK=1

PY_VERSION="$(/usr/bin/python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
ARCH="$(uname -m)"

# aarch64 (Jetson) 上 CUDA 版 torch 只能用 NVIDIA 自建轮子；x86 开发机走 PyPI。
# JetPack 7 / CUDA 13 的轮子索引见 https://pypi.jetson-ai-lab.io/
if [[ -z "${TORCH_WHEEL:-}" ]]; then
    if [[ "$ARCH" == "aarch64" ]]; then
        echo "错误: aarch64 上需要显式指定 JetPack 7 / cp${PY_VERSION/./} 的 CUDA torch 轮子。"
        echo "      例: TORCH_WHEEL=<url-or-path-to-.whl> ./install.sh"
        echo "      索引: https://pypi.jetson-ai-lab.io/jp7/cu130"
        exit 1
    fi
    TORCH_WHEEL="torch"
fi

if ! command -v uv &> /dev/null; then
    echo "错误: 未找到 uv，请先安装: curl -LsSf https://astral.sh/uv/install.sh | sh"
    exit 1
fi

echo "=== Step 1: 创建 python${PY_VERSION} 虚拟环境 (匹配系统 python3) ==="
uv venv --python "$PY_VERSION" --seed .venv

echo "=== Step 2: 安装 PyTorch ($TORCH_WHEEL) ==="
uv pip install --python .venv/bin/python --no-cache-dir "$TORCH_WHEEL"

echo ""
echo "=== 验证（在系统 python3 上按 launch 的注入方式检查）==="
NEW=".venv/lib/python${PY_VERSION}/site-packages"
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
