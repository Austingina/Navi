#!/bin/bash
# Jetson Orin 安装脚本 (JetPack 6.x + Python 3.10)
# 用法: ./install_jetson.sh

set -e

echo "=== Jetson Orin VRR-VPR 安装脚本 (uv 虚拟环境) ==="

# 检查 uv 是否安装
if ! command -v uv &> /dev/null; then
    echo "错误: 未找到 uv，请先安装: curl -LsSf https://astral.sh/uv/install.sh | sh"
    exit 1
fi

# NVIDIA Jetson wheel 文件名与内部版本不完全匹配，需要跳过检查
export UV_SKIP_WHEEL_FILENAME_CHECK=1

# 设置 wheel URL (JetPack 6.x)
# 如果使用 JetPack 5.x，请修改为 jp/v51
TORCH_WHEEL="https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torch-2.4.0a0+07cecf4168.nv24.05.14710581-cp310-cp310-linux_aarch64.whl"

echo ""
echo "=== Step 1: 创建 uv 虚拟环境 ==="
uv venv --python 3.10 --seed  # --seed 会安装 pip 和 setuptools

echo ""
echo "=== Step 2: 安装项目其他依赖 ==="
# 跳过 torch 和 torchvision，使用前面手动安装的版本
uv sync --no-install-package torch --no-install-package torchvision

echo ""
echo "=== Step 3: 安装 PyTorch (CUDA enabled) ==="
uv pip install --no-cache-dir "$TORCH_WHEEL"

echo ""
echo "=== Step 4: 安装 torchvision 编译依赖 ==="
# setuptools>=82 移除了 pkg_resources，torchvision 编译需要它
uv pip install 'setuptools<81' wheel numpy pillow packaging ninja

echo ""
echo "=== Step 5: 安装 torchvision (从源码编译，需要较长时间) ==="
# 使用虚拟环境中的 pip 编译安装
# v0.18.0 对应 torch 2.3，与 Jetson torch 2.4a API 更兼容
.venv/bin/pip install --no-cache-dir --no-build-isolation 'git+https://github.com/pytorch/vision.git@v0.18.0'



# echo ""
# echo "=== 验证安装 ==="
# uv run python -c "import torch; print(f'PyTorch: {torch.__version__}, CUDA: {torch.cuda.is_available()}')"
# uv run python -c "import torchvision; print(f'torchvision: {torchvision.__version__}')"

echo ""
echo "=== 安装完成! ==="
echo "使用 'uv run python <script.py>' 运行脚本"
