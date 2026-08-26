#!/bin/bash
# VRR-VPR / NaviSkill 运行环境安装脚本 (Jetson Thor / JetPack 7 / Python 3.12 / CUDA 13)
#
# 说明：本目录的 .venv 同时供 VPR 重定位子进程与 NaviSkill server.py 使用
# （启动方式：vpr/.venv/bin/python3 server.py）。它是 --system-site-packages
# venv：继承系统 ROS2 / apt / ~/.local 里的包（rclpy、ament、rqt、cupy 等），
# 只把 VPR/NaviSkill 自己的 python 依赖装进 venv 本地。
#
# 本脚本按 requirements-thor.txt「精确 pin」复刻当前 .venv 的 venv-local 包集合
# （由 `pip freeze --local` 导出）。torch/torchvision 及 nvidia-*-cu13 等 CUDA 轮子
# 来自 requirements 顶部的 Thor 索引 https://pypi.jetson-ai-lab.io/jp7/cu130 ，
# 其余走 PyPI；lightglue 固定到 git commit。
#
# 与 install_jetson.sh 的区别：后者是 Orin / JetPack 6 / py3.10 / torch 2.4a，已废弃。
#
# 用法:
#   ./install_thor.sh
# 可覆盖的环境变量:
#   REQUIREMENTS  (默认 requirements-thor.txt)  精确 pin 列表
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

REQUIREMENTS="${REQUIREMENTS:-requirements-thor.txt}"

# venv 的 python 小版本必须与系统 python3 一致（Ubuntu 24.04 = 3.12），否则
# --system-site-packages 注入系统扩展模块会 ABI 不兼容。
PY_VERSION="$(/usr/bin/python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
ARCH="$(uname -m)"

if [[ "$ARCH" != "aarch64" ]]; then
    echo "警告: 本脚本面向 Jetson Thor (aarch64)。当前架构 $ARCH，CUDA 轮子可能无法解析。"
fi
if [[ ! -f "$REQUIREMENTS" ]]; then
    echo "错误: 未找到 $REQUIREMENTS"
    exit 1
fi

echo "=== Step 1: 创建 python${PY_VERSION} 虚拟环境 (--system-site-packages, 匹配系统 python3) ==="
# 与当前 .venv 的建法一致（见 pyvenv.cfg: python3 -m venv --system-site-packages）。
/usr/bin/python3 -m venv --system-site-packages .venv

echo ""
echo "=== Step 2: 升级 pip 工具链 ==="
.venv/bin/python -m pip install --no-cache-dir --upgrade pip setuptools wheel

echo ""
echo "=== Step 3: 按精确 pin 安装依赖 ($REQUIREMENTS) ==="
# requirements-thor.txt 顶部已带 --extra-index-url 指向 Thor cu130 索引，
# torch/torchvision/nvidia-*-cu13/cupy 等从该索引解析，其余走 PyPI。
.venv/bin/python -m pip install --no-cache-dir -r "$REQUIREMENTS"

echo ""
echo "=== 验证安装 ==="
.venv/bin/python - <<'PY'
import torch, torchvision, faiss, lightglue  # noqa: F401
print("torch      :", torch.__version__)
print("torchvision:", torchvision.__version__)
print("cuda_built :", torch.version.cuda)
print("cuda_avail :", torch.cuda.is_available())
print("faiss/lightglue import: OK")
# 验证 --system-site-packages 生效：能看到系统 ROS 的 rclpy
try:
    import rclpy  # noqa: F401
    print("rclpy (system-site): OK")
except Exception as e:
    print("rclpy 不可见(检查 --system-site-packages / source ROS):", e)
print("OK")
PY

echo ""
echo "=== 完成！NaviSkill/VPR 会使用 $HERE/.venv/bin/python3 ==="
