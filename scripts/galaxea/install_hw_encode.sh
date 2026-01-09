#!/bin/bash
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "请以 sudo 运行: sudo bash $0"
  exit 1
fi

echo "[1/3] 配置 NVIDIA Jetson L4T r36.3 软件源..."
mkdir -p /usr/share/keyrings
curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc | gpg --dearmor --batch --yes -o /usr/share/keyrings/nvidia-jetson-archive-keyring.gpg

cat >/etc/apt/sources.list.d/nvidia-l4t.list <<'EOF'
deb [signed-by=/usr/share/keyrings/nvidia-jetson-archive-keyring.gpg arch=arm64] https://repo.download.nvidia.com/jetson/common r36.3 main
deb [signed-by=/usr/share/keyrings/nvidia-jetson-archive-keyring.gpg arch=arm64] https://repo.download.nvidia.com/jetson/t234 r36.3 main
EOF

apt update

echo "[2/3] 安装 Jetson 的 GStreamer 硬件插件 (nvidia-l4t-gstreamer)..."
DEBIAN_FRONTEND=noninteractive apt install -y nvidia-l4t-gstreamer

echo "[3/3] 验证关键插件是否可用..."
/usr/bin/gst-inspect-1.0 nvv4l2h264enc | head -n 10 || true
/usr/bin/gst-inspect-1.0 nvvidconv | head -n 10 || true
/usr/bin/gst-inspect-1.0 nvarguscamerasrc | head -n 10 || true

echo "完成。现在可以使用硬件编码。"


