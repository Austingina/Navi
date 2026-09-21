#! /bin/bash
# 单独诊断 SAM3 语义感知链：depth_to_color_cloud + segmentation_sam3。
# 正常 outdoor 流程已由 outdoor_bringup_heading_align.launch.py 延迟拉起，无需运行本脚本。
#
# 产出 /semantic/camera1/{points,segmentation,confidence,label_info}，唯一消费者是
# param_outdoor_elevation.yaml 里 local_costmap 的 lawn_layer（草坪抬代价）。该层只在
# 室外那份参数里出现，所以本脚本挂在 session=outdoor 而不是常驻的 session=navi ——
# 室内 SLAM 跑它纯属白烧 GPU（TensorRT FP16 单提示词 640x480 约 150 ms/帧）。
#
# 现在是"影子模式"：lawn_layer 仍是 enabled: False，但 SemanticSegmentationLayer 的
# activate() 无条件 subscribe（semantic_segmentation_layer.cpp:669-691），只有
# updateBounds/updateCosts 会因 !enabled_ 提前返回。所以整条链会真的跑起来、能在
# RViz 看 /semantic/camera1/segmentation_color，但对代价图零影响。验收后打开：
#   ros2 param set /local_costmap/local_costmap lawn_layer.enabled true
# 永久生效则改 param_outdoor_elevation.yaml 的 lawn_layer.enabled。
#
# 依赖 base_link -> camera1_link（outdoor_localization.launch.py 发）和相机驱动发的
# camera1_link -> camera1_color_optical_frame。缺后者那一层会静默丢帧，见
# outdoor_localization.launch.py 里 tf_base_link_to_camera1 的注释。
#
# 用法:
#   ./start_semantic.bash          # 真实环境（RealSense D457）
#   ./start_semantic.bash -s       # 仿真环境（Isaac ZED，相机 profile 见下）

source "$(dirname "$0")/common.sh"

# 让 INFO 级别日志也实时落盘，绕开 spdlog 文件 sink 的 flush_on(error) 缓冲。
# 详见 start_localization.bash 的注释。
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export RCUTILS_COLORIZED_OUTPUT=0
export PYTHONUNBUFFERED=1

USE_SIM_TIME=false
for arg in "$@"; do
    case "$arg" in
        -s|--simulated) USE_SIM_TIME=true ;;
    esac
done

LOG_DIR="$NAVI_ROOT_DIR/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/semantic_$(date +%Y%m%d_%H%M%S).log"
echo "logging to $LOG_FILE"

# sim 同时置上 use_sim_time 和 sim，两者管的是两件事（同 start_localization.bash）：
#   use_sim_time -> 时钟源走 /clock
#   sim          -> 相机 profile：Isaac 的 ZED 话题名/分辨率/深度编码/外参来源都与
#                   真机 RealSense 不同，详见 semantic_perception.launch.py 的
#                   _camera_profile()。
ros2 launch navi_semantic_perception semantic_perception.launch.py \
    use_sim_time:=$USE_SIM_TIME sim:=$USE_SIM_TIME 2>&1 \
    | stdbuf -oL -eL tee -a "$LOG_FILE"
