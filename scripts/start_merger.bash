#! /bin/bash
# 启动前后双雷达点云合并节点(livox_multi_merger)。
#
# 为什么必须常驻:mid360.yaml 的 common.lid_topic 指向 /livox/fused_lidar,由本节点产出。
# 一旦它停了,FAST-LIO 就【没有任何点云输入】,定位直接中断。所以它跟雷达一样属于
# 常驻数据管道,放在 session=navi 里(与 slam/outdoor 的切换无关,两条链路都要用)。
#
# 回退单雷达时(mid360.yaml 的 lid_topic 改回 /livox/front_lidar)本节点可以不跑,
# 留着也无害 —— 它只会因为没有订阅者而每 30s 提醒一次。
#
# 外参与合并参数见:
#   ros2_ws/src/livox_multi_merger/config/lidar_extrinsics.yaml
# 设计约束与踩过的坑见:
#   ros2_ws/src/livox_multi_merger/README.md

source "$(dirname "$0")/common.sh"
ros2 launch livox_multi_merger merger.launch.py
