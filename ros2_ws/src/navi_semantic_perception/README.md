# navi_semantic_perception

给 nav2 的 `semantic_segmentation_layer` 供数：把 RealSense 的深度重投影到**彩色像素
网格**上产出组织化点云，再用 TensorRT 跑语义分割出掩码，两者**逐像素对齐、stamp 严格
1:1**。当前用途是识别草坪并在局部代价图上抬高代价。

## 为什么需要这个包

那一层的关联方式是**纯索引**（`segmentation_buffer.cpp:203-206`，`pixel_idx = v*width+u`
与 x/y/z 迭代器同步步进），README:88 原话 "Must be pixel-aligned and time-synchronized
with segmentation."。也就是说**点 N 必须等于像素 N**。

而 RealSense D457 的深度和彩色是**两个不同的像素网格**（内参不同 + 59.10 mm 基线）。
直接把驱动的 `depth/points` 和彩色图按同索引关联，实测偏差 **12~65 px**，
横向 **7~16 cm = 1.4~3.2 个局部代价图格子**（0.05 m）：

    du(Z) = (cx_c - cx_d) + fx_c·tx/Z = -7.910 - 22.926/Z   [px]

两项同号相加，**不存在互相抵消的距离**。而且深度与彩色都是 640×480，那一层的尺寸校验
（`semantic_segmentation_layer.cpp:495`）只比 `width*height` 的乘积，**会通过** ->
静默错 12~65 px，运行时不可发现。

驱动侧开 `pointcloud`/`align_depth` 解决不了：`pointcloud_filter.cpp:139,147-152` 的
点云网格永远取自 `depth_intrin`，且 `base_realsense_node.cpp:260` 把点云滤波压在对齐
滤波【之前】。所以必须自己投影 —— 这就是 `warp.py`。

## 组成

| 可执行 | 作用 |
|---|---|
| `depth_to_color_cloud` | 深度 -> 彩色网格组织化点云 `/semantic/camera1/points` |
| `segmentation_trt` | TensorRT 语义分割 -> 掩码 + confidence + LabelInfo |
| `stub_segmentation` | 固定区域打标，没模型时打通全链 |
| `label_info_publisher` | 发 LabelInfo（**仅 stub 路径需要**，trt 自带） |
| `build_trt_engine` | 预编译 TRT 引擎（省掉首启的 9 分钟） |

| 模块 | |
|---|---|
| `warp.py` | 投影数学，**只此一份**。离线验证脚本与运行期节点 import 同一个实现 |
| `trt_engine.py` | 引擎编译 + 全 GPU 推理（预处理/argmax/softmax 都在显存里） |

## 怎么用

### 一次性：预编译引擎

```bash
cd ~/Navi/ros2_ws && source install/setup.bash
ros2 run navi_semantic_perception build_trt_engine     # 约 9 分钟 -> 67 MB
```

产物在 `~/.cache/navi_semantic_perception/`，文件名含 onnx 的 sha256 + 设备 + 算力 +
TRT 版本 + 形状，任一变化自动重编。**不做也能跑**，节点会自己编，但会在启动时卡 9 分钟。

### 日常

```bash
ros2 launch navi_semantic_perception semantic_perception.launch.py
```

`base_link -> camera1_link` 的 TF **不用单独起** —— 室内栈
(`odometry_lslidar.launch.py`) 与室外栈 (`outdoor_localization.launch.py`) 各自内联了
一个 `static_transform_publisher`，数值读 `fast_lio/config/camera_extrinsics.yaml`。

常用参数：

| | 默认 | |
|---|---|---|
| `segmentation` | `trt` | `trt` / `stub` / `none` |
| `target_rate` | `10.0` | 点云与掩码的**共同**速率，必须一致 |
| `input_scale` | `1.0` | 原生 640×480；GPU 有 43 倍余量，别调小 |
| `min_mean_brightness` | `100.0` | 暗场 fail-safe，见下 |
| `extrinsics_source` | `topic` | bag 回放缺 extrinsics 时用 `tf` |

### 两个"省电"行为

`depth_to_color_cloud` 和 `segmentation_trt` 在**没有订阅者时完全不干活**。所以单独起来
`ros2 topic hz` 看不到东西是正常的，得有 nav2 的层或 RViz 在订阅。

### 检查

```bash
ros2 topic hz /semantic/camera1/points /semantic/camera1/segmentation   # 应 ~10 Hz 且相等
ros2 topic echo --once /semantic/camera1/label_info                     # latched，应立刻返回
ros2 run tf2_tools view_frames    # 应是一棵树：odom -> base_link -> camera1_link -> ..._optical_frame
```

这三条日志**一条都不该出现**，出现即静默失效：
`Pointcloud and segmentation sizes are different` / `Class map is empty` /
`Skipping undefined class_id`。

## 模型

`models/model.onnx` 取自 nav2 官方教程 `nav2_semantic_segmentation_demo`
（BSD-3-Clause，见 `models/MODEL_LICENSE.md`）。类别 **0=background 1=sidewalk 2=grass**，
H/W 动态 float16，所以 640×480 可直接喂、掩码原生就在彩色网格上。

后端是 **TensorRT 不是 onnxruntime**：本机是 NVIDIA Thor = **sm_110**，PyPI 的
`onnxruntime-gpu` aarch64 wheel 只烧了 sm_89/90a/120a/121a 且**不含 PTX**，
实测报 `cudaErrorNoKernelImageForDevice`；jetson-ai-lab 的 jp7 索引里没有这个包。
TensorRT 在运行时按实机架构编译，且 `tensorrt`/`cupy` 与 `rclpy` 同一解释器
（系统 python 3.12），**所以这个包不需要 venv**。

实测 640×480：**2.33 ms / 429.7 Hz**（CPU 4 线程是 308.8 ms / 3.2 Hz，快 133 倍）。

## 暗场 fail-safe（`min_mean_brightness`）

这个模型在本园区的**暗湿铺装**上会整片误判成草坪。实测（240 帧）近场条带平均亮度与
"路面被误判成 grass 的比例"相关系数 **r = -0.850**：

| 近场亮度 | 误判中位% | >20% 的帧 |
|---|---|---|
| 60-80 | 54.9 | 100% |
| 80-100 | 21.4 | 59% |
| 100-120 | **0.0** | 3% |

所以近场亮度低于 100 时直接发**全 background 掩码**（保持与点云 1:1 的 stamp，但这一层
什么都不标）。效果：近场 >20% 的帧从 22% 降到 **2%**，最大 89.4% 降到 35.7%，代价是真
草坪面积保留 53%。

**置信度阈值分不开这两者**（坏帧上要 240 才压住，而真草坪 conf 的 p75 才 231），亮度可以。

## 已知限制

- `camera1_z` 仍是待定的 `0.0`。对 2D 代价图影响很小（平移不改 x/y），但要精确需用
  RViz 把 `/semantic/camera1/points` 与雷达 `/cloud_registered` 叠起来标定。
- **camera2 未接**。`rs_multi_camera_launch.py:70-78` 发了一条全零 dummy
  `camera1_link -> camera2_link`；补上 `base_link -> camera1_link` 后 camera2 会从
  "明显断开"变成"静默地等于 camera1"。要用它得先改 robot-sensors。
- `config/ontology.yaml` 里的 `model.device` 是上游遗留字段，本包不读。
- 层本身在 `param_outdoor_elevation.yaml` 里是 `enabled: False`，见那里的注释。

## 相关分析

| | |
|---|---|
| `analysis/rgbd_align/` | 深度↔彩色对齐的量化与离线验证 |
| `analysis/lawn_layer/` | 层的 bag 回放测试台（RViz 配色代价图 + 视频） |
