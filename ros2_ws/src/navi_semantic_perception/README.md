# navi_semantic_perception

给 nav2 的 `semantic_segmentation_layer` 供数：把 RealSense 的深度重投影到**彩色像素
网格**上产出组织化点云，再用 SAM3 跑语义分割出掩码，两者**逐像素对齐、stamp 严格
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

## SAM3 迁移

SAM3 源码和工具保存在 `sam3/`，运行环境安装到仓库内的
`envs/sam3/.venv`。模型配置、tokenizer 和预处理参数随本 ROS 包提交并安装；
checkpoint 位于 `<Navi>/checkpoints/sam3/model.safetensors`，生成的 TensorRT 引擎位于
`<Navi>/checkpoints/sam3/trt_engines/`，由根目录 `.gitignore` 排除，不提交 Git。

换机器 clone 后按 [SAM3 部署步骤](../../../envs/sam3/README.md)执行
`bash envs/sam3/install.sh`，装上 `envs/sam3/requirements-export.txt` 的导出依赖，
再执行 `bash envs/sam3/build_trt.sh` 构建本机引擎（约 10 分钟）。
权重可从 Meta 官方 `facebook/sam3` 下载（需访问权限），
也可从旧机器导入；TensorRT 引擎必须在目标机器上重新构建，不能拷贝。
运行不依赖仓库外的 SAM3 源码或虚拟环境。

- 源码：`sam3/`（Meta 原始包、工具、示例和 LICENSE）。
- 默认导航推理后端：SAM3 TensorRT FP16，视觉编码器和检测器分为两个引擎。
- 可选 `segmentation:=sam3` 使用 Transformers `Sam3Model` / PyTorch bfloat16。
- 配置：已安装 ROS share 目录下的 `models/sam3/config.json`。
- 预处理/分词器：同目录的 `models/sam3/processor/`。
- Python：`<Navi>/envs/sam3/.venv/bin/python`，与 ROS Jazzy 同为 Python 3.12。
- 运行期仅本地读取文件，不联网、不编译 TensorRT。

`config/ontology.yaml` 仍然定义 **0=background、1=sidewalk、2=grass**。
默认仅对 `grass` 提示词推理；`sidewalk` 保留 ID，但 `sam3_prompt: ""` 表示不查询。
需要更多类别时可修改 `sam3_prompt`，图像编码器会在同一帧的多个提示词之间复用。
重叠实例由最高分决定类别；未检测到的像素输出 background/0 confidence。

置信度是 SAM3 的 **检测分数 × presence 分数**，量化为 0..255，在实例掩码内填充。
它不再是旧模型的逐像素 softmax；`lawn_layer.camera1.lawn.mark_confidence: 128`
对应约 0.5 分数。SAM3 的实例筛选阈值和 mask 阈值均默认 0.5。
旧模型的 `min_mean_brightness=100` 暗场过滤只保留在旧模型 `segmentation:=trt` 后端，不作用于 SAM3。

## 构建与启动

```bash
cd ~/Navi/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select navi_semantic_perception
source install/setup.bash
ros2 launch navi_semantic_perception semantic_perception.launch.py
```

launch 自动为 SAM3 节点选择现有虚拟环境解释器，其他 ROS 节点继续使用系统 Python。
`extrinsics_source:=topic` 需要先 source 包含 `realsense2_camera_msgs` 的传感器工作区。
若 bag 只有相机静态 TF、没有工厂 extrinsics 消息：

```bash
ros2 launch navi_semantic_perception semantic_perception.launch.py \
  use_sim_time:=true extrinsics_source:=tf
```

`base_link -> camera1_link` 由已有室内/室外定位 launch 提供，安装外参见
`fast_lio/config/camera_extrinsics.yaml`。输入分辨率为 640×480。

**室外 `lawn_layer` 的既有开关仍为 `enabled: False`，室外总启动脚本也没有自动包含本 launch。**
先单独启动上面的感知链，检查 RViz 叠加图和点云，再启用 costmap 层：

```bash
ros2 param set /local_costmap/local_costmap lawn_layer.enabled true
```

永久启用时修改 `fast_lio/config/param_outdoor_elevation.yaml` 的 `lawn_layer.enabled`，
并将感知 launch 纳入自己的启动流程。模型迁移本身不会启动底盘或导航。
LabelInfo 使用 RELIABLE + TRANSIENT_LOCAL，晚启动的 Nav2 也可收到类别表。

| launch 参数 | 默认 | 说明 |
|---|---|---|
| `segmentation` | `sam3_trt` | `sam3_trt` / `sam3`（PyTorch）/ `trt`（旧模型）/ `none` |
| `sam3_engine_dir` | `<Navi>/checkpoints/sam3/trt_engines` | 本机构建的 SAM3 引擎目录 |
| `sam3_model_path` | `<Navi>/checkpoints/sam3` | checkpoint 目录；也支持环境变量 `SAM3_CHECKPOINT_DIR` |
| `sam3_python` | 空 | 自动定位 `<Navi>/envs/sam3/.venv/bin/python` |
| `sam3_device` | `cuda` | TensorRT 仅支持 CUDA；PyTorch 也支持 `cpu` |
| `target_rate` | `3.0` | 输入点云限速；忙时只保留最新待处理帧 |
| `camera_ns` | `/camera1/camera1` | 深度和彩色共同的相机命名空间 |
| `use_sim_time` | `false` | bag 回放设为 true，并发布 /clock |
| `extrinsics_source` | `topic` | `topic` / `tf` |

`precision` 仅影响 PyTorch；TensorRT 精度由构建时决定。其他 SAM3 参数放在 `config/sam3.yaml`：`precision`、`score_threshold`、
`mask_threshold`、`max_pair_dt`、`max_result_age`、`model_path`、`config_path` 和 `processor_path`。
CPU 自动使用 float32，但速度远低于实时；离线 CPU 调试需提高 `max_result_age`
（例如 60 秒），否则默认 2 秒会丢弃结果。实时导航使用 CUDA。

## 慢推理与帧同步

```text
color/image_raw ─────────────────────────────┐
depth -> depth_to_color_cloud -> points_raw ──┤ SAM3 单工作线程
                                            └─ segmentation + confidence + points
                                               三者均带原点云的采集 stamp/frame_id
```

输入彩色与深度时间差最大 50 ms。SAM3 后处理将实例 mask 恢复到原始彩色尺寸，
节点同时校验 width 和 height，避免像素总数相同却转置的错配。
单个推理工作线程不会阻塞 ROS 接收；推理期间更新最新待处理点云，不累积推理任务。
只在推理成功后同时发布对应点云、类别和置信度，避免 Nav2 的同步队列先淘汰点云。
推理失败或结果过期时整组丢弃，不将旧结果改盖新时间戳。
默认 `max_result_age=2.0` 秒，同时检查接收后的墙钟耗时和采集时间相对 ROS 时钟的年龄。
该策略保证帧关联；SAM3 本身约 0.3 秒的观测延迟仍需在移动机器人上评估。

| 话题 | 作用 |
|---|---|
| `/semantic/camera1/points_raw` | SAM3 内部输入；Nav2 不订阅 |
| `/semantic/camera1/points` | 推理完成后发布的原始组织化点云 |
| `/semantic/camera1/segmentation` | mono8 类别 ID，供 lawn_layer 消费 |
| `/semantic/camera1/confidence` | mono8 0..255 实例分数 |
| `/semantic/camera1/label_info` | latched 类别表 |
| `/semantic/camera1/segmentation_color` | rgb8 叠加图，RELIABLE，供 RViz 查看 |

没有掩码/置信度/输出点云/可视化订阅者时不执行推理。
检查输出速率时对话题分别执行 `ros2 topic hz`；点云、掩码与置信度的输出帧数应一致。
`segmentation` 的值只有 0/2，在 RViz Image 中看起来近乎全黑，应查看 `_color` 话题。

## 验证与旧模型回退

```bash
cd ~/Navi/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select navi_semantic_perception
colcon test-result --verbose
```

测试覆盖类别 ID、实例重叠和 confidence、空检测、RGB/BGR 行填充、彩色晚到、
慢推理时跳过中间帧、结果过期、模型异常，以及点云/掩码逐维和时间戳一致性。
测试使用受控推理器，不需要模型权重/GPU。

本机迁移验证使用已有 `analysis/rgbd_align/frames.npz` 的真实 640×480 图像。
PyTorch 后端首帧约 844 ms，随后 294–308 ms/帧；默认的 TensorRT FP16 后端首帧约
230 ms，随后 148–153 ms/帧，掩码像素数与 PyTorch 的差异在 0.1% 以内。
两者输出均为 uint8 类别/置信度图。
这仅验证执行链和数据契约，不代表园区草地精度已经完成验收。

需要回退时：

```bash
ros2 launch navi_semantic_perception semantic_perception.launch.py \
  segmentation:=trt target_rate:=10.0
```

旧 TensorRT 模型和 `build_trt_engine` 仍可用，类别 ID 不变；其亮度过滤仍默认 100。
引擎缓存位于 `~/.cache/navi_semantic_perception/`，首次缺失时会编译。
`segmentation:=none` 则只发布点云，没有类别表/掩码，lawn_layer 无法工作。

## 已知限制

- 相机安装高度 `camera1_z` 仍为待标定的 0.0，精确边界需结合雷达点云标定。
- 当前只接 camera1；camera2 的安装外参尚未标定。
- SAM3 的草地提示词、分数阈值和观测延迟需结合实际路线验收。
