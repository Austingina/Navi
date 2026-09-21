# SAM3 仓库布局与部署

SAM3 的源码、工具、配置、tokenizer 和安装脚本都在 Navi 中。运行不依赖用户家目录下
另一份 SAM3 checkout。checkpoint 权重和本机生成的 ONNX/TensorRT 引擎放在仓库内、
但由 `.gitignore` 排除的 `checkpoints/sam3/` 中。

```text
Navi/
├── sam3/                    # 原始 SAM3 源码、工具、示例、LICENSE
├── envs/sam3/
│   ├── install.sh / requirements.txt    # 可复现的环境安装入口
│   ├── prepare_assets.py               # 权重下载/导入/校验
│   └── .venv/                          # 本机环境，安装生成，不提交 Git
├── checkpoints/sam3/                   # 权重、ONNX、TensorRT 引擎，不提交 Git
│   └── model.safetensors               # 唯一需要单独获取的模型权重
└── ros2_ws/src/navi_semantic_perception/
    ├── navi_semantic_perception/        # SAM3 推理适配器和 ROS 节点
    └── models/sam3/                     # Git 跟踪，colcon 安装
        ├── config.json / LICENSE
        └── processor/                  # tokenizer + 预处理配置
```

默认导航后端使用从 Transformers `Sam3Model` 导出的 TensorRT 引擎，原始 Meta 实现保存在
`sam3` 供开发和工具使用。Python 第三方库安装到仓库内的 `.venv`；
不把平台相关的二进制依赖提交 Git。

## 新机器安装

前提：Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12，目标机器已安装相应 NVIDIA 驱动、
`python3-venv`，以及与驱动匹配的系统 TensorRT —— `.venv` 以 `--system-site-packages`
创建，直接复用系统的 `tensorrt` Python 包，不单独安装。磁盘需为
`<Navi>/checkpoints/sam3` 预留约 8 GB（权重 3.3 GB + ONNX 中间产物 3.2 GB + 引擎 1.6 GB），
`envs/sam3/.venv` 另需约 5 GB。

从零到可用共五步，前三步在 Navi 根目录执行，后两步的细节见下面各节：

1. `bash envs/sam3/install.sh` —— 建环境并获取权重（本节）
2. `envs/sam3/.venv/bin/python -m pip install -r envs/sam3/requirements-export.txt`
3. `bash envs/sam3/build_trt.sh` —— 导出 ONNX 并构建本机引擎，约 10 分钟
4. 在 `ros2_ws` 下 `colcon build --symlink-install --packages-select navi_semantic_perception`
5. 跑一次真实推理确认引擎可用，见下面「构建与检查」

第一步在 Navi 根目录执行：

```bash
bash envs/sam3/install.sh --env-only
# 先在 https://huggingface.co/facebook/sam3 获得模型访问权限，再登录：
envs/sam3/.venv/bin/hf auth login
envs/sam3/.venv/bin/python envs/sam3/prepare_assets.py
```

已登录且具有权限时，直接执行 `bash envs/sam3/install.sh` 即可。
权重来自 [Meta 官方 facebook/sam3](https://huggingface.co/facebook/sam3)，固定 revision
`3c879f39826c281e95690f02c7821c4de09afae7`；仅下载约 3.44 GB 的 `model.safetensors`。
分词器和预处理配置已随 clone 获取，下载器不会覆盖仓库中的配置。

安装器使用系统 Python 创建 `envs/sam3/.venv`，优先复用可用的 CUDA torch/torchvision，
否则从官方 CUDA 13.0 索引安装已验证的 torch 2.13.0 / torchvision 0.28.0
（适用于本项目的 Thor 环境）。版本可通过 `SAM3_TORCH_VERSION` 和
`SAM3_TORCHVISION_VERSION` 覆盖。其他 GPU/驱动需选择
匹配的索引或先安装厂商 wheel，例如：

```bash
SAM3_TORCH_INDEX_URL=https://download.pytorch.org/whl/cu128 \
  bash envs/sam3/install.sh --env-only
```

CUDA 12.8 只是覆盖示例，以目标机器的 [PyTorch 安装说明](https://pytorch.org/get-started/locally/)
和 [Jetson 安装说明](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)
为准。不要从另一台机器直接复制 `.venv`。

## 导入现有 checkpoint

无需下载，也无需复制配置或源码。接收旧机器的 `model.safetensors` 后执行：

```bash
bash envs/sam3/install.sh --from-local /data/model.safetensors
# 若环境已经准备好，只导入权重：
envs/sam3/.venv/bin/python envs/sam3/prepare_assets.py --from-local /data/model.safetensors
```

`--from-local` 也接受含有 `model.safetensors` 的目录，以及旧部署中包含
`checkpoints/sam3/model.safetensors` 的根目录。模型导入过程不联网，环境安装仍可能
下载 Python 包。该后端使用 HF 权重，不能以原始 Meta `sam3.pt` 替代。

自定义 checkpoint 位置：

```bash
SAM3_CHECKPOINT_DIR=/data/models/sam3 bash envs/sam3/install.sh
# 启动时使用同一个环境变量，或显式指定：
ros2 launch navi_semantic_perception semantic_perception.launch.py \
  segmentation:=sam3 sam3_model_path:=/data/models/sam3
```

## TensorRT 构建与启动

目标机需提供与驱动匹配的 TensorRT Python 包；本机使用系统 TensorRT 10.13.3.9。
安装 checkpoint 后，在 Navi 根目录执行：

```bash
envs/sam3/.venv/bin/python -m pip install -r envs/sam3/requirements-export.txt
bash envs/sam3/build_trt.sh
```

脚本从仓库配置和本地 checkpoint 导出两个 ONNX 图，再在当前 GPU 上构建 FP16 引擎。
默认输出为 `<Navi>/checkpoints/sam3/onnx/` 和 `<Navi>/checkpoints/sam3/trt_engines/`。
可用 `SAM3_CHECKPOINT_DIR`、`SAM3_ONNX_DIR`、`SAM3_ENGINE_DIR` 覆盖目录。
已有 ONNX 时可用 `--skip-export`。检测器按单提示词构建，多个提示词顺序执行并复用视觉特征。
换 GPU 或 TensorRT 版本后重新构建；不要将旧机器的引擎直接作为新机器部署产物。

本机实测：ONNX 导出在 CPU 上进行，两个引擎分别约 246 s 和 203 s 构建完成，整体约
10 分钟，产出 `vision_encoder.plan` 873 MB 和 `detector_body.plan` 725 MB。
FP16 构建期间打印的 `out-of-range weights ... clipping to +/- 65504` 属正常现象。
`onnx/` 只是中间产物，引擎构建完成后可删除以回收 3.2 GB；
之后若要重建引擎而不重新导出，需保留该目录才能用 `--skip-export`。

下面的 ROS launch 默认使用 `segmentation:=sam3_trt`，启动时只加载本地引擎，
不会下载模型或编译。缺失或不兼容时明确报错。自定义引擎路径可传
`sam3_engine_dir:=/data/sam3/trt_engines`。临时切回 PyTorch 用 `segmentation:=sam3`；
`segmentation:=trt` 是迁移前的旧分割模型。

## 构建与检查

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select navi_semantic_perception
source install/setup.bash
ros2 launch navi_semantic_perception semantic_perception.launch.py
```

launch 会从当前仓库自动定位 `envs/sam3/.venv/bin/python`，不依赖工作目录。
独立部署 ROS install 目录时，可通过 `NAVI_ROOT` 指向完整 Navi 仓库，或用
`sam3_python:=/absolute/path/to/python` 覆盖解释器。
模型配置和 tokenizer 从已安装的 ROS package share 目录读取。

在仓库根目录检查：

```bash
envs/sam3/.venv/bin/python envs/sam3/prepare_assets.py --check-only
python3 -m pytest envs/sam3/tests -q
```

`--check-only` 不联网、不加载 GPU，检查 checkpoint 文件结构及随仓库提供的配置。

上面两条命令只能确认文件齐备，引擎本身可用必须跑一次推理才算验证。启动上面的
launch，日志出现类似这一行即表示引擎已加载、后端确实是 TensorRT：

```text
[segmentation_sam3]: Loading local SAM3 (tensorrt): <SAM3_ENGINE_DIR>; prompts=[(2, 'grass')]
```

引擎缺失或与本机 GPU/TensorRT 不匹配时启动即报错，不会静默回退到 PyTorch。
本机（Thor / TensorRT 10.13.3.9 / torch 2.13.0+cu130）单提示词 640×480 首帧约 230 ms，
随后 148–153 ms，掩码像素数与 PyTorch 后端的差异在 0.1% 以内。
ROS 包自身的单元测试使用受控推理器，不需要权重或 GPU：

```bash
cd ros2_ws && colcon test --packages-select navi_semantic_perception
colcon test-result --test-result-base build/navi_semantic_perception --verbose
```

完整 ROS 数据链、GPU 推理及 lawn_layer 启用方式见
[感知包说明](../../ros2_ws/src/navi_semantic_perception/README.md)。
