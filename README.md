# 编译与运行说明

## 1. 下载代码

```bash
cd ~/
git clone -b navi https://gitlab.autel.com/avant/AI-Embodied/Mobility/fast_lio2.git Navi
```

## 2. 编译安装 3D BBS

```bash
cd ~/Navi/ros2_ws/src/3rdparty/3d_bbs
mkdir build
cd build
cmake ..
make -j
sudo make install
```
## 3 编译安装Livox SDK
```bash
cd ~/Navi/ros2_ws/src/3rdparty/Livox-SDK2
mkdir build && cd build && cmake ..
make
sudo make install
```

## 4. 编译 ROS2 包

包括 fastlio、cloud_to_laserscan、slope_behavior、livox_ros_driver2、grid_map_pcl(重写了grid_map_pcl) 等。

`elevation_mapping_cupy` 的节点 `import ros2_numpy`，而这个包任何 ROS 发行版都没有
apt 包，只能从源码拉。首次编译前执行一次：

```bash
cd ~/Navi/ros2_ws
vcs import src < src/elevation_mapping_cupy/docker/src.repos
```

再补齐 rosdep 能解析的依赖（首次编译或改过 package.xml 后执行）。`rosdep` 首次
使用需要初始化：

```bash
sudo rosdep init          # 只需一次
rosdep update

cd ~/Navi/ros2_ws
# cupy-cuda13x 与 simple-parsing 是 elevation_mapping_cupy 自定义的 rosdep key
# （定义在它的 docker/extra_rosdeps.yaml 里，未注册到系统），由 pip / venv 提供，
# 必须另外手工装 —— 见下面 4.1。
rosdep install --from-paths src --ignore-src -y \
  --skip-keys "cupy-cuda13x simple-parsing"
```

> 若这台机器上装过 conda，`rosdep` 可能把系统识别成 `conda` 而报
> `Unsupported OS [conda]`。此时加 `ROS_OS_OVERRIDE=ubuntu:noble:noble`。

### 4.1 Python 运行环境（rosdep 装不到，必须手工装）

**这一节漏掉任何一项，节点都是能编过、一起跑就崩**，而且报错只是一句
`ModuleNotFoundError`，不看这里很难联想到是环境没装齐。

| 装什么 | 谁需要 | 怎么被加载 |
|---|---|---|
| `NaviSkill/vpr/.venv` | NaviSkill `server.py`、VPR 重定位子进程 | 直接以 `vpr/.venv/bin/python3` 启动 |
| `envs/sam3/.venv` | `segmentation_sam3` 语义分割节点 | launch 用 `prefix=` 指定，可用 `sam3_python` 覆盖 |
| `envs/elevation_mapping/.venv` | `elevation_mapping_node.py` 的 torch 后端（`core_param.yaml` 里 `use_chainer: false`） | 节点跑在系统 `python3` 上，launch 只把它的 `site-packages` 前置到 `PYTHONPATH` |
| `simple_parsing` | `elevation_mapping_cupy/parameter.py` | 用户 site；自定义 rosdep key 被 `--skip-keys` 跳过，**apt 无此包** |
| `cupy`（cuda13x） | `elevation_mapping_cupy/elevation_map.py` | 用户 site；同上，且必须匹配 CUDA 13 / aarch64 |

前两个 venv 带 `--system-site-packages`，继承系统 ROS 的 `rclpy` / `ament`；
后两项由 `/usr/bin/python3` 加载（节点 shebang 是 `#!/usr/bin/env python3`）。
缺失时在节点 `__init__` 之前的 import 阶段就挂，进程直接退出：

```
[elevation_mapping_node.py-3] ModuleNotFoundError: No module named 'simple_parsing'
[ERROR] [elevation_mapping_node.py-3]: process has died [pid ..., exit code 1, ...]
```

装法：

```bash
cd ~/Navi

# 1. 用户 site 的两个模块。Ubuntu 24.04 有 PEP 668 标记，不加
#    --break-system-packages 会被直接拒绝；--user 只写 ~/.local，不需要 sudo。
python3 -m pip install --user --break-system-packages \
  simple-parsing "cupy-cuda13x==13.6.0"

# 2. elevation_mapping 的 torch。脚本依赖 uv，且在 aarch64 上拒绝猜轮子。
command -v uv || curl -LsSf https://astral.sh/uv/install.sh | sh
TORCH_WHEEL="torch==2.13.0" bash envs/elevation_mapping/install.sh

# 3. NaviSkill / VPR
bash NaviSkill/vpr/install_thor.sh

# 4. SAM3。checkpoint 获取与离线导入见 envs/sam3/README.md
bash envs/sam3/install.sh
envs/sam3/.venv/bin/python -m pip install -r envs/sam3/requirements-export.txt
bash envs/sam3/build_trt.sh   # 约 10 分钟；引擎必须本机构建，不能跨机器拷贝
```

三个脚本结尾都会自检并打印 `torch.cuda.is_available()`，没看到就别往下走。

> **轮子来源**（2026-09-18 实测）：`envs/sam3/install.sh` 用 PyTorch 官方
> `download.pytorch.org/whl/cu130`；elevation_mapping 走 PyPI —— aarch64 的
> `torch==2.13.0` 轮子本身就是 CUDA 13 构建（`torch.version.cuda == 13.0`，
> `get_arch_list()` 含 Thor 的 `sm_110`），**不需要**特殊索引。但
> `NaviSkill/vpr/requirements-thor.txt` 顶部的
> `--extra-index-url https://pypi.jetson-ai-lab.io/jp7/cu130` **已 404**，
> 重装 vpr 时若因该行解析失败，删掉它走 PyPI。

> **三个环境不要合并**，三种 numpy 状态是 ABI 逼出来的：`elevation_mapping` 注入的
> `site-packages` 一旦带上 `opencv 5.x`，系统 `cv_bridge`（针对 `cv2 4.6.0` 编译）会
> `cv2_to_imgmsg` 报 `KeyError: 16`；带上 numpy 2 则踩到 apt `scipy 1.11.4` 里已被
> numpy 2.0 删除的 `numpy.Inf`。三份 torch 重复约 8 GB，嫌占地方就用
> `hardlink --content --minimum-size 1M` 对三个 `site-packages` 去重，不影响 import。

### 4.2 `campus_nav` 需要 pyosmium（3.x，别装 4.x）

`campus_nav/route_graph.py` 读 `.osm.pbf` 时 local import `osmium`（`build_graph_from_pbf`
/ `build_graph_merged`）。节点默认 `source=pbf`，所以这是**运行时硬依赖** —— 缺了
`route_planner_node` 会在 `__init__` 里直接 `ModuleNotFoundError: No module named 'osmium'`
退出，室外栈就没有全局路网规划。

已作为 `<exec_depend>python3-pyosmium</exec_depend>` 写进 `campus_nav/package.xml`，
所以**上面第 4 节那条 `rosdep install` 会自动装它**，一般无需手工干预。单独装：

```bash
sudo apt install python3-pyosmium
```

> **必须是 pyosmium 3.x。** 代码用的 `osmium.SimpleHandler` + `apply_file(..., locations=True)`
> 是 3.x 的 API，4.x 已经改掉。apt 的 `python3-pyosmium` 是 3.7.0，正确；
> **不要 `pip install pyosmium`** 装最新版，会因 API 变更而挂。

### 在本机编译
```bash
cd ~/Navi/ros2_ws
colcon build --cmake-args -DROS_EDITION=ROS2 -DCMAKE_MESSAGE_LOG_LEVEL=VERBOSE --event-handlers console_direct+ --symlink-install
```

### 在编译机编译
```bash
cd ~/Navi/ros2_ws
colcon build --cmake-args -DROS_EDITION=ROS2 -DCMAKE_MESSAGE_LOG_LEVEL=VERBOSE --event-handlers console_direct+ 
```

> `-DHUMBLE_ROS=humble` 已不需要：`livox_ros_driver2` 现在按 `$ROS_DISTRO` 自己选
> rosidl typesupport 接口。

> 若这台机器上装过 conda（尤其是 RoboStack 的 ROS 环境），编译前务必
> `conda deactivate`，并确认 `RMW_IMPLEMENTATION` 指向已安装的中间件，否则
> CMake 会优先找到 conda 里的 ROS 而不是 `/opt/ros/jazzy`。

## (Optional) Building Docker Image
参考：
https://q00enigbkuh.feishu.cn/wiki/BnDVw0KoUiMOW9kuYVXcJkCAnzg?from=from_copylink

## 6. 运行脚本

### 启动和结束导航

```bash
cd ~/Navi/scripts/
./start_navi.sh
./stop_navi.sh
```

### 查看可视化结果

```bash
./start_rviz.sh
```
### 启动NaviSkill

```bash
./start_mcp.sh
```
## 7. 编译运行环境
### 1. 硬件与系统环境
**硬件要求：**
- **Jetson Thor**（aarch64，Blackwell GPU，compute capability `sm_110`）
- 也支持 x86_64 开发机

**系统要求：**
- **Ubuntu 24.04**

**CUDA 环境要求：**
- **CUDA 版本**：`13.x`（Thor 上随 JetPack 7 提供）

### 2. ROS 2 Jazzy 环境
```bash
sudo apt install ros-jazzy-desktop-full
```

### 3. 特定 ros2 包环境

nav2 与 grid_map 都**不要再按版本号 pin**。原先 pin 到 `1.1.20` / `2.0.1` 是因为
仓库里带着 Humble 版的 nav2 源码副本；这些副本已经删除，现在直接用 Jazzy 的
发行版本（nav2 1.3.12、grid_map 2.2.2）。

- **nav2**
  ```bash
  sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-dwb-controller
  ```
- **grid_map**：只装本仓库真正依赖的子包。**不要装 `ros-jazzy-grid-map` 元包**，
  因为它会带进 `grid-map-pcl`，而这个在本仓库里是带本地扩展的源码版（坡度层、
  可通行性层等），装了会造成同名包重复。
  ```bash
  sudo apt install ros-jazzy-grid-map-cmake-helpers ros-jazzy-grid-map-core \
                   ros-jazzy-grid-map-msgs ros-jazzy-grid-map-ros \
                   ros-jazzy-grid-map-cv ros-jazzy-grid-map-filters \
                   ros-jazzy-grid-map-loader ros-jazzy-grid-map-octomap \
                   ros-jazzy-grid-map-costmap-2d ros-jazzy-grid-map-sdf \
                   ros-jazzy-grid-map-rviz-plugin ros-jazzy-grid-map-visualization
  ```
- **CGAL**：`convex_plane_decomposition` 需要。
  ```bash
  sudo apt install libcgal-dev libgmp-dev libmpfr-dev
  ```
- **pyosmium**：`campus_nav` 读 `.osm.pbf` 需要，必须 3.x（见 4.2）。已声明在
  package.xml 里，rosdep 会自动装。
- **其余 ROS 依赖**统一交给 rosdep（见第 4 节）。
  但注意 **rosdep 盖不住 `simple-parsing` / `cupy-cuda13x` / `torch`** ——
  前两者虽然写在 `elevation_mapping_cupy` 的 package.xml 里，却没有对应的 rosdep 规则
  （`rosdep resolve simple-parsing` 直接报 `no rosdep rule`），所以被 `--skip-keys` 跳过，
  **必须手工装**，否则 `elevation_mapping_node` 一起跑就崩。
- **三个 Python 虚拟环境**（VPR/NaviSkill、SAM3、elevation_mapping 的 torch）
  同样在 rosdep 之外，缺了对应节点直接起不来。两者装法都见 **4.1**。

### 4. 星海图系统环境（编译不需要，运行需要）
在 ~/galaxea/install目录是星海图的系统环境，

烧录教程参考： https://docs.galaxea-dynamics.com/zh/Guide/ros_upgrade_guide_all/ros2_upgrade_guide_pro/#8