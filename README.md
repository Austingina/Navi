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

### 4.1 rosdep 装不到的运行时依赖（必须手工装）

**这一节漏掉任何一项，节点都是能编过、一起跑就崩**，而且报错只是一句
`ModuleNotFoundError`，不看这里很难联想到是环境没装齐。全部由 `/usr/bin/python3`
加载（节点 shebang 是 `#!/usr/bin/env python3`）。

| 模块 | 谁需要 | 为什么 rosdep 装不到 |
|---|---|---|
| `simple_parsing` | `elevation_mapping_cupy`（`parameter.py`） | 自定义 rosdep key，被 `--skip-keys` 跳过；**apt 无此包** |
| `cupy`（cuda13x） | `elevation_mapping_cupy`（`elevation_map.py`） | 同上；且必须匹配 CUDA 13 / aarch64 |
| `torch` | `elevation_mapping_cupy` 的可通行性滤波（`core_param.yaml` 是 `use_chainer: false`，走 torch 后端） | 由 `envs/elevation_mapping/install.sh` 建的 venv 提供，launch 把它前置到 PYTHONPATH |

缺失时的典型报错（三者都在节点 `__init__` 之前的 import 阶段就挂，进程直接退出）：

```
[elevation_mapping_node.py-3] ModuleNotFoundError: No module named 'simple_parsing'
[ERROR] [elevation_mapping_node.py-3]: process has died [pid ..., exit code 1, ...]
```

装法：`simple_parsing` 与 `cupy` 装到**用户 site**（`~/.local/lib/python3.12/site-packages`，
系统 python3 默认就在 `sys.path` 里，不需要 sudo、也不污染系统包）；`torch` 走
`envs/elevation_mapping/install.sh`（aarch64 上必须显式给 `TORCH_WHEEL`，脚本会拒绝
猜轮子）。`install.sh` 里的注释有这三者的分工与 ABI 约束，改之前先读。

> **轮子索引现状**（2026-09-01 实测）：`install.sh` 里印的
> `https://pypi.jetson-ai-lab.io/jp7/cu130` **已失效（404）**。该 devpi 服务器现在只有
> `jp6/cu126`、`jp6/cu128`、`jp6/cu129`、`sbsa/cu130`、`root`。CUDA 13 + aarch64 + cp312
> 对应的是 **`sbsa/cu130`**，里面有 `torch-2.11.0` 与 `cupy_cuda13x-13.6.0` 的
> cp312 aarch64 轮子，且该 index 会代理上游 PyPI（纯 python 包也能从它解析到）。
> 若机器的网络是白名单式的（实测 `pypi.org` 与清华/阿里镜像均超时），这可能是唯一可达的源。

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
  **必须手工装**，否则 `elevation_mapping_node` 一起跑就崩。装法见 **4.1**。

### 4. 星海图系统环境（编译不需要，运行需要）
在 ~/galaxea/install目录是星海图的系统环境，

烧录教程参考： https://docs.galaxea-dynamics.com/zh/Guide/ros_upgrade_guide_all/ros2_upgrade_guide_pro/#8