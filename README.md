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

## 4 编译安装g2o
```bash
cd ~/Navi/ros2_ws/src/3rdparty/g2o
mkdir build && cd build && cmake ..
make
sudo make install
```

## 5. 编译 ROS2 包

包括 fastlio、cloud_to_laserscan、slope_behavior、livox_ros_driver2、grid_map_pcl(重写了grid_map_pcl) 等。

先补齐 rosdep 能解析的依赖（首次编译或改过 package.xml 后执行）：

```bash
cd ~/Navi/ros2_ws
rosdep install --from-paths src --ignore-src -y
```

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
  因为它会带进 `grid-map-pcl` 与 `grid-map-filters-rsl`，而这两个在本仓库里是
  带本地扩展的源码版（坡度层、可通行性层等），装了会造成同名包重复。
  ```bash
  sudo apt install ros-jazzy-grid-map-core ros-jazzy-grid-map-msgs \
                   ros-jazzy-grid-map-ros ros-jazzy-grid-map-cv \
                   ros-jazzy-grid-map-cmake-helpers
  ```
- **其余 ROS 依赖**统一交给 rosdep（见第 5 节）。

### 4. 星海图系统环境（编译不需要，运行需要）
在 ~/galaxea/install目录是星海图的系统环境，

烧录教程参考： https://docs.galaxea-dynamics.com/zh/Guide/ros_upgrade_guide_all/ros2_upgrade_guide_pro/#8