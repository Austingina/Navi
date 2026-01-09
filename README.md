# 编译与运行说明

## 1. 下载代码

```bash
cd ~/
git clone -b navi https://gitlab.autel.com/avant/AI-Embodied/Mobility/fast_lio2.git Navi
```

## 2. 编译安装 3D BBS

```bash
cd ~/Navi/ros2_ws/src/3d_bbs
mkdir build
cd build
cmake ..
make -j
sudo make install
```

## 3. 编译 ROS2 包

包括 fastlio、cloud_to_laserscan、slope_behavior、livox_ros_driver2、grid_map_pcl(重写了grid_map_pcl) 等。

### 在本机编译
```bash
cd ~/Navi/ros2_ws
colcon build --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DCMAKE_MESSAGE_LOG_LEVEL=VERBOSE --event-handlers console_direct+ --symlink-install
```

### 在编译机编译
```bash
cd ~/Navi/ros2_ws
colcon build --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DCMAKE_MESSAGE_LOG_LEVEL=VERBOSE --event-handlers console_direct+ 
```
## 4. 运行脚本

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
### 启动NaviAgent

```bash
./start_mcp.sh
```
## 5. 编译运行环境
### 1. 硬件与系统环境
硬件：Jetson Orin系列

系统：Ubuntu 22.04，CUDA环境
### 2. ROS2 Humble 环境
sudo apt install ros-humble-desktop-full
### 3. 特定ros2包环境
包括：

nav2相关，nav2-common， nav2-bringup，

sudo apt install ros-humble-nav2*

grid-map相关，grid-map-msgs， grid-map-visualization,  grid-map-cv,  grid-map-filters

sudo apt install ros-humble-grid-map*
### 4. 星海图系统环境（编译不需要，运行需要）
在 ~/galaxea/install目录是星海图的系统环境，

烧录教程参考： https://docs.galaxea-dynamics.com/zh/Guide/ros_upgrade_guide_all/ros2_upgrade_guide_pro/#8