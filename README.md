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

包括 fastlio、cloud_to_laserscan、slope_behavior、livox_ros_driver2、grid_map_pcl 等。

```bash
cd ~/Navi/ros2_ws
colcon build --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DCMAKE_MESSAGE_LOG_LEVEL=VERBOSE --event-handlers console_direct+ --symlink-install
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