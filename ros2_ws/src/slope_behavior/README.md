# Slope Behavior - Nav2 行为树节点

Nav2 行为树节点插件，用于在机器人导航经过高坡度区域时自动检测并控制重心。

## 功能描述

该插件提供三个 BT 节点，集成到 Nav2 行为树中实现坡度检测和重心控制：

1. **IsSlopeAhead** - 条件节点：检测前方路径是否存在陡坡
2. **LowerGravity** - 动作节点：降低重心（躯干弯曲、手臂抬起）
3. **RestoreGravity** - 动作节点：恢复直立状态

## BT 节点说明

### IsSlopeAhead (条件节点)

检测前方路径是否有坡度超过阈值的区域。

| 端口 | 类型 | 默认值 | 描述 |
|------|------|--------|------|
| `slope_threshold` | double | 15.0 | 坡度阈值（度） |
| `check_distance` | double | 2.0 | 检查前方距离（米） |
| `grid_map_topic` | string | "/grid_map" | grid_map 话题 |
| `path_topic` | string | "/local_plan" | 局部路径话题 |
| `slope_layer` | string | "slope" | 坡度层名称 |

### LowerGravity (动作节点)

降低重心，使机器人进入低重心姿态（躯干弯曲、手臂抬起90度）。

| 端口 | 类型 | 默认值 | 描述 |
|------|------|--------|------|
| `duration` | double | 2.0 | 动作持续时间（秒） |
| `torso_pitch` | double | 1.74 | 躯干俯仰角 |
| `torso_roll` | double | -2.70 | 躯干横滚角 |
| `torso_yaw` | double | -0.96 | 躯干偏航角 |

### RestoreGravity (动作节点)

恢复直立状态（躯干直立、手臂下垂）。

| 端口 | 类型 | 默认值 | 描述 |
|------|------|--------|------|
| `duration` | double | 2.0 | 动作持续时间（秒） |

## 关节状态对应

| 状态 | 左臂 | 右臂 | 躯干 |
|------|------|------|------|
| **直立** | `[0,0,0,0,0,0,0]` | `[0,0,0,0,0,0,0]` | `[0,0,0,0]` |
| **低重心** | `[0,0,0,-1.57,0,0,0]` | `[0,0,0,-1.57,0,0,0]` | `[1.74,-2.70,-0.96,0]` |

## 话题接口

### 订阅话题
- `/grid_map` (`grid_map_msgs/msg/GridMap`): 包含 slope 层的 grid_map 数据
- `/local_plan` (`nav_msgs/msg/Path`): 局部路径规划结果

### 发布话题
- `/motion_target/target_joint_state_arm_left` (`sensor_msgs/msg/JointState`): 左臂关节状态
- `/motion_target/target_joint_state_arm_right` (`sensor_msgs/msg/JointState`): 右臂关节状态
- `/motion_target/target_joint_state_torso` (`sensor_msgs/msg/JointState`): 躯干关节状态
- `/motion_target/target_speed_chassis` (`geometry_msgs/msg/TwistStamped`): 底盘速度命令
- `/motion_target/brake_mode` (`std_msgs/msg/Bool`): 制动模式

## 配置步骤

### 1. 编译

```bash
cd ~/work/Navi/ros2_ws
colcon build --packages-select slope_behavior
source install/setup.bash
```

### 2. 更新 param_r1pro.yaml

在 `bt_navigator` 的 `plugin_lib_names` 中添加插件库：

```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      # ... 其他插件 ...
      - slope_bt_nodes  # 添加这行
```

### 3. 使用行为树

可以使用提供的示例行为树，或在现有行为树中添加节点：

```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: /path/to/navigate_to_pose_with_slope_detection.xml
```

## 行为树 XML 示例

```xml
<!-- 检测陡坡并处理 -->
<Fallback name="SlopeHandling">
  <!-- 没有陡坡，继续导航 -->
  <Inverter>
    <IsSlopeAhead 
      slope_threshold="15.0"
      check_distance="2.0"
      grid_map_topic="/grid_map"
      path_topic="/local_plan"
      slope_layer="slope"/>
  </Inverter>
  
  <!-- 有陡坡，降低重心 -->
  <LowerGravity 
    duration="2.0"
    torso_pitch="1.74"
    torso_roll="-2.70"
    torso_yaw="-0.96"/>
</Fallback>

<!-- 跟随路径 -->
<FollowPath path="{path}" controller_id="FollowPath"/>

<!-- 导航完成后恢复直立 -->
<RestoreGravity duration="2.0"/>
```

## 工作流程

```
导航开始
    ↓
计算全局路径
    ↓
IsSlopeAhead 检测前方坡度
    ↓
┌─── FAILURE (无陡坡) ───→ 直接 FollowPath
│
└─── SUCCESS (有陡坡)
            ↓
      LowerGravity 降低重心
      (停止底盘、躯干弯曲、手臂抬起)
            ↓
      FollowPath 继续导航
            ↓
      RestoreGravity 恢复直立
```

## 依赖

- ROS2 Humble
- nav2_core
- nav2_util
- behaviortree_cpp_v3
- grid_map_ros
- grid_map_msgs
- sensor_msgs

## 文件结构

```
slope_behavior/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/slope_behavior/
│   ├── gravity_control_action.hpp
│   └── is_slope_ahead_condition.hpp
├── src/
│   ├── gravity_control_action.cpp
│   └── is_slope_ahead_condition.cpp
└── behavior_trees/
    └── navigate_to_pose_with_slope_detection.xml
```

## 注意事项

1. 确保有节点发布包含 `slope` 层的 `grid_map` 数据
2. 机器人底层需要订阅 `/motion_target/` 相关话题并执行关节控制
3. 可以根据实际机器人调整 `duration` 和躯干角度参数
