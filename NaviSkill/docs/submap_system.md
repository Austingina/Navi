# Submap 多楼层地图系统文档

## 1. 概述

Submap 系统是 NaviAgent 中用于管理多楼层/多区域地图的核心模块。它支持：

- **多子地图管理**：同时管理多个独立的地图区域（如不同楼层、室内外）
- **语义点位**：在每个子地图中定义命名的导航目标点
- **传送门（Portal）**：定义子地图之间的连接点，支持跨楼层导航
- **坐标变换**：不同子地图间的坐标转换
- **巡检路线**：预定义的跨子地图巡检路线

---

## 2. 配置文件结构

配置文件为 JSON 格式，通常命名为 `multifloor.json`。

### 2.1 完整结构示例

```json
{
  "_comment": "多楼层地图配置文件",
  
  "global": {
    "default_submap": "floor"
  },
  
  "submaps": {
    "floor": { ... },
    "underground": { ... },
    "5floor": { ... }
  },
  
  "routes": {
    "线路A": { ... }
  }
}
```

### 2.2 global 配置

| 字段 | 类型 | 说明 |
|------|------|------|
| `default_submap` | string | 默认子地图 ID，系统启动时加载此地图 |

### 2.3 submaps 配置

每个 submap 的配置结构：

```json
{
  "floor": {
    "name": "地面层",
    "description": "Ground floor / 1F",
    "map_file": "floor_2cm.pcd",
    "transform": {
      "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
      "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    "submap_portals": [ ... ],
    "semantic_points": { ... }
  }
}
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `name` | string | 是 | 子地图显示名称 |
| `description` | string | 否 | 子地图描述 |
| `map_file` | string | 是 | 点云地图文件路径（相对或绝对路径） |
| `transform` | object | 是 | 相对于 default_submap 的坐标变换 |
| `submap_portals` | array | 否 | 传送门列表（连接到其他子地图） |
| `semantic_points` | object | 是 | 语义点位定义 |

### 2.4 transform 坐标变换

```json
"transform": {
  "translation": {"x": -47.5, "y": -8.4, "z": 0.0},
  "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
}
```

**含义**：该子地图坐标系相对于 default_submap 坐标系的变换。

**计算公式**：
- 子地图坐标 → 全局坐标：$P_{global} = R \cdot P_{local} + T$
- 全局坐标 → 子地图坐标：$P_{local} = R^{-1} \cdot (P_{global} - T)$

**示例**：
- `floor`（default_submap）：`translation = (0, 0, 0)`，单位变换
- `underground`：`translation = (-47.5, -8.4, 0)`，表示 underground 的原点在 floor 坐标系中位于 `(-47.5, -8.4, 0)`

### 2.5 semantic_points 语义点位

支持两种类型：

#### 单点位（single_pose）

```json
"北面车库": {
  "type": "single_pose",
  "pose": {
    "position": {"x": -50.2, "y": -4.90, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.7071, "w": 0.7071}
  }
}
```

#### 多点位（multi_pose）

用于需要依次经过多个位置的场景（如出入口）：

```json
"东门-出门方向": {
  "type": "multi_pose",
  "poses": [
    {
      "position": {"x": -0.06, "y": -10.77, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": -0.7071, "w": 0.7071}
    },
    {
      "position": {"x": -0.06, "y": -11.50, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": -0.7071, "w": 0.7071}
    }
  ]
}
```

### 2.6 submap_portals 传送门

传送门是连接两个子地图的特殊点位：

```json
"submap_portals": [
  {
    "portal": "北面车库",
    "submap": "underground"
  },
  {
    "portal": "电梯B",
    "submap": "5floor"
  }
]
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `portal` | string | 传送门点位名称（必须在 semantic_points 中定义） |
| `submap` | string | 目标子地图 ID |

**注意**：传送门是单向的，需要在两个子地图中分别定义。

### 2.7 routes 巡检路线

预定义的跨子地图巡检路线：

```json
"routes": {
  "线路A": {
    "description": "从地面层出发，经过地下层，最后返回地面层",
    "waypoints": [
      {"submap": "floor", "semantic_point": "北面车库"},
      {"submap": "underground", "semantic_point": "地下层巡检起点"},
      {"submap": "floor", "semantic_point": "机器人巡检起点"}
    ]
  }
}
```

---

## 3. NaviAgent 模块架构

### 3.1 整体架构

NaviAgent 是一个基于 ROS2 和 MCP（Model Context Protocol）的智能导航代理系统。它将机器人导航能力封装为 AI 可调用的工具，支持大语言模型（LLM）直接控制机器人导航。

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           NaviAgent 系统架构                             │
└─────────────────────────────────────────────────────────────────────────┘

                          ┌──────────────────┐
                          │   LLM / AI Agent │
                          │  (Claude, GPT等)  │
                          └────────┬─────────┘
                                   │ MCP Protocol (HTTP/SSE)
                                   ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                            server.py                                     │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                   NavigationToolManager                          │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │   │
│  │  │   FastMCP    │  │  MapManager  │  │   WGS84Converter     │  │   │
│  │  │ (HTTP服务器)  │  │  (地图管理)   │  │   (经纬度转换)        │  │   │
│  │  └──────────────┘  └──────────────┘  └──────────────────────┘  │   │
│  │                           │                                      │   │
│  │  MCP Tools:               │                                      │   │
│  │  • set_goal              • get_submap_list                       │   │
│  │  • get_status            • change_map                            │   │
│  │  • wait_goal             • get_portal_to_submap                  │   │
│  │  • pause / resume        • get_route / get_route_names           │   │
│  │  • get_nearest_point     • ...                                   │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                   │                                     │
└───────────────────────────────────┼─────────────────────────────────────┘
                                    │ 调用
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                            nav_node.py                                   │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                         NavNode (ROS2 Node)                      │   │
│  │                                                                  │   │
│  │  发布者 (Publishers):              订阅者 (Subscribers):          │   │
│  │  • /goal_pose                     • /map (OccupancyGrid)        │   │
│  │  • /cmd_vel                       • /navigate_to_pose/status    │   │
│  │  • /change_map                                                   │   │
│  │  • /pose_in_target                TF 监听:                       │   │
│  │                                   • odom → base_link             │   │
│  │  Action Client:                                                  │   │
│  │  • NavigateToPose                 服务调用:                       │   │
│  │                                   • get_parameters (地图路径)     │   │
│  └─────────────────────────────────────────────────────────────────┘   │
└───────────────────────────────────┼─────────────────────────────────────┘
                                    │ ROS2 Topics/Services/Actions
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                          ROS2 Navigation Stack                           │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌────────────┐  │
│  │  Nav2 Stack  │  │ Localization │  │  Map Server  │  │  Sensors   │  │
│  │  (路径规划)   │  │   (定位)      │  │  (地图服务)   │  │  (传感器)   │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  └────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
```

### 3.2 核心模块说明

| 模块 | 文件 | 职责 |
|------|------|------|
| **NavigationToolManager** | `server.py` | MCP 服务入口，注册所有导航工具，管理 ROS 线程 |
| **NavNode** | `nav_node.py` | ROS2 节点，与导航栈交互，发布目标、监听状态 |
| **MapManager** | `map_manager.py` | 多楼层地图配置管理，语义点位、传送门、路线管理 |
| **WGS84Converter** | `wgs84.py` | GPS 经纬度与地图坐标的相互转换 |

### 3.3 数据流

```
用户指令 (自然语言)
    │
    ▼
LLM 解析并调用 MCP Tool
    │
    ▼
NavigationToolManager 处理请求
    │
    ├─► MapManager: 查询点位/地图/路线
    │
    ├─► NavNode: 发布导航目标
    │       │
    │       ▼
    │   ROS2 Navigation Stack
    │       │
    │       ▼
    │   机器人执行导航
    │
    └─► 返回结果给 LLM
```

### 3.4 模块依赖关系

```
server.py (NavigationToolManager)
    │
    ├── nav_node.py (NavNode)      # ROS2 通信
    │       └── rclpy, tf2_ros, nav2_msgs
    │
    ├── map_manager.py (MapManager) # 地图配置
    │       └── 纯 Python，无 ROS 依赖
    │
    ├── wgs84.py (WGS84Converter)   # 坐标转换
    │       └── pyproj
    │
    └── fastmcp                     # MCP 服务框架
```

---

## 4. MapManager 模块设计

### 4.1 MapManager 在架构中的位置

MapManager 是 NaviAgent 中**唯一负责地图配置管理**的模块：

- **向上**：为 NavigationToolManager 提供点位查询、地图切换、路线获取等能力
- **向下**：读取 JSON 配置文件，不直接与 ROS 交互
- **特点**：纯 Python 实现，可独立于 ROS 环境进行单元测试

### 4.2 类结构

```
MapManager
├── 配置加载
│   ├── load_config()
│   └── config_path, config, submaps
├── Submap 管理
│   ├── get_submap_list()
│   ├── get_submap_info()
│   ├── get_current_submap()
│   ├── set_current_submap()
│   ├── get_submap_map_file()
│   ├── get_submap_full_map_path()
│   └── get_submap_id_by_map_file()
├── 语义点位管理
│   ├── get_semantic_points_in_submap()
│   ├── get_pose_in_submap()
│   ├── get_multi_poses_in_submap()
│   ├── is_multi_pose()
│   ├── find_pose_in_all_submaps()
│   └── pose_map_current_submap (属性)
├── Portal 管理
│   ├── get_submap_portals()
│   ├── get_portal_to_submap()
│   ├── get_portal_pose()
│   └── find_path_between_submaps()
├── 路线管理
│   ├── get_route_list()
│   ├── get_route_by_name()
│   └── get_route_names()
└── 坐标变换
    ├── get_submap_transform()
    └── get_init_pose_after_changemap()
```

### 4.3 核心方法说明

#### 4.3.1 坐标变换

```python
def get_init_pose_after_changemap(
    current_pose: Dict,
    from_submap: str,
    to_submap: str
) -> Optional[Dict]
```

**功能**：计算切换地图后机器人在目标子地图中的初始位姿。

**算法**：
1. 当前层坐标 → 全局坐标：$P_{global} = T_{from} \cdot P_{current}$
2. 全局坐标 → 目标层坐标：$P_{target} = T_{to}^{-1} \cdot P_{global}$

**输入**：
- `current_pose`：当前位姿 `{position: {x,y,z}, orientation: {x,y,z,w}}`
- `from_submap`：当前子地图 ID
- `to_submap`：目标子地图 ID

**输出**：目标子地图中的位姿

#### 4.3.2 路径查找

```python
def find_path_between_submaps(from_submap: str, to_submap: str) -> Optional[List[Dict]]
```

**功能**：使用 BFS 算法查找从起始子地图到目标子地图的传送门路径。

**返回**：`[{submap, portal, next_submap}, ...]` 或 `None`

---

## 5. MCP 工具接口

NaviAgent 通过 FastMCP 暴露以下 submap 相关工具：

### 5.1 子地图查询

| 工具名称 | 参数 | 返回值 | 说明 |
|---------|------|--------|------|
| `get_submap_list` | 无 | `{submaps: [...], count: N}` | 获取所有子地图 ID |
| `get_submap_info` | `submap_id` | `{submap_id, info: {...}}` | 获取子地图详细信息 |
| `get_current_map_id` | 无 | `{map_id, map_path}` | 获取当前加载的地图 |

### 5.2 地图切换

| 工具名称 | 参数 | 返回值 | 说明 |
|---------|------|--------|------|
| `change_map` | `map_id`, `timeout=30.0` | `{map_id, pose_in_source, pose_in_target}` | 切换到指定地图 |

**change_map 详细流程**：

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        地图切换流程 (change_map)                          │
└─────────────────────────────────────────────────────────────────────────┘

NaviAgent (server.py)                    ROS2 节点
      │                                      │
      │  1. 校验 map_id 是否存在              │
      │                                      │
      │  2. 获取当前地图路径                   │
      │  ─────────────────────────────────►  │ get_current_map_path()
      │  ◄─────────────────────────────────  │ 返回 /path/to/floor.pcd
      │                                      │
      │  3. 获取当前位姿 (current_pose)       │
      │                                      │
      │  4. 计算目标地图中的位姿              │
      │     (get_init_pose_after_changemap)  │
      │     应用坐标变换:                     │
      │     T_from → T_global → T_to^{-1}   │
      │                                      │
      │  5. 发送地图切换请求                  │
      │  ─────────────────────────────────►  │ /change_map (String)
      │                                      │
      │                                      │  ┌─ grid_map_pcl_loader_node ─┐
      │                                      │  │ ├─ 取消现有定时器            │
      │                                      │  │ ├─ 加载新 PCD 点云           │
      │                                      │  │ ├─ 计算 elevation/slope 层  │
      │                                      │  │ ├─ 生成 OccupancyGrid       │
      │                                      │  │ ├─ 发布 /map 话题            │
      │                                      │  │ └─ 更新 map_file_path 参数  │
      │                                      │  └────────────────────────────┘
      │                                      │
      │                                      │  ┌─ laserOdometry3dbbs ────────┐
      │                                      │  │ ├─ 清空 ikdtree              │
      │                                      │  │ ├─ 加载新地图 PCD            │
      │                                      │  │ ├─ 更新 map_file_path 参数  │
      │                                      │  │ └─ 触发重定位                │
      │                                      │  └────────────────────────────┘
      │                                      │
      │  6. 轮询等待地图切换完成 (500ms间隔)   │
      │  ─────────────────────────────────►  │ get_parameters (map_file_path)
      │  ◄─────────────────────────────────  │ 返回新地图路径
      │     比较 basename 确认切换完成        │
      │                                      │
      │  7. 更新 MapManager.current_submap   │
      │                                      │
      │  8. 发布重定位提示位姿                │
      │  ─────────────────────────────────►  │ /pose_in_target (PoseStamped)
      │                                      │
      │                                      │  laserOdometry3dbbs.cpp:
      │                                      │  ├─ 订阅 /pose_in_target 获取提示位姿
      │                                      │  ├─ 设置 relocalize_hint_pos
      │                                      │  └─ 基于提示位置搜索配准
      │                                      │
      │  9. 返回结果                          │
      └──────────────────────────────────────┘
```

**流程说明**：

| 步骤 | 操作 | 说明 |
|------|------|------|
| 1 | 校验 map_id | 检查目标地图是否在配置中存在 |
| 2 | 获取当前地图 | 通过 ROS2 参数服务查询 `/grid_map_pcl_loader` 的 `map_file_path` |
| 3 | 记录当前位姿 | 从 TF 获取机器人在当前地图中的位姿 |
| 4 | 坐标变换 | 使用四元数变换计算机器人在目标地图中的位姿 |
| 5 | 发送切换请求 | 向 `/change_map` 发布目标地图路径 |
| 6 | 轮询等待 | 每 500ms 查询一次，确认地图已切换（最长 30 秒） |
| 7 | 更新状态 | 更新 MapManager 内部的 current_submap |
| 8 | 发布重定位提示 | 向 `/pose_in_target` 发布计算的位姿，帮助定位节点快速收敛 |
| 9 | 返回结果 | 包含切换前后的位姿信息 |

**ROS2 节点响应**：

`/change_map` 消息会被两个节点同时接收处理：

**1. grid_map_pcl_loader_node（栅格地图服务）**：

| 步骤 | 操作 | 说明 |
|------|------|------|
| 1 | 取消定时器 | 停止现有的 GridMap 和 OccupancyGrid 定时发布 |
| 2 | 加载点云 | 从新 PCD 文件加载点云数据 |
| 3 | 计算高程层 | 生成 elevation layer |
| 4 | 计算坡度层 | 基于 elevation 生成 slope layer |
| 5 | 计算可通行性 | 基于障碍点密度生成 traversability layer |
| 6 | 生成栅格图 | 融合 traversability 和 slope 生成 OccupancyGrid |
| 7 | 发布话题 | 发布 `/map` (OccupancyGrid) 和 `grid_map_from_raw_pointcloud` |
| 8 | 更新参数 | 同步更新 ROS2 参数 `map_file_path` |
| 9 | 启动定时器 | 创建新定时器周期性重发布地图 |

**2. laserOdometry3dbbs（定位节点）**：

| 步骤 | 操作 | 说明 |
|------|------|------|
| 1 | 清空 ikdtree | 重建空的增量 KD 树 |
| 2 | 加载点云 | 从新 PCD 文件加载点云并构建 ikdtree |
| 3 | 更新参数 | 同步更新 ROS2 参数 `map_file_path` |
| 4 | 触发重定位 | 设置 `flg_first_scan = true`，下次扫描执行全局配准 |

当收到 `/pose_in_target` 消息时（仅 laserOdometry3dbbs）：
1. **设置搜索提示**：使用接收的位置作为搜索中心
2. **缩小搜索范围**：在提示位置附近搜索，而非全地图搜索
3. **加速收敛**：显著减少重定位时间

### 5.3 传送门查询

| 工具名称 | 参数 | 返回值 | 说明 |
|---------|------|--------|------|
| `get_portal_to_submap` | `target_submap` | `{portal, portal_pose, from_submap, target_submap}` | 获取到目标子地图的传送门 |

### 5.4 路线管理

| 工具名称 | 参数 | 返回值 | 说明 |
|---------|------|--------|------|
| `get_route_names` | 无 | `{route_names: [...], count: N}` | 获取所有路线名称 |
| `get_route` | `route_name` | `{name, description, waypoints: [...]}` | 获取路线详情 |

### 5.5 点位查询

| 工具名称 | 参数 | 返回值 | 说明 |
|---------|------|--------|------|
| `get_pose_list_in_current_submap` | 无 | `{keys: [...], count: N}` | 获取当前子地图的点位列表 |

---

## 6. ROS2 话题接口

### 6.1 发布话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/change_map` | `std_msgs/String` | 地图切换请求（发送目标地图的完整路径） |
| `/pose_in_target` | `geometry_msgs/PoseStamped` | 切换地图后的重定位提示位姿 |

### 6.2 订阅/服务

| 名称 | 类型 | 说明 |
|------|------|------|
| `/grid_map_pcl_loader_node/get_parameters` | Service | 获取当前地图路径参数 |

---

## 7. 测试客户端使用

### 7.1 启动服务端

```bash
cd /home/nvidia/Navi/NaviAgent
python server.py -t -m /home/nvidia/Navi/maps/multifloor/multifloor.json
```

参数说明：
- `-t`：测试模式（端口 9999）
- `-m`：指定多楼层配置文件路径

### 7.2 运行测试客户端

```bash
cd /home/nvidia/Navi/NaviAgent/test
python test_client_mf.py
```

### 7.3 测试流程时序

```
┌─────────────────────────────────────────────────────────────────┐
│                    test_client_mf.py 执行流程                     │
└─────────────────────────────────────────────────────────────────┘

1. 初始化阶段
   │
   ├─► get_pose_list_in_current_submap()  获取当前子地图点位列表
   │
   ├─► get_route("线路A")                  获取预定义路线
   │
   ├─► get_status()                        获取机器人当前状态
   │
   └─► get_nearest_point()                 获取最近点位

2. 巡检循环（遍历 waypoints）
   │
   ├─► 解析 waypoint
   │   │
   │   ├── 简单字符串: target_name = waypoint
   │   │
   │   └── 字典格式: {submap, semantic_point}
   │       │
   │       └─► 需要跨子地图导航
   │
   ├─► 跨子地图导航（如需要）
   │   │
   │   ├─► get_current_map_id()            获取当前子地图 ID
   │   │
   │   ├─► get_portal_to_submap()          获取传送门信息
   │   │
   │   ├─► set_goal_by_xy()                导航到传送门位置
   │   │
   │   ├─► wait_goal_with_pause()          等待到达传送门
   │   │
   │   └─► change_map()                    切换地图
   │       │
   │       ├── 发送 /change_map
   │       ├── 轮询等待切换完成
   │       └── 发布 /pose_in_target
   │
   ├─► 导航到目标点
   │   │
   │   ├─► set_goal()                      下发导航目标
   │   │
   │   └─► wait_goal_with_pause()          等待到达（支持暂停/恢复）
   │
   └─► 记录结果，继续下一个 waypoint

3. 生成测试报告
```

### 7.4 键盘控制

在 `wait_goal_with_pause()` 等待期间：
- **空格键**：暂停/恢复导航
- **Ctrl+C**：中断测试

### 7.5 环境变量

| 变量名 | 说明 | 示例 |
|--------|------|------|
| `ROUTE_NAME` | 指定使用的路线名称 | `export ROUTE_NAME="线路A"` |

---

## 8. 坐标变换详解

### 8.1 Transform 定义

每个子地图的 `transform` 定义了该子地图坐标系相对于 default_submap（通常是 floor）的变换关系。

### 8.2 计算示例

假设：
- `floor` transform: `{translation: (0, 0, 0), rotation: identity}`
- `underground` transform: `{translation: (-47.5, -8.4, 0), rotation: identity}`

机器人在 `floor` 坐标系的位置 `(-50.2, -4.90, 0)`，切换到 `underground` 后的位置：

```
Step 1: floor → global (floor 就是 global)
P_global = P_floor = (-50.2, -4.90, 0)

Step 2: global → underground
P_underground = P_global - T_underground
             = (-50.2, -4.90, 0) - (-47.5, -8.4, 0)
             = (-2.7, 3.5, 0)
```

### 8.3 如何确定 Transform

1. 找到两个子地图中**物理上同一位置**的点
2. 记录该点在两个子地图中的坐标
3. 计算：`T = P_floor - P_submap`

---

## 9. 最佳实践

### 9.1 配置文件组织

```
maps/
└── multifloor/
    ├── multifloor.json      # 配置文件
    ├── floor_2cm.pcd        # 地面层点云
    ├── underground_2cm.pcd  # 地下层点云
    └── 5floor_2cm.pcd       # 5楼点云
```

### 9.2 传送门设计

- 传送门应设置在两个子地图**物理上重叠**的位置（如电梯、楼梯、坡道）
- 确保在**两个子地图中都定义**对应的传送门点位
- 传送门点位的坐标应反映实际的物理位置

### 9.3 路线规划

- 路线应考虑传送门的顺序
- 跨子地图时，waypoint 必须使用 `{submap, semantic_point}` 格式
- 同一子地图内的连续 waypoint 可以使用简单字符串

### 9.4 坐标系对齐

- 建议所有子地图使用一致的坐标系方向
- 如果存在旋转差异，需要在 `transform.rotation` 中正确配置四元数

---

## 10. 故障排查

### 10.1 常见问题

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| 地图切换超时 | ROS 服务不可用 | 检查 `/grid_map_pcl_loader_node` 是否运行 |
| 切换后位置错误 | transform 配置错误 | 重新测量并校准 transform |
| 找不到传送门 | portal 未在两边定义 | 在两个子地图中都添加传送门配置 |
| 点位不存在 | submap 未切换 | 确认 `change_map` 成功执行 |

### 10.2 调试命令

```bash
# 查看当前地图参数
ros2 param get /grid_map_pcl_loader_node map_file_path

# 监听地图切换消息
ros2 topic echo /change_map

# 监听切换后位姿
ros2 topic echo /pose_in_target
```

---

## 11. API 快速参考

### Python (MapManager)

```python
from map_manager import MapManager

# 初始化
mm = MapManager("/path/to/multifloor.json")

# 获取子地图列表
submaps = mm.get_submap_list()  # ['floor', 'underground', '5floor']

# 获取当前子地图
current = mm.get_current_submap()  # 'floor'

# 切换子地图
mm.set_current_submap('underground')

# 获取点位
pose = mm.get_pose_in_submap('北面车库')

# 获取传送门
portal = mm.get_portal_to_submap('underground')

# 坐标变换
new_pose = mm.get_init_pose_after_changemap(current_pose, 'floor', 'underground')

# 获取路线
route = mm.get_route_by_name('线路A')
```

### MCP 工具调用

```python
from fastmcp import Client

async with Client("http://localhost:9999/mcp") as client:
    # 获取子地图列表
    result = await client.call_tool("get_submap_list")
    
    # 切换地图
    result = await client.call_tool("change_map", {"map_id": "underground"})
    
    # 获取路线
    result = await client.call_tool("get_route", {"route_name": "线路A"})
```
