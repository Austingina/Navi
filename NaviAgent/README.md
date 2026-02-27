# NaviAgent - 基于MCP的机器人导航智能体

## 📖 概述

NaviAgent 是一个基于 **FastMCP（Model Context Protocol）** 框架和 **ROS2** 的机器人导航智能体系统。它将 ROS2 Navigation2 导航栈封装为标准化的 MCP 工具接口，使得大语言模型（LLM）或其他上层应用能够通过 MCP 协议直接控制机器人完成自主导航任务。

### 核心能力

- 🗺️ **语义地图导航**：通过预设点名称（如"配电房"、"灭火器箱"）下发导航目标
- 🌍 **GPS经纬度导航**：输入WGS84经纬度坐标，自动转换为地图坐标并导航
- 📍 **地图坐标导航**：直接使用地图坐标系下的 XY 坐标进行导航
- 🔄 **巡逻路线管理**：支持预设巡逻路线的查询与执行
- 📡 **实时状态查询**：获取机器人位姿、最近预设点、栅格地图等信息
- 🎯 **精确到达控制**：Navigation2 导航 + PID 精调两阶段控制，确保到达精度

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────┐
│                  上层应用 / LLM Agent                 │
│            (MCP Client / Web UI / 测试脚本)           │
└──────────────────────┬──────────────────────────────┘
                       │  MCP (Streamable HTTP)
                       ▼
┌─────────────────────────────────────────────────────┐
│                    server.py                         │
│              NavigationToolManager                   │
│  ┌──────────┐  ┌──────────────┐  ┌───────────────┐  │
│  │ FastMCP  │  │ SemanticMap  │  │ WGS84Converter│  │
│  │ (工具注册)│  │ (语义地图)    │  │ (坐标转换)     │  │
│  └──────────┘  └──────────────┘  └───────────────┘  │
│                       │                              │
│               ┌───────▼────────┐                     │
│               │  MCPNavNode    │                     │
│               │  (ROS2 节点)    │                     │
│               └───────┬────────┘                     │
└───────────────────────┼─────────────────────────────┘
                        │  ROS2 Topics / Actions / TF
                        ▼
┌─────────────────────────────────────────────────────┐
│              ROS2 Navigation2 导航栈                  │
│    (SLAM定位 / 路径规划 / 局部避障 / 地图服务)         │
└─────────────────────────────────────────────────────┘
```

---

## 📁 项目结构

```
NaviAgent/
├── server.py              # 主入口，MCP 服务 & 工具注册
├── mcp_nav_node.py        # ROS2 导航节点（发布目标、状态监控、PID控制）
├── semantic_map.py        # 语义地图管理（加载预设点位配置）
├── wgs84.py               # WGS84 经纬度 ↔ 地图坐标系 转换器
├── start_mcp.sh           # 启动脚本
├── requirements.txt       # Python 依赖列表
├── config/                # 配置文件目录
│   ├── 525.json           # 525 场景预设点位（示例）
│   ├── caihong.json       # 彩虹场景预设点位
│   ├── mapuse.txt         # 当前使用的地图配置名称
│   ├── map.txt            # WGS84 原点 & 朝向参数（lon0,lat0,theta）
│   └── traj.txt           # GPS轨迹数据（用于自动标定坐标转换参数）
├── audio/                 # 音频反馈文件
│   ├── success.mp3        # 定位成功提示音
│   └── fail.mp3           # 定位丢失提示音
├── test/                  # 测试工具
│   ├── test_client.py     # Python 巡检测试客户端
│   ├── test_client_ui.py  # 带地图可视化的 Tkinter 测试 UI
│   ├── mcp_nav_client.html# Web 测试客户端（带地图渲染）
│   ├── test_client525.py  # 525 场景专用测试
│   └── test_nearest_point.py # 最近点查询测试
└── __pycache__/           # Python 字节码缓存
```

---

## 🔧 模块详解

### 1. `server.py` — MCP 服务与工具注册

**核心类：`NavigationToolManager`**

负责系统的整体协调：
- 初始化 FastMCP 实例，注册所有导航工具
- 初始化 ROS2 节点并在后台线程运行 `spin`
- 加载语义地图（`SemanticMap`）和坐标转换器（`WGS84Converter`）
- 以 Streamable HTTP 方式对外提供 MCP 服务

**启动参数：**

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `-t` / `--test` | 测试模式，使用 9999 端口 | 关闭（使用 9000 端口） |

### 2. `mcp_nav_node.py` — ROS2 导航节点

**核心类：`MCPNavNode(Node)`**

作为 ROS2 节点与 Navigation2 导航栈交互：

| 功能 | 实现方式 |
|------|----------|
| 发布导航目标 | `NavigateToPose` Action Client |
| 获取机器人位姿 | TF2 监听 `map` → `base_link` 变换（10Hz） |
| 速度控制 | 发布 `Twist` 到 `/cmd_vel` |
| 地图边界获取 | 订阅 `/map`（OccupancyGrid） |
| 精确到达控制 | Navigation2 到达后自动进入 PID 二阶段调整（先旋转，后平移） |
| 定位状态监控 | TF 可用性检测，自动播放音频反馈 |

**导航控制流程：**

```
publish_goal() → Navigation2 Action → 到达目标区域
                                          │
                                          ▼
                                    PID 旋转阶段 → yaw 误差 < 容差
                                          │
                                          ▼
                                    PID 平移阶段 → 距离误差 < 容差
                                          │
                                          ▼
                                      导航完成 ✓
```

### 3. `semantic_map.py` — 语义地图管理

**核心类：`SemanticMap`**

从 JSON 配置文件加载命名点位：

- 自动从 `config/mapuse.txt` 读取当前地图名称
- 加载对应的 `config/<mapname>.json` 文件
- 兼容 `pose` 和 `poses` 两种字段格式

**配置文件格式示例：**

```json
{
  "灭火器箱": {
    "desc": "灭火器箱",
    "pose": {
      "position": {"x": -1.3, "y": -2.25, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  }
}
```

### 4. `wgs84.py` — WGS84 坐标转换器

**核心类：`WGS84Converter`**

提供经纬度（WGS84）与地图坐标系之间的双向转换：

- **初始化方式**：优先读取 `config/map.txt`（手动标定参数），否则从 `config/traj.txt`（GPS轨迹）自动拟合
- **`map.txt` 格式**：`经度原点,纬度原点,旋转角(弧度)` 如 `113.935704,22.557444,0.046782`
- **转换算法**：球面近似 + 旋转变换，适用于小范围场景

---

## 🔌 MCP 工具接口

系统通过 MCP 协议暴露以下工具：

### 导航控制类

| 工具名 | 功能 | 关键参数 |
|--------|------|----------|
| `set_goal` | 按预设点名称导航 | `target`: 目标点名称 |
| `set_goal_by_lonlat` | 按 GPS 经纬度导航 | `lon`, `lat`, `distance`(默认1.5m) |
| `set_goal_by_xy` | 按地图坐标导航 | `x`, `y`, `distance`(默认1.5m) |
| `pause_navigation` | 暂停导航 | 无 |
| `resume_navigation` | 继续导航 | 无 |
| `cancel_navigation` | 取消导航 | 无 |
| `wait_goal_reached` | 阻塞等待到达 | `target`, `timeout`(默认120s), `xy_tolerance`(默认0.1m), `yaw_tolerance`(默认0.1rad) |

### 状态查询类

| 工具名 | 功能 | 返回内容 |
|--------|------|----------|
| `get_status` | 获取机器人实时位姿 | position(x,y,z) + orientation(四元数) |
| `get_nearest_point` | 获取最近预设点 | 最近点名称、距离、所有点距离 |
| `get_pose_list` | 获取所有预设点名称 | 点位名称列表 |
| `get_route_names` | 获取所有巡逻路线名称 | 路线名称列表 |
| `get_route` | 获取指定路线的点位序列 | 有序点位列表 |
| `get_map` | 获取当前栅格地图 | 地图元信息 + 栅格数据 |

### 接口返回格式

所有接口统一返回 `Dict[str, Any]`：

```json
// 成功
{"success": true, "msg": "...", ...}

// 失败
{"success": false, "msg": "具体错误信息"}
```

---

## 🚀 快速开始

### 环境要求

| 依赖 | 版本要求 |
|------|---------|
| ROS2 | Humble |
| Python | 3.10+ |
| Navigation2 | 与 ROS2 Humble 匹配 |
| FastMCP | 最新版 |
| NumPy / SciPy | 科学计算 |
| ffplay | 音频播放（可选） |

### 1. 配置地图

编辑 `config/mapuse.txt` 指定当前使用的地图配置名称：

```
525
```

确保 `config/` 目录下存在对应的 JSON 点位文件（如 `config/525.json`）。

如需 GPS 导航功能，确保 `config/map.txt` 包含正确的坐标转换参数：

```
113.93570448233005,22.557444947600175,0.04678200461339619
```

### 2. 启动服务

**生产模式（端口 9000）：**

```bash
cd NaviAgent
python server.py
```

**测试模式（端口 9999）：**

```bash
cd NaviAgent
python server.py -t
```

**使用启动脚本：**

```bash
bash start_mcp.sh   # 激活 conda 环境并以测试模式启动
```

### 3. 连接测试

**Python 客户端：**

```python
import asyncio
from fastmcp import Client

async def main():
    async with Client("http://localhost:9999/mcp") as client:
        # 获取可用点位
        result = await client.call_tool("get_pose_list")
        print(result)

        # 导航到目标点
        result = await client.call_tool("set_goal", {"target": "灭火器箱"})
        print(result)

        # 等待到达
        result = await client.call_tool("wait_goal_reached", {
            "target": "灭火器箱",
            "timeout": 120.0
        })
        print(result)

asyncio.run(main())
```

**Web 测试客户端：**

直接在浏览器中打开 `test/mcp_nav_client.html`，支持地图可视化、点击导航、巡逻路线管理等功能。

**带 UI 的 Python 客户端：**

```bash
python test/test_client_ui.py
```

提供 Tkinter 图形界面，包含栅格地图显示、实时机器人位置、路线巡检控制。

---

## 🗺️ 地图配置指南

### 添加新场景

1. 在 `config/` 下创建新的 JSON 文件（如 `mysite.json`）：

```json
{
  "点位A": {
    "desc": "描述信息",
    "pose": {
      "position": {"x": 1.0, "y": 2.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
    }
  },
  "点位B": {
    "desc": "另一个点位",
    "pose": {
      "position": {"x": 5.0, "y": -3.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  }
}
```

2. 修改 `config/mapuse.txt` 为新的配置名（不含 `.json` 后缀）：

```
mysite
```

3. 如需 GPS 导航，配置 `config/map.txt` 或提供 `config/traj.txt` 自动标定。

### 获取点位坐标

可以通过以下方式获取点位的 `position` 和 `orientation`：

- **RViz2**：使用 "2D Pose Estimate" 工具在地图上标记点位，从终端读取坐标
- **MCP get_status 接口**：手动将机器人移至目标位置后调用 `get_status` 获取当前位姿

---

## 🔄 典型使用流程

### 单点导航

```
get_pose_list → set_goal("灭火器箱") → wait_goal_reached("灭火器箱")
```

### 巡逻路线

```
get_route_names → get_route("线路A") → 
  for point in route:
    set_goal(point) → wait_goal_reached(point)
```

### GPS 坐标导航

```
set_goal_by_lonlat(lon=113.935, lat=22.557, distance=2.0) → 
  wait_goal_reached("lonlat(...)")
```

---

## ⚙️ ROS2 话题与服务

### 订阅的话题

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/map` | `nav_msgs/OccupancyGrid` | 获取栅格地图和边界信息 |

### 发布的话题

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | PID 精调阶段的速度控制 |

### 使用的 Action

| Action | 类型 | 说明 |
|--------|------|------|
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` | 向 Navigation2 下发导航目标 |

### TF 变换

| 源坐标系 | 目标坐标系 | 说明 |
|----------|-----------|------|
| `base_link` | `map` | 机器人在地图坐标系中的位姿（10Hz 查询） |

---

## ⚠️ 常见问题

### 1. "无实时位姿数据"

**原因**：TF 变换不可用，通常是定位节点（如 AMCL / FAST-LIO）未启动或定位丢失。

**解决**：检查定位节点是否正常运行，RViz 中确认 TF 树完整。

### 2. "目标点 xxx 不在白名单中"

**原因**：请求的目标点名称不在当前加载的语义地图中。

**解决**：调用 `get_pose_list` 查看可用点位名称，或更新 JSON 配置文件。

### 3. "超出地图边界"

**原因**：目标坐标超出了从 `/map` 话题获取的栅格地图范围。

**解决**：确认坐标参数正确，检查地图是否正确加载。

### 4. GPS 导航坐标偏移大

**原因**：`config/map.txt` 中的坐标转换参数不准确。

**解决**：
- 收集多组对应的地图坐标和GPS坐标存入 `config/traj.txt`
- 删除 `config/map.txt`，系统会自动从轨迹数据拟合最优参数
- 或手动调整 `map.txt` 中的经度原点、纬度原点和旋转角

---

## 📋 版本信息

- **框架**: FastMCP (Streamable HTTP)
- **ROS 版本**: ROS2 Humble
- **Python**: 3.10+
- **导航栈**: Navigation2
