# 导航服务 WebSocket 接口说明

本文档描述 `server_fast.py` 提供的 **FastAPI + WebSocket** 导航 RPC 接口，供客户端（如 `test/test_client_fast.py`）或第三方系统接入。

---

## 1. 服务启动与前缀

### 1.1 启动命令

```bash
cd NaviAgent
# 默认端口 9000（生产）
python server_fast.py -m /path/to/multifloor.json

# 测试端口 9999
python server_fast.py -t -m /path/to/multifloor.json
```

| 参数 | 说明 |
|------|------|
| `-m/--map` | 多楼层地图配置文件路径（`multifloor.json` 等）。**强烈建议传入**，否则依赖 `MapManager` 的接口会失败。 |
| `-t/--test` | 使用测试端口 **9999**；不加则为 **9000**。 |

### 1.2 端点

| 类型 | URL | 说明 |
|------|-----|------|
| WebSocket | `ws://<host>:<port>/ws` | JSON-RPC 风格请求/响应，长连接。 |
| HTTP GET | `http://<host>:<port>/health` | 健康检查，并返回当前注册的 `methods` 列表。 |

示例：

- 本机测试：`ws://127.0.0.1:9999/ws`
- 默认生产：`ws://127.0.0.1:9000/ws`

---

## 2. WebSocket 报文协议

### 2.1 请求（客户端 → 服务端）

单条消息为一个 **JSON 对象**（文本帧）：

```json
{
  "id": "任意可选，服务端原样回显，便于异步关联",
  "method": "方法名字符串，见第4节列表",
  "params": {
    "参数名": "值"
  }
}
```

- `method`：**必填**。
- `params`：可选；缺省或 `null` 时按空对象 `{}` 处理。
- 参数通过 `**params` 传入 Python  handler，须与文档中的参数名、类型一致。

### 2.2 成功响应（服务端 → 客户端）

```json
{
  "id": "与请求一致",
  "result": { }
}
```

`result` 为业务返回值，一般为 **字典**。多数接口包含：

- `success`: `true` / `false`
- `msg`: 人类可读说明（尤其失败时）

具体字段见第 4 节。

### 2.3 错误响应

| 场景 | 示例 |
|------|------|
| 缺 `method` | `{"id": ..., "error": "missing field: method"}` |
| JSON 非法 | `{"error": "invalid json: ..."}`（可能无 `id`） |
| 未知方法 | `result` 内 `{"success": false, "msg": "未知方法: ...", ...}` |
| 参数不匹配 | `result` 内 `{"success": false, "msg": "参数不匹配: ..."}` |
| 服务端异常 | 可能下发 `{"error": "..."}`（WebSocket 异常路径） |

客户端应同时判断：**顶层 `error`**、以及 **`result.success`**。

### 2.4 并发与顺序

实现上为 **单连接内串行处理**：一问一答。若需并行，请使用多条 WebSocket 连接（需注意 ROS/机器人侧并发能力）。

### 2.5 异步方法

以下方法在服务端为 **async**，可能长时间占用连接：

- `set_goal_by_multipose`
- `wait_goal_reached`

---

## 3. HTTP：`GET /health`

**响应示例：**

```json
{
  "ok": true,
  "methods": [
    "cancel_navigation",
    "change_map",
    "get_current_map_id",
    "get_map",
    "get_nearest_point",
    "get_pose_list_in_current_submap",
    "get_portal_to_submap",
    ...
  ]
}
```

（`methods` 顺序为字母序，以实际服务为准。）

---

## 4. 方法列表与参数

> 说明：未特别注明时，`params` 可为 `{}`。所有涉及 **当前子图语义点** 的方法依赖 **已加载地图** 与 **`MapManager` 配置**。

### 4.1 导航目标

#### `set_goal`

下发命名语义点目标（Nav2 / `publish_goal`）。

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `target` | string | 是 | 预设点名称，须在 `pose_map_current_submap` 中。 |

**成功示例字段：** `success`, `msg`, `target`。

---

#### `set_goal_by_multipose`

对 **multi_pose** 类型点位，依次用 **PID** 走完全部位姿（非 Nav2 路径）。

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `target` | string | 必填 | 点名，且须为 multi_pose。 |
| `timeout_per_pose` | number | 60.0 | 单点最大耗时（秒）。 |
| `xy_tolerance` | number | 0.04 | 位置容差（米）。 |
| `yaw_tolerance` | number | 0.05 | 航向容差（弧度）。 |

**成功：** `success`, `msg`, `target`, `total_poses`, `completed_poses`, `results`（含每点 `reached`, `elapsed_time` 等）。  
**部分失败：** `success=false`，含已完成索引与 `results`。

---

#### `set_goal_by_lonlat`

经纬度 → 地图坐标 → 在目标前 `distance` 米处停车并朝向目标。

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `lon` | number | 必填 | 经度。 |
| `lat` | number | 必填 | 纬度。 |
| `distance` | number | 1.5 | 距目标点距离（米）。 |

**成功字段示例：** `success`, `msg`, `target_str`, `original_target`, `actual_target`, `distance`。

---

#### `set_goal_by_xy`

地图 `odom` 系下坐标，逻辑与 `set_goal_by_lonlat` 类似（停在目标点前 `distance` 米并朝向目标）。

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `x` | number | 必填 | X（米）。 |
| `y` | number | 必填 | Y（米）。 |
| `distance` | number | 1.5 | 距目标点距离（米）。 |

---

### 4.2 导航控制

#### `pause_navigation` / `cancel_navigation` / `resume_navigation`

无参数。用于暂停、取消、恢复（重新发上一目标）。  
返回：`success`, `msg`。

---

### 4.3 地图与点位查询

#### `get_pose_list_in_current_submap`

无参数。返回当前子图**预设点名称列表**。

**成功：** `success`, `keys`（string 数组）, `count`。

---

#### `get_submap_list`

无参数。返回所有子图 ID。

**成功：** `success`, `submaps`, `count`。

---

#### `get_submap_info`

| 参数 | 类型 | 必填 |
|------|------|------|
| `submap_id` | string | 是 |

**成功：** `success`, `submap_id`, `info`（结构由 `MapManager` 决定）。

---

#### `get_current_map_id`

无参数。根据 ROS 当前地图路径解析 **逻辑 map_id**。

**成功：** `success`, `map_id`, `map_path`, `msg`。  
**失败：** 无法读当前地图路径等。

---

#### `change_map`

切换到指定子图（含等待加载超时）。

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `map_id` | string | 必填 | 目标子图 ID。 |
| `timeout` | number | 30.0 | 等待切换超时（秒）。 |

**成功时常见字段：** `success`, `map_id`, `map_path`, `from_map_id`, `msg`；可能包含 `pose_in_source`, `pose_in_target`（跨图位姿传递）。  
（服务端在成功切换后会更新 `MapManager` 当前 submap，并可能 `publish_pose_in_target`。）

---

#### `get_portal_to_submap`

从 **当前子图** 到 **目标子图** 的传送门信息。

| 参数 | 类型 | 必填 |
|------|------|------|
| `target_submap` | string | 是 |

**成功：** `success`, `portal`, `target_submap`, `from_submap`, `portal_pose`, `msg`。  
**无法直达：** 可能 `success=false` 且带 `path` 提示经中间子图。

---

### 4.4 路线（巡逻）

#### `get_route_names`

无参数。

**成功：** `success`, `route_names`, `routes`, `count`。

---

#### `get_route`

| 参数 | 类型 | 必填 |
|------|------|------|
| `route_name` | string | 是 |

**成功：** `success`, `name`, `description`, `waypoints`, `count`。  
`waypoints` 元素可能为字典（含 `submap`, `semantic_point` 等），与工程内路线定义一致。

---

### 4.5 状态与到达判定

#### `get_status`

无参数。当前里程计位姿。

**成功：** `success`, `position` `{x,y,z}`, `orientation` 四元数。

---

#### `wait_goal_reached`

在给定时间内轮询是否到达当前目标（与 `target` 名称关联用于返回文案）。

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `target` | string | 必填 | 目标标识（日志/返回信息用）。 |
| `timeout` | number | 120.0 | 秒。 |
| `xy_tolerance` | number | 0.1 | 米。 |
| `yaw_tolerance` | number | 0.1 | 弧度。 |

**成功：** `success`, `msg`, `timestamp`。  
**超时：** `success=false`, `msg` 提示超时或未到达。

适合与 `set_goal` 等配合；短 `timeout` 可形成轮询（见测试客户端）。

---

#### `get_nearest_point`

无参数。在**当前子图**预设点中找距离当前位置最近的点。

**成功：** `success`, `nearest_point`, `distance`, `current_position`, `all_distances`, `msg`。

---

#### `get_map`

无参数。返回 **完整栅格** `data`，体积可能很大，请谨慎调用。

**成功：** `success`, `info`（分辨率、宽高、原点）, `bound`, `data`（int 列表，-1 未知 / 0 空闲 / 1–100 占用）, `msg`。  
**失败：** 尚未收到 `/map` 等。

---

## 5. 客户端示例

### 5.1 Python（`websockets`）

```python
import asyncio
import json
import websockets

async def main():
    uri = "ws://127.0.0.1:9000/ws"
    async with websockets.connect(uri) as ws:
        req = {"id": 1, "method": "get_status", "params": {}}
        await ws.send(json.dumps(req))
        res = json.loads(await ws.recv())
        print(res)  # {"id": 1, "result": {...}}

asyncio.run(main())
```

### 5.2 本仓库自带巡检测试

```bash
python test/test_client_fast.py --ws ws://127.0.0.1:9000/ws
```

环境变量：`NAV_WS_URL`、`ROUTE_NAME`。

---

## 6. 依赖与环境说明

- 服务进程内初始化 **ROS 2**（`rclpy`）及 **`NavNode`**，需在机器人或已配置 ROS 的环境中运行。
- **经纬度导航** 依赖 `WGS84Converter` 与标定。
- 未使用 `-m` 加载 `MapManager` 时，凡访问 `self.map_manager` 的接口可能异常或返回错误，请以实际部署为准。

---

## 7. 版本与文件

- 实现文件：`NaviAgent/server_fast.py`
- 方法注册：`NavigationToolManager.register_tools` 末尾 `self._handlers` 字典
- 本文档随代码维护，如有新增方法请以 `GET /health` 返回列表与源码为准。
