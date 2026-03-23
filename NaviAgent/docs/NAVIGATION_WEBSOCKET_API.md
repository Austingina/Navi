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

#### `set_goal_by_name`

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

#### `get_pose_in_submap`

| 参数 | 类型 | 必填 |
|------|------|------|
| `pose_name` | string | 是 |
| `submap_id` | string | 否 |

返回指定语义点的完整 `point_info`（包括 `type` 与 `pose` 等字段）。

`point_info` 中 `type` 字段说明与使用建议：

- `single_pose`: 单一位姿点；使用 `set_goal_by_name` 下发目标并导航到该点。
- `multi_pose`: 多位姿点；需要使用 `set_goal_by_multipose`，服务端会依次导航到该点的所有位姿。
- `elevator_call`: 电梯呼叫点；机器人到达该点后应由外部 agent 执行呼叫电梯操作（本服务仅负责导航到该位置）。
- `elevator_wait`: 电梯等待点；机器人到达该点后应等待外部 agent 确认电梯已到达且车门已打开，随后才继续下一步。

客户端使用建议：先调用 `get_pose_in_submap` 检查点位 `type`，再选择合适的导航或外部协同流程（例如：
对 `multi_pose` 调用 `set_goal_by_multipose`；对 `single_pose` 调用 `set_goal_by_name`；对电梯相关点与电梯控制 agent 协作）。


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

适合与 `set_goal_by_name` 等配合；短 `timeout` 可形成轮询（见测试客户端）。

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


### 5.2 巡检示例（按路线）

下面示例展示：先获取 `route_names`，根据 `route_name` 获取 `waypoints`，然后对每个点执行巡检：

- 若点位 `type` 为 `single_pose`，调用 `set_goal_by_name` 并使用 `wait_goal_reached` 等待到达；
- 若点位 `type` 为 `multi_pose`，调用 `set_goal_by_multipose`（该方法本身会按顺序完成内部位姿导航）。

```python
import asyncio, json, websockets

async def patrol_by_route(ws_url, route_name):
  async with websockets.connect(ws_url) as ws:
    # 获取路由信息
    await ws.send(json.dumps({"id":1, "method":"get_route", "params":{"route_name":route_name}}))
    res = json.loads(await ws.recv())['result']
    waypoints = res.get('waypoints', [])

    for wp in waypoints:
      # 假设 waypoint 包含字段 semantic_point 或 name
      point_name = wp.get('semantic_point') or wp.get('name') or wp.get('pose')
      if not point_name:
        continue

      # 查询点位完整信息
      await ws.send(json.dumps({"id":2, "method":"get_pose_in_submap", "params":{"pose_name": point_name}}))
      info = json.loads(await ws.recv())['result']
      if not info.get('success'):
        print('skip', point_name, info.get('msg'))
        continue

      point_info = info['point_info']
      ptype = point_info.get('type', 'single_pose')

      if ptype == 'multi_pose':
        # 启动 multi_pose 导航并等待其返回结果
        await ws.send(json.dumps({"id":3, "method":"set_goal_by_multipose", "params":{"target": point_name}}))
        res = json.loads(await ws.recv())['result']
        print('multi_pose result:', res)
      else:
        # single_pose / elevator_* 都使用普通下发目标，然后等待到达
        await ws.send(json.dumps({"id":4, "method":"set_goal_by_name", "params":{"target": point_name}}))
        res = json.loads(await ws.recv())['result']
        print('set_goal:', res)

        # 等待到达
        await ws.send(json.dumps({"id":5, "method":"wait_goal_reached", "params":{"target": point_name}}))
        reached = json.loads(await ws.recv())['result']
        print('reached:', reached)

asyncio.run(patrol_by_route('ws://127.0.0.1:9000/ws', '巡逻线路A'))
```

### 5.3 模拟 agent处理电梯点位（elevator_call / elevator_wait）

下面示例展示如何在巡检中与外部 agent 协作：当遇到 `elevator_call`，机器人到达后等待 agent 执行呼叫；当遇到 `elevator_wait`，机器人到达后等待 agent 确认电梯已到达并开门后继续。

示例采用简单的本地回调/等待机制来模拟 agent 响应（真实系统中可通过 HTTP、消息队列或另一路 WebSocket 与电梯控制系统通信）：

```python
import asyncio, json, websockets

async def simulate_agent_wait_event(event_name, delay=5.0):
  # 模拟外部 agent 在若干秒后完成动作
  await asyncio.sleep(delay)
  return {"event": event_name, "status": "ok"}

async def handle_point(ws, point_name, point_info):
  ptype = point_info.get('type', 'single_pose')
  if ptype == 'multi_pose':
    await ws.send(json.dumps({"id":1, "method":"set_goal_by_multipose", "params":{"target": point_name}}))
    print(await ws.recv())
  else:
    await ws.send(json.dumps({"id":2, "method":"set_goal_by_name", "params":{"target": point_name}}))
    print(await ws.recv())
    await ws.send(json.dumps({"id":3, "method":"wait_goal_reached", "params":{"target": point_name}}))
    print(await ws.recv())

  if ptype == 'elevator_call':
    print('Reached elevator_call point; instructing agent to call elevator...')
    # 模拟 agent 执行呼叫电梯
    ev = await simulate_agent_wait_event('elevator_called', delay=3.0)
    print('Agent response:', ev)

  if ptype == 'elevator_wait':
    print('Reached elevator_wait point; waiting for elevator arrival/open door...')
    ev = await simulate_agent_wait_event('elevator_arrived', delay=6.0)
    print('Agent response:', ev)

async def patrol_with_agent(ws_url, route_name):
  async with websockets.connect(ws_url) as ws:
    await ws.send(json.dumps({"id":10, "method":"get_route", "params":{"route_name":route_name}}))
    route = json.loads(await ws.recv())['result']
    for wp in route.get('waypoints', []):
      point_name = wp.get('semantic_point') or wp.get('name')
      if not point_name:
        continue
      await ws.send(json.dumps({"id":11, "method":"get_pose_in_submap", "params":{"pose_name": point_name}}))
      info = json.loads(await ws.recv())['result']
      if not info.get('success'):
        continue
      await handle_point(ws, point_name, info['point_info'])

asyncio.run(patrol_with_agent('ws://127.0.0.1:9000/ws', '巡逻线路A'))
```

### 5.4 切换楼层并继续巡检示例

当巡检路线跨楼层时，客户端需要在到达目标点所在楼层前，先导航到当前楼层到目标楼层的 `portal`，然后调用 `change_map` 切换楼层，切换成功后继续导航到目标点。下面示例展示该流程：

```python
import asyncio, json, websockets

async def patrol_cross_floor(ws_url, route_name):
  async with websockets.connect(ws_url) as ws:
    # 获取路线
    await ws.send(json.dumps({"id":1, "method":"get_route", "params":{"route_name": route_name}}))
    route = json.loads(await ws.recv())['result']
    waypoints = route.get('waypoints', [])

    # 获取当前楼层
    await ws.send(json.dumps({"id":2, "method":"get_current_map_id", "params":{}}))
    cur = json.loads(await ws.recv())['result']
    current_map = cur.get('map_id')

    for wp in waypoints:
      target_submap = wp.get('submap')
      point_name = wp.get('semantic_point') or wp.get('name') or wp.get('pose')
      if not point_name:
        continue

      # 如果目标点不在当前楼层，先导航到传送门并切换楼层
      if target_submap and target_submap != current_map:
        await ws.send(json.dumps({"id":3, "method":"get_portal_to_submap", "params":{"target_submap": target_submap}}))
        portal_resp = json.loads(await ws.recv())['result']
        portal_name = portal_resp.get('portal')
        portal_pose = portal_resp.get('portal_pose')

        if portal_name and portal_pose:
          # 导航到传送门点
          await ws.send(json.dumps({"id":4, "method":"set_goal_by_name", "params":{"target": portal_name}}))
          print('navigate to portal ->', json.loads(await ws.recv())['result'])
          await ws.send(json.dumps({"id":5, "method":"wait_goal_reached", "params":{"target": portal_name}}))
          print('portal reached ->', json.loads(await ws.recv())['result'])

        # 切换楼层
        await ws.send(json.dumps({"id":6, "method":"change_map", "params":{"map_id": target_submap}}))
        print('change_map ->', json.loads(await ws.recv())['result'])

        # 更新当前楼层信息
        await ws.send(json.dumps({"id":7, "method":"get_current_map_id", "params":{}}))
        cur = json.loads(await ws.recv())['result']
        current_map = cur.get('map_id')

      # 获取点位信息并导航（multi_pose 用 multipose 接口）
      await ws.send(json.dumps({"id":8, "method":"get_pose_in_submap", "params":{"pose_name": point_name}}))
      pinfo = json.loads(await ws.recv())['result']
      if not pinfo.get('success'):
        print('skip', point_name, pinfo.get('msg'))
        continue

      ptype = pinfo['point_info'].get('type', 'single_pose')
      if ptype == 'multi_pose':
        await ws.send(json.dumps({"id":9, "method":"set_goal_by_multipose", "params":{"target": point_name}}))
        print('multi_pose ->', json.loads(await ws.recv())['result'])
      else:
        await ws.send(json.dumps({"id":10, "method":"set_goal_by_name", "params":{"target": point_name}}))
        print('set_goal ->', json.loads(await ws.recv())['result'])
        await ws.send(json.dumps({"id":11, "method":"wait_goal_reached", "params":{"target": point_name}}))
        print('reached ->', json.loads(await ws.recv())['result'])

asyncio.run(patrol_cross_floor('ws://127.0.0.1:9000/ws', '巡逻线路A'))
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
