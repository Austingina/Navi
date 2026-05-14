# 导航服务 WebSocket 接口说明

本文档描述 `NaviSkill/server.py` 提供的 **FastAPI + WebSocket** 导航 RPC 接口，供客户端（如 `NaviSkill/test/test_client.py`）或第三方系统接入。

---

## 1. 服务启动与前缀

### 1.1 启动命令

```bash
cd NaviSkill

# 默认端口 9000（生产）
python server.py -m /path/to/multifloor.json

# 测试端口 9999
python server.py -t -m /path/to/multifloor.json

# 显式指定初始 submap（跳过 VPR 自动识别）
python server.py -m /path/to/multifloor.json -s floor

# 自定义 VPR 相机话题与设备
python server.py -m /path/to/multifloor.json \
  --image_topic /hdas/camera_head/left_raw/image_raw_color/compressed \
  --vpr_device cuda
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `-m/--map` | 无 | 多楼层地图配置文件路径（`multifloor.json` 等）。**强烈建议传入**，否则依赖 `MapManager` 的接口会失败。 |
| `-t/--test` | 关 | 使用测试端口 **9999**；不加则为 **9000**。 |
| `-s/--submap` | 无 | 显式指定初始 submap ID。**省略时**：若至少一个 submap 配置了 VPR 数据库，则启动后自动用相机图像通过 VPR 识别 submap；VPR 不可用时回退到 `default_submap`。 |
| `--image_topic` | `NavNode` 默认值 | VPR 引导/重定位使用的 **CompressedImage** 话题，默认 `/hdas/camera_head/left_raw/image_raw_color/compressed`。 |
| `--vpr_device` | `cuda` | VPR 子进程使用的设备，可选 `cuda` / `cpu`。 |

### 1.2 端点

| 类型 | URL | 说明 |
|------|-----|------|
| WebSocket | `ws://<host>:<port>/ws` | JSON-RPC 风格请求/响应，长连接。 |
| HTTP GET | `http://<host>:<port>/health` | 健康检查（不再返回 methods 列表，仅返回定位状态）。 |

示例：

- 本机测试：`ws://127.0.0.1:9999/ws`
- 默认生产：`ws://127.0.0.1:9000/ws`

### 1.3 启动流程

1. 解析 CLI 参数并加载 `MapManager`（如指定 `-m`）。
2. 初始化 `NavNode`（ROS 2），后台线程跑 `SingleThreadedExecutor`。
3. 若任意 submap 含 VPR 数据库，则启动 **VPR 子进程**（位于独立 `vpr/.venv`），等待模型 ready。
4. 若未指定 `-s/--submap` 且 VPR 可用，**用最新一帧相机图像识别初始 submap**。
5. 调用 `_change_map` 把 ROS 端地图同步到目标 submap。
6. 启动 FastAPI/WebSocket 服务及 **VPR tf_ready watchdog**（仅当 VPR 可用）。

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
- 参数通过 `**params` 传入 Python handler，须与文档中的参数名、类型一致。

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
- `get_navigation_plan`

其他方法默认在线程池中执行（`asyncio.to_thread`），不会阻塞事件循环。

---

## 3. HTTP：`GET /health`

仅返回服务状态与定位状态，不再返回方法列表。

**响应示例：**

```json
{
  "status": "ok",
  "service": "NaviSkill",
  "message": "已定位"
}
```

字段：

- `status`：固定 `"ok"`。
- `service`：固定 `"NaviSkill"`。
- `message`：`"已定位"` 或 `"未定位"`，由 `NavNode.is_map_localized()`（map→base_link TF 是否稳定）决定。

如需所有方法列表，请直接参考第 4 节或源码。

---

## 4. 方法列表与参数

> 说明：未特别注明时，`params` 可为 `{}`。所有涉及 **当前子图语义点** 的方法依赖 **已加载地图** 与 **`MapManager` 配置**。

完整方法列表（按字母序）：

```
cancel_navigation, change_map, enter_elevator, exit_elevator,
get_current_map_id, get_default_route, get_map, get_navigation_plan,
get_nearest_point, get_pose_in_submap, get_pose_list_in_current_submap,
get_portal_to_submap, get_route, get_route_names, get_status,
get_submap_info, get_submap_list, pause_navigation, resume_navigation,
set_goal_by_lonlat, set_goal_by_multipose, set_goal_by_name,
set_goal_by_xy, set_xy_goal_tolerance, wait_goal_reached
```

### 4.1 导航目标

#### `set_goal_by_name`

下发命名语义点目标（Nav2 / `publish_goal`）。

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `target` | string | 是 | 预设点名称，须在 `pose_map_current_submap` 中。 |

**自动行为：**

- 若目标点 `type` 为 `elevator_in`，下发目标时 **自动调用 `enter_elevator`**（锁定 Z 轴），在前往电梯途中即生效。

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

**成功：** `success`, `msg`, `target`, `total_poses`, `completed_poses`, `results`（含每点 `pose_index`, `position`, `reached`, `elapsed_time`）。
**部分失败：** `success=false`，含 `completed_poses`、`results`，以及 `msg` 中的失败索引。

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
| `distance` | number | 0.5 | 距目标点距离（米）。 |

---

### 4.2 导航控制

#### `pause_navigation`

无参数。当前实现等价于 **取消当前 Nav2 goal**（保留 `last_goal`，可被 `resume_navigation` 重新下发）。
返回：`success`, `msg`。

#### `cancel_navigation`

无参数。取消当前 goal。返回：`success`, `msg`。

#### `resume_navigation`

无参数。重新下发上一次的目标（`last_goal`）。返回：`success`, `msg`。

---

### 4.3 电梯模式

用于多楼层电梯场景。进入电梯模式后，FAST-LIO 会**锁定 Z 轴**位姿，避免电梯升降造成的 IMU 漂移；同时定位节点订阅 `/elevator_moving` 话题：当检测到电梯升降时，导航与 PID 精调会被暂停并保持零速，**升降过程中自动锁 XY**；升降结束（`/elevator_moving=false`）后导航会自动恢复，必要时重发 goal。

电梯模式通过 ROS 服务 **`/elevator_mode`**（`std_srvs/SetBool`）控制。

#### `enter_elevator`

进入电梯模式：锁定 Z 轴。应在机器人**进入电梯前**调用。

无参数。

**成功：** `success`, `msg`。

> **提示：** 通常不需要手动调用——`set_goal_by_name` 目标为 `elevator_in` 类型点时会**自动触发**。

---

#### `exit_elevator`

退出电梯模式：解锁 Z 轴，恢复正常 3D 里程计。应在机器人**出电梯后**调用。

无参数。

**成功：** `success`, `msg`。

> 当前 `wait_goal_reached` **不会**自动调用 `exit_elevator`，请客户端在到达 `elevator_out` 类型点位后显式调用。

---

#### `set_xy_goal_tolerance`

临时修改 Nav2 `controller_server` 的 `general_goal_checker.xy_goal_tolerance` 参数。适合电梯场景需要放大到位容差时使用。

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `tolerance` | number | 是 | 新的位置容差（米），例如 `0.3`。 |

**成功：** `success`, `msg`。

---

### 4.4 地图与点位查询

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

| type 值 | 说明 | 客户端使用方式 |
|---------|------|----------------|
| `single_pose` | 单一位姿点 | `set_goal_by_name` 下发目标并导航。 |
| `multi_pose` | 多位姿点 | `set_goal_by_multipose`，服务端依次导航到各个位姿。 |
| `LowPrecision_pose` | 低精度点位 | `set_goal_by_name` 下发；`wait_goal_reached` 会**自动放宽**到位容差（xy ≥ 0.6 m，yaw ≥ 3.14 rad）。 |
| `elevator_call` | 电梯呼叫点 | 到达后由外部 agent 执行呼叫电梯操作。 |
| `elevator_wait` | 电梯等待点 | 到达后等待 agent 确认电梯已到达且门已打开。 |
| `elevator_in` | 进梯点 | `set_goal_by_name` 下发时 **自动 `enter_elevator`**（锁 Z）；`wait_goal_reached` **自动放宽**到位容差。 |
| `elevator_out` | 出梯点 | 到达后客户端**应显式调用 `exit_elevator`**（解锁 Z）。 |

客户端使用建议：先调用 `get_pose_in_submap` 检查点位 `type`，再选择合适的导航或外部协同流程。

---

#### `get_current_map_id`

无参数。根据 ROS 当前地图路径解析 **逻辑 map_id**；ROS 参数服务暂不可用时回退到 `MapManager` 缓存。

**成功：** `success`, `map_id`, `map_path`, `msg`。
**失败：** 无法读当前地图路径且无缓存。

---

#### `change_map`

切换到指定子图（含等待加载超时）。当前已定位且能确定源 `from_map_id` 时，会调用 `MapManager.get_init_pose_after_changemap` 计算目标 submap 中的初始位姿，并在切换完成后发布到 `/pose_in_target`（强制 `z=0`）。

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `map_id` | string | 必填 | 目标子图 ID。 |
| `timeout` | number | 60.0 | 等待 ROS 端切到目标地图的超时（秒）。 |

**成功时常见字段：** `success`, `map_id`, `map_path`, `from_map_id`, `msg`；当切换前已定位时还会包含 `pose_in_source` 与 `pose_in_target`。

> 如果目标地图与当前 ROS 端地图相同，会直接返回 `success=true` 并同步 `MapManager` 缓存，不会重新下发 change_map 请求。

---

#### `get_portal_to_submap`

从 **当前子图** 到 **目标子图** 的传送门信息。当存在多条可直达的 portal 时，若机器人当前位姿可用，则**按 XY 距离选最近的一条**。

| 参数 | 类型 | 必填 |
|------|------|------|
| `target_submap` | string | 是 |

**成功：** `success`, `portal`, `target_submap`, `from_submap`, `portal_pose`, `portal_candidates`（所有候选 portal 名称列表）, `selected_by_nearest`（bool，是否按距离选取）, `msg`。
**无法直达：** `success=false` 且带提示信息。

---

### 4.5 路线（巡逻）

#### `get_route_names`

无参数。

**成功：** `success`, `route_names`, `routes`, `count`。

---

#### `get_default_route`

无参数。返回地图配置中 `global.default_route` 字段。

**成功：** `success`, `default_route`。
**失败：** `success=false`, `msg`（未配置 `default_route`）。

---

#### `get_route`

| 参数 | 类型 | 必填 |
|------|------|------|
| `route_name` | string | 是 |

**成功：** `success`, `name`, `description`, `waypoints`, `count`。
`waypoints` 元素可能为字典（含 `submap`, `semantic_point` 等），与工程内路线定义一致。

---

### 4.6 状态与到达判定

#### `get_status`

无参数。当前里程计位姿及定位状态。

**成功：** `success`, `localized`（bool, map→base_link TF 是否稳定可用）, `position` `{x,y,z}`, `orientation` 四元数。

---

#### `wait_goal_reached`

在给定时间内轮询是否到达当前目标（与 `target` 名称关联用于返回文案）。

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `target` | string | 必填 | 目标标识（日志/返回信息用）。 |
| `timeout` | number | 120.0 | 秒。 |
| `xy_tolerance` | number | 0.1 | 米。 |
| `yaw_tolerance` | number | 0.1 | 弧度。 |

**自动放宽容差：** 若 `target` 在当前子图中类型为 `elevator_in` 或 `LowPrecision_pose`，服务端会强制：
`xy_tolerance = max(xy_tolerance, 0.6)`，`yaw_tolerance = max(yaw_tolerance, 3.14)`。

**成功：** `success`, `msg`, `timestamp`。
**超时：** `success=false`, `msg` 提示超时或未到达, `nav_failed=false`。
**导航失败：** `success=false`, `msg` 含失败原因, `nav_failed=true`。

适合与 `set_goal_by_name` 等配合；短 `timeout` 可形成轮询（见测试客户端）。

---

#### `get_navigation_plan`

获取 Nav2 当前全局路径（`/plan` 话题）。每次下发新目标后服务端会清空缓存并递增代次，只保留与当前目标对应的新路径。

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `timeout` | number | 10.0 | 等待非空路径的最长时间（秒）。 |

**成功：** `success`, `plan`（含 `frame_id`, `stamp`, `poses`, `pose_count`）, `msg`。
**超时：** `success=false`，提示未收到 `/plan`。

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

## 5. VPR 自动重定位

服务集成 **VPR (Visual Place Recognition)** 子进程，用于跨楼层/重定位场景下自动判别当前所在 submap。

### 5.1 启用条件

- `MapManager` 已加载，且 **至少一个 submap 配置了 VPR 数据库**。
- VPR 子进程独立运行于 `vpr/.venv`，启动时会阻塞至模型 ready。

### 5.2 启动引导

- 若启动时**未指定 `-s/--submap`** 且 VPR 可用：抓取最新一帧 CompressedImage（最多等 10 s），调用 VPR 识别 submap，然后通过 `_change_map` 同步 ROS 端地图。
- 识别失败时回退到 `MapManager.get_current_submap()`（即 `default_submap`）。

### 5.3 运行时 watchdog

服务启动后会运行 `_tf_localization_watchdog` 协程：

| 参数 | 取值 | 说明 |
|------|------|------|
| `check_interval_sec` | 2.0 s | 检查 `tf_ready` 频率 |
| `grace_sec` | 10.0 s | 定位丢失超过该时长才视为真的丢失 |
| `cooldown_sec` | 30.0 s | 两次 VPR 重定位尝试之间的最小间隔 |

当 `is_map_localized()` 持续为 `False` 超过 grace 时间，watchdog 会抓取最新相机图像 → 调 VPR → 若识别到的 submap 与当前不同则触发 `_change_map`，从而恢复定位。

### 5.4 失败行为

- VPR 子进程未启动（无数据库）或不可用：跳过引导与 watchdog，沿用 `default_submap`。
- VPR 查询失败/超时：日志告警，本次跳过；watchdog 在冷却时间后再次尝试。

---

## 6. 客户端示例

### 6.1 Python（`websockets`）

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


### 6.2 巡检示例（按路线）

下面示例展示：先获取 `route_names`，根据 `route_name` 获取 `waypoints`，然后对每个点执行巡检：

- 若点位 `type` 为 `single_pose` / `LowPrecision_pose`，调用 `set_goal_by_name` 并使用 `wait_goal_reached` 等待到达；
- 若点位 `type` 为 `multi_pose`，调用 `set_goal_by_multipose`（该方法本身会按顺序完成内部位姿导航）。

```python
import asyncio, json, websockets

async def patrol_by_route(ws_url, route_name):
  async with websockets.connect(ws_url) as ws:
    await ws.send(json.dumps({"id":1, "method":"get_route", "params":{"route_name":route_name}}))
    res = json.loads(await ws.recv())['result']
    waypoints = res.get('waypoints', [])

    for wp in waypoints:
      point_name = wp.get('semantic_point') or wp.get('name') or wp.get('pose')
      if not point_name:
        continue

      await ws.send(json.dumps({"id":2, "method":"get_pose_in_submap", "params":{"pose_name": point_name}}))
      info = json.loads(await ws.recv())['result']
      if not info.get('success'):
        print('skip', point_name, info.get('msg'))
        continue

      point_info = info['point_info']
      ptype = point_info.get('type', 'single_pose')

      if ptype == 'multi_pose':
        await ws.send(json.dumps({"id":3, "method":"set_goal_by_multipose", "params":{"target": point_name}}))
        res = json.loads(await ws.recv())['result']
        print('multi_pose result:', res)
      else:
        await ws.send(json.dumps({"id":4, "method":"set_goal_by_name", "params":{"target": point_name}}))
        res = json.loads(await ws.recv())['result']
        print('set_goal:', res)

        await ws.send(json.dumps({"id":5, "method":"wait_goal_reached", "params":{"target": point_name}}))
        reached = json.loads(await ws.recv())['result']
        print('reached:', reached)

asyncio.run(patrol_by_route('ws://127.0.0.1:9000/ws', '巡逻线路A'))
```

### 6.3 跨楼层电梯巡检完整流程

当巡检路线跨楼层且涉及电梯时，完整的工作流如下：

1. 在源楼层导航到 `elevator_call` 点 → agent 呼叫电梯；
2. 导航到 `elevator_wait` 点 → agent 确认电梯到达、门已开；
3. 导航到 `elevator_in` 点（自动 `enter_elevator`，锁定 Z；`wait_goal_reached` 自动放宽容差）；
4. 电梯运行中，FAST-LIO 通过 `/elevator_moving` 通告升降状态，导航与 PID 精调会被自动暂停并保持零速；
5. 电梯到达目标楼层后，调用 `change_map` 切换子图（VPR watchdog 也可在丢失定位时辅助识别）；
6. 导航到 `elevator_out` 点 → **客户端显式调用 `exit_elevator`** 解锁 Z 轴；
7. 在目标楼层继续后续巡检。

```python
import asyncio, json, websockets

async def rpc(ws, method, params=None, req_id=1):
    await ws.send(json.dumps({"id": req_id, "method": method, "params": params or {}}))
    return json.loads(await ws.recv())['result']

async def simulate_agent_wait(event_name, delay=5.0):
    await asyncio.sleep(delay)
    return {"event": event_name, "status": "ok"}

async def patrol_with_elevator(ws_url, route_name):
  async with websockets.connect(ws_url) as ws:
    route = await rpc(ws, "get_route", {"route_name": route_name})
    cur = await rpc(ws, "get_current_map_id")
    current_map = cur.get('map_id')

    for wp in route.get('waypoints', []):
      target_submap = wp.get('submap')
      point_name = wp.get('semantic_point') or wp.get('name')
      if not point_name:
        continue

      # 需要切换楼层时，先导航到 portal
      if target_submap and target_submap != current_map:
        portal = await rpc(ws, "get_portal_to_submap", {"target_submap": target_submap})
        if portal.get('success') and portal.get('portal'):
          await rpc(ws, "set_goal_by_name", {"target": portal['portal']})
          await rpc(ws, "wait_goal_reached", {"target": portal['portal']})
        await rpc(ws, "change_map", {"map_id": target_submap})
        cur = await rpc(ws, "get_current_map_id")
        current_map = cur.get('map_id')

      pinfo = await rpc(ws, "get_pose_in_submap", {"pose_name": point_name})
      if not pinfo.get('success'):
        print('skip', point_name)
        continue

      ptype = pinfo['point_info'].get('type', 'single_pose')

      if ptype == 'multi_pose':
        print(await rpc(ws, "set_goal_by_multipose", {"target": point_name}))
      else:
        print(await rpc(ws, "set_goal_by_name", {"target": point_name}))
        print(await rpc(ws, "wait_goal_reached", {"target": point_name}))

      # 电梯协同
      if ptype == 'elevator_call':
        print('Agent 呼叫电梯...')
        await simulate_agent_wait('elevator_called', delay=3.0)
      elif ptype == 'elevator_wait':
        print('等待电梯到达并开门...')
        await simulate_agent_wait('elevator_arrived', delay=6.0)
      elif ptype == 'elevator_out':
        # 客户端必须显式退出电梯模式
        print(await rpc(ws, "exit_elevator"))

asyncio.run(patrol_with_elevator('ws://127.0.0.1:9000/ws', '跨楼层巡逻'))
```

### 6.4 切换楼层并继续巡检示例

当巡检路线跨楼层时，客户端需要在到达目标点所在楼层前，先导航到当前楼层到目标楼层的 `portal`，然后调用 `change_map` 切换楼层，切换成功后继续导航到目标点。下面示例展示该流程：

```python
import asyncio, json, websockets

async def patrol_cross_floor(ws_url, route_name):
  async with websockets.connect(ws_url) as ws:
    await ws.send(json.dumps({"id":1, "method":"get_route", "params":{"route_name": route_name}}))
    route = json.loads(await ws.recv())['result']
    waypoints = route.get('waypoints', [])

    await ws.send(json.dumps({"id":2, "method":"get_current_map_id", "params":{}}))
    cur = json.loads(await ws.recv())['result']
    current_map = cur.get('map_id')

    for wp in waypoints:
      target_submap = wp.get('submap')
      point_name = wp.get('semantic_point') or wp.get('name') or wp.get('pose')
      if not point_name:
        continue

      if target_submap and target_submap != current_map:
        await ws.send(json.dumps({"id":3, "method":"get_portal_to_submap", "params":{"target_submap": target_submap}}))
        portal_resp = json.loads(await ws.recv())['result']
        portal_name = portal_resp.get('portal')
        portal_pose = portal_resp.get('portal_pose')

        if portal_name and portal_pose:
          await ws.send(json.dumps({"id":4, "method":"set_goal_by_name", "params":{"target": portal_name}}))
          print('navigate to portal ->', json.loads(await ws.recv())['result'])
          await ws.send(json.dumps({"id":5, "method":"wait_goal_reached", "params":{"target": portal_name}}))
          print('portal reached ->', json.loads(await ws.recv())['result'])

        await ws.send(json.dumps({"id":6, "method":"change_map", "params":{"map_id": target_submap}}))
        print('change_map ->', json.loads(await ws.recv())['result'])

        await ws.send(json.dumps({"id":7, "method":"get_current_map_id", "params":{}}))
        cur = json.loads(await ws.recv())['result']
        current_map = cur.get('map_id')

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


### 6.5 导航暂停 / 继续 / 取消 示例

说明：在执行导航任务时，客户端可能需要临时暂停当前导航、取消目标或在暂停后继续导航。服务端提供：`pause_navigation`、`cancel_navigation`、`resume_navigation`。注意 `pause_navigation` 实际上会取消当前 Nav2 goal，但保留 `last_goal`，可被 `resume_navigation` 重新下发。

```python
import asyncio, json, websockets

async def example_pause_resume(ws_url, point_name):
  async with websockets.connect(ws_url) as ws:
    await ws.send(json.dumps({"id":1, "method":"set_goal_by_name", "params":{"target": point_name}}))
    print('set_goal:', json.loads(await ws.recv()))

    await asyncio.sleep(2)
    await ws.send(json.dumps({"id":2, "method":"pause_navigation", "params":{}}))
    print('pause:', json.loads(await ws.recv()))

    await asyncio.sleep(3)
    await ws.send(json.dumps({"id":3, "method":"resume_navigation", "params":{}}))
    print('resume:', json.loads(await ws.recv()))

    await ws.send(json.dumps({"id":4, "method":"wait_goal_reached", "params":{"target": point_name}}))
    print('reached:', json.loads(await ws.recv()))

async def example_cancel(ws_url, point_name):
  async with websockets.connect(ws_url) as ws:
    await ws.send(json.dumps({"id":10, "method":"set_goal_by_name", "params":{"target": point_name}}))
    print('set_goal:', json.loads(await ws.recv()))
    await ws.send(json.dumps({"id":11, "method":"cancel_navigation", "params":{}}))
    print('cancel:', json.loads(await ws.recv()))

asyncio.run(example_pause_resume('ws://127.0.0.1:9000/ws', 'PointA'))
```

环境变量（`test_client.py`）：`NAV_WS_URL`、`ROUTE_NAME`。

---

## 7. 依赖与环境说明

- 服务进程内初始化 **ROS 2**（`rclpy`）及 **`NavNode`**，需在机器人或已配置 ROS 的环境中运行。
- **经纬度导航** 依赖 `WGS84Converter` 与标定。
- 未使用 `-m` 加载 `MapManager` 时，凡访问 `self.map_manager` 的接口可能异常或返回错误，请以实际部署为准。
- **电梯模式** 依赖 FAST-LIO 提供的 `/elevator_mode` 服务（`std_srvs/SetBool`）以及 `/elevator_moving` 话题（`std_msgs/Bool`），需确保定位节点已启动。
- **VPR 引导/重定位** 依赖：
  - 至少一个 submap 配置了 VPR 数据库
  - `vpr/.venv` 中可用的 VPR 子进程
  - 相机 CompressedImage 话题（默认 `/hdas/camera_head/left_raw/image_raw_color/compressed`，可由 `--image_topic` 指定）

---

## 8. 版本与文件

- 实现文件：`NaviSkill/server.py`
- 方法注册：`NavigationToolManager.register_tools` 末尾的 `self._handlers` 字典
- VPR 引导/重定位：`NaviSkill/vpr_bootstrap.py`、`NavigationToolManager._vpr_*` / `_tf_localization_watchdog`
- 本文档随代码维护，如有新增方法请以源码 `self._handlers` 字典为准。
- 最后更新：2026-05-14
