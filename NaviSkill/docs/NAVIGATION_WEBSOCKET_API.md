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

# SLAM 室内模式：指定初始 slam_map/submap，并拉起 start_slam.sh
python server.py -m /path/to/scene.json --ss zhichengA2A3/1f

# 室外 GPS 模式（省略 --ss）：拉起 start_outdoor.sh
python server.py -m /path/to/scene.json

# 自定义 VPR 相机话题与设备
python server.py -m /path/to/scene.json --ss default/2f \
  --image_topic /hdas/camera_head/left_raw/image_raw_color/compressed \
  --vpr_device cuda

# 附带 rerun 可视化
python server.py -m /path/to/scene.json --ss caihong/2f -v
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `-m/--map` | 无 | 地图配置来源：本地 `*.json` 路径，或 mapserver URL（会缓存到 `mapdata/<id>/`）。**强烈建议传入**。 |
| `-t/--test` | 关 | 使用测试端口 **9999**；不加则为 **9000**。 |
| `--ss` | 无 | 初始定位目标，格式 **`slam_map_id/submap_id`**（如 `caihong/2f`）。**给出 → SLAM 室内模式**，解析 submap 的 pcd 并拉起 `start_slam.sh`；**省略 → 室外 GPS 模式**，拉起 `start_outdoor.sh`。（与仿真常用的 `-s` use_sim_time 不同，见 `start_mcp.sh` 注释。） |
| `--image_topic` | `NavNode` 默认值 | VPR 引导/重定位使用的 **CompressedImage** 话题，默认 `/hdas/camera_head/left_raw/image_raw_color/compressed`。 |
| `--vpr_device` | `cuda` | VPR 子进程使用的设备，可选 `cuda` / `cpu`。 |
| `-v/--viz` | 关 | 启动 rerun 可视化（依赖 `rerun-sdk`）。 |
| `--viz_lidar_topic` | `/cloud_registered` | 可视化 LiDAR 话题；也可设为 `/livox/lidar`（需 lidar→base_link 外参）。 |
| `--viz_mode` | `spawn` | `spawn` / `connect` / `serve` / `save`。 |
| `--viz_addr` | 依 mode | connect 默认 `127.0.0.1:9876`；serve 默认 `0.0.0.0:9876`。 |
| `--viz_save_path` | `/tmp/navi_rerun.rrd` | `viz_mode=save` 时的输出路径。 |

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
2. 依据 `--ss` 调用 `_launch_nav_stack`：**有 `--ss` → SLAM 栈**；**无 `--ss` → 室外 GPS 栈**。
3. 初始化 `NavNode`（ROS 2，`initial_nav_mode` 与上一步一致），后台 `MultiThreadedExecutor` 跑 spin。
4. 若地图含 VPR 数据库，启动 **VPR 子进程**（独立 `vpr/.venv`），等待模型 ready。
5. **仅 SLAM 模式**（给出 `--ss`）：MapManager 已设为对应 `slam_map/submap`（不再走 VPR 启动引导，因 submap 已显式指定），随后 `_sync_map_state` 与 ROS 端对齐。
6. **室外 GPS 模式**（省略 `--ss`）：跳过 submap 识别与 `_change_map`。
7. 可选 `-v` 启动 `NaviRerunViz`。
8. 启动 FastAPI/WebSocket；若 VPR 可用则启动 **tf_ready watchdog**（见 §5.3）。

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

### 方法分类

| 类别 | 方法 |
|------|------|
| **导航动作类** | `set_goal_by_slam`, `set_goal_by_outdoor`, `pause_navigation`, `cancel_navigation`, `resume_navigation`, `set_xy_goal_tolerance` |
| **导航模式类** | `set_nav_mode`, `get_nav_mode` |
| **导航状态获取** | `get_map`, `get_navigation_plan`, `wait_goal_reached`, `get_status`, `get_nearest_point` |
| **地图数据获取** | `get_pose_list_in_current_submap`, `get_pose_in_submap`, `get_submap_list`, `get_submap_info`, `get_current_map_id` |
| **路线数据获取** | `get_route_names`, `get_default_route`, `get_route` |

---

### 4.1 导航动作类

#### `set_goal_by_slam`

SLAM 模式下发目标（自动确保处于 `slam` 模式）。两种用法：

1. **命名语义点**：从地图取该点位姿下发（Nav2 / `publish_goal`）。
2. **临时坐标点**（`target == "tmp"`）：不查地图，直接以 `x`/`y` 为目标，朝向取「当前位置指向目标」的接近方向；到达精度由 `xy_tolerance`/`yaw_tolerance` 控制。

| 参数 | 类型 | 必填 | 默认 | 说明 |
|------|------|------|------|------|
| `target` | string | 是 | — | 预设点名称（须在 `pose_map_current_submap` 中）；传 `"tmp"` 表示用 `x`/`y` 临时坐标。 |
| `x` | number | `target=="tmp"` 时必填 | — | 地图(odom)系 X（米）。 |
| `y` | number | `target=="tmp"` 时必填 | — | 地图(odom)系 Y（米）。 |
| `slam_map` | string | 否 | — | 仅命名点位；目标所在 SLAM 地图 id，与当前不同会自动切换。 |
| `submap` | string | 否 | — | 仅命名点位；目标所在 submap id，与当前不同会自动换图。 |
| `xy_tolerance` | number | 否 | 0.1 | 到达判定位置容差（米）；随目标记录，供 `wait_goal_reached` 取回。 |
| `yaw_tolerance` | number | 否 | 0.1 | 到达判定朝向容差（弧度）；同上。 |

**自动行为：**

- 若命名点位 `type` 为 `elevator_in`，下发目标时 **自动锁定 Z 轴**（内部调用 `/elevator_mode`，无对外 RPC），在前往电梯途中即生效。
- 若命名点位 `type` 为 `elevator_in` / `LowPrecision_pose`，下发时**自动放宽**容差（`xy_tolerance ≥ 0.6`，`yaw_tolerance ≥ 3.14`）。
- 出梯后随即换图（换 submap/切栈）时，Z 锁状态会被**自动重置**，无需单独解锁。

**成功字段：** 命名点位 `success`, `msg`, `target`, `mode_switch`, `xy_tolerance`, `yaw_tolerance`；`tmp` 另含 `target_str`, `target_point`。

---

#### `set_goal_by_outdoor`

室外 GPS 模式下发目标（按距离选择 Nav2 直达或 campus 路网规划；自动切到 `gps` 模式）。两种用法：

1. **命名场景语义点**：经纬度**始终**从场景语义点 `target` 读取（忽略入参 `lon`/`lat`）。
2. **临时坐标点**（`target == "tmp"`）：不查地图，直接以入参 `lon`/`lat` 为目标。

| 参数 | 类型 | 必填 | 默认 | 说明 |
|------|------|------|------|------|
| `target` | string | 是 | — | 目标语义点名称；传 `"tmp"` 表示用 `lon`/`lat` 临时坐标。随目标记录，供 `wait_goal_reached` 判定到达。 |
| `lon` | number | `target=="tmp"` 时必填 | — | 目标经度；仅 `target=="tmp"` 时使用。 |
| `lat` | number | `target=="tmp"` 时必填 | — | 目标纬度；仅 `target=="tmp"` 时使用。 |
| `tolerance` | number | 否 | **10.0** | 到达判定距离阈值（米）；下发时记录，供 `wait_goal_reached` 取回。 |
| `direct_threshold` | number | 否 | **150.0** | 与机器人直线距离小于该值（米）时走 Nav2 直达 `/goal_pose`；否则走 campus 路网规划器。 |

**成功字段：** `success`, `mode`（`gps`）, `channel`（`nav2`/`campus`）, `distance`, `mode_switch`, `target`, `goal`（`lon`/`lat`）, `tolerance`, `msg`。

> **跨模式 / 换图（内部自动，无独立 RPC）**
>
> - **换图**：`set_goal_by_slam` 在已处于 slam 模式且目标 `submap` 与当前不一致时，由 `_ensure_submap` 先导航到传送门、到达后 `_change_map`（无 `change_map` / `get_portal_to_submap` RPC）。
> - **跨模式**：`_ensure_nav_mode` 在当前模式下先导航到场景传送门，到达后再重启目标栈（GPS↔SLAM）。失败时不切换，返回 `mode_switch`。
> - 客户端只需在下发目标时携带 `slam_map`/`submap`，或调用对应的 `set_goal_*`。

---

#### `pause_navigation`

无参数。当前实现等价于 **取消当前 Nav2 goal**（保留 `last_goal`，可被 `resume_navigation` 重新下发）。
返回：`success`, `msg`。

#### `cancel_navigation`

无参数。取消当前 goal。返回：`success`, `msg`。

#### `resume_navigation`

无参数。重新下发上一次的目标（`last_goal`）。返回：`success`, `msg`。

#### `set_xy_goal_tolerance`

临时修改 Nav2 `controller_server` 的 `general_goal_checker.xy_goal_tolerance` 参数。适合需要放大 Nav2 到位容差时使用（与 `set_goal_by_slam` 记录的到达判定容差相互独立）。

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `tolerance` | number | 是 | 新的位置容差（米），例如 `0.3`。 |

**成功：** `success`, `msg`。

---

### 4.2 导航模式类

#### `get_nav_mode`

无参数。返回当前导航模式。

**成功：** `success`, `nav_mode`（`slam` 或 `gps`）。

#### `set_nav_mode`

手动切换导航模式（会停当前目标、复位定位判据）。**不会**自动走传送门衔接；跨模式切换请用 `set_goal_by_slam` / `set_goal_by_outdoor`（内部 `_ensure_nav_mode`）。

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `mode` | string | 是 | `slam` 或 `gps`。 |

**成功：** `success`, `nav_mode`, `msg`。

---

### 4.3 导航状态获取

#### `get_map`

无参数。返回 **完整栅格** `data`，体积可能很大，请谨慎调用。

**成功：** `success`, `info`（分辨率、宽高、原点）, `bound`, `data`（int 列表，-1 未知 / 0 空闲 / 1–100 占用）, `msg`。
**失败：** 尚未收到 `/map` 等。

#### `get_navigation_plan`

获取 Nav2 当前全局路径（`/plan` 话题）。每次下发新目标后服务端会清空缓存并递增代次，只保留与当前目标对应的新路径。

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `timeout` | number | 10.0 | 等待非空路径的最长时间（秒）。 |

**成功：** `success`, `plan`（含 `frame_id`, `stamp`, `poses`, `pose_count`）, `msg`。
**超时：** `success=false`，提示未收到 `/plan`。

#### `wait_goal_reached`

在给定时间内轮询是否到达当前目标（与 `target` 名称关联用于返回文案）。

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `target` | string | 必填 | 目标标识（日志/返回信息用）。 |
| `timeout` | number | 120.0 | 秒。 |

**到达容差不在此传入**：SLAM 由 `set_goal_by_slam(xy_tolerance=…, yaw_tolerance=…)`、室外由 `set_goal_by_outdoor(tolerance=…)` 在**下发目标时**确定并记录，本接口取回使用。`elevator_in` / `LowPrecision_pose` 的容差放宽也已移至 `set_goal_by_slam` 下发时处理。

**成功：** `success`, `msg`, `timestamp`。
**超时：** `success=false`, `msg` 提示超时或未到达, `nav_failed=false`。
**导航失败：** `success=false`, `msg` 含失败原因, `nav_failed=true`。

适合与 `set_goal_by_slam` 等配合；短 `timeout` 可形成轮询（见测试客户端）。

#### `get_status`

无参数。当前位姿及按**当前导航模式**判定的定位状态。

**成功：** `success`, `nav_mode`（`slam`|`gps`）, `localized`（按模式综合判定）, `map_localized`, `gps_available`, `slam_map`, `position`, `orientation`；GPS 模式下另含 `gps_fix`。
**无位姿时：** `success=false`, `msg`, 仍含 `nav_mode`, `localized`, `map_localized`, `gps_available`。

#### `get_nearest_point`

无参数。在**当前子图**预设点中找距离当前位置最近的点。

**成功：** `success`, `nearest_point`, `distance`, `current_position`, `all_distances`, `msg`。

---

### 4.4 地图数据获取

#### `get_pose_list_in_current_submap`

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `slam_map` | string | 否 | 指定 SLAM 地图 id；默认当前。 |
| `submap_id` | string | 否 | 指定 submap id；默认当前。 |

**成功：** `success`, `keys`, `count`, `slam_map`, `submap`。

#### `get_pose_in_submap`

| 参数 | 类型 | 必填 |
|------|------|------|
| `pose_name` | string | 是 |
| `submap_id` | string | 否 |
| `slam_map` | string | 否 |

**成功：** `success`, `pose_name`, `point_info`。

`point_info` 中 `type` 字段说明与使用建议：

| type 值 | 说明 | 客户端使用方式 |
|---------|------|----------------|
| `single_pose` | 单一位姿点 | `set_goal_by_slam` 下发目标并导航。 |
| `multi_pose` | 多位姿点 | 当前无专用下发接口（一串按顺序经过的位姿）。 |
| `LowPrecision_pose` | 低精度点位 | `set_goal_by_slam` 下发时会**自动放宽**到位容差（xy ≥ 0.6 m，yaw ≥ 3.14 rad）。 |
| `elevator_call` | 电梯呼叫点 | 到达后由外部 agent 执行呼叫电梯操作。 |
| `elevator_wait` | 电梯等待点 | 到达后等待 agent 确认电梯已到达且门已打开。 |
| `elevator_in` | 进梯点 | `set_goal_by_slam` 下发时 **自动锁 Z**（内部处理）并**自动放宽**到位容差。 |
| `elevator_out` | 出梯点 | 出梯后随即换图，Z 锁状态自动重置，无需解锁。 |
| `door_open` | 开门点 | 到达后由外部 agent 执行开门动作。 |

客户端使用建议：先调用 `get_pose_in_submap` 检查点位 `type`，再选择合适的导航或外部协同流程。

#### `get_submap_list`

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `slam_map` | string | 否 | 指定 SLAM 地图 id；默认当前。 |

**成功：** `success`, `submaps`, `count`, `slam_map`, `slam_maps`（全部 SLAM 地图 id 列表）。

#### `get_submap_info`

| 参数 | 类型 | 必填 |
|------|------|------|
| `submap_id` | string | 是 |
| `slam_map` | string | 否 |

**成功：** `success`, `submap_id`, `slam_map`, `info`。

#### `get_current_map_id`

无参数。根据 ROS 当前地图路径解析 **逻辑 submap id**；GPS 模式下无 SLAM 点云地图。

**成功（SLAM）：** `success`, `nav_mode`（`slam`）, `map_id`, `slam_map`, `map_path`, `msg`。
**成功（GPS）：** `success`, `nav_mode`（`gps`）, `map_id`/`slam_map` 均为 `null`, `map_path` 为空字符串, `msg` 说明无 SLAM 地图。
**失败：** 无法读当前地图路径且无 MapManager 缓存。

---

### 4.5 路线数据获取

#### `get_route_names`

无参数。

**成功：** `success`, `route_names`, `routes`（每项含 `scope`: `global`|`slam`）, `global_routes`, `slam_routes`, `count`。

#### `get_default_route`

无参数。返回地图配置中 `global.default_route` 字段。

**成功：** `success`, `default_route`。
**失败：** `success=false`, `msg`（未配置 `default_route`）。

#### `get_route`

| 参数 | 类型 | 必填 |
|------|------|------|
| `route_name` | string | 是 |

**成功：** `success`, `scope`（`global`|`slam`）, `name`, `description`, `waypoints`, `count`。

`waypoints` 采用**统一定义**，每个元素含 `name` 与 `type`，**`type` 唯一确定导航模式**：

- `type == "slam"`：某 SLAM 地图 submap 内的语义点 —— `{name, type:"slam", slam_map, submap}`（室内）；
- `type == "gps"`：场景语义点 / 经纬度点 —— `{name, type:"gps", area, lon, lat, heading_deg}`（室外）；
- `type == "instruction"`：VLN 指令 —— `{name:null, type:"instruction", instruction}`。

客户端按 `type` 顺序路由即可：`gps` → `set_goal_by_outdoor(target=name)`；
`slam` → `set_goal_by_slam(target=name, submap=submap)`。下发目标时服务端会按需自动
切换导航模式（`_ensure_nav_mode`/`_ensure_submap`），因此**不再返回 `nav_mode` / `segments`**，
`get_route_plan` RPC 也已移除。

---

## 5. VPR 自动重定位

服务集成 **VPR (Visual Place Recognition)** 子进程，用于跨楼层/重定位场景下自动判别当前所在 submap。

### 5.1 启用条件

- `MapManager` 已加载，且 **至少一个 submap 配置了 VPR 数据库**。
- VPR 子进程独立运行于 `vpr/.venv`，启动时会阻塞至模型 ready。

### 5.2 启动引导（SLAM 模式）

- 给出 **`--ss slam_map/submap`** 时：submap 已显式指定，**跳过** VPR 启动引导，直接 `_sync_map_state` 与 ROS 对齐。
- **室外 GPS 模式**（省略 `--ss`）：不做 submap 引导与换图。

### 5.3 运行时 watchdog

服务启动后，若 VPR 可用，运行 `_tf_localization_watchdog` 协程（基于 `is_map_localized()`，即 map→base_link TF）：

| 参数 | 取值 | 说明 |
|------|------|------|
| `check_interval_sec` | 2.0 s | 检查频率 |
| `grace_sec` | **150.0 s** | 定位丢失持续超过该时长才触发 VPR 重定位 |
| `fail_cooldown_sec` | 30.0 s | 两次 VPR **尝试**之间的最小间隔 |
| `success_cooldown_sec` | 60.0 s | VPR **成功**后的静默期（此期间即使仍未定位也不再触发） |

当 `is_map_localized()` 持续为 `False` 超过 `grace_sec`，watchdog 抓取相机图像 → VPR 共识 → 若 submap 变化则 `_change_map`。

### 5.4 失败行为

- VPR 子进程未启动（无数据库）或不可用：跳过 watchdog；SLAM 启动时若未指定 `--ss` 且 VPR 不可用，使用 MapManager 当前 submap。
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

下面示例展示：先 `get_route`，再按 waypoint 的 **`type`** 路由（`gps` → `set_goal_by_outdoor`；`slam` → `set_goal_by_slam`），并用 `wait_goal_reached` 等待到达。

```python
import asyncio, json, websockets

async def patrol_by_route(ws_url, route_name):
  async with websockets.connect(ws_url) as ws:
    await ws.send(json.dumps({"id":1, "method":"get_route", "params":{"route_name":route_name}}))
    res = json.loads(await ws.recv())['result']
    waypoints = res.get('waypoints', [])

    for wp in waypoints:
      wp_type = wp.get('type', 'slam')
      if wp_type == 'instruction':
        print('skip instruction:', wp)
        continue
      if wp_type == 'gps':
        target = wp.get('name') or 'tmp'
        params = {"target": target}
        if target == 'tmp':
          params.update({"lon": wp['lon'], "lat": wp['lat']})
        await ws.send(json.dumps({"id":2, "method":"set_goal_by_outdoor", "params": params}))
        print('outdoor:', json.loads(await ws.recv())['result'])
        await ws.send(json.dumps({"id":3, "method":"wait_goal_reached", "params":{"target": target, "timeout": 600}}))
        print('reached:', json.loads(await ws.recv())['result'])
        continue

      point_name = wp.get('name')
      submap = wp.get('submap')
      slam_map = wp.get('slam_map')
      params = {"target": point_name, "xy_tolerance": 0.1, "yaw_tolerance": 0.1}
      if submap:
        params["submap"] = submap
      if slam_map:
        params["slam_map"] = slam_map
      await ws.send(json.dumps({"id":4, "method":"set_goal_by_slam", "params": params}))
      print('slam:', json.loads(await ws.recv())['result'])
      await ws.send(json.dumps({"id":5, "method":"wait_goal_reached", "params":{"target": point_name}}))
      print('reached:', json.loads(await ws.recv())['result'])

asyncio.run(patrol_by_route('ws://127.0.0.1:9000/ws', '巡逻线路A'))
```

### 6.3 跨楼层电梯巡检完整流程

当巡检路线跨楼层且涉及电梯时，完整的工作流如下：

1. 在源楼层导航到 `elevator_call` 点 → agent 呼叫电梯；
2. 导航到 `elevator_wait` 点 → agent 确认电梯到达、门已开；
3. 导航到 `elevator_in` 点（自动锁定 Z；`set_goal_by_slam` 同时自动放宽容差）；
4. 电梯运行中，FAST-LIO 通过 `/elevator_moving` 通告升降状态，导航与 PID 精调会被自动暂停并保持零速；
5. 电梯到达目标楼层后，下发目标时携带目标 `submap`，服务端会**自动换图**（VPR watchdog 也可在丢失定位时辅助识别），换图会**自动重置 Z 锁状态**；
6. 导航到 `elevator_out` 点（无需手动解锁 Z）；
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

      # 需要切换楼层时无需手动处理：下发目标点时带 submap，服务端会自动
      # 先导航到传送门、到达后换图（_ensure_submap），再前往目标点。
      if target_submap and target_submap != current_map:
        current_map = target_submap

      pinfo = await rpc(ws, "get_pose_in_submap", {"pose_name": point_name})
      if not pinfo.get('success'):
        print('skip', point_name)
        continue

      ptype = pinfo['point_info'].get('type', 'single_pose')

      if ptype == 'multi_pose':
        print('skip multi_pose (无专用下发接口):', point_name)
      else:
        print(await rpc(ws, "set_goal_by_slam", {"target": point_name, "submap": target_submap}))
        print(await rpc(ws, "wait_goal_reached", {"target": point_name}))

      # 电梯协同
      if ptype == 'elevator_call':
        print('Agent 呼叫电梯...')
        await simulate_agent_wait('elevator_called', delay=3.0)
      elif ptype == 'elevator_wait':
        print('等待电梯到达并开门...')
        await simulate_agent_wait('elevator_arrived', delay=6.0)
      elif ptype == 'elevator_out':
        # 无需手动退出电梯模式：出梯后换图会自动重置 Z 锁状态
        print('已出梯，Z 锁将随换图自动重置')

asyncio.run(patrol_with_elevator('ws://127.0.0.1:9000/ws', '跨楼层巡逻'))
```

### 6.4 切换楼层并继续巡检示例

当巡检路线跨楼层时，客户端**无需手动导航到传送门或换图**：只需在下发目标点时携带目标 `submap`，服务端（`_ensure_submap`）会先导航到当前楼层→目标楼层的传送门、到达后换图，再前往目标点。下面示例展示该流程：

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

      # 跨楼层无需手动取传送门/导航/换图：下方 set_goal_by_slam(submap=...)
      # 会由服务端 _ensure_submap 自动完成（先导航到传送门，到达后换图）。
      if target_submap and target_submap != current_map:
        current_map = target_submap

      await ws.send(json.dumps({"id":8, "method":"get_pose_in_submap", "params":{"pose_name": point_name}}))
      pinfo = json.loads(await ws.recv())['result']
      if not pinfo.get('success'):
        print('skip', point_name, pinfo.get('msg'))
        continue

      ptype = pinfo['point_info'].get('type', 'single_pose')
      if ptype == 'multi_pose':
        print('skip multi_pose (无专用下发接口):', point_name)
      else:
        await ws.send(json.dumps({"id":10, "method":"set_goal_by_slam", "params":{"target": point_name, "submap": target_submap}}))
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
    await ws.send(json.dumps({"id":1, "method":"set_goal_by_slam", "params":{"target": point_name}}))
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
    await ws.send(json.dumps({"id":10, "method":"set_goal_by_slam", "params":{"target": point_name}}))
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
- 最后更新：2026-08-06
