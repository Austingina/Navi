# 操作手册：启动导航前置服务 & 发起路线导航

本手册面向**现场操作 / 联调人员**，说明两件事：

1. 如何把**导航所需的前置服务**（ROS2 导航栈 + WebSocket 导航服务）拉起来；
2. 如何用 `NaviSkill/test/test_client.py` **发起一条路线（巡检）导航**。

> 配套文档：
> - `NaviSkill/docs/NAVIGATION_WEBSOCKET_API.md`（WebSocket RPC 方法参考）
> - `NaviSkill/docs/submap_system.md`（子地图 / 传送门 / 路线 / 地图切换原理）
> - `MapEditor/docs/操作手册-多子地图SLAM地图制作.md`（如何制作带 routes 的地图）

---

## 1. 系统总览

发起一次路线导航涉及三层，**自下而上依赖**：

```
┌──────────────────────────────────────────────────────────────┐
│  ③ 测试/发起客户端  test_client.py   （发起路线导航、暂停/恢复）   │
│      └── WebSocket 连接 ws://<host>:9000/ws                    │
├──────────────────────────────────────────────────────────────┤
│  ② WebSocket 导航服务  server.py    （端口 9000；内含 NavNode）  │
│      └── 加载 multifloor 地图(routes/点位/传送门)，下发 ROS 目标 │
├──────────────────────────────────────────────────────────────┤
│  ① ROS2 导航栈                                                  │
│      • grid_map_pcl_loader  → 发布 /map 栅格地图                 │
│      • FAST_LIO 定位 + Nav2  → 定位、全局/局部规划、控制          │
│      • （实车）LiDAR / 传感器驱动                                │
└──────────────────────────────────────────────────────────────┘
```

- **①** 由 `ros2 launch` 拉起，提供地图、定位与 Nav2 规划控制。
- **②** `server.py` 在端口 **9000**（加 `-t` 测试模式则为 **9999**）暴露 WebSocket `/ws`，
  把“按名字/坐标下发目标、查询状态、切图、路线/点位查询”封装成 RPC。
- **③** `test_client.py` 连到 `/ws`，读取路线并**按途经点依次发起导航**。

三层通常由一键脚本 `scripts/start_navi.sh` 用 tmux 一起拉起。

---

## 2. 前置条件（首次运行前确认）

| 项 | 说明 |
|----|------|
| ROS2 工作区已编译 | 存在 `ros2_ws/install/setup.bash`（所有脚本都会 `source` 它） |
| Python 虚拟环境 | 服务端与客户端都用 `NaviSkill/vpr/.venv/bin/python3` |
| 地图数据 | `maps/<id>/<id>.json`（默认地图 `maps/408/408.json`）；栅格底图 `maps/xunjian_2cm.pcd` |
| 地图含路线 | 目标地图的 `routes` 非空，否则无法“路线导航”（用 MapEditor 添加，见配套手册） |
| tmux / tmuxp | 一键脚本依赖；`pip install tmuxp` 或 `apt install tmux` |
| 实车 | LiDAR / 底盘 / 传感器驱动已上电正常（否则定位无法收敛） |

> 下文命令以**仓库根目录**为工作目录（此处为 `/home/navi/Navi`，部署机常见为 `/home/nvidia/Navi`）。
> 脚本内部通过 `scripts/common.sh` 自动推导 `NAVI_ROOT_DIR`，无需手改路径。

---

## 3. 启动前置服务（方式 A：一键，推荐）

```bash
cd /home/navi/Navi
bash scripts/start_navi.sh
```

它会：

1. 先执行 `scripts/stop_navi.sh` 清理旧的 `navi` tmux 会话；
2. 用 `tmuxp` 加载 `scripts/navi.slam.yaml`，在 `navi` 会话里**依次**拉起三个服务（见 `navi.slam.yaml`）：

   | 顺序 | pane 命令 | 作用 |
   |------|-----------|------|
   | 1 | `start_gridmap.bash -v` | 加载 `xunjian_2cm.pcd`，发布 `/map` 栅格地图（`-v` 带 RViz） |
   | 2 | `start_navigation.bash` | `ros2 launch fast_lio r1pro_nav2.launch.py`：FAST_LIO 定位 + Nav2 |
   | 3 | `start_mcp.sh` | 启动 `server.py`（WebSocket 端口 9000，默认地图 `408.json`） |

查看/操作会话：

```bash
tmux attach -t navi     # 进入，Ctrl+b 再按数字/方向键切 pane
# Ctrl+b d 脱离（服务继续在后台运行）
```

**停止全部服务：**

```bash
bash scripts/stop_navi.sh   # kill tmux 会话 navi
```

---

## 4. 启动前置服务（方式 B：手动逐个，排障用）

每个服务开一个终端；每个脚本都会自动 `source ros2_ws/install/setup.bash`。**严格按 ①→②→③ 顺序**，
上一个基本就绪再起下一个。

**① 栅格地图服务**

```bash
cd /home/navi/Navi
bash scripts/start_gridmap.bash -v
# 等价于: ros2 launch grid_map_pcl grid_map_pcl_loader_launch.py \
#           rviz:=true map_file_path:=$NAVI_ROOT_DIR/maps/xunjian_2cm.pcd
```
产出：`/map`（OccupancyGrid）。

**② 定位 + Nav2**

```bash
bash scripts/start_navigation.bash
# 等价于: ros2 launch fast_lio r1pro_nav2.launch.py
```
产出：机器人 TF/定位、Nav2 的 `NavigateToPose` action、`/plan` 全局路径等。
> 实车若 LiDAR 驱动未包含在该 launch 中，需另行启动（见 `scripts/start_lidar.bash`），否则定位不收敛。

**③ WebSocket 导航服务（server.py）**

```bash
bash scripts/start_mcp.sh                       # 默认地图 maps/408/408.json，端口 9000
bash scripts/start_mcp.sh -m /home/navi/Navi/maps/408/408.json   # 显式指定地图
bash scripts/start_mcp.sh -s underground        # 指定初始 submap
bash scripts/start_mcp.sh -t                    # 测试模式，端口改为 9999
```
`start_mcp.sh` 把参数原样透传给 `server.py`（`-m/--map`、`-s/--submap`、`--image_topic`、`--vpr_device`、`-v/--viz` 等）。
启动成功会打印：`FastAPI WebSocket: ws://0.0.0.0:9000/ws | GET /health`。

---

## 5. 校验前置服务是否就绪

```bash
# 1) WebSocket 服务健康检查
curl http://localhost:9000/health          # 期望返回 ok/健康标识

# 2) 关键话题存在
source /home/navi/Navi/ros2_ws/install/setup.bash
ros2 topic list | grep -E "/map|/plan"

# 3) 定位是否收敛（用客户端查一次状态；能返回 position 即 OK）
cd /home/navi/Navi/NaviSkill/test
/home/navi/Navi/NaviSkill/vpr/.venv/bin/python3 - <<'PY'
import asyncio, json, websockets
async def main():
    async with websockets.connect("ws://localhost:9000/ws") as ws:
        await ws.send(json.dumps({"id":1,"method":"get_status","params":{}}))
        print(await ws.recv())
asyncio.run(main())
PY
```

三项都正常，即可发起路线导航。

---

## 6. 发起路线导航（test_client.py）

`test_client.py` 就是“**发起路线导航**”的入口。推荐用封装脚本启动（内部用 venv Python）：

```bash
cd /home/navi/Navi
bash scripts/start_mcpclient.sh [参数...]
# 等价于: NaviSkill/vpr/.venv/bin/python3 NaviSkill/test/test_client.py [参数...]
```

> **关键：必须带 `-p`（巡检模式）才会真正发起路线导航**；不带 `-p` 也不带 `--set-xy` 时客户端只会打印提示后退出。

### 6.1 常用命令

| 目的 | 命令 |
|------|------|
| 用**默认路线 / 第一条路线**跑 | `bash scripts/start_mcpclient.sh -p` |
| 指定路线名 | `bash scripts/start_mcpclient.sh -p -r 线路A` |
| 从路线**第 N 个点**开始（N 从 1 起） | `bash scripts/start_mcpclient.sh -p -r 线路A 3` |
| **断点续跑**：从距当前最近的途经点继续（容差 2m） | `bash scripts/start_mcpclient.sh -p -c 2.0` |
| **自动确认**电梯提示（无人值守） | `bash scripts/start_mcpclient.sh -p -Y -r 线路A` |
| 先**切到指定子地图**再执行 | `bash scripts/start_mcpclient.sh -s underground -p -r 线路A` |
| 自定义**目的地序列**（同层点位名） | `bash scripts/start_mcpclient.sh -p 点位A 点位B 点位C` |
| 连测试端口的服务（server 用了 `-t`=9999） | `bash scripts/start_mcpclient.sh --ws ws://127.0.0.1:9999/ws -p -r 线路A` |
| 单点坐标导航自测（不走路线） | `bash scripts/start_mcpclient.sh --set-xy 12.3 -4.5` |

参数说明（见 `test_client.py` 的 argparse）：

- `--ws`：WebSocket 地址，默认 `ws://localhost:9000/ws`（也可用环境变量 `NAV_WS_URL`）。
  **端口要与 server 一致**：不带 `-t` 起的 server 是 9000；带 `-t` 是 9999。
- `-r NAME [START_INDEX]`：路线名，可跟起始序号（从 1 起）。
- `-p [DEST...]`：开启巡检；不给 DEST 时用 `-r` 指定路线或服务器默认路线。
- `-c TOL`：从距当前位置最近的途经点续跑，`TOL` 为容差（米）。
- `-Y`：自动确认 `elevator_wait/elevator_in/elevator_out` 三类电梯提示。
- `-s ID`：执行前先 `change_map` 切到该子地图。

### 6.2 运行中的交互

- **空格键**：暂停 / 恢复当前导航（客户端调用 `pause_navigation` / `resume_navigation`）。
- **电梯点位**：途经点类型为 `elevator_wait/in/out` 时会**停下等你回车确认**（除非加了 `-Y`）。
- **Ctrl+C**：中断本次巡检。
- 结束后客户端会打印**巡检报告**（总点位、成功/失败数、成功率、总耗时/均耗时）。

---

## 7. 一条路线导航“内部发生了什么”

了解流程有助于定位卡点（对应 `test_client.py::start_patrol_test`）：

1. **列出子地图**：`get_submap_list` / `get_submap_info`（打印各图点位、传送门数量）。
2. **确定路线**：
   - 有 `-p DEST...` → 直接用该序列；
   - 否则 `get_route_names` 取全部路线 → 用 `-r` 指定，或 `get_default_route`，否则取第一条 → `get_route` 拉出 `waypoints`。
3. **准备**：`get_status`（当前位置）、`get_nearest_point`；`-c` 模式据此选起点。
4. **逐个途经点**（`{submap, semantic_point}`）：
   - **需要跨子地图**（`get_current_map_id` ≠ 目标 submap）时：
     `get_portal_to_submap` 取传送门 → `set_goal_by_xy` 导到传送门 → `wait_goal_reached`
     → `change_map` 切图 → 等待 3s。
   - **到点**：读 `get_pose_in_submap`（电梯类型会提示确认）→ `set_goal_by_name` 下发目标
     → `wait_goal_reached` 轮询到达（支持空格暂停；单点默认超时 120s，最多重试 2 轮）。
   - 任一途经点失败即**中止**并出报告。

涉及的 RPC 方法细节见 `NAVIGATION_WEBSOCKET_API.md`。

---

## 8. 常见问题（FAQ）

| 现象 | 可能原因 | 处理 |
|------|---------|------|
| 客户端连不上 / `/health` 不通 | server 未起，或端口不对 | 确认 `start_mcp.sh` 在跑；核对端口（9000；`-t` 为 9999，客户端要 `--ws ...9999`） |
| `❌ 服务器未配置任何路线` / 途经点为空 | 地图 `routes` 为空 | 用 MapEditor 给该地图添加路线（见配套手册）并重启 server |
| `get_status` 失败 / 位置乱跳 | 定位未收敛 | 检查 `start_navigation.bash`、LiDAR/传感器、初始 `submap` 是否正确 |
| `change_map` 超时 | 栅格地图服务未运行 / 目标 PCD 路径问题 | 确认 `start_gridmap.bash` 在跑；检查目标子地图 `map_file` |
| 找不到传送门、跨层失败 | 传送门只配了单向 | 在两个子地图各配一条传送门（见 `submap_system.md`） |
| 卡在电梯点位不动 | 未确认电梯提示 | 手动回车确认，或加 `-Y` 无人值守 |
| tmux 会话找不到服务 | 一键脚本未成功加载 | `tmux attach -t navi` 看各 pane 报错；或用第 4 节手动逐个启动排障 |

---

**快速回顾（最短路径）**

```bash
# 1. 起前置服务（三合一）
cd /home/navi/Navi && bash scripts/start_navi.sh
# 2. 确认就绪
curl http://localhost:9000/health
# 3. 发起路线导航（默认路线）
bash scripts/start_mcpclient.sh -p
#    指定路线：bash scripts/start_mcpclient.sh -p -r 线路A
# 4. 结束后停服务
bash scripts/stop_navi.sh
```
