# VPR Bootstrap 模块文档（实时检测策略）

> 源文件：`NaviSkill/vpr_bootstrap.py`
>
> 上层调用方：`NaviSkill/server.py`（启动引导 + watchdog 重定位）
>
> 子进程实现：`NaviSkill/vpr/vpr_detect.py`（`--serve` 模式）

---

## 1. 模块定位

`vpr_bootstrap.py` 是 NaviSkill 在线 **VPR（Visual Place Recognition）重定位**的核心模块，目的是回答两个问题：

1. **启动时**：机器人开机站在哪个 submap 上？（"启动引导"）
2. **运行中**：当 TF 定位丢失时，机器人现在到底在哪个 submap 上？（"watchdog 重定位"）

之所以单独成一个模块、并跑成独立子进程，是因为：

- VPR 推理依赖 **PyTorch / torchvision / FAISS / SuperPoint + LightGlue**，依赖体积大且与 Jetson 上 CUDA wheel 强绑定，必须有独立 venv。
- NaviSkill 的主进程是 ROS + FastAPI WebSocket 的 Python 环境，**不能污染** VPR 的依赖。
- 模型 + 数据库加载需要 5～10s（Jetson 上更久），不能每次查询都启动一次进程。

因此整体结构是：

```
NaviSkill (主进程)
    │
    │  spawn 一次
    ▼
vpr/.venv/bin/python  vpr_detect.py --serve --databases ...
    ▲   │
    │   └── 一次性输出 {"event": "ready", ...}
    │
    └──  每帧一条 stdin JSON：{"id":..., "image_path":..., "top_k":...}
         每帧一条 stdout JSON：{"id":..., "results":[...], "elapsed_ms":...}
```

---

## 2. 模块组成

| 类 / 函数 | 职责 |
|-----------|------|
| `VprBootstrap` | 单次查询层。管理子进程生命周期，提供 `locate(jpeg_bytes)`：把一帧 JPEG 送给子进程，拿回 top-K 结果，并把 `database` 路径反查成 `submap_id`。 |
| `VprRelocator` | 多帧策略层。在 `VprBootstrap` 之上做"多帧采集 + 运动门控 + 共识投票 + 软共识降级 + 调试可视化"，最终输出一个稳健的 `submap_id` 并调用 `change_map_fn` 切图。 |
| `_undistort_and_center_crop()` | 查询前预处理：用 `PinHoleUndistorter` 去畸变，再中心裁剪到 1024×768，**必须与离线建库时几何一致**，否则描述符空间对不齐。 |
| `save_consensus_collage()` | 把一轮的 (query, match) 拼成 2×N 大图写到 `/tmp/vpr_debug/`，调试/复盘必备。 |

---

## 3. 单帧检测流程 `VprBootstrap.locate`

每次主进程要做一次"识别当前位置"，调用链是：

```
ros_node.get_latest_image_bytes()   # 取最新一帧 ROS CompressedImage
        │
        ▼
_undistort_and_center_crop()        # 去畸变 + 中心裁剪到 1024x768
        │
        ▼
write JPEG → /tmp/vpr_bootstrap_xxx/q_<uuid>.jpg
        │
        ▼
stdin → {"id":..., "image_path":..., "top_k":K, "rerank":bool, "rerank_top_k":20}
        │
        ▼      vpr_detect.py 子进程：
        │      1. encode_raw_bytes()        FAISS 召回 max(K, 20) 候选
        │      2. rerank_results()          GPU 模式下用 SuperPoint+LightGlue
        │                                   重排 top-20，截到 top-K
        ▼
stdout ← {"id":..., "results":[...], "elapsed_ms":..., "faiss_ms":..., "rerank_ms":...}
        │
        ▼
locate() 返回 dict：
  submap_id           database 路径反查得到的 submap id
  distance            FAISS L2 距离（越小越好）
  x, y, z, timestamp  匹配到的数据库参考图位姿（在 submap 坐标系下）
  rerank_inliers      LightGlue 内点数（越大越好；-1 表示该候选超过 rerank 截止）
  top_k_results       完整 top-K 列表（供软共识使用）
  query_image_path    去畸变 + 裁剪后的临时查询图路径（用于离线复盘）
```

**关键细节**：

- **图像预处理是强制的**：建库时几何 = 实时查询时几何，否则描述符无意义。`_undistort_and_center_crop` 第一次失败后会把 `_preprocess_enabled` 标 False，后续请求降级为原图直送（精度会掉，但不阻断）。
- **临时文件保留**：`q_<uuid>.jpg` 不会主动删除，子进程退出时 `tempfile.TemporaryDirectory` 才整体清理。这样有需要时可以拿原图离线复盘。
- **CUDA 自动降级**：`start()` 前用 `_resolve_device()` 跑一次 `torch.cuda.is_available()` 探针；如果 venv 装的是 CPU 版 torch，会自动降级到 `device=cpu` 而不是让子进程崩溃。

---

## 4. 多帧共识策略 `VprRelocator.collect_consensus`（重点）

单帧 top-1 在感知混淆（perceptual aliasing）场景下不可靠，所以实时检测的**最小决策单元是一轮共识**：

```
一轮共识
├── 帧 1 ──► VPR locate(top_K)  ──► (submap_id, top-K 候选, 采样时 odom)
├── 帧 2 ──► 运动门控 + locate   ──► (...)
├── 帧 3 ──► 运动门控 + locate   ──► (...)
├── 帧 4 ──► 运动门控 + locate   ──► (...)
└── 帧 5 ──► 运动门控 + locate   ──► (...)
        │
        ├── 严格共识？     (5 帧 top-1 全部相同)  ──► 成功
        │
        ├── 软共识？       (≥3 帧一致 + 其余帧 top-K 含同 submap
        │                    + odom/VPR 轨迹一致)  ──► 成功
        │
        └── 否则           ──► 进入下一轮（最多 MAX_ROUNDS 轮）
```

### 4.1 帧间运动门控（`_wait_for_motion`）

如果机器人静止不动，连续 5 帧拍同一面墙完全等价于 1 帧 ×5，浪费投票。改造为：

- 至少 `FRAME_INTERVAL_SEC = 1.0s` 间隔；
- 然后等到 odom 显示位移 ≥ `MIN_MOVE_M = 0.10m` **或** 旋转 ≥ `MIN_ROT_DEG = 20°` 才开始下一帧；
- `MOTION_WAIT_TIMEOUT_SEC = 30s` 后强采，避免机器人确实静止时永远收敛不了；
- 每帧之后调用 `ros_node.play_audio("search.mp3")` 提示操作员可以挪一下。

**降级**：`ros_node.get_latest_odom_pose` 不存在或返回 `None` 时，退化为纯 `time.sleep(FRAME_INTERVAL_SEC)`，行为与"运动门控"改造前完全一致。

### 4.2 几何重排（`SuperPoint + LightGlue`）

- 检测到 `device == "cuda"` 时自动开启 `rerank=True`：FAISS 先召回 `RERANK_TOP_K = 20` 个候选，再用 SuperPoint + LightGlue 算每对 (query, candidate) 的几何内点数，按内点数降序作为最终 top-K。
- CPU 模式自动关闭 rerank（单对图 1～3s，5 帧 ×20 候选会让一轮共识超过 1 分钟）。
- 重排显著缓解"远看长得像、近看其实不一样"这种感知混淆。

### 4.3 严格共识

最朴素也是最强的判定：

```python
consensus_ok = (
    not round_aborted
    and len(votes) == CONSENSUS_FRAMES        # 全部 5 帧都成功
    and len(set(votes)) == 1                  # top-1 全部相同
    and None not in votes
)
```

任何一帧 `locate` 失败 → 本轮 abort、直接进入下一轮；不会进入软共识。

### 4.4 软共识（严格共识失败时启用）

严格共识在"有 1～2 帧因为视角偏 / 反光 / 部分遮挡 top-1 错位"时容易整轮否决，所以加了一个**降级方案**。条件为：

| # | 条件 |
|---|------|
| 1 | 至少 `SOFT_CONSENSUS_MIN_VOTES = 3` 帧的 top-1 投同一个 `submap X` |
| 2 | 其余帧的 top-`SOFT_CONSENSUS_TOP_K = 10` 候选里**也存在** `submap X` 的匹配 |
| 3 | 把这 5 帧各自属于 `submap X` 的最高排名候选位姿 V_i，与采样时的 odom 位姿 O_i 做 2D 刚体对齐（**Kabsch / Procrustes**），残差在阈值内 |

第 3 步等价于一个**硬判 Viterbi-HMM**：
- *状态* = submap 坐标系下的真实位姿；
- *转移* 由 odom 相对位移约束；
- *观测* 是每帧 top-K 中匹配到 `submap X` 的候选位姿；
- 最优状态序列即 "每帧选属于 submap X 的最优候选"，再用 Kabsch 把 odom 坐标系拟合到 submap 坐标系。

```
R*, t* = argmin Σ ||(R · O_i + t) - V_i||²

判定通过 ⇔ mean ‖residual‖ ≤ TRAJ_FIT_MEAN_RESIDUAL_M (= 3.0m)
         且 max  ‖residual‖ ≤ TRAJ_FIT_MAX_RESIDUAL_M  (= 5.0m)
```

**退化分支**：若 odom 的总延展 < `TRAJ_STATIC_ODOM_M = 0.10m`，Kabsch 是奇异的；这时只要 VPR 总延展也 < `TRAJ_STATIC_VPR_M = 1.0m`，认为机器人确实在静止/微动，判通过。

**注意**：odom 不可用时（任一帧 `_get_odom_pose()` 返回 `None`）软共识直接判失败 —— 没有 odom 就没法做轨迹一致性校验，宁可放弃软共识也不要错切图。

---

## 5. 配置参数速查

所有阈值都是 `VprRelocator` 的 **class-level 属性**，方便子类化或在实例上直接覆盖（如 `relocator.MIN_MOVE_M = 0.3`）。

### 5.1 采集与运动门控

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `CONSENSUS_FRAMES` | `5` | 每轮采集多少帧 |
| `FRAME_INTERVAL_SEC` | `1.0` | 帧最小间隔（也是运动门控的轮询周期） |
| `MAX_ROUNDS` | `3` | 共识最多重试多少轮 |
| `IMAGE_WAIT_SEC` | `5.0` | watchdog 场景每帧等相机出图超时 |
| `STARTUP_IMAGE_WAIT_SEC` | `10.0` | 启动场景每帧等相机出图超时（相机话题可能还没建好） |
| `MIN_MOVE_M` | `0.10` | 相邻两次采样最小位移（米） |
| `MIN_ROT_DEG` | `20.0` | 相邻两次采样最小旋转（度） |
| `MOTION_WAIT_TIMEOUT_SEC` | `30.0` | 运动门控最长等待时间，超时强采 |

### 5.2 检索与重排

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `RERANK_TOP_K` | `20` | SuperPoint + LightGlue 重排的候选数 |
| `SOFT_CONSENSUS_TOP_K` | `10` | 每帧请求的 top-K 候选数（也是软共识里非一致帧的检索深度） |

> 注意：`collect_consensus` 内部把 `locate_top_k = max(1, SOFT_CONSENSUS_TOP_K)`，所以**每帧实际请求的就是 top-10**。

### 5.3 软共识阈值

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `SOFT_CONSENSUS_MIN_VOTES` | `3` | 软共识需要的最少 top-1 一致票数 |
| `TRAJ_FIT_MEAN_RESIDUAL_M` | `3.0` | Kabsch 拟合后平均残差阈值（米） |
| `TRAJ_FIT_MAX_RESIDUAL_M` | `5.0` | Kabsch 拟合后单点残差阈值（米） |
| `TRAJ_STATIC_ODOM_M` | `0.10` | odom 总延展 < 此值视为机器人基本静止 |
| `TRAJ_STATIC_VPR_M` | `1.0` | 静止分支：VPR 总延展也应 < 此值 |

### 5.4 调试与提示音

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `DEBUG_DIR` | `/tmp/vpr_debug` | 共识可视化大图保存目录 |
| `FRAME_DONE_AUDIO` | `"search.mp3"` | 每张成功检测后由 `ros_node.play_audio()` 播放 |

---

## 6. 上层入口：启动引导 vs Watchdog 重定位

`VprRelocator` 对外只暴露两个高层方法，对应 `server.py` 里的两个调用点：

### 6.1 启动引导：`pick_initial_submap()`

```
server.py: 启动时若用户未显式指定 submap
    └── VprRelocator.pick_initial_submap()
            └── collect_consensus("startup bootstrap",
                                  image_wait_sec=STARTUP_IMAGE_WAIT_SEC=10.0)
                    └── 成功 → change_map_fn(submap_id) → ROS 端切到该 submap
                    └── 失败 → 回退到 MapManager 当前 submap（默认）
```

### 6.2 Watchdog 重定位：`relocate_and_change_map(reason)`

`server.py` 里有个异步 watchdog 协程 `_tf_watchdog_task`：

```
while True:
    if tf_ready:          # 定位还好，重置丢失计数
        loss_since = None
    else:
        # 1) 静默期：上次成功后 success_cooldown_sec 内不重复触发
        # 2) 宽限期：丢失 grace_sec 才正式判失
        # 3) 失败冷却：上次尝试失败后 fail_cooldown_sec 内不重试
        if 满足触发条件:
            await asyncio.to_thread(
                vpr_relocator.relocate_and_change_map,
                f"tf_ready=False 已 {lost_for:.1f}s",
            )
```

注意 `relocate_and_change_map` 是**同步阻塞**的（含多帧采集 + Kabsch + change_map 整套流程，可能持续 10s+），所以 server.py 用 `asyncio.to_thread(...)` 丢到线程池，避免卡住 asyncio 事件循环。

`VprRelocator` 本身**不**关心 cooldown / 触发条件 / 静默策略，那些都是 server 端的职责。

---

## 7. 日志与调试可视化

模块自己挂了一个 INFO 级 `StreamHandler`（`server.py` 把根 logger 设到了 WARNING，所以单独挂以保证可见），关闭 propagate 避免 WARNING+ 走两遍 handler。

### 7.1 关键日志行

每帧 locate 都会打 top-K 一览：

```
VprBootstrap.locate top-10 结果 (faiss=12.3ms, rerank=210.5ms, total=222.8ms):
  [0] submap=floor2  dist=0.4231  inliers=85  xyz=(12.34,5.67,0.10)  /datasets/2f/img_0123.jpg
  [1] submap=floor2  dist=0.4582  inliers=42  xyz=(12.20,5.50,0.10)  /datasets/2f/img_0124.jpg
  [2] submap=floor3  dist=0.5114  inliers=8   xyz=(8.10,2.30,3.20)   /datasets/3f/img_0050.jpg
  ...
```

每帧采样会带上 odom：

```
[VPR] 第 3/5 帧 → submap='floor2' (rerank_inliers=85, dist=0.4231, 250ms, odom=(1.23,4.56,90.0°))
```

软共识检查会打出残差和判定：

```
[VPR] 软共识候选：submap='floor2' (top-1 票数 3/5, 各帧来源=['top1','top1','top2','top1','top3'])
[VPR] 软共识轨迹检查：n=5, odom_extent=2.34m, vpr_extent=2.21m, residual mean=0.413m / max=0.890m
       (阈值 mean≤3.00, max≤5.00) → 通过
```

### 7.2 调试可视化大图

`_save_debug_image` 每轮都写一张到 `DEBUG_DIR`，**无论成败**：

```
/tmp/vpr_debug/vpr_20260525_174203_round1_softok_floor2.jpg
```

文件名格式：`vpr_<timestamp>_round<N>_<status>_<chosen>.jpg`

| status | 含义 |
|--------|------|
| `ok` | 严格共识达成 |
| `softok` | 软共识达成（top-1 票不一致但通过 Kabsch 校验） |
| `noconsensus` | 5 帧采到但严格 + 软共识都没过 |
| `framefail` | 某帧 locate 失败，整轮 abort |

图片结构：顶行 N 张 query 图，底行 N 张 top-1 匹配图，标题里有 votes / inliers / reason。可以直接 `ls -lt /tmp/vpr_debug/` 按时间找最近一次的复盘。

---

## 8. 失败降级矩阵

| 失败位置 | 行为 | 影响 |
|----------|------|------|
| `vpr/.venv` 不存在或 Python 解释器缺失 | `VprBootstrap.start()` 直接返回 `False` | 全程禁用 VPR，启动 / watchdog 都跳过 |
| 数据库目录不全 | `_discover_database_dirs` 警告 + 跳过该 submap | 该 submap 不可被 VPR 识别 |
| CUDA 探针失败 | `_resolve_device()` 自动降级为 `cpu` | VPR 仍可用，rerank 会被关闭 |
| 子进程在 ready 之前退出 | `_wait_for_ready` 超时，回显最后 80 行 stderr | VPR 不可用 |
| 单帧 `locate` 失败 | 本轮 abort，下一轮重试（最多 `MAX_ROUNDS` 轮） | 当前轮浪费，下一轮再来 |
| 整轮严格共识失败 | 进入软共识；软共识失败再进入下一轮 | 多花时间，但更鲁棒 |
| 所有轮全部失败 | `collect_consensus` 返回 `None` | 启动场景回退到 MapManager 缓存；watchdog 场景由 server cooldown 控制下次重试 |
| `_get_odom_pose()` 返回 `None` | 运动门控降级为固定周期；软共识直接判失败 | 严格共识仍然有效，软共识失效 |
| 图像预处理（去畸变）失败 | `_preprocess_enabled = False`，原图直送 | 描述符精度下降，匹配可能不稳；但不阻断 |

---

## 9. 集成示意

```
server.py NavigationToolManager.__init__
    │
    ├── VprBootstrap(map_manager, device=cuda/cpu)      # 发现 submap 数据库
    │       └── _discover_database_dirs                 # NaviSkill/vpr/<submap>/database_*
    │
    ├── VprBootstrap.start()                            # 起子进程 + 等 ready
    │       └── vpr/.venv/bin/python vpr_detect.py --serve ...
    │
    ├── VprRelocator(vpr_bootstrap, ros_node,
    │                map_manager, change_map_fn=self._change_map)
    │
    ├── if not submap_specified:
    │       relocator.pick_initial_submap()             # 启动引导
    │
    └── async _tf_watchdog_task:
            while True:
                if tf 丢失超过 grace 且 cooldown 满足:
                    await asyncio.to_thread(
                        relocator.relocate_and_change_map, reason
                    )
```

---

## 10. 调参建议

| 场景 | 建议 |
|------|------|
| 室内空旷、无纹理走廊 | 增大 `RERANK_TOP_K`（如 30~40），让 LightGlue 有更多机会找到几何一致的真值 |
| 走廊两边长得像（强混淆） | 减小 `TRAJ_FIT_MEAN_RESIDUAL_M` / `TRAJ_FIT_MAX_RESIDUAL_M`，拒绝 Kabsch 拼凑出来的"伪一致" |
| 机器人移动慢、采样困难 | 调小 `MIN_MOVE_M` / `MIN_ROT_DEG`，避免门控超时强采的频率过高 |
| Jetson 算力紧张 | `RERANK_TOP_K` 减到 10、`SOFT_CONSENSUS_TOP_K` 减到 5；或直接 `device="cpu"` 关 rerank |
| watchdog 误触发频繁 | 拉大 server.py 端的 `grace_sec` / `success_cooldown_sec`，不要改 `VprRelocator` 自己的阈值 |
| 软共识误识别 | 提升 `SOFT_CONSENSUS_MIN_VOTES` 到 4；或临时禁用：设 `SOFT_CONSENSUS_MIN_VOTES = CONSENSUS_FRAMES + 1` |

---

## 11. 相关文件

- `NaviSkill/vpr_bootstrap.py` — 本文档对应源码
- `NaviSkill/vpr/vpr_detect.py` — 子进程实现（`--serve` 协议、FAISS 检索、几何重排）
- `NaviSkill/vpr/vpr_preprocess.py` — 离线建库脚本（决定描述符空间 + 去畸变设置）
- `NaviSkill/vpr/install_jetson.sh` — Jetson 上 venv 的 bootstrap
- `NaviSkill/server.py` — 上层调用方（启动引导 + watchdog）
- `NaviSkill/nav_node.py` — `get_latest_image_bytes` / `get_latest_odom_pose` / `change_map` 的 ROS 端实现
- `NaviSkill/docs/submap_system.md` — submap 配置、坐标变换、portals 等多楼层语义
