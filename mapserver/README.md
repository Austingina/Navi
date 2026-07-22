# Navi Map Server

地图分发服务：把 `/maps/<map_id>/` 下的整套地图资源通过 HTTP 暴露给机器人，并支持增量更新。
支持两种地图数据布局（均由 MapEditor 产出）：

- **场景地图（Scene，推荐 / 新格式）**：`scene.json` + OSM 底图（`osm/*.osm|*.osm.pbf`）
  + 各 SLAM 地图的 PCD（按 `pcd_base` 分子目录）。一个 `scene.json` 内联 OSM、多张 SLAM 地图与跨地图全局路线。
- **单张 SLAM 地图工程（旧格式，兼容）**：`<map_id>.json`（`multifloor.json` 风格）+ 若干 `.pcd` + VPR 数据库等。

两种布局的分发/增量同步机制完全一致——服务端只按文件 `sha256` 做差异，与地图格式无关。

## 设计要点

- **接口**：`GET http://ip:port/mapserver/<map_id>` 返回 manifest（JSON）。
- **增量更新**：客户端按 manifest 中每个文件的 `sha256` 比对，只重传变更或缺失的文件（格式无关）。
- **版本号**：按优先级读取（详见「版本号来源」）：
  1. `<map_dir>/scene.json` 的 `scene.build_no`（**场景地图**，顶层无 `global`）；
  2. `<map_dir>/<map_id>.json` 的 `global.version` / `global.build_no`（**单张 SLAM 地图工程**）；
  3. 独立的 `<map_dir>/version.json`（兼容旧布局）；
  4. 都没有时由服务端按 mtime 自动派生。
  MapEditor 的 `build_no` 均为 14 位 `YYYYMMDDHHMMSS`，且**仅在对应内容有更新时**才刷新时间戳。
- **断点续传**：服务端支持 HTTP `Range`；客户端写 `*.partial` → 校验 sha256 → 原子 `os.replace`。
- **崩溃可续**：客户端每下完一个文件就持久化一次本地 manifest，下次重启不会重传已下完的部分。
- **路径遍历防护**：所有相对路径 `..`、绝对路径、转义字符都会被服务端拒绝。
- **认证**：暂未做（默认仅在内网部署），可在反向代理层加 Basic Auth 或 mTLS。

## 目录结构

**场景地图（新格式）** —— 一个 `map_id` 目录即一个「场景」：

```
/home/nvidia/Navi/maps/
├── park/                ← map_id="park"（场景目录）
│   ├── scene.json       ← scene.build_no 即权威版本号（顶层无 global）
│   ├── osm/
│   │   └── park.osm.pbf ← OSM 底图（scene.osm.source 相对引用）
│   ├── buildingA/       ← 某张 SLAM 地图的 PCD（slam_maps.buildingA.pcd_base）
│   │   ├── 1f.pcd
│   │   └── 2f.pcd
│   ├── buildingB/
│   │   └── 1F.pcd
│   └── .mapserver_manifest.json  ← 客户端自动生成（同步状态）
```

`scene.json` 顶部（MapEditor「保存场景」写出）：

```json
{
  "scene": {
    "name": "示例园区",
    "crs": "WGS84",
    "build_no": 20260721204312,
    "osm": { "source": "osm/park.osm.pbf", "bounds": { "...": "..." } }
  },
  "slam_maps": { "buildingA": { "geo_anchor": {"...": "..."}, "global": {"...": "..."} } },
  "routes": { "...": "跨地图全局路线" }
}
```

- 场景版本以 **`scene.build_no`** 为准（`YYYYMMDDHHMMSS`）：**仅当场景内容有更新时**才刷新
  （任一 SLAM 地图内部信息变化也会带动它更新）；服务端 version 取 `scene.name`、build_no 取 `scene.build_no`。
- 每张 SLAM 地图各自的 `slam_maps.<id>.global.build_no` 也遵循“**其内部信息有更新才刷新**”。它们随
  `scene.json` 的内容一起参与文件哈希，因此任何一张地图变化都会触发对应文件重传。

**单张 SLAM 地图工程（旧格式，兼容）** —— 一个 `map_id` 目录即一张多子地图 SLAM 地图：

```
/home/nvidia/Navi/maps/
├── 408/                 ← map_id="408"
│   ├── 408.json         ← global.version / global.build_no 即权威版本号
│   ├── 1f.pcd
│   ├── 2f.pcd
│   ├── vpr/2f/...
│   ├── version.json     ← (可选) 兼容旧布局，独立维护版本号
│   └── .mapserver_manifest.json
```

`408.json` 顶部（MapEditor 写出）：

```json
{
  "global": {
    "default_submap": "2f",
    "default_route": "route262",
    "version": "1.2.3",
    "build_no": 20260518205930
  },
  ...
}
```

- **`version`**：人维护，从 `"1.0"` 开始，按需在 MapEditor 的"文件 → 修改版本号..."里改。
- **`build_no`**：内容有更新时刷为本地时间 `YYYYMMDDHHMMSS`（如 `20260518205930` 表示 2026-05-18 20:59:30）。14 位整数，单调递增 + 一眼可读。

可选的独立 `version.json`（仅当地图不由 MapEditor 编辑时使用）：

```json
{ "version": "1.2.3", "build_no": 20260518205930 }
```

### 版本号来源（服务端解析优先级）

1. `scene.json` → `scene.build_no`（场景地图）；
2. `<map_id>.json` → `global.version` / `global.build_no`（单张 SLAM 地图工程）；
3. `version.json`（兼容旧约定）；
4. 都没有 → 自动派生（`version="auto-<8位sha>"`、`build_no` 取最新 mtime 的 `YYYYMMDDHHMMSS`，仅适合开发环境）。

## 部署服务端

```bash
# Python 依赖：fastapi + uvicorn（NaviSkill 已装）
python /home/nvidia/Navi/mapserver/server.py \
    --maps-root /home/nvidia/Navi/maps \
    --host 0.0.0.0 \
    --port 8080
```

接口速览：

| Method | Path                                     | 说明                       |
|--------|------------------------------------------|----------------------------|
| GET    | `/healthz`                               | 健康检查                   |
| GET    | `/mapserver/`                            | 列出所有可用 map_id        |
| GET    | `/mapserver/{map_id}`                    | 取该 map 的 manifest（JSON）|
| GET    | `/mapserver/{map_id}/manifest`           | 同上（显式路径）           |
| GET    | `/mapserver/{map_id}/files/{path}`       | 下载单文件（支持 Range）   |
| POST   | `/mapserver/{map_id}/invalidate`         | 运维：清缓存（git pull 后） |

### 手工 curl 示例

```bash
# 看有哪些 map
curl http://10.0.0.1:8080/mapserver/

# 取 manifest
curl http://10.0.0.1:8080/mapserver/408 | jq

# 下载单文件
curl -o 408.json http://10.0.0.1:8080/mapserver/408/files/408.json

# 断点续传（从第 1MB 开始）
curl -H "Range: bytes=1048576-" -o 1f.pcd.partial \
     http://10.0.0.1:8080/mapserver/408/files/1f.pcd
```

## 部署客户端

### CLI 一行同步

```bash
python /home/nvidia/Navi/mapserver/client.py \
    --server http://10.0.0.1:8080 \
    --map 408 \
    --local /home/nvidia/Navi/maps/408
```

输出（示例）：

```json
{
  "status": "updated",
  "server_version": "1.2.3",
  "server_build_no": 42,
  "local_version": "1.2.2",
  "local_build_no": 41,
  "downloaded": ["408.json", "vpr/2f/database_descriptors.npy"],
  "deleted": [],
  "failed": [],
  "bytes_downloaded": 9924567,
  "elapsed_sec": 3.41
}
```

常用选项：

- `--dry-run`：只打印差异，不下载。
- `--delete-extras`：删除本地有、server 没有的文件（默认仅在 manifest 中移除条目）。

### 库 API

```python
from mapserver.client import sync_map, SyncStatus

result = sync_map(
    "http://10.0.0.1:8080",
    "408",
    "/home/nvidia/Navi/maps/408",
)
if result.status == SyncStatus.UP_TO_DATE:
    print("nothing to do")
elif result.status == SyncStatus.UPDATED:
    print(f"pulled {result.bytes_downloaded} bytes")
else:
    print(f"failed files: {result.failed}")
```

### 嵌入到机器人启动流程

可以在 `NaviSkill/server.py` 启动 ROS / VPR 之前先跑一次同步：

```python
from mapserver.client import sync_map, SyncStatus

result = sync_map(MAP_SERVER_URL, map_id, f"/home/nvidia/Navi/maps/{map_id}")
if result.status == SyncStatus.FAILED:
    logging.warning("map sync failed, using local copy: %s", result.failed)
```

也可以做成 systemd timer 每天巡检一次。

## 客户端判定逻辑（详细）

```
1. fetch server manifest = GET /mapserver/<id>
2. load local manifest   = .mapserver_manifest.json
3. if local && local.version == server.version
                       && local.build_no == server.build_no:
       return UP_TO_DATE
4. for each f in server.files:
       if not local || local[f.path].sha256 != f.sha256:
           download to <path>.partial → verify sha256 → os.replace
           merge into local manifest, save
5. if --delete-extras:
       remove files in local but not in server
6. on success: write local manifest := server manifest
   on any failure: keep local manifest as a "mixed" state with old
                   version/build_no, so next run will retry the failed files
```

关键属性：

- 客户端只在**全部下载并校验成功**之后才把本地的 `version` / `build_no`
  切到服务器版本。换句话说：失败的同步绝不会假装升级成功，下一次同步会自然重试。
- 服务器端 manifest 用 **(path, size, mtime_ns) 指纹** 做内存缓存：
  没有文件变化 → 直接返回缓存；改了哪怕一个字节 → 重哈希一遍。

## 增量更新的实际效果

对当前 `408/` 地图（568 MiB，18 个文件）：

- 只改 `408.json`（路径图，46 KiB）→ client 只重传 **46 KiB**，~毫秒级。
- 重新建一个楼层的 VPR 数据库（~25 MiB）→ client 只重传 **25 MiB**。
- 重新跑一次完整建图（所有 PCD 变化）→ 退化为全量 568 MiB；
  断点续传仍生效，网络抖动不会从头开始。

## 地图数据备份 / 恢复（mapserver.backup）

对线上地图目录做**全量快照**，并能从历史快照恢复。围绕两个目录约定：

- **front（线上）**：`~/mapserver/front/<map_id>`，MapEditor / 导航栈实际读写的当前地图。
- **backup（备份）**：`~/mapserver/backup`（默认根），按
  `~/mapserver/backup/<map_id>/<build_no>` 分层存放。

`build_no` 按与 manifest 相同的优先级解析（`scene.json` 的 `scene.build_no` →
`<map_id>.json` 的 `global.build_no` → `version.json`；均为 14 位时间戳 `YYYYMMDDHHMMSS`），
所以"同一张地图/场景的每个保存版本"自然落到不同的备份目录。

```
~/mapserver/
├── front/
│   └── 408/                       ← 线上地图（MapEditor 编辑、导航读取）
│       ├── 408.json               ← global.build_no = 20260601151834
│       └── ...
└── backup/
    └── 408/
        ├── 20260518205930/        ← 历史快照（build_no 即目录名）
        └── 20260601151834/        ← 最新快照
```

### CLI

```bash
# 备份：把 front/408 整个快照到 backup/408/<build_no>/
#（build_no 自动读 408.json 的 global.build_no；目录已存在则跳过，--force 覆盖）
python -m mapserver.backup backup --map 408

# 列出 408 的所有历史备份（按 build_no 倒序：新 → 旧）
python -m mapserver.backup list --map 408

# 恢复：交互式从备份列表里选一个，覆盖回 front/408
python -m mapserver.backup restore --map 408
# 或直接指定 build_no（非交互，便于脚本调用）
python -m mapserver.backup restore --map 408 --build-no 20260601151834
```

公共参数：`--front-root`（默认 `~/mapserver/front`）、`--backup-root`
（默认 `~/mapserver/backup`）、`--log-level`。

### 关键属性

- **原子替换**：备份 / 恢复都是"先拷到同目录下的临时 `.staging` → `os.replace`"，
  中途崩溃不会留下半个目录、也不会破坏现有 front（被覆盖的旧目录先挪到临时 trash，
  成功后才删除）。
- **跳过临时产物**：拷贝时忽略 `__pycache__`、`*.partial`、`.staging`，其余文件
  （含 `.mapserver_manifest.json`）全量保留。
- **build_no 回退**：`<map_id>.json` / `version.json` 都读不到 `build_no` 时，
  退化为当前本地时间 `YYYYMMDDHHMMSS`，保证总能产出快照目录。

### 库 API

```python
from mapserver.backup import backup_map, list_backups, restore_map

backup_map("408")                              # 快照 front/408 -> backup/408/<build_no>
backups = list_backups("408")                  # [BackupInfo, ...] 按 build_no 倒序
restore_map("408", backups[0].build_no)         # 用最新备份覆盖回 front/408
```

## TODO / 后续

- 加 ETag / If-None-Match，让 manifest 也能走 CDN 缓存。
- 提供 `version.json` 自动 bump 的 helper 脚本（每次发布把 build_no += 1）。
- 多机灰度：在 manifest 里加 `rollout: ["robot-A", "robot-B"]` 字段，server 按
  客户端 IP 决定返回哪个 manifest。
