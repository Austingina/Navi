# 多模态导航地图数据文件

本文件描述 NaviAgent 使用的**多模态导航地图数据文件**的设计目标、核心概念与文件结构，
并给出一份完整的带注释样例。它是 MapEditor 的产物、也是运行时（`MapManager` / 导航 FSM）
消费的输入。

> 关联文档：
> - `NaviSkill/docs/fsm_navigation_design.md`（多模态导航 FSM 设计，本文件的§1、§4 是本设计的来源）
> - `NaviSkill/docs/submap_system.md`（运行时如何加载/使用地图：地图切换、传送门寻路、路线）
> - `MapEditor/docs/操作手册-多子地图SLAM地图制作.md`（如何用 MapEditor 制作本数据文件）

---

## 目录

1. [设计目标](#1-设计目标)
2. [主要概念](#2-主要概念)
3. [完整样例（带注释）](#3-完整样例带注释)

---

## 1. 设计目标

### 1.1 支持多模态导航（SLAM / GNSS / VLN）

地图数据文件服务于一套**多模态导航**系统：一条路线是一个**异构途经点（waypoint）序列**，
执行器**逐个 waypoint 判断类型并路由到对应导航后端**，在段与段之间自动切换导航模式。因此
同一条路线可以同时包含三种导航：

| waypoint 类型 | 数据形态 | 导航模式 | 适用区域 |
|---------------|----------|----------|----------|
| **map 坐标 / 语义点** | `{slam_map, submap, semantic_point}` 或 `{x, y}` | **SLAM** | 某张 SLAM 点云地图覆盖范围内（室内为主） |
| **经纬度（GPS）坐标** | `{lon, lat}` | **GNSS** | 室外、无 SLAM 覆盖但有 GPS 信号的区域 |
| **VLN 描述性指令** | `{instruction: "自然语言"}` | **VLN** | 任意（实时感知，不依赖预存地图） |

> 例：一条「室外 GPS 段（GNSS）→ 进楼后定点段（SLAM）→ 末端『在会议桌旁停下』（VLN）」的路线，
> 会被切成 3 段依次执行并在段间切换后端。

### 1.2 同时支持室内 SLAM 导航与室外 GPS 导航

- **室内 SLAM 导航**：机器人在一张已建好的 SLAM 点云地图里，依赖 `map → base_link` 的 TF 定位，
  目标点来自地图文件中预定义的**语义点位 / 路线**。一片连续可 SLAM 定位的区域（如一栋楼）是一张
  **SLAM 地图**，其内部再用多个 **submap** 组织（分层、按需加载、跨 submap 用 `transform` 对齐）。
- **室外 GPS 导航**：整个大范围场景不可能建成一张连续 SLAM 点云地图（建图/定位代价、漂移累积、
  维护困难），室外更适合用**经纬度坐标系**描述。因此以 **OpenStreetMap（OSM）经纬度主干**表达室外
  道路 / 人行道网络与可通行区域，走 GPS waypoint 导航。

### 1.3 用分层地图模型把两者统一起来

为兼顾「室外大范围经纬度」与「室内多张独立 SLAM 地图」，数据文件采用**三层分层地图模型**，
以 OSM（WGS84 经纬度）为主干，把各张 SLAM 地图**锚定（对齐）到 OSM** 上：

```
Scene（场景，WGS84 / OSM 主干）
 ├── OSM 底图         —— 经纬度道路/人行道网 → GNSS 路线与可通行区域
 ├── SLAM 地图 A（geo_anchor 锚定到 OSM）   ← 一栋楼 / 一片连续 SLAM 区域
 │     ├── submap: 2f（TF 树根 = default_submap）
 │     └── submap: 1f（parent = 2f）
 └── SLAM 地图 B（geo_anchor 锚定到 OSM）
       └── submap: ...
```

| 层级 | 坐标系 | 角色 | 对应导航 |
|------|--------|------|----------|
| **Scene（场景）** | WGS84 经纬度（OSM） | 主干：OSM 道路网、跨地图全局路线、各 SLAM 地图的地理摆放 | GNSS（室外） |
| **SLAM 地图（slam_map）** | 各自局部 `map` 系（点云地图） | 一片连续可 SLAM 定位的区域；用 `geo_anchor` 锚定到 OSM | SLAM（室内） |
| **子地图（submap）** | `map` 系（相对所属 SLAM 地图的 `default_submap`） | 语义点位、传送门、跨 submap `transform` | SLAM |

**向后兼容**：一张单独的 SLAM 地图文件（无 `scene` / `slam_maps` 顶层键）视为「只有一张 SLAM 地图的
退化场景」，老配置零改动继续工作。

---

## 2. 主要概念

数据文件自上而下分为两大部分：**场景层（Scene）** 与 **SLAM 地图层（slam_map，内含 submap）**。

### 2.1 场景地图（Scene）相关概念

场景是最外层容器，以经纬度为主干，承载 OSM 底图、场景级语义点位/区域、场景传送门与跨地图全局路线。

#### OSM 底图（`scene.osm`）

- 整个场景的**地理主干**：来自 OpenStreetMap 的道路 / 人行道 / 建筑 / 水体等（经纬度 WGS84）。
- 数据源为 `.osm`（XML）或 `.osm.pbf`（二进制），文件复制进场景目录，`source` 用**相对路径**引用。
- `bounds` 给出经纬度范围（min/max）。
- 场景 `origin`（经纬度）是本地 ENU 平面换算的基准；未显式设置时取 OSM 边界中心。

#### geo_anchor（SLAM 地图 → OSM 的锚定）

每张 SLAM 地图带一个 `geo_anchor`，描述其局部 `map` 原点（即 `default_submap` 原点）落在 WGS84 的位姿，
从而把整张 SLAM 地图「贴」到 OSM 上：

| 字段 | 含义 |
|------|------|
| `origin.lon` / `origin.lat` / `origin.alt` | `map` 原点对应的经纬度 / 海拔 |
| `heading_deg` | `map` 系 +X 轴相对 ENU 正东（East）的方位角（逆时针为正） |
| `scale` | 缩放（通常为 1，用于弥补尺度差异） |

- 完整的「submap 点 → 经纬度」链路 = **submap→default_submap**（`transform`）叠加
  **default_submap→WGS84**（`geo_anchor`）。
- 既能把 SLAM 内点位投影到 OSM 做全局展示 / 跨地图规划，也能把 OSM/GPS 位置反算到某张 SLAM 地图的
  `map` 系，用于**进入 SLAM 区时的初始定位**。

> 在 MapEditor 里，`geo_anchor` 通过在 OSM 底图上**拖拽 / 旋转 / 缩放** SLAM 地图的正交投影足迹得到；
> 确认后可**锁定（`locked`）**，`ortho_visible` 控制其在总览里是否叠加显示。

#### 场景语义点位（`scene.semantic_points`）与区域（`area`）

标注在 **OSM 底图**上的命名位置（园区出入口、广场、路口等），用**经纬度 + 航向角**表示（区别于 SLAM
内部语义点用局部坐标）：

| 字段 | 含义 |
|------|------|
| `lon` / `lat` / `alt` | 经纬度 / 海拔 |
| `heading_deg` | 航向角：与正北的夹角，顺时针为正（0=正北，90=正东） |
| `type` | 类型，与 SLAM 语义点位一致（默认 `single_pose`） |
| `need_inspect` | 是否需要巡检（默认 `false`） |
| `area` | 所属**区域**名（空=未归属） |

- **区域（area）**：对场景语义点位做**分组聚合管理**（如「智城」「彩虹」两个园区），仅用于组织，不影响坐标；
  区域清单单独列在 `scene.semantic_point_areas`。

#### 场景传送门（`scene.portals`）

把一个**场景语义点位**定义为**某张 SLAM 地图的入口**（即“从园区地理层进入某栋楼的 SLAM 地图”）。

- 以**语义点位为标识**（键即场景语义点位名），无需单独命名。
- 值为 `{ "target_slam_map": "<SLAM 地图 id>" }`。

#### 全局路线（顶层 `routes`）

可**跨多张 SLAM 地图 / OSM 底图**的一串有序途经点（区别于单张 SLAM 地图内部的“内部路线”）。
途经点是**异构**的：

| 途经点类型 | JSON 形态 | 导航模式 |
|-----------|-----------|----------|
| 场景语义点位 | `{"scene_point": "名称"}` | SLAM（进入对应 SLAM 地图）/ 定位锚点 |
| 地图内点位 | `{"slam_map", "submap", "semantic_point"}` | SLAM |
| **内部路线段** | `{"slam_map", "route"}`（把某 SLAM 地图一条内部路线整体作为一段） | SLAM |
| GPS 点 | `{"lon", "lat"}` | GNSS |
| 指令 | `{"instruction": "..."}` | VLN |

### 2.2 SLAM 底图（slam_map）相关概念

`slam_maps.<id>` 是一张独立的 SLAM 地图（一栋楼 / 一片连续 SLAM 区域），键 `<id>` 即其**唯一标识（code）**，
全局路线与场景传送门都以它引用。除 `geo_anchor` 与场景级附加项（`name`/`pcd_base`/`ortho_visible`/`locked`）
外，其内部结构与单张 SLAM 地图工程（`multifloor.json` 风格）**完全一致**。

#### 子地图（submap）

SLAM 地图的**最小可加载单元**，通常对应一个楼层 / 一个区域 / 一次连续 SLAM 建图的结果：

| 字段 | 含义 |
|------|------|
| `map_file` | 该子地图的点云文件（`.pcd`，相对 `pcd_base` 目录） |
| `name` / `description` | 人类可读名称与描述 |
| `parent_submap` | 父子地图 id（为空表示它是 TF 树的**根**，即 `default_submap`） |
| `transform` | 该子地图**本地坐标系 → 父坐标系**的刚体变换 |
| `semantic_points` | 该子地图内的语义点位 |
| `submap_portals` | 从该子地图通往其它子地图的传送门 |

#### TF 树、`transform` 与 `default_submap`

多个 submap 通过**父子关系**组成一棵 **TF 树**：

- 每个 submap 的 `transform` 表示**本地坐标 → 父坐标**的刚体变换：`translation`（平移）+ `rotation`（四元数）。
- 根节点（`default_submap`）无父节点，`transform` 为单位阵，其坐标系即该 SLAM 地图的**全局 `map` 系**。
- 任意 submap 局部点到根系的变换 = 沿 TF 树逐级级联（`submapToRootMatrix`）：

$$
T_{sub \to root} = T_{parent \to root}\cdot T_{sub \to parent}
$$

- **只有 TF 树的根节点才能设为 `default_submap`**；`geo_anchor` 锚定的正是该根系原点。

#### 语义点位（submap 内，局部坐标）

子地图内一个**命名的导航目标 / 关键位姿**，坐标是该子地图的**局部坐标**：

- `type`：`single_pose`（单目标位姿，最常用）、`multi_pose`（一串按顺序经过的位姿）、
  `elevator_call` / `elevator_wait` / `elevator_in` / `elevator_out`、`door_open`、`LowPrecision_pose` 等。
- 位姿：`single_pose` 用 `pose = { position{x,y,z}, orientation{x,y,z,w} }`；
  `multi_pose` 用 `poses`（位姿数组）。
- `need_inspect`：是否需要巡检（`single_pose` 常为 `true`，其它类型常为 `false`）。

#### 传送门（submap 间）

**两个子地图之间的连接点**，用于跨子地图（跨楼层）导航时“切图”：

- 由 `portal`（一个**语义点位名**，即传送点的物理位置）+ `submap`（**目标子地图 id**）组成。
- **单向**：`1f → 2f` 与 `2f → 1f` 需**分别定义**；应设在两图**物理重叠**处（电梯口、楼梯口、坡道口）。

#### 内部路线（slam_map 内 `routes`）

单张 SLAM 地图内部的一串有序途经点，可跨该地图内的多个 submap：

- 每个 waypoint 引用 `{submap, semantic_point}`；也可**嵌套引用另一条内部路线**（`{"route": "名称"}`）实现复用。
- `global.default_route` 指定默认内部路线。
- 内部路线可被**顶层全局路线**以 `{"slam_map", "route"}` 整体引用为一段（见 §2.1 全局路线）。

#### 版本号与构建号

- `global.version`：语义版本号（如 `1.0.1`），手动维护。
- `global.build_no` 与顶层 `scene.build_no`：格式均为 `YYYYMMDDHHMMSS`，**仅在对应内容有更新时**才在保存时
  刷新时间戳，供下游比对“是否有新版本”。

---

## 3. 完整样例（带注释）

下面是一份完整场景文件的**带注释**示例（为便于讲解使用了 `//` 注释，属 JSONC 风格；
**真实文件为严格 JSON，不含 `//` 注释**，仅可保留可选的 `_comment` 字段）。
示例包含：1 张 OSM 底图 + 2 张 SLAM 地图（`buildingA` 多楼层、`buildingB` 单层）+ 场景语义点位/区域/传送门
+ 一条同时用到多种导航的跨地图全局路线。

```jsonc
{
  // ========================= 场景层（Scene）=========================
  "scene": {
    "name": "示例园区",
    "crs": "WGS84",                       // 坐标参考系：WGS84 经纬度
    "build_no": 20260721204312,           // 场景构建号（内容有更新才刷新）

    "origin": {                           // 本地 ENU 换算基准（经纬度）
      "lon": 113.9301,
      "lat": 22.5302
    },

    "osm": {                              // OSM 底图
      "source": "osm/park.osm.pbf",       // 相对场景目录的路径（.osm 或 .osm.pbf）
      "bounds": {                         // 经纬度范围
        "min": { "lon": 113.9280, "lat": 22.5290 },
        "max": { "lon": 113.9340, "lat": 22.5330 }
      }
    },

    "semantic_point_areas": [ "A区", "B区" ],   // 场景语义点位的区域（分组）清单

    "semantic_points": {                  // 场景语义点位（标注在 OSM 底图上，经纬度 + 航向角）
      "A栋南门": {
        "type": "single_pose",           // 类型（默认 single_pose）
        "area": "A区",                    // 所属区域（对应 semantic_point_areas 之一）
        "lon": 113.9301,
        "lat": 22.5302,
        "alt": 0,
        "heading_deg": 90,               // 航向角：与正北夹角，顺时针为正（0=北，90=东）
        "need_inspect": false            // 是否巡检（默认 false）
      },
      "B栋北门": {
        "type": "single_pose",
        "area": "B区",
        "lon": 113.9318,
        "lat": 22.5316,
        "alt": 0,
        "heading_deg": 270,
        "need_inspect": false
      },
      "园区西广场": {                       // 未归属区域（无 area 字段）的场景点
        "type": "single_pose",
        "lon": 113.9290,
        "lat": 22.5300,
        "alt": 0,
        "heading_deg": 0,
        "need_inspect": true
      }
    },

    "portals": {                          // 场景传送门：场景语义点位 → 某张 SLAM 地图入口
      "A栋南门": { "target_slam_map": "buildingA" },   // 键 = 场景语义点位名（即传送门标识）
      "B栋北门": { "target_slam_map": "buildingB" }
    }
  },

  // ==================== SLAM 地图层（slam_maps）====================
  "slam_maps": {

    // ---------- SLAM 地图 A：多楼层（内联 submaps）----------
    "buildingA": {                        // 键 = SLAM 地图唯一标识 code（被 routes / portals 引用）
      "name": "A 栋",                     // 人类可读全名
      "pcd_base": "buildingA",           // 解析各 submap map_file 的基准目录（相对场景目录）
      "ortho_visible": true,             // 在场景总览里是否叠加显示其正交投影（编辑器 UI 状态）
      "locked": true,                    // 是否锁定对齐结果（锁定后不可单独拖动，编辑器 UI 状态）

      "geo_anchor": {                    // 把本 SLAM 地图 map 原点锚定到 OSM
        "origin": { "lon": 113.9301, "lat": 22.5302, "alt": 0 },
        "heading_deg": 37.5,             // map +X 轴相对 ENU 正东的方位角（逆时针为正）
        "scale": 1                       // 缩放（一般为 1）
      },

      "global": {                        // 本 SLAM 地图的全局设置（等同单图 multifloor.json 的 global）
        "default_submap": "2f",          // TF 树根 = geo_anchor 锚定的坐标系
        "default_route": "A栋巡检",       // 默认内部路线
        "version": "1.0.1",
        "build_no": 20260717160629       // 本 SLAM 地图构建号（其内部信息有更新才刷新）
      },

      "submaps": {                       // 子地图集合（TF 树）
        "2f": {                          // 根子地图（parent_submap 省略）
          "name": "2楼",
          "description": "",
          "map_file": "2f.pcd",          // 相对 pcd_base 的点云文件
          "transform": {                 // 根节点：本地→父=单位阵
            "translation": { "x": 0, "y": 0, "z": 0 },
            "rotation":    { "x": 0, "y": 0, "z": 0, "w": 1 }
          },
          "semantic_points": {
            "电梯口": {
              "type": "single_pose",
              "pose": {                  // single_pose：单个位姿（局部坐标）
                "position":    { "x": -2.0, "y": 0.0, "z": 0.0 },
                "orientation": { "x": 0, "y": 0, "z": 0, "w": 1 }   // 朝向四元数
              },
              "need_inspect": false
            },
            "大厅入口": {
              "type": "multi_pose",      // multi_pose：一串按顺序经过的位姿（用 poses 数组）
              "poses": [
                { "position": { "x": 5.0, "y": 1.0, "z": 0 },
                  "orientation": { "x": 0, "y": 0, "z": 0, "w": 1 } },
                { "position": { "x": 7.0, "y": 1.0, "z": 0 },
                  "orientation": { "x": 0, "y": 0, "z": 0.7071, "w": 0.7071 } }
              ],
              "need_inspect": false
            }
          },
          "submap_portals": [            // 从 2f 通往其它 submap 的传送门（单向）
            { "portal": "电梯口", "submap": "1f" }   // portal=本图语义点名, submap=目标子地图
          ]
        },

        "1f": {                          // 子节点：挂在 2f 下
          "name": "1楼",
          "description": "",
          "map_file": "1f.pcd",
          "parent_submap": "2f",         // 父子地图 id
          "transform": {                 // 本地(1f)→父(2f) 的刚体变换
            "translation": { "x": -1.0688, "y": -1.484, "z": -3.9833 },
            "rotation":    { "x": 0.0076, "y": 0.0015, "z": -0.0056, "w": 0.99995 }
          },
          "semantic_points": {
            "充电桩": {
              "type": "single_pose",
              "pose": {
                "position":    { "x": 48.0, "y": -31.46, "z": 0 },
                "orientation": { "x": 0, "y": 0, "z": 1, "w": 0 }
              },
              "need_inspect": true
            },
            "电梯口": {
              "type": "single_pose",
              "pose": {
                "position":    { "x": 0.0, "y": 0.0, "z": 0 },
                "orientation": { "x": 0, "y": 0, "z": 0, "w": 1 } },
              "need_inspect": false
            }
          },
          "submap_portals": [
            { "portal": "电梯口", "submap": "2f" }   // 反向传送门（与 2f→1f 成对，双向需各建一条）
          ]
        }
      },

      "routes": {                        // 本 SLAM 地图的内部路线（跨其内部 submap）
        "A栋巡检": {
          "description": "1楼充电桩 → 电梯 → 2楼大厅",
          "waypoints": [
            { "submap": "1f", "semantic_point": "充电桩" },
            { "submap": "1f", "semantic_point": "电梯口" },
            { "submap": "2f", "semantic_point": "大厅入口" }
          ]
        },
        "A栋大厅片段": {
          "description": "可被复用/被全局路线整体引用的片段",
          "waypoints": [
            { "submap": "2f", "semantic_point": "电梯口" },
            { "submap": "2f", "semantic_point": "大厅入口" }
          ]
        }
      }
    },

    // ---------- SLAM 地图 B：单层（结构相同，从略）----------
    "buildingB": {
      "name": "B 栋",
      "pcd_base": "buildingB",
      "ortho_visible": true,
      "locked": false,
      "geo_anchor": {
        "origin": { "lon": 113.9318, "lat": 22.5316, "alt": 0 },
        "heading_deg": -12.0,
        "scale": 1
      },
      "global": { "default_submap": "1F", "version": "1.0" },
      "submaps": {
        "1F": {
          "name": "1楼",
          "map_file": "1F.pcd",
          "transform": {
            "translation": { "x": 0, "y": 0, "z": 0 },
            "rotation":    { "x": 0, "y": 0, "z": 0, "w": 1 }
          },
          "semantic_points": {
            "前台": {
              "type": "single_pose",
              "pose": { "position": { "x": 3.2, "y": -1.0, "z": 0 },
                        "orientation": { "x": 0, "y": 0, "z": 0, "w": 1 } },
              "need_inspect": true
            }
          },
          "submap_portals": []
        }
      }
    }
  },

  // ==================== 顶层：跨地图全局路线 ====================
  "routes": {
    "周界到会议室巡检": {
      "description": "混合路线：室外GPS段(GNSS) → 进楼定点段(SLAM) → 末端VLN段(VLN)",
      "waypoints": [
        { "scene_point": "园区西广场" },                                  // ① 场景语义点位（OSM 上）
        { "lon": 113.9305, "lat": 22.5308 },                             // ② GPS 点 → GNSS 段
        { "scene_point": "A栋南门" },                                     // ③ 场景传送门点 → 进入 buildingA
        { "slam_map": "buildingA", "route": "A栋大厅片段" },              // ④ 整体引用某内部路线为一段
        { "slam_map": "buildingA", "submap": "1f", "semantic_point": "充电桩" }, // ⑤ 地图内点位 → SLAM 段
        { "instruction": "进门后在会议桌旁停下，面向白板" }                 // ⑥ VLN 描述性指令 → VLN 段
      ]
    }
  }
}
```

**样例要点回顾：**

- `scene` 是主干：OSM 底图 + 场景语义点位（经纬度+航向角，按 `area` 分组）+ 场景传送门（点位→SLAM 地图）。
- `slam_maps.<code>` 各带 `geo_anchor` 锚定到 OSM；内部是标准的多 submap 结构（TF 树 + 语义点位 + 传送门 + 内部路线）。
- submap 的 `transform` 是「本地→父」，根子地图（`default_submap`）为单位阵；传送门单向、需成对建立。
- 顶层 `routes` 是跨地图全局路线，途经点异构：`scene_point` / `{slam_map,submap,semantic_point}` /
  `{slam_map,route}` / `{lon,lat}` / `{instruction}`，分别路由到 SLAM / GNSS / VLN 后端。
- SLAM 地图也可**以引用方式**嵌入：用 `slam_maps.<id>.source` 指向独立 SLAM 地图 JSON 文件，
  由运行时就地展开其 `default_submap` / `submaps`（详见 `fsm_navigation_design.md` §4.3）。
