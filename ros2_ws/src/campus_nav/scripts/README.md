# campus_nav / scripts

离线工具脚本（数据准备 / 底图导出）。规划核心仍在上一级：`route_graph.py`、`route_planner_node.py` 等。

建议用系统 Python（与 ROS / pyosmium 一致）：

```bash
cd /home/navi/nav_test/campus_nav
/usr/bin/python3 scripts/<script>.py ...
```

| 脚本 | 作用 |
|------|------|
| `merge_pbf_gpkg.py` | 把 QGIS 编辑的 `new_road.gpkg` 烘焙进底图 PBF，产出生产用融合路网 |
| `make_entries_gpkg.py` | 生成空的入口点图层 `campus_entries.gpkg`，供 QGIS 标注广场/院子的开口 |
| `zhicheng_align.py` | L2Pro / 3DGS 点云 PLY → 俯视 GeoTIFF 底图（EPSG:32649） |

---

## merge_pbf_gpkg.py

QGIS 改完路网后跑一次，生成规划器默认读取的融合 PBF。

**依赖：** `osmium`（PyPI 包名）、numpy；会 import 上一级的 `route_graph`。

**默认路径：**

| 参数 | 默认 |
|------|------|
| `--pbf` | `<repo>/xili_5km_with_extra.osm.pbf` |
| `--gpkg` | `<campus_nav>/new_road.gpkg` |
| `--output` | `<repo>/xili_5km_merged.osm.pbf` |
| `--table` | `osmfile` |

```bash
/usr/bin/python3 scripts/merge_pbf_gpkg.py

/usr/bin/python3 scripts/merge_pbf_gpkg.py \
  --pbf ../../xili_5km_with_extra.osm.pbf \
  --gpkg ../new_road.gpkg \
  --output ../../xili_5km_merged.osm.pbf
```

合并后规划器应关闭开发叠加：

```bash
/usr/bin/python3 route_planner_node.py --ros-args \
  -p pbf_path:=/home/navi/nav_test/xili_5km_merged.osm.pbf \
  -p dev_overlay:=false
```

---

## make_entries_gpkg.py

**为什么需要它：** 终点落在广场、院子这类没有路的地方时，规划器原来只能把它吸附到**直线距离最近**的路段——那条路往往在栅栏外侧，机器人沿人行道开到栅栏边才发现进不去。栅栏在 OSM 里基本没测绘（整个 5 km 底图只有 49 条 `barrier=fence`），光靠几何判断不出哪里能进，所以开口要手工标。

标好以后，规划器对离网终点改用「入口选择」：在候选入口里取

```
路网距离(起点 → 入口) + entry_offroad_weight × (穿开口段 + 接入段)
```

最小的那个。路径尾部是三段：

```
… → 路网终点(入口在路上的投影) → 你标的原始点 → 真实终点
```

**你标的点会成为一个真实途经点**，所以点标在开口内侧几米就能指定「从这个方向进门」，机器人会先被引导穿过开口，再朝终点走——而不是从路边直接给一条到终点的直线。点如果就标在路上（离路不到 1 米），中间这个途经点会被省掉，退化成两段。

两段离网跳都是直线，**都不插中间 waypoint**（插出来的点可能正好落在栅栏上），各自由 Nav2 单独规划——Nav2 的 global costmap 是 100×100 m rolling window，配合高程图能看见栅栏。

日志会把三段分开报，便于核对：

```
Goal is 26.6 m off-network -> entering via 'plaza_west'
(614 m on roads + 8 m through the opening + 19 m access leg)
```

**依赖：** 无（只用标准库 sqlite3，不需要 GDAL/ogr）。

```bash
/usr/bin/python3 scripts/make_entries_gpkg.py
# 默认输出 <campus_nav>/campus_entries.gpkg，图层 entries，POINT，EPSG:4326
```

**QGIS 标注步骤：**

1. 图层 → 添加图层 → 添加矢量图层，选 `campus_entries.gpkg`
2. 加载 `zhicheng_ortho.tif`（见下面 `zhicheng_align.py`）作为对照底图，也可以同时加载 `finetuned_road.gpkg` 看路网
3. 切换编辑模式，在**每个开口**点一个点。可以标在人行道一侧的开口处，也可以往里挪几米来指定进门方向（往里挪的点会成为真实途经点）。但**不要标得离路超过 15 米**：预筛用的是原始坐标，太深的点可能根本不进候选
4. 保存。规划器重启即生效，**不需要重跑 `merge_pbf_gpkg.py`**

**字段（都可选）：**

| 字段 | 作用 |
|------|------|
| `name` | 出现在规划日志和 RViz 标签里，便于核对选了哪个入口 |
| `radius_m` | 覆盖该入口的作用半径；留空用 `entry_search_radius_m`（默认 40 m） |

**RViz 核对：** 加一个 MarkerArray 显示订阅 `/campus_entry_markers`，入口是青色立方体加名字标签。发一个广场上的终点，看日志里的 `entering via '<name>'` 是否选中了预期的开口。

**相关参数**（`config/route_planner.yaml`）：

| 参数 | 默认 | 说明 |
|------|------|------|
| `entry_enable` | `true` | 关掉即完全回到旧行为 |
| `off_network_m` | `5.0` | 终点离最近路段超过这个距离才启用入口模式 |
| `entry_search_radius_m` | `40.0` | 只考虑离终点这么近的入口，同时也是接入段长度上限（见下） |
| `entry_offroad_weight` | `1.0` | 大于 1 更偏向离网段短的入口，即使路网要绕远 |

**离网跳有长度上限，广场大就多标几个入口：** 两段离网跳各自一次性交给 Nav2 规划，而 `global_costmap` 是以机器人为中心的 100×100 m 滚动窗口，`NavfnPlanner` 对窗口外的目标直接抛 `GoalOutsideMapBounds`，规划失败、BT abort（重试 `nav2_final_max_retry` 次后日志 `giving up on the final pose ... stopped X m short`）。所以每一跳都必须在 50 m 内；`entry_search_radius_m` 默认取 40 就是这条硬约束留了余量后的值，**穿开口段和接入段都按它判定**，超限的入口直接不参与候选。**不要为了让远处终点能规划而把它调大**。

注意这两段距离都是从「入口在路上的投影」往外量的，不是从你点击的坐标量——点标得越靠里，穿开口段越长、接入段越短，总和不变。

结论是：广场跨度超过 ~80 m 时一个开口不够，要在多个开口都标点，保证任意可能的终点离最近入口都在 40 m 内。日志里那句 `no entry point is within 40 m of it` 就是提醒你该补点了。

注意 `allow_unknown: true` / `track_unknown_space: false` 解决的是另一个问题（激光没扫到的格子算不算可通行），管不了目标压根不在 costmap 数组里的情况。

**窄开口的兜底：** 如果某个开口太窄、Nav2 自己找不到门，可以在 `finetuned_road.gpkg` 里从人行道开口画一条伸进去的短线，属性加 `highway=footway` + `campus_entry=yes`，规划器会把它悬空那一端也当作候选入口，机器人被路网强制穿过开口。注意这条线要画得**深过 6 米**，否则 `merge_pbf_gpkg.py` 的 `node_dangling_ends_to_base(tol_m=6.0)` 会把它自动吸附连接到附近的既有道路上，"只能从开口进"的约束就失效了。这种方式需要重跑 merge。

---

## zhicheng_align.py

把智城扫描点云渲成带地理参考的俯视 GeoTIFF，供 QGIS 描路。

**依赖：** numpy、Pillow。无需 GDAL / pyproj。

**默认路径：**

| 参数 | 默认 |
|------|------|
| `ply`（位置参数） | `<repo>/zhicheng/point_cloud/iteration_100/point_cloud.ply` |
| `--out` | `<campus_nav>/zhicheng_ortho.tif` |
| `--res` | `0.2`（米/像素） |
| `--epsg` | PLY header 或 `32649` |

```bash
/usr/bin/python3 scripts/zhicheng_align.py
/usr/bin/python3 scripts/zhicheng_align.py --res 0.1 --out /tmp/ortho.tif
/usr/bin/python3 scripts/zhicheng_align.py /path/to/point_cloud.ply
```

QGIS：直接加载 `.tif`（地理参考写在文件内，无需 `.pgw`）。
