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
