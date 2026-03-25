# 多楼层点云逐层分割——算法设计文档

**版本：** 1.0 | **日期：** 2026-03-19 | **实现语言：** Python 3.10+ (Open3D / scipy / numpy)

## 概述

本文档描述一套将大型多层建筑 LiDAR 点云自动分割为逐层 submap 的算法 pipeline。

**核心思路：** 先屏蔽楼梯间等垂直连通结构，再通过 Z-histogram 分析定位楼层边界，最后用 connected-component labeling 清理分割结果并回填楼梯间点。

### 适用场景

- **输入**：Terrestrial LiDAR（如 Share S20）对多层建筑的整体扫描，任意 Open3D 支持的格式（PLY、PCD、XYZ 等）或 numpy 数组
- **输出**：每层独立的 PLY 点云文件（`floor_00_z{lo}_{hi}.ply`, `floor_01_z{lo}_{hi}.ply`, ...）
- **目标建筑**：标准多层商业/机构建筑（2–10 层），含楼梯间和电梯井
- **点云规模**：数千万至数亿点

### 前提假设

1. 建筑各层楼板大致水平（允许小幅倾斜，但非坡道式结构）
2. 层高在 2.5–4.0 m 范围内且各层大致一致
3. 楼梯间和电梯井的 XY 投影面积在 4–100 m² 范围内
4. 输入点云已完成多站配准，坐标系 Z 轴朝上

---

## 项目文件结构

所有楼层分离相关的代码、文档集中在 `floor_separator/` 目录下：

```
rerun_colorized_lidar_demo/
├── main.py                            # Potree v2 数据加载工具（共用）
├── pyproject.toml                     # 项目依赖 + 打包配置 + CLI entry point
└── floor_separator/                   # 楼层分离：算法 + 入口 + 文档
    ├── __init__.py                    # 包入口，暴露 FloorSeparatorConfig, run_pipeline
    ├── __main__.py                    # 支持 python -m floor_separator 运行
    ├── pipeline.py                    # Pipeline 编排器，串联 Phase 0–4，定义配置与结果数据结构
    ├── preprocessing.py               # Phase 0: Open3D voxel 降采样（GPU 可用时加速）+ 法线估计
    ├── stairwell_detector.py          # Phase 1: XY 网格 voxel-column 分析 + scipy CCL
    ├── floor_detector.py              # Phase 2: Normal filtering + Z-histogram + peak/valley 检测
    ├── segmentation.py                # Phase 3: Z-slice 切割 + 并行 6-connected 3D CCL 清理
    ├── export.py                      # Phase 4: 楼梯间点 Z 值回填 + 并行逐层 PLY 导出
    ├── cli.py                         # 通用 CLI：读 PLY/PCD → 分层 → 导出
    ├── multi_floor.py                 # Demo：加载 Potree 数据 + Rerun 可视化（非核心依赖）
    ├── tests/                         # pytest 单元测试和集成测试
    └── floor_seperator_design.md      # 算法设计文档（本文件）
```

### 关键接口

`FloorSeparatorConfig`（`pipeline.py`）— 所有可调参数的统一配置 dataclass，各 Phase 参数分组管理。

`run_pipeline(positions, colors, config) → FloorSeparatorResult`（`pipeline.py`）— 主入口函数。

- 输入：`positions` (N, 3) float 点坐标，`colors` (N, 3) uint8 RGB（可选），`config` 配置对象
- 输出：`FloorSeparatorResult` 包含：
  - `floor_labels` (N,) int — 每个点的楼层 ID
  - `cut_planes` (K,) float — 切割平面 Z 高程
  - `n_floors` int — 检测到的楼层数
  - `noise_mask` (N,) bool — 被 CCL 丢弃的噪声点
  - `exported_files` — 导出的 PLY 文件路径列表
  - 各阶段中间结果（用于 debug 可视化）

**各阶段模块的输入输出：**


| 模块                      | 输入                                    | 输出                                                    |
| ----------------------- | ------------------------------------- | ----------------------------------------------------- |
| `preprocessing.py`      | 原始 (N,3) 坐标 + 颜色                      | `PreprocessedCloud`：降采样坐标 (M,3)、法线 (M,3)、颜色           |
| `stairwell_detector.py` | 降采样坐标 (M,3)                           | `StairwellMaskResult`：clean_mask、stairwell_mask、区域元数据 |
| `floor_detector.py`     | 清理后坐标 + 法线                            | `FloorBoundaryResult`：cut_planes、histogram、峰值、诊断数据    |
| `segmentation.py`       | 全分辨率坐标 (N,3) + cut_planes             | `SegmentationResult`：floor_labels、noise_mask          |
| `export.py`             | floor_labels + stairwell_mask + 坐标/颜色 | 更新后的 labels + 逐层 PLY 文件                               |


### 依赖


| 包        | 用途                                                          |
| -------- | ----------------------------------------------------------- |
| `open3d` | Voxel 降采样、法线估计、PLY 导出                                       |
| `scipy`  | `ndimage.label`（2D/3D CCL）、`gaussian_filter1d`、`find_peaks` |
| `numpy`  | 全部数值计算                                                      |
| `loguru` | 结构化日志                                                       |


### 使用方式

`floor_separator` 是一个纯 Python 包，输入为 numpy 数组，不绑定任何特定的数据格式或可视化工具。

**安装依赖：**

```bash
cd rerun_colorized_lidar_demo
uv sync
```

**命令行直接运行（推荐）：**

```bash
# 自动检测楼层，导出逐层 PLY
uv run python floor_separator/cli.py building.ply -o ./output

# 指定预期楼层数和层高
uv run python floor_separator/cli.py scan.pcd --num-floors 5 --expected-floor-height 3.5 -o ./output

# 禁用楼梯间检测和 CCL 清理（加速）
uv run python floor_separator/cli.py building.ply --skip-stairwell-mask --no-ccl -o ./output
```

支持所有 Open3D 可读取的格式（PLY、PCD、XYZ、OBJ 等）。

**在 Share S20 原始扫描数据上运行：**

S20 扫描数据经 PotreeConverter 转换后为 Potree v2 格式（含 `metadata.json` + `octree.bin`），需使用 `multi_floor.py`（内含 Potree 解码器 + Rerun 可视化）：

```bash
# 自动检测 + 导出逐层 PLY + 保存 Rerun 录制文件
uv run python floor_separator/multi_floor.py \
    --data-dir /path/to/2026-01-27_15-50-02_colorized_las_converted \
    --output-dir ./output/per_floor_maps \
    --save multi_floor.rrd

# SSH 远程查看（需转发 9090 + 9876 端口）
uv run python floor_separator/multi_floor.py \
    --data-dir /path/to/2026-01-27_15-50-02_colorized_las_converted \
    --output-dir ./output/per_floor_maps \
    --serve --port 9090 --debug
```

已验证的实际运行示例（199M 点，4x 降采样，自动检测 7 层，~10s 完成）：

```bash
uv run python floor_separator/multi_floor.py \
    --data-dir /home/weiying/vrr/VPR-codebase/2026-01-27_15-50-02_colorized_las_converted \
    --output-dir /tmp/floor_output \
    --save /tmp/floor_output.rrd
```

**Python API 调用：**

```python
from pathlib import Path
import numpy as np
from floor_separator import FloorSeparatorConfig, run_pipeline

positions = np.load("my_pointcloud_xyz.npy")   # (N, 3) float
colors = np.load("my_pointcloud_rgb.npy")       # (N, 3) uint8 或 None
config = FloorSeparatorConfig(
    num_floors=5,
    expected_floor_height=3.0,
    output_dir=Path("./output"),
)
result = run_pipeline(positions, colors, config)
print(f"{result.n_floors} floors, cut_planes={result.cut_planes}")
```

** `FloorSeparatorConfig` 完整参数表：**

| 参数 | 默认值 | Phase | 说明 |
|------|--------|-------|------|
| `voxel_size` | 0.10 | 0 | 分析云降采样 voxel 大小（米） |
| `normal_knn` | 30 | 0 | 法线估计邻居数 |
| `expected_floor_height` | 3.0 | 1 | 预期层高（米），影响 Z-span 阈值 |
| `stairwell_grid_cell` | 0.5 | 1 | XY column 分析网格大小（米） |
| `stairwell_z_bin` | 0.15 | 1 | 柱 Z 方向 bin 大小（米） |
| `stairwell_min_col_pts` | 20 | 1 | 忽略点数少于此值的柱 |
| `stairwell_occupancy_thresh` | 0.5 | 1 | 连续分布判定阈值 |
| `stairwell_min_area` | 4.0 | 1 | 楼梯间最小面积（m²） |
| `stairwell_max_area` | 100.0 | 1 | 楼梯间最大面积（m²） |
| `skip_stairwell_mask` | `False` | 1 | 跳过楼梯间检测 |
| `num_floors` | `None` | 2 | 预期楼层数，`None` = 自动检测 |
| `histogram_bin_size` | 0.05 | 2 | Z-histogram bin 宽度（米） |
| `smooth_sigma_bins` | 3.0 | 2 | Gaussian 平滑 σ（以 bin 为单位） |
| `normal_filter_angle_deg` | 15.0 | 2 | Normal pre-filter 角度阈值（度） |
| `min_floor_height` | 2.0 | 2 | 物理层高下限（米） |
| `max_floor_height` | 5.0 | 2 | 自动模式下单层最大高度（米） |
| `peak_height_frac` | 0.10 | 2 | 最小峰值高度（全局最大值的比例） |
| `peak_prominence_frac` | 0.10 | 2 | 最小峰值 prominence（全局最大值的比例） |
| `ccl_voxel_size` | 0.15 | 3 | 3D occupancy grid voxel 大小（米） |
| `ccl_min_component` | 100 | 3 | 最小 component 大小（voxel 数） |
| `enable_ccl` | `True` | 3 | 启用 connected-component 清理 |
| `boundary_margin` | 0.0 | 4 | 楼梯间边界复制距离（米），0 = 不复制 |
| `output_dir` | `None` | 4 | 逐层 PLY 导出目录，`None` = 不导出 |

**`FloorSeparatorResult` 输出字段：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `floor_labels` | `(N,) int` | 每个点的楼层 ID（0-indexed） |
| `cut_planes` | `(K,) float` | 楼层间切割平面 Z 高程 |
| `n_floors` | `int` | 检测到的楼层总数 |
| `floor_z_ranges` | `(n_floors, 2) float` | 每层的 [z_min, z_max] |
| `noise_mask` | `(N,) bool` | 被 CCL 标记为噪声的点 |
| `exported_files` | `list[Path]` | 导出的 PLY 文件路径列表 |


---

## Pipeline 总览

```
原始点云
   │
   ▼
┌─────────────────────┐
│  Phase 0: 预处理     │  验证 → 降采样(GPU/CPU) → 法线估计
└─────────┬───────────┘
          │
          ▼
┌─────────────────────┐
│  Phase 1: 楼梯间检测 │  voxel-column 分析 → 2D 连通域聚类 → 分离楼梯间点
└─────────┬───────────┘
          │  输出: cloud_clean + cloud_stairwell
          ▼
┌─────────────────────┐
│  Phase 2: 楼层边界   │  normal filtering → Z-histogram → peak/valley 检测
│          检测        │  → 自动/手动楼层数 → cut planes
└─────────┬───────────┘
          │  输出: N-1 个 cut plane 高程
          ▼
┌─────────────────────┐
│  Phase 3: 逐层分割   │  Z-slice 切割 → 3D voxel 化 → 6-connected CCL → 尺寸过滤
└─────────┬───────────┘
          │
          ▼
┌─────────────────────┐
│  Phase 4: 楼梯间回填 │  按 Z 值分配楼梯间点 → 可选边界复制
└─────────┬───────────┘
          │
          ▼
   逐层点云输出
```

---

## Phase 0: 预处理

**目标：** 将含噪声的原始点云清洗为干净、密度均衡的数据集，并计算法线信息。

### 步骤

| 步骤 | 操作 | 说明 |
|------|------|------|
| 0.1 | 输入验证 | 检查数组形状 (N,3)、非空、无 NaN |
| 0.2 | Voxel 降采样 | 默认 0.10 m（GPU 加速可用时自动启用），均衡密度差异 |
| 0.3 | 法线估计 | KNN=30，为后续 normal filtering 提供数据 |

### 关键设计决策

**为何降采样？** 分析云均衡了不同楼层因扫描站数不同导致的密度差异，使 histogram 峰值可比较。全分辨率原始点云保留在内存中用于最终的 Z-slice 分割和 PLY 导出。

**GPU 加速策略：** Phase 0 的 voxel 降采样在 GPU 可用时通过 Open3D Tensor API (CUDA) 加速（47M 点 0.09s vs CPU 4.17s）。法线估计保留在 CPU，因为 Open3D 的 GPU KNN 实现在当前版本上比 CPU KDTree 更慢。

---

## Phase 1: 楼梯间检测与屏蔽

**目标：** 在楼层边界检测之前识别并排除楼梯间和电梯井区域。

**为何这一步至关重要？** 楼梯间和电梯井在 Z 方向上产生连续的点分布，若不排除，会在 Z-histogram 中产生覆盖多个楼层的弥散信号，严重降低峰值的分辨能力，导致楼层边界检测失败。

### 算法逻辑

**1) 构建 XY 网格垂直柱**

将点云的 XY 范围按 0.5 m cell 划分网格，每个 cell 内的点构成一根垂直"柱"。

**2) 分析每根柱的 Z 分布特征**

对每根柱计算两个指标：

- **Z-span** = `max(z) - min(z)`：垂直跨度
- **Occupancy ratio**：将 Z 值按 0.15 m 分 bin，统计有 ≥3 点的 bin 占总 bin 数的比例

判定逻辑：


| 条件                                              | 含义                                |
| ----------------------------------------------- | --------------------------------- |
| `z_span > 1.5 × 预期层高` 且 `occupancy_ratio > 0.5` | 标记为 vertical transition candidate |


**核心区分原理：** 普通楼层的柱在 2–3 个离散高度（楼板、天花板）有点聚集，occupancy ratio 低；楼梯间的柱因台阶连续分布，occupancy ratio 高。

**3) 2D 连通域聚类并按面积过滤**

将标记的 cell 投影为 2D binary mask → 8-connected component labeling → 面积过滤：


| 类型      | 面积范围     | 处理       |
| ------- | -------- | -------- |
| 噪声（家具等） | < 4 m²   | 丢弃       |
| 楼梯间     | 6–30 m²  | 保留       |
| 电梯井     | 4–10 m²  | 保留       |
| 中庭      | > 100 m² | 丢弃（另行处理） |


**区分中庭与楼梯间的辅助判据：** XY 面积 >50 m² 且无规律间距（0.15–0.20 m）的阶梯状水平面 → 中庭。中庭特征为中部稀疏、顶底密集；楼梯间为连续阶梯几何。

**4) 分离点云**

使用 2D mask 将点云拆为 `cloud_clean`（非楼梯区域）和 `cloud_stairwell`（楼梯区域），后者暂存待 Phase 4 回填。

---

## Phase 2: Z-histogram 楼层边界检测

**目标：** 从清理后的点云中定位各楼层之间的切割平面高程。

### 算法逻辑

**1) Normal pre-filtering（可选但强烈推荐）**

仅保留法线接近垂直的点（`|normal_z| > cos(15°) ≈ 0.966`），即楼板和天花板表面。这将墙壁、家具、管线等杂物完全排除，使 histogram 峰值显著更尖锐。

> 这是对基础 histogram 分析**最有效的单个增强手段**（Oesau et al., 2013）。

**2) 构建 Z-histogram**

将所有点按 Z 坐标投入 bin size = 0.05 m 的直方图。5 层建筑约 15 m 高度，仅需约 300 个 bin。

**3) Gaussian smoothing**

用 σ = 3 bins（15 cm）的 1D Gaussian kernel 平滑直方图，消除扫描噪声和家具小峰，保留楼板/天花板的主导峰值。

**4) Peak detection 三重过滤**

在平滑 histogram 上检测 local maxima，依次应用：


| 过滤器           | 阈值             | 作用                           |
| ------------- | -------------- | ---------------------------- |
| 最小高度          | 全局最大值的 10%     | 排除家具峰（桌面、书架等，点数远少于整层楼板）      |
| 最小 prominence | 全局最大值的 10%     | **最有效的伪峰过滤器**——排除嵌入更大峰侧翼的次级峰 |
| 最小间距          | 2.0 m（40 bins） | 物理层高下限约束                     |


5 层建筑期望约 10 个主要峰（每层一个楼板峰 + 一个天花板峰），但相邻的楼板/天花板峰常合并为一个宽峰。

**5) 谷值检测 → 切割平面**

在每对相邻峰之间取平滑值最小的 bin，其 Z 坐标即为切割平面。5 层建筑期望 4 个切割平面。

**6) 楼层数验证与自适应**


| 情况     | 策略                                               |
| ------ | ------------------------------------------------ |
| 切割平面过多 | 保留分隔质量最好（最深）的 `num_floors - 1` 个谷值               |
| 切割平面过少 | 放松 prominence 阈值 50% 重新检测；或按预期层高 2.5–4.0 m 拆分合并峰 |


---

## Phase 3: 逐层分割与空间连通性清理

**目标：** 按切割平面把点云切为逐层 segment，并用连通性分析去除错误归属的点。

### 算法逻辑

**1) Z-slice 切割**

按相邻切割平面的 Z 范围，从全分辨率原始点云中提取每层点云。第一层底部和最后一层顶部取点云全局 Z 极值。

**2) 3D voxel 化**

每层点云构建 0.15 m 的 3D occupancy grid（稀疏表示）。仅 ≥3 点的 voxel 标记为 occupied。

**3) 6-connected 3D connected-component labeling**

使用 two-pass union-find 算法，**仅用 6-connectivity（面邻接）**。

> **设计关键：** 必须用 6-connectivity 而非 26-connectivity。26-connectivity 允许对角邻接，会在薄楼板缝隙处产生泄漏，将相邻楼层错误连通。6-connectivity 仅沿坐标轴方向扩展，有效隔离各层。

**4) 尺寸过滤**

丢弃 < 100 voxel 的 component（噪声簇）。每层最大 component 为主楼层结构。较小有效 component 可能是阳台或不相连房间。

**5) Label 映射**

将 component label 从 voxel 空间映射回原始点。

---

## Phase 4: 楼梯间点回填

**目标：** 将 Phase 1 中暂存的楼梯间点重新分配到正确的楼层。


| 步骤  | 操作       | 说明                             |
| --- | -------- | ------------------------------ |
| 4.1 | Z 值分配    | 按点的 Z 坐标落入哪个楼层的 Z 范围，直接归属      |
| 4.2 | 边界复制（可选） | 距切割平面 0.3 m 内的点复制到相邻两层，保证过渡连续性 |
| 4.3 | 并行导出     | 每层一个 PLY 文件（`ThreadPoolExecutor`） |


---

**最可能需要按项目调整的参数：** 预期层高（影响 Z-span 阈值）、楼梯间面积范围。

---

## 已知局限与适用边界


| 场景        | 局限性                   | 可能的缓解措施                               |
| --------- | --------------------- | ------------------------------------- |
| 倾斜楼板、坡道   | Z-histogram 假设楼板水平    | 改用 RANSAC 或 cloth simulation          |
| 夹层 / 跃层   | 层高低于 2.0 m 时峰值间距过滤会丢失 | 降低最小 peak 间距阈值                        |
| 跨多层中庭     | 中庭区域缺少楼板，切割面意义不明      | 中庭单独处理，不参与楼层分割                        |
| 吊顶 / 架空结构 | 可能在非楼板位置产生额外峰值        | Prominence filtering 可抑制大部分；严重情况需人工检查 |
| 非均匀层高     | 固定的 Z-span 阈值可能漏检/误检  | 使用自适应层高估计                             |


---

## 替代方案评估

在设计阶段评估了以下替代方法：


| 方法                                            | 优点              | 缺点                | 本 pipeline 中的角色    |
| --------------------------------------------- | --------------- | ----------------- | ------------------ |
| **纯 RANSAC 迭代**                               | 对不均匀密度更鲁棒       | 慢；可能误检桌面为楼板       | 用于 Phase 2 验证，非主方法 |
| **Region growing**                            | 适合碎片化楼板         | 产生大量小 cluster 需合并 | 复杂楼层几何时的备选         |
| **Graph cuts (supervoxel + normalized cuts)** | 最鲁棒，可处理非标准结构    | 实现复杂度高            | 非标准建筑的 fallback 方案 |
| **高度约束 CCL**                                  | 单 pass，无需预先检测边界 | 会错误拆分高大中庭         | 简单建筑的快速替代          |


**选定方案的设计理由：** Z-histogram + 楼梯间屏蔽作为主方法，因为它是 O(N) 复杂度、中间结果可解释、对标准多层建筑可靠。楼梯间屏蔽是工程实践中最关键的一步（学术文献常忽略），防止垂直过渡区将点弥散到整个 Z 范围从而破坏边界检测。

---

## 错误处理

| 场景 | 行为 | 级别 |
|------|------|------|
| `positions` 形状错误、为空或含 NaN | 抛出 `ValueError` | 入口验证 |
| `voxel_size`、`bin_size` 等参数 <= 0 | 抛出 `ValueError` | 参数验证 |
| Z 范围小于 bin_size（所有点在同一高度） | 返回 `n_floors=1`，不崩溃 | 优雅降级 |
| 未检测到楼层边界 | 返回 `n_floors=1`，日志 `ERROR` | 优雅降级 |
| GPU 不可用或 CUDA 运行失败 | 自动 fallback 到 CPU，日志 `WARNING` | 透明降级 |
| CCL grid 过大（>500M voxel） | 跳过该层清理，日志 `WARNING` | 局部跳过 |
| 某层 0 个点 | 跳过导出，日志 `WARNING` | 局部跳过 |

---

## 参考文献

- Okorn et al. (2010), 3DPVT — Z-histogram 楼板/天花板检测的奠基工作
- Oesau et al. (2014), ISPRS — Normal filtering 增强 Z-histogram 峰值
- Pexman et al. (2021), Remote Sensing — 多层分离达 94.74% 准确率
- Staats et al. (2023), VOX2BIM+, PFG — Voxel-based 楼层分割，指出 histogram 方法的敏感性
- Bidirectional cloth simulation (2024), Journal of Building Engineering — 当前 SOTA，~99% weighted F1
- Yang et al. (2019) — 点云中楼梯空间连通性检测
- Nikoohemat et al. (2020), Automation in Construction — 多层重建中的显式楼梯检测

