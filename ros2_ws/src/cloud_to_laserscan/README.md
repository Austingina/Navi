# cloud_to_laserscan

把 3D 点云 / 高程图压成 Nav2 能吃的 2D 观测。包内两个可执行:

| 节点 | 作用 |
|---|---|
| `cloud_to_2d_laserscan` | 纯几何:按高度带把 `PointCloud2` 切成 `LaserScan`。无地形语义,作为兜底/对照。 |
| **`terrain_scan`** | 主力。融合 `elevation_mapping_cupy` 的高程图,输出**地形分级代价** + **悬崖(负障碍)** + 2D scan。 |

本文档主要讲 `terrain_scan`,重点是 **3. 悬崖检测**——那部分的推理链最长,最容易看不懂。

---

## 1. terrain_scan 数据流

```
/livox/points ─────────────────┐
(PointCloud2, body 系)          │
                                ├─► terrain_scan ─┬─► /livox_scan2        (LaserScan, base_link 拍平)
/grid_map_from_raw_pointcloud ──┘                 ├─► /terrain_cost_cloud (PointCloud2 + cost 字段, odom 系)
(GridMap: elevation,                              ├─► /terrain_grid_map   (GridMap, 含全部中间层, 给 RViz/离线扫参)
 upper_bound, is_upper_bound)                     └─► /slope_costmap      (OccupancyGrid, 可视化用)
```

`/terrain_cost_cloud` 是唯一进 Nav2 的通路,被 `terrain_costmap_layer::TerrainLayer` 订阅,
局部/全局两张 costmap 各挂一份(见 `FAST_LIO/config/param_outdoor_elevation.yaml`)。
`cost` 语义:`0` = FREE(主动清除),`0<cost<100` = 渐变代价(缓坡软避让),`100` = LETHAL。

节点内部逐层加工(全部落在同一张 `grid_map` 上,随 `/terrain_grid_map` 发出可见):

```
elevation ──► ground          邻域低分位(7x7 窗取第 20 百分位), 对植被/戳出来的障碍免疫, 又跟随坡面
          ──► terrain_ok      |elevation - ground| < slope_object_height, 区分"地形"与"物体"
              ground ──► slope        中心差分梯度(slope_window=3 格)
                     ──► slope_cost   坡度 -> 0..100 分级代价
          ──► cliff           ★ 悬崖(本文档 3 节)
          ──► cliff_drop      ★ 每格的下沉深度, 给离线扫参
```

---

## 2. 为什么坡度判据抓不到悬崖

`slope` 是**差分类**判据:它算的是 `ground` 层的梯度,需要**差分窗两端都有地面高程**。

悬崖恰好破坏这个前提。激光从上沿掠过后,下一个回波落在很远的地方,中间是一片**遮挡阴影**——
既没有崖上的地面,也没有崖下的地面,`elevation` 全是 NaN,梯度算不出来。

阴影宽度:

```
W ≈ H · d / h_lidar        H=落差, d=距离, h_lidar≈0.7m
```

`H=0.8m, d=3m` → `W ≈ 3.4m`,而 `slope_window=3` 的差分跨度只有 **0.6m**。阴影比模板宽 3~10 倍,
**而且悬崖越深阴影越宽**——判据在最危险的情况下最先失效。这不是调参能救的,是判据类型不对。

10~30cm 的小路沿能抓到,正因为它的阴影窄到能落进差分窗内。

---

## 3. 悬崖检测:换一种证据——射线穿透

### 3.1 核心思路

差分判据问"两边地面差多少",要求**两边都有地面**。
悬崖场景里我们没有崖下地面,但有另一种证据:

> **有一束激光从这一格上方飞过去了,并且落在了更远的地方。**
> 那么这一格的地面**不可能高于**那束激光在此处的高度——否则激光会被挡住,飞不到更远。

这给出的是地面高度的**上界**(upper bound),而不是地面高度本身。对判"有没有掉下去"来说,上界就够了。

`elevation_mapping_cupy` 的 `visibility_cleanup` 已经在算这个了,只是默认不发布。我们只需要把它发出来用。

### 3.2 upper_bound / is_upper_bound 是怎么算出来的

**关键澄清:算法从来不检测"缺失的回波"。它只用有回波的点,也就是点云本身,没有别的输入。**

`custom_kernels.py` 的 `visibility_cleanup` 是个 `ElementwiseKernel`,对**每一个测量点**执行一次:

```cuda
// 第 i 个测量点, 变换到 map 系
U x = transform_p(rx, ry, rz, R[0], R[1], R[2], t[0]);  // t = 雷达位置(由 frame_id 的 TF 得到)
...
// 射线 = 雷达 -> 这个点。ray_vector 的定义就是 (p - t) 归一化
float16 ray_length = ray_vector(t[0], t[1], t[2], x, y, z, ray_x, ray_y, ray_z);
ray_length = min(ray_length, max_ray_length);           // 10.0 m

for (float16 s = ray_step; s < ray_length; s += ray_step) {   // ray_step = res/√2 ≈ 0.071 m
    U nx = t[0] + ray_x * s;  U ny = ...;  U nz = t[2] + ray_z * s;   // 射线上的采样点
    int nidx = get_idx(nx, ny, ...);
    if (last_nidx == nidx) continue; else last_nidx = nidx;   // 同一格只处理一次
    float16 d = (x-nx)*(x-nx) + (y-ny)*(y-ny) + (z-nz)*(z-nz);  // 到终点的距离【平方】
    if (d < 0.1 || !is_valid(...)) continue;            // < 0.32m 就跳过, 免得污染终点自己那格
    if (nmap_valid < 0.5) {                             // ★ 这一格自己没有有效高程
      if (nz < nmap_upper || nmap_is_upper < 0.5) {
        map[get_map_idx(nidx, 5)] = nz;                 // upper_bound  <- 射线在这里的高度
        map[get_map_idx(nidx, 6)] = 1.0f;               // is_upper_bound <- 置位
      }
      continue;
    }
    ...
}
```

所以两层的精确定义是:

- **`is_upper_bound = 1`** — 曾有射线**途经**此格,且此格自身无有效高程。这是**"被穿越"的记录**。
- **`upper_bound`** — 所有穿越射线中的**最低**高度(取 min,保留约束最紧的那条),即地面高度的上界。

"射线穿过某格" = **该格落在"雷达"与"某个确实有回波的点"之间的线段上**。
证据来自**更远处有回波**,不是"这里没回波"。这是标准 raycasting,和占据栅格地图的"击中/穿越"同一套路。

```
   雷达 O
    ●
    │╲╲
    │ ╲ ╲___ 射线1 -> 打到上沿 = 点 A(有回波)
    │  ╲    ╲___
────┴───┴────────╲──────┐   <- 上沿
   地面          ╲      │
                  ╲     │   <- 阴影区: 自己无回波,
                   ╲    │      但射线2 从上方穿过 -> upper_bound 有值
                    ╲   │
                     ╲  └──────● 点 B(有回波: 崖底 / 对面墙 / 远处地面)
                      ╲
            射线2 = O->B, 途经阴影区
```

`is_upper_bound = 0` 有两种**完全不同**的含义,判据全靠它区分:

| 情况 | 为什么是 0 |
|---|---|
| 有回波的正常地面 | kernel 融合该点时执行 `map[idx,6] = 0.0` 主动清零 |
| **遮挡区 / 视野外 / 从没扫到** | **没有任何射线经过它** |

第二行就是"不会堵路"的全部秘密:未观测区域不是"我们判它安全",而是**它根本进不了候选**。
flood-fill 也因此天然有界——它只能在阴影内蔓延,漏不进未知空间。

### 3.3 深度判据

```
参考地面 = z_foot - cliff_base_height        (0.46)
drop     = 参考地面 - upper_bound
```

```
odom z 轴
   ↑   ● base_link 原点          <- z_foot, 每帧从 TF 查(实测约 -0.088)
   │   │
   │   │ 0.46 m  车身离地高度(机械常数)
   │   ↓
───┼──────────────────────────   <- 参考地面 ≈ -0.548
   │            ╎
   │            ╎ drop
   │            ↓
   │   - - - - - - - - - - -     <- upper_bound(该格地面的【上界】)
   │            ┊
   │            ┊ 真实地面可能还更低, 但我们不知道
   │            ▼
```

**必须减 0.46**:`z_foot` 是 base_link 原点,直接拿它当地面会凭空多出 0.46m,平地上就误报。

**关键性质:`drop` 是真实下沉深度的下界(保守估计)。**
因为 `upper_bound` 是上界(真实地面 ≤ upper_bound):

```
真实下沉 = 参考地面 - 真实地面  ≥  参考地面 - upper_bound  =  drop
```

射线只是**恰好掠过**时,`upper_bound` 贴近地面,`drop` 很小——即使那里其实是万丈深渊也只报个小值。
**只会低估,不会高估**;低估导致漏报,高估才会误报。在"宁可漏报也不许堵路"的约束下,这个偏向正是要的。

`cliff_base_height` 的方向性也由此确定,**宁可取大**:偏大 → 参考地面更低 → drop 更小 → 更保守。
实测:`0.30` 在阈值 0.25 下产生 **7.4 格/帧**误报;`0.39` 及以上为 **0**。取 0.46(实测 `z_foot - ground中位 = 0.462`)。

> 参考高度试过 4 种,前 3 种都不行:邻域地面 min(**20.7 误报/帧**——贴墙时整个邻域都是墙根)、
> `terrain_ok` 过滤后的地面(**84.2/帧**)、车自身所在格的高程(**永远 NaN**,盲区没有高程)。
> 只有 `z_foot - 0.46` 给出 **0**。

### 3.4 为什么是双阈值(滞回)

```
drop > cliff_grow_thresh  (0.10)  -> cand = 1   可参与连通域
drop > cliff_depth_thresh (0.25)  -> seed = 1   算"深种子"
整片连通域判 lethal:  n_seed >= 6  &&  n_cells >= 10
```

**根源:`drop` 随"离上沿的距离"线性增长。**射线是斜的,擦过上沿时射线高度≈地面高度:

```
雷达 ●
      ╲
       ╲   <- 擦过上沿的射线
────────┴─────┐   <- 上沿格: drop ≈ 0.03   证据最弱
          ╲   │
           ╲  │   <- 中间格: drop ≈ 0.15
            ╲ │
             ╲│   <- 远端格: drop ≈ 0.55   证据最强
              └────●(崖底回波点)
```

矛盾就在这里:

- **证据最弱的上沿格,恰恰是最危险的位置**——车轮是从那儿掉下去的。
- **证据最强的远端格,车根本到不了**——它在崖下面。

于是两个问题对证据质量的需求**正好相反**:

| 问题 | 需要什么证据 | 阈值 |
|---|---|---|
| **这一片是不是悬崖?**(存在性) | 少量**高质量**证据即可——几格深达 0.25m 就足以排除噪声 | `depth_thresh 0.25` 严 |
| **这个悬崖延伸到哪?**(范围) | 大量**低质量**证据——每个边缘格都要覆盖,而它们 drop 本就小 | `grow_thresh 0.10` 松 |

单阈值两个方向都死:

- **都用 0.25** → 上沿格(drop 0.03)不入选,lethal 只落在阴影远端,**上沿永远标不上**,
  Nav2 可以"合法地"把路径规划到前轮悬空处;而且区域碎成孤立小块,过不了 `min_area_cells`,整个检测丢掉。
- **不设 grow 门**(改之前就是这样) → 候选只剩"射线穿过 + 无 elevation + 在距离环内",
  而**盲区格也满足这三条**:射线途经车身附近去打远处,那些格自己拿不到 elevation,
  但射线是**擦地而过**的,所以 `upper_bound ≈ 真实地面` → `drop ≈ 0`。
  它们恰恰证明"这里地面正常",却混进候选,flood-fill 从悬崖一路连到车脚下。
  实测 **159 格 lethal 压在车实际走过的路上**,这些格的 drop 中位数 **-0.042**(比参考地面还高,压根没下沉)。

双阈值给出**两道防线**,缺一不可:

1. **低阈格自己产生不了 lethal**——必须连到有 ≥6 个深种子的连通域上。盲区整体 drop≈0、零种子,单独存在永不触发。
2. **`0.10` 挡住盲区去"蹭"真悬崖的连通域**——这是第一条防线管不到的地方。0.10 正好卡在"擦地而过(≈0)"与"真下沉"之间。

结果:**159 → 0**,同时检测量保留 **99.5%**。单个阈值做不到,因为它要在同一根轴上二选一。

> 这就是 **Canny 边缘检测的滞回阈值**(hysteresis thresholding):高阈决定"边存在吗",
> 低阈决定"边延伸到哪",两者靠连通性绑定。区别只是 Canny 在梯度图上做,这里在下沉深度图上做。

`0.25` 的依据是噪声底:15° 坡道在 0.7m 窗内自然起伏 0.19m + LIO 俯仰误差 0.07m,留约 6cm 余量。

### 3.5 五道门(全部通过才判 lethal)

`addCliffLayer()`,四个 pass:

| # | 门 | 挡住什么 |
|---|---|---|
| 1 | `is_upper_bound > 0.5` | 未观测区(没有射线经过) |
| 2 | **`elevation` 必须是 NaN** | 见下方"竞态"一条,实测每帧约 **206 格**踩这个 |
| 3 | `distance ∈ [1.2, 4.5] m` | 近处盲区 + 远处稀疏噪声 |
| 4 | `drop > 0.10` / `> 0.25` + `n_seed>=6 && n_cells>=10` | 噪声、单格闪烁、盲区蔓延 |
| 5 | 时间确认(`cliff_confirm_frames = 2`) | 瞬时抖动 |

**Pass 3 的时间确认**用的是 **odom 系锚定的 0.1m 格点** hash(`(kx<<32)^ky`),不是 grid 索引——
grid 是滚动窗口,索引会随车漂移。命中则 `hits+1`(上限 8),未命中 `hits-1`(下限 0),
`hits >= 2` 才判 lethal。远离车 `cliff_max_range+1` 的 key 每帧清理,map 不会无界增长。

`min_area_cells = 10` 在 0.1m 分辨率下 = **0.1 m²**。

### 3.6 结果怎么进 costmap

**没有新增 costmap 层。**悬崖语义上就是地形的一部分,并入 `/terrain_cost_cloud` 的最高优先级:

```cpp
if (is_cliff)                 out_cost = 100.0f;   // LETHAL, 压过一切
else if (std::isfinite(c))    out_cost = c;        // slope_cost 分级代价
else /* free_on_empty */      out_cost = 0.0f;     // 半径内"连高程都没有" -> 清除
```

优先级顺序有依据:悬崖是**"射线穿过、地面确实不在那里"的正面证据**,
而 `free_on_empty` 只是**"没数据"**——两者可信度不在一个量级。

注意这里必须**主动发 100**,而不是"悬崖格跳过 free 分支":跳过只是保留旧值,
只有发 100 才能让 `TerrainLayer` 把它写成 LETHAL。

> 曾经考虑过独立的 `cliff_layer`,理由是"怕和 `footprint_clearing` / `free_on_empty` 打架"。
> 核对后该理由不成立:footprint 半长 **0.4m** < `cliff_min_range` **1.2m**,**没有重叠**;
> `free_on_empty_radius` 已收到 **2.0m**,重叠只剩 1.2~2.0m 的环带,且已被上面的优先级解决。
> 所以两张 costmap 的 `plugins` 均未改动。

### 3.7 `cliff` 与 `cliff_drop` 的区别

| 层 | 内容 | 用途 |
|---|---|---|
| `cliff` | `1.0` = 判定为悬崖,否则 NaN | **决策**。喂 cost=100 |
| `cliff_drop` | **每一格**的 `drop` 值(米) | **诊断**。RViz 看深度分布;离线重扫阈值 |

`cliff_drop` 在两个阈值**之前**就写入(`dropm(i,j) = drop`),不管有没有入选。
这样改 `depth_thresh` / `grow_thresh` **不用重跑数据**,直接在录好的 `/terrain_grid_map` 上重算即可。

---

## 4. 参数

配置在 `FAST_LIO/launch/elevation_mapping_cupy_fastlio.launch.py` 的 `terrain_scan_node` 块。

| 参数 | 默认 | 说明 |
|---|---|---|
| `cliff_enable` | `True` | 总开关。关掉退化为改动前的行为 |
| `cliff_upper_bound_layer` | `upper_bound` | 高程图层名 |
| `cliff_is_upper_bound_layer` | `is_upper_bound` | 同上 |
| `cliff_depth_thresh` | `0.25` | **高阈**:深种子。依据 = 15°坡起伏 0.19 + LIO 俯仰 0.07 |
| `cliff_grow_thresh` | `0.10` | **低阈**:连通域蔓延。挡盲区(drop≈0)蹭连通域 |
| `cliff_base_height` | `0.46` | base_link 离地高。**宁可取大**(更保守)。0.30→7.4误报/帧, ≥0.39→0 |
| `cliff_min_seed_cells` | `6` | 连通域至少几个深种子 |
| `cliff_min_area_cells` | `10` | 连通域最小面积(0.1m 分辨率 → 0.1 m²) |
| `cliff_confirm_frames` | `2` | 时间确认帧数(odom 锚定计数器) |
| `cliff_min_range` | `1.2` | 距离下限,避开近处盲区 |
| `cliff_max_range` | `4.5` | 距离上限。0.5~1.5 m/s 下给出 3~9s 前瞻 |

调参方向速查:

- **漏报**(该报没报)→ 降 `depth_thresh`,或降 `min_seed_cells` / `min_area_cells`,或调大 `max_range`
- **误报**(不该报报了)→ **先加大 `cliff_base_height`**(最灵敏、最安全的那个旋钮),再升 `depth_thresh`
- **lethal 蔓延到车走过的路上** → 升 `grow_thresh`(这是它唯一的职责)

---

## 5. 依赖的上游配置(必须)

`elevation_mapping_cupy` 侧,缺一项整个检测器**静默失效**:

```yaml
# FAST_LIO/config/elevation_mapping_cupy_fastlio.yaml
enable_visibility_cleanup: true          # 不开 -> is_upper_bound 恒 0
use_only_above_for_upper_bound: false    # 开了会砍掉大半证据
publishers:
  elevation_map_raw:
    layers: [elevation, upper_bound, is_upper_bound]   # 默认不含后两层!
    basic_layers: [elevation]
    fps: 5.0
```

`terrain_scan` 对这两种失效都有主动告警,不会闷着(见下节)。

---

## 6. 能力边界(必须知道)

**1. 依赖"更远处有回波"。** 证据链是"某点有回波 → 途经的格被穿越"。如果悬崖深到、远到
连崖底和对面都没有任何回波点,就没有射线穿过阴影区,**upper_bound 无值,检测不到**。
现实中通常有救(崖底 / 对面墙 / 远处地面),但"车停在高台边缘、前方 10m 内空无一物"时这个方法是瞎的。
`max_ray_length: 10.0` 也限制了单条射线的追踪长度。

> 这也解释了 bag 重放里前 131 秒 lethal 恒为 0——那段路的几何产生不了"跨越阴影的射线"。

**2. 只保证不高估。** `drop` 是下界,深悬崖可能只报浅值。用于**判有无**可靠,**判具体多深不可靠**。

**3. 悬垂结构不处理。** 桥下、树冠这类"上方有回波遮挡视线"的场景,阴影区会被正常观测到的顶部遮住。

**4. 设计上倾向漏报。** 硬约束是"先保证不堵路",所以每一处取舍都偏向保守。
**它是最后一道兜底,不是唯一防线**——不能因为有它就取消低速接近、不能替代 collision_monitor。

---

## 7. 运维与排障

```bash
# 看两层有没有发出来(没有 -> 查第 5 节的 yaml)
ros2 topic echo /grid_map_from_raw_pointcloud --field layers --once

# 看候选/lethal/耗时(每 50 帧一条 INFO)
ros2 topic echo /rosout | grep 悬崖层耗时
#   悬崖层耗时: 近 50 帧 均值 1.42 ms 峰值 2.71 ms | 候选 87 格, lethal 34 格

# RViz: 加 grid_map 显示 /terrain_grid_map, 层选 cliff_drop 看深度分布, cliff 看最终判定

# 一键回退(不用重新编译)
ros2 param set /terrain_scan_node cliff_enable false     # ← 注意: 此路径尚未实测, 见第 8 节
```

节点自带的三种告警:

| 日志 | 含义 | 处理 |
|---|---|---|
| `高程图缺少 'upper_bound'/'is_upper_bound' 层` | 上游没发布 | 改第 5 节的 yaml |
| `连续 50 帧 'is_upper_bound' 全为 0` | 层发了但恒 0 | 查 `enable_visibility_cleanup` |
| `取不到 <grid>←base_link 的 TF` | 连最新可用都没有 | 查 TF 树;本帧跳过,不影响其它层 |

TF 查询是**两次尝试**:先按 `stamp` 精确查,失败回退 `TimePointZero` 取最新可用。
故意不复用 `gridCallback` 里那次带时刻的查询——那次 extrapolation 失败会让整帧检测停摆(实测频繁发生),
而这里只需要"车大致在哪、脚下地面大致多高",宁可用几十毫秒前的位姿也不要整帧不检测。

**无论走哪条路径,`cliff` / `cliff_drop` 两层都会被加上(退化时填 NaN)**。
早期版本在 `have_foot == false` 时直接 return,导致下游和离线脚本拿不到层而崩。

---

## 8. 验证状态

已验证:

| 项 | 结果 |
|---|---|
| 平地误报 | **0 格/帧** |
| bag 重放 `rosbag2_2026_08_19-16_22_31` | 2644 帧中 1558 帧有 lethal;drop 中位 **0.55m** / 峰值 **1.29m** |
| 方位集中度 | **0.48**(均匀分布应为 0.125)→ 检测集中在特定方向,符合真实几何而非噪声 |
| 车走过的走廊被堵格数 | **0**(修复前 159) |
| 耗时 | **0.9~2.7 ms/帧**,约单核 1% |
| 高程图发布率 | 仍 **5.000 Hz**(无退化) |
| GPU 功耗 | 4.3 → **4.8 W** |

**尚未验证(重要):**

- `cliff_enable: False` 的回退路径**从未跑过**
- **没有真机正样本**——只有离线 bag。需要有人在场时低速接近已知落差
- **真实坡道的误报率未测**,只测了平地

---

## 9. 踩过的坑

1. **`is_upper_bound` 与 `is_valid` 会同时为 1。**
   `custom_kernels.py:245-261` 的第二个 visibility_cleanup 分支会在**本来有效**的格子上也写
   `is_upper_bound = 1`,而两个分支在同一个 elementwise kernel 里、线程无序——一帧内同一格
   可能既 `is_valid=1` 又 `is_upper_bound=1`,还带着过时的低 `upper_bound`。
   **必须加"本格 elevation 必须是 NaN"的守卫**,否则真实可行驶地面被标成悬崖。实测每帧约 **206 格**命中。

2. **参考地面不能用邻域统计。** 贴墙时整个邻域都是墙根,`ground` 的 min 会被拉低 → 20.7 误报/帧。
   车自身所在格的高程永远是 NaN(盲区无高程)。只有 `z_foot - 常数` 可靠。

3. **单阈值 flood-fill 会连进盲区。** 见 3.4。修复前 159 格 lethal 压在车实际走过的路上。

4. **`grid_map` 是滚动窗口,索引会漂移。** 时间确认必须用 odom 系锚定的格点 hash,不能用 grid 索引。

5. **退化路径也必须 `grid.add()`。** 早期 `have_foot == false` 直接 return,下游拿不到层而崩。

6. **scan 拍平必须在 `base_link` 而不是 `body`。** `body` 有 15° 俯仰,拍出来的 scan 是斜的。
   现在用 `toMatrix(t_body_to_base) * toMatrix(t_body_to_grid).inverse()` 复合,
   静态那一段用 `TimePointZero` 查——这样只有一个 TF 失败点,scan 频率才保得住
   (改错过一次:把 `t_foot` 设成必需并 return,scan 从 10Hz 掉到 4.6Hz)。

7. **绝对不能把 `elevation_layer` 换成 `inpaint`。**
   `plugin_config.yaml` 的 `inpainting.enable` 默认是 **True**(经 `core_param.yaml` 加载),
   但它写的是**独立的 `inpaint` 层**,`elevation` 本身干净,所以现状无害。
   隐患在于:inpainting 的职责就是**把 NaN 补上**,而第 1 条的守卫依赖的正是
   "本格 elevation 是 NaN"——一旦 `elevation_layer` 指向 `inpaint`,守卫全线失效,
   阴影区会被补出假高程,悬崖检测同时**丢掉证据**并**开始误报**。
