# livox_multi_merger

把驱动分路发布的 N 个 Livox `CustomMsg` 点云流合并成一路,供只接受单个 `lid_topic` 的
FAST-LIO 消费。这样后雷达的数据才能进入 SLAM,并顺带让所有下游(elevation_mapping_cupy、
terrain_scan、Nav2 obstacle/terrain layer、collision_monitor)一次性受益 —— 它们吃的都是
FAST-LIO 的派生话题 `/cloud_registered_body`。

```
/livox/front_lidar ─┐
/livox/back_lidar  ─┼─> livox_merger ─> /livox/fused_lidar ─> FAST-LIO ─> /cloud_registered_body
(未来 left/right)  ─┘                                                       └─> 下游全部零改动
```

## 当前状态

| 项 | 状态 |
|---|---|
| 合并节点 | **已完成并在真机验证通过** |
| 硬约束自检(`check_fused.py`) | **全部通过** |
| 后雷达外参标定 | **已完成,验收通过**(共视区点到面残差 1.14cm) |
| 接入 FAST-LIO(改 `lid_topic`) | 未做 |

> 排查提示:若 merger 日志显示合并正常(2.00 路/帧、点数正确)而订阅端一帧都收不到,
> 先怀疑本机 DDS 而不是本包 —— 曾遇到过新起进程之间无法互相发现的故障,
> 此时 `ros2 topic info -v` 仍会显示两端都在、type hash 一致,极具误导性。
> 判定方法:另起两个无关的最简进程收发一次;若也不通就是环境问题,
> 重启 ROS 生态(`sudo systemctl restart robot-sensors`,会连带重启雷达)可恢复。

## 用法

```bash
# 启动合并节点(只新增话题,不影响任何现有链路)
ros2 launch livox_multi_merger merger.launch.py

# 附带发布调试用的 static TF(仅供 RViz 观察,不参与数据流)
ros2 launch livox_multi_merger merger.launch.py publish_debug_tf:=true

# 校验输出是否满足 FAST-LIO 的全部硬约束
ros2 run livox_multi_merger check_fused.py

# 标定后雷达外参(车辆静止即可,须在结构丰富的室内)
ros2 run livox_multi_merger calib_extrinsic.py
```

## 真机实测基线

前后各一台 MID360,`robot-sensors.service` 的 `livox_lidar_publisher` 以 `multi_topic=1`
分路发布。稳定运行时 `livox_merger` 的统计日志:

```
近 101 帧: 平均 2.00 路/帧, 入 39749 点 -> 出 35558 点
           (零点 4190, 距离 1, line 0, 车身 0)
         | 凑齐等待 均值 38.8ms 峰值 73.8ms | 不完整 0 帧
  [front] 到达间隔 mean=100.0 ms | stamp 间隔 mean=100.0 min=99.7 max=100.4 ms
  [back]  到达间隔 mean=99.8  ms | stamp 间隔 mean=100.0 min=99.8 max=100.4 ms
```

`check_fused.py` 同时确认:10.04 Hz、stamp 严格单调、offset_time 升序、零点已清零、
`line` 无越界、point_num 与实际点数一致。

几个读数的正常范围与解读:

- **出帧 10 Hz**,入点约 39800/帧(两路满额),出点取决于当时的无回波零点数量
- **零点数随环境变化很大**:实测过 1400/帧 也见过 4200/帧(近处物体多、朝向不同都会影响)。
  它只是被正确剔除的无效点,不是异常
- **凑齐等待随系统负载变化很大**:空闲时曾低到均值 1.2ms / 峰值 4.1ms,常态是均值
  35~40ms / 峰值 70~100ms。**看到几十毫秒不必惊慌**,`max_wait_ms=90` 是按后者定的。
  真正要看的是「不完整 N 帧」是否为 0
- **`stamp 间隔` 必须稳定在 100.0±0.5ms**。这一项失稳才说明真有问题(丢帧),
  而「到达间隔」抖动(实测可到 min 20ms / max 450ms)属于传输层常态,不影响正确性
- CPU 约 31~34% 单核(14 核机器上约 2.4%)
- 刚重启完 ROS 生态时会有一段不稳期(实测不完整帧从 63% 逐步收敛到 0,约一分钟),
  属正常,不要在这个窗口里调参

## 踩过的坑(改代码前务必读)

### 1. QoS 必须两端都 RELIABLE

`CustomMsg` 每帧约 400KB,DDS 必须分片。一开始按"传感器数据惯例"用了
`SensorDataQoS`(BEST_EFFORT, depth 5)订阅,结果**大量整帧丢失**:相邻收到帧的 stamp 间隔
从应有的 100ms 恶化到 mean 193ms / max 7500ms(平均丢一半,最长断 75 帧)。当时 CPU 仅
14.9%、UDP `rmem_max` 已是 25MiB,都不是瓶颈 —— 纯粹是 best_effort 丢分片且不重传。

驱动端是 RELIABLE 发布,FAST-LIO 用 `create_subscription(lid_topic, 20, ...)`(默认 QoS
即 RELIABLE)也工作正常,说明这条链路本就需要 reliable。现在两端都是 RELIABLE + depth 20。

发布端还有一层硬性理由:**best_effort 的 publisher 与 reliable 的 subscriber 不兼容**,
若发布端降成 best_effort,FAST-LIO 会静默收不到任何数据。

### 2. 无回波零点必须在各自坐标系里剔除,不能先变换

驱动对无回波点填的是**精确 (0,0,0)**(实测 front 占 2.82%、back 占 4.18%,约 830 点/帧)。
单雷达时它们被 FAST-LIO 的 `blind=0.5` 干净滤掉。但若先做外参变换再滤,back 的零点会整体
平移到它的安装位置(|t| 约 1.27m > blind),**全部存活**,在 costmap 里形成永久障碍物、
在 SLAM 里形成虚假特征簇。

所以处理顺序固定为:**剔零点 → per-lidar blind(自身系)→ 外参变换**,不可颠倒。

### 3. 不能在收到 primary 帧时立即合并

两路 stamp 对得极准(实测偏差 |max| 1.1ms,PTP 正常),但**到达顺序不保证** —— 实测约 65%
的情况下 back 帧比同期 front 帧后到。早期版本在 primary 回调里直接取 buffer 里"最近"的
back 帧,拿到的是上一帧(差约 100ms),超容差后被判失步,于是在 100ms 内反复
降级/恢复刷屏。

现在改为「等待凑齐、超时降级」:primary 帧先入 pending 队列,任一路到达时尝试 flush,
凑齐即发,等满 `max_wait_ms` 才降级。定时器兜底防止某路停流时卡死。输出严格按 primary
帧顺序(只处理队首、不跳帧),以保证 stamp 单调。

### 4. 输出必须按 offset_time 排序、stamp 必须严格单调

- `laserMapping.cpp:412` 用 `points.back().curvature` 推 `lidar_end_time` —— 取的是数组
  **最后一个**元素而非最大值。不排序则 IMU 积分区间截错,去畸变失效。
- `laserMapping.cpp:324` 一旦发现时间回退就 `"lidar loop back, clear buffer"`,缓冲区被
  反复清空。

### 5. `line` 不能加偏移,`lidar_id` 不能用来区分来源

- `preprocess.cpp:169` 的 `avia_handler` 丢弃 `line >= scan_line`(=4)的点,所以不能给
  第二路的 `line` 加偏移来标记来源。
- `CustomMsg.lidar_id` 也指望不上:它是 `uint8`,而 handle 是 IP 的数值形式,
  `192.168.1.101/102` 截断后都只剩首字节 **192**(实测两路都是 192)。

**来源只能靠话题名区分。**

### 5b. 外参标定:yaw 不是 180°,不要被驱动配置误导

驱动配置 `MID360s_config.json` 给后雷达写的是 `yaw: 180.0`,很容易据此认为后雷达朝后装、
外参应该绕 Z 转 180°。**实测标定出来 yaw ≈ -1°**,即两台雷达朝向基本一致。

原因:**MID360 的水平 FOV 本身就是 360° 全向**,"前/后"指的是安装位置,不是朝向。那个
`yaw: 180.0` 既是死代码(见坑 7),也只是"装在后面"的语义标记。

标定结果(点到面 GICP,车辆静止,结构丰富室内):

```
xyz_rpy = [-0.28563, 0.44809, -0.10994,  -0.524487, 0.019861, -0.017750]
          即 t = [-0.286, +0.448, -0.110] m,rpy = [-30.05, +1.14, -1.02] 度

共视区点到面残差 median = 0.0114 m   (训练)  /  0.0176 m (换一批数据交叉验证)
有符号残差均值           = -0.0009 m  -> 无残余平移
共视比例(<0.2m)          = 90.0%
3/3 个不同初值收敛到同一解
```

`roll = -30.05°` 几乎正好是整数,与"前雷达 +15°、后雷达 -15°"的机械设计吻合
(前雷达的 15° 倾角在 `outdoor_localization.launch.py` 里也有体现)。

两个关于**评估指标**的坑,值得单独记:

1. **别用"最近邻距离 median"当验收主判据。** ICP 内部只能拿平面点子集做匹配目标
   (点到面需要法向,实测仅约 50% 的点合格),用这个子集算最近邻会系统性高估残差 ——
   同一组外参,子集算出 0.084 m,用全部点算只有 0.042 m。
2. **median 会混入视野差异。** 两台雷达各自独有区域的点在对方点云里本就没有对应,残差
   必然大(实测 P90 之后从 0.06 陡增到 0.68)。这与标定准不准无关。

所以主判据用**共视区内的点到面残差**加**有符号残差均值**(检查残余平移),最近邻 median
只留作宽松的健全性检查。

标定的硬性前置条件:**车辆静止**(不需要移动)+ **结构丰富的室内**(墙角/走廊,能同时
看到多个不同朝向的平面)。在开阔场地标定必然失败 —— 那里点到点 ICP 只能得到
median 0.51 m / inlier 20.9%。脚本会做静止性和法向可观测性检查并主动拒绝。

采集一次可以用 `--save-clouds` 存盘,之后 `--load-clouds` 反复离线复算,不必重跑现场。

### 6. 用 python 工具订阅会反压整条链路

`check_fused.py` 最初用 RELIABLE 订阅,而 python 反序列化 4 万点跟不上 10Hz,发布端为这个
慢订阅者保留并重传数据,**反压了上游** —— merger 的凑齐率从 2.00 掉到 1.34、上游到达间隔
抖到 1474ms。测量工具把被测对象搞坏了。

所以诊断脚本一律用 BEST_EFFORT 订阅(与 RELIABLE 发布端兼容),并用 **stamp 间隔的最小值**
估计真实帧率 —— 丢帧只会让间隔变成真实周期的整数倍,不会让它变小。

### 7. 驱动配置里的 extrinsic 是死代码

`robot-sensors` 的 `MID360s_config.json` 给后雷达配了 `yaw: 180.0`,但
`pub_handler.cpp:132` 硬编码 `packet.extrinsic_enable = false`,而坐标变换只在
`if (pkt.extrinsic_enable)` 分支内执行。**那份配置完全没有生效**,`/livox/back_lidar` 是后
雷达自身原始坐标系的点云。外参变换必须由本节点自己做。

## 参数

见 `config/lidar_extrinsics.yaml`,每一项都有注释说明取值依据。几个容易调错的:

| 参数 | 默认 | 说明 |
|---|---|---|
| `sync_tolerance_ms` | 20.0 | 判定两帧"同期"的 stamp 容差。**不是**等待时长 |
| `max_wait_ms` | 90.0 | 等第二路到达的上限。实测等待仅 1ms 级,这是宽松保险 |
| `degrade_timeout_ms` | 800.0 | **仅影响日志措辞**。设小了会把 DDS 成簇送达的正常抖动误报成"掉线" |
| `qos_depth` | 20 | 与 FAST-LIO 对齐,别改小 |

## 扩展到 4 路

驱动已支持 front/back/left/right。加一路只需在 yaml 的 `lidars` 列表里加名字,再补一段
同名子配置(`topic` / `xyz_rpy` / `min_range` / `max_range`),节点代码无需改动。

届时需留意:点数会翻倍到约 8 万/帧,`std::sort` 的开销随之上升(当前 34% 单核)。若成为
瓶颈,可利用"每路内部已按 offset_time 有序"这一性质,改用 `std::merge` 把 O(n log n)
降到 O(n)。
