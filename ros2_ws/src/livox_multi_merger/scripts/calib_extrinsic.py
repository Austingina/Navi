#!/usr/bin/env python3
"""标定非 primary 雷达相对 primary 雷达的外参(点到面 GICP)。

只依赖 numpy + scipy(实测系统里是 numpy 1.26.4 / scipy 1.11.4)。刻意不引入 open3d
—— 本机 pip 装 numpy 系依赖有钉版本的坑,而点到面 ICP 用 cKDTree + 局部 PCA 就够。

## 硬性前置条件(不满足就别浪费时间,脚本会自己检查并拒绝)

  1. 车辆【静止】。全程不需要移动,满足真机安全约束。
  2. 环境【结构丰富】:墙角、走廊、有多个不同朝向的平面。
     绝不能在开阔场地标定 —— 先前在开阔地用点到点 ICP 得到 median 残差 0.51 m、
     inlier 20.9%,完全不可用。脚本会做法向可观测性检查并在欠定时拒绝。
  3. 两路雷达都在正常出数据。

## 用法

    source /opt/ros/jazzy/setup.bash
    source ~/work/robot-sensors/install/setup.bash      # 或 Navi 的 install,提供 CustomMsg

    # 标定 back 相对 front(默认)
    ros2 run livox_multi_merger calib_extrinsic.py

    # 采更多帧、指定话题
    ros2 run livox_multi_merger calib_extrinsic.py --frames 150 \
        --target-topic /livox/front_lidar --source-topic /livox/back_lidar

    # 直接把结果写回 yaml(默认只打印,不改文件)
    ros2 run livox_multi_merger calib_extrinsic.py --write path/to/lidar_extrinsics.yaml --name back

## 输出约定

打印的 xyz_rpy 是「source 系 -> target 系」的变换,旋转顺序 R = Rz(yaw)*Ry(pitch)*Rx(roll)
(scipy 的 'xyz' 外旋),与 lidar_extrinsics.yaml 和 merger 节点 C++ 端完全一致,可直接粘贴。
"""

import argparse
import sys
import time
import warnings

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

# ---------------------------------------------------------------------------
# 验收阈值。
#
# 主判据是【共视区内的点到面残差】,它才是标定精度的直接度量。曾经用"最近邻距离的
# median"当主判据,那是错的,有两处系统性偏差:
#   1. ICP 内部只能用平面点子集作匹配目标(点到面需要法向,实测仅约 50% 的点合格),
#      拿这个子集算最近邻会高估残差 —— 实测同一组外参,用子集算 median=0.084,
#      用全部点算只有 0.042。
#   2. median 会混入两台雷达【视野差异】造成的 outlier:各自独有区域的点在对方点云里
#      本就没有对应,残差必然大(实测 P90 之后从 0.06 陡增到 0.68),这与标定准不准无关。
# 所以最近邻 median 只保留为宽松的健全性检查,精度由点到面残差和系统偏置裁定。
# 反例基准:开阔场地 + 点到点 ICP 曾得到 median 0.51 m / inlier 20.9%,那是真正的失败。
# ---------------------------------------------------------------------------
ACCEPT_P2PLANE_MEDIAN_M = 0.03   # 共视区点到面残差中位数:标定精度的主判据
ACCEPT_BIAS_M = 0.01             # 有符号残差均值:非零说明还有残余平移
ACCEPT_INLIER_RATIO = 0.70       # 共视比例:太低说明压根没对齐
SANITY_MEDIAN_M = 0.15           # 最近邻 median 的宽松健全性上限
INLIER_DIST_M = 0.20

# 法向可观测性:法向散布矩阵的 λ_min/λ_max 低于此值说明场景法向过于单一
# (例如只有一堵墙、或只有地面),旋转/平移的某些自由度欠定。
MIN_NORMAL_ANISOTROPY = 0.05

# 静止性:首尾两批点云的最近邻距离中位数超过此值,说明车动了或场景有大范围变化。
MAX_STATIONARY_DRIFT_M = 0.05


def collect(target_topic, source_topic, n_frames, timeout_s):
    """从两路 CustomMsg 各采 n_frames 帧。返回 (target_batches, source_batches)。

    每个 batch 是一帧的 (N,3) float64 数组,保留分帧结构以便做静止性检查。
    """
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from livox_ros_driver2.msg import CustomMsg
    except ImportError as e:
        sys.exit(
            f"导入 ROS 消息失败: {e}\n"
            "请先 source 提供 livox_ros_driver2 的工作空间,例如:\n"
            "  source /opt/ros/jazzy/setup.bash\n"
            "  source ~/work/robot-sensors/install/setup.bash"
        )

    rclpy.init()
    node = Node("calib_collector")
    batches = {target_topic: [], source_topic: []}

    def make_cb(topic):
        def cb(msg):
            if len(batches[topic]) >= n_frames:
                return
            pts = np.array([(p.x, p.y, p.z) for p in msg.points], dtype=np.float64)
            batches[topic].append(pts)

        return cb

    for topic in (target_topic, source_topic):
        node.create_subscription(CustomMsg, topic, make_cb(topic), qos_profile_sensor_data)

    print(f"采集中: 每路 {n_frames} 帧 (约 {n_frames / 10.0:.0f} s)，请保持车辆静止 ...")
    t0 = time.time()
    last_report = 0.0
    while time.time() - t0 < timeout_s:
        if all(len(v) >= n_frames for v in batches.values()):
            break
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - last_report > 3.0:
            last_report = time.time()
            print(
                f"  target {len(batches[target_topic])}/{n_frames}  "
                f"source {len(batches[source_topic])}/{n_frames}"
            )
    node.destroy_node()
    rclpy.shutdown()

    for topic, v in batches.items():
        if len(v) < max(10, n_frames // 4):
            sys.exit(f"话题 {topic} 只收到 {len(v)} 帧，数据太少或未发布，放弃标定")
        if len(v) < n_frames:
            print(f"  警告: {topic} 只收到 {len(v)}/{n_frames} 帧，继续但精度可能下降")

    return batches[target_topic], batches[source_topic]


def _save_clouds(path, tgt_batches, src_batches):
    """把分帧点云存成 npz(拼接 + 分帧边界),供离线复算标定。"""
    d = {
        "target": np.vstack(tgt_batches),
        "target_sizes": np.array([len(b) for b in tgt_batches]),
        "source": np.vstack(src_batches),
        "source_sizes": np.array([len(b) for b in src_batches]),
    }
    np.savez_compressed(path, **d)
    print(f"\n已保存点云到 {path}(可用 --load-clouds 复算,无需再跑现场)")


def _load_clouds(path):
    d = np.load(path)

    def unpack(flat, sizes):
        out, i = [], 0
        for n in sizes:
            out.append(flat[i : i + n])
            i += n
        return out

    return (
        unpack(d["target"], d["target_sizes"]),
        unpack(d["source"], d["source_sizes"]),
    )


def preprocess(batches, voxel, r_min, r_max):
    """合并多帧、剔无效点、按距离裁剪、voxel 降采样。"""
    pts = np.vstack(batches)
    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    r = np.linalg.norm(pts, axis=1)
    # 驱动对无回波填的是精确 (0,0,0)，r_min 过滤会连带把它们剔掉
    pts = pts[(r >= r_min) & (r <= r_max)]
    if voxel > 0:
        keys = np.floor(pts / voxel).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        pts = pts[idx]
    return pts


def check_stationary(batches, voxel, r_min, r_max, label):
    """比较首尾各若干帧，确认采集期间车辆和场景基本没变。"""
    k = max(1, len(batches) // 5)
    head = preprocess(batches[:k], voxel, r_min, r_max)
    tail = preprocess(batches[-k:], voxel, r_min, r_max)
    if len(head) < 100 or len(tail) < 100:
        print(f"  [{label}] 静止性检查跳过(有效点太少)")
        return True
    d, _ = cKDTree(head).query(tail, workers=-1)
    med = float(np.median(d))
    ok = med <= MAX_STATIONARY_DRIFT_M
    print(
        f"  [{label}] 静止性: 首尾点云最近邻中位距 {med:.4f} m "
        f"(阈值 {MAX_STATIONARY_DRIFT_M}) -> {'通过' if ok else '不通过'}"
    )
    return ok


def estimate_normals(pts, k=16):
    """局部 PCA 估法向。返回 (normals, valid_mask)。

    法向取局部邻域协方差最小特征值对应的特征向量。planarity 太低(邻域不像平面)的点
    标记为无效 —— 点到面 ICP 只在真正的平面上才有意义。
    """
    tree = cKDTree(pts)
    _, idx = tree.query(pts, k=min(k, len(pts)), workers=-1)
    nbr = pts[idx]                              # (M, k, 3)
    ctr = nbr - nbr.mean(axis=1, keepdims=True)
    cov = np.einsum("mki,mkj->mij", ctr, ctr) / max(1, nbr.shape[1] - 1)
    w, v = np.linalg.eigh(cov)                  # 特征值升序
    normals = v[:, :, 0]                        # 最小特征值对应的方向
    # planarity: λ0 远小于 λ1 才算平面(避免把边缘/杂点当平面)
    with np.errstate(divide="ignore", invalid="ignore"):
        planar = w[:, 0] / np.maximum(w[:, 1], 1e-12)
    valid = np.isfinite(planar) & (planar < 0.1) & np.isfinite(normals).all(axis=1)
    return normals, valid


def check_observability(normals):
    """法向散布分析：法向方向必须足够多样，否则外参欠定。

    N = Σ n nᵀ / M 的最小/最大特征值之比，衡量"有多少个不同朝向的面"。
    只有一堵墙 -> 比值接近 0；墙角+地面 -> 比值明显大于 0。
    """
    scatter = normals.T @ normals / len(normals)
    w = np.linalg.eigvalsh(scatter)
    aniso = float(w[0] / max(w[-1], 1e-12))
    ok = aniso >= MIN_NORMAL_ANISOTROPY
    print(
        f"  法向可观测性: λmin/λmax = {aniso:.4f} "
        f"(阈值 {MIN_NORMAL_ANISOTROPY}) -> {'通过' if ok else '不通过'}"
    )
    if not ok:
        print(
            "    场景法向过于单一(例如只有一面墙或只有地面)，外参的部分自由度无法约束。\n"
            "    请换到墙角、走廊拐角等能同时看到多个不同朝向平面的位置重采。"
        )
    return ok


def evaluate(src, dst, dst_normals, dst_valid, R, t):
    """用【全部】dst 点评估外参质量。

    注意与 ICP 内部的区别:ICP 必须把匹配目标限制在平面点子集上(点到面需要法向),
    但评估不能沿用那个子集 —— 匹配目标少一半会系统性高估残差。

    返回的 p2plane_median 是共视区内的点到面残差,这是标定精度的直接度量;
    bias 是有符号残差的均值,非零意味着还有未收敛的残余平移。
    """
    P = src @ R.T + t
    d, j = cKDTree(dst).query(P, workers=-1)
    inl = d < INLIER_DIST_M
    # 点到面只在"最近邻本身是可靠平面点"时才有意义
    sel = inl & dst_valid[j]
    if sel.sum() < 100:
        return dict(median=float(np.median(d)), inlier=float(inl.mean()),
                    p2plane_median=float("nan"), bias=float("nan"), n_p2p=0)
    signed = np.einsum("ij,ij->i", dst_normals[j[sel]], P[sel] - dst[j[sel]])
    return dict(
        median=float(np.median(d)),
        inlier=float(inl.mean()),
        p2plane_median=float(np.median(np.abs(signed))),
        bias=float(signed.mean()),
        n_p2p=int(sel.sum()),
        dists=d,
    )


def fit_ground_plane(pts, nz_min=0.6, thresh=0.06, iters=1500, seed=0):
    """找"地面":法向大致朝上、且位于雷达下方的最大平面。

    返回 (n, d, inlier_count),平面为 n·p + d = 0,n 为单位向量且朝上。
    此时 d 正好是雷达相对该平面的安装高度,可作为合理性检查。

    区分地面和天花板靠 d 的符号:n 朝上时,雷达在平面上方 <=> d > 0。
    否则很容易把天花板当地面,两台雷达一个抓地面一个抓天花板会让对齐彻底跑偏。
    """
    if len(pts) < 500:
        return None
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        p = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        if n[2] < 0:
            n = -n                     # 统一朝上
        if n[2] < nz_min:
            continue                   # 不够水平,不是地面
        d = -n @ p[0]
        if d <= 0.2 or d > 4.0:
            continue                   # 只接受雷达下方 0.2~4m 的平面(排除天花板与远处斜面)
        inl = np.abs(pts @ n + d) < thresh
        c = int(inl.sum())
        if best is None or c > best[0]:
            best = (c, inl)
    if best is None or best[0] < 300:
        return None
    # 用 inlier 做一次 SVD 精修
    q = pts[best[1]]
    ctr = q.mean(0)
    _, _, vt = np.linalg.svd(q - ctr, full_matrices=False)
    n = vt[2]
    if n[2] < 0:
        n = -n
    return n, float(-n @ ctr), best[0]


def build_initial_guesses(src, dst, yaw_step_deg, extra_yaws):
    """构造 ICP 初值。

    关键思路:两台雷达都能看到同一个地面,而地面法向能钉住 roll/pitch(2 DOF)、
    地面高度差能钉住沿法向的平移(1 DOF)。于是 6 维搜索塌缩成【只需扫 yaw】,
    剩下的 xy 平移交给 ICP 收敛。这比"全部从 0 开始 + 只扫 yaw"鲁棒得多 ——
    后者在平移达 0.8m 量级时会陷在局部极小(实测 median 卡在 0.31m、inlier 33%)。
    """
    guesses = []
    g_s = fit_ground_plane(src, seed=1)
    g_d = fit_ground_plane(dst, seed=2)

    if g_s and g_d:
        n_s, d_s, c_s = g_s
        n_d, d_d, c_d = g_d
        frac_d, frac_s = c_d / len(dst), c_s / len(src)
        print(
            f"  地面平面: target inlier={c_d} ({frac_d * 100:.1f}%) 高度={d_d:.3f}m | "
            f"source inlier={c_s} ({frac_s * 100:.1f}%) 高度={d_s:.3f}m"
        )
        print(f"    -> 两台雷达高度差 {d_s - d_d:+.3f} m")
        # 可信度检查。真地面通常占相当比例的点;占比过低说明拟到的是小块水平面
        # (台阶、平台、桌面),它的"高度"没有意义。此时初值不可信,但仍留作候选之一
        # —— 反正后面会和纯 yaw 初值一起竞争,由残差裁决。
        if min(frac_d, frac_s) < 0.05:
            print(
                "    警告: 地面 inlier 占比过低,拟到的很可能不是真地面(而是小块水平面),"
                "该初值不可信"
            )
        if abs(d_s - d_d) > 0.5:
            print(
                f"    警告: 高度差 {abs(d_s - d_d):.2f}m 对同一台车上的两个雷达偏大,"
                "两者很可能拟到了不同的平面"
            )
        # 把 source 的地面法向转到 target 的地面法向上(定 roll/pitch)。
        # 单个向量对齐时绕该向量的旋转本就不唯一 —— 那正是留给 yaw 扫描的自由度,
        # 所以这里的 UserWarning 是预期的,抑制掉以免误导。
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            R_align = Rotation.align_vectors(n_d[None, :], n_s[None, :])[0].as_matrix()
        # 让两个地面重合(定沿法向的平移)。d 即安装高度,故沿 n_d 的平移为 d_s - d_d。
        t0 = (d_s - d_d) * n_d
        for yaw in np.arange(0.0, 360.0, yaw_step_deg):
            R_yaw = Rotation.from_rotvec(np.radians(yaw) * n_d).as_matrix()
            guesses.append((f"ground+yaw{yaw:.0f}", R_yaw @ R_align, t0.copy()))
    else:
        which = []
        if not g_d:
            which.append("target")
        if not g_s:
            which.append("source")
        print(
            f"  警告: {'/'.join(which)} 的地面平面拟合失败,退回纯 yaw 扫描。\n"
            "    (地面可能被遮挡、或雷达倾角过大。此时初值质量差,标定更容易失败)"
        )

    # 无论如何都补上"平移全 0 + 纯 yaw"的对照初值
    for yaw in extra_yaws:
        guesses.append(
            (f"plain_yaw{yaw:.0f}", Rotation.from_euler("z", yaw, degrees=True).as_matrix(),
             np.zeros(3))
        )
    return guesses


def icp_point_to_plane(src, dst, dst_normals, R0, t0, iters=60, max_dist0=1.5, max_dist_min=0.15):
    """点到面 ICP。src/dst 为 (N,3)，dst_normals 与 dst 一一对应。

    每轮把小角度增量线性化求解：
        r = n·(R p + t - q)
        ∂r/∂ω = (p × n)   ∂r/∂t = n
    解 6x6 正规方程得 [ω, t]，再用指数映射更新。
    """
    tree = cKDTree(dst)
    R, t = R0.copy(), t0.copy()
    max_dist = max_dist0

    for _ in range(iters):
        P = src @ R.T + t
        d, j = tree.query(P, workers=-1)
        m = d < max_dist
        if m.sum() < 100:
            break
        p = P[m]
        q = dst[j[m]]
        n = dst_normals[j[m]]

        A = np.hstack([np.cross(p, n), n])       # (M, 6)
        b = -np.einsum("ij,ij->i", n, p - q)     # (M,)

        # 轻微阻尼，避免欠定方向上解爆掉
        H = A.T @ A + 1e-9 * np.eye(6)
        g = A.T @ b
        try:
            x = np.linalg.solve(H, g)
        except np.linalg.LinAlgError:
            break

        dR = Rotation.from_rotvec(x[:3]).as_matrix()
        R = dR @ R
        t = dR @ t + x[3:]

        # 逐步收紧对应距离，从粗到精
        # 逐步收紧对应距离,从粗到精。下限不能留得太宽:对应距离比目标精度大一个量级时,
        # 错误对应会把残差顶在下限附近降不下来。
        max_dist = max(max_dist_min, max_dist * 0.9)
        if np.linalg.norm(x[:3]) < 1e-7 and np.linalg.norm(x[3:]) < 1e-7:
            break

    P = src @ R.T + t
    d, _ = tree.query(P, workers=-1)
    return R, t, d


def main():
    ap = argparse.ArgumentParser(
        description="标定 source 雷达相对 target 雷达的外参(点到面 GICP)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--target-topic", default="/livox/front_lidar", help="primary(参考)雷达话题")
    ap.add_argument("--source-topic", default="/livox/back_lidar", help="待标定雷达话题")
    ap.add_argument("--frames", type=int, default=100, help="每路采集帧数")
    ap.add_argument("--timeout", type=float, default=120.0, help="采集超时(秒)")
    ap.add_argument("--voxel", type=float, default=0.08, help="voxel 降采样边长(m)")
    ap.add_argument("--range-min", type=float, default=0.6, help="最近距离裁剪(m)")
    ap.add_argument("--range-max", type=float, default=30.0, help="最远距离裁剪(m)")
    ap.add_argument(
        "--yaw-step",
        type=float,
        default=20.0,
        help="地面预对齐后绕地面法向扫描 yaw 的步长(度)",
    )
    ap.add_argument(
        "--yaw-init",
        type=float,
        nargs="*",
        default=[0.0, 180.0],
        help="额外的『平移全 0 + 纯 yaw』对照初值(度)。"
        "注意 MID360 水平 FOV 是 360° 全向,前后雷达是安装位置不同而非朝向不同,"
        "所以 yaw 未必接近 180",
    )
    ap.add_argument(
        "--fine-max-dist",
        type=float,
        default=0.10,
        help="精修阶段对应距离的下限(m)。留得比目标精度大一个量级会让残差降不下来",
    )
    ap.add_argument("--write", metavar="YAML", help="把结果写回该 yaml(默认只打印)")
    ap.add_argument("--name", default="back", help="--write 时要更新的雷达名")
    ap.add_argument("--force", action="store_true", help="前置检查不通过也继续(不推荐)")
    ap.add_argument(
        "--save-clouds",
        metavar="NPZ",
        help="把采集到的原始点云存盘。存一次就能反复离线调标定,不必再跑现场",
    )
    ap.add_argument(
        "--load-clouds", metavar="NPZ", help="从 npz 读点云而不连 ROS(配合 --save-clouds)"
    )
    args = ap.parse_args()

    if args.load_clouds:
        tgt_batches, src_batches = _load_clouds(args.load_clouds)
        print(f"从 {args.load_clouds} 载入: target {len(tgt_batches)} 帧, source {len(src_batches)} 帧")
    else:
        tgt_batches, src_batches = collect(
            args.target_topic, args.source_topic, args.frames, args.timeout
        )
        if args.save_clouds:
            _save_clouds(args.save_clouds, tgt_batches, src_batches)

    print("\n--- 前置检查 ---")
    ok = True
    ok &= check_stationary(tgt_batches, args.voxel, args.range_min, args.range_max, "target")
    ok &= check_stationary(src_batches, args.voxel, args.range_min, args.range_max, "source")

    dst = preprocess(tgt_batches, args.voxel, args.range_min, args.range_max)
    src = preprocess(src_batches, args.voxel, args.range_min, args.range_max)
    print(f"  预处理后点数: target {len(dst)}  source {len(src)}")
    if len(dst) < 1000 or len(src) < 1000:
        print("  有效点太少，无法可靠标定")
        ok = False

    print("  估计 target 法向 ...")
    normals, valid = estimate_normals(dst)
    print(f"  平面点 {valid.sum()}/{len(dst)} ({valid.mean() * 100:.1f}%)")
    if valid.sum() < 1000:
        print("  平面点太少(场景缺少可用平面)，无法做点到面配准")
        ok = False
        dst_p, n_p = dst, normals
    else:
        dst_p, n_p = dst[valid], normals[valid]
        ok &= check_observability(n_p)

    if not ok:
        if not args.force:
            sys.exit(
                "\n前置检查未通过，已放弃标定。请按提示更换环境后重采"
                "(或用 --force 强行继续，但结果很可能不可用)。"
            )
        print("\n--force 生效：前置检查未通过仍继续。")

    print("\n--- 构造初值(地面平面预对齐)---")
    guesses = build_initial_guesses(src, dst, args.yaw_step, args.yaw_init)
    print(f"  共 {len(guesses)} 个初值")

    # 两阶段:先用降采样点云粗筛出最有希望的几个初值,再全分辨率精修。
    # 直接对所有初值做全分辨率 ICP 太慢,而粗筛的排序已足够可靠。
    print("\n--- 阶段 1/2: 粗筛 ---")
    coarse_voxel = max(args.voxel * 3, 0.2)
    src_c = preprocess([src], coarse_voxel, 0.0, 1e9)
    dst_c = preprocess([dst], coarse_voxel, 0.0, 1e9)
    n_c, v_c = estimate_normals(dst_c, k=12)
    if v_c.sum() > 300:
        dst_c, n_c = dst_c[v_c], n_c[v_c]
    ranked = []
    for label, R0, t0 in guesses:
        R, t, d = icp_point_to_plane(src_c, dst_c, n_c, R0, t0, iters=25, max_dist0=3.0)
        med = float(np.median(d))
        ranked.append((med, label, R, t))
    ranked.sort(key=lambda z: z[0])
    for med, label, _, _ in ranked[:6]:
        print(f"  {label:20s} median={med:.4f} m")
    if len(ranked) > 6:
        print(f"  ... 其余 {len(ranked) - 6} 个略")

    print("\n--- 阶段 2/2: 精修 top 3 ---")
    best = None
    finals = []
    for med_c, label, R0, t0 in ranked[:3]:
        R, t, d = icp_point_to_plane(
            src, dst_p, n_p, R0, t0, iters=150, max_dist0=1.0, max_dist_min=args.fine_max_dist
        )
        med = float(np.median(d))
        inl = float((d < INLIER_DIST_M).mean())
        rpy_i = Rotation.from_matrix(R).as_euler("xyz", degrees=True)
        print(
            f"  {label:20s} median={med:.4f} m  inlier={inl * 100:5.1f}%  "
            f"t=[{t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f}] rpy=[{rpy_i[0]:+.1f},{rpy_i[1]:+.1f},"
            f"{rpy_i[2]:+.1f}]°"
        )
        finals.append((med, inl, R, t, label))
        # 必须引用 finals 里的那个对象,后面的一致性统计用 `is not` 判断自身
        if best is None or med < best[0]:
            best = finals[-1]

    _, _, R, t, best_label = best
    rpy = Rotation.from_matrix(R).as_euler("xyz")  # 与 yaml / C++ 端的 Rz*Ry*Rx 一致

    # 用【全部】dst 点重新评估(上面 ICP 内部用的是平面点子集,会高估残差)
    ev = evaluate(src, dst, normals, valid, R, t)
    med, inl = ev["median"], ev["inlier"]

    # 多个不同初值收敛到同一解,是"这不是局部极小"的有力证据
    agree = [
        f for f in finals
        if f is not best
        and np.linalg.norm(f[3] - t) < 0.03
        and np.degrees(np.linalg.norm(
            Rotation.from_matrix(f[2] @ R.T).as_rotvec())) < 0.5
    ]
    print(f"\n  收敛一致性: {len(agree) + 1}/{len(finals)} 个初值收敛到同一解"
          f"(平移差<3cm 且角度差<0.5°)")
    if len(agree) == 0 and len(finals) > 1:
        print("    只有一个初值到达该解 —— 有可能是局部极小,建议换位置复标交叉验证")

    print("\n=== 标定结果 ===")
    print(f"  最优初值 = {best_label}")
    print(f"  translation [x y z]      = [{t[0]:+.5f}, {t[1]:+.5f}, {t[2]:+.5f}] m")
    print(
        f"  rotation    [r p y] (rad) = [{rpy[0]:+.6f}, {rpy[1]:+.6f}, {rpy[2]:+.6f}]"
        f"   (= [{np.degrees(rpy[0]):+.3f}, {np.degrees(rpy[1]):+.3f}, "
        f"{np.degrees(rpy[2]):+.3f}] °)"
    )
    print("\n  --- 质量指标(基于全部 target 点,非仅平面点子集)---")
    print(
        f"  共视区点到面残差 median = {ev['p2plane_median']:.4f} m"
        f"   (主判据, 阈值 < {ACCEPT_P2PLANE_MEDIAN_M})   [n={ev['n_p2p']}]"
    )
    print(
        f"  有符号残差均值          = {ev['bias']:+.4f} m"
        f"   (残余平移, 阈值 |·| < {ACCEPT_BIAS_M})"
    )
    print(
        f"  inlier<{INLIER_DIST_M}m              = {inl * 100:.1f}%"
        f"        (共视比例, 阈值 > {ACCEPT_INLIER_RATIO * 100:.0f}%)"
    )
    print(
        f"  最近邻 median            = {med:.4f} m"
        f"   (健全性检查, 阈值 < {SANITY_MEDIAN_M};含视野差异,不代表精度)"
    )
    if "dists" in ev:
        d_all = ev["dists"]
        qs = " ".join(f"P{p}={np.percentile(d_all, p):.3f}" for p in (50, 75, 90, 95))
        print(f"    残差分位: {qs}  <- P90 之后陡增属正常(各自独有视野)")

    checks = [
        ("共视区点到面残差", np.isfinite(ev["p2plane_median"])
         and ev["p2plane_median"] < ACCEPT_P2PLANE_MEDIAN_M),
        ("无残余平移偏置", np.isfinite(ev["bias"]) and abs(ev["bias"]) < ACCEPT_BIAS_M),
        ("共视比例", inl > ACCEPT_INLIER_RATIO),
        ("最近邻健全性", med < SANITY_MEDIAN_M),
    ]
    passed = all(c[1] for c in checks)
    print("\n  逐项:")
    for name, okk in checks:
        print(f"    {'[OK]  ' if okk else '[FAIL]'} {name}")
    print(f"\n  验收: {'*** 通过 ***' if passed else '*** 未通过 ***'}")
    if not passed:
        print(
            "  参考:开阔场地 + 点到点 ICP 的失败基准是 median 0.51 m / inlier 20.9%。\n"
            "  若当前结果接近该量级，问题几乎肯定在环境:请换到墙角/走廊，确保能同时看到\n"
            "  多个不同朝向的平面，并确认车辆完全静止后重采。"
        )

    print("\n--- 可直接粘贴进 lidar_extrinsics.yaml ---")
    print(f"    {args.name}:")
    print(f"      xyz_rpy: [{t[0]:.5f}, {t[1]:.5f}, {t[2]:.5f}, "
          f"{rpy[0]:.6f}, {rpy[1]:.6f}, {rpy[2]:.6f}]")

    print(
        "\n下一步:填入外参后启动 merger，在 RViz 里叠加 /livox/fused_lidar 检查\n"
        "墙面和地面【没有双层 ghosting】。ghosting 是比数值更灵敏的判据。\n"
        "建议换 1-2 个不同位置各标一次，平移差应 < 3 cm、角度差 < 0.5°。"
    )

    if args.write:
        if not passed and not args.force:
            sys.exit("\n验收未通过，拒绝写入 yaml(可用 --force 覆盖此保护)。")
        _write_yaml(args.write, args.name, t, rpy)


def _write_yaml(path, name, t, rpy):
    """就地替换指定雷达的 xyz_rpy 行，保留文件其余内容与注释。"""
    import re

    with open(path, "r") as f:
        lines = f.readlines()

    # 定位 "  <name>:" 段，然后替换该段内第一处 xyz_rpy
    name_re = re.compile(rf"^(\s*){re.escape(name)}:\s*$")
    start = None
    indent = ""
    for i, ln in enumerate(lines):
        m = name_re.match(ln)
        if m:
            start = i
            indent = m.group(1)
            break
    if start is None:
        sys.exit(f"在 {path} 里找不到 '{name}:' 段，未做修改")

    new_val = (
        f"[{t[0]:.5f}, {t[1]:.5f}, {t[2]:.5f}, "
        f"{rpy[0]:.6f}, {rpy[1]:.6f}, {rpy[2]:.6f}]"
    )
    for i in range(start + 1, len(lines)):
        cur = lines[i]
        # 遇到同级或更外层的新键就说明本段结束了
        if cur.strip() and not cur.startswith(indent + " ") and not cur.strip().startswith("#"):
            break
        m = re.match(r"^(\s*xyz_rpy:\s*).*$", cur)
        if m:
            lines[i] = f"{m.group(1)}{new_val}\n"
            with open(path, "w") as f:
                f.writelines(lines)
            print(f"\n已写入 {path} 的 {name}.xyz_rpy = {new_val}")
            return
    sys.exit(f"'{name}:' 段里没找到 xyz_rpy 行，未做修改")


if __name__ == "__main__":
    main()
