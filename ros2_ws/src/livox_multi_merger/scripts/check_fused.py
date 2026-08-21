#!/usr/bin/env python3
"""校验合并后的点云是否满足 FAST-LIO 的各项硬约束(阶段 1 验收用)。

merger 的几条关键不变量靠 `ros2 topic echo` 是看不出来的 —— 点是否按 offset_time 升序、
stamp 是否严格单调、无回波零点是否已剔除、line 是否越界。这个脚本一次全查。

用法:
    source /opt/ros/jazzy/setup.bash
    source ~/work/robot-sensors/install/setup.bash
    ros2 run livox_multi_merger check_fused.py

    # 同时对比合并前的单路,看点数增益
    ros2 run livox_multi_merger check_fused.py --compare /livox/front_lidar
"""

import argparse
import sys
import time

import numpy as np


def main():
    ap = argparse.ArgumentParser(
        description="校验合并点云是否满足 FAST-LIO 的硬约束",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--topic", default="/livox/fused_lidar", help="待检查的合并话题")
    ap.add_argument("--compare", default=None, help="可选:用于对比点数的单路话题")
    ap.add_argument("--frames", type=int, default=50, help="采样帧数")
    ap.add_argument("--timeout", type=float, default=60.0, help="超时(秒)")
    ap.add_argument("--scan-line", type=int, default=4, help="FAST-LIO 的 scan_line(line 上界)")
    ap.add_argument(
        "--min-range", type=float, default=0.5, help="期望的最小距离(应与 merger 的 min_range 一致)"
    )
    args = ap.parse_args()

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from livox_ros_driver2.msg import CustomMsg
    except ImportError as e:
        sys.exit(f"导入 ROS 消息失败: {e}\n请先 source 提供 livox_ros_driver2 的工作空间")

    rclpy.init()
    node = Node("fused_checker")
    topics = [args.topic] + ([args.compare] if args.compare else [])
    frames = {t: [] for t in topics}

    def make_cb(topic):
        def cb(msg):
            if len(frames[topic]) >= args.frames:
                return
            pts = np.array(
                [(p.x, p.y, p.z, p.offset_time, p.line, p.tag) for p in msg.points],
                dtype=np.float64,
            )
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frames[topic].append(
                dict(
                    stamp=stamp,
                    timebase=msg.timebase,
                    point_num=msg.point_num,
                    frame_id=msg.header.frame_id,
                    pts=pts,
                )
            )

        return cb

    # 【必须 BEST_EFFORT】。本脚本是 python,反序列化每帧约 4 万点的 CustomMsg 远跟不上
    # 10Hz;若用 RELIABLE 订阅,发布端会为这个慢订阅者保留并重传数据,从而【反压整条链路】
    # —— 实测这么干会让 merger 的凑齐率从 1.99 路/帧掉到 1.34、上游到达间隔抖到 1474ms,
    # 也就是测量工具把被测对象搞坏了。BEST_EFFORT 订阅 RELIABLE 发布端是兼容的,代价是
    # 本脚本会丢帧,但下面的频率估计对丢帧是鲁棒的(用 stamp 间隔的最小值)。
    qos = QoSProfile(depth=5)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    for t in topics:
        node.create_subscription(CustomMsg, t, make_cb(t), qos)

    print(f"采集 {args.frames} 帧 ...")
    t0 = time.time()
    while time.time() - t0 < args.timeout:
        if all(len(v) >= args.frames for v in frames.values()):
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    fs = frames[args.topic]
    if len(fs) < 5:
        sys.exit(
            f"只收到 {len(fs)} 帧 {args.topic}。\n"
            "  - merger 是否在运行? ros2 node list | grep livox_merger\n"
            "  - 若一帧都没有,注意 merger 必须以 RELIABLE 发布才能被默认 QoS 订阅到"
        )

    failures = []
    warnings = []

    print(f"\n=== {args.topic} ({len(fs)} 帧) ===")
    print(f"  frame_id: {set(f['frame_id'] for f in fs)}")

    # --- 频率 ---
    # 本脚本自身会丢帧(见上文 QoS 注释),所以不能用平均间隔算频率。
    # 丢帧只会让相邻 stamp 间隔变成真实周期的整数倍,不会让它变小 —— 因此用【最小间隔】
    # 估计 merger 的真实出帧周期,这对脚本侧丢帧是鲁棒的。
    stamps = np.array([f["stamp"] for f in fs])
    dt = np.diff(stamps)
    period_ms = dt.min() * 1e3
    true_hz = 1000.0 / period_ms if period_ms > 0 else float("nan")
    observed_hz = 1.0 / dt.mean() if len(dt) and dt.mean() > 0 else float("nan")
    # 间隔基本落在真实周期的整数倍上 => 差异纯粹来自脚本丢帧,而非 merger 出帧不稳
    ratios = dt * 1e3 / period_ms
    integer_multiples = bool(np.all(np.abs(ratios - np.round(ratios)) < 0.15))
    dropped_ratio = 1.0 - len(fs) / (1.0 + (stamps[-1] - stamps[0]) / (period_ms / 1e3))

    print(
        f"  出帧周期(按最小间隔): {period_ms:.2f} ms -> {true_hz:.2f} Hz"
        f"   [脚本实收 {observed_hz:.2f} Hz, 自身丢帧约 {dropped_ratio * 100:.0f}%]"
    )
    if not (8.0 <= true_hz <= 12.0):
        failures.append(f"真实出帧频率 {true_hz:.2f} Hz 偏离预期的 ~10 Hz")
    elif not integer_multiples:
        warnings.append(
            "帧间隔不是出帧周期的整数倍,可能 merger 出帧确实不稳(而非本脚本丢帧)。"
            "请以 merger 日志里的『近 N 帧』计数为准"
        )

    # --- stamp 严格单调(违反会让 FAST-LIO 反复清缓冲) ---
    if np.any(dt <= 0):
        n_bad = int((dt <= 0).sum())
        failures.append(f"header.stamp 非严格单调: {n_bad} 处时间未前进")
    else:
        print("  stamp 严格单调: 通过")

    # --- timebase 与 stamp 一致 ---
    tb_ok = all(abs(f["timebase"] * 1e-9 - f["stamp"]) < 1e-6 for f in fs)
    print(f"  timebase == header.stamp: {'通过' if tb_ok else '不一致'}")
    if not tb_ok:
        failures.append("timebase 与 header.stamp 不一致")

    # --- point_num 与实际点数一致(avia_handler 用它 resize) ---
    mismatch = [f for f in fs if f["point_num"] != len(f["pts"])]
    if mismatch:
        failures.append(f"point_num 与实际点数不符({len(mismatch)}/{len(fs)} 帧)")
    else:
        print("  point_num 与实际点数一致: 通过")

    # 点数用中位数看:point_num 是消息内容,不受脚本丢帧影响,但降级帧(只含 primary)
    # 会把 min 拉低,所以中位数才代表常态。
    counts = np.array([len(f["pts"]) for f in fs])
    print(
        f"  点数/帧: median={np.median(counts):.0f} min={counts.min()} max={counts.max()}"
        f" mean={counts.mean():.0f}"
    )
    if counts.min() < np.median(counts) * 0.65:
        warnings.append(
            f"有帧点数仅 {counts.min()}(中位数 {np.median(counts):.0f}),说明这些帧降级成了单路。"
            "看 merger 日志的『不完整 N 帧』确认原因"
        )

    # --- 逐帧检查 offset_time 升序 / line 范围 / 零点 / 最小距离 ---
    unsorted_frames = 0
    line_bad = 0
    zero_pts = 0
    near_pts = 0
    off_max_ms = []
    for f in fs:
        p = f["pts"]
        if len(p) == 0:
            continue
        off = p[:, 3]
        if np.any(np.diff(off) < 0):
            unsorted_frames += 1
        off_max_ms.append(off.max() / 1e6)
        line_bad += int((p[:, 4] >= args.scan_line).sum())
        r = np.linalg.norm(p[:, :3], axis=1)
        zero_pts += int((r == 0).sum())
        near_pts += int((r < args.min_range * 0.99).sum())

    print(
        f"  offset_time 升序: {'通过' if unsorted_frames == 0 else f'{unsorted_frames}/{len(fs)} 帧乱序'}"
    )
    if unsorted_frames:
        failures.append(
            f"{unsorted_frames} 帧的 offset_time 未升序 —— "
            "FAST-LIO 用 points.back() 推 lidar_end_time,乱序会导致去畸变错误"
        )

    if off_max_ms:
        om = np.array(off_max_ms)
        print(f"  offset_time 跨度: min={om.min():.2f} max={om.max():.2f} ms")
        # 合并两路后跨度应仍约一帧(100ms)+ 两路 stamp 偏差,远大于此说明基准算错了
        if om.max() > 250.0:
            failures.append(f"offset_time 跨度达 {om.max():.1f} ms,远超一帧,timebase 基准可能算错")

    print(f"  line >= {args.scan_line} 的点: {line_bad}  {'通过' if line_bad == 0 else '会被下游丢弃'}")
    if line_bad:
        failures.append(f"{line_bad} 个点的 line >= {args.scan_line},会被 avia_handler 丢弃")

    print(f"  r == 0 的无回波点: {zero_pts}  {'通过' if zero_pts == 0 else '未剔除'}")
    if zero_pts:
        failures.append(
            f"{zero_pts} 个零点残留 —— 非 primary 路的零点会平移到其安装位置形成虚假点簇"
        )

    print(f"  r < {args.min_range} m 的点: {near_pts}  {'通过' if near_pts == 0 else '超出预期'}")
    if near_pts:
        warnings.append(
            f"{near_pts} 个点近于 min_range —— 若非 primary 路的点被变换后落入该球内则属正常"
        )

    # --- 与单路对比点数增益 ---
    if args.compare and len(frames[args.compare]) >= 5:
        cs = np.array([len(f["pts"]) for f in frames[args.compare]])
        # 单路含未剔除的零点,这里按有效点(r>0)比较才公平
        cs_valid = []
        for f in frames[args.compare]:
            p = f["pts"]
            r = np.linalg.norm(p[:, :3], axis=1)
            cs_valid.append(int(((r > 0) & (r >= args.min_range)).sum()))
        cs_valid = np.array(cs_valid)
        base = float(np.median(cs_valid))
        fused = float(np.median(counts))
        print(f"\n=== 对比 {args.compare} ===")
        print(f"  原始点数/帧(中位): {np.median(cs):.0f}   其中有效(r>={args.min_range}): {base:.0f}")
        print(f"  合并后点数/帧(中位): {fused:.0f}")
        gain = fused / max(base, 1.0)
        print(f"  增益: {gain:.2f}×")
        if gain < 1.5:
            warnings.append(
                f"点数增益仅 {gain:.2f}× —— 双雷达应接近 2×。先看 merger 日志的『平均 N 路/帧』:"
                "若那里是 2.00 而这里偏低,则是本脚本同时订阅两个话题导致负载过高、"
                "采到了较多降级帧;去掉 --compare 再测一次"
            )

    # --- 汇总 ---
    print("\n" + "=" * 60)
    if failures:
        print(f"*** 未通过: {len(failures)} 项 ***")
        for x in failures:
            print(f"  [FAIL] {x}")
    else:
        print("*** 全部硬约束通过 ***")
    for x in warnings:
        print(f"  [WARN] {x}")

    if not failures:
        print(
            "\n下一步:在 RViz 里叠加该话题,确认\n"
            "  1) 墙面/地面没有双层 ghosting(标定是否准确)\n"
            "  2) 第二台雷达的安装位置没有虚假点簇(零点剔除是否生效)\n"
            "  3) 后方视野确实张开了"
        )

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
