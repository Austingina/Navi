"""把 RealSense 的深度图重采样到【彩色像素网格】，产出 organized 点云。

为什么需要这个模块
------------------
semantic_segmentation_layer 完全不做投影：它按索引硬关联
(segmentation_buffer.cpp:203-206 —— pixel_idx = v*width+u 与 x/y/z 三个迭代器
同步步进)，所以「点 N 必须等于像素 N」。而 D457 的深度和彩色是两套内参、两个
光学系，同索引关联实测偏 12~65 px（横向 7~16 cm = 1.4~3.2 个 0.05 m 代价图格）。

【本模块是唯一一份投影实现】离线检查脚本 analysis/rgbd_align/check_align.py 和
运行期节点 depth_to_color_cloud 都 import 它。这样「离线验证通过」等价于「节点的
数学已验证」，不存在第二份实现漂移。

坑：Extrinsics.rotation 是列主序
--------------------------------
realsense-ros 的 tfs.cpp:104-111 按 rotation[0],[3],[6] / [1],[4],[7] / [2],[5],[8]
读这个数组，即【列主序】。用 reshape(3,3) 会拿到转置（实测代价：u 向最多 3.9 px、
v 向 2.5 px）。必须用 rotation_to_matrix() 解码。
方向已确认为 P_color = R · P_depth + t（tfs.cpp:144，base=depth）。
"""

from __future__ import annotations

import numpy as np

# 深度单位：RealSense Z16 = 毫米
DEPTH_SCALE = 0.001

# 畸变模型
DIST_NONE = "none"
DIST_PLUMB_BOB = "plumb_bob"
DIST_INVERSE_BROWN_CONRADY = "inverse_brown_conrady"
DIST_MODELS = (DIST_NONE, DIST_PLUMB_BOB, DIST_INVERSE_BROWN_CONRADY)


def rotation_to_matrix(rotation_flat) -> np.ndarray:
    """realsense2_camera_msgs/Extrinsics.rotation（9 元，列主序）-> 3x3。

    见 realsense-ros/realsense2_camera/src/tfs.cpp:104-111。
    """
    r = np.asarray(rotation_flat, dtype=np.float64).reshape(3, 3)
    return r.T          # 等价于 reshape(3, 3, order='F')


def _distort_plumb_bob(xn, yn, D):
    """OpenCV / plumb_bob 前向畸变：径向与切向都作用在原始归一化坐标上。

    D = [k1, k2, p1, p2, k3]
    """
    k1, k2, p1, p2, k3 = (list(D) + [0.0] * 5)[:5]
    r2 = xn * xn + yn * yn
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    x_t = 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn)
    y_t = p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn
    return xn * radial + x_t, yn * radial + y_t


def _distort_inverse_brown_conrady(xn, yn, D):
    """librealsense RS2_DISTORTION_INVERSE_BROWN_CONRADY 的前向投影。

    与 plumb_bob 的差别只有两点（rs2_project_point_to_pixel）：
      1. 先用径向因子把 x,y 缩放，切向项作用在【缩放后】的 x,y 上
      2. r2 不重新计算
    实测两种解释相差 中位 4.20 px / 最大 12.48 px —— 必须靠度量择一，不能假设。
    """
    k1, k2, p1, p2, k3 = (list(D) + [0.0] * 5)[:5]
    r2 = xn * xn + yn * yn
    f = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    x = xn * f
    y = yn * f
    dx = x + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    dy = y + 2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y)
    return dx, dy


def apply_distortion(xn, yn, D, model):
    if model == DIST_NONE:
        return xn, yn
    if model == DIST_PLUMB_BOB:
        return _distort_plumb_bob(xn, yn, D)
    if model == DIST_INVERSE_BROWN_CONRADY:
        return _distort_inverse_brown_conrady(xn, yn, D)
    raise ValueError(f"unknown distortion model: {model!r} (expected one of {DIST_MODELS})")


class DepthToColorWarp:
    """预计算一次，逐帧调用 warp()。

    Kd/Kc 3x3；Dc 为彩色畸变系数（深度已 rectify，实测 Dd 全 0，不需去畸变）。
    R_dc/t_dc 为 depth->color 外参，R_dc 须由 rotation_to_matrix() 得到。
    """

    def __init__(self, Kd, Kc, R_dc, t_dc, depth_size, color_size,
                 Dc=None, min_range=0.30, max_range=8.0,
                 distortion_model=DIST_PLUMB_BOB, splat="nearest"):
        self.Kd = np.asarray(Kd, dtype=np.float64).reshape(3, 3)
        self.Kc = np.asarray(Kc, dtype=np.float64).reshape(3, 3)
        self.R_dc = np.asarray(R_dc, dtype=np.float64).reshape(3, 3)
        self.t_dc = np.asarray(t_dc, dtype=np.float64).reshape(3)
        self.Dc = np.zeros(5) if Dc is None else np.asarray(Dc, dtype=np.float64)
        self.dw, self.dh = int(depth_size[0]), int(depth_size[1])
        self.cw, self.ch = int(color_size[0]), int(color_size[1])
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        if distortion_model not in DIST_MODELS:
            raise ValueError(f"distortion_model must be one of {DIST_MODELS}")
        self.distortion_model = distortion_model
        if splat not in ("nearest", "2x2"):
            raise ValueError("splat must be 'nearest' or '2x2'")
        self.splat = splat

        # 深度射线表：depth 畸变全 0，纯针孔
        v, u = np.mgrid[0:self.dh, 0:self.dw].astype(np.float64)
        self.ray_x = ((u - self.Kd[0, 2]) / self.Kd[0, 0]).astype(np.float64)
        self.ray_y = ((v - self.Kd[1, 2]) / self.Kd[1, 1]).astype(np.float64)
        self._u_d = u
        self._v_d = v

    # ---- 深度像素 -> 彩色像素（不做栅格化，供度量/诊断用）-------------------
    def project(self, depth_u16):
        """返回 (u_c, v_c, P_c, valid_mask)。P_c 形状 (3, N)，N=有效点数。"""
        Z = np.asarray(depth_u16, dtype=np.float64) * DEPTH_SCALE
        valid = (Z >= self.min_range) & (Z <= self.max_range)
        Zv = Z[valid]
        Pd = np.stack([self.ray_x[valid] * Zv, self.ray_y[valid] * Zv, Zv])
        Pc = self.R_dc @ Pd + self.t_dc[:, None]
        with np.errstate(divide="ignore", invalid="ignore"):
            xn = Pc[0] / Pc[2]
            yn = Pc[1] / Pc[2]
        xn, yn = apply_distortion(xn, yn, self.Dc, self.distortion_model)
        u_c = self.Kc[0, 0] * xn + self.Kc[0, 2]
        v_c = self.Kc[1, 1] * yn + self.Kc[1, 2]
        return u_c, v_c, Pc, valid

    # ---- 完整 warp：栅格化到彩色网格，NaN 占位 ----------------------------
    def warp(self, depth_u16):
        """返回 (xyz, stats)。

        xyz: (ch, cw, 3) float32，无效格为 NaN —— 这是 layer 要求的 organized 契约。
        stats: dict 诊断（有效率、填充率、z-buffer 碰撞率等）。
        """
        u_c, v_c, Pc, valid = self.project(depth_u16)
        n_src = int(valid.sum())

        # 三个 buffer 预填 NaN，只在投中的索引上 scatter；全流程无压缩步骤。
        npix = self.ch * self.cw
        out = np.full((3, npix), np.nan, dtype=np.float32)

        if self.splat == "nearest":
            offs = ((0, 0),)
        else:
            offs = ((0, 0), (0, 1), (1, 0), (1, 1))

        base_iu = np.floor(u_c).astype(np.int64) if self.splat == "2x2" \
            else np.rint(u_c).astype(np.int64)
        base_iv = np.floor(v_c).astype(np.int64) if self.splat == "2x2" \
            else np.rint(v_c).astype(np.int64)

        zbuf = np.full(npix, np.inf, dtype=np.float64)
        cand = []
        for du, dv in offs:
            iu = base_iu + du
            iv = base_iv + dv
            ok = (iu >= 0) & (iu < self.cw) & (iv >= 0) & (iv < self.ch) & np.isfinite(Pc[2])
            idx = iv[ok] * self.cw + iu[ok]
            z = Pc[2][ok]
            np.minimum.at(zbuf, idx, z)
            cand.append((idx, ok, z))

        n_written = 0
        n_collide = 0
        for idx, ok, z in cand:
            win = z <= zbuf[idx]                 # 只有最近的那个点写入
            sel = np.where(ok)[0][win]
            tgt = idx[win]
            n_collide += int(win.size - np.unique(tgt).size)
            out[0, tgt] = Pc[0][sel].astype(np.float32)
            out[1, tgt] = Pc[1][sel].astype(np.float32)
            out[2, tgt] = Pc[2][sel].astype(np.float32)
            n_written += int(win.sum())

        xyz = np.moveaxis(out.reshape(3, self.ch, self.cw), 0, -1)
        filled = int(np.isfinite(xyz[..., 2]).sum())
        stats = dict(
            n_src_valid=n_src,
            src_valid_frac=n_src / float(self.dh * self.dw),
            n_filled=filled,
            fill_frac=filled / float(npix),
            nan_frac=1.0 - filled / float(npix),
            n_collisions=n_collide,
            collision_frac=n_collide / max(n_src, 1),
            distortion_model=self.distortion_model,
            splat=self.splat,
        )
        return xyz, stats

    # ---- 闭式解：同索引关联的偏差（供报告与验收对照）----------------------
    def naive_offset_closed_form(self, Z):
        """du(Z), dv(Z) —— 同索引关联的理论偏差（px）。R≈I、忽略 tz 的一阶近似。"""
        Z = np.asarray(Z, dtype=np.float64)
        du = (self.Kc[0, 2] - self.Kd[0, 2]) + self.Kc[0, 0] * self.t_dc[0] / Z
        dv = (self.Kc[1, 2] - self.Kd[1, 2]) + self.Kc[1, 1] * self.t_dc[1] / Z
        return du, dv
