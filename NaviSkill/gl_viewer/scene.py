"""Scene：加载 submap PCD + 计算 submap→root 变换 + 路由 geom。

主要复用 NaviSkill.visualization 里写好的 _parse_pcd_xyz / 四元数等工具函数。
"""

from __future__ import annotations
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import logging
import numpy as np

try:
    from visualization import (
        _parse_pcd_xyz, _quat_xyzw, _quat_to_mat3, _quat_mul, _apply_tf,
    )
except ImportError:
    # 允许直接脚本运行
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from visualization import (
        _parse_pcd_xyz, _quat_xyzw, _quat_to_mat3, _quat_mul, _apply_tf,
    )

logger = logging.getLogger("gl_viewer.scene")


# 给不同 submap 一组温和的"灯光"颜色调，避免一片全白
_SUBMAP_PALETTE: List[Tuple[float, float, float]] = [
    (0.78, 0.85, 1.00),   # 冷蓝
    (1.00, 0.85, 0.70),   # 暖橙
    (0.80, 1.00, 0.85),   # 草绿
    (1.00, 0.78, 0.90),   # 粉紫
    (0.90, 0.95, 0.70),   # 黄绿
    (0.75, 0.90, 1.00),   # 浅天蓝
    (1.00, 0.90, 0.80),   # 米色
    (0.85, 0.80, 1.00),   # 淡紫
]


@dataclass
class SubmapGeom:
    """一个 submap 的几何数据。"""
    sm_id: str
    xyz: np.ndarray            # (N, 3) float32, submap-local
    rgb: np.ndarray            # (N, 3) float32 ∈ [0, 1]
    translation: np.ndarray    # (3,) float32, submap→root
    rotation_xyzw: np.ndarray  # (4,) float32
    palette_color: Tuple[float, float, float]
    model_mat: np.ndarray = field(default_factory=lambda: np.eye(4, dtype=np.float32))


def _color_by_z(xyz: np.ndarray, base: Tuple[float, float, float]) -> np.ndarray:
    """按高度 (z) 给一个 (N, 3) float32 颜色 ∈ [0, 1]，与 base palette 颜色调和。"""
    if len(xyz) == 0:
        return np.zeros((0, 3), dtype=np.float32)
    z = xyz[:, 2]
    z_lo, z_hi = np.percentile(z, [3.0, 97.0])
    denom = max(1e-3, float(z_hi - z_lo))
    t = np.clip((z - z_lo) / denom, 0.0, 1.0).astype(np.float32)
    # 简单 rainbow（HSV → RGB 近似）：低=冷蓝 → 高=暖橙
    r = (0.30 + 0.70 * t)
    g = (0.60 - 0.20 * np.abs(t - 0.5) * 2.0)
    b = (1.00 - 0.85 * t)
    rainbow = np.stack([r, g, b], axis=1).astype(np.float32)
    # palette 混色：保留 60% 高度色 + 40% palette
    base_arr = np.array(base, dtype=np.float32).reshape(1, 3)
    return np.clip(rainbow * 0.6 + base_arr * 0.4, 0.0, 1.0)


def _build_model_mat(t_xyz: np.ndarray, q_xyzw: np.ndarray) -> np.ndarray:
    """构造一个 4x4 model 矩阵，OpenGL/GLSL 用 row-major。"""
    m = np.eye(4, dtype=np.float32)
    m[:3, :3] = _quat_to_mat3(q_xyzw).astype(np.float32)
    m[:3, 3] = t_xyz.astype(np.float32)
    return m


class Scene:
    """从 MapManager 拿到所有 submap + transform，准备给 viewer 上传。

    lidar / 机器人状态由 ros_bridge 维护，不在这里处理；viewer 直接组合两者。
    """

    def __init__(self, map_manager, max_submap_points: int = 200_000):
        self._map_manager = map_manager
        self.max_submap_points = int(max_submap_points)
        self.submaps: Dict[str, SubmapGeom] = {}
        self.routes_cache: Dict[str, np.ndarray] = {}
        self._loaded = False

    def load_all_submaps(self) -> None:
        """一次性把所有 submap PCD 读到内存。"""
        for sm_id in self._map_manager.get_submap_list():
            try:
                self._load_one(sm_id)
            except Exception:
                logger.exception("加载 submap %s 失败", sm_id)
        self._loaded = True

    def _load_one(self, sm_id: str) -> None:
        pcd_path = self._map_manager.get_submap_full_map_path(sm_id)
        if not pcd_path or not Path(pcd_path).exists():
            logger.warning("submap %s 没找到 PCD: %s", sm_id, pcd_path)
            return
        xyz = _parse_pcd_xyz(pcd_path)
        if xyz is None or len(xyz) == 0:
            logger.warning("submap %s 解析 PCD 失败: %s", pcd_path, sm_id)
            return
        n_raw = len(xyz)
        if n_raw > self.max_submap_points:
            rng = np.random.default_rng(seed=42)
            keep = rng.choice(n_raw, self.max_submap_points, replace=False)
            xyz = xyz[keep]

        tf = self._map_manager.get_submap_transform(sm_id) or {}
        t = tf.get("translation", {"x": 0.0, "y": 0.0, "z": 0.0})
        q = tf.get("rotation", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})
        t_arr = np.array([t["x"], t["y"], t["z"]], dtype=np.float32)
        q_arr = _quat_xyzw(q).astype(np.float32)

        palette_idx = len(self.submaps) % len(_SUBMAP_PALETTE)
        color = _color_by_z(xyz, _SUBMAP_PALETTE[palette_idx])

        geom = SubmapGeom(
            sm_id=sm_id,
            xyz=xyz.astype(np.float32, copy=False),
            rgb=color,
            translation=t_arr,
            rotation_xyzw=q_arr,
            palette_color=tuple(_SUBMAP_PALETTE[palette_idx]),
            model_mat=_build_model_mat(t_arr, q_arr),
        )
        self.submaps[sm_id] = geom
        logger.info("submap %s 已加载: %d 点（原始 %d）", sm_id, len(xyz), n_raw)

    def resolve_route_world(self, route_name: str) -> Optional[np.ndarray]:
        """把 route 上所有 waypoint 变换到 root 坐标系，返回 (K, 3) float32。"""
        route = self._map_manager.get_route_by_name(route_name)
        if not route:
            return None
        wps = route.get("waypoints") or []
        if not wps:
            return None
        out = []
        for wp in wps:
            sub_id = wp.get("submap")
            name = wp.get("semantic_point") or wp.get("name")
            if not (sub_id and name):
                continue
            local = self._map_manager.get_pose_in_submap(name, sub_id)
            if not local:
                continue
            sm = self.submaps.get(sub_id)
            if sm is None:
                continue
            p = np.array([local["x"], local["y"], local.get("z", 0.0)], dtype=np.float32)
            world = sm.model_mat[:3, :3] @ p + sm.model_mat[:3, 3]
            out.append(world)
        if not out:
            return None
        return np.stack(out, axis=0).astype(np.float32)


# ── Geometry helpers ────────────────────────────────────────────────

def make_box_lines(half: float = 0.333, color=(0.4, 1.0, 0.4)
                   ) -> Tuple[np.ndarray, np.ndarray]:
    """生成一个轴对齐 box 的 wire-frame (verts, colors)。
    verts 是 GL_LINES 用的，pair 顶点构成一条线段。
    """
    h = float(half)
    corners = np.array([
        [-h, -h, -h], [+h, -h, -h], [+h, +h, -h], [-h, +h, -h],
        [-h, -h, +h], [+h, -h, +h], [+h, +h, +h], [-h, +h, +h],
    ], dtype=np.float32)
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),   # 底
        (4, 5), (5, 6), (6, 7), (7, 4),   # 顶
        (0, 4), (1, 5), (2, 6), (3, 7),   # 竖
    ]
    verts = np.stack([corners[i] for pair in edges for i in pair]).astype(np.float32)
    colors = np.tile(np.array(color, dtype=np.float32), (len(verts), 1))
    return verts, colors


def make_axes(length: float = 1.0) -> Tuple[np.ndarray, np.ndarray]:
    """GL_LINES 三色轴。"""
    L = float(length)
    verts = np.array([
        [0, 0, 0], [L, 0, 0],
        [0, 0, 0], [0, L, 0],
        [0, 0, 0], [0, 0, L],
    ], dtype=np.float32)
    colors = np.array([
        [1, 0, 0], [1, 0, 0],
        [0, 1, 0], [0, 1, 0],
        [0, 0, 1], [0, 0, 1],
    ], dtype=np.float32)
    return verts, colors
