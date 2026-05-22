"""轨道相机：绕 pivot 旋转 + 平移 + 缩放，Z-up 世界坐标。

所有矩阵均为 row-major numpy (4x4 float32)。upload 给 GL 时用 transpose=GL_TRUE。
"""

from __future__ import annotations
import math
import numpy as np


def _normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / max(n, 1e-9)


def look_at(eye: np.ndarray, target: np.ndarray, up: np.ndarray) -> np.ndarray:
    """row-major 4x4 lookAt 矩阵。"""
    eye = np.asarray(eye, dtype=np.float32)
    target = np.asarray(target, dtype=np.float32)
    up = np.asarray(up, dtype=np.float32)
    f = _normalize(target - eye)         # forward
    s = _normalize(np.cross(f, up))      # right
    u = np.cross(s, f)                   # true up
    m = np.eye(4, dtype=np.float32)
    m[0, :3] = s
    m[1, :3] = u
    m[2, :3] = -f
    m[0, 3] = -float(np.dot(s, eye))
    m[1, 3] = -float(np.dot(u, eye))
    m[2, 3] = float(np.dot(f, eye))
    return m


def perspective(fovy_deg: float, aspect: float, znear: float, zfar: float) -> np.ndarray:
    """row-major 4x4 透视投影矩阵。"""
    f = 1.0 / math.tan(math.radians(fovy_deg) * 0.5)
    m = np.zeros((4, 4), dtype=np.float32)
    m[0, 0] = f / max(aspect, 1e-6)
    m[1, 1] = f
    m[2, 2] = (zfar + znear) / (znear - zfar)
    m[2, 3] = (2.0 * zfar * znear) / (znear - zfar)
    m[3, 2] = -1.0
    return m


class OrbitCamera:
    def __init__(self, pivot=(0.0, 0.0, 0.0), distance: float = 15.0,
                 yaw_deg: float = 45.0, pitch_deg: float = 35.0,
                 fovy: float = 50.0, near: float = 0.1, far: float = 500.0,
                 aspect: float = 16.0 / 9.0):
        self.pivot = np.array(pivot, dtype=np.float32)
        self.distance = float(distance)
        self.yaw = float(yaw_deg)
        self.pitch = float(pitch_deg)
        self.fovy = float(fovy)
        self.near = float(near)
        self.far = float(far)
        self.aspect = float(aspect)

    def rotate(self, dyaw_deg: float, dpitch_deg: float) -> None:
        self.yaw += dyaw_deg
        self.pitch = float(np.clip(self.pitch + dpitch_deg, -85.0, 85.0))

    def pan(self, dx_screen: float, dy_screen: float) -> None:
        """dx/dy 是屏幕归一化坐标 (0-1)，把 pivot 沿相机右/上方向平移。"""
        right, _fwd, up = self._frame()
        scale = self.distance * 1.2
        self.pivot = self.pivot - right * (dx_screen * scale) + up * (dy_screen * scale)

    def zoom(self, factor: float) -> None:
        """factor < 1 拉近，> 1 拉远。"""
        self.distance = float(np.clip(self.distance * factor, 0.3, 500.0))

    def set_pivot(self, p) -> None:
        self.pivot = np.asarray(p, dtype=np.float32).reshape(3)

    def _frame(self):
        """返回相机坐标系下的 (right, forward, up)。forward 指向 pivot。"""
        cy, sy = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        cp, sp = math.cos(math.radians(self.pitch)), math.sin(math.radians(self.pitch))
        # eye 相对 pivot
        offset = np.array([cp * cy, cp * sy, sp], dtype=np.float32) * self.distance
        fwd = _normalize(-offset)
        # Z-up，相机的 up 用世界 Z 投影到相机平面
        cam_up = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        right = _normalize(np.cross(fwd, cam_up))
        up = np.cross(right, fwd)
        return right, fwd, up

    def eye(self) -> np.ndarray:
        cy, sy = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        cp, sp = math.cos(math.radians(self.pitch)), math.sin(math.radians(self.pitch))
        offset = np.array([cp * cy, cp * sy, sp], dtype=np.float32) * self.distance
        return self.pivot + offset

    def view(self) -> np.ndarray:
        return look_at(self.eye(), self.pivot, np.array([0.0, 0.0, 1.0], dtype=np.float32))

    def proj(self) -> np.ndarray:
        return perspective(self.fovy, self.aspect, self.near, self.far)

    def vp(self) -> np.ndarray:
        """proj * view, row-major 4x4。"""
        return self.proj() @ self.view()
