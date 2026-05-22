"""GLViewer 主窗口（离屏 NVIDIA EGL + QImage 显示）。

架构：
  - QtWidgets.QWidget （**普通 widget**，不走 Qt 自己的 GL 上下文）
  - 自己持有一个 OffscreenEGLContext（NVIDIA EGL device 创建，硬件加速）
  - paintEvent:
      egl.make_current()
        → 跑全部渲染 pass（点云 / lidar / bloom / composite）
        → glReadPixels(BGRA) → numpy → QImage
      egl.release()
      QPainter.drawImage(...)

  ⚠ 这绕开了 "Jetson X11+Mesa = 软渲染" 的问题。GPU 端 NVIDIA 硬件加速，
  CPU 端只多一次 ~5MB GPU→CPU 拷贝 + QPainter blit，远比软渲染快。

控制：
- 左键拖动        : 旋转
- 中键 / Shift+左键 : 平移
- 滚轮            : 缩放
- F               : 锁定/解锁视角跟随机器人
- R               : 复位相机
- B               : 切换 bloom 开关
- C               : 切换圆形 splat
"""

from __future__ import annotations
import logging
import time
from typing import Dict
import numpy as np

from PySide2 import QtCore, QtGui, QtWidgets
from OpenGL import GL

from .camera import OrbitCamera
from .scene import Scene, SubmapGeom, make_box_lines
from .egl_context import OffscreenEGLContext
from .shaders import (
    CLOUD_VS, CLOUD_FS,
    FULLSCREEN_VS,
    BRIGHT_FS, BLUR_FS, COMPOSITE_FS,
    SIMPLE_VS, SIMPLE_FS,
)

logger = logging.getLogger("gl_viewer.viewer")


# ── GL 工具：编译 + 链接 ─────────────────────────────────────────────

def _compile_shader(src: str, kind) -> int:
    shader = GL.glCreateShader(kind)
    GL.glShaderSource(shader, src)
    GL.glCompileShader(shader)
    if not GL.glGetShaderiv(shader, GL.GL_COMPILE_STATUS):
        log = GL.glGetShaderInfoLog(shader).decode()
        GL.glDeleteShader(shader)
        raise RuntimeError(f"Shader 编译失败 ({kind}): {log}")
    return shader


def _link_program(vs_src: str, fs_src: str) -> int:
    vs = _compile_shader(vs_src, GL.GL_VERTEX_SHADER)
    fs = _compile_shader(fs_src, GL.GL_FRAGMENT_SHADER)
    prog = GL.glCreateProgram()
    GL.glAttachShader(prog, vs)
    GL.glAttachShader(prog, fs)
    GL.glLinkProgram(prog)
    if not GL.glGetProgramiv(prog, GL.GL_LINK_STATUS):
        log = GL.glGetProgramInfoLog(prog).decode()
        raise RuntimeError(f"Shader 链接失败: {log}")
    GL.glDeleteShader(vs)
    GL.glDeleteShader(fs)
    return prog


# ── Uniform location 缓存 ─────────────────────────────────────────────
_ULOC_CACHE: Dict[int, int] = {}


def _uloc(prog: int, name: str) -> int:
    key = (prog << 32) ^ hash(name)
    loc = _ULOC_CACHE.get(key)
    if loc is None:
        loc = GL.glGetUniformLocation(prog, name)
        _ULOC_CACHE[key] = loc
    return loc


def _set_mat4(prog: int, name: str, mat: np.ndarray) -> None:
    loc = _uloc(prog, name)
    if loc >= 0:
        GL.glUniformMatrix4fv(loc, 1, GL.GL_TRUE, mat.astype(np.float32, copy=False).flatten())


def _set_vec3(prog: int, name: str, v) -> None:
    loc = _uloc(prog, name)
    if loc >= 0:
        v = np.asarray(v, dtype=np.float32).flatten()
        GL.glUniform3f(loc, float(v[0]), float(v[1]), float(v[2]))


def _set_vec2(prog: int, name: str, v) -> None:
    loc = _uloc(prog, name)
    if loc >= 0:
        v = np.asarray(v, dtype=np.float32).flatten()
        GL.glUniform2f(loc, float(v[0]), float(v[1]))


def _set_float(prog: int, name: str, v: float) -> None:
    loc = _uloc(prog, name)
    if loc >= 0:
        GL.glUniform1f(loc, float(v))


def _set_int(prog: int, name: str, v: int) -> None:
    loc = _uloc(prog, name)
    if loc >= 0:
        GL.glUniform1i(loc, int(v))


def _set_bool(prog: int, name: str, v: bool) -> None:
    _set_int(prog, name, 1 if v else 0)


# ─────────────────────────────────────────────────────────────────────


class GLPointCloudWidget(QtWidgets.QWidget):
    """点云 viewer（普通 QWidget，靠 EGL 离屏 + QImage 显示）。"""

    def __init__(self, scene: Scene, bridge, parent=None,
                 target_fps: float = 15.0, enable_bloom: bool = True):
        super().__init__(parent)
        self._scene = scene
        self._bridge = bridge

        self.camera = OrbitCamera(pivot=(0.0, 0.0, 0.0), distance=15.0,
                                  yaw_deg=45.0, pitch_deg=35.0)
        self.follow_robot = True
        self.enable_bloom = bool(enable_bloom)
        self.circular_points = True
        self.point_size = 4.0
        self.bg_color = (0.04, 0.05, 0.08)

        self._last_mouse = None
        self._last_button = None
        self._rng = np.random.default_rng(seed=2024)

        # 重绘节流
        interval_ms = max(20, int(1000.0 / max(1.0, target_fps)))
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(interval_ms)
        self._timer.setTimerType(QtCore.Qt.CoarseTimer)
        self._timer.timeout.connect(self._tick)
        self._timer.start()

        # 性能计数
        self._cnt_t0 = time.monotonic()
        self._cnt_paint = 0
        self._cnt_tick = 0
        self._cnt_skip_tick = 0
        self._cnt_gpu_ms_sum = 0.0
        self._cnt_blit_ms_sum = 0.0

        # 重绘必要性追踪
        self._last_drawn_lidar_seq = -1
        self._last_drawn_robot_stamp = -1
        self._dirty = True

        # GL 资源
        self._gl_inited = False
        self._prog_cloud = 0
        self._prog_bright = 0
        self._prog_blur = 0
        self._prog_composite = 0
        self._prog_simple = 0
        self._fsq_vao = 0
        self._submap_vaos: Dict[str, int] = {}
        self._lidar_vao = 0
        self._lidar_vbo = 0
        self._lidar_capacity = 0
        self._lidar_count = 0
        self._gpu_lidar_seq = -1
        self._robot_vao = 0
        self._robot_vbo = 0
        self._robot_count = 0
        self._robot_glow_vao = 0
        self._robot_glow_vbo = 0
        self._scene_fbo = 0
        self._scene_color = 0
        self._scene_depth = 0
        self._bright_fbo = 0
        self._bright_color = 0
        self._blur_fbo = [0, 0]
        self._blur_color = [0, 0]
        self._fbo_w = 0
        self._fbo_h = 0

        # ── 关键：自己创建 NVIDIA EGL 离屏上下文 ──
        init_w = max(64, self.width() or 1280)
        init_h = max(64, self.height() or 800)
        self._egl = OffscreenEGLContext(init_w, init_h)

        # WA_OpaquePaintEvent：告诉 Qt 我们 paint 时会覆盖整个 widget，
        # Qt 不需要先擦背景。略省 CPU；恢复显示后仍由 paintEvent 重画。
        self.setAttribute(QtCore.Qt.WA_OpaquePaintEvent, True)
        self.setAttribute(QtCore.Qt.WA_NoSystemBackground, True)
        # 注意：**不要** 开 WA_PaintOnScreen——它会绕过 Qt 双缓冲，
        # 在最小化-恢复后空白；我们要的是普通 widget paint 流程。
        self.setAutoFillBackground(False)
        self.setMouseTracking(True)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)

    # ── 公共动作 ────────────────────────────────────────────────────

    def toggle_follow(self) -> None:
        self.follow_robot = not self.follow_robot

    def toggle_bloom(self) -> None:
        self.enable_bloom = not self.enable_bloom

    def reset_camera(self) -> None:
        self.camera = OrbitCamera(pivot=tuple(self.camera.pivot.tolist()),
                                  distance=15.0, yaw_deg=45.0, pitch_deg=35.0)
        self.camera.aspect = max(1.0, self.width() / max(1, self.height()))

    # ── GL 初始化（懒加载） ─────────────────────────────────────────

    def _ensure_gl_inited(self) -> None:
        if self._gl_inited:
            return
        self._egl.make_current()
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_PROGRAM_POINT_SIZE)
        GL.glEnable(GL.GL_BLEND)
        GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)

        self._prog_cloud = _link_program(CLOUD_VS, CLOUD_FS)
        self._prog_bright = _link_program(FULLSCREEN_VS, BRIGHT_FS)
        self._prog_blur = _link_program(FULLSCREEN_VS, BLUR_FS)
        self._prog_composite = _link_program(FULLSCREEN_VS, COMPOSITE_FS)
        self._prog_simple = _link_program(SIMPLE_VS, SIMPLE_FS)
        logger.info("GL 程序编译完成 (4 + 1 个)")

        self._fsq_vao = GL.glGenVertexArrays(1)

        for sm_id, geom in self._scene.submaps.items():
            self._upload_submap(sm_id, geom)
        box_v, box_c = make_box_lines(half=0.30, color=(0.4, 1.0, 0.4))
        self._robot_vao, self._robot_vbo, self._robot_count = \
            self._make_lines_vao(box_v, box_c)
        self._robot_glow_vbo, self._robot_glow_vao = self._make_dynamic_vao(1)

        self._lidar_capacity = 100_000
        self._lidar_vbo, self._lidar_vao = self._make_dynamic_vao(self._lidar_capacity)

        self._gl_inited = True

    # ── Qt 事件 ────────────────────────────────────────────────────

    def resizeEvent(self, ev: QtGui.QResizeEvent) -> None:
        w, h = max(64, ev.size().width()), max(64, ev.size().height())
        self.camera.aspect = w / h
        if self._gl_inited:
            self._egl.resize(w, h)
            self._recreate_fbos(w, h)
        else:
            self._egl.resize(w, h)
        self._mark_dirty()
        super().resizeEvent(ev)

    def paintEvent(self, ev: QtGui.QPaintEvent) -> None:
        _t_render0 = time.perf_counter()

        # ── 1. 离屏 GL 渲染 ───────────────────────────────────────
        self._ensure_gl_inited()
        if (self.width() != self._egl.width() or self.height() != self._egl.height()):
            self._egl.resize(self.width(), self.height())
        self._egl.make_current()
        if self._fbo_w == 0 or self._fbo_w != self._egl.width() or self._fbo_h != self._egl.height():
            self._recreate_fbos(self._egl.width(), self._egl.height())
        self._render_scene()
        GL.glFinish()
        gpu_ms = (time.perf_counter() - _t_render0) * 1000.0

        # ── 2. 把 GL framebuffer 读到 CPU ────────────────────────
        _t_blit0 = time.perf_counter()
        pixels = self._egl.read_pixels_bgra()
        self._egl.release()
        h, w = pixels.shape[:2]
        # QImage 直接包装 numpy 内存，保留引用避免 GC
        self._last_pixels = pixels
        qimg = QtGui.QImage(pixels.data, w, h, w * 4, QtGui.QImage.Format_ARGB32)

        # ── 3. QPainter 显示 ─────────────────────────────────────
        p = QtGui.QPainter(self)
        p.drawImage(0, 0, qimg)
        p.end()
        blit_ms = (time.perf_counter() - _t_blit0) * 1000.0

        # ── 4. 数据版本 + 性能统计 ───────────────────────────────
        self._cnt_paint += 1
        self._cnt_gpu_ms_sum += gpu_ms
        self._cnt_blit_ms_sum += blit_ms

        now = time.monotonic()
        if now - self._cnt_t0 > 2.0:
            dt = now - self._cnt_t0
            paint_hz = self._cnt_paint / dt
            tick_hz = self._cnt_tick / dt
            skip_hz = self._cnt_skip_tick / dt
            avg_gpu_ms = self._cnt_gpu_ms_sum / max(1, self._cnt_paint)
            avg_blit_ms = self._cnt_blit_ms_sum / max(1, self._cnt_paint)
            lidar_hz, tf_hz = 0.0, 0.0
            if hasattr(self._bridge, "get_counters"):
                ld, tf = self._bridge.get_counters(reset=True)
                lidar_hz, tf_hz = ld / dt, tf / dt
            self._cnt_t0 = now
            self._cnt_paint = self._cnt_tick = self._cnt_skip_tick = 0
            self._cnt_gpu_ms_sum = self._cnt_blit_ms_sum = 0.0
            wnd = self.window()
            if wnd:
                wnd.setWindowTitle(
                    f"GL  |  paint {paint_hz:4.1f}Hz (gpu {avg_gpu_ms:4.1f} + blit {avg_blit_ms:4.1f} ms)  |  "
                    f"tick {tick_hz:4.1f}(skip {skip_hz:4.1f})  |  "
                    f"lidar {lidar_hz:4.1f}Hz tf {tf_hz:4.1f}Hz  |  "
                    f"follow={'ON' if self.follow_robot else 'off'} "
                    f"bloom={'ON' if self.enable_bloom else 'off'}")

    # ── 核心渲染 ────────────────────────────────────────────────────

    def _render_scene(self) -> None:
        light_pos = np.zeros(3, dtype=np.float32)
        robot = self._bridge.get_robot_state() if self._bridge else None
        if robot is not None:
            sm = self._scene.submaps.get(robot.submap_id)
            if sm is not None:
                world = sm.model_mat[:3, :3] @ robot.local_pos + sm.model_mat[:3, 3]
                light_pos = world.astype(np.float32)
                if self.follow_robot:
                    self.camera.set_pivot(world)

        view = self.camera.view()
        proj = self.camera.proj()
        cam_eye = self.camera.eye()

        # ──────── PASS 1 — 点云到 HDR FBO ────────
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self._scene_fbo)
        GL.glViewport(0, 0, self._fbo_w, self._fbo_h)
        GL.glClearColor(*self.bg_color, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glEnable(GL.GL_DEPTH_TEST)

        GL.glUseProgram(self._prog_cloud)
        _set_vec3(self._prog_cloud, "uLightPos", light_pos)
        _set_vec3(self._prog_cloud, "uLightColor", (1.30, 1.10, 0.85))
        _set_float(self._prog_cloud, "uLightRadius", 6.0)
        _set_vec3(self._prog_cloud, "uAmbient", (0.55, 0.65, 0.85))
        _set_float(self._prog_cloud, "uAmbientStrength", 0.25)
        _set_vec3(self._prog_cloud, "uFogColor", self.bg_color)
        _set_float(self._prog_cloud, "uFogStart", 12.0)
        _set_float(self._prog_cloud, "uFogEnd", 50.0)
        _set_float(self._prog_cloud, "uExposure", 1.0)
        _set_float(self._prog_cloud, "uEmissive", 0.18)
        _set_vec3(self._prog_cloud, "uCameraPos", cam_eye)

        _set_bool(self._prog_cloud, "uCircular", self.circular_points)
        _set_float(self._prog_cloud, "uPointSize", self.point_size)
        _set_float(self._prog_cloud, "uPointSizeFalloff", 0.0)
        for sm_id, vao in self._submap_vaos.items():
            geom = self._scene.submaps[sm_id]
            mvp = proj @ view @ geom.model_mat
            _set_mat4(self._prog_cloud, "uMVP", mvp)
            _set_mat4(self._prog_cloud, "uModel", geom.model_mat)
            GL.glBindVertexArray(vao)
            GL.glDrawArrays(GL.GL_POINTS, 0, len(geom.xyz))

        # lidar
        lidar = self._bridge.get_latest_lidar() if self._bridge else None
        if lidar is not None and lidar.interleaved is not None and len(lidar.interleaved) > 0:
            sm = self._scene.submaps.get(lidar.submap_id)
            if sm is not None:
                self._upload_lidar(lidar.interleaved, lidar.seq)
                mvp = proj @ view @ sm.model_mat
                _set_mat4(self._prog_cloud, "uMVP", mvp)
                _set_mat4(self._prog_cloud, "uModel", sm.model_mat)
                _set_bool(self._prog_cloud, "uCircular", self.circular_points)
                _set_float(self._prog_cloud, "uPointSize", 6.0)
                _set_float(self._prog_cloud, "uPointSizeFalloff", 0.0)
                GL.glBindVertexArray(self._lidar_vao)
                GL.glDrawArrays(GL.GL_POINTS, 0, self._lidar_count)

        # 机器人外框 + 发光核心
        if robot is not None:
            sm = self._scene.submaps.get(robot.submap_id)
            if sm is not None:
                world = sm.model_mat[:3, :3] @ robot.local_pos + sm.model_mat[:3, 3]
                model = np.eye(4, dtype=np.float32)
                model[:3, 3] = world
                GL.glUseProgram(self._prog_simple)
                _set_mat4(self._prog_simple, "uMVP", proj @ view @ model)
                _set_float(self._prog_simple, "uAlpha", 1.0)
                GL.glBindVertexArray(self._robot_vao)
                GL.glDrawArrays(GL.GL_LINES, 0, self._robot_count)
                # 发光核心
                self._upload_robot_glow(world)
                GL.glUseProgram(self._prog_cloud)
                _set_vec3(self._prog_cloud, "uLightPos", world)
                _set_float(self._prog_cloud, "uLightRadius", 6.0)
                _set_float(self._prog_cloud, "uEmissive", 6.0)
                _set_float(self._prog_cloud, "uAmbientStrength", 0.0)
                _set_bool(self._prog_cloud, "uCircular", True)
                identity4 = np.eye(4, dtype=np.float32)
                _set_mat4(self._prog_cloud, "uMVP", proj @ view)
                _set_mat4(self._prog_cloud, "uModel", identity4)
                _set_float(self._prog_cloud, "uPointSize", 1600.0)
                _set_float(self._prog_cloud, "uPointSizeFalloff", 0.05)
                GL.glBindVertexArray(self._robot_glow_vao)
                GL.glDrawArrays(GL.GL_POINTS, 0, 1)

        # ──────── PASS 2 — 亮度阈值 ────────
        if self.enable_bloom:
            GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self._bright_fbo)
            GL.glViewport(0, 0, self._fbo_w // 2, self._fbo_h // 2)
            GL.glDisable(GL.GL_DEPTH_TEST)
            GL.glClear(GL.GL_COLOR_BUFFER_BIT)
            GL.glUseProgram(self._prog_bright)
            GL.glActiveTexture(GL.GL_TEXTURE0)
            GL.glBindTexture(GL.GL_TEXTURE_2D, self._scene_color)
            _set_int(self._prog_bright, "uScene", 0)
            _set_float(self._prog_bright, "uThreshold", 0.85)
            _set_float(self._prog_bright, "uKnee", 0.25)
            GL.glBindVertexArray(self._fsq_vao)
            GL.glDrawArrays(GL.GL_TRIANGLES, 0, 3)

            # ──────── PASS 3 — 高斯模糊 ping-pong ────────
            GL.glUseProgram(self._prog_blur)
            src = self._bright_color
            for i in range(2):
                dst_fbo = self._blur_fbo[i % 2]
                dst_color = self._blur_color[i % 2]
                GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, dst_fbo)
                GL.glViewport(0, 0, self._fbo_w // 2, self._fbo_h // 2)
                GL.glClear(GL.GL_COLOR_BUFFER_BIT)
                GL.glActiveTexture(GL.GL_TEXTURE0)
                GL.glBindTexture(GL.GL_TEXTURE_2D, src)
                _set_int(self._prog_blur, "uTex", 0)
                if i % 2 == 0:
                    _set_vec2(self._prog_blur, "uDir", (1.0 / (self._fbo_w // 2), 0.0))
                else:
                    _set_vec2(self._prog_blur, "uDir", (0.0, 1.0 / (self._fbo_h // 2)))
                GL.glBindVertexArray(self._fsq_vao)
                GL.glDrawArrays(GL.GL_TRIANGLES, 0, 3)
                src = dst_color
            bloom_tex = src
        else:
            bloom_tex = 0

        # ──────── PASS 4 — 合成到 EGL pbuffer surface (FBO 0) ────────
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)
        GL.glViewport(0, 0, self._egl.width(), self._egl.height())
        GL.glDisable(GL.GL_DEPTH_TEST)
        GL.glClearColor(0.0, 0.0, 0.0, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT)
        GL.glUseProgram(self._prog_composite)
        GL.glActiveTexture(GL.GL_TEXTURE0)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self._scene_color)
        _set_int(self._prog_composite, "uScene", 0)
        if bloom_tex:
            GL.glActiveTexture(GL.GL_TEXTURE1)
            GL.glBindTexture(GL.GL_TEXTURE_2D, bloom_tex)
            _set_int(self._prog_composite, "uBloom", 1)
            _set_float(self._prog_composite, "uBloomStrength", 0.65)
        else:
            GL.glActiveTexture(GL.GL_TEXTURE1)
            GL.glBindTexture(GL.GL_TEXTURE_2D, self._scene_color)
            _set_int(self._prog_composite, "uBloom", 1)
            _set_float(self._prog_composite, "uBloomStrength", 0.0)
        _set_float(self._prog_composite, "uGamma", 2.2)
        GL.glBindVertexArray(self._fsq_vao)
        GL.glDrawArrays(GL.GL_TRIANGLES, 0, 3)

        if lidar is not None:
            self._last_drawn_lidar_seq = lidar.seq
        if robot is not None:
            self._last_drawn_robot_stamp = robot.stamp_ns

    # ── 数据上传辅助 ────────────────────────────────────────────────

    def _make_dynamic_vao(self, capacity: int):
        vbo = GL.glGenBuffers(1)
        vao = GL.glGenVertexArrays(1)
        GL.glBindVertexArray(vao)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, capacity * 6 * 4, None, GL.GL_DYNAMIC_DRAW)
        GL.glEnableVertexAttribArray(0)
        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, 6 * 4, GL.ctypes.c_void_p(0))
        GL.glEnableVertexAttribArray(1)
        GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, GL.GL_FALSE, 6 * 4, GL.ctypes.c_void_p(3 * 4))
        GL.glBindVertexArray(0)
        return vbo, vao

    def _make_lines_vao(self, verts: np.ndarray, colors: np.ndarray):
        n = len(verts)
        data = np.concatenate([verts.astype(np.float32, copy=False),
                               colors.astype(np.float32, copy=False)], axis=1).flatten()
        vbo = GL.glGenBuffers(1)
        vao = GL.glGenVertexArrays(1)
        GL.glBindVertexArray(vao)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, data.nbytes, data, GL.GL_STATIC_DRAW)
        GL.glEnableVertexAttribArray(0)
        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, 6 * 4, GL.ctypes.c_void_p(0))
        GL.glEnableVertexAttribArray(1)
        GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, GL.GL_FALSE, 6 * 4, GL.ctypes.c_void_p(3 * 4))
        GL.glBindVertexArray(0)
        return vao, vbo, n

    def _upload_submap(self, sm_id: str, geom: SubmapGeom) -> None:
        n = len(geom.xyz)
        data = np.concatenate([geom.xyz, geom.rgb], axis=1).astype(np.float32).flatten()
        vbo = GL.glGenBuffers(1)
        vao = GL.glGenVertexArrays(1)
        GL.glBindVertexArray(vao)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, data.nbytes, data, GL.GL_STATIC_DRAW)
        GL.glEnableVertexAttribArray(0)
        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, 6 * 4, GL.ctypes.c_void_p(0))
        GL.glEnableVertexAttribArray(1)
        GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, GL.GL_FALSE, 6 * 4, GL.ctypes.c_void_p(3 * 4))
        GL.glBindVertexArray(0)
        self._submap_vaos[sm_id] = vao
        logger.info("submap %s 上传 GL: %d 点", sm_id, n)

    def _upload_robot_glow(self, world_pos: np.ndarray) -> None:
        data = np.array(
            [world_pos[0], world_pos[1], world_pos[2], 1.0, 1.6, 0.6],
            dtype=np.float32,
        )
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self._robot_glow_vbo)
        GL.glBufferSubData(GL.GL_ARRAY_BUFFER, 0, data.nbytes, data)

    def _upload_lidar(self, interleaved: np.ndarray, seq: int) -> None:
        if seq == self._gpu_lidar_seq:
            return
        n = len(interleaved)
        if n == 0:
            self._lidar_count = 0
            self._gpu_lidar_seq = seq
            return
        if n > self._lidar_capacity:
            interleaved = interleaved[:self._lidar_capacity]
            n = self._lidar_capacity
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self._lidar_vbo)
        GL.glBufferSubData(GL.GL_ARRAY_BUFFER, 0, interleaved.nbytes, interleaved)
        self._lidar_count = n
        self._gpu_lidar_seq = seq

    # ── FBO ─────────────────────────────────────────────────────────

    def _recreate_fbos(self, w: int, h: int) -> None:
        w, h = max(1, int(w)), max(1, int(h))
        if (w, h) == (self._fbo_w, self._fbo_h):
            return
        for fbo in [self._scene_fbo, self._bright_fbo] + self._blur_fbo:
            if fbo:
                GL.glDeleteFramebuffers(1, [fbo])
        for tex in [self._scene_color, self._scene_depth, self._bright_color] + self._blur_color:
            if tex:
                GL.glDeleteTextures(1, [tex])

        self._scene_color = self._make_color_tex(w, h, hdr=True)
        self._scene_depth = self._make_depth_tex(w, h)
        self._scene_fbo = self._make_fbo(self._scene_color, self._scene_depth)

        hw, hh = max(1, w // 2), max(1, h // 2)
        self._bright_color = self._make_color_tex(hw, hh, hdr=True)
        self._bright_fbo = self._make_fbo(self._bright_color, 0)
        self._blur_color = [self._make_color_tex(hw, hh, hdr=True) for _ in range(2)]
        self._blur_fbo = [self._make_fbo(c, 0) for c in self._blur_color]

        self._fbo_w, self._fbo_h = w, h
        logger.info("FBO 重建: %dx%d (bright/blur=%dx%d)", w, h, hw, hh)

    def _make_color_tex(self, w: int, h: int, hdr: bool = True) -> int:
        tex = GL.glGenTextures(1)
        GL.glBindTexture(GL.GL_TEXTURE_2D, tex)
        if hdr:
            GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_R11F_G11F_B10F, w, h, 0,
                            GL.GL_RGB, GL.GL_FLOAT, None)
        else:
            GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_RGBA8, w, h, 0,
                            GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, None)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, GL.GL_CLAMP_TO_EDGE)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, GL.GL_CLAMP_TO_EDGE)
        return int(tex)

    def _make_depth_tex(self, w: int, h: int) -> int:
        tex = GL.glGenTextures(1)
        GL.glBindTexture(GL.GL_TEXTURE_2D, tex)
        GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_DEPTH_COMPONENT24, w, h, 0,
                        GL.GL_DEPTH_COMPONENT, GL.GL_FLOAT, None)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_NEAREST)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_NEAREST)
        return int(tex)

    def _make_fbo(self, color_tex: int, depth_tex: int) -> int:
        fbo = GL.glGenFramebuffers(1)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, fbo)
        GL.glFramebufferTexture2D(GL.GL_FRAMEBUFFER, GL.GL_COLOR_ATTACHMENT0,
                                  GL.GL_TEXTURE_2D, color_tex, 0)
        if depth_tex:
            GL.glFramebufferTexture2D(GL.GL_FRAMEBUFFER, GL.GL_DEPTH_ATTACHMENT,
                                      GL.GL_TEXTURE_2D, depth_tex, 0)
        status = GL.glCheckFramebufferStatus(GL.GL_FRAMEBUFFER)
        if status != GL.GL_FRAMEBUFFER_COMPLETE:
            raise RuntimeError(f"FBO 不完整 status=0x{status:x}")
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)
        return int(fbo)

    # ── tick / 交互 ────────────────────────────────────────────────

    def _tick(self) -> None:
        self._cnt_tick += 1
        if self._bridge is not None:
            lidar_seq = self._bridge.peek_lidar_seq()
            robot_stamp = self._bridge.peek_robot_stamp()
            if (lidar_seq != self._last_drawn_lidar_seq
                    or robot_stamp != self._last_drawn_robot_stamp):
                self._dirty = True
        if self._dirty:
            self._dirty = False
            self.update()
        else:
            self._cnt_skip_tick += 1

    def _mark_dirty(self) -> None:
        self._dirty = True

    def closeEvent(self, ev) -> None:
        try:
            self._egl.destroy()
        except Exception:
            pass
        super().closeEvent(ev)

    # ── 窗口生命周期：从隐藏/最小化恢复时强制重画 ────────────────
    # 否则 _tick 在隐藏期间 self.update() 被 Qt 吞掉，且我们已经把
    # _last_drawn_lidar_seq 推到了"最新值"，恢复后 _tick 觉得没新数据，
    # 既不画 lidar 也不画机器人 → 整窗一直是上一帧（或全黑）。

    def showEvent(self, ev) -> None:
        self._force_full_redraw()
        super().showEvent(ev)

    def changeEvent(self, ev) -> None:
        if ev.type() == QtCore.QEvent.WindowStateChange:
            # 从最小化恢复 → 强制重画
            if not bool(self.windowState() & QtCore.Qt.WindowMinimized):
                self._force_full_redraw()
        elif ev.type() in (QtCore.QEvent.ActivationChange,):
            if self.isActiveWindow():
                self._force_full_redraw()
        super().changeEvent(ev)

    def event(self, ev) -> bool:
        # Expose（X11 窗口被遮挡部分露出来）也兜一手
        if ev.type() == QtCore.QEvent.Expose:
            self._force_full_redraw()
        return super().event(ev)

    def _force_full_redraw(self) -> None:
        """重置数据版本水位线，让下一帧一定真正渲染。"""
        self._last_drawn_lidar_seq = -1
        self._last_drawn_robot_stamp = -1
        self._dirty = True
        # update() 在 widget 可见时才会真正排队 paintEvent；
        # 不可见时 Qt 自己会忽略，所以这里直接调用安全。
        self.update()

    def mousePressEvent(self, ev):
        self._last_mouse = (ev.x(), ev.y())
        self._last_button = ev.button()
        self.setFocus()

    def mouseMoveEvent(self, ev):
        if self._last_mouse is None:
            return
        dx = ev.x() - self._last_mouse[0]
        dy = ev.y() - self._last_mouse[1]
        self._last_mouse = (ev.x(), ev.y())
        if self._last_button == QtCore.Qt.LeftButton:
            mods = QtWidgets.QApplication.keyboardModifiers()
            if mods & QtCore.Qt.ShiftModifier:
                self.camera.pan(dx / self.height(), dy / self.height())
            else:
                # 鼠标拖动方向 = 场景旋转方向（左右本来就是同向，
                # 这里把 dy 也改成 +dy，让上下也是"鼠标↑→场景↑"，统一手感）
                self.camera.rotate(-dx * 0.35, dy * 0.35)
        elif self._last_button == QtCore.Qt.MiddleButton:
            self.camera.pan(dx / self.height(), dy / self.height())
        self._mark_dirty()

    def mouseReleaseEvent(self, ev):
        self._last_button = None

    def wheelEvent(self, ev):
        steps = ev.angleDelta().y() / 120.0
        self.camera.zoom(0.9 ** steps)
        self._mark_dirty()

    def keyPressEvent(self, ev):
        k = ev.key()
        if k == QtCore.Qt.Key_F:
            self.toggle_follow(); self._mark_dirty()
        elif k == QtCore.Qt.Key_R:
            self.reset_camera(); self._mark_dirty()
        elif k == QtCore.Qt.Key_B:
            self.toggle_bloom(); self._mark_dirty()
        elif k == QtCore.Qt.Key_C:
            self.circular_points = not self.circular_points
            self._mark_dirty()
        super().keyPressEvent(ev)
