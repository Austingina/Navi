"""NVIDIA Tegra Orin 硬件加速 EGL 上下文（不依赖窗口系统）。

Jetson L4T 在 X11 桌面下，NVIDIA driver 不挂 Xorg，所以 X11+EGL 路径只能
拿到 Mesa llvmpipe 软件渲染。NVIDIA 硬件 GL 必须通过 EGL "device platform"
访问——纯离屏，无窗口。本模块封装这条路径。

用法：
    egl = OffscreenEGLContext(width=1280, height=800)
    egl.make_current()
    # ... 任意 OpenGL 调用 ...
    pixels = egl.read_pixels_bgra()   # 拿一帧给 QImage 显示
    egl.resize(new_w, new_h)
    egl.destroy()
"""

from __future__ import annotations
import ctypes
import ctypes.util
import logging
import numpy as np

logger = logging.getLogger("gl_viewer.egl_context")


# ── EGL 常量 ─────────────────────────────────────────────────────────
EGL_DEFAULT_DISPLAY = 0
EGL_NO_DISPLAY = 0
EGL_NO_CONTEXT = 0
EGL_NO_SURFACE = 0
EGL_TRUE = 1
EGL_FALSE = 0

EGL_PBUFFER_BIT = 0x0001
EGL_OPENGL_BIT = 0x0008

EGL_SURFACE_TYPE = 0x3033
EGL_RENDERABLE_TYPE = 0x3040
EGL_NONE = 0x3038
EGL_HEIGHT = 0x3056
EGL_WIDTH = 0x3057
EGL_RED_SIZE = 0x3024
EGL_GREEN_SIZE = 0x3023
EGL_BLUE_SIZE = 0x3022
EGL_ALPHA_SIZE = 0x3021
EGL_DEPTH_SIZE = 0x3025

EGL_OPENGL_API = 0x30A2

EGL_VENDOR = 0x3053
EGL_PLATFORM_DEVICE_EXT = 0x313F
EGL_PLATFORM_SURFACELESS_MESA = 0x31DD

EGL_CONTEXT_MAJOR_VERSION = 0x3098
EGL_CONTEXT_MINOR_VERSION = 0x30FB
EGL_CONTEXT_OPENGL_PROFILE_MASK = 0x30FD
EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT = 0x00000001

# GL 常量
GL_VENDOR = 0x1F00
GL_RENDERER = 0x1F01
GL_VERSION = 0x1F02
GL_BGRA = 0x80E1
GL_RGBA = 0x1908
GL_UNSIGNED_BYTE = 0x1401
GL_PACK_ALIGNMENT = 0x0D05


class OffscreenEGLContext:
    """走 NVIDIA EGL device 创建硬件加速 GL 3.3 Core context。

    维护一个 pbuffer surface 作为渲染目标。resize 时重建。
    """

    def __init__(self, width: int = 1280, height: int = 800,
                 gl_major: int = 3, gl_minor: int = 3):
        self._w = max(1, int(width))
        self._h = max(1, int(height))
        self._gl_major = gl_major
        self._gl_minor = gl_minor

        self._libegl = ctypes.CDLL(ctypes.util.find_library("EGL") or "libEGL.so.1")
        self._libgl = ctypes.CDLL(ctypes.util.find_library("GL") or "libGL.so.1")
        self._setup_bindings()

        self._display: int = 0
        self._context: int = 0
        self._surface: int = 0
        self._config: int = 0
        self._renderer_str: str = ""

        self._init_display_context()
        self._create_surface(self._w, self._h)

    # ── 公开 API ─────────────────────────────────────────────────────

    def make_current(self) -> None:
        if not self._libegl.eglMakeCurrent(
            self._display, self._surface, self._surface, self._context
        ):
            raise RuntimeError(f"eglMakeCurrent 失败 err=0x{self._libegl.eglGetError():x}")

    def release(self) -> None:
        self._libegl.eglMakeCurrent(self._display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT)

    def width(self) -> int:
        return self._w

    def height(self) -> int:
        return self._h

    def renderer(self) -> str:
        return self._renderer_str

    def resize(self, w: int, h: int) -> None:
        w, h = max(1, int(w)), max(1, int(h))
        if (w, h) == (self._w, self._h):
            return
        self.release()
        if self._surface:
            self._libegl.eglDestroySurface(self._display, self._surface)
            self._surface = 0
        self._create_surface(w, h)
        self.make_current()
        self._w, self._h = w, h

    def read_pixels_bgra(self) -> np.ndarray:
        """读 default framebuffer，返回 (H, W, 4) uint8 BGRA（已 vflip 适配 QImage）。"""
        buf = np.empty((self._h, self._w, 4), dtype=np.uint8)
        self._libgl.glPixelStorei(GL_PACK_ALIGNMENT, 4)
        self._libgl.glReadPixels(
            0, 0, self._w, self._h, GL_BGRA, GL_UNSIGNED_BYTE,
            buf.ctypes.data_as(ctypes.c_void_p),
        )
        return np.ascontiguousarray(buf[::-1])

    def destroy(self) -> None:
        try:
            self.release()
        except Exception:
            pass
        if self._surface:
            self._libegl.eglDestroySurface(self._display, self._surface)
            self._surface = 0
        if self._context:
            self._libegl.eglDestroyContext(self._display, self._context)
            self._context = 0
        if self._display:
            self._libegl.eglTerminate(self._display)
            self._display = 0

    # ── 内部 ─────────────────────────────────────────────────────────

    def _setup_bindings(self) -> None:
        e = self._libegl
        for fn, rest, args in [
            ("eglGetDisplay",          ctypes.c_void_p, [ctypes.c_void_p]),
            ("eglInitialize",          ctypes.c_int,    [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]),
            ("eglTerminate",           ctypes.c_int,    [ctypes.c_void_p]),
            ("eglChooseConfig",        ctypes.c_int,    [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_void_p), ctypes.c_int, ctypes.POINTER(ctypes.c_int)]),
            ("eglBindAPI",             ctypes.c_int,    [ctypes.c_int]),
            ("eglCreateContext",       ctypes.c_void_p, [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]),
            ("eglDestroyContext",      ctypes.c_int,    [ctypes.c_void_p, ctypes.c_void_p]),
            ("eglCreatePbufferSurface", ctypes.c_void_p, [ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]),
            ("eglDestroySurface",      ctypes.c_int,    [ctypes.c_void_p, ctypes.c_void_p]),
            ("eglMakeCurrent",         ctypes.c_int,    [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]),
            ("eglGetError",            ctypes.c_int,    []),
            ("eglQueryString",         ctypes.c_char_p, [ctypes.c_void_p, ctypes.c_int]),
            ("eglGetProcAddress",      ctypes.c_void_p, [ctypes.c_char_p]),
        ]:
            f = getattr(e, fn)
            f.restype = rest
            f.argtypes = args

        g = self._libgl
        g.glGetString.restype = ctypes.c_char_p
        g.glGetString.argtypes = [ctypes.c_uint]
        g.glReadPixels.restype = None
        g.glReadPixels.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                                    ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p]
        g.glPixelStorei.restype = None
        g.glPixelStorei.argtypes = [ctypes.c_uint, ctypes.c_int]

        addr = e.eglGetProcAddress(b"eglGetPlatformDisplayEXT")
        if not addr:
            raise RuntimeError("eglGetPlatformDisplayEXT 不可用（EGL 实现太老）")
        self._GetPlatformDisplay = ctypes.CFUNCTYPE(
            ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)
        )(addr)
        addr = e.eglGetProcAddress(b"eglQueryDevicesEXT")
        self._QueryDevices = None
        if addr:
            self._QueryDevices = ctypes.CFUNCTYPE(
                ctypes.c_int, ctypes.c_int,
                ctypes.POINTER(ctypes.c_void_p),
                ctypes.POINTER(ctypes.c_int)
            )(addr)

    def _open_nvidia_display(self):
        """优先用 EGL device platform（NVIDIA Tegra 推荐），fallback 到 surfaceless。"""
        if self._QueryDevices:
            devs = (ctypes.c_void_p * 8)()
            ndev = ctypes.c_int(0)
            if self._QueryDevices(8, devs, ctypes.byref(ndev)) and ndev.value > 0:
                d = self._GetPlatformDisplay(EGL_PLATFORM_DEVICE_EXT, devs[0], None)
                if d:
                    return d, "EGL_PLATFORM_DEVICE_EXT"
        d = self._GetPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA, None, None)
        if d:
            return d, "EGL_PLATFORM_SURFACELESS_MESA"
        return self._libegl.eglGetDisplay(EGL_DEFAULT_DISPLAY), "EGL_DEFAULT_DISPLAY"

    def _init_display_context(self) -> None:
        e = self._libegl
        self._display, plat = self._open_nvidia_display()
        if not self._display:
            raise RuntimeError("eglGetDisplay 失败：没有任何可用 EGL display")
        logger.info("EGL display 通过 %s 创建", plat)

        maj, mn = ctypes.c_int(0), ctypes.c_int(0)
        if not e.eglInitialize(self._display, ctypes.byref(maj), ctypes.byref(mn)):
            raise RuntimeError(f"eglInitialize 失败 err=0x{e.eglGetError():x}")
        v = e.eglQueryString(self._display, EGL_VENDOR)
        logger.info("EGL %d.%d  vendor=%s", maj.value, mn.value, v.decode() if v else "?")

        cfg_attribs = (ctypes.c_int * 15)(
            EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
            EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8,
            EGL_DEPTH_SIZE, 24,
            EGL_NONE,
        )
        cfg = ctypes.c_void_p(0)
        num = ctypes.c_int(0)
        if not e.eglChooseConfig(self._display, cfg_attribs, ctypes.byref(cfg), 1, ctypes.byref(num)) or num.value == 0:
            raise RuntimeError(f"eglChooseConfig 失败 err=0x{e.eglGetError():x}")
        self._config = cfg.value

        if not e.eglBindAPI(EGL_OPENGL_API):
            raise RuntimeError(f"eglBindAPI(OPENGL_API) 失败 err=0x{e.eglGetError():x}")

        ctx_attribs = (ctypes.c_int * 7)(
            EGL_CONTEXT_MAJOR_VERSION, self._gl_major,
            EGL_CONTEXT_MINOR_VERSION, self._gl_minor,
            EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
            EGL_NONE,
        )
        self._context = e.eglCreateContext(self._display, self._config, None, ctx_attribs)
        if not self._context:
            raise RuntimeError(f"eglCreateContext 失败 err=0x{e.eglGetError():x}")

    def _create_surface(self, w: int, h: int) -> None:
        e = self._libegl
        attribs = (ctypes.c_int * 5)(EGL_WIDTH, w, EGL_HEIGHT, h, EGL_NONE)
        self._surface = e.eglCreatePbufferSurface(self._display, self._config, attribs)
        if not self._surface:
            raise RuntimeError(f"eglCreatePbufferSurface({w}x{h}) 失败 err=0x{e.eglGetError():x}")
        if not self._renderer_str:
            self.make_current()
            self._renderer_str = (self._libgl.glGetString(GL_RENDERER) or b"?").decode()
            vendor = (self._libgl.glGetString(GL_VENDOR) or b"?").decode()
            version = (self._libgl.glGetString(GL_VERSION) or b"?").decode()
            logger.warning("──────────────────────────────────────────────")
            logger.warning("  GL_VENDOR   = %s", vendor)
            logger.warning("  GL_RENDERER = %s", self._renderer_str)
            logger.warning("  GL_VERSION  = %s", version)
            if "NVIDIA" in vendor:
                logger.warning("  ✅ NVIDIA 硬件加速")
            else:
                logger.error("  ❌ 仍未拿到 NVIDIA 硬件加速")
            logger.warning("──────────────────────────────────────────────")
