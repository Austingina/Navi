"""GL Viewer 独立入口。

用法：
    python3 -m gl_viewer.main -m /path/to/map.json
    python3 -m gl_viewer.main -m /path/to/map.json --lidar /cloud_registered

也可以从 server.py ``--gl_viewer`` 一键启动（会 fork 一个子进程跑此模块）。
"""

from __future__ import annotations
import argparse
import logging
import os
import sys
from pathlib import Path

# ⚠ 必须在 `from OpenGL import GL` 之前设置！
#
# 1) PYOPENGL_PLATFORM=egl
#    PyOpenGL 默认假设平台是 GLX（X11 桌面下走 glXGetCurrentContext）。
#    我们的 GL context 是裸 EGL 创建的，GLX 找不到 → state 跟踪函数
#    （glVertexAttribPointer 等）会抛 "no valid context"。
#    强制 PyOpenGL 用 EGL platform，让它通过 eglGetCurrentContext 查我们的 context。
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

# 2) 关闭 PyOpenGL 的逐调用 glGetError 检查（默认开，纯 Python 开销巨大）。
#    设 GL_VIEWER_DEBUG_GL=1 可以重新打开调试。
if not os.environ.get("GL_VIEWER_DEBUG_GL"):
    try:
        import OpenGL as _OG  # noqa: N814
        _OG.ERROR_CHECKING = False
        _OG.ERROR_LOGGING = False
        # 注意：不能开 ERROR_ON_COPY，否则 list 参数（glDeleteFramebuffers 等）会抛错。
    except Exception:
        pass


def setup_paths():
    """允许 ``python3 -m gl_viewer.main`` 和直接脚本执行两种方式。"""
    here = Path(__file__).resolve().parent.parent
    if str(here) not in sys.path:
        sys.path.insert(0, str(here))


def main() -> int:
    setup_paths()

    parser = argparse.ArgumentParser(description="NaviSkill GL Viewer (PyQt + OpenGL)")
    parser.add_argument("-m", "--map", type=str, required=True,
                        help="地图 JSON 路径（如 /home/nvidia/Navi/maps/408/408.json）")
    parser.add_argument("--lidar", type=str, default="/cloud_registered",
                        help="LiDAR topic（默认 /cloud_registered，可选 /livox/lidar 等）")
    parser.add_argument("--voxel_size", type=float, default=0.5,
                        help="点云体素降采样边长（米），默认 0.5m。设 <=0 关闭。")
    parser.add_argument("--max_submap_points", type=int, default=1_500_000,
                        help="每 submap 体素降采样后的兜底上限（防止稀疏户外场景过大）")
    parser.add_argument("--max_lidar_points", type=int, default=20_000,
                        help="单帧 lidar 最大点数（throttle 后）")
    parser.add_argument("--lidar_hz", type=float, default=5.0,
                        help="lidar 处理上限频率（spin 线程 throttle，多余帧直接丢）")
    parser.add_argument("--tf_hz", type=float, default=2.0,
                        help="TF 查询频率（spin 线程，多了没用，2-5Hz 够演示）")
    parser.add_argument("--target_fps", type=float, default=15.0,
                        help="渲染目标 FPS。静止时几乎不渲染；有新数据时按此节奏")
    parser.add_argument("--no_bloom", action="store_true",
                        help="关闭 bloom 后处理（省 4 个 pass，显著减 GPU 负担）")
    parser.add_argument("--window", type=str, default="1280x800",
                        help="初始窗口尺寸 WxH")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="[%(name)s] %(levelname)s: %(message)s",
    )

    # ── Qt 端：用普通 X11 平台，完全不用 Qt 的 GL 上下文 ──
    # （viewer 自己用 NVIDIA EGL device 创建离屏 GL，渲染好后用 QImage 显示）
    if os.environ.get("QT_QPA_PLATFORM") is None:
        os.environ["QT_QPA_PLATFORM"] = "xcb"
    # 我们要的是 NVIDIA EGL（通过 EGL_PLATFORM_DEVICE_EXT），强制只允许 NVIDIA vendor
    if os.environ.get("__EGL_VENDOR_LIBRARY_FILENAMES") is None:
        nvidia_icd = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
        if os.path.exists(nvidia_icd):
            os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = nvidia_icd

    logging.info(
        "启动环境: QT_QPA_PLATFORM=%s | PYOPENGL_PLATFORM=%s | __EGL_VENDOR_LIBRARY_FILENAMES=%s",
        os.environ.get("QT_QPA_PLATFORM"),
        os.environ.get("PYOPENGL_PLATFORM"),
        os.environ.get("__EGL_VENDOR_LIBRARY_FILENAMES", "(default=both)"),
    )

    from map_manager import MapManager
    from gl_viewer.scene import Scene
    from gl_viewer.ros_bridge import RosBridge

    mm = MapManager(args.map)
    scene = Scene(mm, voxel_size=args.voxel_size,
                  max_submap_points=args.max_submap_points)
    scene.load_all_submaps()

    bridge = RosBridge(
        mm,
        lidar_topic=args.lidar,
        tf_lookup_hz=args.tf_hz,
        lidar_max_hz=args.lidar_hz,
        max_lidar_points=args.max_lidar_points,
    )
    bridge.start()

    from PySide2 import QtWidgets
    from gl_viewer.viewer import GLPointCloudWidget

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)

    w, h = 1280, 800
    if "x" in args.window:
        try:
            w, h = [int(x) for x in args.window.split("x")]
        except Exception:
            pass

    widget = GLPointCloudWidget(
        scene, bridge,
        target_fps=args.target_fps,
        enable_bloom=not args.no_bloom,
    )
    widget.resize(w, h)
    widget.setWindowTitle("NaviSkill GL Viewer")
    widget.show()

    rc = app.exec_()
    bridge.stop()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
