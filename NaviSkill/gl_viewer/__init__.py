"""NaviSkill GL Viewer：基于 PyQt + OpenGL 的"演示级"点云/机器人可视化。

设计目标：
- 比 rerun 视觉效果更"震撼"：点光源、Phong-lite 着色、深度雾、HDR + Bloom
- 完全独立于 rerun：自己起 rclpy 节点订阅 ROS topic，无需 server.py 也能跑
- 可以从 server.py 用 ``--gl_viewer`` 一键起子进程
"""

from .scene import Scene  # noqa: F401
from .camera import OrbitCamera  # noqa: F401
