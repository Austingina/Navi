"""把 GL viewer 的每帧 BGRA 像素流写到视频文件。

实现：subprocess 起一个 ffmpeg，stdin 接收原始 BGRA 帧，stdout 编码 H.264。
喂帧走 queue + 后台线程，不阻塞渲染主循环（队列满了就丢帧，宁可掉帧也不卡画面）。

Jetson 上 ffmpeg 通常已经装好（apt 包），并且 libx264 可用。

用法：
    rec = VideoRecorder("/tmp/demo.mp4", w=1280, h=800, fps=15)
    rec.start()
    # ... 每帧渲染完调一次 ...
    rec.feed(pixels_bgra_uint8_HxWx4)
    rec.stop()   # 退出时
"""

from __future__ import annotations
import logging
import os
import queue
import shutil
import subprocess
import threading
from typing import Optional

import numpy as np

logger = logging.getLogger("gl_viewer.recorder")


class VideoRecorder:
    """异步录制 BGRA 帧序列到视频文件。"""

    def __init__(self, path: str, w: int, h: int, fps: int = 15,
                 crf: int = 23, preset: str = "veryfast",
                 queue_size: int = 6):
        self.path = path
        self.w = int(w)
        self.h = int(h)
        self.fps = int(fps)
        self.crf = int(crf)
        self.preset = preset

        # 容量小一些：满了直接丢帧，不让录制反压到渲染
        self._q: "queue.Queue[bytes]" = queue.Queue(maxsize=queue_size)
        self._stop_evt = threading.Event()
        self._proc: Optional[subprocess.Popen] = None
        self._worker: Optional[threading.Thread] = None
        self._frames_fed = 0
        self._frames_dropped = 0

    # ── 公开 API ─────────────────────────────────────────────────────

    def start(self) -> None:
        if shutil.which("ffmpeg") is None:
            raise RuntimeError("找不到 ffmpeg；请 `sudo apt install ffmpeg`。")

        os.makedirs(os.path.dirname(os.path.abspath(self.path)) or ".", exist_ok=True)
        self._proc = subprocess.Popen(
            self._build_cmd(),
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            bufsize=0,
        )
        self._worker = threading.Thread(target=self._loop, name="gl_recorder",
                                        daemon=True)
        self._worker.start()
        logger.info("录制已启动: %s (%dx%d @ %dFPS)",
                    self.path, self.w, self.h, self.fps)

    def feed(self, pixels_bgra: np.ndarray) -> None:
        """喂一帧。pixels 必须是 (H, W, 4) uint8 BGRA。"""
        if self._proc is None:
            return
        if pixels_bgra.shape[:2] != (self.h, self.w):
            # 尺寸不匹配：通常意味着用户 resize 了窗口；为了不让 ffmpeg 报错
            # 直接丢帧（主入口 main.py 已经把窗口 setFixedSize 锁死了）
            self._frames_dropped += 1
            return
        try:
            self._q.put_nowait(pixels_bgra.tobytes())
            self._frames_fed += 1
        except queue.Full:
            self._frames_dropped += 1

    def stop(self) -> None:
        if self._proc is None:
            return
        self._stop_evt.set()
        # 等 worker 把 queue 清空
        if self._worker is not None:
            self._worker.join(timeout=3.0)
        # 关 ffmpeg stdin → ffmpeg flush 编码缓冲 → 退出
        try:
            if self._proc.stdin:
                self._proc.stdin.close()
        except Exception:
            pass
        try:
            self._proc.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            self._proc.kill()
        logger.info("录制结束: %s（喂 %d 帧 / 丢 %d 帧）",
                    self.path, self._frames_fed, self._frames_dropped)
        self._proc = None

    @property
    def frames_fed(self) -> int:
        return self._frames_fed

    @property
    def frames_dropped(self) -> int:
        return self._frames_dropped

    # ── 内部 ─────────────────────────────────────────────────────────

    def _build_cmd(self):
        return [
            "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
            # 输入：原始 BGRA
            "-f", "rawvideo",
            "-pix_fmt", "bgra",
            "-s", f"{self.w}x{self.h}",
            "-r", str(self.fps),
            "-i", "-",
            # 输出：H.264 mp4
            "-c:v", "libx264",
            "-preset", self.preset,
            "-crf", str(self.crf),
            "-pix_fmt", "yuv420p",   # 多数播放器要求
            # 让宽高都是偶数（H.264 要求；窗口尺寸如果是奇数会失败）
            "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
            self.path,
        ]

    def _loop(self) -> None:
        assert self._proc is not None and self._proc.stdin is not None
        stdin = self._proc.stdin
        while True:
            try:
                buf = self._q.get(timeout=0.5)
            except queue.Empty:
                if self._stop_evt.is_set():
                    break
                continue
            try:
                stdin.write(buf)
            except Exception:
                logger.exception("写 ffmpeg 失败，录制中断")
                break
