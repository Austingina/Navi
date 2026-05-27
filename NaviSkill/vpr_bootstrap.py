"""VPR-based submap bootstrap / re-localization helper for ``server.py``.

This wraps ``NaviSkill/vpr/vpr_detect.py`` running in **service mode**
(``--serve``) as a long-lived subprocess. The subprocess uses its own
virtual environment (PyTorch / torchvision / faiss / loguru, tuned for the
target machine -- typically a Jetson), which is intentionally decoupled from
NaviSkill's conda environment that only carries ROS + FastAPI deps.

NaviSkill (parent)
    ──spawn──>  python .venv/bin/python vpr_detect.py --serve --databases ...
    <──"ready" line on stdout
    ──request lines on stdin   (one JSON per line, with id+image_path+top_k)
    <──response lines on stdout

A ``locate()`` call:
    1. writes the live JPEG bytes to a temp file,
    2. sends a query line to the subprocess,
    3. reads the matching response line,
    4. maps the winning database directory back to a submap id.

The detector subprocess validates that all submap databases were built with
the same VPR model / image_size / undistortion settings, so the user just
needs to drop ``database_*.{npy,npz,txt,json}`` under
``<project_dir>/vpr/<submap_id>/`` for every submap they want to be
recognizable.
"""

from __future__ import annotations

import json
import logging
import os
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from typing import Any, Callable, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)
# 让本模块的 INFO 日志可见：server.py 把根 logger 设到了 WARNING，会把
# 我们的 INFO 全部过滤掉。这里给本模块单独挂一个 INFO 级别的 StreamHandler，
# 并关掉 propagate 以避免 WARNING+ 同时也走根 handler 造成重复输出。
logger.setLevel(logging.INFO)
if not any(
    getattr(h, "_vpr_bootstrap_handler", False) for h in logger.handlers
):
    _vpr_handler = logging.StreamHandler()
    _vpr_handler.setLevel(logging.INFO)
    _vpr_handler.setFormatter(
        logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )
    )
    _vpr_handler._vpr_bootstrap_handler = True  # type: ignore[attr-defined]
    logger.addHandler(_vpr_handler)
    logger.propagate = False


# Where vpr_detect.py and its dedicated venv live. Kept here so server.py
# doesn't need to know about layout details. The VPR module is a sibling of
# this file (``NaviSkill/vpr/``); the dedicated venv lives at
# ``NaviSkill/vpr/.venv/`` and is bootstrapped by ``vpr/install_jetson.sh``.
_DEFAULT_VPR_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "vpr")
)


# 与构建数据库时使用的 R1Pro 图像规格保持一致：先去畸变，再中心裁剪到 1024x768。
# 这样在线查询的图像几何尺寸与离线建库的图像保持一致，否则会显著掉精度。
_VPR_QUERY_TARGET_W = 1024
_VPR_QUERY_TARGET_H = 768

# 延迟导入：cv2 / numpy / PinHoleUndistorter 都依赖于本进程能装载 OpenCV，
# 一次性失败就关掉预处理，让查询退化为 "原图直接送" 的旧行为。
_preprocess_init_lock = threading.Lock()
_preprocess_initialized = False
_preprocess_enabled = False
_cv2 = None
_np = None
_pinhole_undistorter = None  # type: Optional[object]


def _ensure_preprocess_loaded() -> bool:
    """加载 cv2 / numpy / PinHoleUndistorter，返回是否可用。"""
    global _preprocess_initialized, _preprocess_enabled
    global _cv2, _np, _pinhole_undistorter
    if _preprocess_initialized:
        return _preprocess_enabled
    with _preprocess_init_lock:
        if _preprocess_initialized:
            return _preprocess_enabled
        try:
            import cv2  # type: ignore
            import numpy as np  # type: ignore
            if _DEFAULT_VPR_DIR not in sys.path:
                sys.path.insert(0, _DEFAULT_VPR_DIR)
            from utils.undistort2 import PinHoleUndistorter  # type: ignore

            _cv2 = cv2
            _np = np
            _pinhole_undistorter = PinHoleUndistorter.from_defaults()
            _preprocess_enabled = True
            logger.info(
                "VprBootstrap: 已启用查询图像预处理（PinHoleUndistorter 去畸变 + 中心裁剪到 %dx%d）",
                _VPR_QUERY_TARGET_W, _VPR_QUERY_TARGET_H,
            )
        except Exception as e:
            _preprocess_enabled = False
            logger.warning(
                "VprBootstrap: 无法加载 PinHoleUndistorter (%s)，"
                "查询图像将直接送给 VPR（不去畸变 / 不裁剪），可能影响匹配精度。",
                e,
            )
        finally:
            _preprocess_initialized = True
    return _preprocess_enabled


def _undistort_and_center_crop(
    jpeg_bytes: bytes,
    target_w: int = _VPR_QUERY_TARGET_W,
    target_h: int = _VPR_QUERY_TARGET_H,
) -> bytes:
    """对 JPEG 字节流去畸变并中心裁剪到 target_w x target_h，返回新的 JPEG 字节。

    若任一步失败（解码失败 / 尺寸过小 / 编码失败），返回原始 jpeg_bytes，
    保证在线查询路径不会因为预处理异常而整体失败。
    """
    if not _ensure_preprocess_loaded():
        return jpeg_bytes

    cv2 = _cv2
    np = _np
    undistorter = _pinhole_undistorter
    try:
        buf = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if img is None:
            logger.warning("VprBootstrap: JPEG 解码失败，预处理跳过")
            return jpeg_bytes

        undistorted = undistorter.undistort(img)
        h, w = undistorted.shape[:2]
        if w < target_w or h < target_h:
            logger.warning(
                "VprBootstrap: 去畸变后图像尺寸 %dx%d 小于目标 %dx%d，跳过裁剪",
                w, h, target_w, target_h,
            )
            cropped = undistorted
        else:
            x0 = (w - target_w) // 2
            y0 = (h - target_h) // 2
            cropped = undistorted[y0:y0 + target_h, x0:x0 + target_w]

        ok, enc = cv2.imencode(".jpg", cropped, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            logger.warning("VprBootstrap: JPEG 编码失败，回退到原始字节")
            return jpeg_bytes
        return enc.tobytes()
    except Exception as e:
        logger.warning(
            "VprBootstrap: 查询图像预处理异常 (%s)，回退到原始字节", e,
        )
        return jpeg_bytes


def save_consensus_collage(
    items: List[tuple],
    save_path: str,
    *,
    tile_w: int = 400,
    tile_h: int = 300,
    pad: int = 8,
    title: Optional[str] = None,
) -> bool:
    """把若干 (query_image_path, locate_result) 配对拼成 2 行 N 列大图保存。

    顶行：依次为 query 图（标 Q1..QN）。
    底行：依次为 locate 结果的 top-1 候选图（标 submap_id / inliers / dist）。

    任一项 query 缺失或 result 为 None 时会在对应格子里放 "MISSING"/"FAIL" 占位。

    :param items: 每帧的 ``(query_image_path: str|None, result: dict|None)``。
                  ``result`` 期望含 ``image_path``、``submap_id``、
                  可选 ``rerank_inliers`` / ``distance``。
    :param save_path: 输出 JPEG 路径（一般放 ``/tmp/...``）。
    :param title: 可选的总标题（写在顶部一条额外条带里）。
    :return: True 写盘成功；False 失败（cv2 不可用 / 写入异常）。
    """
    if not _ensure_preprocess_loaded():
        logger.warning("save_consensus_collage: cv2/numpy 不可用，跳过保存")
        return False
    cv2 = _cv2
    np = _np

    n = len(items)
    if n == 0:
        return False

    label_h = 28
    title_h = 32 if title else 0
    canvas_w = n * tile_w + (n + 1) * pad
    canvas_h = title_h + 2 * (tile_h + label_h) + 3 * pad
    canvas = np.full((canvas_h, canvas_w, 3), 245, dtype=np.uint8)

    if title:
        cv2.putText(
            canvas, title, (pad, title_h - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2,
        )

    def _load_tile(path: Optional[str], missing_label: str, color=(60, 60, 60)) -> "np.ndarray":
        if not path or not os.path.isfile(path):
            tile = np.full((tile_h, tile_w, 3), color, dtype=np.uint8)
            cv2.putText(
                tile, missing_label, (10, tile_h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2,
            )
            return tile
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            return _load_tile(None, missing_label, color)
        h, w = img.shape[:2]
        scale = min(tile_w / w, tile_h / h)
        new_w, new_h = max(1, int(w * scale)), max(1, int(h * scale))
        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
        tile = np.zeros((tile_h, tile_w, 3), dtype=np.uint8)
        x0 = (tile_w - new_w) // 2
        y0 = (tile_h - new_h) // 2
        tile[y0:y0 + new_h, x0:x0 + new_w] = resized
        return tile

    def _put_label(text: str, x0: int, y_top: int):
        cv2.rectangle(
            canvas, (x0, y_top), (x0 + tile_w, y_top + label_h),
            (230, 230, 230), -1,
        )
        cv2.putText(
            canvas, text, (x0 + 4, y_top + label_h - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1,
        )

    y_q = title_h + pad
    y_q_label = y_q + tile_h
    y_m = y_q_label + label_h + pad
    y_m_label = y_m + tile_h

    for col, item in enumerate(items):
        q_path, result = item if item is not None else (None, None)
        x0 = pad + col * (tile_w + pad)

        canvas[y_q:y_q + tile_h, x0:x0 + tile_w] = _load_tile(q_path, "MISSING")
        _put_label(f"Q{col + 1}", x0, y_q_label)

        if result and result.get("image_path"):
            canvas[y_m:y_m + tile_h, x0:x0 + tile_w] = _load_tile(
                result["image_path"], "MATCH MISSING",
            )
            sm = result.get("submap_id", "?")
            inl = result.get("rerank_inliers")
            dist = result.get("distance")
            parts = [str(sm)]
            if inl is not None and inl >= 0:
                parts.append(f"inl={inl}")
            if dist is not None:
                try:
                    parts.append(f"d={float(dist):.3f}")
                except (TypeError, ValueError):
                    pass
            _put_label(" ".join(parts)[: 60], x0, y_m_label)
        else:
            canvas[y_m:y_m + tile_h, x0:x0 + tile_w] = _load_tile(
                None, "FAIL", color=(20, 20, 80),
            )
            _put_label("no result", x0, y_m_label)

    try:
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        ok = cv2.imwrite(save_path, canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    except Exception as e:
        logger.warning(f"save_consensus_collage: 写盘失败 {save_path}: {e}")
        return False
    return bool(ok)


class VprBootstrapError(RuntimeError):
    """Raised when the VPR helper cannot complete a request."""


class VprBootstrap:
    """Drive vpr_detect.py --serve as a subprocess and expose ``locate()``.

    Parameters
    ----------
    map_manager : MapManager
        Used to enumerate submaps and to resolve ``project_dir`` from the
        loaded multifloor config.
    vpr_dir : str, optional
        Override the VPR module directory (defaults to ``./vpr`` relative
        to this file -- i.e. ``NaviSkill/vpr/``).
    python_executable : str, optional
        Python interpreter to invoke (defaults to the venv at
        ``<vpr_dir>/.venv/bin/python``).
    device : str
        ``cuda`` or ``cpu`` -- passed to ``vpr_detect.py``.
    ready_timeout_sec : float
        How long to wait for the ``"ready"`` event after spawning the
        subprocess (model + databases load on Jetson is ~10 s the first
        time).
    """

    READY_TIMEOUT_DEFAULT = 60.0
    QUERY_TIMEOUT_DEFAULT = 30.0
    # 子进程 stderr 的环形缓冲长度——用于在子进程未 ready 就退出时回显
    _STDERR_TAIL_MAX = 80

    def __init__(
        self,
        map_manager,
        vpr_dir: Optional[str] = None,
        python_executable: Optional[str] = None,
        device: str = "cuda",
        ready_timeout_sec: float = READY_TIMEOUT_DEFAULT,
    ):
        self.map_manager = map_manager
        self.vpr_dir = os.path.abspath(vpr_dir or _DEFAULT_VPR_DIR)
        self.python_executable = python_executable or os.path.join(
            self.vpr_dir, ".venv", "bin", "python"
        )
        self.device = device
        self.ready_timeout_sec = ready_timeout_sec

        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()  # serialize stdin writes + stdout reads
        self._tmpdir: Optional[tempfile.TemporaryDirectory] = None
        self._stderr_thread: Optional[threading.Thread] = None
        # 子进程 stderr 的最近若干行，方便 _wait_for_ready 失败时回显
        self._stderr_tail: List[str] = []

        # 数据库路径 → submap_id
        self._db_to_submap: Dict[str, str] = {}
        # 数据库元信息（来自 ready 事件），缓存以便上层查看
        self.ready_info: Optional[dict] = None

        self._db_dirs: List[str] = self._discover_database_dirs()
        if not self._db_dirs:
            logger.warning(
                "VprBootstrap: 未发现任何 submap 的 VPR 数据库，submap 引导将被禁用"
            )

    # ------------------------------------------------------------------
    # Discovery & lifecycle
    # ------------------------------------------------------------------

    def _discover_database_dirs(self) -> List[str]:
        """Find ``<project_dir>/vpr/<submap_id>/`` directories that contain a
        complete VPR database (descriptors + poses + paths + meta)."""
        if self.map_manager is None:
            return []
        cfg_path = getattr(self.map_manager, "config_path", None)
        if not cfg_path:
            return []
        project_dir = os.path.dirname(os.path.abspath(cfg_path))

        dirs: List[str] = []
        for sm_id in self.map_manager.get_submap_list():
            db_dir = os.path.join(project_dir, "vpr", sm_id)
            required = [
                "database_descriptors.npy",
                "database_poses.npy",
                "database_image_paths.txt",
                "database_meta.npz",
            ]
            if all(os.path.isfile(os.path.join(db_dir, f)) for f in required):
                self._db_to_submap[os.path.abspath(db_dir)] = sm_id
                dirs.append(os.path.abspath(db_dir))
            else:
                logger.debug(
                    f"VprBootstrap: submap '{sm_id}' 缺少 VPR 数据库（{db_dir}），跳过"
                )
        return dirs

    @property
    def available(self) -> bool:
        """True if the subprocess has been successfully started and is alive."""
        if self._proc is None:
            return False
        return self._proc.poll() is None

    @property
    def has_databases(self) -> bool:
        return bool(self._db_dirs)

    @property
    def submap_ids(self) -> List[str]:
        return [self._db_to_submap[d] for d in self._db_dirs]

    def _resolve_device(self) -> str:
        """启动前探测 venv 里的 torch.cuda 是否真的可用；不可用则降级到 cpu。

        典型故障：``uv sync`` 把 Jetson 专用的 CUDA torch wheel 覆盖成了 PyPI
        的标准 CPU wheel（``torch==X.Y+cpu``），导致子进程 ``.to("cuda")``
        抛 ``AssertionError: Torch not compiled with CUDA enabled``。
        我们宁可让 VPR 慢点跑 CPU，也别让整个 watchdog 完全失效。
        """
        if self.device == "cpu":
            return "cpu"
        try:
            probe = subprocess.run(
                [
                    self.python_executable, "-c",
                    "import torch,sys; "
                    "sys.stdout.write('1' if torch.cuda.is_available() else '0')",
                ],
                capture_output=True, text=True, timeout=15,
            )
            ok = probe.returncode == 0 and probe.stdout.strip() == "1"
        except Exception as e:
            logger.warning(f"VprBootstrap: 探测 CUDA 失败 ({e})，降级到 cpu")
            return "cpu"
        if not ok:
            logger.error(
                "VprBootstrap: vpr venv 里的 torch 不支持 CUDA（probe stdout=%r, "
                "stderr=%r），自动降级到 device=cpu；如需 GPU 加速，请按 "
                "install_jetson.sh 重新安装 NVIDIA 的 Jetson torch wheel。",
                probe.stdout.strip() if probe else "", probe.stderr.strip() if probe else "",
            )
            return "cpu"
        return "cuda"

    def start(self) -> bool:
        """Spawn the vpr_detect.py --serve subprocess and wait for ready.

        :return: True on success, False if no databases or startup failed.
        """
        if self._proc is not None:
            return self.available
        if not self._db_dirs:
            return False
        if not os.path.isfile(self.python_executable):
            logger.error(
                f"VprBootstrap: 解释器不存在 {self.python_executable}，"
                f"请先按 install_jetson.sh 配置 vpr 的 .venv"
            )
            return False

        # 探测后可能从 cuda 降级到 cpu；以最终值为准启动子进程
        self.device = self._resolve_device()

        script_path = os.path.join(self.vpr_dir, "vpr_detect.py")
        cmd = [
            self.python_executable,
            "-u",  # 关闭 stdout/stderr 缓冲，保证协议帧实时到达
            script_path,
            "--serve",
            "--device", self.device,
            "--databases", *self._db_dirs,
        ]
        logger.info(
            "VprBootstrap: 启动 VPR 服务子进程 (device=%s, databases=%d) ...",
            self.device, len(self._db_dirs),
        )
        logger.debug("VprBootstrap cmd: %s", " ".join(cmd))

        # 大图临时文件目录（每次 locate() 写到这里，按需清理）
        self._tmpdir = tempfile.TemporaryDirectory(prefix="vpr_bootstrap_")

        try:
            self._proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,  # 行缓冲
                cwd=self.vpr_dir,
            )
        except Exception as e:
            logger.error(f"VprBootstrap: 启动子进程失败: {e}")
            self._proc = None
            return False

        # 把 stderr 中转到 logging（便于看到 loguru 输出 + 异常 traceback）
        self._stderr_thread = threading.Thread(
            target=self._pump_stderr, daemon=True, name="vpr-detect-stderr",
        )
        self._stderr_thread.start()

        if not self._wait_for_ready():
            logger.error(
                "VprBootstrap: 子进程未在 %.1fs 内 ready，将禁用 VPR 引导",
                self.ready_timeout_sec,
            )
            self.stop()
            return False

        logger.info(
            "VprBootstrap: VPR 服务就绪 (method=%s, dim=%d, image_size=%s, undistort=%s, 总图像=%d)",
            self.ready_info.get("method"),
            self.ready_info.get("descriptor_dim"),
            self.ready_info.get("image_size"),
            self.ready_info.get("undistort"),
            sum(db.get("num_images", 0) for db in self.ready_info.get("databases", [])),
        )
        for db in self.ready_info.get("databases", []):
            sm_id = self._db_to_submap.get(os.path.abspath(db.get("path", "")), "?")
            logger.info(
                "  - submap=%s  images=%d  path=%s",
                sm_id, db.get("num_images", 0), db.get("path"),
            )
        return True

    def stop(self) -> None:
        """Terminate the subprocess and clean up resources."""
        if self._proc is not None:
            try:
                if self._proc.stdin and not self._proc.stdin.closed:
                    try:
                        self._proc.stdin.close()
                    except Exception:
                        pass
                if self._proc.poll() is None:
                    self._proc.terminate()
                    try:
                        self._proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        self._proc.kill()
            except Exception:
                pass
            self._proc = None
        if self._tmpdir is not None:
            try:
                self._tmpdir.cleanup()
            except Exception:
                pass
            self._tmpdir = None

    def __del__(self):
        try:
            self.stop()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Subprocess I/O
    # ------------------------------------------------------------------

    def _pump_stderr(self) -> None:
        proc = self._proc
        if proc is None or proc.stderr is None:
            return
        try:
            for line in proc.stderr:
                line = line.rstrip()
                if not line:
                    continue
                # 兼顾两点：
                # 1) 让默认日志级别（WARNING）下也能看到 Python traceback /
                #    子进程的错误信息，方便定位 "ready 之前就退出" 这类故障。
                # 2) 保留最近若干行作为 ring buffer，便于在 ``stop()`` /
                #    ready-timeout 时一次性回显给上层日志。
                self._stderr_tail.append(line)
                if len(self._stderr_tail) > self._STDERR_TAIL_MAX:
                    self._stderr_tail.pop(0)
                logger.warning("[vpr_detect] %s", line)
        except Exception as e:
            logger.debug("VprBootstrap: stderr 抽取退出: %s", e)

    def _wait_for_ready(self) -> bool:
        proc = self._proc
        if proc is None or proc.stdout is None:
            return False
        deadline = time.monotonic() + self.ready_timeout_sec
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                logger.error(
                    "VprBootstrap: 子进程在 ready 之前退出 (code=%s)",
                    proc.returncode,
                )
                # 给 stderr pumping 线程一点时间把最后几行 flush 出来
                time.sleep(0.3)
                if self._stderr_tail:
                    logger.error(
                        "VprBootstrap: 子进程 stderr 最后 %d 行：\n%s",
                        len(self._stderr_tail),
                        "\n".join(self._stderr_tail),
                    )
                return False
            # readline 会阻塞直到一行可用或 stdout 关闭
            line = proc.stdout.readline()
            if not line:
                # 子进程已结束
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                logger.warning(
                    "VprBootstrap: 子进程输出非 JSON 行（忽略）: %s",
                    line.rstrip(),
                )
                continue
            if event.get("event") == "ready":
                self.ready_info = event
                return True
            logger.warning(
                "VprBootstrap: ready 之前收到意料外事件: %s", event,
            )
        return False

    def _send_request(self, payload: dict, timeout_sec: float) -> dict:
        proc = self._proc
        if proc is None or proc.stdin is None or proc.stdout is None:
            raise VprBootstrapError("VPR subprocess not running")
        if proc.poll() is not None:
            raise VprBootstrapError(
                f"VPR subprocess has exited (code={proc.returncode})"
            )
        req_id = payload.get("id") or uuid.uuid4().hex
        payload["id"] = req_id

        with self._lock:
            line = json.dumps(payload, ensure_ascii=False)
            try:
                proc.stdin.write(line + "\n")
                proc.stdin.flush()
            except Exception as e:
                raise VprBootstrapError(f"failed to write request: {e}") from e

            # 读取响应（响应严格一行一次；子进程是单线程顺序处理的）
            deadline = time.monotonic() + max(1.0, float(timeout_sec))
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    raise VprBootstrapError(
                        f"VPR subprocess exited mid-request (code={proc.returncode})"
                    )
                resp_line = proc.stdout.readline()
                if not resp_line:
                    continue
                try:
                    resp = json.loads(resp_line)
                except json.JSONDecodeError:
                    logger.warning(
                        "VprBootstrap: 忽略非 JSON 响应: %s", resp_line.rstrip()
                    )
                    continue
                if resp.get("id") != req_id:
                    logger.warning(
                        "VprBootstrap: 收到 id 不匹配的响应（已丢弃）: got=%s want=%s",
                        resp.get("id"), req_id,
                    )
                    continue
                return resp
            raise VprBootstrapError(
                f"VPR query timed out after {timeout_sec:.1f}s (id={req_id})"
            )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def locate(
        self,
        jpeg_bytes: bytes,
        top_k: int = 1,
        timeout_sec: float = QUERY_TIMEOUT_DEFAULT,
        rerank: bool = False,
        rerank_top_k: int = 20,
    ) -> Optional[dict]:
        """Match a raw camera JPEG against all submap databases.

        :param rerank: If True, after FAISS retrieval the detector subprocess
                       runs SuperPoint+LightGlue geometric re-ranking on the
                       top candidates and returns the best one after rerank.
                       Significantly improves robustness to perceptual
                       aliasing at the cost of latency (~0.5-2s per query
                       depending on rerank_top_k and device).
        :param rerank_top_k: Number of candidates to feed into the geometric
                             reranker. Only effective when rerank=True.
        :return: dict with keys ``submap_id``, ``distance``, ``x``, ``y``,
                 ``z``, ``database``, ``image_rel``, ``image_path``,
                 ``elapsed_ms`` (and ``rerank_inliers`` for top-1 if
                 rerank=True) -- or ``None`` if VPR is unavailable or no
                 result was returned. Raises ``VprBootstrapError`` on
                 transport-level failures.
        """
        if not self.available:
            return None
        if self._tmpdir is None:
            return None

        # 与离线建库一致的预处理：先按 R1Pro 内参去畸变，再中心裁剪到 1024x768。
        # 失败会回退为原始 jpeg_bytes，不会阻断查询。
        processed_bytes = _undistort_and_center_crop(jpeg_bytes)

        # 写到子进程可读的临时文件（避免 stdin 内塞二进制数据，协议是行 JSON）
        # 注意：暂不删除该临时文件，便于离线 debug；进程退出时 self._tmpdir 会整体清理。
        path = os.path.join(self._tmpdir.name, f"q_{uuid.uuid4().hex}.jpg")
        with open(path, "wb") as f:
            f.write(processed_bytes)
        logger.warning(f"write image to tmp file: {path}")
        req = {"image_path": path, "top_k": top_k}
        if rerank:
            req["rerank"] = True
            req["rerank_top_k"] = int(rerank_top_k)
        resp = self._send_request(req, timeout_sec=timeout_sec)

        if "error" in resp:
            raise VprBootstrapError(resp["error"])
        results = resp.get("results") or []
        if not results:
            return None
        best = results[0]
        submap_id = self._db_to_submap.get(
            os.path.abspath(best.get("database", ""))
        )
        if submap_id is None:
            logger.warning(
                "VprBootstrap: 匹配到未知 database '%s'", best.get("database"),
            )
            return None

        logger.info(
            "VprBootstrap.locate top-%d 结果 (faiss=%sms, rerank=%sms, total=%sms):",
            len(results),
            resp.get("faiss_ms"), resp.get("rerank_ms"), resp.get("elapsed_ms"),
        )
        for r in results:
            r_db = r.get("database", "")
            r_sm = self._db_to_submap.get(os.path.abspath(r_db)) or "?"
            r_inl = r.get("rerank_inliers")
            r_dist = r.get("distance")
            try:
                dist_str = f"{float(r_dist):.4f}"
            except (TypeError, ValueError):
                dist_str = str(r_dist)
            logger.info(
                "  [%d] submap=%s  dist=%s  inliers=%s  xyz=(%.2f,%.2f,%.2f)  %s",
                int(r.get("rank", -1)),
                r_sm, dist_str, r_inl,
                float(r.get("x") or 0.0), float(r.get("y") or 0.0), float(r.get("z") or 0.0),
                r.get("image_path"),
            )

        return {
            "submap_id": submap_id,
            "distance": best.get("distance"),
            "x": best.get("x"),
            "y": best.get("y"),
            "z": best.get("z"),
            "timestamp": best.get("timestamp"),
            "database": best.get("database"),
            "image_rel": best.get("image_rel"),
            "image_path": best.get("image_path"),
            "rerank_inliers": best.get("rerank_inliers"),
            "elapsed_ms": resp.get("elapsed_ms"),
            "faiss_ms": resp.get("faiss_ms"),
            "rerank_ms": resp.get("rerank_ms"),
            "top_k_results": results,
            # 已经做过 undistort + center-crop 的 query 临时图（与子进程实际比对
            # 的图一致），供上层做共识可视化等离线复核。
            "query_image_path": path,
        }


# ---------------------------------------------------------------------------
# High-level multi-frame consensus + map-switching orchestrator
# ---------------------------------------------------------------------------

class VprRelocator:
    """多帧 VPR 共识 + 触发换图的高层编排器。

    职责：
    - **共识采集**：以 ``FRAME_INTERVAL_SEC`` 为间隔取 ``CONSENSUS_FRAMES`` 帧，
      每帧 ``VprBootstrap.locate`` 后取 top-1 submap_id，全部一致才认为成功。
      最多重试 ``MAX_ROUNDS`` 轮。
    - **几何重排**：GPU 模式自动开启 SuperPoint+LightGlue 重排；CPU 模式自动
      关闭（因为单对图就要秒级，会让一轮共识超过 1 分钟）。
    - **调试可视化**：每轮结束（无论成败）把 N 帧 (query, match) 拼成大图写到
      ``DEBUG_DIR``，文件名包含时间戳/轮次/状态/submap。
    - **换图**：把识别到的 submap_id 交给上层 ``change_map_fn``（一般是
      ``server._change_map``，因为换图本身涉及 ROS 端协调，不属于 VPR 模块）。

    本类**不**关心 watchdog 的 cooldown / 失定位检测 —— 那是 server 端的职责。
    server 只需在合适的时机调用 ``pick_initial_submap()`` 或
    ``relocate_and_change_map(reason)``。
    """

    # 可调超参（class-level，便于子类化或直接覆盖实例属性）
    CONSENSUS_FRAMES = 5          # 每轮采集多少帧
    FRAME_INTERVAL_SEC = 1.0      # 帧间隔（秒）—— 也是"运动门控"的轮询周期
    MAX_ROUNDS = 3                # 最多重试多少轮（防止无限循环）
    RERANK_TOP_K = 20             # 几何重排候选数
    IMAGE_WAIT_SEC = 5.0          # 单帧等待相机的超时（watchdog 场景）
    STARTUP_IMAGE_WAIT_SEC = 10.0 # 启动场景的首帧等待（相机话题可能还没建好）
    DEBUG_DIR = "/tmp/vpr_debug"  # 共识可视化保存目录

    # --- 帧间运动门控（避免连续 5 帧拍同一面墙，等于浪费 votes） ---
    # 仅在 ros_node 提供 ``get_latest_odom_pose()`` 时启用；否则退化为纯
    # ``time.sleep(FRAME_INTERVAL_SEC)``，行为与改造前一致。
    MIN_MOVE_M = 0.10            # 相邻两次成功采样之间，至少位移这么多米
    MIN_ROT_DEG = 20.0           # 或者至少旋转这么多度（满足其一即可采）
    MOTION_WAIT_TIMEOUT_SEC = 30.0   # 等运动满足的最大时长；超过后强采一帧，
                                     # 避免机器人静止不动导致共识永远无法收敛
    FRAME_DONE_AUDIO = "search.mp3"  # 每张成功检测后由 ros_node.play_audio() 播放

    # --- 软共识降级方案（严格共识失败后启用） ---
    # 触发条件：5 帧 top-1 中至少有 SOFT_CONSENSUS_MIN_VOTES 帧投同一个 submap X，
    #          且其余帧 top-SOFT_CONSENSUS_TOP_K 中也存在 submap X 的匹配；
    # 验证条件：把这 5 帧（每帧选 submap X 对应的候选位姿）的 VPR 位姿 V_i 与
    #          采集时的 odom 位姿 O_i 做 2D 刚体对齐（Kabsch / Procrustes，
    #          这等价于 HMM 里"状态=位姿、转移=odom 增量"的硬判 Viterbi），
    #          残差小于阈值才认为轨迹一致、可降级判共识成功。
    SOFT_CONSENSUS_MIN_VOTES = 3       # 至少有几帧 top-1 一致
    SOFT_CONSENSUS_TOP_K = 10           # 非一致帧在 top-K 内检索匹配 submap
    TRAJ_FIT_MEAN_RESIDUAL_M = 3.0     # Kabsch 拟合后平均残差阈值（米）
    TRAJ_FIT_MAX_RESIDUAL_M = 5.0      # Kabsch 拟合后单点残差阈值（米）
    TRAJ_STATIC_ODOM_M = 0.10          # odom 位移 < 此值视为机器人基本静止
    TRAJ_STATIC_VPR_M = 1.0            # odom 静止时，VPR 位移也应 < 此值

    def __init__(
        self,
        vpr_bootstrap: VprBootstrap,
        ros_node: Any,
        map_manager: Any,
        change_map_fn: Callable[[str], Dict[str, Any]],
    ):
        """
        :param vpr_bootstrap: 已 ``start()`` 的 VprBootstrap；本类不负责其生命周期。
        :param ros_node: 提供 ``get_latest_image_bytes(timeout_sec=...)`` 与
                         ``camera_image_topic`` 属性（用于日志）。
        :param map_manager: 可选，仅用于日志里打印 "当前 submap"（``get_current_submap``）。
                            传 None 时相关日志会显示 "n/a"。
        :param change_map_fn: 取到 submap_id 后用于真正切换地图的回调。期望
                              返回形如 ``{"success": bool, "msg": str, ...}`` 的字典。
        """
        self.vpr_bootstrap = vpr_bootstrap
        self.ros_node = ros_node
        self.map_manager = map_manager
        self.change_map_fn = change_map_fn

    @property
    def available(self) -> bool:
        return (
            self.vpr_bootstrap is not None
            and self.vpr_bootstrap.available
        )

    # ------------------------------------------------------------------
    # Single-frame query
    # ------------------------------------------------------------------

    def _query_latest_image(
        self,
        image_wait_sec: float,
        top_k: int = 1,
        rerank: bool = False,
        rerank_top_k: int = 20,
    ) -> Optional[Dict[str, Any]]:
        """抓取最新一帧相机图像并调用 VPR；返回 ``locate`` 结果或 None。"""
        if not self.available:
            return None
        img = self.ros_node.get_latest_image_bytes(timeout_sec=image_wait_sec)
        if img is None:
            logger.warning(
                "VPR: 在 %.1fs 内未收到相机图像（topic=%s）",
                image_wait_sec, getattr(self.ros_node, "camera_image_topic", "?"),
            )
            return None
        jpeg_bytes, _stamp_ns = img
        try:
            return self.vpr_bootstrap.locate(
                jpeg_bytes,
                top_k=top_k,
                rerank=rerank,
                rerank_top_k=rerank_top_k,
            )
        except VprBootstrapError as e:
            logger.error(f"VPR 查询失败: {e}")
            return None
        except Exception as e:
            logger.error(f"VPR 查询异常: {e}", exc_info=True)
            return None

    # ------------------------------------------------------------------
    # Motion gating + audio cue helpers (used by collect_consensus)
    # ------------------------------------------------------------------

    @staticmethod
    def _pose_delta(
        ref: Tuple[float, float, float],
        cur: Tuple[float, float, float],
    ) -> Tuple[float, float]:
        """Return ``(distance_m, abs_yaw_deg)`` between two ``(x, y, yaw)`` poses."""
        import math
        dx = cur[0] - ref[0]
        dy = cur[1] - ref[1]
        dist = math.hypot(dx, dy)
        dyaw = math.atan2(
            math.sin(cur[2] - ref[2]), math.cos(cur[2] - ref[2])
        )
        return dist, abs(math.degrees(dyaw))

    def _get_odom_pose(self) -> Optional[Tuple[float, float, float]]:
        """Fetch the latest /odom pose as ``(x, y, yaw)``; None if unsupported."""
        fn = getattr(self.ros_node, "get_latest_odom_pose", None)
        if fn is None:
            return None
        try:
            p = fn()
        except Exception:
            return None
        if not p:
            return None
        # nav_node returns (x, y, yaw, stamp); strip stamp.
        return float(p[0]), float(p[1]), float(p[2])

    def _wait_for_motion(
        self,
        ref_pose: Optional[Tuple[float, float, float]],
        frame_idx: int,
    ) -> None:
        """Sleep one frame interval, then keep sleeping until the robot has moved.

        The motion gate kicks in only when:
        * ros_node exposes ``get_latest_odom_pose`` (otherwise no gate, just sleep)
        * AND ``ref_pose`` is not None (otherwise no reference to compare)
        Either way always sleeps at least one ``FRAME_INTERVAL_SEC`` so we don't
        burn CPU when /odom is missing.
        """
        time.sleep(self.FRAME_INTERVAL_SEC)
        if ref_pose is None:
            return

        deadline = time.monotonic() + self.MOTION_WAIT_TIMEOUT_SEC
        waited_extra = 0.0
        warned = False
        while True:
            cur = self._get_odom_pose()
            if cur is not None:
                dist, rot_deg = self._pose_delta(ref_pose, cur)
                if dist >= self.MIN_MOVE_M or rot_deg >= self.MIN_ROT_DEG:
                    if waited_extra > 0:
                        logger.info(
                            "[VPR] 第 %d 帧运动门控满足 (Δdist=%.2fm, Δrot=%.1f°)，"
                            "额外等待 %.1fs 后开始采样",
                            frame_idx + 1, dist, rot_deg, waited_extra,
                        )
                    return
                # Not enough motion yet -- log once per round, then keep polling.
                if not warned:
                    warned = True
                    logger.info(
                        "[VPR] 第 %d 帧运动不足 (Δdist=%.2fm < %.2fm 且 "
                        "Δrot=%.1f° < %.1f°)，等下一周期 (%.1fs) 再判定",
                        frame_idx + 1, dist, self.MIN_MOVE_M,
                        rot_deg, self.MIN_ROT_DEG, self.FRAME_INTERVAL_SEC,
                    )
            else:
                # /odom not arriving (e.g. chassis_ekf not up) -> degrade silently.
                logger.debug(
                    "[VPR] 运动门控：/odom 暂不可用，按固定周期推进"
                )
                return

            if time.monotonic() >= deadline:
                logger.warning(
                    "[VPR] 第 %d 帧运动门控超时 (>%.0fs)，强制采样以推进共识",
                    frame_idx + 1, self.MOTION_WAIT_TIMEOUT_SEC,
                )
                return
            time.sleep(self.FRAME_INTERVAL_SEC)
            waited_extra += self.FRAME_INTERVAL_SEC

    def _play_frame_done_audio(self) -> None:
        """Best-effort 'one frame captured' chime; never throws."""
        fn = getattr(self.ros_node, "play_audio", None)
        if fn is None:
            return
        try:
            fn(self.FRAME_DONE_AUDIO)
        except Exception as e:
            logger.debug(f"[VPR] play_audio({self.FRAME_DONE_AUDIO}) 失败: {e}")

    # ------------------------------------------------------------------
    # Multi-frame consensus
    # ------------------------------------------------------------------

    def collect_consensus(
        self,
        reason: str,
        *,
        image_wait_sec: Optional[float] = None,
    ) -> Optional[str]:
        """多帧共识识别 submap_id；返回投票一致的结果或 None。

        - 1s 间隔取 ``CONSENSUS_FRAMES`` 帧，每帧（GPU 模式下）开启几何重排
          后取 top-1 submap_id。
        - 仅当 N 帧 top-1 完全一致（且全部非空）才算成功。
        - 不一致则进入下一轮（最多 ``MAX_ROUNDS`` 轮）；超过则放弃。
        - 任一帧 locate 失败立即终止该轮，直接进入下一轮。
        - 每轮结束（无论成败）写一张调试大图到 ``DEBUG_DIR``。

        :param image_wait_sec: 每帧等待相机出图的超时。None 时使用
                                ``IMAGE_WAIT_SEC``；启动场景建议传
                                ``STARTUP_IMAGE_WAIT_SEC``。
        """
        if not self.available:
            return None
        eff_image_wait = (
            float(image_wait_sec) if image_wait_sec is not None
            else self.IMAGE_WAIT_SEC
        )

        # CPU 模式下 SuperPoint+LightGlue 慢到不可用（~1-3s/对 × 20 候选 × 5 帧）
        # 自动关闭 rerank，仅依赖 FAISS top-1 投票。GPU 模式才开 rerank。
        use_rerank = getattr(self.vpr_bootstrap, "device", "cpu") == "cuda"

        # 每帧请求 top-K 候选；K=1 时与旧行为一致，>=3 时可触发软共识降级。
        locate_top_k = max(1, int(self.SOFT_CONSENSUS_TOP_K))

        for round_idx in range(self.MAX_ROUNDS):
            logger.info(
                "[VPR] 共识第 %d/%d 轮开始 (%s)：采集 %d 帧 / 间隔 %.1fs / "
                "rerank=%s / rerank_top_k=%d / locate_top_k=%d",
                round_idx + 1, self.MAX_ROUNDS, reason,
                self.CONSENSUS_FRAMES, self.FRAME_INTERVAL_SEC,
                use_rerank, self.RERANK_TOP_K, locate_top_k,
            )
            votes: List[Optional[str]] = []
            inliers_log: List[Optional[int]] = []
            collage_items: List[Tuple[Optional[str], Optional[Dict[str, Any]]]] = []
            # 每帧的 top-K 完整候选（含 database/x/y/z），用于软共识查找
            top_k_per_frame: List[List[Dict[str, Any]]] = []
            # 每帧采集时的 odom pose (x, y, yaw)；None 表示 /odom 不可用
            odom_poses: List[Optional[Tuple[float, float, float]]] = []
            round_aborted = False
            # 上一次成功采样时的 /odom pose，下一帧据此做运动门控
            last_sample_pose: Optional[Tuple[float, float, float]] = None

            for frame_idx in range(self.CONSENSUS_FRAMES):
                if frame_idx > 0:
                    # 不再是裸 sleep —— 改成"睡一周期后，若位移/转角都没达到
                    # 阈值就接着等"。/odom 不可用或 ros_node 不支持该 API 时
                    # 自动降级为原来的固定周期等待。
                    self._wait_for_motion(last_sample_pose, frame_idx)
                t0 = time.monotonic()
                # 在 locate 前抓一次 /odom，作为"该帧采集时的机器人位姿"，
                # 供后续软共识里的 VPR / odom 轨迹一致性检查使用。
                sample_odom = self._get_odom_pose()
                result = self._query_latest_image(
                    image_wait_sec=eff_image_wait,
                    top_k=locate_top_k,
                    rerank=use_rerank,
                    rerank_top_k=self.RERANK_TOP_K,
                )
                dt_ms = (time.monotonic() - t0) * 1000.0
                if result is None:
                    logger.warning(
                        "[VPR] 第 %d/%d 帧 locate 失败（%.0fms），放弃本轮",
                        frame_idx + 1, self.CONSENSUS_FRAMES, dt_ms,
                    )
                    collage_items.append((None, None))
                    round_aborted = True
                    break

                submap_id = result.get("submap_id")
                inliers = result.get("rerank_inliers")
                votes.append(submap_id)
                inliers_log.append(inliers)
                collage_items.append((result.get("query_image_path"), result))
                top_k_per_frame.append(result.get("top_k_results") or [])
                odom_poses.append(sample_odom)
                logger.info(
                    "[VPR] 第 %d/%d 帧 → submap='%s' (rerank_inliers=%s, "
                    "dist=%.4f, %.0fms, odom=%s)",
                    frame_idx + 1, self.CONSENSUS_FRAMES,
                    submap_id, inliers,
                    float(result.get("distance") or 0.0), dt_ms,
                    "(%.2f,%.2f,%.1f°)" % (
                        sample_odom[0], sample_odom[1],
                        sample_odom[2] * 57.29578,
                    ) if sample_odom else "n/a",
                )

                # 每张成功检测后：播放 search.mp3 提示操作员可以挪/转一下，
                # 并把当前 odom pose 记下来给下一帧的运动门控做参考点。
                self._play_frame_done_audio()
                last_sample_pose = self._get_odom_pose()

            consensus_ok = (
                not round_aborted
                and len(votes) == self.CONSENSUS_FRAMES
                and len(set(votes)) == 1
                and None not in votes
            )

            # 严格共识失败时，尝试软共识降级方案（仅在本轮完整采到 N 帧才有意义）。
            soft_consensus_submap: Optional[str] = None
            if (
                not consensus_ok
                and not round_aborted
                and len(votes) == self.CONSENSUS_FRAMES
            ):
                soft_consensus_submap = self._try_soft_consensus(
                    votes=votes,
                    top_k_per_frame=top_k_per_frame,
                    odom_poses=odom_poses,
                )

            self._save_debug_image(
                round_idx=round_idx,
                reason=reason,
                items=collage_items,
                votes=votes,
                inliers_log=inliers_log,
                round_aborted=round_aborted,
                consensus_ok=consensus_ok,
                soft_consensus_submap=soft_consensus_submap,
            )

            if consensus_ok:
                chosen = votes[0]
                logger.info(
                    "[VPR] 共识达成: submap='%s' (votes=%s, inliers=%s)",
                    chosen, votes, inliers_log,
                )
                return chosen

            if soft_consensus_submap is not None:
                logger.info(
                    "[VPR] 软共识达成: submap='%s' (votes=%s, inliers=%s)，"
                    "通过 top-%d 回捞 + odom/VPR 轨迹一致性校验",
                    soft_consensus_submap, votes, inliers_log,
                    self.SOFT_CONSENSUS_TOP_K,
                )
                return soft_consensus_submap

            if round_aborted:
                continue

            logger.warning(
                "[VPR] 共识失败 (round %d/%d): votes=%s inliers=%s，进入下一轮",
                round_idx + 1, self.MAX_ROUNDS, votes, inliers_log,
            )

        logger.warning(
            "[VPR] %d 轮共识全部失败，放弃本次重定位 (%s)",
            self.MAX_ROUNDS, reason,
        )
        return None

    def _save_debug_image(
        self,
        *,
        round_idx: int,
        reason: str,
        items: List[Tuple[Optional[str], Optional[Dict[str, Any]]]],
        votes: List[Optional[str]],
        inliers_log: List[Optional[int]],
        round_aborted: bool,
        consensus_ok: bool,
        soft_consensus_submap: Optional[str] = None,
    ) -> None:
        """把一轮的 (query, match) 拼成大图写到 ``DEBUG_DIR``。失败只 warning。"""
        try:
            import datetime
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            if consensus_ok:
                status = "ok"
                chosen = votes[0] if votes else "-"
            elif soft_consensus_submap is not None:
                status = "softok"
                chosen = soft_consensus_submap
            elif round_aborted:
                status = "framefail"
                chosen = "-"
            else:
                status = "noconsensus"
                chosen = "-"
            safe_chosen = str(chosen).replace("/", "_").replace(" ", "_")
            fname = f"vpr_{ts}_round{round_idx + 1}_{status}_{safe_chosen}.jpg"
            save_path = os.path.join(self.DEBUG_DIR, fname)
            title = (
                f"{ts}  round {round_idx + 1}/{self.MAX_ROUNDS}  "
                f"{status}  votes={votes}  inliers={inliers_log}  ({reason})"
            )
            ok = save_consensus_collage(items, save_path, title=title)
            if ok:
                logger.info("[VPR] 共识可视化已保存: %s", save_path)
            else:
                logger.warning("[VPR] 共识可视化未写入: %s", save_path)
        except Exception as e:
            logger.warning(f"[VPR] 保存共识可视化异常: {e}")

    # ------------------------------------------------------------------
    # Soft consensus (relaxed fallback with HMM-style trajectory check)
    # ------------------------------------------------------------------

    def _result_to_submap_id(self, result: Dict[str, Any]) -> Optional[str]:
        """把单条 VPR 结果里的 ``database`` 反查回 ``submap_id``。"""
        db = result.get("database")
        if not db:
            return None
        return self.vpr_bootstrap._db_to_submap.get(os.path.abspath(db))

    def _try_soft_consensus(
        self,
        *,
        votes: List[Optional[str]],
        top_k_per_frame: List[List[Dict[str, Any]]],
        odom_poses: List[Optional[Tuple[float, float, float]]],
    ) -> Optional[str]:
        """严格共识失败后的降级方案。

        条件（与用户需求一致）：
        1. 至少 ``SOFT_CONSENSUS_MIN_VOTES`` 帧的 top-1 投同一个 ``submap X``；
        2. 其余帧的 top-``SOFT_CONSENSUS_TOP_K`` 中也存在 ``submap X`` 的匹配；
        3. 把这 5 帧 "对齐到 submap X 的候选位姿" 与采集时的 odom 位姿做 2D
           刚体（Kabsch / Procrustes）对齐，残差在阈值内 —— 这等价于一个
           硬判 Viterbi-HMM：状态是 submap 坐标系下的真实位姿，转移由 odom
           增量约束，观测是每帧 top-K 中匹配到 ``submap X`` 的候选位姿。

        三个条件都满足时返回 ``submap_id``，否则返回 ``None``。
        """
        if len(votes) != len(top_k_per_frame) or len(votes) != len(odom_poses):
            logger.debug(
                "[VPR] 软共识跳过：输入长度不一致 votes=%d top_k=%d odom=%d",
                len(votes), len(top_k_per_frame), len(odom_poses),
            )
            return None
        if len(votes) < self.SOFT_CONSENSUS_MIN_VOTES:
            return None

        from collections import Counter
        valid_votes = [v for v in votes if v is not None]
        if not valid_votes:
            return None
        counter = Counter(valid_votes)
        submap_id, top_count = counter.most_common(1)[0]
        if top_count < self.SOFT_CONSENSUS_MIN_VOTES:
            logger.info(
                "[VPR] 软共识不满足：top-1 票数最大才 %d (<%d)，候选 submap='%s'",
                top_count, self.SOFT_CONSENSUS_MIN_VOTES, submap_id,
            )
            return None

        # 为每帧挑出 "属于 submap X 的最靠前候选"；任一帧 top-K 内都没有就失败
        chosen: List[Dict[str, Any]] = []
        per_frame_origin: List[str] = []   # "top1" or "topN" 仅作日志
        for i, candidates in enumerate(top_k_per_frame):
            picked = None
            picked_rank = -1
            for rank, r in enumerate(candidates[: self.SOFT_CONSENSUS_TOP_K]):
                if self._result_to_submap_id(r) == submap_id:
                    picked = r
                    picked_rank = rank
                    break
            if picked is None:
                logger.info(
                    "[VPR] 软共识不满足：第 %d 帧 top-%d 中未出现 submap='%s' "
                    "(top-1 实际为 '%s')",
                    i + 1, self.SOFT_CONSENSUS_TOP_K, submap_id, votes[i],
                )
                return None
            chosen.append(picked)
            per_frame_origin.append("top1" if picked_rank == 0 else f"top{picked_rank + 1}")

        # odom 必须全部可用，否则没法做轨迹一致性检查
        if any(o is None for o in odom_poses):
            logger.info(
                "[VPR] 软共识不满足：odom 在部分帧缺失 (%s)，无法做轨迹一致性校验",
                [o is not None for o in odom_poses],
            )
            return None

        vpr_xy = [(float(c.get("x", 0.0)), float(c.get("y", 0.0))) for c in chosen]
        odom_xy = [(float(o[0]), float(o[1])) for o in odom_poses]  # type: ignore[index]

        logger.info(
            "[VPR] 软共识候选：submap='%s' (top-1 票数 %d/%d, 各帧来源=%s)，"
            "进入 VPR/odom 轨迹一致性校验",
            submap_id, top_count, len(votes), per_frame_origin,
        )
        if not self._trajectory_consistent(vpr_xy, odom_xy):
            return None
        return submap_id

    def _trajectory_consistent(
        self,
        vpr_xy: List[Tuple[float, float]],
        odom_xy: List[Tuple[float, float]],
    ) -> bool:
        """检查 VPR 位姿序列与 odom 位姿序列描述的是同一段轨迹。

        因为 odom 坐标系 / submap 坐标系之间差一个未知的 2D 刚体变换，所以
        不能直接逐点比 ``|V_i - O_i|``。这里用 Kabsch (2D Procrustes) 最小
        二乘地把 odom 旋转 / 平移到 submap，然后看每点残差是否都在阈值内：

            R*, t* = argmin Σ ||(R O_i + t) - V_i||^2
            ok ⇔ mean ||residual|| ≤ TRAJ_FIT_MEAN_RESIDUAL_M
                 且 max ||residual|| ≤ TRAJ_FIT_MAX_RESIDUAL_M

        机器人几乎静止时（odom 总位移 < TRAJ_STATIC_ODOM_M），Kabsch 拟合不可靠，
        改为退化判定：只要 VPR 位移也很小（< TRAJ_STATIC_VPR_M）即认为一致。
        """
        n = len(vpr_xy)
        if n != len(odom_xy) or n < 2:
            logger.warning(
                "[VPR] 软共识轨迹检查：点数不合法 (vpr=%d, odom=%d)", n, len(odom_xy),
            )
            return False

        try:
            import numpy as np  # 本地导入，避免在不需要时强依赖
        except Exception as e:
            logger.warning(
                "[VPR] 软共识轨迹检查：numpy 不可用 (%s)，无法做拟合，判失败", e,
            )
            return False

        V = np.asarray(vpr_xy, dtype=float)
        O = np.asarray(odom_xy, dtype=float)

        odom_extent = float(np.linalg.norm(O - O.mean(axis=0), axis=1).max())
        vpr_extent = float(np.linalg.norm(V - V.mean(axis=0), axis=1).max())

        if odom_extent < self.TRAJ_STATIC_ODOM_M:
            # 机器人基本没动：Kabsch 退化为奇异；如果 VPR 也基本静止就视作一致。
            ok = vpr_extent <= self.TRAJ_STATIC_VPR_M
            logger.info(
                "[VPR] 软共识轨迹检查（静止分支）：odom_extent=%.3fm < %.2fm, "
                "vpr_extent=%.3fm (阈值 ≤%.2f) → %s",
                odom_extent, self.TRAJ_STATIC_ODOM_M,
                vpr_extent, self.TRAJ_STATIC_VPR_M,
                "通过" if ok else "失败",
            )
            return ok

        # Kabsch / Procrustes：对齐 O → V 的 2D 刚体（rotation + translation）
        Vc = V - V.mean(axis=0)
        Oc = O - O.mean(axis=0)
        H = Oc.T @ Vc
        try:
            U, _, Vt = np.linalg.svd(H)
        except np.linalg.LinAlgError as e:
            logger.warning(
                "[VPR] 软共识轨迹检查：SVD 失败 (%s)，判失败", e,
            )
            return False
        d = float(np.sign(np.linalg.det(Vt.T @ U.T)))
        if d == 0.0:
            d = 1.0
        R = Vt.T @ np.diag([1.0, d]) @ U.T
        t = V.mean(axis=0) - R @ O.mean(axis=0)
        fit = (R @ O.T).T + t
        residuals = np.linalg.norm(V - fit, axis=1)
        mean_res = float(residuals.mean())
        max_res = float(residuals.max())

        ok = (
            mean_res <= self.TRAJ_FIT_MEAN_RESIDUAL_M
            and max_res <= self.TRAJ_FIT_MAX_RESIDUAL_M
        )
        logger.info(
            "[VPR] 软共识轨迹检查：n=%d, odom_extent=%.2fm, vpr_extent=%.2fm, "
            "residual mean=%.3fm / max=%.3fm "
            "(阈值 mean≤%.2f, max≤%.2f) → %s",
            n, odom_extent, vpr_extent,
            mean_res, max_res,
            self.TRAJ_FIT_MEAN_RESIDUAL_M, self.TRAJ_FIT_MAX_RESIDUAL_M,
            "通过" if ok else "失败",
        )
        return ok

    # ------------------------------------------------------------------
    # High-level entry points (server.py 直接调用这两个)
    # ------------------------------------------------------------------

    def _current_submap_str(self) -> str:
        if self.map_manager is None:
            return "n/a"
        try:
            return str(self.map_manager.get_current_submap())
        except Exception:
            return "?"

    def pick_initial_submap(self) -> bool:
        """启动时引导：多帧共识识别 submap，并调用 ``change_map_fn`` 切过去。

        :return: 共识达成且 change_map_fn 成功时为 True；否则 False（上层一般
                 不需要 raise，回退到 MapManager 缓存的 submap 即可）。
        """
        if not self.available:
            logger.warning("VprRelocator: VPR 不可用，跳过启动引导")
            return False
        logger.info(
            "启动时未指定 submap：通过 VPR 多帧共识 (%d 帧 × 间隔 %.1fs × 最多 %d 轮) 自动识别 ...",
            self.CONSENSUS_FRAMES, self.FRAME_INTERVAL_SEC, self.MAX_ROUNDS,
        )
        submap_id = self.collect_consensus(
            "startup bootstrap",
            image_wait_sec=self.STARTUP_IMAGE_WAIT_SEC,
        )
        if submap_id is None:
            logger.warning(
                "VPR 引导未能达成共识，将回退到 MapManager 当前 submap: %s",
                self._current_submap_str(),
            )
            return False
        logger.info("VPR 引导共识 submap='%s'，开始换图", submap_id)
        cm = self.change_map_fn(submap_id)
        if not cm.get("success"):
            logger.error("VPR 引导 change_map 失败: %s", cm.get("msg"))
            return False
        return True

    def relocate_and_change_map(self, reason: str) -> bool:
        """watchdog 调用：多帧共识 → change_map_fn。成功返回 True。"""
        if not self.available:
            logger.warning("VprRelocator: VPR 不可用，跳过重定位 (%s)", reason)
            return False
        logger.warning("[VPR] 触发重定位 (%s) ...", reason)
        target_submap = self.collect_consensus(reason)
        if target_submap is None:
            return False
        logger.info(
            "[VPR] 共识 submap='%s' (当前='%s')，开始换图",
            target_submap, self._current_submap_str(),
        )
        cm = self.change_map_fn(target_submap)
        if not cm.get("success"):
            logger.error("[VPR] change_map 失败: %s", cm.get("msg"))
            return False
        return True
