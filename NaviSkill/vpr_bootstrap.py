"""VPR-based submap bootstrap / re-localization helper for ``server.py``.

This wraps ``vpr/vpr-pipeline-v1/vpr_detect.py`` running in **service mode**
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
from typing import Dict, List, Optional

logger = logging.getLogger(__name__)


# Where vpr_detect.py and its dedicated venv live. Kept here so server.py
# doesn't need to know about layout details.
_DEFAULT_VPR_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "vpr", "vpr-pipeline-v1")
)


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
        Override the vpr-pipeline-v1 directory (defaults to ``../vpr/vpr-pipeline-v1``
        relative to this file).
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
                if line:
                    logger.info("[vpr_detect] %s", line)
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
    ) -> Optional[dict]:
        """Match a raw camera JPEG against all submap databases.

        :return: dict with keys ``submap_id``, ``distance``, ``x``, ``y``,
                 ``z``, ``database``, ``image_rel``, ``image_path``,
                 ``elapsed_ms`` -- or ``None`` if VPR is unavailable or no
                 result was returned. Raises ``VprBootstrapError`` on
                 transport-level failures.
        """
        if not self.available:
            return None
        if self._tmpdir is None:
            return None

        # 写到子进程可读的临时文件（避免 stdin 内塞二进制数据，协议是行 JSON）
        path = os.path.join(self._tmpdir.name, f"q_{uuid.uuid4().hex}.jpg")
        try:
            with open(path, "wb") as f:
                f.write(jpeg_bytes)
            resp = self._send_request(
                {"image_path": path, "top_k": top_k},
                timeout_sec=timeout_sec,
            )
        finally:
            try:
                os.unlink(path)
            except OSError:
                pass

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
            "elapsed_ms": resp.get("elapsed_ms"),
            "top_k_results": results,
        }
