"""地图分发服务器（FastAPI）。

接口：

  GET /mapserver/{map_id}                返回 manifest JSON
  GET /mapserver/{map_id}/manifest       同上（显式路径）
  GET /mapserver/{map_id}/files/{path}   下载单文件，支持 HTTP Range
  GET /mapserver/                        列出所有可用 map_id
  GET /healthz                           健康检查

启动::

  python -m mapserver.server --maps-root /home/nvidia/Navi/maps --host 0.0.0.0 --port 8080

或直接::

  python mapserver/server.py --maps-root /home/nvidia/Navi/maps --port 8080
"""

from __future__ import annotations

import argparse
import logging
import os
import re
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Tuple

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse, Response, StreamingResponse

# 允许 ``python mapserver/server.py`` 直接运行
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from mapserver.manifest import MapManifest, build_manifest  # type: ignore
else:
    from .manifest import MapManifest, build_manifest

logger = logging.getLogger("mapserver")

# 默认每个 chunk 1MB，平衡内存与 syscall
_DEFAULT_CHUNK = 1 << 20

_RANGE_RE = re.compile(r"^bytes=(\d*)-(\d*)$")


# ---------------------------------------------------------------------------
# Manifest cache：避免每次请求都重算 568MB 的 sha256
# ---------------------------------------------------------------------------

class _ManifestCache:
    """基于 (path, size, mtime) 指纹的内存缓存。

    指纹未变 → 直接复用上次的 manifest；变化（即使只新增/修改一个文件）→ 重建。
    重建大概几秒（取决于磁盘带宽），可接受。
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cache: Dict[str, Tuple[Tuple, MapManifest]] = {}

    def get(self, map_dir: Path, map_id: str) -> MapManifest:
        signature = _scan_signature(map_dir)
        with self._lock:
            cached = self._cache.get(map_id)
            if cached is not None and cached[0] == signature:
                return cached[1]
        # 重建（无锁）。多个并发请求可能同时算，但结果相同，最后写回即可。
        t0 = time.time()
        manifest = build_manifest(map_dir, map_id=map_id)
        logger.info(
            "[mapserver] rebuilt manifest map=%s files=%d size=%dB elapsed=%.2fs",
            map_id, len(manifest.files), manifest.total_bytes(), time.time() - t0,
        )
        with self._lock:
            self._cache[map_id] = (signature, manifest)
        return manifest

    def invalidate(self, map_id: Optional[str] = None) -> None:
        with self._lock:
            if map_id is None:
                self._cache.clear()
            else:
                self._cache.pop(map_id, None)


def _scan_signature(map_dir: Path) -> Tuple:
    """快速指纹：所有文件的 (relpath, size, mtime_ns) 排序后的元组。"""
    items: List[Tuple[str, int, int]] = []
    for root, _dirs, files in os.walk(map_dir):
        for fname in files:
            fp = Path(root) / fname
            try:
                st = fp.stat()
            except FileNotFoundError:
                continue
            rel = fp.relative_to(map_dir).as_posix()
            items.append((rel, st.st_size, st.st_mtime_ns))
    items.sort()
    return tuple(items)


# ---------------------------------------------------------------------------
# Range helpers
# ---------------------------------------------------------------------------

def _parse_range(header: str, file_size: int) -> Optional[Tuple[int, int]]:
    """解析 ``Range: bytes=START-END``。返回 ``(start, end)`` inclusive，或 None 表示无法解析。

    仅支持单 range。语法：

    - ``bytes=0-499``          → (0, 499)
    - ``bytes=500-``           → (500, file_size-1)
    - ``bytes=-500``           → 最后 500 字节 → (file_size-500, file_size-1)
    """
    m = _RANGE_RE.match(header.strip())
    if not m:
        return None
    start_s, end_s = m.group(1), m.group(2)
    if not start_s and not end_s:
        return None
    if not start_s:
        # 后缀长度
        length = int(end_s)
        if length <= 0:
            return None
        start = max(0, file_size - length)
        end = file_size - 1
    else:
        start = int(start_s)
        end = int(end_s) if end_s else file_size - 1
    if start > end or start >= file_size:
        return None
    end = min(end, file_size - 1)
    return start, end


def _iter_file_chunk(
    path: Path, start: int, end: int, chunk: int = _DEFAULT_CHUNK,
) -> Iterator[bytes]:
    remaining = end - start + 1
    with open(path, "rb") as f:
        f.seek(start)
        while remaining > 0:
            buf = f.read(min(chunk, remaining))
            if not buf:
                break
            remaining -= len(buf)
            yield buf


# ---------------------------------------------------------------------------
# App factory
# ---------------------------------------------------------------------------

def create_app(maps_root: Path) -> FastAPI:
    maps_root = Path(maps_root).resolve()
    if not maps_root.is_dir():
        raise NotADirectoryError(f"maps_root not a directory: {maps_root}")

    app = FastAPI(title="Navi Map Server", version="1.0")
    cache = _ManifestCache()

    def _resolve_map_dir(map_id: str) -> Path:
        # 防 path traversal：不允许 .. 或绝对路径
        if "/" in map_id or "\\" in map_id or map_id in ("", ".", ".."):
            raise HTTPException(status_code=400, detail="invalid map_id")
        d = (maps_root / map_id).resolve()
        if not str(d).startswith(str(maps_root) + os.sep) and d != maps_root:
            raise HTTPException(status_code=400, detail="invalid map_id")
        if not d.is_dir():
            raise HTTPException(status_code=404, detail=f"map '{map_id}' not found")
        return d

    @app.get("/healthz")
    def healthz() -> Dict[str, str]:
        return {"status": "ok", "maps_root": str(maps_root)}

    @app.get("/mapserver/")
    def list_maps() -> Dict[str, List[str]]:
        items = sorted(
            d.name for d in maps_root.iterdir()
            if d.is_dir() and not d.name.startswith(".")
        )
        return {"maps": items}

    def _manifest_response(map_id: str) -> JSONResponse:
        d = _resolve_map_dir(map_id)
        manifest = cache.get(d, map_id)
        return JSONResponse(manifest.to_dict())

    @app.get("/mapserver/{map_id}")
    def get_manifest_root(map_id: str) -> JSONResponse:
        return _manifest_response(map_id)

    @app.get("/mapserver/{map_id}/manifest")
    def get_manifest(map_id: str) -> JSONResponse:
        return _manifest_response(map_id)

    @app.post("/mapserver/{map_id}/invalidate")
    def invalidate_cache(map_id: str) -> Dict[str, str]:
        """运维接口：手动让缓存失效（比如 git pull 之后）。"""
        cache.invalidate(map_id)
        return {"status": "ok", "map_id": map_id}

    @app.get("/mapserver/{map_id}/files/{file_path:path}")
    def get_file(map_id: str, file_path: str, request: Request) -> Response:
        d = _resolve_map_dir(map_id)
        # 路径遍历防护
        target = (d / file_path).resolve()
        if not str(target).startswith(str(d) + os.sep):
            raise HTTPException(status_code=400, detail="path traversal")
        if not target.is_file():
            raise HTTPException(status_code=404, detail="file not found")

        st = target.stat()
        file_size = st.st_size
        last_modified = time.strftime(
            "%a, %d %b %Y %H:%M:%S GMT", time.gmtime(st.st_mtime),
        )
        common_headers = {
            "Accept-Ranges": "bytes",
            "Last-Modified": last_modified,
        }

        range_header = request.headers.get("range")
        if range_header:
            parsed = _parse_range(range_header, file_size)
            if parsed is None:
                return Response(
                    status_code=416,
                    headers={"Content-Range": f"bytes */{file_size}"},
                )
            start, end = parsed
            length = end - start + 1
            headers = {
                **common_headers,
                "Content-Range": f"bytes {start}-{end}/{file_size}",
                "Content-Length": str(length),
            }
            return StreamingResponse(
                _iter_file_chunk(target, start, end),
                status_code=206,
                media_type="application/octet-stream",
                headers=headers,
            )

        # 全量
        headers = {
            **common_headers,
            "Content-Length": str(file_size),
        }
        return StreamingResponse(
            _iter_file_chunk(target, 0, file_size - 1),
            status_code=200,
            media_type="application/octet-stream",
            headers=headers,
        )

    return app


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Navi map distribution server")
    p.add_argument("--maps-root", required=True,
                   help="所有 map 目录的父目录，例如 /home/nvidia/Navi/maps")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--log-level", default="info",
                   choices=["critical", "error", "warning", "info", "debug", "trace"])
    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )
    app = create_app(Path(args.maps_root))
    logger.info(
        "[mapserver] starting host=%s port=%d maps_root=%s",
        args.host, args.port, args.maps_root,
    )
    import uvicorn
    uvicorn.run(app, host=args.host, port=args.port, log_level=args.log_level)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
