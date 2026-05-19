"""地图分发客户端：根据服务器 manifest 做增量同步。

同步流程：

1. ``GET {server_url}/mapserver/{map_id}`` 取服务器 manifest。
2. 读本地 ``<local_dir>/.mapserver_manifest.json``（缺失即视为首次同步）。
3. 比较 ``(version, build_no)``：完全一致 → 返回 UP_TO_DATE。
4. 否则按文件 sha256 比对，只下载缺失/变更的文件。
5. 下载流程：写入 ``<path>.partial`` → 校验 sha256 → 原子 ``os.replace`` 到位
   → 立即把已成功的条目 merge 进本地 manifest 并持久化。
   中途崩溃可断点续传（已下完的文件不会重传）。
6. 全部成功后，把本地 manifest 完全替换为 server manifest（自动清理掉
   服务器已删除的条目）。``--delete`` 可同时删除本地多余的实际文件。

库 API::

    from mapserver.client import sync_map
    result = sync_map("http://10.0.0.1:8080", "408", "/home/nvidia/Navi/maps/408")
    print(result)

CLI::

    python -m mapserver.client --server http://10.0.0.1:8080 --map 408 \\
        --local /home/nvidia/Navi/maps/408
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import logging
import os
import shutil
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from urllib import error as urlerror
from urllib import parse as urlparse
from urllib import request as urlrequest
from urllib.parse import urlparse as _urlparse

# 允许直接 ``python mapserver/client.py`` 跑
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from mapserver.manifest import (  # type: ignore
        FileEntry, MapManifest, diff_manifest,
        load_local_manifest, save_local_manifest,
    )
else:
    from .manifest import (
        FileEntry, MapManifest, diff_manifest,
        load_local_manifest, save_local_manifest,
    )

logger = logging.getLogger("mapserver.client")


# ---------------------------------------------------------------------------
# Result types
# ---------------------------------------------------------------------------

class SyncStatus:
    UP_TO_DATE = "up_to_date"   # version+build_no 完全一致，未做任何下载
    UPDATED = "updated"         # 已下载并应用更新
    FAILED = "failed"           # 至少一个文件下载/校验失败


@dataclass
class SyncResult:
    status: str
    server_version: str
    server_build_no: int
    local_version: Optional[str]
    local_build_no: Optional[int]
    downloaded: List[str] = field(default_factory=list)
    deleted: List[str] = field(default_factory=list)
    failed: List[str] = field(default_factory=list)
    bytes_downloaded: int = 0
    elapsed_sec: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "status": self.status,
            "server_version": self.server_version,
            "server_build_no": self.server_build_no,
            "local_version": self.local_version,
            "local_build_no": self.local_build_no,
            "downloaded": self.downloaded,
            "deleted": self.deleted,
            "failed": self.failed,
            "bytes_downloaded": self.bytes_downloaded,
            "elapsed_sec": round(self.elapsed_sec, 2),
        }


# ---------------------------------------------------------------------------
# HTTP helpers
# ---------------------------------------------------------------------------

def _http_get_json(url: str, timeout: float = 30.0) -> Dict[str, Any]:
    req = urlrequest.Request(url, headers={"Accept": "application/json"})
    with urlrequest.urlopen(req, timeout=timeout) as resp:
        if resp.status != 200:
            raise RuntimeError(f"GET {url} -> {resp.status}")
        return json.loads(resp.read().decode("utf-8"))


def _http_download(
    url: str,
    dest: Path,
    *,
    expected_sha256: str,
    chunk: int = 1 << 20,
    timeout: float = 120.0,
    resume: bool = True,
) -> int:
    """下载到 ``dest``。支持 Range 断点续传：

    - 若 ``<dest>.partial`` 存在 → 用 ``Range: bytes=N-`` 续传（server 必须支持 Range）。
    - 写完后校验 sha256，匹配再 ``os.replace`` 到 ``dest``。
    - 失败时保留 partial，下次可继续。
    - 返回实际写入字节数（含已存在部分）。
    """
    dest.parent.mkdir(parents=True, exist_ok=True)
    partial = dest.with_suffix(dest.suffix + ".partial")
    h = hashlib.sha256()
    mode = "wb"
    headers: Dict[str, str] = {}
    existing = 0
    if resume and partial.exists():
        existing = partial.stat().st_size
        if existing > 0:
            mode = "ab"
            headers["Range"] = f"bytes={existing}-"
            # 把已下载部分喂进 hasher
            with open(partial, "rb") as f:
                while True:
                    buf = f.read(chunk)
                    if not buf:
                        break
                    h.update(buf)

    req = urlrequest.Request(url, headers=headers)
    try:
        with urlrequest.urlopen(req, timeout=timeout) as resp:
            if resp.status not in (200, 206):
                raise RuntimeError(f"GET {url} -> {resp.status}")
            # 若我们请求了 Range 但服务器以 200 返回（不支持 Range），需要重置
            if "Range" in headers and resp.status == 200:
                logger.warning("server ignored Range for %s, restarting", url)
                partial.unlink(missing_ok=True)
                h = hashlib.sha256()
                mode = "wb"
                existing = 0
            with open(partial, mode) as f:
                while True:
                    buf = resp.read(chunk)
                    if not buf:
                        break
                    f.write(buf)
                    h.update(buf)
    except urlerror.URLError as e:
        raise RuntimeError(f"download {url} failed: {e}") from e

    got = h.hexdigest()
    if got != expected_sha256.lower():
        # 校验失败，丢弃 partial 以便完整重传
        with contextlib.suppress(FileNotFoundError):
            partial.unlink()
        raise RuntimeError(
            f"sha256 mismatch for {url}: expected={expected_sha256[:16]}... got={got[:16]}..."
        )

    os.replace(partial, dest)
    return partial.stat().st_size if partial.exists() else dest.stat().st_size


# ---------------------------------------------------------------------------
# Core sync logic
# ---------------------------------------------------------------------------

def _fetch_server_manifest(server_url: str, map_id: str, timeout: float) -> MapManifest:
    url = server_url.rstrip("/") + "/mapserver/" + urlparse.quote(map_id, safe="")
    data = _http_get_json(url, timeout=timeout)
    return MapManifest.from_dict(data)


def _merge_local_manifest(
    local_dir: Path,
    server: MapManifest,
    succeeded: Dict[str, FileEntry],
    base: Optional[MapManifest],
) -> MapManifest:
    """构造一份"截至目前"的本地 manifest 用于持久化（增量保存，崩溃可续）。"""
    base_files = dict(base.file_map()) if base is not None else {}
    server_paths = set(server.file_map().keys())
    # 删除 server 已经移除的条目
    for p in list(base_files):
        if p not in server_paths:
            base_files.pop(p, None)
    # 应用本次已成功下载的
    base_files.update(succeeded)
    # 注意：尚未下载的文件仍保留旧值（如果有），保证下次能正确续传
    files = sorted(base_files.values(), key=lambda f: f.path)
    return MapManifest(
        map_id=server.map_id,
        # 在所有文件都成功之前，不能宣称已切到 server 的 version/build_no，
        # 否则中途崩溃后 client 会误判 UP_TO_DATE。这里保留 base 或空串。
        version=base.version if base is not None else "",
        build_no=base.build_no if base is not None else 0,
        generated_at=base.generated_at if base is not None else "",
        files=files,
    )


def sync_map(
    server_url: str,
    map_id: str,
    local_dir: str | os.PathLike,
    *,
    delete_extras: bool = False,
    dry_run: bool = False,
    request_timeout: float = 30.0,
    download_timeout: float = 600.0,
) -> SyncResult:
    """从 ``server_url`` 同步 ``map_id`` 到 ``local_dir``，做增量更新。

    :param delete_extras: 若 server manifest 不含某本地文件，是否物理删除。
        默认 False（保守），仅在本地 manifest 中移除条目。
    :param dry_run: 仅打印差异，不下载也不修改。
    """
    local_dir_p = Path(local_dir)
    local_dir_p.mkdir(parents=True, exist_ok=True)

    t0 = time.time()
    server = _fetch_server_manifest(server_url, map_id, timeout=request_timeout)
    local = load_local_manifest(local_dir_p)
    diff = diff_manifest(server, local)

    result = SyncResult(
        status=SyncStatus.UP_TO_DATE,
        server_version=server.version,
        server_build_no=server.build_no,
        local_version=local.version if local else None,
        local_build_no=local.build_no if local else None,
    )

    # ---- 快路径：版本号一致 ----
    if diff.same_version and not diff.to_download:
        result.elapsed_sec = time.time() - t0
        logger.info(
            "[client] up to date map=%s version=%s build_no=%d",
            map_id, server.version, server.build_no,
        )
        return result

    if diff.same_version and diff.to_download:
        # 版本号一致但文件不一致：本地疑似被破坏，照样按 sha256 比对补齐
        logger.warning(
            "[client] version matches but %d files differ; repairing",
            len(diff.to_download),
        )

    logger.info(
        "[client] update needed map=%s local=(%s, %s) -> server=(%s, %d) "
        "to_download=%d (%.1f MiB) to_delete=%d",
        map_id,
        result.local_version, result.local_build_no,
        server.version, server.build_no,
        len(diff.to_download), diff.total_download_bytes / (1 << 20),
        len(diff.to_delete),
    )

    if dry_run:
        result.status = SyncStatus.UPDATED  # 表示"会做更新"
        result.downloaded = [f.path for f in diff.to_download]
        result.deleted = list(diff.to_delete)
        result.bytes_downloaded = diff.total_download_bytes
        result.elapsed_sec = time.time() - t0
        return result

    # ---- 真正下载 ----
    succeeded: Dict[str, FileEntry] = {}
    for fe in diff.to_download:
        url = (
            server_url.rstrip("/")
            + "/mapserver/" + urlparse.quote(map_id, safe="")
            + "/files/" + urlparse.quote(fe.path)
        )
        dest = local_dir_p / fe.path
        try:
            _http_download(
                url, dest,
                expected_sha256=fe.sha256,
                timeout=download_timeout,
            )
            succeeded[fe.path] = fe
            result.downloaded.append(fe.path)
            result.bytes_downloaded += fe.size
            logger.info("[client] downloaded %s (%.1f MiB)", fe.path, fe.size / (1 << 20))
            # 增量持久化 manifest，让崩溃后能续传
            try:
                interim = _merge_local_manifest(local_dir_p, server, succeeded, local)
                save_local_manifest(local_dir_p, interim)
            except OSError as e:
                logger.warning("[client] interim manifest save failed: %s", e)
        except Exception as e:  # noqa: BLE001
            logger.error("[client] failed to download %s: %s", fe.path, e)
            result.failed.append(fe.path)

    # ---- 清理多余文件 ----
    if delete_extras:
        for rel in diff.to_delete:
            fp = local_dir_p / rel
            try:
                if fp.is_file():
                    fp.unlink()
                result.deleted.append(rel)
                logger.info("[client] deleted extra %s", rel)
            except OSError as e:
                logger.warning("[client] failed to delete %s: %s", rel, e)

    # ---- 收尾 ----
    if result.failed:
        result.status = SyncStatus.FAILED
        logger.error("[client] sync FAILED with %d failures", len(result.failed))
    else:
        # 全部成功才把本地 manifest 完整切到 server 版本
        # （否则 local manifest 仍是混合状态，version 字段保持旧值）
        save_local_manifest(local_dir_p, server)
        result.status = SyncStatus.UPDATED
        logger.info(
            "[client] sync done map=%s -> %s/%d files=+%d/-%d bytes=%.1f MiB",
            map_id, server.version, server.build_no,
            len(result.downloaded), len(result.deleted),
            result.bytes_downloaded / (1 << 20),
        )

    result.elapsed_sec = time.time() - t0
    return result


# ---------------------------------------------------------------------------
# 高层入口：把 "本地路径 OR mapserver URL" 解析为本地 multifloor.json 路径
# ---------------------------------------------------------------------------

def _looks_like_url(s: str) -> bool:
    try:
        p = _urlparse(s)
    except (TypeError, ValueError):
        return False
    return p.scheme in ("http", "https") and bool(p.netloc)


def parse_mapserver_url(url: str) -> Tuple[str, str]:
    """``http://host:port/mapserver/<map_id>[/...]`` → ``(base_url, map_id)``。

    末尾允许带斜杠或额外路径（如 ``/manifest``），都会被截断。
    """
    p = _urlparse(url)
    if p.scheme not in ("http", "https") or not p.netloc:
        raise ValueError(f"不是合法的 http(s) URL: {url}")
    parts = [s for s in p.path.split("/") if s]
    if len(parts) < 2 or parts[0] != "mapserver":
        raise ValueError(
            f"URL 不符合 .../mapserver/<map_id> 形式: {url}"
        )
    base_url = f"{p.scheme}://{p.netloc}"
    map_id = parts[1]
    return base_url, map_id


def resolve_map_source(
    arg: str,
    *,
    cache_root: str | os.PathLike,
    network_required: bool = False,
    request_timeout: float = 30.0,
    download_timeout: float = 600.0,
) -> Path:
    """把 ``-m`` 参数解析为可直接喂给 ``MapManager`` 的本地 ``<map_id>.json`` 路径。

    ``arg`` 接受两种形式：

    1. **本地路径**（绝对/相对，且存在）→ 原样返回 ``Path(arg).resolve()``。
    2. **mapserver URL** ``http(s)://host:port/mapserver/<map_id>[/...]``：

       a. 解析出 ``base_url`` 与 ``map_id``
       b. 在 ``cache_root/<map_id>/`` 调 :func:`sync_map` 做增量同步：

          - 本地已有最新版（``status == up_to_date``）→ 零下载，仅返回路径
          - 缺文件或版本落后 → 仅拉取变更，写入本地后返回路径
          - 全新（本地无 ``<map_id>.json``）→ 全量下载后返回路径
       c. 同步失败时的降级策略：

          - 若本地 ``<map_id>.json`` 已存在 → 打 WARNING，复用本地缓存
          - 否则 → 抛 ``RuntimeError``

    :param cache_root: 同步目标根目录。最终地图落到
        ``cache_root/<map_id>/`` 下，与本地编辑共享同一目录约定。
    :param network_required: 默认 False，允许网络失败时降级到本地缓存；
        置 True 表示必须以服务器为准（失败即抛出）。
    """
    # ── 形态判定：先看是不是本地存在的路径 ─────────────────────────
    p = Path(arg)
    if p.exists():
        return p.resolve()

    if not _looks_like_url(arg):
        raise FileNotFoundError(
            f"地图参数既不是已存在的本地路径，也不是合法 URL: {arg}"
        )

    # ── URL 分支：同步 → 返回本地 json ─────────────────────────────
    base_url, map_id = parse_mapserver_url(arg)
    local_dir = Path(cache_root) / map_id
    json_path = local_dir / f"{map_id}.json"
    have_cache = json_path.is_file()

    logger.info(
        "[mapsource] resolving map: url=%s -> local=%s (cache=%s)",
        arg, local_dir, "hit" if have_cache else "miss",
    )

    try:
        result = sync_map(
            base_url, map_id, local_dir,
            request_timeout=request_timeout,
            download_timeout=download_timeout,
        )
    except Exception as e:  # 网络异常、URL 错、服务器 5xx 等
        if have_cache and not network_required:
            logger.warning(
                "[mapsource] sync failed (%s); falling back to local cache: %s",
                e, json_path,
            )
            return json_path.resolve()
        raise RuntimeError(
            f"无法从 {arg} 同步地图且本地无可用缓存: {e}"
        ) from e

    logger.info(
        "[mapsource] sync %s: server=(%s, %s) local=(%s, %s) "
        "downloaded=%d files (%.1f MiB) failed=%d elapsed=%.1fs",
        result.status,
        result.server_version, result.server_build_no,
        result.local_version, result.local_build_no,
        len(result.downloaded), result.bytes_downloaded / (1 << 20),
        len(result.failed), result.elapsed_sec,
    )

    if result.status == SyncStatus.FAILED:
        # 部分文件失败；若本地原本就有可用 json，可以尝试继续
        if json_path.is_file() and not network_required:
            logger.warning(
                "[mapsource] sync had %d failed files; using current local copy: %s",
                len(result.failed), json_path,
            )
            return json_path.resolve()
        raise RuntimeError(
            f"同步 {arg} 失败的文件: {result.failed}"
        )

    if not json_path.is_file():
        raise FileNotFoundError(
            f"同步完成但未在本地找到 {json_path}，请检查服务器上是否存在 "
            f"{map_id}/{map_id}.json"
        )
    return json_path.resolve()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Navi map distribution client (incremental sync)")
    p.add_argument("--server", required=True,
                   help="服务器基址，例如 http://10.0.0.1:8080")
    p.add_argument("--map", required=True, dest="map_id",
                   help="map_id（与 URL 中 /mapserver/<map_id> 对应）")
    p.add_argument("--local", required=True,
                   help="本地保存目录")
    p.add_argument("--delete-extras", action="store_true",
                   help="同时删除本地有、server 没有的文件")
    p.add_argument("--dry-run", action="store_true",
                   help="仅打印将要执行的操作")
    p.add_argument("--log-level", default="info",
                   choices=["debug", "info", "warning", "error"])
    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )
    try:
        result = sync_map(
            args.server, args.map_id, args.local,
            delete_extras=args.delete_extras,
            dry_run=args.dry_run,
        )
    except Exception as e:  # noqa: BLE001
        logger.error("sync_map failed: %s", e)
        return 2
    print(json.dumps(result.to_dict(), indent=2, ensure_ascii=False))
    return 0 if result.status != SyncStatus.FAILED else 1


if __name__ == "__main__":
    raise SystemExit(main())
