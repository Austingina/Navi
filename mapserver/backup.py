"""地图数据全量备份 / 恢复工具。

围绕两个目录约定工作：

- **front（线上目录）**：``~/mapserver/front/<map_id>``，MapEditor / 导航栈实际读写的
  当前地图数据。
- **backup（备份目录）**：``~/mapserver/backup``（默认），按
  ``~/mapserver/backup/<map_id>/<build_no>`` 分层存放历史快照。

``build_no`` 取当前地图 ``<map_id>.json`` 中 ``global.build_no``
（14 位时间戳 ``YYYYMMDDHHMMSS``，如 ``20260601151834``），既单调递增又人类可读，
天然适合做"同一张地图的多个历史版本"的目录名。

典型用法::

    # 备份：把 front/408 整个快照到 backup/408/<build_no>/
    python -m mapserver.backup backup --map 408

    # 列出 408 的所有历史备份（按 build_no 倒序）
    python -m mapserver.backup list --map 408

    # 恢复：交互式从备份列表里选一个，覆盖回 front/408
    python -m mapserver.backup restore --map 408
    # 或直接指定 build_no
    python -m mapserver.backup restore --map 408 --build-no 20260601151834

库 API::

    from mapserver.backup import backup_map, restore_map, list_backups
    info = backup_map("408")
    backups = list_backups("408")
    restore_map("408", backups[0].build_no)
"""

from __future__ import annotations

import argparse
import logging
import os
import shutil
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

# 复用 manifest 里已有的版本号解析逻辑（同包内部调用）。
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from mapserver.manifest import _resolve_version_info  # type: ignore
else:
    from .manifest import _resolve_version_info

logger = logging.getLogger("mapserver.backup")

# 默认目录约定（位于用户 home 下的 mapserver）。
DEFAULT_FRONT_ROOT = Path.home() / "mapserver" / "front"
DEFAULT_BACKUP_ROOT = Path.home() / "mapserver" / "backup"

# 拷贝时跳过的临时 / 缓存产物（全量备份仍包含 .mapserver_manifest.json 等状态文件）。
_COPY_IGNORE = shutil.ignore_patterns("__pycache__", "*.partial", ".staging")


# ---------------------------------------------------------------------------
# Result types
# ---------------------------------------------------------------------------

@dataclass
class BackupInfo:
    """一个历史备份快照的元信息。"""
    map_id: str
    build_no: str
    path: Path
    size_bytes: int
    mtime: float

    @property
    def mtime_str(self) -> str:
        return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.mtime))


@dataclass
class BackupResult:
    map_id: str
    build_no: str
    src: Path
    dest: Path
    size_bytes: int
    skipped: bool          # 该 build_no 已存在且未 --force，跳过未拷贝
    elapsed_sec: float


@dataclass
class RestoreResult:
    map_id: str
    build_no: str
    src: Path
    dest: Path
    size_bytes: int
    elapsed_sec: float


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _dir_size(path: Path) -> int:
    total = 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            fp = Path(root) / f
            try:
                total += fp.stat().st_size
            except OSError:
                continue
    return total


def _resolve_build_no(map_dir: Path, map_id: str) -> str:
    """确定 front 目录当前应使用的 build_no。

    优先级：``<map_id>.json`` 的 ``global.build_no`` → ``version.json`` →
    都没有时回退到当前本地时间 ``YYYYMMDDHHMMSS``（保证总能产出一个目录名）。
    """
    info = _resolve_version_info(map_dir, map_id)
    if info is not None:
        _version, build_no = info
        if build_no:
            return str(build_no)
    fallback = time.strftime("%Y%m%d%H%M%S", time.localtime())
    logger.warning(
        "[backup] map=%s 未能从 %s.json/version.json 读到 build_no，回退到当前时间 %s",
        map_id, map_id, fallback,
    )
    return fallback


def _validate_map_id(map_id: str) -> None:
    """防 path traversal：map_id 不允许包含分隔符或相对路径片段。"""
    if not map_id or "/" in map_id or "\\" in map_id or map_id in (".", ".."):
        raise ValueError(f"非法 map_id: {map_id!r}")


# ---------------------------------------------------------------------------
# Backup
# ---------------------------------------------------------------------------

def backup_map(
    map_id: str,
    *,
    front_root: os.PathLike | str = DEFAULT_FRONT_ROOT,
    backup_root: os.PathLike | str = DEFAULT_BACKUP_ROOT,
    build_no: Optional[str] = None,
    force: bool = False,
) -> BackupResult:
    """把 ``front_root/<map_id>`` 整个目录快照到 ``backup_root/<map_id>/<build_no>``。

    :param build_no: 缺省时从 ``<map_id>.json`` 的 ``global.build_no`` 读取。
    :param force: 目标 ``<build_no>`` 目录已存在时，True 覆盖，False 跳过（默认）。
    :raises ValueError: ``map_id`` 非法。
    :raises FileNotFoundError: front 目录不存在。
    """
    _validate_map_id(map_id)
    t0 = time.time()

    src = Path(front_root) / map_id
    if not src.is_dir():
        raise FileNotFoundError(f"front 目录不存在: {src}")

    bn = str(build_no) if build_no else _resolve_build_no(src, map_id)
    dest = Path(backup_root) / map_id / bn

    if dest.exists():
        if not force:
            logger.info(
                "[backup] map=%s build_no=%s 已存在备份，跳过: %s",
                map_id, bn, dest,
            )
            return BackupResult(
                map_id=map_id, build_no=bn, src=src, dest=dest,
                size_bytes=_dir_size(dest), skipped=True,
                elapsed_sec=time.time() - t0,
            )
        # 覆盖：先拷到临时目录再原子替换，避免中途失败留下半个备份。
        logger.warning("[backup] map=%s build_no=%s 已存在，--force 覆盖", map_id, bn)

    dest.parent.mkdir(parents=True, exist_ok=True)
    staging = dest.parent / f".{bn}.staging-{os.getpid()}"
    if staging.exists():
        shutil.rmtree(staging)

    shutil.copytree(src, staging, ignore=_COPY_IGNORE, symlinks=False)

    # 原子替换：同一父目录下的 rename。
    if dest.exists():
        trash = dest.parent / f".{bn}.trash-{os.getpid()}"
        os.replace(dest, trash)
        os.replace(staging, dest)
        shutil.rmtree(trash, ignore_errors=True)
    else:
        os.replace(staging, dest)

    size = _dir_size(dest)
    elapsed = time.time() - t0
    logger.info(
        "[backup] done map=%s build_no=%s size=%.1f MiB -> %s (%.1fs)",
        map_id, bn, size / (1 << 20), dest, elapsed,
    )
    return BackupResult(
        map_id=map_id, build_no=bn, src=src, dest=dest,
        size_bytes=size, skipped=False, elapsed_sec=elapsed,
    )


# ---------------------------------------------------------------------------
# List
# ---------------------------------------------------------------------------

def list_backups(
    map_id: str,
    *,
    backup_root: os.PathLike | str = DEFAULT_BACKUP_ROOT,
) -> List[BackupInfo]:
    """列出 ``backup_root/<map_id>/`` 下的所有备份，按 build_no 倒序（新→旧）。"""
    _validate_map_id(map_id)
    base = Path(backup_root) / map_id
    if not base.is_dir():
        return []
    out: List[BackupInfo] = []
    for d in base.iterdir():
        # 跳过临时 staging / trash 目录
        if not d.is_dir() or d.name.startswith("."):
            continue
        st = d.stat()
        out.append(BackupInfo(
            map_id=map_id,
            build_no=d.name,
            path=d,
            size_bytes=_dir_size(d),
            mtime=st.st_mtime,
        ))
    # build_no 多为数字时间戳；数字优先按整数排，否则退化为字符串排序。
    def _key(b: BackupInfo):
        return (0, int(b.build_no)) if b.build_no.isdigit() else (1, b.build_no)
    out.sort(key=_key, reverse=True)
    return out


# ---------------------------------------------------------------------------
# Restore
# ---------------------------------------------------------------------------

def restore_map(
    map_id: str,
    build_no: str,
    *,
    front_root: os.PathLike | str = DEFAULT_FRONT_ROOT,
    backup_root: os.PathLike | str = DEFAULT_BACKUP_ROOT,
) -> RestoreResult:
    """把 ``backup_root/<map_id>/<build_no>`` 恢复（覆盖）到 ``front_root/<map_id>``。

    采用"先拷到临时目录 → 原子替换"的方式，恢复过程中崩溃也不会破坏现有 front。
    被覆盖的旧 front 会先挪到临时 trash，替换成功后才删除。

    :raises ValueError: ``map_id`` 非法。
    :raises FileNotFoundError: 指定的备份不存在。
    """
    _validate_map_id(map_id)
    t0 = time.time()

    src = Path(backup_root) / map_id / str(build_no)
    if not src.is_dir():
        raise FileNotFoundError(f"备份不存在: {src}")

    front_root_p = Path(front_root)
    dest = front_root_p / map_id
    (front_root_p / map_id).parent.mkdir(parents=True, exist_ok=True)

    staging = dest.parent / f".{map_id}.staging-{os.getpid()}"
    if staging.exists():
        shutil.rmtree(staging)

    shutil.copytree(src, staging, ignore=_COPY_IGNORE, symlinks=False)

    if dest.exists():
        trash = dest.parent / f".{map_id}.trash-{os.getpid()}"
        if trash.exists():
            shutil.rmtree(trash)
        os.replace(dest, trash)
        os.replace(staging, dest)
        shutil.rmtree(trash, ignore_errors=True)
    else:
        os.replace(staging, dest)

    size = _dir_size(dest)
    elapsed = time.time() - t0
    logger.info(
        "[restore] done map=%s build_no=%s size=%.1f MiB -> %s (%.1fs)",
        map_id, build_no, size / (1 << 20), dest, elapsed,
    )
    return RestoreResult(
        map_id=map_id, build_no=str(build_no), src=src, dest=dest,
        size_bytes=size, elapsed_sec=elapsed,
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _fmt_size(n: int) -> str:
    mib = n / (1 << 20)
    if mib >= 1024:
        return f"{mib / 1024:.2f} GiB"
    return f"{mib:.1f} MiB"


def _interactive_pick(backups: List[BackupInfo]) -> Optional[BackupInfo]:
    """从备份列表里交互式选一个；返回 None 表示用户取消。"""
    print(f"\n找到 {len(backups)} 个备份（新 → 旧）：\n")
    for i, b in enumerate(backups, 1):
        print(f"  [{i}] build_no={b.build_no}  {b.mtime_str}  {_fmt_size(b.size_bytes)}")
    print()
    try:
        raw = input("请选择要恢复的序号（回车取消）: ").strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return None
    if not raw:
        return None
    try:
        idx = int(raw)
    except ValueError:
        print(f"无效输入: {raw!r}")
        return None
    if not (1 <= idx <= len(backups)):
        print(f"序号超出范围: {idx}")
        return None
    return backups[idx - 1]


def _cmd_backup(args: argparse.Namespace) -> int:
    res = backup_map(
        args.map_id,
        front_root=args.front_root,
        backup_root=args.backup_root,
        build_no=args.build_no,
        force=args.force,
    )
    if res.skipped:
        print(f"已存在备份，跳过: {res.dest}（使用 --force 覆盖）")
    else:
        print(f"备份完成: {res.dest}  ({_fmt_size(res.size_bytes)}, {res.elapsed_sec:.1f}s)")
    return 0


def _cmd_list(args: argparse.Namespace) -> int:
    backups = list_backups(args.map_id, backup_root=args.backup_root)
    if not backups:
        print(f"map={args.map_id} 暂无备份（{Path(args.backup_root) / args.map_id}）")
        return 0
    print(f"map={args.map_id} 共 {len(backups)} 个备份（新 → 旧）：")
    for b in backups:
        print(f"  build_no={b.build_no}  {b.mtime_str}  {_fmt_size(b.size_bytes)}  {b.path}")
    return 0


def _cmd_restore(args: argparse.Namespace) -> int:
    build_no = args.build_no
    if not build_no:
        backups = list_backups(args.map_id, backup_root=args.backup_root)
        if not backups:
            print(f"map={args.map_id} 暂无备份，无法恢复")
            return 1
        picked = _interactive_pick(backups)
        if picked is None:
            print("已取消")
            return 1
        build_no = picked.build_no

    res = restore_map(
        args.map_id, build_no,
        front_root=args.front_root,
        backup_root=args.backup_root,
    )
    print(f"恢复完成: {res.src} -> {res.dest}  ({_fmt_size(res.size_bytes)}, {res.elapsed_sec:.1f}s)")
    return 0


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="mapserver.backup",
        description="地图数据全量备份 / 恢复（front <-> backup）",
    )
    p.add_argument("--front-root", default=str(DEFAULT_FRONT_ROOT),
                   help=f"线上地图根目录（默认 {DEFAULT_FRONT_ROOT}）")
    p.add_argument("--backup-root", default=str(DEFAULT_BACKUP_ROOT),
                   help=f"备份根目录（默认 {DEFAULT_BACKUP_ROOT}）")
    p.add_argument("--log-level", default="info",
                   choices=["debug", "info", "warning", "error"])

    sub = p.add_subparsers(dest="command", required=True)

    pb = sub.add_parser("backup", help="备份 front/<map_id> 到 backup/<map_id>/<build_no>")
    pb.add_argument("--map", required=True, dest="map_id")
    pb.add_argument("--build-no", default=None,
                    help="指定 build_no（默认读 <map_id>.json 的 global.build_no）")
    pb.add_argument("--force", action="store_true",
                    help="目标 build_no 已存在时覆盖（默认跳过）")
    pb.set_defaults(func=_cmd_backup)

    pl = sub.add_parser("list", help="列出 <map_id> 的所有备份")
    pl.add_argument("--map", required=True, dest="map_id")
    pl.set_defaults(func=_cmd_list)

    pr = sub.add_parser("restore", help="从备份恢复到 front/<map_id>")
    pr.add_argument("--map", required=True, dest="map_id")
    pr.add_argument("--build-no", default=None,
                    help="要恢复的 build_no（缺省则交互式从列表选择）")
    pr.set_defaults(func=_cmd_restore)

    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )
    try:
        return args.func(args)
    except (ValueError, FileNotFoundError) as e:
        logger.error("%s", e)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
