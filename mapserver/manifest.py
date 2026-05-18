"""Manifest 数据模型 + 构建 / 持久化 / 比对工具。

Manifest 是服务器和客户端之间的共享契约：

```json
{
  "map_id": "408",
  "version": "1.2.3",
  "build_no": 42,
  "generated_at": "2026-05-18T12:34:56Z",
  "files": [
    {"path": "408.json",                   "size": 46027,    "sha256": "ab...", "mtime": 1747551... },
    {"path": "vpr/2f/database_meta.npz",   "size": 612,      "sha256": "cd...", "mtime": 1747551... },
    {"path": "1f.pcd",                     "size": 342648202, "sha256": "ef...", "mtime": 1747551... }
  ]
}
```

- ``version`` / ``build_no``：客户端判断"是否需要更新"的主要凭据。一致 → 无需同步。
- ``files[*].sha256``：客户端判断"哪些文件要重传"的依据；不依赖 mtime（mtime 在
  rsync 等流程里可能被改写）。
- 路径一律使用 ``posix`` 形式（``/`` 分隔），跨平台一致。

版本号来源（按优先级）：

1. ``<map_dir>/<map_id>.json`` 中 ``global.version`` 与 ``global.build_no``
   —— **MapEditor 写出的位置**，单一信息源，地图数据本身和版本号天然耦合。
2. ``<map_dir>/version.json`` 形如 ``{"version": "...", "build_no": N}``
   —— 向后兼容的备用入口。
3. 都没有时自动回退：``version = "auto-<8位 sha>"``、``build_no = int(max(mtime))``。
   保证 manifest 始终可用，但 build_no 不严格单调，仅适合开发环境。
"""

from __future__ import annotations

import hashlib
import json
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple

# 客户端本地存放上次同步 manifest 的文件名（藏在 map_dir 下）。
# 用 dot 前缀避免被当成普通地图资源同步回服务器。
LOCAL_MANIFEST_FILENAME = ".mapserver_manifest.json"

# 服务器构建 manifest 时跳过的文件 / 目录（相对 map_dir 的 posix 路径前缀）。
# 客户端持久化的本地 manifest 也要跳过，否则会自包含、自比对。
DEFAULT_EXCLUDES: Tuple[str, ...] = (
    LOCAL_MANIFEST_FILENAME,
    ".staging/",
    "__pycache__/",
)


@dataclass(frozen=True)
class FileEntry:
    path: str           # 相对 map_dir 的 posix 路径
    size: int
    sha256: str         # 小写 hex
    mtime: float        # epoch 秒，仅参考

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "FileEntry":
        return cls(
            path=str(d["path"]),
            size=int(d["size"]),
            sha256=str(d["sha256"]).lower(),
            mtime=float(d.get("mtime", 0.0)),
        )


@dataclass
class MapManifest:
    map_id: str
    version: str
    build_no: int
    generated_at: str           # ISO 8601 UTC
    files: List[FileEntry] = field(default_factory=list)

    # --- 序列化 ---

    def to_dict(self) -> Dict[str, Any]:
        return {
            "map_id": self.map_id,
            "version": self.version,
            "build_no": self.build_no,
            "generated_at": self.generated_at,
            "files": [f.to_dict() for f in self.files],
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "MapManifest":
        return cls(
            map_id=str(d["map_id"]),
            version=str(d["version"]),
            build_no=int(d["build_no"]),
            generated_at=str(d.get("generated_at", "")),
            files=[FileEntry.from_dict(x) for x in d.get("files", [])],
        )

    # --- 查询 ---

    def file_map(self) -> Dict[str, FileEntry]:
        """``path -> FileEntry`` 字典；同名文件以最后出现者为准。"""
        return {f.path: f for f in self.files}

    def total_bytes(self) -> int:
        return sum(f.size for f in self.files)


# ---------------------------------------------------------------------------
# Hashing helpers
# ---------------------------------------------------------------------------

def _hash_file(path: Path, chunk: int = 1 << 20) -> str:
    """流式 sha256，避免一次性读入超大 PCD。"""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            buf = f.read(chunk)
            if not buf:
                break
            h.update(buf)
    return h.hexdigest()


# ---------------------------------------------------------------------------
# Manifest construction (server-side)
# ---------------------------------------------------------------------------

def _iter_files(
    map_dir: Path,
    excludes: Iterable[str],
) -> Iterable[Path]:
    """递归列出 map_dir 下的常规文件（过滤掉 excludes 命中的）。"""
    map_dir = map_dir.resolve()
    excludes_dir: Set[str] = {e.rstrip("/") for e in excludes if e.endswith("/")}
    excludes_file: Set[str] = {e for e in excludes if not e.endswith("/")}
    for root, dirs, files in os.walk(map_dir):
        # 原地裁剪 dirs，避免下钻
        dirs[:] = [d for d in dirs if d not in excludes_dir]
        for fname in files:
            abs_path = Path(root) / fname
            rel = abs_path.relative_to(map_dir).as_posix()
            if rel in excludes_file:
                continue
            # 软链接、设备文件等一律跳过
            if not abs_path.is_file() or abs_path.is_symlink():
                continue
            yield abs_path


def _read_version_from_main_json(map_dir: Path, map_id: str) -> Optional[Tuple[str, int]]:
    """从 ``<map_dir>/<map_id>.json`` 的 ``global.version`` / ``global.build_no`` 读取。

    这是 MapEditor 保存时写入的位置，是首选信息源。
    缺字段或文件不存在时返回 None。
    """
    main_json = map_dir / f"{map_id}.json"
    if not main_json.is_file():
        return None
    try:
        data = json.loads(main_json.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError):
        return None
    if not isinstance(data, dict):
        return None
    global_obj = data.get("global")
    if not isinstance(global_obj, dict):
        return None
    if "version" not in global_obj and "build_no" not in global_obj:
        return None
    try:
        version = str(global_obj.get("version", "1.0"))
        build_no = int(global_obj.get("build_no", 0))
    except (TypeError, ValueError):
        return None
    return version, build_no


def _read_version_sidecar(map_dir: Path) -> Optional[Tuple[str, int]]:
    """Fallback：从独立的 ``<map_dir>/version.json`` 读取。"""
    vfile = map_dir / "version.json"
    if not vfile.is_file():
        return None
    try:
        data = json.loads(vfile.read_text(encoding="utf-8"))
        version = str(data["version"])
        build_no = int(data["build_no"])
        return version, build_no
    except (OSError, KeyError, ValueError, TypeError, json.JSONDecodeError):
        return None


def _resolve_version_info(map_dir: Path, map_id: str) -> Optional[Tuple[str, int]]:
    """按优先级返回 (version, build_no)：

    1. ``<map_id>.json/global``（MapEditor 写出）
    2. ``version.json``（兼容旧约定）
    3. 都没有 → None，由调用方决定 fallback
    """
    info = _read_version_from_main_json(map_dir, map_id)
    if info is not None:
        return info
    return _read_version_sidecar(map_dir)


def build_manifest(
    map_dir: Path,
    *,
    map_id: Optional[str] = None,
    excludes: Iterable[str] = DEFAULT_EXCLUDES,
) -> MapManifest:
    """扫描 ``map_dir``，给每个文件算 sha256，构建一份 ``MapManifest``。

    :param map_id: 默认取 ``map_dir.name``。
    :param excludes: 跳过的文件/目录（用 ``"foo/"`` 表示目录）。
    :raises FileNotFoundError: ``map_dir`` 不存在。
    :raises NotADirectoryError: ``map_dir`` 不是目录。
    """
    map_dir = Path(map_dir)
    if not map_dir.exists():
        raise FileNotFoundError(f"map dir not found: {map_dir}")
    if not map_dir.is_dir():
        raise NotADirectoryError(f"not a directory: {map_dir}")

    entries: List[FileEntry] = []
    for fp in _iter_files(map_dir, excludes):
        st = fp.stat()
        entries.append(FileEntry(
            path=fp.relative_to(map_dir).as_posix(),
            size=st.st_size,
            sha256=_hash_file(fp),
            mtime=st.st_mtime,
        ))
    # 按 path 排序，让 manifest 顺序稳定（便于人读 + diff）
    entries.sort(key=lambda e: e.path)

    # 版本号：先 <map_id>.json 的 global，再 version.json，最后自动派生
    effective_map_id = map_id or map_dir.name
    version_info = _resolve_version_info(map_dir, effective_map_id)
    if version_info is not None:
        version, build_no = version_info
    else:
        # 自动派生：版本号取整体 sha8，build_no 取最新 mtime（秒）
        agg = hashlib.sha256()
        for e in entries:
            agg.update(e.sha256.encode())
        version = "auto-" + agg.hexdigest()[:8]
        build_no = int(max((int(e.mtime) for e in entries), default=int(time.time())))

    generated_at = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    return MapManifest(
        map_id=map_id or map_dir.name,
        version=version,
        build_no=build_no,
        generated_at=generated_at,
        files=entries,
    )


# ---------------------------------------------------------------------------
# Local manifest persistence (client-side)
# ---------------------------------------------------------------------------

def load_local_manifest(local_dir: Path) -> Optional[MapManifest]:
    """读取 ``<local_dir>/.mapserver_manifest.json``；不存在或损坏返回 None。"""
    p = Path(local_dir) / LOCAL_MANIFEST_FILENAME
    if not p.is_file():
        return None
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
        return MapManifest.from_dict(data)
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
        return None


def save_local_manifest(local_dir: Path, manifest: MapManifest) -> Path:
    """原子写 ``<local_dir>/.mapserver_manifest.json``（先写 .tmp 再 rename）。"""
    local_dir = Path(local_dir)
    local_dir.mkdir(parents=True, exist_ok=True)
    p = local_dir / LOCAL_MANIFEST_FILENAME
    tmp = p.with_suffix(p.suffix + ".tmp")
    tmp.write_text(json.dumps(manifest.to_dict(), indent=2, ensure_ascii=False),
                   encoding="utf-8")
    os.replace(tmp, p)
    return p


# ---------------------------------------------------------------------------
# Diff (client-side)
# ---------------------------------------------------------------------------

@dataclass
class ManifestDiff:
    """``server`` 相对 ``local`` 的差异。"""
    same_version: bool                  # (version, build_no) 完全一致
    to_download: List[FileEntry]        # 需要从 server 下载的（新增或变更）
    to_delete: List[str]                # 本地多余的（路径），由调用方决定要不要删
    unchanged: List[FileEntry]          # 不需要动的

    @property
    def total_download_bytes(self) -> int:
        return sum(f.size for f in self.to_download)


def diff_manifest(
    server: MapManifest,
    local: Optional[MapManifest],
) -> ManifestDiff:
    """根据 sha256 比对，得到一份精确到文件的差异。

    若 ``local`` 是 None（首次同步），等价于 ``to_download = server.files``。
    """
    same_version = (
        local is not None
        and local.version == server.version
        and local.build_no == server.build_no
    )

    local_map = local.file_map() if local is not None else {}
    server_map = server.file_map()

    to_download: List[FileEntry] = []
    unchanged: List[FileEntry] = []
    for path, sf in server_map.items():
        lf = local_map.get(path)
        if lf is None or lf.sha256 != sf.sha256:
            to_download.append(sf)
        else:
            unchanged.append(sf)

    to_delete = [p for p in local_map.keys() if p not in server_map]

    return ManifestDiff(
        same_version=same_version,
        to_download=to_download,
        to_delete=to_delete,
        unchanged=unchanged,
    )
