"""Map distribution service: server + incremental-sync client.

模块布局：
- ``manifest`` —— 服务器-客户端共享的 manifest 数据模型与构建/比对函数
- ``server``   —— FastAPI HTTP 服务，可作为 ``python -m mapserver.server`` 启动
- ``client``   —— 增量同步客户端，可作为 ``python -m mapserver.client`` 启动

参见 ``README.md`` 获取部署与使用说明。
"""

from .manifest import (
    FileEntry,
    MapManifest,
    build_manifest,
    diff_manifest,
    load_local_manifest,
    save_local_manifest,
)
from .client import (
    SyncResult,
    SyncStatus,
    parse_mapserver_url,
    resolve_map_source,
    sync_map,
)

__all__ = [
    "FileEntry",
    "MapManifest",
    "build_manifest",
    "diff_manifest",
    "load_local_manifest",
    "save_local_manifest",
    "SyncResult",
    "SyncStatus",
    "sync_map",
    "resolve_map_source",
    "parse_mapserver_url",
]
