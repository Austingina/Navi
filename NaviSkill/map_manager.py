# 地图语义管理类：聚合 SlamMap（室内 SLAM）与 OutdoorMap（室外 Scene/OSM）
import os
import json
from typing import Dict, Optional

from slam_map import SlamMap
from outdoor_map import OutdoorMap


class MapManager:
    """
    多模式 / 分层地图管理器（聚合器 / Facade）。

    支持两类地图描述文件（详见
    ``MapEditor/docs/multi-modal_navigation_mapdata_file-format.md`` 与
    ``NaviSkill/docs/fsm_navigation_design.md``）：

    1. **场景文件（scene）**：三层分层地图模型
       ``Scene(OSM/WGS84 主干) → slam_map(geo_anchor 锚定) → submap``。
       顶层含 ``scene``（OSM 底图、场景语义点位/区域/传送门）、``slam_maps``
       （多张 SLAM 地图，各带 ``geo_anchor``、内部为标准多 submap 结构）与顶层
       ``routes``（可跨地图的全局路线）。
    2. **单张 SLAM 地图文件（multifloor.json）**：顶层直接是 ``global`` /
       ``submaps`` / ``routes``。加载时会被包装成「只有一张 SLAM 地图的退化场景」，
       老配置零改动继续工作。

    内部拆分为两个组件，本类聚合并对外暴露统一接口：

    - :class:`SlamMap`（``self.slam``）：室内 SLAM 地图 —— slam_map / submap /
      语义点位 / 传送门 / 内部路线 / 级联变换 / geo_anchor 换算；
    - :class:`OutdoorMap`（``self.outdoor``）：室外 Scene/OSM 主干 —— 场景语义点位 /
      区域 / 场景传送门 / 跨地图全局路线。

    对 :class:`SlamMap` / :class:`OutdoorMap` 上的公共方法通过 ``__getattr__``
    自动委派（如 ``get_submap_list`` / ``get_scene_point`` / ``get_route_by_name``
    / ``get_global_route_by_name`` 等），调用方无需关心其归属组件。
    """

    # 退化场景（旧 multifloor.json）里隐式 SLAM 地图的 id
    DEFAULT_SLAM_MAP_ID = SlamMap.DEFAULT_SLAM_MAP_ID

    def __init__(self, map_config_path: str = None):
        """
        初始化地图管理器

        :param map_config_path: 地图配置文件路径（scene.json 或 multifloor.json）
        """
        self.config_path = map_config_path
        self.config = {}          # 原始配置文件内容（完整）

        # 两个子组件（load_config 后重建；未加载时为空实例，方法安全降级）
        self.slam = SlamMap()
        self.outdoor = OutdoorMap()

        if map_config_path:
            self.load_config(map_config_path)

    # ==================== 加载 ====================

    def load_config(self, config_path: str):
        """
        加载地图配置文件（自动识别 scene / 单图两种格式），并重建子组件。

        :param config_path: 配置文件路径
        """
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"地图配置文件不存在: {config_path}")

        self.config_path = config_path
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = json.load(f)

        is_scene = ('scene' in self.config) or ('slam_maps' in self.config)

        if is_scene:
            scene = self.config.get('scene', {}) or {}
            global_routes = self.config.get('routes', {}) or {}
            raw_slam_maps = self.config.get('slam_maps', {}) or {}
        else:
            # 退化场景：把整份单图配置包装成唯一的隐式 SLAM 地图
            scene = {}
            global_routes = {}
            implicit = dict(self.config)
            implicit.setdefault('global', self.config.get('global', {}) or {})
            implicit.setdefault('submaps', self.config.get('submaps', {}) or {})
            implicit.setdefault('routes', self.config.get('routes', {}) or {})
            raw_slam_maps = {self.DEFAULT_SLAM_MAP_ID: implicit}

        self.slam = SlamMap(
            raw_slam_maps,
            config_path=config_path,
            default_slam_map_hint=scene.get('default_slam_map'),
        )
        self.outdoor = OutdoorMap(
            scene,
            global_routes,
            config_path=config_path,
            slam=self.slam,
        )

        print(
            f"MapManager: 已加载地图配置: {config_path}, "
            f"场景={'是' if is_scene else '否(单图退化)'}, "
            f"slam_map数: {len(self.slam.slam_maps)}, 当前slam_map: {self.slam.current_slam_map}, "
            f"submap数: {len(self.slam.submaps)}, 当前submap: {self.slam.current_submap}"
        )

    # ==================== 聚合层：跨室内外 / 类级 ====================

    def has_scene(self) -> bool:
        """是否为场景文件（含 OSM 主干 / 多张 SLAM 地图）。"""
        sm = self.slam.slam_maps
        return bool(self.outdoor.scene) or len(sm) > 1 or (
            len(sm) == 1 and self.DEFAULT_SLAM_MAP_ID not in sm)

    @staticmethod
    def classify_waypoint(wp: Dict) -> str:
        """为一个 waypoint 标注类型（委派 :meth:`OutdoorMap.classify_waypoint`）。"""
        return OutdoorMap.classify_waypoint(wp)

    # 供外部（如 server.py）复用的 SLAM 位姿/四元数静态工具（委派到 SlamMap）
    _quaternion_multiply = staticmethod(SlamMap._quaternion_multiply)
    _quaternion_inverse = staticmethod(SlamMap._quaternion_inverse)
    _rotate_point_by_quaternion = staticmethod(SlamMap._rotate_point_by_quaternion)
    _apply_transform = staticmethod(SlamMap._apply_transform)
    _apply_inverse_transform = staticmethod(SlamMap._apply_inverse_transform)

    # ==================== 委派 ====================

    def __getattr__(self, name: str):
        """把未在本类定义的实例属性/方法委派给子组件（先 SLAM 后室外）。

        覆盖 ``get_submap_*`` / ``get_pose_in_submap`` / ``get_route_by_name`` /
        ``get_slam_map_*`` / geo_anchor 换算（SlamMap）以及 ``get_scene_*`` /
        ``get_global_route_*`` / ``find_scene_portals_to_slam_map`` /
        ``get_slam_entry_pose_from_scene_point``（OutdoorMap）等全部公共接口，
        同时兼容以实例访问的 ``pose_map_current_submap`` 属性与 ``current_*`` 状态。
        """
        # 避免子组件尚未建立时（如 __init__ 早期）递归
        if name in ('slam', 'outdoor'):
            raise AttributeError(name)
        slam = self.__dict__.get('slam')
        outdoor = self.__dict__.get('outdoor')
        for comp in (slam, outdoor):
            if comp is not None:
                try:
                    return getattr(comp, name)
                except AttributeError:
                    continue
        raise AttributeError(f"'MapManager' object has no attribute '{name}'")
