# 室外（Scene / OSM 主干）地图管理类：场景语义点位、区域、传送门、全局路线
import os
from typing import Dict, List, Optional, TYPE_CHECKING

if TYPE_CHECKING:  # 仅类型提示，避免运行时循环依赖
    from slam_map import SlamMap


class OutdoorMap:
    """
    室外地图管理器：场景（Scene / OSM 主干）层。

    管理 OSM 底图信息、场景语义点位（经纬度 + 航向角，标注在 OSM 底图上）、
    区域分组、场景传送门（进出某张 SLAM 地图的衔接点）以及可跨地图的顶层全局路线。

    全局路线展开与「场景点 → SLAM 入口位姿」换算需要 SLAM 侧信息，通过构造时注入的
    ``slam``（``SlamMap`` 实例）引用完成，二者由 ``MapManager`` 聚合。
    """

    def __init__(self, scene: Dict = None, global_routes: Dict = None,
                 config_path: str = None, slam: "SlamMap" = None):
        """
        :param scene: scene 段（OSM 底图 / 场景语义点位 / 区域 / 传送门）
        :param global_routes: 顶层跨地图全局路线
        :param config_path: 地图配置文件路径（用于解析相对 OSM 数据源路径）
        :param slam: ``SlamMap`` 引用（全局路线展开 / 场景点入口位姿换算需要）
        """
        self.scene: Dict = scene or {}
        self.global_routes: Dict = global_routes or {}
        self.config_path = config_path
        self.slam = slam

    # ==================== 场景（Scene / OSM 主干）层 ====================

    def get_scene_info(self) -> Dict:
        """场景级元信息（name / crs / origin / build_no 等，不含大块 osm 数据）。"""
        if not self.scene:
            return {}
        return {
            'name': self.scene.get('name'),
            'crs': self.scene.get('crs'),
            'origin': self.scene.get('origin'),
            'build_no': self.scene.get('build_no'),
        }

    def get_scene_origin(self) -> Optional[Dict]:
        """场景本地 ENU 换算基准经纬度 {lon, lat}；未设置时回退 OSM 边界中心。"""
        if not self.scene:
            return None
        origin = self.scene.get('origin')
        if origin and origin.get('lon') is not None and origin.get('lat') is not None:
            return origin
        bounds = (self.scene.get('osm', {}) or {}).get('bounds')
        if bounds and bounds.get('min') and bounds.get('max'):
            return {
                'lon': (bounds['min']['lon'] + bounds['max']['lon']) / 2.0,
                'lat': (bounds['min']['lat'] + bounds['max']['lat']) / 2.0,
            }
        return None

    def get_osm_info(self) -> Optional[Dict]:
        """OSM 底图信息 {source, bounds}。"""
        if not self.scene:
            return None
        return self.scene.get('osm')

    def get_osm_full_path(self) -> Optional[str]:
        """OSM 数据源（.osm / .osm.pbf）的完整路径（相对 scene 目录补齐）。"""
        osm = self.get_osm_info() or {}
        src = osm.get('source')
        if not src:
            return None
        if os.path.isabs(src):
            return src
        if self.config_path:
            config_dir = os.path.dirname(os.path.abspath(self.config_path))
            return os.path.join(config_dir, src)
        return src

    def get_scene_semantic_points(self) -> Dict:
        """场景语义点位（经纬度 + 航向角，标注在 OSM 底图上）。"""
        if not self.scene:
            return {}
        return self.scene.get('semantic_points', {}) or {}

    def get_scene_point(self, name: str) -> Optional[Dict]:
        """获取指定名称的场景语义点位（含 lon/lat/heading_deg/type/area 等）。"""
        return self.get_scene_semantic_points().get(name)

    def get_scene_point_areas(self) -> List[str]:
        """场景语义点位的区域（分组）清单。"""
        if not self.scene:
            return []
        return list(self.scene.get('semantic_point_areas', []) or [])

    def get_scene_points_in_area(self, area: str) -> Dict:
        """按区域筛选场景语义点位。"""
        return {
            name: pt for name, pt in self.get_scene_semantic_points().items()
            if pt.get('area') == area
        }

    def get_scene_portals(self) -> Dict:
        """场景传送门 {场景语义点位名: {target_slam_map: ...}}。"""
        if not self.scene:
            return {}
        return self.scene.get('portals', {}) or {}

    def get_scene_portal_target(self, scene_point: str) -> Optional[str]:
        """某场景语义点位（作为传送门）指向的 SLAM 地图 id。"""
        portal = self.get_scene_portals().get(scene_point)
        if not portal:
            return None
        return portal.get('target_slam_map')

    def find_scene_portals_to_slam_map(self, slam_map_id: str) -> List[str]:
        """返回所有通往指定 SLAM 地图的场景传送门（场景语义点位名）列表。"""
        return [
            name for name, portal in self.get_scene_portals().items()
            if (portal or {}).get('target_slam_map') == slam_map_id
        ]

    # ==================== 全局路线（跨地图，顶层 routes） ====================

    def get_global_route_list(self) -> List[Dict[str, str]]:
        """顶层全局路线列表（每项 {name, description}）。"""
        result = []
        for name, cfg in self.global_routes.items():
            result.append({'name': name, 'description': cfg.get('description', '')})
        return result

    def get_global_route_names(self) -> List[str]:
        """顶层全局路线名称列表。"""
        return list(self.global_routes.keys())

    # ==================== 统一 waypoint 定义 ====================
    #
    # 路线（局部/全局）展开后统一为以下两类 waypoint：
    #   - SLAM 型（来自局部路线的 semantic_point）：
    #       {name, type:'slam', slam_map, submap}
    #   - GPS 型（来自全局路线的 scene_point）：
    #       {name, type:'gps', area, lon, lat, heading_deg}
    # 其中 name 统一为点位名（semantic_point / scene_point）。

    @staticmethod
    def classify_waypoint(wp: Dict) -> str:
        """
        为一个 waypoint 标注类型（供 FSM 的 ModeRouter 逐 waypoint 路由）：

        - ``instruction``：``{instruction: "..."}`` → VLN
        - ``scene_point``：``{scene_point: "..."}`` → 场景语义点位（进入对应 SLAM 地图 / 定位锚点）
        - ``gps``：``{lon, lat}`` → GNSS
        - ``map``：``{(slam_map,) submap, semantic_point}`` 或 ``{x, y}`` → SLAM
        - ``unknown``：无法识别
        """
        if 'instruction' in wp:
            return 'instruction'
        if 'scene_point' in wp:
            return 'scene_point'
        if 'lon' in wp and 'lat' in wp:
            return 'gps'
        if 'semantic_point' in wp or ('x' in wp and 'y' in wp):
            return 'map'
        return 'unknown'

    def _gps_waypoint(self, name: str) -> Dict:
        """构造统一的 GPS 型 waypoint：来自场景语义点（scene_point）。

        area / lon / lat / heading_deg 从场景语义点查出（缺失时为 None）。
        """
        pt = self.get_scene_point(name) or {}
        return {
            'name': name,
            'type': 'gps',
            'area': pt.get('area'),
            'lon': pt.get('lon'),
            'lat': pt.get('lat'),
            'heading_deg': pt.get('heading_deg'),
        }

    def _expand_global_waypoints(self, waypoints: List[Dict], visited: set) -> List[Dict]:
        """
        展开全局路线的异构 waypoint 序列，统一为 SLAM / GPS 两类 waypoint：

        - ``{slam_map, route}``：把该 SLAM 地图的一条内部路线整体展开为若干
          统一 SLAM 型 ``{name, type:'slam', slam_map, submap}``；
        - ``{route}``（无 slam_map）：递归展开另一条**全局路线**；
        - ``{scene_point}``：统一 GPS 型 ``{name, type:'gps', area, lon, lat, heading_deg}``；
        - ``{(slam_map,) submap, semantic_point}`` / ``{x,y}``：统一 SLAM 型；
        - ``{lon,lat}``（无 scene_point 名）：统一 GPS 型（area/name 尽力补全）；
        - ``{instruction}``：透传为 ``{name:None, type:'instruction', instruction}``。
        """
        flat: List[Dict] = []
        cur_slam_map = self.slam.current_slam_map if self.slam else None
        for wp in waypoints:
            # 1) SLAM 地图内部路线整体引用 → 已是统一 SLAM 型
            if 'route' in wp and 'slam_map' in wp:
                sid = wp['slam_map']
                sub = self.slam.get_route_by_name(wp['route'], slam_map_id=sid) if self.slam else None
                if not sub:
                    print(f"OutdoorMap: 全局路线引用的内部路线不存在，跳过: {sid}/{wp['route']}")
                    continue
                flat.extend(sub['waypoints'])
                continue
            # 2) 嵌套全局路线引用
            if 'route' in wp and 'slam_map' not in wp:
                ref = wp['route']
                if ref in visited:
                    print(f"OutdoorMap: 检测到全局路线循环引用，跳过: {ref}")
                    continue
                ref_cfg = self.global_routes.get(ref)
                if not ref_cfg:
                    print(f"OutdoorMap: 嵌套全局路线不存在，跳过: {ref}")
                    continue
                visited.add(ref)
                flat.extend(self._expand_global_waypoints(ref_cfg.get('waypoints', []), visited))
                visited.discard(ref)
                continue
            # 3) 普通异构 waypoint → 统一为 SLAM / GPS
            wtype = self.classify_waypoint(wp)
            if wtype == 'scene_point':
                flat.append(self._gps_waypoint(wp['scene_point']))
            elif wtype == 'gps':
                # 裸经纬度点（无 scene_point 名）
                flat.append({
                    'name': wp.get('name'),
                    'type': 'gps',
                    'area': wp.get('area'),
                    'lon': wp.get('lon'),
                    'lat': wp.get('lat'),
                    'heading_deg': wp.get('heading_deg'),
                })
            elif wtype == 'map':
                from slam_map import SlamMap
                item = SlamMap._slam_waypoint(
                    wp.get('semantic_point'), wp.get('submap'),
                    wp.get('slam_map') or cur_slam_map)
                if 'x' in wp and 'y' in wp:
                    item['x'] = wp['x']
                    item['y'] = wp['y']
                flat.append(item)
            elif wtype == 'instruction':
                flat.append({
                    'name': None,
                    'type': 'instruction',
                    'instruction': wp.get('instruction'),
                })
            else:
                item = dict(wp)
                item['type'] = 'unknown'
                flat.append(item)
        return flat

    def get_global_route_by_name(self, route_name: str) -> Optional[Dict]:
        """
        根据名称获取顶层全局路线，返回展开并统一后的 waypoint 序列。

        :return: {name, description, waypoints:[{name, type: slam|gps|instruction, ...}]}
                 路线不存在返回 None
        """
        cfg = self.global_routes.get(route_name)
        if not cfg:
            return None
        flat = self._expand_global_waypoints(cfg.get('waypoints', []), {route_name})
        return {
            'name': route_name,
            'description': cfg.get('description', ''),
            'waypoints': flat,
        }

    def get_slam_entry_pose_from_scene_point(self, scene_point: str) -> Optional[Dict]:
        """
        由某场景语义点位（通常为场景传送门）计算进入对应 SLAM 地图的根系初始位姿。

        :return: {slam_map, lon, lat, pose}；场景点不存在返回 None，
                 非传送门（无 target_slam_map）或缺 geo_anchor 时 pose 为 None。
        """
        pt = self.get_scene_point(scene_point)
        if not pt:
            return None
        target = self.get_scene_portal_target(scene_point)
        pose = None
        if target and self.slam:
            pose = self.slam.lonlat_to_slam_root_pose(
                pt.get('lon'), pt.get('lat'), pt.get('heading_deg'), target)
        return {
            'slam_map': target,
            'lon': pt.get('lon'),
            'lat': pt.get('lat'),
            'pose': pose,
        }
