# SLAM 地图管理类（多张 SLAM 地图 / submap / 内部路线 / 级联变换 / geo_anchor 换算）
import os
import math
from typing import Dict, List, Optional, Tuple


class SlamMap:
    """
    SLAM（室内点云）地图管理器。

    管理一份 ``slam_maps`` 集合：每张 SLAM 地图由 ``geo_anchor`` 锚定到 WGS84，
    内部为标准多 ``submap`` 结构（含语义点位、传送门、级联 ``transform``、内部路线）。

    对外接口分为两层：

    - **slam_map 层**（``get_slam_map_*`` / ``set_current_slam_map`` /
      ``get_slam_map_geo_anchor`` 等）：在多张 SLAM 地图间选择与查询；
    - **submap 层**（``get_submap_*`` / ``get_pose_in_submap`` / ``get_route_by_name``
      等）：作用于**当前 SLAM 地图**（``current_slam_map``）。

    另提供 geo_anchor 换算（SLAM map 坐标 ↔ WGS84）与跨 submap 位姿换算，
    供室内外衔接（GNSS/OSM ↔ SLAM）与换图初始定位使用。
    """

    # 退化场景（旧 multifloor.json）里隐式 SLAM 地图的 id
    DEFAULT_SLAM_MAP_ID = "default"

    def __init__(self, raw_slam_maps: Dict = None, config_path: str = None,
                 default_slam_map_hint: str = None):
        """
        :param raw_slam_maps: 原始 ``{slam_map_id: 配置}``（未归一化，可能含 ``source``）
        :param config_path: 地图配置文件路径（用于解析相对 ``pcd_base`` / ``source``）
        :param default_slam_map_hint: 默认 SLAM 地图 id 提示（来自 scene.default_slam_map）
        """
        self.config_path = config_path
        config_dir = (os.path.dirname(os.path.abspath(config_path))
                      if config_path else None)

        self.slam_maps: Dict = {}         # {slam_map_id: 归一化后的 slam_map 配置}
        for sid, raw_cfg in (raw_slam_maps or {}).items():
            self.slam_maps[sid] = self._resolve_slam_map(raw_cfg, config_dir)

        self.default_slam_map = None      # 默认 SLAM 地图 id
        self.current_slam_map = None      # 当前 SLAM 地图 id

        # submap 层（指向“当前 SLAM 地图”）
        self.submaps: Dict = {}           # 当前 SLAM 地图的 submap 配置
        self.current_submap = None        # 当前 submap ID
        self.default_submap = None        # 当前 SLAM 地图的默认 submap ID
        self.default_route = None         # 当前 SLAM 地图的默认路线名称

        if self.slam_maps:
            self.default_slam_map = self._determine_default_slam_map(default_slam_map_hint)
            if self.default_slam_map:
                self._activate_slam_map(self.default_slam_map)

    # ==================== 加载 / 归一化 ====================

    def _resolve_slam_map(self, raw_cfg: Dict, config_dir: str) -> Dict:
        """
        归一化一条 ``slam_maps.<id>`` 配置。

        - 若含 ``source``：读取外部单图 JSON（相对 scene 目录解析），把其
          ``global`` / ``submaps`` / ``routes`` **就地展开**，scene 条目中显式写出的
          字段优先覆盖；外部文件不得再含 ``scene`` / ``slam_maps``（防多层嵌套）。
        - 记录 ``_source_dir`` 供 ``pcd_base`` 缺省时解析点云路径。
        """
        import json
        cfg = dict(raw_cfg)
        source = cfg.pop('source', None)
        if not source:
            return cfg

        ext_path = source if os.path.isabs(source) else os.path.join(config_dir, source)
        if not os.path.exists(ext_path):
            print(f"SlamMap: slam_map source 文件不存在，忽略引用: {ext_path}")
            return cfg
        with open(ext_path, 'r', encoding='utf-8') as f:
            ext = json.load(f)
        if 'scene' in ext or 'slam_maps' in ext:
            raise ValueError(
                f"slam_map source 文件不应再包含 scene/slam_maps（禁止多层嵌套）: {ext_path}")

        merged = dict(ext)
        # scene 条目字段覆盖外部同名字段（global 做浅合并，便于场景级微调）
        for k, v in cfg.items():
            if k == 'global' and isinstance(v, dict) and isinstance(merged.get('global'), dict):
                g = dict(merged.get('global', {}))
                g.update(v)
                merged['global'] = g
            else:
                merged[k] = v
        merged['_source_dir'] = os.path.dirname(ext_path)
        return merged

    def _determine_default_slam_map(self, hint: str = None) -> Optional[str]:
        """选择默认 SLAM 地图：hint（scene.default_slam_map）> 第一张有 submap 的 > 第一张。"""
        if not self.slam_maps:
            return None
        if hint and hint in self.slam_maps:
            return hint
        for sid, cfg in self.slam_maps.items():
            if cfg.get('submaps'):
                return sid
        return next(iter(self.slam_maps))

    def _activate_slam_map(self, slam_map_id: str):
        """把某张 SLAM 地图设为“当前”，并同步 submap 层的兼容属性。"""
        cfg = self.slam_maps.get(slam_map_id, {})
        self.current_slam_map = slam_map_id

        g = cfg.get('global', {}) or {}
        self.default_submap = g.get('default_submap') or None
        self.default_route = g.get('default_route') or None
        self.submaps = cfg.get('submaps', {}) or {}

        if self.default_submap and self.default_submap in self.submaps:
            self.current_submap = self.default_submap
        elif self.submaps:
            self.current_submap = list(self.submaps.keys())[0]
        else:
            self.current_submap = None

    # ---- 内部：按 slam_map 取子结构 ----

    def _get_submaps(self, slam_map_id: str = None) -> Dict:
        """取指定（默认当前）SLAM 地图的 submap 集合。"""
        if slam_map_id is None or slam_map_id == self.current_slam_map:
            return self.submaps
        cfg = self.slam_maps.get(slam_map_id)
        return (cfg.get('submaps', {}) or {}) if cfg else {}

    def _get_slam_routes(self, slam_map_id: str = None) -> Dict:
        """取指定（默认当前）SLAM 地图的内部路线集合。"""
        if slam_map_id is None or slam_map_id == self.current_slam_map:
            slam_map_id = self.current_slam_map
        cfg = self.slam_maps.get(slam_map_id)
        return (cfg.get('routes', {}) or {}) if cfg else {}

    def _current_slam_routes(self) -> Dict:
        return self._get_slam_routes(self.current_slam_map)

    def _default_submap_of(self, slam_map_id: str) -> Optional[str]:
        """指定 SLAM 地图的默认 submap（无显式配置则取第一张）。"""
        cfg = self.slam_maps.get(slam_map_id)
        if not cfg:
            return None
        submaps = cfg.get('submaps', {}) or {}
        default = (cfg.get('global', {}) or {}).get('default_submap') or None
        if default and default in submaps:
            return default
        return next(iter(submaps)) if submaps else None

    # ==================== SLAM 地图（slam_map）层 ====================

    def get_slam_map_list(self) -> List[str]:
        """所有 SLAM 地图 id 列表。"""
        return list(self.slam_maps.keys())

    def get_slam_map_info(self, slam_map_id: str = None) -> Optional[Dict]:
        """指定（默认当前）SLAM 地图的配置。"""
        slam_map_id = slam_map_id or self.current_slam_map
        return self.slam_maps.get(slam_map_id)

    def get_current_slam_map(self) -> Optional[str]:
        """当前 SLAM 地图 id。"""
        return self.current_slam_map

    def get_default_slam_map(self) -> Optional[str]:
        """默认 SLAM 地图 id。"""
        return self.default_slam_map

    def set_current_slam_map(self, slam_map_id: str) -> bool:
        """
        切换当前 SLAM 地图（同时把 submap 层复位到该地图的默认 submap）。
        """
        if slam_map_id in self.slam_maps:
            self._activate_slam_map(slam_map_id)
            print(f"SlamMap: 当前slam_map已切换为: {slam_map_id}, submap: {self.current_submap}")
            return True
        print(f"SlamMap: slam_map不存在: {slam_map_id}")
        return False

    def get_slam_map_pcd_base(self, slam_map_id: str = None) -> Optional[str]:
        """指定 SLAM 地图的 ``pcd_base``（解析各 submap map_file 的基准目录）。"""
        slam_map_id = slam_map_id or self.current_slam_map
        cfg = self.slam_maps.get(slam_map_id)
        return cfg.get('pcd_base') if cfg else None

    def get_submap_list(self, slam_map_id: str = None) -> List[str]:
        """获取（指定 / 当前 SLAM 地图的）所有 submap ID 列表"""
        return list(self._get_submaps(slam_map_id).keys())

    def get_submap_info(self, submap_id: str, slam_map_id: str = None) -> Optional[Dict]:
        """获取（指定 / 当前 SLAM 地图的）指定 submap 的信息"""
        return self._get_submaps(slam_map_id).get(submap_id)

    def get_submap_map_file(self, submap_id: str) -> Optional[str]:
        """获取指定 submap 的地图文件名"""
        submap = self.submaps.get(submap_id)
        if submap:
            return submap.get('map_file')
        return None

    def _pcd_base_dir(self, slam_map_id: str = None) -> Optional[str]:
        """
        解析某张 SLAM 地图点云文件（``map_file``）所在的基准目录。

        优先级：显式 ``pcd_base``（相对 scene 目录 / source 目录，或绝对路径）
        > ``source`` 外部文件所在目录 > 配置文件所在目录（旧单图行为）。
        """
        slam_map_id = slam_map_id or self.current_slam_map
        config_dir = (os.path.dirname(os.path.abspath(self.config_path))
                      if self.config_path else None)
        cfg = self.slam_maps.get(slam_map_id, {}) or {}
        pcd_base = cfg.get('pcd_base')
        src_dir = cfg.get('_source_dir')
        if pcd_base:
            if os.path.isabs(pcd_base):
                return pcd_base
            base_dir = src_dir or config_dir
            return os.path.join(base_dir, pcd_base) if base_dir else pcd_base
        if src_dir:
            return src_dir
        return config_dir

    def get_submap_full_map_path(self, submap_id: str, slam_map_id: str = None) -> Optional[str]:
        """
        获取指定 submap 的完整地图（点云）文件路径。

        相对 ``map_file`` 会依次基于 SLAM 地图的 ``pcd_base`` / ``source`` 目录 /
        配置文件目录补齐（见 ``_pcd_base_dir``）。

        :param submap_id: submap ID
        :param slam_map_id: SLAM 地图 id，默认当前 SLAM 地图
        :return: 完整的地图文件路径，如果 submap 不存在返回 None
        """
        submaps = self._get_submaps(slam_map_id)
        submap = submaps.get(submap_id)
        map_file = submap.get('map_file') if submap else None
        if not map_file:
            return None

        # 如果已经是绝对路径，直接返回
        if os.path.isabs(map_file):
            return map_file

        base_dir = self._pcd_base_dir(slam_map_id)
        if base_dir:
            return os.path.join(base_dir, map_file)
        return map_file

    def set_current_submap(self, submap_id: str) -> bool:
        """设置当前 submap"""
        if submap_id in self.submaps:
            self.current_submap = submap_id
            print(f"SlamMap: 当前submap已切换为: {submap_id}")
            return True
        print(f"SlamMap: submap不存在: {submap_id}")
        return False

    def get_current_submap(self) -> Optional[str]:
        """获取当前 submap ID"""
        return self.current_submap

    def get_semantic_points_in_submap(self, submap_id: str = None, slam_map_id: str = None) -> Dict:
        """获取指定 submap 的所有语义点位（默认当前 SLAM 地图 / 当前 submap）"""
        submaps = self._get_submaps(slam_map_id)
        if submap_id is None:
            if slam_map_id is None or slam_map_id == self.current_slam_map:
                submap_id = self.current_submap
            else:
                submap_id = self._default_submap_of(slam_map_id)
        if not submap_id or submap_id not in submaps:
            return {}
        return submaps[submap_id].get('semantic_points', {})

    def get_pose_in_submap(self, pose_name: str, submap_id: str = None,
                           slam_map_id: str = None) -> Optional[Dict]:
        """
        获取指定名称的点位信息（返回整个 point_info）

        :param pose_name: 点位名称
        :param submap_id: submap ID，默认为当前 submap
        :param slam_map_id: SLAM 地图 id，默认为当前 SLAM 地图
        :return: point_info 字典（包含 `type`, `pose` 等），找不到返回 None

        point_info 中 `type` 字段说明：
        - "single_pose": 单一位姿点；使用普通导航接口 `set_goal_by_slam` 下发目标。
        - "multi_pose": 多位姿点（一串按顺序经过的位姿，用 `poses` 数组）；当前无专用下发接口。
        - "elevator_call": 电梯呼叫点；机器人到达该点后应等待外部 agent 执行呼叫电梯的动作。
        - "elevator_wait": 电梯等待点；机器人到达该点后应等待外部 agent 确认电梯已到达且车门已打开。
        - "elevator_in": 进梯后/新楼层落位点；`wait_goal_reached` 在到达后轮询 map 定位（`get_status.localized`），就绪即继续。
        """
        semantic_points = self.get_semantic_points_in_submap(submap_id, slam_map_id)
        point_info = semantic_points.get(pose_name)
        if point_info:
            return point_info
        return None

    def get_multi_poses_in_submap(self, pose_name: str, submap_id: str = None) -> List[Dict]:
        """
        获取指定名称的所有位姿（支持 multi_pose 类型）

        :param pose_name: 点位名称
        :param submap_id: submap ID，默认为当前 submap
        :return: 位姿列表
        """
        submap_id = submap_id or self.current_submap
        semantic_points = self.get_semantic_points_in_submap(submap_id)
        point_info = semantic_points.get(pose_name)
        if not point_info:
            return []

        if point_info.get('type') == 'multi_pose':
            return point_info.get('poses', [])
        else:
            pose = point_info.get('pose')
            return [pose] if pose else []

    def is_multi_pose(self, pose_name: str, submap_id: str = None) -> bool:
        """判断指定名称是否为多位姿点"""
        submap_id = submap_id or self.current_submap
        semantic_points = self.get_semantic_points_in_submap(submap_id)
        point_info = semantic_points.get(pose_name)
        return point_info is not None and point_info.get('type') == 'multi_pose'

    @staticmethod
    def default_need_inspect(point_type: Optional[str]) -> bool:
        """根据点位 type 返回 need_inspect 的默认值。

        约定：single_pose 默认 True，其它类型默认 False。
        """
        return point_type == 'single_pose'

    def get_need_inspect(self, pose_name: str, submap_id: str = None) -> bool:
        """获取指定语义点的 need_inspect 字段。

        若 JSON 中显式指定则按其值返回；否则按 type 的默认值返回
        （single_pose -> True，其它 -> False）。点位不存在时返回 False。
        """
        submap_id = submap_id or self.current_submap
        semantic_points = self.get_semantic_points_in_submap(submap_id)
        point_info = semantic_points.get(pose_name)
        if not point_info:
            return False
        if 'need_inspect' in point_info:
            return bool(point_info.get('need_inspect'))
        return self.default_need_inspect(point_info.get('type'))

    def find_pose_in_all_submaps(self, pose_name: str) -> List[Tuple[str, Dict]]:
        """
        在所有 submap 中查找指定名称的点位

        :param pose_name: 点位名称
        :return: [(submap_id, point_info), ...] 包含该点位的 submap 和位姿信息列表
        """
        results = []
        for submap_id, submap_config in self.submaps.items():
            semantic_points = submap_config.get('semantic_points', {})
            if pose_name in semantic_points:
                point_info = semantic_points[pose_name]
                results.append((submap_id, point_info))
        return results

    def get_pose_with_submap_info(self, pose_name: str) -> Dict:
        """
        获取位姿信息，同时返回该点位所在的所有 submap

        :param pose_name: 点位名称
        :return: {
            'found': bool,
            'submaps': [submap_id, ...],
            'poses': {submap_id: point_info, ...}
        }
        """
        submap_poses = self.find_pose_in_all_submaps(pose_name)
        if not submap_poses:
            return {'found': False, 'submaps': [], 'poses': {}}

        return {
            'found': True,
            'submaps': [sp[0] for sp in submap_poses],
            'poses': {sp[0]: sp[1] for sp in submap_poses}
        }

    def get_submap_portals(self, submap_id: str = None) -> List[Dict]:
        """获取指定 submap 的所有传送门配置"""
        submap_id = submap_id or self.current_submap
        if not submap_id or submap_id not in self.submaps:
            return []
        return self.submaps[submap_id].get('submap_portals', [])

    def get_portals_to_submap(self, target_submap: str, from_submap: str = None) -> List[Dict]:
        """
        获取从起始 submap 直达目标 submap 的所有传送门配置（可能多条）。
        """
        from_submap = from_submap or self.current_submap
        if not from_submap:
            return []
        return [
            p for p in self.get_submap_portals(from_submap)
            if p.get('submap') == target_submap
        ]

    def _portal_semantic_position(
        self, portal_name: str, submap_id: str = None
    ) -> Optional[Tuple[float, float, float]]:
        """传送门语义点位置（portal 非 multi_pose，仅用单 pose.position）。"""
        pi = self.get_pose_in_submap(portal_name, submap_id)
        if not pi:
            return None
        pose = pi.get('pose')
        if not pose:
            return None
        pos = pose.get('position') or {}
        return (
            float(pos.get('x', 0.0)),
            float(pos.get('y', 0.0)),
            float(pos.get('z', 0.0)),
        )

    def get_portal_to_submap(
        self,
        target_submap: str,
        from_submap: str = None,
        robot_position: Optional[Tuple[float, float, float]] = None,
        xy_distance_only: bool = True,
    ) -> Optional[Dict]:
        """
        获取从当前 submap 到目标 submap 的传送门。

        若存在多个 portal 均可直达目标 submap，且提供了 ``robot_position`` ``(x,y,z)``，
        则按与 portal 对应语义点位置的距离选最近的一条；距离默认仅使用 XY 平面。
        若无位姿或无法解析某 portal 坐标，则回退为列表中的第一条。

        :param target_submap: 目标 submap ID
        :param from_submap: 起始 submap ID，默认为当前 submap
        :param robot_position: 机器人当前位置（与地图语义点一致坐标系，通常为 map/odom）
        :param xy_distance_only: True 时只用 sqrt(dx^2+dy^2) 比较
        :return: 传送门配置 {portal, submap, ...}
        """
        from_submap = from_submap or self.current_submap
        if not from_submap:
            return None

        candidates = self.get_portals_to_submap(target_submap, from_submap)
        if not candidates:
            return None
        if len(candidates) == 1:
            return candidates[0]

        if robot_position is None:
            return candidates[0]

        rx, ry, rz = robot_position[0], robot_position[1], robot_position[2]
        best: Optional[Dict] = None
        best_d = float('inf')

        for cfg in candidates:
            pname = cfg.get('portal')
            if not pname:
                continue
            pos = self._portal_semantic_position(pname, from_submap)
            if pos is None:
                continue
            px, py, pz = pos
            if xy_distance_only:
                d = math.hypot(px - rx, py - ry)
            else:
                d = math.sqrt((px - rx) ** 2 + (py - ry) ** 2 + (pz - rz) ** 2)
            if d < best_d:
                best_d = d
                best = cfg

        return best if best is not None else candidates[0]

    def get_portal_pose(self, portal_name: str, submap_id: str = None) -> Optional[Dict]:
        """
        获取传送门点位的点位信息（与 `get_pose_in_submap` 行为一致）

        :param portal_name: 传送门语义点名称
        :param submap_id: submap ID，默认为当前 submap
        :return: point_info 字典（包含 type, pose 等），找不到返回 None
        """
        return self.get_pose_in_submap(portal_name, submap_id)

    def get_all_pose_names_in_submap(self, submap_id: str = None) -> List[str]:
        """获取指定 submap 的所有点位名称"""
        semantic_points = self.get_semantic_points_in_submap(submap_id)
        return list(semantic_points.keys())

    def get_all_pose_names_all_submaps(self) -> Dict[str, List[str]]:
        """获取所有 submap 的点位名称"""
        result = {}
        for submap_id in self.submaps:
            result[submap_id] = self.get_all_pose_names_in_submap(submap_id)
        return result

    @property
    def pose_map_current_submap(self) -> Dict:
        """
        向后兼容属性：返回当前 submap 的语义点位映射
        格式：{pose_name: point_info, ...}
        """
        return self.get_semantic_points_in_submap(self.current_submap)

    # ========== 路线管理接口（当前 SLAM 地图的内部路线） ==========

    def get_route_list(self, slam_map_id: str = None) -> List[Dict[str, str]]:
        """
        获取（指定 / 当前 SLAM 地图的）所有内部路线列表

        :return: 路线列表，每个元素包含 {name, description}
        """
        routes = self._get_slam_routes(slam_map_id)
        result = []
        for route_name, route_config in routes.items():
            result.append({
                'name': route_name,
                'description': route_config.get('description', '')
            })
        return result

    def _flatten_waypoints(self, waypoints: List[Dict], visited: set,
                           routes: Dict = None) -> List[Dict]:
        """
        递归展开嵌套路线引用，返回扁平的 waypoint 列表。

        waypoint 格式：
        - 普通点: {"submap": "...", "semantic_point": "..."}
        - 嵌套路线引用: {"route": "其他路线名"}

        :param waypoints: 原始 waypoint 列表（可能包含嵌套引用）
        :param visited: 已访问的路线名集合（用于防止循环引用）
        :param routes: 供解析嵌套引用的路线集合，默认当前 SLAM 地图的内部路线
        :return: 展开后的 [{submap, semantic_point}, ...] 列表
        """
        if routes is None:
            routes = self._current_slam_routes()
        flat = []
        for wp in waypoints:
            if 'route' in wp:
                ref_name = wp['route']
                if ref_name in visited:
                    print(f"SlamMap: 检测到路线循环引用，跳过: {ref_name}")
                    continue
                ref_config = routes.get(ref_name)
                if not ref_config:
                    print(f"SlamMap: 嵌套路线不存在，跳过: {ref_name}")
                    continue
                visited.add(ref_name)
                flat.extend(self._flatten_waypoints(ref_config.get('waypoints', []), visited, routes))
                visited.discard(ref_name)
            else:
                flat.append(wp)
        return flat

    @staticmethod
    def _slam_waypoint(name: Optional[str], submap: Optional[str],
                       slam_map: Optional[str]) -> Dict:
        """构造统一的 SLAM 型 waypoint：来自某 SLAM 地图某 submap 的语义点。"""
        return {
            'name': name,
            'type': 'slam',
            'slam_map': slam_map,
            'submap': submap,
        }

    def get_route_by_name(self, route_name: str, slam_map_id: str = None) -> Optional[Dict]:
        """
        根据路线名称获取（指定 / 当前 SLAM 地图的）内部路线详细信息
        （自动展开嵌套路线引用）

        :param route_name: 路线名称
        :param slam_map_id: SLAM 地图 id，默认当前 SLAM 地图
        :return: 路线配置，包含 description 和 waypoints
                 waypoints 为统一 SLAM 型: [{name, type:'slam', slam_map, submap}, ...]
                 嵌套路线引用会被递归展开为扁平列表
                 如果路线不存在返回 None
        """
        routes = self._get_slam_routes(slam_map_id)
        route_config = routes.get(route_name)
        if not route_config:
            return None
        eff_slam_map = slam_map_id or self.current_slam_map
        raw_waypoints = route_config.get('waypoints', [])
        flat_waypoints = self._flatten_waypoints(raw_waypoints, {route_name}, routes)
        waypoints = []
        for wp in flat_waypoints:
            item = self._slam_waypoint(
                wp.get('semantic_point'), wp.get('submap'),
                wp.get('slam_map') or eff_slam_map)
            # 罕见：局部路线里直接给 map 系坐标 {x, y}（无语义点名），原样保留
            if 'x' in wp and 'y' in wp:
                item['x'] = wp['x']
                item['y'] = wp['y']
            waypoints.append(item)
        return {
            'name': route_name,
            'description': route_config.get('description', ''),
            'waypoints': waypoints
        }

    def get_default_route(self) -> Optional[str]:
        """获取当前 SLAM 地图的默认路线名称"""
        return self.default_route

    def get_route_names(self, slam_map_id: str = None) -> List[str]:
        """
        获取（指定 / 当前 SLAM 地图的）所有内部路线名称

        :return: 路线名称列表
        """
        return list(self._get_slam_routes(slam_map_id).keys())

    def get_submap_id_by_map_file(self, map_file_path: str) -> Optional[str]:
        """
        根据地图文件路径获取 submap ID

        :param map_file_path: 地图文件路径（可以是完整路径或文件名）
        :return: submap ID，如果找不到返回 None
        """
        # 提取文件名（不含路径）
        map_filename = os.path.basename(map_file_path)

        for submap_id, submap_config in self.submaps.items():
            submap_map_file = submap_config.get('map_file', '')
            # 支持完整路径或文件名匹配
            if submap_map_file == map_file_path or submap_map_file == map_filename:
                return submap_id
            # 也支持 submap 配置中是文件名，输入是完整路径的情况
            if os.path.basename(submap_map_file) == map_filename:
                return submap_id
        return None

    # ==================== submap 级联坐标变换 ====================

    def get_submap_local_transform(self, submap_id: str, slam_map_id: str = None) -> Optional[Dict]:
        """
        获取指定 submap 相对于其父 submap 的局部坐标变换

        :param submap_id: submap ID
        :param slam_map_id: SLAM 地图 id，默认当前 SLAM 地图
        :return: transform 字典 {translation: {x, y, z}, rotation: {x, y, z, w}}
        """
        submap = self._get_submaps(slam_map_id).get(submap_id)
        if submap:
            return submap.get('transform')
        return None

    def get_submap_transform(self, submap_id: str, slam_map_id: str = None) -> Optional[Dict]:
        """
        获取指定 submap 到根节点 (default_submap) 的级联坐标变换。

        沿 parent_submap 链逐级左乘变换：T_root = T_parent_chain * ... * T_local

        :param submap_id: submap ID
        :param slam_map_id: SLAM 地图 id，默认当前 SLAM 地图
        :return: transform 字典 {translation: {x, y, z}, rotation: {x, y, z, w}}
        """
        submaps = self._get_submaps(slam_map_id)
        identity_t = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        identity_r = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}

        result_t = dict(identity_t)
        result_r = dict(identity_r)

        visited = set()
        cur = submap_id
        while cur:
            if cur in visited:
                break  # cycle guard
            visited.add(cur)
            submap_cfg = submaps.get(cur)
            if not submap_cfg:
                break
            local_tf = submap_cfg.get('transform')
            if local_tf:
                local_t = local_tf.get('translation', identity_t)
                local_r = local_tf.get('rotation', identity_r)
                # result = local * result  (先应用 result，再应用 local)
                rotated = self._rotate_point_by_quaternion(
                    {'x': result_t['x'], 'y': result_t['y'], 'z': result_t['z']}, local_r)
                result_t = {
                    'x': rotated['x'] + local_t.get('x', 0.0),
                    'y': rotated['y'] + local_t.get('y', 0.0),
                    'z': rotated['z'] + local_t.get('z', 0.0),
                }
                result_r = self._quaternion_multiply(local_r, result_r)
            cur = submap_cfg.get('parent_submap', '')
            if not cur:
                break

        return {'translation': result_t, 'rotation': result_r}

    @staticmethod
    def _quaternion_multiply(q1: Dict, q2: Dict) -> Dict:
        """
        四元数乘法: q1 * q2

        :param q1: 四元数 {x, y, z, w}
        :param q2: 四元数 {x, y, z, w}
        :return: 结果四元数 {x, y, z, w}
        """
        x1, y1, z1, w1 = q1['x'], q1['y'], q1['z'], q1['w']
        x2, y2, z2, w2 = q2['x'], q2['y'], q2['z'], q2['w']

        return {
            'x': w1*x2 + x1*w2 + y1*z2 - z1*y2,
            'y': w1*y2 - x1*z2 + y1*w2 + z1*x2,
            'z': w1*z2 + x1*y2 - y1*x2 + z1*w2,
            'w': w1*w2 - x1*x2 - y1*y2 - z1*z2
        }

    @staticmethod
    def _quaternion_inverse(q: Dict) -> Dict:
        """
        四元数求逆（对于单位四元数，逆等于共轭）

        :param q: 四元数 {x, y, z, w}
        :return: 逆四元数 {x, y, z, w}
        """
        return {
            'x': -q['x'],
            'y': -q['y'],
            'z': -q['z'],
            'w': q['w']
        }

    @staticmethod
    def _rotate_point_by_quaternion(point: Dict, q: Dict) -> Dict:
        """
        使用四元数旋转一个点: p' = q * p * q^(-1)

        :param point: 点坐标 {x, y, z}
        :param q: 四元数 {x, y, z, w}
        :return: 旋转后的点 {x, y, z}
        """
        # 将点转换为四元数形式 (x, y, z, 0)
        p = {'x': point['x'], 'y': point['y'], 'z': point['z'], 'w': 0}

        # q * p
        qp = SlamMap._quaternion_multiply(q, p)

        # q^(-1)
        q_inv = SlamMap._quaternion_inverse(q)

        # q * p * q^(-1)
        result = SlamMap._quaternion_multiply(qp, q_inv)

        return {'x': result['x'], 'y': result['y'], 'z': result['z']}

    @staticmethod
    def _apply_transform(point: Dict, translation: Dict, rotation: Dict) -> Dict:
        """
        应用变换: p' = R * p + t

        :param point: 点坐标 {x, y, z}
        :param translation: 平移 {x, y, z}
        :param rotation: 旋转四元数 {x, y, z, w}
        :return: 变换后的点 {x, y, z}
        """
        # 先旋转
        rotated = SlamMap._rotate_point_by_quaternion(point, rotation)

        # 再平移
        return {
            'x': rotated['x'] + translation['x'],
            'y': rotated['y'] + translation['y'],
            'z': rotated['z'] + translation['z']
        }

    @staticmethod
    def _apply_inverse_transform(point: Dict, translation: Dict, rotation: Dict) -> Dict:
        """
        应用逆变换: p = R^(-1) * (p' - t)

        :param point: 变换后的点坐标 {x, y, z}
        :param translation: 平移 {x, y, z}
        :param rotation: 旋转四元数 {x, y, z, w}
        :return: 原始点 {x, y, z}
        """
        # 先减去平移
        translated = {
            'x': point['x'] - translation['x'],
            'y': point['y'] - translation['y'],
            'z': point['z'] - translation['z']
        }

        # 再应用逆旋转
        q_inv = SlamMap._quaternion_inverse(rotation)
        return SlamMap._rotate_point_by_quaternion(translated, q_inv)

    def get_init_pose_after_changemap(
        self,
        current_pose: Dict,
        from_submap: str,
        to_submap: str
    ) -> Optional[Dict]:
        """
        计算切换地图后机器人在目标层的初始坐标

        根据当前层的 transform 和目标层的 transform，
        将机器人在当前层的坐标转换为目标层的坐标。

        转换逻辑 (使用完整的矩阵乘法):
        1. 当前层坐标 -> 全局坐标: pos_global = T_from * pos_current
        2. 全局坐标 -> 目标层坐标: pos_target = T_to^(-1) * pos_global

        即: pos_target = T_to^(-1) * T_from * pos_current

        对于朝向: ori_target = q_to^(-1) * q_from * ori_current

        :param current_pose: 当前位姿 {position: {x, y, z}, orientation: {x, y, z, w}}
        :param from_submap: 当前 submap ID
        :param to_submap: 目标 submap ID
        :return: 目标层的位姿 {position: {x, y, z}, orientation: {x, y, z, w}}
        """
        # 获取两个 submap 的 transform
        from_transform = self.get_submap_transform(from_submap)
        to_transform = self.get_submap_transform(to_submap)

        if not from_transform or not to_transform:
            print(f"SlamMap: 缺少 transform 配置, from={from_submap}, to={to_submap}")
            return None

        # 获取平移量
        from_trans = from_transform.get('translation', {'x': 0, 'y': 0, 'z': 0})
        to_trans = to_transform.get('translation', {'x': 0, 'y': 0, 'z': 0})

        # 获取旋转四元数
        from_rot = from_transform.get('rotation', {'x': 0, 'y': 0, 'z': 0, 'w': 1})
        to_rot = to_transform.get('rotation', {'x': 0, 'y': 0, 'z': 0, 'w': 1})

        # 当前位置和朝向
        curr_pos = current_pose.get('position', {'x': 0, 'y': 0, 'z': 0})
        curr_ori = current_pose.get('orientation', {'x': 0, 'y': 0, 'z': 0, 'w': 1})

        # Step 1: 当前层坐标 -> 全局坐标
        # pos_global = R_from * pos_current + t_from
        pos_global = self._apply_transform(curr_pos, from_trans, from_rot)

        # Step 2: 全局坐标 -> 目标层坐标
        # pos_target = R_to^(-1) * (pos_global - t_to)
        target_pos = self._apply_inverse_transform(pos_global, to_trans, to_rot)

        # 计算目标朝向: ori_target = q_to^(-1) * q_from * ori_current
        to_rot_inv = self._quaternion_inverse(to_rot)
        # q_from * ori_current
        ori_intermediate = self._quaternion_multiply(from_rot, curr_ori)
        # q_to^(-1) * (q_from * ori_current)
        target_ori = self._quaternion_multiply(to_rot_inv, ori_intermediate)

        return {
            'position': target_pos,
            'orientation': target_ori
        }

    # ==================== geo_anchor：SLAM 地图 ↔ WGS84 换算 ====================

    def get_slam_map_geo_anchor(self, slam_map_id: str = None) -> Optional[Dict]:
        """指定（默认当前）SLAM 地图的 geo_anchor（其 map 原点在 WGS84 的位姿）。"""
        slam_map_id = slam_map_id or self.current_slam_map
        cfg = self.slam_maps.get(slam_map_id)
        return cfg.get('geo_anchor') if cfg else None

    @staticmethod
    def _enu_to_lonlat(e: float, n: float, origin_lon: float, origin_lat: float) -> Tuple[float, float]:
        """局部 ENU 平面 (e 东, n 北, 米) → 经纬度（小范围球面近似，与 wgs84.py 一致）。"""
        R = 6378137.0
        lat = origin_lat + (n / R) * 180.0 / math.pi
        lon = origin_lon + (e / (R * math.cos(math.radians(origin_lat)))) * 180.0 / math.pi
        return lon, lat

    @staticmethod
    def _lonlat_to_enu(lon: float, lat: float, origin_lon: float, origin_lat: float) -> Tuple[float, float]:
        """经纬度 → 局部 ENU 平面 (e 东, n 北, 米)。"""
        R = 6378137.0
        e = math.radians(lon - origin_lon) * math.cos(math.radians(origin_lat)) * R
        n = math.radians(lat - origin_lat) * R
        return e, n

    @staticmethod
    def _yaw_to_quat(yaw: float) -> Dict:
        """绕 Z 轴的偏航角（弧度）→ 四元数 {x, y, z, w}。"""
        return {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2.0), 'w': math.cos(yaw / 2.0)}

    def slam_root_xy_to_lonlat(self, x: float, y: float,
                               slam_map_id: str = None) -> Optional[Tuple[float, float]]:
        """
        某 SLAM 地图**根系（default_submap）** map 坐标 (x, y) → 经纬度 (lon, lat)。

        (e, n) = scale · R(heading) · (x, y)，再按 anchor.origin 换算到经纬度。
        无 geo_anchor 时返回 None。
        """
        anchor = self.get_slam_map_geo_anchor(slam_map_id)
        if not anchor:
            return None
        origin = anchor.get('origin', {}) or {}
        olon, olat = origin.get('lon'), origin.get('lat')
        if olon is None or olat is None:
            return None
        h = math.radians(anchor.get('heading_deg', 0.0) or 0.0)
        scale = anchor.get('scale', 1.0) or 1.0
        e = (x * math.cos(h) - y * math.sin(h)) * scale
        n = (x * math.sin(h) + y * math.cos(h)) * scale
        return self._enu_to_lonlat(e, n, olon, olat)

    def lonlat_to_slam_root_xy(self, lon: float, lat: float,
                               slam_map_id: str = None) -> Optional[Tuple[float, float]]:
        """经纬度 (lon, lat) → 某 SLAM 地图**根系** map 坐标 (x, y)。无 geo_anchor 返回 None。"""
        anchor = self.get_slam_map_geo_anchor(slam_map_id)
        if not anchor:
            return None
        origin = anchor.get('origin', {}) or {}
        olon, olat = origin.get('lon'), origin.get('lat')
        if olon is None or olat is None:
            return None
        h = math.radians(anchor.get('heading_deg', 0.0) or 0.0)
        scale = anchor.get('scale', 1.0) or 1.0
        e, n = self._lonlat_to_enu(lon, lat, olon, olat)
        e /= scale
        n /= scale
        x = e * math.cos(h) + n * math.sin(h)
        y = -e * math.sin(h) + n * math.cos(h)
        return x, y

    def slam_point_to_lonlat(self, submap_id: str, x: float, y: float, z: float = 0.0,
                             slam_map_id: str = None) -> Optional[Tuple[float, float]]:
        """
        某 submap **局部坐标** (x, y) → 经纬度。

        链路 = submap→根系（``transform`` 级联）叠加 根系→WGS84（``geo_anchor``）。
        """
        slam_map_id = slam_map_id or self.current_slam_map
        tf = self.get_submap_transform(submap_id, slam_map_id)
        if not tf:
            return None
        root = self._apply_transform(
            {'x': x, 'y': y, 'z': z}, tf['translation'], tf['rotation'])
        return self.slam_root_xy_to_lonlat(root['x'], root['y'], slam_map_id)

    def lonlat_to_slam_root_pose(self, lon: float, lat: float, heading_deg: float = None,
                                 slam_map_id: str = None) -> Optional[Dict]:
        """
        经纬度（可选场景航向角）→ 某 SLAM 地图**根系**的初始位姿。

        用于 GNSS/OSM 段进入 SLAM 段时反算 map 系初始定位（配合 ``/pose_in_target``）。
        ``heading_deg`` 采用场景约定（与正北夹角、顺时针为正，0=北 90=东）；
        转换为 map 系偏航角：``map_yaw = (90° - heading_deg) - anchor.heading_deg``。

        :return: {position:{x,y,z}, orientation:{x,y,z,w}}，无 geo_anchor 返回 None
        """
        xy = self.lonlat_to_slam_root_xy(lon, lat, slam_map_id)
        if xy is None:
            return None
        x, y = xy
        ori = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        if heading_deg is not None:
            anchor = self.get_slam_map_geo_anchor(slam_map_id) or {}
            anchor_h = anchor.get('heading_deg', 0.0) or 0.0
            yaw_enu = math.radians(90.0 - heading_deg)          # 场景航向 → ENU 偏航(ccw from East)
            map_yaw = yaw_enu - math.radians(anchor_h)          # → SLAM 地图根系偏航
            ori = self._yaw_to_quat(map_yaw)
        return {'position': {'x': x, 'y': y, 'z': 0.0}, 'orientation': ori}
