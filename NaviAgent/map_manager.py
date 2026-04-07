# 地图语义管理类
import os
import json
import math
from typing import Dict, List, Optional, Tuple, Any


class MapManager:
    """
    多楼层地图管理器
    
    功能：
    1. 加载多楼层地图配置文件 (multifloor.json 格式)
    2. 管理多个 submap 的语义点位
    3. 支持跨 submap 的 pose 查询
    4. 提供 portal 导航接口
    """
    
    def __init__(self, map_config_path: str = None):
        """
        初始化地图管理器
        
        :param map_config_path: 地图配置文件路径 (multifloor.json)
        """
        self.config_path = map_config_path
        self.config = {}
        self.submaps = {}  # submap配置 {submap_id: submap_config}
        self.current_submap = None  # 当前 submap ID
        self.default_submap = None  # 默认 submap ID
        
        if map_config_path:
            self.load_config(map_config_path)
    
    def load_config(self, config_path: str):
        """
        加载地图配置文件
        
        :param config_path: 配置文件路径
        """
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"地图配置文件不存在: {config_path}")
        
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = json.load(f)
        
        # 解析全局配置
        global_config = self.config.get('global', {})
        self.default_submap = global_config.get('default_submap', None)
        
        # 解析 submap 配置
        self.submaps = self.config.get('submaps', {})
        
        # 设置当前 submap 为默认 submap
        if self.default_submap and self.default_submap in self.submaps:
            self.current_submap = self.default_submap
        elif self.submaps:
            self.current_submap = list(self.submaps.keys())[0]
        
        print(f"MapManager: 已加载地图配置: {config_path}, submap数: {len(self.submaps)}, 当前submap: {self.current_submap}")
    
    def get_submap_list(self) -> List[str]:
        """获取所有 submap ID 列表"""
        return list(self.submaps.keys())
    
    def get_submap_info(self, submap_id: str) -> Optional[Dict]:
        """获取指定 submap 的信息"""
        return self.submaps.get(submap_id)
    
    def get_submap_map_file(self, submap_id: str) -> Optional[str]:
        """获取指定 submap 的地图文件名"""
        submap = self.submaps.get(submap_id)
        if submap:
            return submap.get('map_file')
        return None
    
    def get_submap_full_map_path(self, submap_id: str) -> Optional[str]:
        """
        获取指定 submap 的完整地图文件路径
        
        如果 multifloor.json 中的 map_file 是相对路径，
        则根据 multifloor.json 所在目录自动补齐成完整路径
        
        :param submap_id: submap ID
        :return: 完整的地图文件路径，如果 submap 不存在返回 None
        """
        map_file = self.get_submap_map_file(submap_id)
        if not map_file:
            return None
        
        # 如果已经是绝对路径，直接返回
        if os.path.isabs(map_file):
            return map_file
        
        # 相对路径：根据配置文件所在目录补齐
        if self.config_path:
            config_dir = os.path.dirname(os.path.abspath(self.config_path))
            return os.path.join(config_dir, map_file)
        
        return map_file
    
    def set_current_submap(self, submap_id: str) -> bool:
        """设置当前 submap"""
        if submap_id in self.submaps:
            self.current_submap = submap_id
            print(f"MapManager: 当前submap已切换为: {submap_id}")
            return True
        print(f"MapManager: submap不存在: {submap_id}")
        return False
    
    def get_current_submap(self) -> Optional[str]:
        """获取当前 submap ID"""
        return self.current_submap
    
    def get_semantic_points_in_submap(self, submap_id: str = None) -> Dict:
        """获取指定 submap 的所有语义点位"""
        submap_id = submap_id or self.current_submap
        if not submap_id or submap_id not in self.submaps:
            return {}
        return self.submaps[submap_id].get('semantic_points', {})
    
    def get_pose_in_submap(self, pose_name: str, submap_id: str = None) -> Optional[Dict]:
        """
        获取指定名称的点位信息（返回整个 point_info）
        
        :param pose_name: 点位名称
        :param submap_id: submap ID，默认为当前 submap
        :return: point_info 字典（包含 `type`, `pose` 等），找不到返回 None

        point_info 中 `type` 字段说明：
        - "single_pose": 单一位姿点；使用普通导航接口 `set_goal_by_name` 下发目标。
        - "multi_pose": 多位姿点；需使用 `set_goal_by_multipose` 来依次导航到各个位姿。
        - "elevator_call": 电梯呼叫点；机器人到达该点后应等待外部 agent 执行呼叫电梯的动作。
        - "elevator_wait": 电梯等待点；机器人到达该点后应等待外部 agent 确认电梯已到达且车门已打开。
        - "elevator_in": 进梯后/新楼层落位点；`wait_goal_reached` 在到达后轮询 map 定位（`get_status.localized`），就绪即继续。
        """
        submap_id = submap_id or self.current_submap
        semantic_points = self.get_semantic_points_in_submap(submap_id)
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
        """获取传送门点位的位姿（传送门点位也是语义点位）"""
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
    
    # ========== 路线管理接口 ==========
    
    def get_route_list(self) -> List[Dict[str, str]]:
        """
        获取所有路线列表
        
        :return: 路线列表，每个元素包含 {name, description}
        """
        routes = self.config.get('routes', {})
        result = []
        for route_name, route_config in routes.items():
            result.append({
                'name': route_name,
                'description': route_config.get('description', '')
            })
        return result
    
    def get_route_by_name(self, route_name: str) -> Optional[Dict]:
        """
        根据路线名称获取路线详细信息
        
        :param route_name: 路线名称
        :return: 路线配置，包含 description 和 waypoints
                 waypoints 格式: [{submap, semantic_point}, ...]
                 如果路线不存在返回 None
        """
        routes = self.config.get('routes', {})
        route_config = routes.get(route_name)
        if not route_config:
            return None
        return {
            'name': route_name,
            'description': route_config.get('description', ''),
            'waypoints': route_config.get('waypoints', [])
        }
    
    def get_route_names(self) -> List[str]:
        """
        获取所有路线名称
        
        :return: 路线名称列表
        """
        return list(self.config.get('routes', {}).keys())

    def get_submap_id_by_map_file(self, map_file_path: str) -> Optional[str]:
        """
        根据地图文件路径获取 submap ID
        
        :param map_file_path: 地图文件路径（可以是完整路径或文件名）
        :return: submap ID，如果找不到返回 None
        """
        import os
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

    def get_submap_transform(self, submap_id: str) -> Optional[Dict]:
        """
        获取指定 submap 相对于 default submap 的坐标变换
        
        :param submap_id: submap ID
        :return: transform 字典 {translation: {x, y, z}, rotation: {x, y, z, w}}
        """
        submap = self.submaps.get(submap_id)
        if submap:
            return submap.get('transform')
        return None

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
        qp = MapManager._quaternion_multiply(q, p)
        
        # q^(-1)
        q_inv = MapManager._quaternion_inverse(q)
        
        # q * p * q^(-1)
        result = MapManager._quaternion_multiply(qp, q_inv)
        
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
        rotated = MapManager._rotate_point_by_quaternion(point, rotation)
        
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
        q_inv = MapManager._quaternion_inverse(rotation)
        return MapManager._rotate_point_by_quaternion(translated, q_inv)

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
            print(f"MapManager: 缺少 transform 配置, from={from_submap}, to={to_submap}")
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
