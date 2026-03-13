# 地图语义管理类
import os
import json
from typing import Dict, List, Optional, Tuple, Any
from collections import deque


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
    
    def get_semantic_points(self, submap_id: str = None) -> Dict:
        """获取指定 submap 的所有语义点位"""
        submap_id = submap_id or self.current_submap
        if not submap_id or submap_id not in self.submaps:
            return {}
        return self.submaps[submap_id].get('semantic_points', {})
    
    def get_pose_in_submap(self, pose_name: str, submap_id: str = None) -> Optional[Dict]:
        """
        获取指定名称的位姿信息
        
        :param pose_name: 点位名称
        :param submap_id: submap ID，默认为当前 submap
        :return: 位姿字典
        """
        submap_id = submap_id or self.current_submap
        semantic_points = self.get_semantic_points(submap_id)
        point_info = semantic_points.get(pose_name)
        if point_info:
            return point_info.get('pose')
        return None
    
    def get_multi_poses_in_submap(self, pose_name: str, submap_id: str = None) -> List[Dict]:
        """
        获取指定名称的所有位姿（支持 multi_pose 类型）
        
        :param pose_name: 点位名称
        :param submap_id: submap ID，默认为当前 submap
        :return: 位姿列表
        """
        submap_id = submap_id or self.current_submap
        semantic_points = self.get_semantic_points(submap_id)
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
        semantic_points = self.get_semantic_points(submap_id)
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
    
    def get_portal_to_submap(self, target_submap: str, from_submap: str = None) -> Optional[Dict]:
        """
        获取从当前 submap 到目标 submap 的传送门
        
        :param target_submap: 目标 submap ID
        :param from_submap: 起始 submap ID，默认为当前 submap
        :return: 传送门配置 {portal, submap}
        """
        from_submap = from_submap or self.current_submap
        if not from_submap:
            return None
        
        portals = self.get_submap_portals(from_submap)
        for portal_config in portals:
            if portal_config.get('submap') == target_submap:
                return portal_config
        return None
    
    def get_portal_pose(self, portal_name: str, submap_id: str = None) -> Optional[Dict]:
        """获取传送门点位的位姿（传送门点位也是语义点位）"""
        return self.get_pose_in_submap(portal_name, submap_id)
    
    def find_path_between_submaps(self, from_submap: str, to_submap: str) -> Optional[List[Dict]]:
        """
        查找从起始 submap 到目标 submap 的路径（通过传送门）
        
        :param from_submap: 起始 submap ID
        :param to_submap: 目标 submap ID
        :return: 路径列表 [{submap, portal, next_submap}, ...]，如果无法到达返回 None
        """
        if from_submap == to_submap:
            return []
        
        # BFS 查找路径
        queue = deque([(from_submap, [])])
        visited = {from_submap}
        
        while queue:
            current_submap, path = queue.popleft()
            portals = self.get_submap_portals(current_submap)
            
            for portal_config in portals:
                next_submap = portal_config.get('submap')
                if not next_submap or next_submap in visited:
                    continue
                
                new_path = path + [{
                    'submap': current_submap,
                    'portal': portal_config.get('portal'),
                    'next_submap': next_submap
                }]
                
                if next_submap == to_submap:
                    return new_path
                
                visited.add(next_submap)
                queue.append((next_submap, new_path))
        
        return None  # 无法到达
    
    def get_all_pose_names_in_submap(self, submap_id: str = None) -> List[str]:
        """获取指定 submap 的所有点位名称"""
        semantic_points = self.get_semantic_points(submap_id)
        return list(semantic_points.keys())
    
    def get_all_pose_names_all_submaps(self) -> Dict[str, List[str]]:
        """获取所有 submap 的点位名称"""
        result = {}
        for submap_id in self.submaps:
            result[submap_id] = self.get_all_pose_names_in_submap(submap_id)
        return result


class SemanticMap:
    def __init__(self, json_path=None):
        """
        初始化SemanticMap
        :param json_path: 地图配置文件路径
        """
        if json_path is None:
            # 从config/mapuse.txt读取地图文件名
            mapuse_file = os.path.join(os.path.dirname(__file__), 'config/mapuse.txt')
            try:
                with open(mapuse_file, 'r', encoding='utf-8') as f:
                    mapuse = f.read().strip()[:10]  # 最多10个字符
                json_path = os.path.join(os.path.dirname(__file__), f'config/{mapuse}.json')
            except FileNotFoundError:
                # 如果mapuse.txt不存在，回退到默认的525.json
                json_path = os.path.join(os.path.dirname(__file__), 'config/525.json')
        print(f"Using map configuration file: {json_path}")
        self.json_path = json_path
        self.pose_map = {}
        self.load_map()

    def load_map(self):
        """
        加载预设点位地图配置，根据type字段区分single/multi类型
        - single: 单个位姿点，使用pose字段
        - multi: 多个位姿点，使用poses数组
        """
        with open(self.json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        pose_map = {}
        for k, v in data.items():
            item_type = v.get('type', 'single_pose')  # 默认为single_pose
            if item_type == 'multi_pose' and 'poses' in v:
                # multi_pose类型：保留完整的poses数组
                poses = v['poses']
                if isinstance(poses, list) and len(poses) > 0:
                    pose_map[k] = {
                        'type': 'multi_pose',
                        'pose': poses[0],  # 兼容：第一个位姿作为默认pose
                        'poses': poses     # 完整的位姿数组
                    }
                else:
                    continue
            else:
                # single_pose类型：单个位姿
                pose_map[k] = {
                    'type': 'single_pose',
                    'pose': v['pose']
                }

        self.pose_map = pose_map

    def get_pose(self, name: str):
        """
        获取指定名称的位姿信息
        :param name: 点位名称
        :return: 位姿字典，如果不存在则返回None
        """
        return self.pose_map.get(name)

    def get_all_poses(self, name: str):
        """
        获取指定名称的所有位姿（用于multi_pose类型）
        :param name: 点位名称
        :return: 位姿列表，如果是single_pose类型则返回包含单个pose的列表
        """
        item = self.pose_map.get(name)
        if not item:
            return None
        if item.get('type') == 'multi_pose':
            return item.get('poses', [])
        else:
            return [item.get('pose')] if item.get('pose') else []

    def is_multi_pose(self, name: str) -> bool:
        """
        判断指定名称是否为多位姿点
        :param name: 点位名称
        :return: True if multi_pose, False otherwise
        """
        item = self.pose_map.get(name)
        return item is not None and item.get('type') == 'multi_pose'
