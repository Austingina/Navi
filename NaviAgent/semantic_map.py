# 地图语义管理类
import os
import json

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
