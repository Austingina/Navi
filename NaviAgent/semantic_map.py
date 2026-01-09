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
        加载预设点位地图配置，兼容多种结构
        """
        with open(self.json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        pose_map = {}
        for k, v in data.items():
            if 'pose' in v:
                pose_map[k] = {'pose': v['pose']}
            elif 'poses' in v:
                if isinstance(v['poses'], list):
                    pose_map[k] = {'pose': v['poses'][0]}
                elif isinstance(v['poses'], dict):
                    pose_map[k] = {'pose': v['poses']}
        self.pose_map = pose_map
