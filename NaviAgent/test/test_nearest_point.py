#!/usr/bin/env python3
import json
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time

# 复用server.py中的load_intent_pose_map函数
def load_intent_pose_map():
    json_path = os.path.join(os.path.dirname(__file__), 'intent_pose_map.json')
    if not os.path.exists(json_path):
        print(f"❌ 找不到文件: {json_path}")
        return {}
    
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    pose_map = {}
    for k, v in data.items():
        if 'pose' in v:
            pose_map[k] = {'pose': v['pose']}
        elif 'poses' in v:
            if isinstance(v['poses'], list):
                # 取第一个点为默认
                pose_map[k] = {'pose': v['poses'][0]}
            elif isinstance(v['poses'], dict):
                pose_map[k] = {'pose': v['poses']}
    return pose_map

class TestNode(Node):
    def __init__(self):
        super().__init__('test_nearest_point_node')
        self.odom_sub = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)
        self.current_odom = None
        self.pose_map = load_intent_pose_map()
        self.get_logger().info("测试节点初始化完成")
        
    def odom_callback(self, msg):
        self.current_odom = msg
        
    def get_nearest_point(self):
        """获取当前位置最接近的预设点"""
        try:
            if not self.current_odom:
                return {"success": False, "msg": "无实时位姿数据"}
            
            current_x = self.current_odom.pose.pose.position.x
            current_y = self.current_odom.pose.pose.position.y
            
            nearest_point = None
            min_distance = float('inf')
            distances = {}
            
            # 遍历所有预设点计算距离
            for point_name, point_data in self.pose_map.items():
                pose_info = point_data.get("pose")
                if not pose_info:
                    continue
                
                target_x = pose_info["position"]["x"]
                target_y = pose_info["position"]["y"]
                
                # 计算欧几里得距离
                distance = ((current_x - target_x) ** 2 + (current_y - target_y) ** 2) ** 0.5
                distances[point_name] = distance
                
                if distance < min_distance:
                    min_distance = distance
                    nearest_point = point_name
            
            if nearest_point is None:
                return {"success": False, "msg": "没有找到有效的预设点"}
            
            return {
                "success": True,
                "nearest_point": nearest_point,
                "distance": round(min_distance, 3),
                "current_position": {
                    "x": round(current_x, 3),
                    "y": round(current_y, 3)
                },
                "all_distances": {k: round(v, 3) for k, v in distances.items()},
                "msg": f"当前位置最接近的预设点是: {nearest_point} (距离: {min_distance:.3f}m)"
            }
        except Exception as e:
            return {"success": False, "msg": f"获取最近点失败: {str(e)}"}

def main():
    try:
        print("🚀 启动获取最近点测试...")
        
        # 初始化ROS
        rclpy.init()
        test_node = TestNode()
        
        print("📍 预设点列表:")
        for name in test_node.pose_map.keys():
            pose = test_node.pose_map[name]['pose']
            x = pose['position']['x']
            y = pose['position']['y']
            print(f"  - {name}: ({x:.3f}, {y:.3f})")
        
        print("\n⏳ 等待接收Odometry数据...")
        
        # 等待接收到odometry数据
        timeout = 10  # 10秒超时
        start_time = time.time()
        
        while rclpy.ok() and test_node.current_odom is None:
            rclpy.spin_once(test_node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                print("❌ 超时：10秒内未接收到Odometry数据")
                print("💡 请检查：")
                print("   1. ROS是否正常运行？")
                print("   2. /Odometry话题是否在发布数据？")
                print("   3. 可以用 'ros2 topic echo /Odometry' 验证")
                return
        
        if test_node.current_odom:
            print("✅ 已接收到Odometry数据")
            
            # 测试获取最近点功能
            result = test_node.get_nearest_point()
            
            print("\n🎯 测试结果:")
            print("=" * 50)
            if result["success"]:
                print(f"✅ {result['msg']}")
                print(f"📍 当前位置: ({result['current_position']['x']}, {result['current_position']['y']})")
                print(f"🎯 最近的预设点: {result['nearest_point']}")
                print(f"📏 距离: {result['distance']} 米")
                print("\n📊 所有距离:")
                for point, dist in result['all_distances'].items():
                    mark = "🎯" if point == result['nearest_point'] else "📍"
                    print(f"  {mark} {point}: {dist} 米")
            else:
                print(f"❌ 测试失败: {result['msg']}")
            print("=" * 50)
        
    except KeyboardInterrupt:
        print("\n👋 测试被用户中断")
    except Exception as e:
        print(f"❌ 测试过程中出错: {e}")
    finally:
        try:
            test_node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == "__main__":
    main() 