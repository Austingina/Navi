# ROS导航节点类
import rclpy  # ROS2 Python客户端库
from rclpy.node import Node  # ROS节点基类
from rclpy.executors import SingleThreadedExecutor  # 单线程执行器
from geometry_msgs.msg import PoseStamped, Twist  # 位姿消息类型和速度控制消息类型
from nav_msgs.msg import OccupancyGrid  # 占据网格消息类型
from action_msgs.msg import GoalStatus, GoalStatusArray  # Action状态消息
import logging  # 日志记录库
import threading  # 线程处理库
import time
import math  # 数学库
# 添加tf2相关导入
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs

# 导入action相关库
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
class MCPNavNode(Node):
    """
    MCP导航节点类
    
    功能：
    1. 发布导航目标点到ROS话题
    2. 订阅里程计数据获取当前位置
    3. 检测是否到达目标点
    """
    def __init__(self):
        """初始化ROS节点和相关发布者/订阅者"""
        # 先确保rclpy已经初始化
        if not rclpy.ok():
            rclpy.init()
        
        # 调用父类构造函数，设置节点名称
        super().__init__('mcp_nav_node')

        # 初始化执行器和线程属性
        self.executor = None
        self.ros_thread = None
        # 在__init__方法中添加停止定时器属性
        self.stop_timer = None
        
        self.map_bound = {
            "xmin": -10.0,
            "xmax": 10.0,
            "ymin": -10.0,
            "ymax": 10.0
        }
        self.map_received = False  # 标记是否已接收到地图
        self.current_map = None  # 保存完整的地图消息

        try:
            # 创建navigate_to_pose action client
            self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            # 创建tf监听器替换里程计订阅
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            # 创建定时器定期获取tf变换
            self.tf_timer = self.create_timer(0.1, self.update_robot_pose)  # 10Hz更新频率

            # 创建速度控制发布者，发布到/cmd_vel_nav话题
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

            # 创建地图订阅者，用于获取地图边界
            self.map_sub = self.create_subscription(
                OccupancyGrid,
                '/map',
                self.map_callback,
                10
            )
            # 存储当前机器人位姿
            self.current_pose = None
            # 存储最后发布的目标点
            self.last_goal = None
            # 存储当前goal handle
            self.current_goal_handle = None
            # 存储导航状态 - 改为更简单的布尔状态
            self.nav_goal_reached = False
            self.nav_goal_active = False
            # PID控制状态
            self.pid_control_active = False
            self.pid_rotation_phase = True  # True为旋转阶段，False为移动阶段

            self.get_logger().info("ROS节点初始化成功")
        except Exception as e:
            self.get_logger().error(f"ROS节点初始化失败: {e}")
            raise

    def map_callback(self, msg):
        """
        地图话题回调函数 - 从地图消息中获取边界信息
        
        参数:
        msg: OccupancyGrid消息，包含地图数据和元信息
        """
        try:
            # 保存完整的地图消息
            self.current_map = msg
            
            # 获取地图元信息
            map_info = msg.info
            resolution = map_info.resolution  # 地图分辨率（米/像素）
            width = map_info.width           # 地图宽度（像素）
            height = map_info.height         # 地图高度（像素）
            origin = map_info.origin         # 地图原点在世界坐标系中的位置
            
            # 计算地图在世界坐标系中的边界
            # 原点是地图左下角的位置
            xmin = origin.position.x
            ymin = origin.position.y
            xmax = origin.position.x + width * resolution
            ymax = origin.position.y + height * resolution
            
            # 更新地图边界
            self.map_bound = {
                "xmin": xmin,
                "xmax": xmax,
                "ymin": ymin,
                "ymax": ymax
            }
            
            # 标记已接收到地图
            if not self.map_received:
                self.map_received = True
                self.get_logger().info(f"地图边界已更新:")
                self.get_logger().info(f"  X范围: [{xmin:.3f}, {xmax:.3f}]")
                self.get_logger().info(f"  Y范围: [{ymin:.3f}, {ymax:.3f}]")
                self.get_logger().info(f"  地图尺寸: {width}x{height} 像素")
                self.get_logger().info(f"  分辨率: {resolution:.3f} 米/像素")
            
        except Exception as e:
            self.get_logger().error(f"处理地图消息失败: {e}")
            import traceback
            self.get_logger().error(f"详细错误: {traceback.format_exc()}")

    def update_robot_pose(self):
        """
        定时更新机器人位姿（从tf获取）
        """
        try:
            # 获取从map到base_link的变换
            transform = self.tf_buffer.lookup_transform(
                'map',  # 目标坐标系
                'base_link',  # 源坐标系
                rclpy.time.Time(),  # 获取最新的变换
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # 将Transform转换为PoseStamped格式（兼容原有代码）
            pose_stamped = PoseStamped()
            pose_stamped.header = transform.header
            pose_stamped.header.frame_id = 'map'
            
            # 复制位置信息
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z
            
            # 复制姿态信息
            pose_stamped.pose.orientation = transform.transform.rotation
            
            # 更新当前位姿
            self.current_pose = pose_stamped
            # print(f"transform:{pose_stamped}")
            # 添加调试日志
            self.get_logger().debug(f"更新机器人位姿: x={pose_stamped.pose.position.x:.3f}, y={pose_stamped.pose.position.y:.3f}")
            
        except TransformException as ex:
            self.get_logger().debug(f"无法获取tf变换: {ex}")
        except Exception as e:
            self.get_logger().error(f"更新机器人位姿时出错: {e}")

    def goal_status_callback(self, future):
        """
        Action反馈回调函数 - 监听导航状态变化
        """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('导航目标未被接受')
                self.nav_goal_active = False
                return
            self.current_goal_handle = goal_handle
            self.get_logger().info('导航目标已被接受，开始执行...')

            # 监听结果
            goal_handle.get_result_async().add_done_callback(self._nav_result_callback)
        except Exception as e:
            self.get_logger().error(f"状态回调出错: {e}")
            import traceback
            self.get_logger().error(f"详细错误: {traceback.format_exc()}")

    def _nav_result_callback(self, future):
        try:
            result = future.result().result
            status = future.result().status
            # 状态名称映射
            status_names = {
                GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
                GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
                GoalStatus.STATUS_EXECUTING: "EXECUTING",
                GoalStatus.STATUS_CANCELING: "CANCELING",
                GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                GoalStatus.STATUS_CANCELED: "CANCELED",
                GoalStatus.STATUS_ABORTED: "ABORTED"
            }
            status_name = status_names.get(status, f"UNKNOWN({status})")
            self.get_logger().info(f"导航状态: {status_name}")
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('✓ Navigation2导航成功完成！进入PID控制状态')
                self.nav_goal_reached = True
                self.nav_goal_active = False
                self.pid_control_active = True
                self.pid_rotation_phase = True
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error('✗ Navigation2导航被中止')
                self.nav_goal_reached = False
                self.nav_goal_active = False
                self.current_goal_handle = None
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Navigation2导航被取消')
                self.nav_goal_reached = False
                self.nav_goal_active = False
                self.current_goal_handle = None
        except Exception as e:
            self.get_logger().error(f"导航结果回调出错: {e}")
            import traceback
            self.get_logger().error(f"详细错误: {traceback.format_exc()}")

    def publish_goal(self, pose: PoseStamped, target):
        """
        通过navigate_to_pose action client发布导航目标点
        """
        try:
            # 重置状态
            self.nav_goal_reached = False
            self.nav_goal_active = True
            self.pid_control_active = False
            self.current_goal_handle = None

            # 构造action goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose

            # 记录最后发布的目标点
            self.last_goal = pose

            self.get_logger().info(f"发布目标点:name={target}, x={pose.pose.position.x}, y={pose.pose.position.y}")
            self.get_logger().info("等待导航服务器可用...")
            self.nav_action_client.wait_for_server()
            send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.goal_status_callback)
            return True
        except Exception as e:
            self.get_logger().error(f"发布目标点失败: {e}")
            return False

    def pause_navigation(self):
        """暂停导航（通过cancel当前goal实现）"""
        if self.current_goal_handle:
            self.get_logger().info("暂停导航...")
            future = self.current_goal_handle.cancel_goal_async()
            future.add_done_callback(lambda f: self.get_logger().info("导航已暂停（已取消当前目标）"))
        else:
            self.get_logger().warn("没有活跃的导航目标可暂停")

    def resume_navigation(self):
        """继续导航（重新发送上一个目标）"""
        if self.last_goal:
            self.get_logger().info("继续导航...")
            self.publish_goal(self.last_goal, target="resume")
        else:
            self.get_logger().warn("没有可继续的目标")

    def cancel_navigation(self):
        """取消导航"""
        if self.current_goal_handle:
            self.get_logger().info("取消导航...")
            future = self.current_goal_handle.cancel_goal_async()
            future.add_done_callback(lambda f: self.get_logger().info("导航已取消"))
        else:
            self.get_logger().warn("没有活跃的导航目标可取消")

    def quaternion_to_euler(self, x, y, z, w):
        """
        将四元数转换为欧拉角
        
        参数:
        x, y, z, w: 四元数分量
        
        返回:
        yaw: 偏航角（弧度）
        """
        # 计算yaw角度
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def normalize_angle(self, angle):
        """
        将角度标准化到[-pi, pi]范围
        
        参数:
        angle: 输入角度（弧度）
        
        返回:
        标准化后的角度
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def pid_control_step(self, yaw_tolerance=0.1, tolerance=0.5):
        """
        执行一步PID控制
        
        参数:
        yaw_tolerance: yaw角度容差（弧度）
        tolerance: 距离容差（米）
        
        返回:
        bool: True表示已到达目标，False表示仍在调整
        """
        timestamp = self.get_clock().now().to_msg()

        if not (self.current_pose and self.last_goal):
            return False

        # 计算位置误差
        dx = self.last_goal.pose.position.x - self.current_pose.pose.position.x
        dy = self.last_goal.pose.position.y - self.current_pose.pose.position.y
        dist = (dx**2 + dy**2) ** 0.5

        if(dist > 3.0):
            self.get_logger().warn(f"当前位置与目标点距离过大，跳过PID控制步骤: dist={dist}")
            return False
        # 计算当前yaw角和目标yaw角
        current_yaw = self.quaternion_to_euler(
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        )
        target_yaw = self.quaternion_to_euler(
            self.last_goal.pose.orientation.x,
            self.last_goal.pose.orientation.y,
            self.last_goal.pose.orientation.z,
            self.last_goal.pose.orientation.w
        )
        yaw_error = self.normalize_angle(target_yaw - current_yaw)

        # 创建速度控制消息
        cmd_vel = Twist()

        if self.pid_rotation_phase:
            # 旋转阶段：调整yaw角度
            if abs(yaw_error) < yaw_tolerance:
                print(f"current quaternion: {self.current_pose.pose.orientation}")
                print(f"yaw角度调整完成: {current_yaw} -> {target_yaw}, error={yaw_error}, tolerance={yaw_tolerance}")
                # yaw角度达到要求，切换到移动阶段
                self.pid_rotation_phase = False
                self.get_logger().info("yaw角度调整完成，切换到移动阶段")
                cmd_vel.angular.z = 0.0
            else:
                # 继续调整yaw角度
                if( abs(yaw_error) > 0.1):
                    kp_yaw = 0.9  # yaw角度的比例控制系数
                else:
                    kp_yaw = 1.2  # yaw角度的比例控制系数
                cmd_vel.angular.z = kp_yaw * yaw_error
                # 限制角速度
                if cmd_vel.angular.z > 0.5:
                    cmd_vel.angular.z = 0.5
                elif cmd_vel.angular.z < -0.5:
                    cmd_vel.angular.z = -0.5
                print(f"yaw_error={yaw_error},cmd={cmd_vel.angular.z},ts={timestamp.sec}.{timestamp.nanosec}")
        else:
            # 移动阶段：调整xy位置
            if dist < tolerance:
                # 位置达到要求，完成调整
                print(f"位置调整完成: x={self.current_pose.pose.position.x}, y={self.current_pose.pose.position.y}, dist={dist}, tolerance={tolerance}")
                self.pid_control_active = False
                self.get_logger().info("位置调整完成，到达目标")
                cmd_vel.linear.x = 0.0
                cmd_vel.linear.y = 0.0
                self.cmd_vel_pub.publish(cmd_vel)
                return True
            else:
                # 继续调整位置
                kp_linear = 0.7  # 线性速度的比例控制系数
                x_local = math.cos(current_yaw) * dx + math.sin(current_yaw) * dy
                y_local = -math.sin(current_yaw) * dx + math.cos(current_yaw) * dy
                cmd_vel.linear.x = kp_linear * x_local
                cmd_vel.linear.y = kp_linear * y_local
                print(f"dx={dx},dy={dy},xlocal={x_local},ylocal={y_local},cmdx={cmd_vel.linear.x},cmdy={cmd_vel.linear.y}")
                # 限制线性速度
                max_vel = 0.3
                if abs(cmd_vel.linear.x) > max_vel:
                    cmd_vel.linear.x = max_vel if cmd_vel.linear.x > 0 else -max_vel
                if abs(cmd_vel.linear.y) > max_vel:
                    cmd_vel.linear.y = max_vel if cmd_vel.linear.y > 0 else -max_vel

        # 发布速度控制指令
        self.cmd_vel_pub.publish(cmd_vel)

        # # 设置10ms后停止的定时器
        # if self.stop_timer is not None:
        #     self.stop_timer.cancel()  # 取消之前的定时器
            
        # self.stop_timer = self.create_timer(0.1, self.stop_robot_once)  # 10ms = 0.01s
            
        return False

    def stop_robot_once(self):
        """发送停止命令并取消定时器"""
        stop_cmd = Twist()  # 所有速度都为0
        self.cmd_vel_pub.publish(stop_cmd)
        
        # 取消定时器，防止重复执行
        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None
            
        self.get_logger().debug("机器人已停止（10ms后）")

    def check_goal_reached(self, tolerance=0.5, yaw_tolerance=0.1):
        """
        检查是否到达目标点
        
        参数:
        tolerance: 到达判断的距离阈值（米）
        yaw_tolerance: yaw角度容差（弧度），默认0.1
        
        返回:
        bool: True表示已到达目标点，False表示未到达
        """
        # 确保有当前位置数据和目标点数据
        if not (self.current_pose and self.last_goal):
            return False

        # 如果已经在PID控制状态中，执行PID控制步骤
        if self.pid_control_active:
            print("pid_control_step")
            return self.pid_control_step(yaw_tolerance, tolerance)

        # 计算当前位置与目标点的距离误差
        dx = self.last_goal.pose.position.x - self.current_pose.pose.position.x
        dy = self.last_goal.pose.position.y - self.current_pose.pose.position.y
        dist = (dx**2 + dy**2) ** 0.5

        # 计算当前yaw角和目标yaw角
        current_yaw = self.quaternion_to_euler(
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        )
        target_yaw = self.quaternion_to_euler(
            self.last_goal.pose.orientation.x,
            self.last_goal.pose.orientation.y,
            self.last_goal.pose.orientation.z,
            self.last_goal.pose.orientation.w
        )
        yaw_error = self.normalize_angle(target_yaw - current_yaw)

        # 检查是否同时满足距离和角度要求
        if dist < tolerance and abs(yaw_error) < yaw_tolerance:
            self.get_logger().info(f"目标已到达 - 距离: {dist:.3f}m, yaw误差: {abs(yaw_error):.3f}rad")
            return True

        # 检查Navigation2是否成功但精度不够
        if self.nav_goal_reached and not self.pid_control_active:
            self.get_logger().info("Navigation2已到达目标但精度不够，进入PID调整状态")
            self.pid_control_active = True
            self.pid_rotation_phase = True
            return False
            
        # 如果目标不活跃且没有到达，说明导航失败了
        if not self.nav_goal_active and not self.nav_goal_reached:
            self.get_logger().debug("导航目标不活跃且未到达")
            return False
            
        # 导航仍在进行中
        return False