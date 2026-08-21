"""启动多雷达点云合并节点。

阶段 1 用法(只加不改,不影响正在运行的导航栈):
    ros2 launch livox_multi_merger merger.launch.py

合并结果发到 /livox/fused_lidar,而 FAST-LIO 此时仍在吃 /livox/front_lidar,
因此可以在真机上反复调标定、看 RViz,全程不干扰现有链路。

可选:同时发布 primary->其它雷达 的 static TF,仅供 RViz 调试用,不参与数据流
(点云已在 merger 内部完成变换)。
    ros2 launch livox_multi_merger merger.launch.py publish_debug_tf:=true
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_extrinsics(config_path):
    """读出 yaml 里的 ros__parameters 段,供 static TF 复用同一份外参。

    外参的单一真源是 config/lidar_extrinsics.yaml。这里只是把它再读一遍用于生成调试
    TF,避免在 launch 里硬编码第二份数值。
    """
    with open(config_path, "r") as f:
        doc = yaml.safe_load(f)
    for key in ("/**", "livox_merger"):
        if key in doc and "ros__parameters" in doc[key]:
            return doc[key]["ros__parameters"]
    raise RuntimeError(f"{config_path} 里找不到 ros__parameters 段")


def _setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("config_file").perform(context)
    publish_debug_tf = LaunchConfiguration("publish_debug_tf").perform(context).lower() == "true"
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"

    nodes = [
        Node(
            package="livox_multi_merger",
            executable="livox_merger",
            name="livox_merger",
            output="screen",
            parameters=[config_path, {"use_sim_time": use_sim_time}],
        )
    ]

    if not publish_debug_tf:
        return nodes

    params = _load_extrinsics(config_path)
    primary = params.get("primary", "front")
    output_frame = params.get("output_frame", "livox_frame")

    for name in params.get("lidars", []):
        if name == primary:
            continue
        xyz_rpy = params.get(name, {}).get("xyz_rpy")
        if not xyz_rpy or len(xyz_rpy) != 6:
            continue
        x, y, z, roll, pitch, yaw = (str(v) for v in xyz_rpy)
        # static_transform_publisher 的参数顺序是 x y z yaw pitch roll,
        # 与 yaml 里 [x y z roll pitch yaw] 的顺序不同 —— 注意别写串。
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"tf_{output_frame}_to_{name}_lidar",
                output="screen",
                arguments=[x, y, z, yaw, pitch, roll, output_frame, f"{name}_lidar_frame"],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )

    return nodes


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("livox_multi_merger"), "config", "lidar_extrinsics.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="多雷达外参与合并参数的 yaml 路径",
            ),
            DeclareLaunchArgument(
                "publish_debug_tf",
                default_value="false",
                description="是否发布 primary->各雷达 的 static TF(仅 RViz 调试用,不参与数据流)",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
