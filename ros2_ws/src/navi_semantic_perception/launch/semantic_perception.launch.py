"""草坪语义感知链：深度->彩色网格组织化点云 + 语义分割掩码 + LabelInfo。

三个话题一起喂给 nav2 的 semantic_segmentation_layer：
    /semantic/camera1/points        组织化点云（彩色网格，NaN 占位）
    /semantic/camera1/segmentation  mono8 类别掩码（像素与点云严格 1:1）
    /semantic/camera1/label_info    类别表（latched）

启动顺序：LabelInfo 必须在那一层收到第一帧之前到位
（semantic_segmentation_layer.cpp:511-516），所以本 launch 要在 nav2 之前起。

segmentation 参数：
    trt  (默认) 真模型，TensorRT FP16，640x480 原生 2.3 ms/帧。
                它自己按 config/ontology.yaml 发 LabelInfo，
                所以【不】再起 label_info_publisher（否则同一 latched 话题两个发布者）。
    stub        固定图像区域打标，用来在没有模型时打通全链。
                stub 不发 LabelInfo，所以这条路径要起 label_info_publisher。
    none        只发点云（调试点云本身时用）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('navi_semantic_perception')
    seg = LaunchConfiguration('segmentation')
    return LaunchDescription([
        DeclareLaunchArgument('camera_ns', default_value='/camera1/camera1'),
        DeclareLaunchArgument('target_rate', default_value='10.0',
                              description='点云与掩码的共同速率；必须一致，否则那一层'
                                          '硬编码的 100 ms 容差会开始丢对'),
        DeclareLaunchArgument('extrinsics_source', default_value='topic',
                              description="'topic'(工厂标定 Extrinsics) 或 "
                                          "'tf'(driver 的 static TF；bag 回放缺 "
                                          "extrinsics 时用这个，两者实测差约 10 um)"),
        DeclareLaunchArgument('color_distortion_model', default_value='none'),
        DeclareLaunchArgument('segmentation', default_value='trt',
                              choices=['trt', 'stub', 'none']),
        DeclareLaunchArgument('input_scale', default_value='1.0',
                              description='1.0=原生 640x480（掩码天生落在彩色网格上，'
                                          '不做任何缩放）。GPU 有 43 倍余量，别调小'),
        DeclareLaunchArgument('min_mean_brightness', default_value='100.0',
                              description='近场条带平均亮度低于此值则发全 background '
                                          '掩码。暗湿铺装会被整片误判成草坪，实测'
                                          '相关系数 r=-0.850；0 表示关闭该 fail-safe'),

        Node(package='navi_semantic_perception', executable='depth_to_color_cloud',
             name='depth_to_color_cloud', output='screen',
             parameters=[{'camera_ns': LaunchConfiguration('camera_ns'),
                          'target_rate': LaunchConfiguration('target_rate'),
                          'extrinsics_source': LaunchConfiguration('extrinsics_source'),
                          'color_distortion_model':
                              LaunchConfiguration('color_distortion_model')}]),

        Node(package='navi_semantic_perception', executable='segmentation_trt',
             name='segmentation_trt', output='screen',
             condition=IfCondition(EqualsSubstitution(seg, 'trt')),
             parameters=[{'input_scale': LaunchConfiguration('input_scale'),
                          'min_mean_brightness':
                              LaunchConfiguration('min_mean_brightness')}]),

        # 只有 stub 路径需要外部发 LabelInfo（segmentation_trt 自己发）
        Node(package='navi_semantic_perception', executable='label_info_publisher',
             name='label_info_publisher', output='screen',
             condition=IfCondition(EqualsSubstitution(seg, 'stub')),
             parameters=[os.path.join(share, 'config', 'lawn_labels.yaml')]),
        Node(package='navi_semantic_perception', executable='stub_segmentation',
             name='stub_segmentation', output='screen',
             condition=IfCondition(EqualsSubstitution(seg, 'stub'))),
    ])
