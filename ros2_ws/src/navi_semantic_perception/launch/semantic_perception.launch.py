"""草坪语义感知链：深度->彩色网格组织化点云 + 语义分割掩码 + LabelInfo。

三个话题一起喂给 nav2 的 semantic_segmentation_layer：
    /semantic/camera1/points        组织化点云（彩色网格，NaN 占位）
    /semantic/camera1/segmentation  mono8 类别掩码（像素与点云严格 1:1）
    /semantic/camera1/label_info    类别表（latched，由 segmentation_trt 自己发）

另有一个只给人看的话题（下游不消费）：
    /semantic/camera1/segmentation_color   rgb8 叠加图，RELIABLE
    上面那个 mono8 掩码的像素值就是类别 id(0/1/2)，在 RViz 里是全黑的；
    而且它走 SensorDataQoS(BEST_EFFORT)，RViz 的 Image display 默认 Reliable 收不到。
    要在 RViz 里看就看这个 _color 话题。

启动顺序：LabelInfo 必须在那一层收到第一帧之前到位
（semantic_segmentation_layer.cpp:511-516），所以本 launch 要在 nav2 之前起。

segmentation 参数：
    trt  (默认) 真模型，TensorRT FP16，640x480 原生 2.3 ms/帧。
                它自己按 config/ontology.yaml 发 LabelInfo。
    none        只发点云（调试点云本身、或不想烧 GPU 时用）。
                此时【没有】掩码和 LabelInfo，那一层无法工作。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
                              choices=['trt', 'none']),
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
    ])
