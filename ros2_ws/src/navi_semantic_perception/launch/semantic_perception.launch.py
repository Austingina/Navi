"""Color-grid depth cloud + SAM3 lawn masks; all public outputs share one stamp."""

import os
import shlex
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from navi_semantic_perception.sam3_paths import checkpoint_directory, runtime_python


def _camera_profile(sim, camera_ns):
    """真机 RealSense 的话题全部由 camera_ns 派生；Isaac 的 ZED 每一项都对不上，单列一份。

    差异不止命名：Isaac 只发一个 /zed/camera_info（深度和彩色是同一个虚拟相机渲染的，
    所以 depth->color 外参恒为单位阵 —— extrinsics_source 用 tf 查同名帧 zed_camera
    就返回单位阵，且仿真里本来也没有 RealSense 的工厂 Extrinsics 可订）；深度是 32FC1
    而不是 16UC1；分辨率是 640x360 而不是 640x480。
    """
    if not sim:
        return {
            'color_topic': f'{camera_ns}/color/image_raw',
            'depth_topic': f'{camera_ns}/depth/image_rect_raw',
            'color_info_topic': f'{camera_ns}/color/camera_info',
            'depth_info_topic': f'{camera_ns}/depth/camera_info',
            'output_frame': 'camera1_color_optical_frame',
            'color_size': [640, 480],
            'stamp_source': 'depth',
            'max_pair_dt': 0.05,
        }
    return {
        'color_topic': '/zed/rgb/image_raw',
        'depth_topic': '/zed/depth/image_rect_raw',
        'color_info_topic': '/zed/camera_info',
        'depth_info_topic': '/zed/camera_info',
        'output_frame': 'zed_camera',
        'color_size': [640, 360],
        # 深度戳用的是另一个纪元，必须重打戳，详见 depth_to_color_cloud 的 stamp_source。
        'stamp_source': 'clock',
        # 重打戳后点云戳是"处理时刻"，与彩色戳差 10~60 ms（/clock 20 Hz 量化 + 渲染延迟），
        # 0.05 s 会卡边。仿真相机 4.6 Hz（217 ms 一帧），0.12 s 足够配上同一帧又够不到邻帧。
        'max_pair_dt': 0.12,
    }


def _nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    backend = value('segmentation')
    share = Path(get_package_share_directory('navi_semantic_perception'))
    camera_ns = value('camera_ns').rstrip('/')
    use_sim_time = value('use_sim_time') == 'true'
    sim = value('sim') == 'true'
    camera = _camera_profile(sim, camera_ns)
    extrinsics_source = 'tf' if sim else value('extrinsics_source')
    cloud_topic = ('/semantic/camera1/points_raw' if backend in ('sam3', 'sam3_trt')
                   else '/semantic/camera1/points')
    nodes = [Node(
        package='navi_semantic_perception', executable='depth_to_color_cloud',
        name='depth_to_color_cloud', output='screen', parameters=[{
            'camera_ns': camera_ns,
            'depth_topic': camera['depth_topic'],
            'depth_info_topic': camera['depth_info_topic'],
            'color_info_topic': camera['color_info_topic'],
            'output_frame': camera['output_frame'],
            'assert_color_size': camera['color_size'],
            'stamp_source': camera['stamp_source'],
            'target_rate': float(value('target_rate')),
            'extrinsics_source': extrinsics_source,
            'color_distortion_model': value('color_distortion_model'),
            'output_topic': cloud_topic,
            'use_sim_time': use_sim_time,
        }],
    )]
    if backend in ('sam3', 'sam3_trt'):
        python = (Path(value('sam3_python')).expanduser() if value('sam3_python')
                  else runtime_python(share))
        if not python.is_file() or not os.access(python, os.X_OK):
            raise RuntimeError(
                f'SAM3 Python not executable: {python}; run bash envs/sam3/install.sh '
                'from the Navi checkout, or set sam3_python')
        nodes.append(Node(
            package='navi_semantic_perception', executable='segmentation_sam3',
            name='segmentation_sam3', output='screen', prefix=shlex.quote(str(python)),
            parameters=[str(share / 'config/sam3.yaml'), {
                'backend': 'tensorrt' if backend == 'sam3_trt' else 'pytorch',
                'model_path': (value('sam3_engine_dir') if backend == 'sam3_trt'
                               else value('sam3_model_path')),
                'device': value('sam3_device'),
                'color_topic': camera['color_topic'],
                'color_width': camera['color_size'][0],
                'color_height': camera['color_size'][1],
                'max_pair_dt': camera['max_pair_dt'],
                'cloud_topic': cloud_topic,
                'use_sim_time': use_sim_time,
            }],
        ))
    elif backend == 'trt':
            nodes.append(Node(
            package='navi_semantic_perception', executable='segmentation_trt',
            name='segmentation_trt', output='screen', parameters=[{
                'color_topic': camera['color_topic'],
                'input_scale': float(value('input_scale')),
                'min_mean_brightness': float(value('min_mean_brightness')),
                'use_sim_time': use_sim_time,
            }],
        ))
    return nodes


def generate_launch_description():
    """Select SAM3 TensorRT by default, using the repository SAM3 environment."""
    return LaunchDescription([
        DeclareLaunchArgument('camera_ns', default_value='/camera1/camera1'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        # sim 与 use_sim_time 分开，同 outdoor_localization.launch.py：use_sim_time 只管
        # 时钟源，sim 管相机硬件 profile（话题名 / 分辨率 / 深度编码 / 外参来源）。
        DeclareLaunchArgument('sim', default_value='false', choices=['true', 'false'],
                              description='Isaac ZED camera profile instead of RealSense; '
                                          'independent of use_sim_time'),
        DeclareLaunchArgument('segmentation', default_value='sam3_trt',
                              choices=['sam3_trt', 'sam3', 'trt', 'none']),
        DeclareLaunchArgument('target_rate', default_value='3.0',
                              description='Input cloud rate; busy SAM3 skips older frames'),
        DeclareLaunchArgument('sam3_model_path', default_value=str(checkpoint_directory()),
                              description='External directory containing model.safetensors'),
        DeclareLaunchArgument('sam3_engine_dir',
                              default_value=str(checkpoint_directory() / 'trt_engines'),
                              description='Local SAM3 TensorRT engine directory'),
        DeclareLaunchArgument('sam3_python', default_value='',
                              description='Default: <Navi>/envs/sam3/.venv/bin/python'),
        DeclareLaunchArgument('sam3_device', default_value='cuda',
                              description='cuda[:index] or cpu; CPU needs larger max_result_age'),
        DeclareLaunchArgument('extrinsics_source', default_value='topic',
                              choices=['topic', 'tf']),
        DeclareLaunchArgument('color_distortion_model', default_value='none'),
        DeclareLaunchArgument('input_scale', default_value='1.0', description='Legacy TRT only'),
        DeclareLaunchArgument('min_mean_brightness', default_value='100.0',
                              description='Legacy TRT dark-scene filter; unused for SAM3'),
        OpaqueFunction(function=_nodes),
    ])
