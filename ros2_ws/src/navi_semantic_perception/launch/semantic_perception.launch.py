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


def _nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    backend = value('segmentation')
    share = Path(get_package_share_directory('navi_semantic_perception'))
    camera_ns = value('camera_ns').rstrip('/')
    use_sim_time = value('use_sim_time') == 'true'
    cloud_topic = ('/semantic/camera1/points_raw' if backend in ('sam3', 'sam3_trt')
                   else '/semantic/camera1/points')
    nodes = [Node(
        package='navi_semantic_perception', executable='depth_to_color_cloud',
        name='depth_to_color_cloud', output='screen', parameters=[{
            'camera_ns': camera_ns,
            'target_rate': float(value('target_rate')),
            'extrinsics_source': value('extrinsics_source'),
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
                'color_topic': f'{camera_ns}/color/image_raw',
                'cloud_topic': cloud_topic,
                'use_sim_time': use_sim_time,
            }],
        ))
    elif backend == 'trt':
        nodes.append(Node(
            package='navi_semantic_perception', executable='segmentation_trt',
            name='segmentation_trt', output='screen', parameters=[{
                'color_topic': f'{camera_ns}/color/image_raw',
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
