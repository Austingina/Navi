from glob import glob
import os
from setuptools import setup

package_name = 'navi_semantic_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'),
         glob('models/*.onnx') + glob('models/MODEL_LICENSE.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='A26317',
    maintainer_email='a26317@autel.com',
    description='Depth->color-grid organized cloud + LabelInfo for semantic_segmentation_layer',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'depth_to_color_cloud = navi_semantic_perception.depth_to_color_cloud:main',
            'label_info_publisher = navi_semantic_perception.label_info_publisher:main',
            'stub_segmentation = navi_semantic_perception.stub_segmentation:main',
            'segmentation_trt = navi_semantic_perception.segmentation_trt:main',
            'build_trt_engine = navi_semantic_perception.trt_engine:main',
        ],
    },
)
