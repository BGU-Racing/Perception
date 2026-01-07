from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='grisha.guterman@gmail.com',
    description='BGRacing Perception Team Package in ROS2',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            "pcl_recording_reader = perception.nodes.PCLRecordingReadNode:main",
            "pcl_preprocess       = perception.nodes.PCLPreprocessingNode:main",
            "pcl_detection        = perception.nodes.PCLDetectionNode:main",
        ],
    },
)