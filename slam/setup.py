from setuptools import setup
import os
from glob import glob

package_name = 'fastslam_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # Numpy is explicitly listed here and must be installed in the environment (via rosdep or pip)
    install_requires=['setuptools', 'numpy'], 
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@todo.todo',
    description='ROS 2 wrapper for FastSLAM 1.0 GPU',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This line makes your file executable via 'ros2 run fastslam_ros fastslam_node'
            'fastslam_node = fastslam_ros.fastslam_node:main'
        ],
    },
)
