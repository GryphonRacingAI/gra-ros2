import os                       
from glob import glob
from setuptools import setup, find_packages    

package_name = 'slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),

	data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    include_package_data=True,

    # Dependencies required at installation time
    install_requires=['setuptools', 'numpy', 'pycuda', 'common_msgs'],
    zip_safe=True,
    maintainer='Hasham Ahmad',
    maintainer_email='hashamahmad1818@gmail.com',
    description='ROS 2 wrapper for FastSLAM 1.0 GPU',
    license='MIT',
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts': [
            'fastslam_node = fastslam64.fastslam_node:main'
        ],
    },
)

