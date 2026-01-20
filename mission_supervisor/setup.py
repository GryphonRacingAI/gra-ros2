from setuptools import find_packages, setup

package_name = 'mission_supervisor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sc23pg',
    maintainer_email='102541470+PrabodhGyawali@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_supervisor_node = mission_supervisor.mission_supervisor_node:main',
            'autonomous_demo = mission_supervisor.autonomous_demo:main',
            'lap_counter = mission_supervisor.lap_counter:main',
        ],
    },
)
