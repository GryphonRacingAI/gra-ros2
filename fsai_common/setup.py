from setuptools import find_packages, setup

package_name = 'fsai_common'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Silverash-i8',
    maintainer_email='user@todo.todo',
    description='Pure-Python helpers shared by the FSAI packages (planar geometry, quaternions).',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
)
