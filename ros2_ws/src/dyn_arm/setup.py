import os
from glob import glob
from setuptools import setup

package_name = 'dyn_arm'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Olivier Ueno',
    maintainer_email='Olivierueno@users.noreply.github.com',
    description='A 3-DOF Dynamixel arm in ROS2 with a kinematic safety floor, layered failsafes, and an RViz twin.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_node = dyn_arm.arm_node:main',
            'joint_command_node = dyn_arm.joint_command_node:main',
            'demo_node = dyn_arm.demo_node:main',
            'twin_echo_node = dyn_arm.twin_echo_node:main',
        ],
    },
)
