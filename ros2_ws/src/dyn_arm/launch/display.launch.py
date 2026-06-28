"""Digital twin only: pose the model in RViz with sliders, NO motors.

  joint_state_publisher_gui --/joint_states--> robot_state_publisher --> RViz

The hardware-free way to explore the model and joint limits. Run via tools/ros_launch.sh.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory("dyn_arm")
    robot_description = xacro.process_file(os.path.join(pkg, "urdf", "arm.xacro")).toxml()
    rviz = os.path.join(pkg, "rviz", "arm.rviz")
    return LaunchDescription([
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz]),
    ])
