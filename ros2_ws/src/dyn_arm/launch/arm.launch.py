"""Drive the REAL arm from RViz sliders, with the live twin.

  joint_state_publisher_gui --/joint_states--> robot_state_publisher --> RViz (twin)
                                       |
                                       +--> joint_command_node --/goal_positions--> arm_node --> motors

Run via tools/ros_launch.sh (sets the RViz display). U2D2 attached, 12 V on.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory("dyn_arm")
    params = os.path.join(pkg, "config", "arm_params.yaml")
    robot_description = xacro.process_file(os.path.join(pkg, "urdf", "arm.xacro")).toxml()
    rviz = os.path.join(pkg, "rviz", "arm.rviz")
    return LaunchDescription([
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz]),
        Node(package="dyn_arm", executable="joint_command_node", output="screen"),
        Node(package="dyn_arm", executable="arm_node", parameters=[params], output="screen"),
    ])
