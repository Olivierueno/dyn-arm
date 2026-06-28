"""Hands-free demo: the arm runs a scripted choreography under its own motor profile.

demo_node sends one goal per pose; the Dynamixel profile does the smoothing. The twin
is driven from the arm's measured feedback, so it mirrors the real motion:

  demo_node --/goal_positions--> arm_node --> motors
  arm_node  --/present_positions--> twin_echo_node --/joint_states--> robot_state_publisher --> RViz

No sliders and no joint_command_node here - the demo commands the motors in ticks
directly. Run via tools/ros_launch.sh. U2D2 attached, 12 V on. Ctrl-C to stop; the
watchdog then eases the arm to home.
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
        Node(package="dyn_arm", executable="demo_node", parameters=[params], output="screen"),
        Node(package="dyn_arm", executable="arm_node", parameters=[params], output="screen"),
        Node(package="dyn_arm", executable="twin_echo_node", output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz]),
    ])
