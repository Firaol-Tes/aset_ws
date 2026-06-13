"""
smoke_test.launch.py  —  Run moveit_smoke_test against a live demo session.

Prerequisites: the demo must already be running in another terminal:
  ros2 launch arm_moveit_config demo.launch.py

Then in a second terminal:
  ros2 launch mobile_manipulator_core smoke_test.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    smoke_test_node = Node(
        package='mobile_manipulator_core',
        executable='moveit_smoke_test',
        name='moveit_smoke_test',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([smoke_test_node])
