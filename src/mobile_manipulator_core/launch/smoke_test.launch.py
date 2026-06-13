"""
smoke_test.launch.py  —  Run moveit_smoke_test against a live demo session.

Prerequisites: the demo must already be running in another terminal:
  ros2 launch arm_moveit_config demo.launch.py

Then in a second terminal:
  ros2 launch mobile_manipulator_core smoke_test.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    desc_share   = get_package_share_directory('robotic_arm_description')
    moveit_share = get_package_share_directory('arm_moveit_config')

    # MoveGroupInterface needs robot_description + robot_description_semantic
    # to build the robot model locally.  Pass them explicitly so the node
    # doesn't have to wait for a topic that move_group doesn't publish by default.
    xacro_file = os.path.join(desc_share, 'urdf', 'robotic_arm.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()

    srdf_path = os.path.join(moveit_share, 'config', 'robotic_arm.srdf')
    with open(srdf_path) as f:
        srdf_content = f.read()

    smoke_test_node = Node(
        package='mobile_manipulator_core',
        executable='moveit_smoke_test',
        name='moveit_smoke_test',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'robot_description_semantic': srdf_content},
            {'use_sim_time': False},
        ],
    )

    return LaunchDescription([smoke_test_node])
