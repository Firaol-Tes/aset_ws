"""
slam.launch.py — Online SLAM with slam_toolbox (drive to build the map).

Prerequisites: sim.launch.py must already be running.

Usage:
  ros2 launch mm_bringup slam.launch.py

Then teleop the robot around the warehouse:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
    --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel

Save the map when done:
  ros2 run nav2_map_server map_saver_cli -f ~/aset_ws/maps/warehouse_map
  (creates warehouse_map.pgm and warehouse_map.yaml)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    mm_bringup = get_package_share_directory('mm_bringup')
    slam_params = os.path.join(mm_bringup, 'config', 'slam_toolbox_params.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
    )

    return LaunchDescription([slam_node])
