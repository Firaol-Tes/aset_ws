"""
slam.launch.py — Online SLAM with slam_toolbox + RViz (drive to build the map).

Prerequisites: sim.launch.py must already be running.

Usage:
  ros2 launch mm_bringup slam.launch.py

Then teleop the robot around the warehouse to build the map.

Save the map when done:
  ros2 run nav2_map_server map_saver_cli -f ~/aset_ws/maps/warehouse_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    mm_bringup  = get_package_share_directory('mm_bringup')
    slam_params = os.path.join(mm_bringup, 'config', 'slam_toolbox_params.yaml')
    rviz_config = os.path.join(mm_bringup, 'config', 'slam_rviz.rviz')

    # async_slam_toolbox_node is a lifecycle node — it won't subscribe to /scan
    # or publish TF until configured+activated.  lifecycle_manager does that.
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox'],
            'bond_timeout': 0.0,   # disable watchdog — slam_toolbox doesn't implement bond
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([slam_node, lifecycle_manager, rviz_node])
