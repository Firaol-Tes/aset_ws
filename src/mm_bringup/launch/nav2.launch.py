"""
nav2.launch.py — Nav2 navigation stack (AMCL localisation + planner + controller).

Prerequisites:
  • sim.launch.py running
  • A saved map from slam.launch.py  (default: ~/aset_ws/maps/warehouse_map.yaml)

Usage:
  ros2 launch mm_bringup nav2.launch.py [map:=/path/to/map.yaml]

Then in RViz:
  1. Add → By topic → /map          (see the map)
  2. Add → By topic → /scan         (see the LiDAR)
  3. Use the "2D Pose Estimate" tool to set the initial robot pose.
  4. Use the "Nav2 Goal" (or "2D Nav Goal") tool to send a goal pose.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    mm_bringup  = get_package_share_directory('mm_bringup')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    default_map = str(Path.home() / 'aset_ws' / 'maps' / 'warehouse_map.yaml')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to the map yaml file',
    )

    nav2_params = os.path.join(mm_bringup, 'config', 'nav2_params.yaml')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'map': LaunchConfiguration('map'),
        }.items(),
    )

    # RViz for nav2
    rviz_config = os.path.join(nav2_bringup, 'rviz', 'nav2_default_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        declare_map,
        nav2,
        rviz,
    ])
