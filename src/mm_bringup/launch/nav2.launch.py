"""
nav2.launch.py — Nav2 navigation stack with real-time SLAM localisation.

Uses slam_toolbox (not AMCL) to provide the map frame so the saved map
quality doesn't matter.  Drive the robot a little first to let SLAM build
enough of the map, then send Nav2 Goals.

Prerequisites:
  • sim.launch.py running (all 4 controllers active)

Usage:
  ros2 launch mm_bringup nav2.launch.py

In RViz:
  1. Wait ~5 s for SLAM to activate and start building the map.
  2. Use the "Nav2 Goal" tool to click a goal on the map.
     The robot will plan a path and drive there automatically.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    mm_bringup   = get_package_share_directory('mm_bringup')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    slam_params  = os.path.join(mm_bringup, 'config', 'slam_toolbox_params.yaml')
    nav2_params  = os.path.join(mm_bringup, 'config', 'nav2_params.yaml')

    # ── 1. slam_toolbox: real-time SLAM → provides /map + map→odom TF ───────
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
    )

    # async_slam_toolbox_node is a lifecycle node — must be activated.
    slam_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox'],
            'bond_timeout': 0.0,   # slam_toolbox doesn't implement bond
        }],
    )

    # ── 2. Nav2 navigation stack (planner + controller, no AMCL) ────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'autostart': 'true',
        }.items(),
    )

    # ── 3. twist_stamper: Nav2 Twist /cmd_vel → TwistStamped controller ─────
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True, 'frame_id': 'base_footprint'}],
        remappings=[
            ('/cmd_vel_in',  '/cmd_vel'),
            ('/cmd_vel_out', '/diff_drive_controller/cmd_vel'),
        ],
        output='screen',
    )

    # ── 4. RViz ──────────────────────────────────────────────────────────────
    rviz_config = os.path.join(nav2_bringup, 'rviz', 'nav2_default_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        slam_node,
        slam_lifecycle,
        nav2,
        twist_stamper,
        rviz,
    ])
