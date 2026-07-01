"""
nav2.launch.py — Nav2 with a pre-saved static map (no SLAM).

map_server serves the warehouse map; a static map→odom TF keeps the robot
at its spawn position (origin).  The map never distorts regardless of what
the arm or gripper does.

Prerequisites:
  • sim.launch.py running (all 4 controllers active)

Usage:
  ros2 launch mm_bringup nav2.launch.py
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

    nav2_params = os.path.join(mm_bringup, 'config', 'nav2_params.yaml')
    map_yaml    = '/home/f/aset_ws/maps/warehouse_map.yaml'

    # ── 1. Static TF: map → odom (identity — robot spawns at origin) ─────────
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── 2. map_server — publishes /map from the saved warehouse map ───────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'yaml_filename': map_yaml},
        ],
    )

    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # ── 3. Nav2 navigation stack (planner + controller, no AMCL) ────────────
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

    # ── 4. twist_stamper: Nav2 Twist /cmd_vel → TwistStamped controller ──────
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

    # ── 5. RViz ───────────────────────────────────────────────────────────────
    rviz_config = os.path.join(nav2_bringup, 'rviz', 'nav2_default_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        map_odom_tf,
        map_server,
        map_lifecycle,
        nav2,
        twist_stamper,
        rviz,
    ])
