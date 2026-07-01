"""
nav2_amcl.launch.py — Nav2 navigation stack with a static map + AMCL.

Replaces nav2.launch.py's continuous slam_toolbox localisation with a
pre-saved map (~/aset_ws/maps/warehouse_map_v2.yaml) + AMCL. Eliminates the
SLAM-drift-over-long-uptime degradation seen with continuous slam_toolbox
during long experiment batches -- the map is fixed, so there's nothing to
drift. Robot pose still needs AMCL to converge after each restart (it starts
at the map origin's best guess; should settle within a few seconds of the
robot moving).

Prerequisites:
  • sim.launch.py running (all 4 controllers active)
  • A saved map at the path below (see map_saver_cli)

Usage:
  ros2 launch mm_bringup nav2_amcl.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    mm_bringup   = get_package_share_directory('mm_bringup')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(mm_bringup, 'config', 'nav2_params.yaml')
    map_yaml    = '/home/f/aset_ws/maps/warehouse_map_v2.yaml'

    # ── 1. map_server + AMCL ─────────────────────────────────────────────────
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'map': map_yaml,
            'autostart': 'true',
        }.items(),
    )

    # ── 1b. Force-load the map via service call after map_server is up ───────
    # The localization_launch.py 'map' argument is unreliable: yaml_filename
    # ends up empty when IncludeLaunchDescription passes it through the
    # conditional Node() logic. Calling load_map directly is the robust fix.
    load_map = TimerAction(
        period=4.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/map_server/load_map',
                 'nav2_msgs/srv/LoadMap',
                 '{map_url: ' + map_yaml + '}'],
            output='screen',
        )],
    )

    # ── 2. Nav2 navigation stack (planner + controller) ─────────────────────
    # Delayed 10 s: give map_server+AMCL time to exchange the map before
    # the full navigation stack starts subscribing (avoids CPU/DDS race).
    nav2 = TimerAction(
        period=10.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params,
                'autostart': 'true',
            }.items(),
        )],
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
        localization,
        load_map,
        nav2,
        twist_stamper,
        rviz,
    ])
