"""
sim.launch.py — Start the full mobile manipulator simulation in Gazebo Harmonic.

Starts:
  • Gazebo Harmonic (gz sim -r warehouse.sdf)
  • robot_state_publisher  (combined URDF)
  • ros_gz_sim/create      (spawns robot into Gazebo)
  • ros_gz_bridge          (clock, /scan, /imu, /camera/*, grasp topics)
  • gz_ros2_control        (activated automatically by robot's Gazebo plugin)
  • Controller spawners:   joint_state_broadcaster, diff_drive_controller,
                           arm_controller, gripper_controller

Usage:
  ros2 launch mm_bringup sim.launch.py

Then in a second terminal, teleop:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():

    mm_bringup  = get_package_share_directory('mm_bringup')
    ros_gz_sim  = get_package_share_directory('ros_gz_sim')

    # ── 1. Robot description ──────────────────────────────────────────────────
    xacro_file = os.path.join(mm_bringup, 'urdf', 'mobile_manipulator.urdf.xacro')
    robot_description_content = xacro.process_file(
        xacro_file,
        mappings={'include_world': 'false', 'include_ros2_control': 'false'},
    ).toxml()
    robot_description = {'robot_description': robot_description_content}

    # ── 2. Gazebo Harmonic ────────────────────────────────────────────────────
    world_file = os.path.join(mm_bringup, 'worlds', 'warehouse.sdf')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # ── 3. robot_state_publisher ──────────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # ── 4. Spawn robot in Gazebo ──────────────────────────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',  'mobile_manipulator',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.01',
        ],
        output='screen',
    )

    # ── 5. ros_gz_bridge (clock + sensors + grasp topics) ────────────────────
    bridge = RosGzBridge(
        bridge_name='gz_bridge',
        config_file=os.path.join(mm_bringup, 'config', 'gz_bridge.yaml'),
    )

    # ── 6. Controller spawners ────────────────────────────────────────────────
    # gz_ros2_control plugin activates after the robot is loaded into Gazebo.
    # Give it 5 s, then spawn joint_state_broadcaster; spawn the rest after.

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )
    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_controllers = TimerAction(period=5.0, actions=[jsb_spawner])

    start_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[diff_drive_spawner, arm_spawner, gripper_spawner],
        )
    )

    return LaunchDescription([
        gz_sim,
        rsp,
        spawn_robot,
        bridge,
        spawn_controllers,
        start_after_jsb,
    ])
