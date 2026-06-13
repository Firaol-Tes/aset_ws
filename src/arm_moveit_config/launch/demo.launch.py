"""
demo.launch.py  —  MoveIt 2 demo with mock hardware (no real robot needed).

Starts:
  • robot_state_publisher  (processes xacro → robot_description)
  • ros2_control controller_manager  (mock_components/GenericSystem)
  • joint_state_broadcaster, arm_controller, gripper_controller
  • move_group  (OMPL / KDL / MoveItSimpleControllerManager)
  • RViz 2 with the MotionPlanning plugin

Usage:
  ros2 launch arm_moveit_config demo.launch.py
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro


def load_file(pkg_share, *rel_path):
    return open(os.path.join(pkg_share, *rel_path)).read()


def load_yaml(pkg_share, *rel_path):
    with open(os.path.join(pkg_share, *rel_path)) as f:
        return yaml.safe_load(f)


def generate_launch_description():

    desc_share   = get_package_share_directory('robotic_arm_description')
    moveit_share = get_package_share_directory('arm_moveit_config')

    # ── 1. Robot description ─────────────────────────────────────────────────
    xacro_file = os.path.join(desc_share, 'urdf', 'robotic_arm.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()
    robot_description = {'robot_description': robot_description_content}

    # ── 2. SRDF ──────────────────────────────────────────────────────────────
    srdf_content = load_file(moveit_share, 'config', 'robotic_arm.srdf')
    robot_description_semantic = {'robot_description_semantic': srdf_content}

    # ── 3. Kinematics ────────────────────────────────────────────────────────
    kinematics_yaml = load_yaml(moveit_share, 'config', 'kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # ── 4. Joint limits ──────────────────────────────────────────────────────
    joint_limits_yaml = load_yaml(moveit_share, 'config', 'joint_limits.yaml')
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}

    # ── 5. Planning pipeline (MoveIt 2.12 / Jazzy layout) ───────────────────
    #
    # From moveit_cpp.hpp:
    #   node->get_parameter("planning_pipelines.pipeline_names", pipeline_names)
    #   node->get_parameter("planning_pipelines.namespace", parent_namespace)  # default ""
    #
    # From planning_pipeline::configure():
    #   ns = parent_namespace.empty() ? name_ : parent_namespace + "." + name_
    #   → with default empty parent_namespace and name_="ompl", ns = "ompl"
    #   → reads "ompl.planning_plugins", "ompl.request_adapters", "ompl.response_adapters"
    #
    # So pipeline configs go at TOP LEVEL under the pipeline name, NOT nested
    # under planning_pipelines. AddTimeOptimalParameterization is a response
    # adapter in 2.12. Prefix is default_planning_request/response_adapters.
    planning_pipelines = {
        'planning_pipelines': {
            'pipeline_names': ['ompl'],
        },
        'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
            ],
        },
    }

    # ── 6. MoveIt controller manager ─────────────────────────────────────────
    moveit_controllers = load_yaml(moveit_share, 'config', 'moveit_controllers.yaml')

    # ── 7. Trajectory execution / planning scene monitor ─────────────────────
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'monitor_dynamics': False,
    }

    # ── Nodes ─────────────────────────────────────────────────────────────────

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    ros2_controllers_yaml = os.path.join(moveit_share, 'config', 'ros2_controllers.yaml')
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_yaml],
        output='screen',
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipelines,
            trajectory_execution,
            planning_scene_monitor,
            moveit_controllers,
            {'use_sim_time': False},
        ],
    )

    rviz_config = os.path.join(moveit_share, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    start_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[arm_spawner, gripper_spawner],
        )
    )

    return LaunchDescription([
        rsp_node,
        ros2_control_node,
        jsb_spawner,
        start_arm_after_jsb,
        move_group_node,
        rviz_node,
    ])
