"""
phase4.launch.py  —  Phase 4 nodes: perception + state manager + MoveIt + tool server.

Prerequisites (run first in separate terminals):
  ros2 launch mm_bringup sim.launch.py
  ros2 launch mm_bringup nav2.launch.py

Then:
  ros2 launch mm_bringup phase4.launch.py

After this is running, verify each component:
  B) ros2 service call /get_scene_state mm_interfaces/srv/GetSceneState
  A) ros2 topic echo /detected_objects
  C) ros2 service call /tool_server/detect_objects mm_interfaces/srv/DetectObjects
  D) ros2 run mobile_manipulator_core tool_test
"""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def load_file(path):
    with open(path) as f:
        return f.read()


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def generate_launch_description():

    mm_bringup   = get_package_share_directory('mm_bringup')
    desc_share   = get_package_share_directory('robotic_arm_description')
    moveit_share = get_package_share_directory('arm_moveit_config')
    core_share   = get_package_share_directory('mobile_manipulator_core')

    # ── Robot description (mobile manipulator, same as sim.launch.py) ─────────
    xacro_file = os.path.join(mm_bringup, 'urdf', 'mobile_manipulator.urdf.xacro')
    robot_description_content = xacro.process_file(
        xacro_file,
        mappings={'include_world': 'false', 'include_ros2_control': 'false'},
    ).toxml()
    robot_description = {'robot_description': robot_description_content}

    # ── MoveIt config ─────────────────────────────────────────────────────────
    srdf_content = load_file(os.path.join(moveit_share, 'config', 'robotic_arm.srdf'))
    robot_description_semantic = {'robot_description_semantic': srdf_content}

    kinematics_yaml = load_yaml(os.path.join(moveit_share, 'config', 'kinematics.yaml'))
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    joint_limits_yaml = load_yaml(os.path.join(moveit_share, 'config', 'joint_limits.yaml'))
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}

    planning_pipelines = {
        'planning_pipelines': {'pipeline_names': ['ompl']},
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

    moveit_controllers = load_yaml(os.path.join(moveit_share, 'config', 'moveit_controllers.yaml'))

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.5,
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

    locations_file = os.path.join(core_share, 'config', 'locations.yaml')

    # ── Nodes ─────────────────────────────────────────────────────────────────

    move_group = Node(
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
            {'use_sim_time': True},
        ],
    )

    grasp_server = Node(
        package='mobile_manipulator_core',
        executable='grasp_server',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    state_manager = Node(
        package='mobile_manipulator_core',
        executable='state_manager_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'locations_file': locations_file},
        ],
    )

    tool_server = Node(
        package='mobile_manipulator_core',
        executable='tool_server_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    perception = Node(
        package='mm_perception',
        executable='perception_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'detector': 'color'},
        ],
    )

    return LaunchDescription([
        move_group,
        grasp_server,
        state_manager,
        tool_server,
        perception,
    ])
