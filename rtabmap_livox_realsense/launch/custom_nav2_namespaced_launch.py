#!/usr/bin/env python3
"""
custom_nav2_namespaced_launch.py

Launches a fully namespaced Nav2 stack for one Leo Rover.
Designed for multi-robot operation — each robot gets its own Nav2 with
all topics and TF frames scoped to its namespace.

Works with rtabmap_livox.launch.py (LiDAR-only SLAM).

Usage:
    ros2 launch rtabmap_livox_realsense custom_nav2_namespaced_launch.py robot_namespace:=leo04
    ros2 launch rtabmap_livox_realsense custom_nav2_namespaced_launch.py robot_namespace:=leo05 use_sim_time:=true

Notes:
    - Use the --show-args flag to see all available launch arguments.
    - The params file (nav2_params_repo.yaml) uses 'leo04' as a namespace
      placeholder. At launch time the placeholder is replaced with the
      actual robot_namespace argument, so no per-robot YAML copies are needed.

Author: Alexandre Frantz
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):
    robot_ns     = LaunchConfiguration('robot_namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    autostart    = LaunchConfiguration('autostart').perform(context) == 'true'
    params_src   = LaunchConfiguration('params_file').perform(context)

    # Replace the 'leo04' namespace placeholder in the params file with the
    # actual robot namespace so one YAML works for all robots.
    with open(params_src, 'r') as f:
        content = f.read()
    content = content.replace('leo04/', f'{robot_ns}/')

    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as tmp:
        tmp.write(content)
        tmp_path = tmp.name

    # Wrap params under the robot namespace key so nested plugin parameters
    # (e.g. DWB critics under FollowPath) resolve correctly for namespaced nodes.
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=tmp_path,
            root_key=robot_ns,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # RTAB-Map publishes TF with namespaced frame IDs (e.g. leo04/odom) to the
    # global /tf topic. Nav2 nodes must subscribe to global /tf, not a namespaced
    # one — so no TF remapping is needed or wanted here.
    tf_remaps = []

    # Nav2 Humble lifecycle nodes managed by lifecycle_manager.
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    return [GroupAction([
        PushRosNamespace(robot_ns),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')]),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=tf_remaps),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=tf_remaps),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=tf_remaps),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=tf_remaps),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=tf_remaps),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=tf_remaps + [
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart':    autostart,
                'node_names':   lifecycle_nodes,
                'bond_timeout': 30.0,  # seconds to wait per state transition (default 4.0 is too short)
            }]),
    ])]


def generate_launch_description():
    pkg_share = get_package_share_directory('rtabmap_livox_realsense')
    default_params = os.path.join(pkg_share, 'config_files', 'nav2_params_repo.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='leo04',
            description='Robot namespace — must match the RTAB-Map launch'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo clock if true; false for real robot'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to Nav2 params YAML (nav2_params_repo.yaml by default)'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Auto-activate Nav2 lifecycle nodes on launch'),

        OpaqueFunction(function=launch_setup),
    ])
