#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command


def _resolve_from_package(package_name: str, relative_path: str) -> str:
    return os.path.join(get_package_share_directory(package_name), relative_path)


def _build_sim_actions(context):
    scenario_file = LaunchConfiguration('scenario_file').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)

    scenario = {}
    with open(scenario_file, 'r', encoding='utf-8') as f:
        scenario = (yaml.safe_load(f) or {}).get('scenario', {})

    sim_cfg = scenario.get('sim', {})
    map_cfg = scenario.get('map', {})
    pose_cfg = scenario.get('start_pose', {})

    sim_config = _resolve_from_package(
        sim_cfg.get('package', 'f1tenth_gym_ros'),
        sim_cfg.get('config', 'config/sim_budapest.yaml'),
    )

    map_yaml = _resolve_from_package(
        map_cfg.get('package', 'f1tenth_driver_benchmark_suite'),
        map_cfg.get('yaml', 'assets/maps/Budapest.yaml'),
    )
    map_path = _resolve_from_package(
        map_cfg.get('package', 'f1tenth_driver_benchmark_suite'),
        map_cfg.get('path', 'assets/maps/Budapest'),
    )

    sx = float(pose_cfg.get('sx', 16.091))
    sy = float(pose_cfg.get('sy', 98.27))
    stheta = float(pose_cfg.get('stheta', 3.333))

    gym_ros_share = get_package_share_directory('f1tenth_gym_ros')

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[sim_config, {'map_path': map_path, 'sx': sx, 'sy': sy, 'stheta': stheta}],
        output='screen',
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': map_yaml,
            'topic': 'map',
            'frame_id': 'map',
            'use_sim_time': True,
        }],
        output='screen',
    )

    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ', os.path.join(gym_ros_share, 'launch', 'ego_racecar.xacro')
            ])
        }],
        remappings=[('/robot_description', 'ego_robot_description')],
        output='screen',
    )

    actions = [bridge_node, map_server_node, nav_lifecycle_node, ego_robot_publisher]

    if mode == 'visual':
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            output='screen',
        )
        actions.append(TimerAction(period=3.0, actions=[rviz_node]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario_file', description='Scenario YAML path'),
        DeclareLaunchArgument('mode', default_value='headless', description='headless|visual'),
        DeclareLaunchArgument('rviz_config', description='RViz config path'),
        OpaqueFunction(function=_build_sim_actions),
    ])
