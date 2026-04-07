#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Pure Pursuit Driver + F1TENTH Gym Simulation Launch File
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    ld = LaunchDescription()
    
    # 패키지 경로 가져오기
    pure_pursuit_share = get_package_share_directory('pp_core')
    gym_ros_share = get_package_share_directory('f1tenth_gym_ros')
    
    # Pure Pursuit 설정
    pure_pursuit_config = os.path.join(pure_pursuit_share, 'config', 'pure_pursuit.yaml')
    raceline_arg = DeclareLaunchArgument(
        'raceline_csv',
        default_value='',
        description='Optional raceline CSV (empty to use pure_pursuit.yaml settings)'
    )
    
    # F1TENTH Gym 시뮬레이션 설정
    sim_config = os.path.join(gym_ros_share, 'config', 'sim.yaml')
    
    # 설정 파일에서 맵 경로 읽기 (동적 맵 사용)
    with open(sim_config, 'r') as f:
        config_dict = yaml.safe_load(f)
        map_path = config_dict['bridge']['ros__parameters']['map_path']
    
    # F1TENTH Gym 시뮬레이션 노드들 (기존 런치 파일에서)
    from launch.substitutions import Command
    rviz_config = os.path.join(gym_ros_share, 'launch', 'gym_bridge.rviz')
    
    # Gym Bridge 노드
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[sim_config],
        output='screen'
    )
    
    # RViz 노드
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # Map Server 노드 (sim.yaml의 map_path 사용 - 동적 맵 설정)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{
            'yaml_filename': map_path + '.yaml',  # sim.yaml의 map_path 사용
            'topic': 'map',
            'frame_id': 'map',
            'output': 'screen',
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # Nav2 Lifecycle Manager
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # Robot State Publisher (Ego)
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
        output='screen'
    )
    
    # Pure Pursuit Driver 노드
    pure_pursuit_node = Node(
        package='pp_core',
        executable='pp_core',
        name='pp_core',
        parameters=[
            pure_pursuit_config,
            {'raceline_csv': LaunchConfiguration('raceline_csv')}
        ],
        output='screen'
    )
    
    # 노드 추가
    ld.add_action(raceline_arg)
    ld.add_action(bridge_node)
    ld.add_action(rviz_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    ld.add_action(pure_pursuit_node)
    
    return ld
