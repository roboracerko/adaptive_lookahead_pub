#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Pure Pursuit Driver + F1TENTH Gym Simulation Launch File (Map-agnostic)
"""

from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    ld = LaunchDescription()
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz2'
    )
    raceline_arg = DeclareLaunchArgument(
        'raceline_csv',
        default_value='',
        description='Optional raceline CSV (empty to use pure_pursuit.yaml settings)'
    )
    
    # 패키지 경로 가져오기
    pure_pursuit_share = get_package_share_directory('pp_core')
    gym_ros_share = get_package_share_directory('f1tenth_gym_ros')
    
    # Pure Pursuit 설정
    pure_pursuit_config = os.path.join(pure_pursuit_share, 'config', 'pure_pursuit.yaml')

    # F1TENTH Gym 시뮬레이션 설정 (맵별 YAML 지정)
    sim_config_default = os.path.join(gym_ros_share, 'config', 'sim.yaml')
    sim_config_arg = DeclareLaunchArgument(
        'sim_config',
        default_value=sim_config_default,
        description='Path to f1tenth_gym_ros sim config yaml'
    )

    def launch_setup(context, *_args, **_kwargs):
        sim_config = LaunchConfiguration('sim_config').perform(context)

        # 설정 파일에서 맵 경로 읽기
        with open(sim_config, 'r') as f:
            config_dict = yaml.safe_load(f)
            map_path = config_dict['bridge']['ros__parameters']['map_path']

        rviz_config = os.path.join(gym_ros_share, 'launch', 'gym_bridge.rviz')

        # Gym Bridge 노드
        bridge_node = Node(
            package='f1tenth_gym_ros',
            executable='gym_bridge',
            name='bridge',
            parameters=[sim_config],
            output='screen'
        )

        # RViz 노드 (조건부 실행)
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        )

        # Map Server 노드
        # 노드 이름을 명시적으로 설정하여 lifecycle_manager가 찾을 수 있도록 함
        map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',  # 노드 이름 명시
            parameters=[{
                'yaml_filename': map_path + '.yaml',
                'topic': 'map',
                'frame_id': 'map',
                'use_sim_time': True  # F1TENTH Gym 시뮬레이션은 시뮬레이션 시간 사용
            }],
            output='screen'
        )

        # Nav2 Lifecycle Manager
        # autostart=True로 설정하면 자동으로 map_server를 활성화함
        nav_lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,  # F1TENTH Gym 시뮬레이션은 시뮬레이션 시간 사용
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

        # RViz2는 map_server가 활성화된 후 실행 (TRANSIENT_LOCAL 메시지를 받기 위해)
        rviz_timer = TimerAction(
            period=5.0,  # map_server 활성화 대기
            actions=[rviz_node]
        )

        return [
            bridge_node,
            map_server_node,
            nav_lifecycle_node,
            ego_robot_publisher,
            pure_pursuit_node,
            rviz_timer,
        ]

    # 노드 추가
    # 순서 중요:
    # 1. map_server가 lifecycle_manager보다 먼저 시작되어야 함
    # 2. RViz2는 map_server가 활성화된 후 실행되어야 TRANSIENT_LOCAL 메시지를 받을 수 있음
    ld.add_action(use_rviz_arg)
    ld.add_action(raceline_arg)
    ld.add_action(sim_config_arg)
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld
