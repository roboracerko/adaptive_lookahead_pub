#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Pure Pursuit Driver Launch File
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    
    # 패키지 경로 가져오기
    package_share = get_package_share_directory('pp_core')
    config_file = os.path.join(package_share, 'config', 'pure_pursuit.yaml')
    raceline_arg = DeclareLaunchArgument(
        'raceline_csv',
        default_value='',
        description='Optional raceline CSV (empty to use pure_pursuit.yaml settings)'
    )
    
    # Pure Pursuit Driver 노드
    pure_pursuit_node = Node(
        package='pp_core',
        executable='pp_core',
        name='pp_core',
        parameters=[
            config_file,
            {'raceline_csv': LaunchConfiguration('raceline_csv')}
        ],
        output='screen'
    )
    
    ld.add_action(raceline_arg)
    ld.add_action(pure_pursuit_node)
    
    return ld
