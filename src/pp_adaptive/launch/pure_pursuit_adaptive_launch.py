#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Pure Pursuit Driver with Adaptive Lookahead Launch File
(Standalone - requires external simulator)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os


def generate_launch_description():
    ld = LaunchDescription()
    
    # 패키지 경로 가져오기
    package_share = get_package_share_directory('pp_adaptive')
    config_file = os.path.join(package_share, 'config', 'pure_pursuit.yaml')

    # RL과 동일 waypoint를 기본값으로 사용 (비교 평가용)
    default_raceline = '/home/jin/ros2_prj/rl_f1tenth/waypoints/Budapest_optimal.csv'
    try:
        rl_share = get_package_share_directory('rl_f1tenth')
        waypoint_from_install = os.path.join(rl_share, 'waypoints', 'Budapest_optimal.csv')
        if os.path.exists(waypoint_from_install):
            default_raceline = waypoint_from_install
    except PackageNotFoundError:
        pass

    if not os.path.exists(default_raceline):
        default_raceline = os.path.join(
            package_share, 'racelines', 'Budapest_raceline_vehicleaware_ld.csv'
        )
    
    # Pure Pursuit Driver 노드 (Adaptive Lookahead)
    pure_pursuit_node = Node(
        package='pp_adaptive',
        executable='pp_adaptive',
        name='pp_adaptive',
        parameters=[
            config_file,
            {'raceline_csv': default_raceline}
        ],
        output='screen'
    )
    
    ld.add_action(pure_pursuit_node)
    
    return ld
