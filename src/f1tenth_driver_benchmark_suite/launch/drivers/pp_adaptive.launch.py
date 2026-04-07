#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('pp_adaptive'),
        'config',
        'pure_pursuit.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('raceline_path', description='Shared raceline CSV path'),
        DeclareLaunchArgument('driver_config_file', default_value=default_cfg),

        Node(
            package='pp_adaptive',
            executable='pp_adaptive',
            name='pp_adaptive',
            parameters=[
                LaunchConfiguration('driver_config_file'),
                # raceline_csv intentionally NOT overridden here:
                # pp_adaptive uses its own ld_m-annotated raceline from the config file.
            ],
            output='screen',
        ),
    ])
