#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    kmpc_share = get_package_share_directory('kmpc_driver')

    default_cfg = os.path.join(
        kmpc_share, 'config', 'kmpc_budapest_benchmark_stable.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('raceline_path', description='Shared raceline CSV path (KMPC format)'),
        DeclareLaunchArgument('driver_config_file', default_value=default_cfg),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(kmpc_share, 'launch', 'kmpc_core.launch.py')
            ),
            launch_arguments={
                'global_waypoints_csv': LaunchConfiguration('raceline_path'),
                'kmpc_params': LaunchConfiguration('driver_config_file'),
            }.items(),
        ),
    ])
