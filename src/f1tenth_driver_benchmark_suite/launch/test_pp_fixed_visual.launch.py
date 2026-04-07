#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    suite_share = get_package_share_directory('f1tenth_driver_benchmark_suite')
    default_raceline = os.path.join(
        suite_share, 'assets', 'racelines', 'Budapest_optimal_rl.csv'
    )

    benchmark_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(suite_share, 'launch', 'benchmark.launch.py')),
        launch_arguments={
            'driver': 'pp_fixed',
            'mode': 'visual',
            'run_tag': LaunchConfiguration('run_tag'),
            'output_root': LaunchConfiguration('output_root'),
            'raceline_path': LaunchConfiguration('raceline_path'),
            'use_realtime_visualizer': LaunchConfiguration('use_realtime_visualizer'),
            'visualizer_update_rate': LaunchConfiguration('visualizer_update_rate'),
            'visualizer_max_history': LaunchConfiguration('visualizer_max_history'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('run_tag', default_value='pp_fixed_visual_test'),
        DeclareLaunchArgument('output_root', default_value='/tmp/f1tenth_driver_benchmark_suite/results'),
        DeclareLaunchArgument(
            'raceline_path',
            default_value=default_raceline,
            description='Raceline path for pp_fixed visual test (default: RL Budapest_optimal converted CSV)',
        ),
        DeclareLaunchArgument(
            'use_realtime_visualizer',
            default_value='false',
            description='Enable matplotlib realtime visualizer window',
        ),
        DeclareLaunchArgument('visualizer_update_rate', default_value='2.0'),
        DeclareLaunchArgument('visualizer_max_history', default_value='1000'),
        benchmark_launch,
    ])
