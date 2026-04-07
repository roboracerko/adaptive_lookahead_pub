#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    suite_share = get_package_share_directory('f1tenth_driver_benchmark_suite')
    default_raceline = os.path.join(
        suite_share, 'assets', 'racelines', 'Budapest_optimal_rl.csv'
    )
    default_metrics_common = os.path.join(suite_share, 'config', 'benchmark_common.yaml')
    rl_metrics_driver_file = os.path.join(suite_share, 'config', 'drivers', 'rl.yaml')

    benchmark_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(suite_share, 'launch', 'benchmark.launch.py')),
        launch_arguments={
            'driver': 'rl',
            'mode': 'visual',
            'run_tag': LaunchConfiguration('run_tag'),
            'output_root': LaunchConfiguration('output_root'),
            'rl_checkpoint_dir': LaunchConfiguration('rl_checkpoint_dir'),
            # RL visualizer is launched directly by this wrapper to support both tuned/non-tuned modes.
            'use_realtime_visualizer': 'false',
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_tuned_script')),
    )

    tuned_script_launch = ExecuteProcess(
        cmd=[
            LaunchConfiguration('tuned_script_path'),
            '--domain-id', LaunchConfiguration('domain_id'),
            '--use-rviz', 'true',
            '--log-file', LaunchConfiguration('log_file'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_tuned_script')),
    )

    tuned_metrics_node = Node(
        package='f1tenth_driver_metrics',
        executable='metrics_collector_node',
        name='metrics_collector',
        parameters=[
            LaunchConfiguration('metrics_common_file'),
            rl_metrics_driver_file,
            {
                'centerline_file': LaunchConfiguration('raceline_path'),
                'output_dir': '/tmp/f1tenth_driver_benchmark_suite/results/rl/rl_visual_test',
                'min_laps': 0,
                'max_laps': 0,
                'auto_shutdown': False,
                # Visual test mode: keep publishing metrics without stopping the simulator.
                'auto_shutdown_on_error': False,
            },
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_tuned_script')),
        additional_env={'ROS_DOMAIN_ID': LaunchConfiguration('domain_id')},
    )

    visualizer_node_benchmark_mode = Node(
        package='f1tenth_driver_metrics',
        executable='realtime_visualizer',
        name='realtime_visualizer',
        parameters=[{
            'odom_topic': '/ego_racecar/odom',
            'lateral_error_topic': '/metrics/debug/lateral_error',
            'lap_summary_topic': '/metrics/lap_summary',
            'centerline_file': LaunchConfiguration('raceline_path'),
            'update_rate': LaunchConfiguration('visualizer_update_rate'),
            'max_history': LaunchConfiguration('visualizer_max_history'),
        }],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('use_realtime_visualizer'), "' == 'true' and ",
            "'", LaunchConfiguration('use_tuned_script'), "' == 'false'",
        ])),
    )

    visualizer_node_tuned_mode = Node(
        package='f1tenth_driver_metrics',
        executable='realtime_visualizer',
        name='realtime_visualizer',
        parameters=[{
            'odom_topic': '/ego_racecar/odom',
            'lateral_error_topic': '/metrics/debug/lateral_error',
            'lap_summary_topic': '/metrics/lap_summary',
            'centerline_file': LaunchConfiguration('raceline_path'),
            'update_rate': LaunchConfiguration('visualizer_update_rate'),
            'max_history': LaunchConfiguration('visualizer_max_history'),
        }],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('use_realtime_visualizer'), "' == 'true' and ",
            "'", LaunchConfiguration('use_tuned_script'), "' == 'true'",
        ])),
        # Tuned script explicitly runs with --domain-id.
        # Force visualizer to the same ROS domain so topics are visible.
        additional_env={'ROS_DOMAIN_ID': LaunchConfiguration('domain_id')},
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_tuned_script',
            default_value='true',
            description='true: use rl_f1tenth/run_vgain1_tuned_3lapplus_220s.sh, false: use benchmark RL node',
        ),
        DeclareLaunchArgument(
            'tuned_script_path',
            default_value='/home/jin/ros2_prj/rl_f1tenth/run_vgain1_tuned_3lapplus_220s.sh',
            description='Path to tuned RL simulation launcher script',
        ),
        DeclareLaunchArgument('domain_id', default_value='90'),
        DeclareLaunchArgument('log_file', default_value='/tmp/rl_highspeed_v1_3lap_tuned_from_benchmark_suite.log'),
        DeclareLaunchArgument('run_tag', default_value='rl_visual_test'),
        DeclareLaunchArgument('output_root', default_value='/tmp/f1tenth_driver_benchmark_suite/results'),
        DeclareLaunchArgument(
            'use_realtime_visualizer',
            default_value='false',
            description='Enable matplotlib realtime visualizer window',
        ),
        DeclareLaunchArgument('visualizer_update_rate', default_value='2.0'),
        DeclareLaunchArgument('visualizer_max_history', default_value='1000'),
        DeclareLaunchArgument(
            'metrics_common_file',
            default_value=default_metrics_common,
            description='Common parameter file for metrics_collector',
        ),
        DeclareLaunchArgument(
            'raceline_path',
            default_value=default_raceline,
            description='Raceline path for matplotlib visualizer centerline',
        ),
        DeclareLaunchArgument(
            'rl_checkpoint_dir',
            default_value='/home/jin/ros2_prj/rl_f1tenth/checkpoints/lookahead/vgain_curriculum_asafe_3lap/s6/best',
            description='RL checkpoint directory',
        ),
        benchmark_launch,
        tuned_script_launch,
        tuned_metrics_node,
        visualizer_node_benchmark_mode,
        visualizer_node_tuned_mode,
    ])
