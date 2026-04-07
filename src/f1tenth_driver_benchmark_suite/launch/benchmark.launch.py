#!/usr/bin/env python3

import csv
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _prepare_rl_raceline_numeric(src_path: str, output_dir: str) -> str:
    """Convert a headered raceline CSV into a numeric-only CSV for rl_f1tenth."""
    if not os.path.exists(src_path):
        raise RuntimeError(f"raceline_path does not exist: {src_path}")

    dst_path = os.path.join(output_dir, 'raceline_rl_numeric.csv')
    wrote_any = False

    with open(src_path, 'r', newline='') as src, open(dst_path, 'w', newline='') as dst:
        reader = csv.reader(src)
        writer = csv.writer(dst)

        for row in reader:
            if not row:
                continue

            values = [col.strip() for col in row]
            if len(values) < 2:
                continue

            try:
                float(values[0])
                float(values[1])
            except ValueError:
                # Skip header or malformed rows.
                continue

            writer.writerow(values)
            wrote_any = True

    if not wrote_any:
        raise RuntimeError(
            f"Failed to prepare RL raceline from '{src_path}': no numeric rows found."
        )

    return dst_path


def _build_actions(context):
    suite_share = get_package_share_directory('f1tenth_driver_benchmark_suite')
    driver = LaunchConfiguration('driver').perform(context)
    run_tag = LaunchConfiguration('run_tag').perform(context)
    output_root = LaunchConfiguration('output_root').perform(context)
    shared_raceline_path = LaunchConfiguration('raceline_path').perform(context)

    if driver not in ('rl', 'pp_fixed', 'pp_adaptive', 'kmpc'):
        raise RuntimeError(f"Unsupported driver='{driver}'. Use rl|pp_fixed|pp_adaptive|kmpc")

    output_dir = os.path.join(output_root, driver, run_tag)
    os.makedirs(output_dir, exist_ok=True)

    sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(suite_share, 'launch', 'drivers', 'sim_common.launch.py')
        ),
        launch_arguments={
            'scenario_file': LaunchConfiguration('scenario_file'),
            'mode': LaunchConfiguration('mode'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
    )

    driver_launch_file = {
        'rl': 'rl.launch.py',
        'pp_fixed': 'pp_fixed.launch.py',
        'pp_adaptive': 'pp_adaptive.launch.py',
        'kmpc': 'kmpc.launch.py',
    }[driver]

    driver_raceline_path = shared_raceline_path
    if driver == 'rl':
        driver_raceline_path = _prepare_rl_raceline_numeric(shared_raceline_path, output_dir)
    elif driver == 'kmpc':
        kmpc_raceline = LaunchConfiguration('kmpc_raceline_path').perform(context)
        if kmpc_raceline:
            driver_raceline_path = kmpc_raceline

    driver_launch_args = {
        'raceline_path': driver_raceline_path,
    }

    if driver == 'rl':
        driver_launch_args.update({
            'checkpoint_dir': LaunchConfiguration('rl_checkpoint_dir'),
            'rl_mode': LaunchConfiguration('rl_mode'),
            'rate_hz': LaunchConfiguration('rl_rate_hz'),
            'vgain': LaunchConfiguration('rl_vgain'),
            'min_speed': LaunchConfiguration('rl_min_speed'),
            'max_speed': LaunchConfiguration('rl_max_speed'),
            'lookahead_distance': LaunchConfiguration('rl_lookahead_distance'),
            'local_index_search': LaunchConfiguration('rl_local_index_search'),
            'recovery_track_error': LaunchConfiguration('rl_recovery_track_error'),
            'recovery_speed_max': LaunchConfiguration('rl_recovery_speed_max'),
            'use_common_speed_constraints': LaunchConfiguration('rl_use_common_speed_constraints'),
            'pp_adv_a_lat_max': LaunchConfiguration('rl_pp_adv_a_lat_max'),
            'pp_adv_slowdown_gain': LaunchConfiguration('rl_pp_adv_slowdown_gain'),
            'offtrack_threshold': LaunchConfiguration('rl_offtrack_threshold'),
            'offtrack_hold_steps': LaunchConfiguration('rl_offtrack_hold_steps'),
            'collision_scan_threshold': LaunchConfiguration('rl_collision_scan_threshold'),
            'collision_hold_steps': LaunchConfiguration('rl_collision_hold_steps'),
        })

    if driver == 'pp_fixed':
        driver_launch_args.update({'driver_config_file': LaunchConfiguration('pp_fixed_config_file')})

    if driver == 'pp_adaptive':
        driver_launch_args.update({'driver_config_file': LaunchConfiguration('pp_adaptive_config_file')})

    if driver == 'kmpc':
        driver_launch_args.update({'driver_config_file': LaunchConfiguration('kmpc_config_file')})

    driver_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(suite_share, 'launch', 'drivers', driver_launch_file)
        ),
        launch_arguments=driver_launch_args.items(),
    )

    metrics_common_file = LaunchConfiguration('metrics_common_file').perform(context)
    metrics_driver_file = os.path.join(suite_share, 'config', 'drivers', f'{driver}.yaml')

    metrics_node = Node(
        package=LaunchConfiguration('metrics_package'),
        executable=LaunchConfiguration('metrics_executable'),
        name='metrics_collector',
        parameters=[
            metrics_common_file,
            metrics_driver_file,
            {
                'centerline_file': shared_raceline_path,
                'output_dir': output_dir,
                'min_laps': ParameterValue(LaunchConfiguration('min_laps'), value_type=int),
                'max_laps': ParameterValue(LaunchConfiguration('max_laps'), value_type=int),
                'auto_shutdown': ParameterValue(LaunchConfiguration('auto_shutdown'), value_type=bool),
            },
        ],
        output='screen',
    )

    visualizer_node = Node(
        package=LaunchConfiguration('metrics_visualizer_package'),
        executable=LaunchConfiguration('metrics_visualizer_executable'),
        name='realtime_visualizer',
        parameters=[{
            'odom_topic': '/ego_racecar/odom',
            'lateral_error_topic': '/metrics/debug/lateral_error',
            'lap_summary_topic': '/metrics/lap_summary',
            'centerline_file': shared_raceline_path,
            'update_rate': LaunchConfiguration('visualizer_update_rate'),
            'max_history': LaunchConfiguration('visualizer_max_history'),
        }],
        condition=IfCondition(LaunchConfiguration('use_realtime_visualizer')),
        output='screen',
    )

    raceline_pub_node = Node(
        package='f1tenth_driver_benchmark_suite',
        executable='raceline_path_publisher',
        name='benchmark_raceline_publisher',
        parameters=[{
            'raceline_csv': shared_raceline_path,
            'path_topic': '/benchmark/raceline_path',
            'frame_id': 'map',
            'publish_hz': 1.0,
        }],
        output='screen',
    )

    return [
        LogInfo(msg=[
            '[benchmark] driver=', driver,
            ', mode=', LaunchConfiguration('mode'),
            ', output_dir=', output_dir,
            ', shared_raceline=', shared_raceline_path,
            ', driver_raceline=', driver_raceline_path,
        ]),
        sim_include,
        metrics_node,
        raceline_pub_node,
        TimerAction(period=2.0, actions=[driver_include]),
        visualizer_node,
    ]


def generate_launch_description():
    suite_share = get_package_share_directory('f1tenth_driver_benchmark_suite')

    default_raceline = os.path.join(
        suite_share,
        'assets',
        'racelines',
        'Budapest_raceline_vehicleaware.csv',
    )

    default_scenario = os.path.join(suite_share, 'config', 'scenarios', 'budapest.yaml')
    default_rviz = os.path.join(suite_share, 'rviz', 'benchmark.rviz')
    default_metrics_common = os.path.join(suite_share, 'config', 'benchmark_common.yaml')

    default_rl_checkpoint = '/home/jin/ros2_prj/rl_f1tenth/checkpoints/lookahead/vgain_curriculum_asafe_3lap/s6/best'

    return LaunchDescription([
        DeclareLaunchArgument('driver', default_value='rl', description='rl|pp_fixed|pp_adaptive'),
        DeclareLaunchArgument('mode', default_value='headless', description='headless|visual'),
        DeclareLaunchArgument('run_tag', default_value='manual', description='Run identifier'),
        DeclareLaunchArgument(
            'output_root',
            default_value='/tmp/f1tenth_driver_benchmark_suite/results',
            description='Benchmark output root directory',
        ),
        DeclareLaunchArgument('scenario_file', default_value=default_scenario),
        DeclareLaunchArgument('raceline_path', default_value=default_raceline),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('metrics_common_file', default_value=default_metrics_common),
        DeclareLaunchArgument(
            'metrics_package',
            default_value='f1tenth_driver_benchmark_suite',
            description='Metrics package name',
        ),
        DeclareLaunchArgument(
            'metrics_executable',
            default_value='metrics_collector_node',
            description='Metrics collector executable',
        ),
        DeclareLaunchArgument(
            'metrics_visualizer_package',
            default_value='f1tenth_driver_benchmark_suite',
            description='Realtime visualizer package',
        ),
        DeclareLaunchArgument(
            'metrics_visualizer_executable',
            default_value='realtime_visualizer',
            description='Realtime visualizer executable',
        ),
        DeclareLaunchArgument('min_laps', default_value='2'),
        DeclareLaunchArgument('max_laps', default_value='2'),
        DeclareLaunchArgument('auto_shutdown', default_value='true'),
        DeclareLaunchArgument(
            'use_realtime_visualizer',
            default_value='false',
            description='Enable matplotlib realtime visualizer (opens a separate GUI window)',
        ),
        DeclareLaunchArgument(
            'visualizer_update_rate',
            default_value='2.0',
            description='Matplotlib visualizer update rate (Hz)',
        ),
        DeclareLaunchArgument(
            'visualizer_max_history',
            default_value='1000',
            description='Maximum history points for matplotlib visualizer',
        ),

        DeclareLaunchArgument('rl_checkpoint_dir', default_value=default_rl_checkpoint),
        DeclareLaunchArgument('rl_mode', default_value='lookahead'),
        DeclareLaunchArgument('rl_rate_hz', default_value='100.0'),
        DeclareLaunchArgument('rl_vgain', default_value='1.2'),
        DeclareLaunchArgument('rl_min_speed', default_value='0.0'),
        DeclareLaunchArgument('rl_max_speed', default_value='99.0'),
        DeclareLaunchArgument('rl_lookahead_distance', default_value='1.0'),
        DeclareLaunchArgument('rl_local_index_search', default_value='false'),
        DeclareLaunchArgument('rl_recovery_track_error', default_value='0.6'),
        DeclareLaunchArgument('rl_recovery_speed_max', default_value='1.8'),
        DeclareLaunchArgument('rl_use_common_speed_constraints', default_value='true'),
        DeclareLaunchArgument('rl_pp_adv_a_lat_max', default_value='3.8'),
        DeclareLaunchArgument('rl_pp_adv_slowdown_gain', default_value='1.4'),
        DeclareLaunchArgument('rl_offtrack_threshold', default_value='1.8'),
        DeclareLaunchArgument('rl_offtrack_hold_steps', default_value='12'),
        DeclareLaunchArgument('rl_collision_scan_threshold', default_value='0.16'),
        DeclareLaunchArgument('rl_collision_hold_steps', default_value='5'),

        DeclareLaunchArgument(
            'pp_fixed_config_file',
            default_value=os.path.join(
                get_package_share_directory('pp_core'),
                'config',
                'pure_pursuit.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'pp_adaptive_config_file',
            default_value=os.path.join(
                get_package_share_directory('pp_adaptive'),
                'config',
                'pure_pursuit.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'kmpc_config_file',
            default_value=os.path.join(
                get_package_share_directory('kmpc_driver'),
                'config',
                'kmpc_budapest_benchmark_stable.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'kmpc_raceline_path',
            default_value=os.path.join(
                get_package_share_directory('f1tenth_driver_benchmark_suite'),
                'assets', 'racelines', 'Budapest_optimal_rl_kmpc.csv',
            ),
            description='KMPC-format raceline CSV (with s_m column).',
        ),

        OpaqueFunction(function=_build_actions),
    ])
