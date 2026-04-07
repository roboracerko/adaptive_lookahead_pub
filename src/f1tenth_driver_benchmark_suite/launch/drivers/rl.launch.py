#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('raceline_path', description='Shared raceline CSV path'),
        DeclareLaunchArgument('checkpoint_dir', description='RL checkpoint directory'),
        DeclareLaunchArgument('rl_mode', default_value='lookahead', description='lookahead|controller|pure_pursuit_adv'),
        DeclareLaunchArgument('rate_hz', default_value='100.0'),
        DeclareLaunchArgument('vgain', default_value='1.2'),
        DeclareLaunchArgument('min_speed', default_value='0.0'),
        DeclareLaunchArgument('max_speed', default_value='99.0'),
        DeclareLaunchArgument('lookahead_distance', default_value='1.0'),
        DeclareLaunchArgument('local_index_search', default_value='false'),
        DeclareLaunchArgument('recovery_track_error', default_value='0.6'),
        DeclareLaunchArgument('recovery_speed_max', default_value='1.8'),
        DeclareLaunchArgument('use_common_speed_constraints', default_value='true'),
        DeclareLaunchArgument('pp_adv_a_lat_max', default_value='3.8'),
        DeclareLaunchArgument('pp_adv_slowdown_gain', default_value='1.4'),
        DeclareLaunchArgument('offtrack_threshold', default_value='1.8'),
        DeclareLaunchArgument('offtrack_hold_steps', default_value='12'),
        DeclareLaunchArgument('collision_scan_threshold', default_value='0.16'),
        DeclareLaunchArgument('collision_hold_steps', default_value='5'),

        Node(
            package='rl_f1tenth',
            executable='gym-bridge-infer',
            name='rl_gym_bridge_infer',
            output='screen',
            arguments=[
                '--mode', LaunchConfiguration('rl_mode'),
                '--checkpoint-dir', LaunchConfiguration('checkpoint_dir'),
                '--waypoint-path', LaunchConfiguration('raceline_path'),
                '--lookahead-distance', LaunchConfiguration('lookahead_distance'),
                '--rate-hz', LaunchConfiguration('rate_hz'),
                '--heading-filter',
                '--lookahead-max-forward', '20',
                '--lookahead-fallback-offset', '5',
                '--network', 'auto',
                '--vgain', LaunchConfiguration('vgain'),
                '--min-speed', LaunchConfiguration('min_speed'),
                '--max-speed', LaunchConfiguration('max_speed'),
                '--local-index-search', LaunchConfiguration('local_index_search'),
                '--recovery-track-error', LaunchConfiguration('recovery_track_error'),
                '--recovery-speed-max', LaunchConfiguration('recovery_speed_max'),
                '--use-common-speed-constraints', LaunchConfiguration('use_common_speed_constraints'),
                '--pp-adv-a-lat-max', LaunchConfiguration('pp_adv_a_lat_max'),
                '--pp-adv-slowdown-gain', LaunchConfiguration('pp_adv_slowdown_gain'),
                '--offtrack-threshold', LaunchConfiguration('offtrack_threshold'),
                '--offtrack-hold-steps', LaunchConfiguration('offtrack_hold_steps'),
                '--collision-scan-threshold', LaunchConfiguration('collision_scan_threshold'),
                '--collision-hold-steps', LaunchConfiguration('collision_hold_steps'),
            ],
        ),

        Node(
            package='rl_f1tenth',
            executable='raceline-publisher',
            name='raceline_publisher',
            output='screen',
            arguments=[
                '--waypoint-path', LaunchConfiguration('raceline_path'),
                '--frame-id', 'map',
                '--marker-topic', '/raceline_marker',
                '--path-topic', '/raceline_path',
            ],
        ),
    ])
