#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pure Pursuit Adaptive Lookahead + F1TENTH Gym 통합 Launch 파일
===============================================================
사전 준비:
  python3 scripts/setup_simulation.py --map_yaml <path> --raceline <path> [--traj]

실행:
  ros2 launch pp_adaptive pp_adaptive_sim_launch.py
  ros2 launch ... rviz:=false
  ros2 launch ... config_file:=/abs/path/to/simulation.yaml

파이프라인:
  config/simulation.yaml 읽기
    → gym_bridge 파라미터 생성 (map_path, spawn, max_laps)
    → pure_pursuit 파라미터 생성 (raceline_csv, override params)
    → 노드 실행: gym_bridge / map_server / lifecycle_manager /
                 ego_robot_state_publisher / pure_pursuit_node / [rviz2]
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command


PKG_NAME = "pp_adaptive"

# launch 파일 위치에서 패키지 소스 루트를 결정
# install 경로: .../install/<pkg>/share/<pkg>/launch/
# source 경로: .../src/<pkg>/launch/
# 소스 경로의 config/simulation.yaml 을 우선 읽고, 없으면 share 사용
_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_SOURCE_ROOT = os.path.dirname(_LAUNCH_DIR)
_SOURCE_SIM_CONFIG = os.path.join(_PKG_SOURCE_ROOT, "config", "simulation.yaml")


def _resolve_config_file(share_dir: str, override: str) -> str:
    """config_file 인수 처리: override → source → share 순으로 탐색"""
    if override and os.path.exists(override):
        return override
    if os.path.exists(_SOURCE_SIM_CONFIG):
        return _SOURCE_SIM_CONFIG
    return os.path.join(share_dir, "config", "simulation.yaml")


def _load_sim_config(config_path: str) -> dict:
    with open(config_path) as f:
        data = yaml.safe_load(f)
    return data.get("simulation", {})


def _infer_map_img_ext(map_yaml_path: str, map_path_no_ext: str) -> str:
    """맵 이미지 확장자(.png/.pgm/...)를 추론한다."""
    try:
        with open(map_yaml_path) as f:
            y = yaml.safe_load(f) or {}
        img = str(y.get("image", "")).strip()
        if img:
            ext = os.path.splitext(img)[1]
            if ext:
                return ext
    except Exception:
        pass

    for ext in (".png", ".pgm", ".jpg", ".jpeg"):
        if os.path.exists(map_path_no_ext + ext):
            return ext
    return ".png"


def _build_gym_bridge_params(sim: dict, map_path_no_ext: str, map_img_ext: str) -> dict:
    """gym_bridge 노드용 ROS 파라미터 딕셔너리 생성"""
    return {
        # topics / namespaces (f1tenth_gym_ros 기본값)
        "ego_namespace":       "ego_racecar",
        "ego_scan_topic":      "scan",
        "ego_odom_topic":      "odom",
        "ego_opp_odom_topic":  "opp_odom",
        "ego_drive_topic":     "drive",
        "opp_namespace":       "opp_racecar",
        "opp_scan_topic":      "opp_scan",
        "opp_odom_topic":      "odom",
        "opp_ego_odom_topic":  "opp_odom",
        "opp_drive_topic":     "opp_drive",
        # transform
        "scan_distance_to_base_link": 0.275,
        # laser
        "scan_fov":   4.7,
        "scan_beams": 1080,
        # map
        "map_path": map_path_no_ext,
        "map_img_ext": map_img_ext,
        # agents
        "num_agent": int(sim.get("num_agent", 1)),
        "max_laps":  int(sim.get("max_laps", 3)),
        # ego spawn
        "sx":     float(sim.get("spawn_x", 0.0)),
        "sy":     float(sim.get("spawn_y", 0.0)),
        "stheta": float(sim.get("spawn_theta", 0.0)),
        # opp spawn (고정값 — opp 없을 때도 필요)
        "sx1": 2.0, "sy1": 0.5, "stheta1": 0.0,
        # misc
        "kb_teleop":        False,
        "debug_lookahead_m": -1.0,
        "train_rl":         False,
    }


def _build_pp_override_params(sim: dict, raceline_path: str) -> dict:
    """pure_pursuit 노드에 전달할 override 파라미터"""
    params = {"raceline_csv": raceline_path}
    # simulation.yaml 에서 non-null 값만 override
    for key in ("lookahead_fallback", "a_lat_max", "slowdown_gain", "v_floor"):
        val = sim.get(key)
        if val is not None:
            params[key] = float(val)
    return params


def launch_setup(context, *args, **kwargs):
    """OpaqueFunction: LaunchConfiguration 값이 확정된 후 노드 생성"""
    share_dir = get_package_share_directory(PKG_NAME)
    gym_share  = get_package_share_directory("f1tenth_gym_ros")

    # ── config_file 결정 ──────────────────────────────────────────
    config_file_arg = LaunchConfiguration("config_file").perform(context)
    config_path = _resolve_config_file(share_dir, config_file_arg)
    if not os.path.exists(config_path):
        raise RuntimeError(
            f"simulation.yaml 을 찾을 수 없습니다: {config_path}\n"
            "먼저 scripts/setup_simulation.py 를 실행하세요."
        )
    sim = _load_sim_config(config_path)
    print(f"[pp_adaptive_sim_launch] config: {config_path}")

    # ── rviz 여부 결정 (launch arg > simulation.yaml) ────────────
    rviz_arg = LaunchConfiguration("rviz").perform(context).lower()
    use_rviz = (rviz_arg == "true") if rviz_arg in ("true", "false") \
               else bool(sim.get("rviz", True))

    # ── 경로 결정 ─────────────────────────────────────────────────
    map_name      = sim.get("map_name", "")
    raceline_file = sim.get("raceline_file", "")

    # maps/ 는 소스 또는 share 에서 탐색
    for maps_dir in [
        os.path.join(_PKG_SOURCE_ROOT, "maps"),
        os.path.join(share_dir, "maps"),
    ]:
        if os.path.exists(os.path.join(maps_dir, map_name + ".yaml")):
            map_path_no_ext = os.path.join(maps_dir, map_name)
            break
    else:
        raise RuntimeError(
            f"맵을 찾을 수 없습니다: {map_name}.yaml\n"
            f"  탐색 위치: {_PKG_SOURCE_ROOT}/maps/, {share_dir}/maps/"
        )

    # racelines/ 탐색
    for racelines_dir in [
        os.path.join(_PKG_SOURCE_ROOT, "racelines"),
        os.path.join(share_dir, "racelines"),
    ]:
        raceline_path = os.path.join(racelines_dir, raceline_file)
        if os.path.exists(raceline_path):
            break
    else:
        raise RuntimeError(
            f"레이스라인을 찾을 수 없습니다: {raceline_file}\n"
            f"  탐색 위치: {_PKG_SOURCE_ROOT}/racelines/, {share_dir}/racelines/"
        )

    pp_config = os.path.join(_PKG_SOURCE_ROOT, "config", "pure_pursuit.yaml")
    if not os.path.exists(pp_config):
        pp_config = os.path.join(share_dir, "config", "pure_pursuit.yaml")

    map_yaml_path = map_path_no_ext + ".yaml"
    map_img_ext = _infer_map_img_ext(map_yaml_path, map_path_no_ext)

    print(f"[pp_adaptive_sim_launch] 맵:        {map_yaml_path}")
    print(f"[pp_adaptive_sim_launch] 맵 이미지:  *{map_img_ext}")
    print(f"[pp_adaptive_sim_launch] 레이스라인: {raceline_path}")
    print(f"[pp_adaptive_sim_launch] RViz:      {use_rviz}")

    # ── 노드 정의 ─────────────────────────────────────────────────

    gym_params = _build_gym_bridge_params(sim, map_path_no_ext, map_img_ext)
    bridge_node = Node(
        package="f1tenth_gym_ros",
        executable="gym_bridge",
        name="bridge",
        parameters=[gym_params],
        output="screen",
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        parameters=[
            {"yaml_filename": map_path_no_ext + ".yaml"},
            {"topic": "map"},
            {"frame_id": "map"},
            {"output": "screen"},
            {"use_sim_time": True},
        ],
        output="screen",
    )

    nav_lifecycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    ego_robot_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="ego_robot_state_publisher",
        parameters=[{
            "robot_description": Command([
                "xacro ",
                os.path.join(gym_share, "launch", "ego_racecar.xacro"),
            ])
        }],
        remappings=[("/robot_description", "ego_robot_description")],
        output="screen",
    )

    pp_override = _build_pp_override_params(sim, raceline_path)
    pure_pursuit_node = Node(
        package=PKG_NAME,
        executable="pp_adaptive",
        name="pp_adaptive",
        parameters=[pp_config, pp_override],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", os.path.join(gym_share, "launch", "gym_bridge.rviz")],
        output="screen",
    )

    # ── 노드 목록 구성 ────────────────────────────────────────────
    nodes = [
        bridge_node,
        map_server_node,
        nav_lifecycle_node,
        ego_robot_publisher,
        pure_pursuit_node,
    ]
    if use_rviz:
        nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz",
            default_value="",  # 빈 문자열 → simulation.yaml 값 사용
            description="RViz 실행 여부 (true/false). 미입력 시 simulation.yaml 설정 사용",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="",  # 빈 문자열 → 자동 탐색
            description="simulation.yaml 경로 (미입력 시 자동 탐색)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
