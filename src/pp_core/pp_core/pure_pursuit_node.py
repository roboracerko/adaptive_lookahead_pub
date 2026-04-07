#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Pure Pursuit Driver Node for F1TENTH Gym ROS2

고정 lookahead 거리를 사용하는 Pure Pursuit 제어기
CSV 파일에서 경로를 로드하고, Odometry를 구독하여 제어 명령을 발행합니다.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
import csv
import math
import os
from ament_index_python.packages import get_package_share_directory


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def nearest_point_idx_window(pos: np.ndarray, traj: np.ndarray, i0: int, win: int) -> int:
    """Find nearest waypoint index within a window"""
    N = len(traj)
    idxs = (np.arange(i0, i0 + win) % N)
    d = np.linalg.norm(traj[idxs] - pos, axis=1)
    return int(idxs[int(np.argmin(d))])

def nearest_point_idx_global(pos: np.ndarray, traj: np.ndarray) -> int:
    """Find nearest waypoint index over the full trajectory"""
    d = np.linalg.norm(traj - pos, axis=1)
    return int(np.argmin(d))

def lookahead_point_from_idx(traj: np.ndarray, i0: int, Ld: float) -> tuple:
    """Find lookahead point from trajectory starting at index i0"""
    N = len(traj)
    dist_acc = 0.0
    i = i0
    guard = 0
    while dist_acc < Ld and guard < N + 5:
        j = (i + 1) % N
        dist_acc += float(np.linalg.norm(traj[j] - traj[i]))
        i = j
        guard += 1
    return traj[i], i


def pure_pursuit_delta(x: float, y: float, theta: float,
                       lookahead_pt: np.ndarray,
                       wheelbase: float, Ld: float,
                       delta_max: float) -> tuple:
    """
    Pure Pursuit controller (Ackermann geometry)
    
    Args:
        x, y, theta: Current vehicle pose
        lookahead_pt: Lookahead point [x, y]
        wheelbase: Vehicle wheelbase (m)
        Ld: Lookahead distance (m)
        delta_max: Maximum steering angle (rad)
    
    Returns:
        delta: Steering angle (rad)
        alpha: Lookahead angle (rad)
    """
    dx = float(lookahead_pt[0] - x)
    dy = float(lookahead_pt[1] - y)
    alpha = wrap_angle(math.atan2(dy, dx) - theta)
    delta = math.atan2(2.0 * wheelbase * math.sin(alpha), max(1e-6, Ld))
    delta = float(np.clip(delta, -delta_max, delta_max))
    return delta, alpha


def lateral_speed_limit(wheelbase: float, delta: float, a_lat_max: float, v_cap: float = 1e9) -> float:
    """
    Calculate maximum speed based on lateral acceleration constraint
    
    Args:
        wheelbase: Vehicle wheelbase (m)
        delta: Steering angle (rad)
        a_lat_max: Maximum lateral acceleration (m/s²)
        v_cap: Maximum speed cap (m/s)
    
    Returns:
        Maximum allowed speed (m/s)
    """
    kappa = abs(math.tan(delta)) / max(1e-6, wheelbase)
    if kappa < 1e-9:
        return float(v_cap)
    return float(min(v_cap, math.sqrt(max(0.0, a_lat_max / kappa))))


class PurePursuitDriver(Node):
    """Pure Pursuit Driver Node"""

    def __init__(self):
        super().__init__('pp_core')
        
        # 파라미터 선언
        self.declare_parameter('raceline_csv', '')
        self.declare_parameter('map_name', '')
        self.declare_parameter('raceline_pattern', '{map_name}_raceline_vehicleaware.csv')
        self.declare_parameter('raceline_package', 'pp_core')
        self.declare_parameter('raceline_packages', ['pp_core'])
        self.declare_parameter('raceline_dir', 'racelines')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('wheelbase', 0.3302)  # F1TENTH Gym default
        self.declare_parameter('delta_max_deg', 24.0)  # F1TENTH Gym default
        self.declare_parameter('lookahead_distance', 1.0)  # 고정 lookahead (m)
        self.declare_parameter('control_hz', 50.0)
        self.declare_parameter('search_window', 300)
        self.declare_parameter('relocalize_distance', 2.0)  # m, use global nearest if pose jumps
        self.declare_parameter('stop_speed_threshold', 0.1)  # m/s, treat as stopped
        self.declare_parameter('stop_time_threshold', 0.5)  # s, require sustained stop
        self.declare_parameter('raceline_frame', 'map')
        self.declare_parameter('publish_raceline', True)
        self.declare_parameter('raceline_pub_hz', 1.0)
        
        # 속도 제약 파라미터
        self.declare_parameter('a_lat_max', 7.5)  # 최대 횡가속도 (m/s²)
        self.declare_parameter('use_steer_slowdown', True)
        self.declare_parameter('slowdown_gain', 0.5)
        self.declare_parameter('v_floor', 1.0)  # 최저 속도 (m/s)
        
        # 파라미터 가져오기
        raceline_csv = self.get_parameter('raceline_csv').value
        map_name = self.get_parameter('map_name').value
        raceline_pattern = self.get_parameter('raceline_pattern').value
        raceline_package = self.get_parameter('raceline_package').value
        raceline_packages = self.get_parameter('raceline_packages').value
        raceline_dir = self.get_parameter('raceline_dir').value

        # 패키지 목록 구성 (신규/기존 맵 모두 검색 가능)
        if not raceline_packages:
            raceline_packages = [raceline_package] if raceline_package else []

        def build_candidates(filename: str):
            candidates = []
            if not filename:
                return candidates
            if os.path.isabs(filename):
                candidates.append(filename)
                return candidates
            for pkg in raceline_packages:
                try:
                    pkg_share = get_package_share_directory(pkg)
                except Exception:
                    continue
                base_dir = raceline_dir if os.path.isabs(raceline_dir) else os.path.join(pkg_share, raceline_dir)
                candidates.append(os.path.join(base_dir, filename))
            return candidates

        candidates = []

        # 1) 명시적 raceline_csv
        if raceline_csv:
            candidates.extend(build_candidates(raceline_csv))

        # 2) map_name + pattern
        if map_name and raceline_pattern:
            try:
                name_by_pattern = raceline_pattern.format(map_name=map_name)
                candidates.extend(build_candidates(name_by_pattern))
            except Exception:
                pass

        # 3) 기본 패턴들 (기존/신규 맵 모두 대응)
        if map_name:
            for suffix in [
                'raceline_vehicleaware.csv',
                'raceline_ld.csv',
                'optimal.csv',
                'centerline.csv',
            ]:
                candidates.extend(build_candidates(f"{map_name}_{suffix}"))

        # 4) 최후의 기본값
        candidates.extend(build_candidates('Budapest_raceline_vehicleaware.csv'))

        # 실제 파일 결정
        raceline_csv = None
        for cand in candidates:
            if cand and os.path.exists(cand):
                raceline_csv = cand
                break

        if not raceline_csv:
            self.get_logger().error('Raceline CSV not found. Tried:')
            for cand in candidates:
                self.get_logger().error(f'  - {cand}')
            raise FileNotFoundError('Raceline CSV not found. Check pure_pursuit.yaml settings.')
        
        self.wheelbase = self.get_parameter('wheelbase').value
        self.delta_max = self.get_parameter('delta_max_deg').value * math.pi / 180.0
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.search_window = self.get_parameter('search_window').value
        self.relocalize_distance = self.get_parameter('relocalize_distance').value
        self.stop_speed_threshold = self.get_parameter('stop_speed_threshold').value
        self.stop_time_threshold = self.get_parameter('stop_time_threshold').value
        self.a_lat_max = self.get_parameter('a_lat_max').value
        self.use_steer_slowdown = self.get_parameter('use_steer_slowdown').value
        self.slowdown_gain = self.get_parameter('slowdown_gain').value
        self.v_floor = self.get_parameter('v_floor').value
        
        # 경로 로드
        self.load_raceline(raceline_csv)

        # RViz 경로 퍼블리셔 (latched)
        self.raceline_frame = self.get_parameter('raceline_frame').value
        self.publish_raceline = self.get_parameter('publish_raceline').value
        self.raceline_pub_hz = float(self.get_parameter('raceline_pub_hz').value)
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.raceline_pub = self.create_publisher(Path, 'raceline_path', qos)
        if self.publish_raceline:
            self.publish_raceline_path()
            if self.raceline_pub_hz > 0.0:
                self.raceline_timer = self.create_timer(1.0 / self.raceline_pub_hz, self.publish_raceline_path)
        
        # 현재 상태
        self.current_pose = None  # (x, y, yaw)
        self.current_velocity = 0.0
        self.last_ref_idx = None
        self.last_pose_xy = None
        self.stopped_since = None
        self.force_global_localization = False
        
        # 구독
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        # 발행
        drive_topic = self.get_parameter('drive_topic').value
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )
        
        # 제어 타이머
        control_period = 1.0 / self.get_parameter('control_hz').value
        self.control_timer = self.create_timer(control_period, self.control_callback)
        
        self.get_logger().info(f'Pure Pursuit Driver initialized')
        self.get_logger().info(f'  Raceline: {raceline_csv} ({len(self.ref_xy)} waypoints)')
        self.get_logger().info(f'  Lookahead distance: {self.lookahead_distance} m')
        self.get_logger().info(f'  Wheelbase: {self.wheelbase} m')
        self.get_logger().info(f'  Max steering: {self.delta_max * 180.0 / math.pi:.1f} deg')
    
    def load_raceline(self, csv_path: str):
        """Load raceline from CSV file"""
        if not os.path.exists(csv_path):
            raise FileNotFoundError(f'Raceline CSV not found: {csv_path}')
        
        # CSV 파일 읽기
        x_list, y_list, v_list = [], [], []
        has_v = False
        
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            
            # 헤더 확인
            if 'x_m' not in reader.fieldnames or 'y_m' not in reader.fieldnames:
                raise ValueError(f'CSV must contain x_m and y_m columns. Found: {list(reader.fieldnames)}')
            
            has_v = 'v_mps' in reader.fieldnames or 'vx_mps' in reader.fieldnames
            
            # 데이터 읽기
            for row in reader:
                x_list.append(float(row['x_m']))
                y_list.append(float(row['y_m']))
                if has_v:
                    v_list.append(float(row.get('v_mps', row.get('vx_mps'))))
        
        # NumPy 배열로 변환
        self.ref_xy = np.column_stack([x_list, y_list]).astype(float)
        
        # 속도 프로파일
        if has_v:
            self.ref_v = np.array(v_list, dtype=float)
        else:
            self.get_logger().warn('v_mps/vx_mps column not found, using default speed 3.0 m/s')
            self.ref_v = np.ones(len(self.ref_xy), dtype=float) * 3.0
        
        self.get_logger().info(f'Loaded {len(self.ref_xy)} waypoints from {csv_path}')

    def publish_raceline_path(self):
        """Publish raceline as nav_msgs/Path for RViz2"""
        if self.ref_xy is None or len(self.ref_xy) == 0:
            return
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.raceline_frame
        poses = []
        for x, y in self.ref_xy:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            poses.append(pose)
        path_msg.poses = poses
        self.raceline_pub.publish(path_msg)
    
    def quaternion_to_yaw(self, quaternion) -> float:
        """Convert quaternion to yaw angle"""
        # Quaternion: (x, y, z, w)
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        qw = quaternion.w
        
        # Roll, pitch, yaw 계산
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def odom_callback(self, msg: Odometry):
        """Update current vehicle state from Odometry"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(ori)
        self.current_pose = (pos.x, pos.y, yaw)
        self.current_velocity = msg.twist.twist.linear.x
    
    def apply_speed_constraints(self, v_target: float, delta: float) -> float:
        """Apply speed constraints based on steering and lateral acceleration"""
        # 조향 감속 (옵션)
        if self.use_steer_slowdown:
            steer_ratio = abs(delta) / max(1e-6, self.delta_max)
            v_target = v_target * (1.0 - self.slowdown_gain * steer_ratio)
            v_target = max(self.v_floor, v_target)
        
        # 횡가속도 제한
        v_lat_cap = lateral_speed_limit(self.wheelbase, delta, self.a_lat_max, v_cap=1e9)
        v_target = min(v_target, v_lat_cap)
        v_target = max(0.0, v_target)
        
        return v_target
    
    def control_callback(self):
        """Main control loop"""
        if self.current_pose is None:
            return
        
        x, y, yaw = self.current_pose
        now = self.get_clock().now()

        # If the car was stopped and starts moving again, relocalize globally
        if abs(self.current_velocity) <= self.stop_speed_threshold:
            if self.stopped_since is None:
                self.stopped_since = now
        else:
            if self.stopped_since is not None:
                stopped_dt = (now - self.stopped_since).nanoseconds / 1e9
                if stopped_dt >= self.stop_time_threshold:
                    self.force_global_localization = True
                self.stopped_since = None
        
        # 가장 가까운 웨이포인트 찾기
        pos = np.array([x, y], dtype=float)
        if self.force_global_localization or self.last_ref_idx is None:
            i = nearest_point_idx_global(pos, self.ref_xy)
            self.force_global_localization = False
        else:
            if self.last_pose_xy is not None:
                dx = x - self.last_pose_xy[0]
                dy = y - self.last_pose_xy[1]
                if math.hypot(dx, dy) > self.relocalize_distance:
                    i = nearest_point_idx_global(pos, self.ref_xy)
                else:
                    i = nearest_point_idx_window(pos, self.ref_xy, i0=self.last_ref_idx, win=self.search_window)
            else:
                i = nearest_point_idx_window(pos, self.ref_xy, i0=self.last_ref_idx, win=self.search_window)
        self.last_ref_idx = i
        self.last_pose_xy = (x, y)
        
        # Lookahead 포인트 찾기
        lookahead_pt, _ = lookahead_point_from_idx(self.ref_xy, i, self.lookahead_distance)
        
        # Pure Pursuit 조향각 계산
        delta, alpha = pure_pursuit_delta(
            x, y, yaw, lookahead_pt,
            self.wheelbase, self.lookahead_distance, self.delta_max
        )
        
        # 목표 속도 결정
        v_ref = float(self.ref_v[i])
        v_target = self.apply_speed_constraints(v_ref, delta)
        
        # 제어 명령 발행
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = v_target
        drive_msg.drive.steering_angle = delta
        drive_msg.drive.steering_angle_velocity = 0.0
        drive_msg.drive.acceleration = 0.0
        drive_msg.drive.jerk = 0.0
        
        self.drive_pub.publish(drive_msg)
        
        # 주기적 로깅 (1초마다)
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        
        if self._log_counter % int(self.get_parameter('control_hz').value) == 0:
            cte = float(np.linalg.norm(self.ref_xy[i] - pos))
            self.get_logger().info(
                f'idx={i:4d} v_ref={v_ref:5.2f} v_tgt={v_target:5.2f} v_act={self.current_velocity:5.2f} '
                f'delta={delta*180/math.pi:6.2f}deg Ld={self.lookahead_distance:.2f}m cte={cte:.3f}m'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
