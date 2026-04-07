#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Pure Pursuit Driver Node with Adaptive Lookahead for F1TENTH Gym ROS2

Adaptive lookahead 거리를 사용하는 Pure Pursuit 제어기
CSV 파일에서 경로와 각 waypoint별 lookahead 거리(ld_m)를 로드하고,
Odometry를 구독하여 제어 명령을 발행합니다.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
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


def nearest_point_idx_full(pos: np.ndarray, traj: np.ndarray) -> int:
    """Find nearest waypoint index in entire trajectory (fallback for large cte)"""
    d = np.linalg.norm(traj - pos, axis=1)
    return int(np.argmin(d))


def nearest_point_idx_window_heading(
    pos: np.ndarray,
    traj: np.ndarray,
    headings: np.ndarray,
    yaw: float,
    i0: int,
    win: int,
    max_heading_err: float,
) -> int:
    """Find nearest waypoint index in window with heading filter."""
    n_points = len(traj)
    idxs = (np.arange(i0, i0 + win) % n_points)
    dists = np.linalg.norm(traj[idxs] - pos, axis=1)
    heading_err = np.abs(np.arctan2(np.sin(yaw - headings[idxs]), np.cos(yaw - headings[idxs])))
    mask = heading_err <= max_heading_err
    if np.any(mask):
        masked = np.where(mask, dists, float('inf'))
        return int(idxs[int(np.argmin(masked))])
    return int(idxs[int(np.argmin(dists))])


def nearest_point_idx_full_heading(
    pos: np.ndarray,
    traj: np.ndarray,
    headings: np.ndarray,
    yaw: float,
    max_heading_err: float,
) -> int:
    """Find nearest waypoint index in entire trajectory with heading filter."""
    dists = np.linalg.norm(traj - pos, axis=1)
    heading_err = np.abs(np.arctan2(np.sin(yaw - headings), np.cos(yaw - headings)))
    mask = heading_err <= max_heading_err
    if np.any(mask):
        masked = np.where(mask, dists, float('inf'))
        return int(np.argmin(masked))
    return int(np.argmin(dists))


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
    """Pure Pursuit Driver Node with Adaptive Lookahead"""
    
    def __init__(self):
        super().__init__('pp_adaptive')
        
        # 파라미터 선언
        self.declare_parameter('raceline_csv', '')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('lookahead_topic', '/controller/lookahead')
        self.declare_parameter('wheelbase', 0.3302)  # F1TENTH Gym default
        self.declare_parameter('delta_max_deg', 24.0)  # F1TENTH Gym default
        self.declare_parameter('lookahead_fallback', 1.0)  # Fallback lookahead if ld_m column missing (m)
        self.declare_parameter('control_hz', 50.0)
        self.declare_parameter('search_window', 300)
        self.declare_parameter('heading_filter', True)  # RL 비교를 위해 기본 활성화
        self.declare_parameter('heading_max_error_deg', 90.0)
        
        # 속도 제약 파라미터
        self.declare_parameter('a_lat_max', 7.5)  # 최대 횡가속도 (m/s²)
        self.declare_parameter('use_steer_slowdown', True)
        self.declare_parameter('slowdown_gain', 0.5)
        self.declare_parameter('v_floor', 1.0)  # 최저 속도 (m/s)
        
        # 파라미터 가져오기
        raceline_csv = self.get_parameter('raceline_csv').value
        if not raceline_csv or not os.path.isabs(raceline_csv):
            # 상대 경로인 경우 패키지 디렉토리 기준으로 변환
            package_share = get_package_share_directory('pp_adaptive')
            if os.path.isabs(raceline_csv):
                # 절대 경로인 경우 그대로 사용
                pass
            else:
                # 상대 경로인 경우 racelines 디렉토리 기준
                raceline_csv = os.path.join(package_share, 'racelines', raceline_csv)
            
            if not os.path.exists(raceline_csv):
                # 기본값 시도
                default_csv = os.path.join(package_share, 'racelines', 'Budapest_raceline_vehicleaware.csv')
                if os.path.exists(default_csv):
                    raceline_csv = default_csv
                    self.get_logger().warn(f'Using default raceline: {raceline_csv}')
                else:
                    self.get_logger().error(f'Raceline CSV not found: {raceline_csv}')
                    raise FileNotFoundError(f'Raceline CSV not found: {raceline_csv}')
        
        self.wheelbase = self.get_parameter('wheelbase').value
        self.delta_max = self.get_parameter('delta_max_deg').value * math.pi / 180.0
        self.lookahead_fallback = self.get_parameter('lookahead_fallback').value
        self.search_window = self.get_parameter('search_window').value
        self.heading_filter = self.get_parameter('heading_filter').value
        self.heading_max_error = self.get_parameter('heading_max_error_deg').value * math.pi / 180.0
        self.a_lat_max = self.get_parameter('a_lat_max').value
        self.use_steer_slowdown = self.get_parameter('use_steer_slowdown').value
        self.slowdown_gain = self.get_parameter('slowdown_gain').value
        self.v_floor = self.get_parameter('v_floor').value
        
        # 경로 로드 (ld_m 컬럼 포함)
        self.load_raceline(raceline_csv)
        
        # 현재 상태
        self.current_pose = None  # (x, y, yaw)
        self.current_velocity = 0.0
        self.last_ref_idx = 0
        
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
        lookahead_topic = self.get_parameter('lookahead_topic').value
        self.lookahead_pub = None
        if lookahead_topic:
            self.lookahead_pub = self.create_publisher(Float32, lookahead_topic, 10)
        
        # 제어 타이머
        control_period = 1.0 / self.get_parameter('control_hz').value
        self.control_timer = self.create_timer(control_period, self.control_callback)
        
        self.get_logger().info(f'Pure Pursuit Driver with Adaptive Lookahead initialized')
        self.get_logger().info(f'  Raceline: {raceline_csv} ({len(self.ref_xy)} waypoints)')
        self.get_logger().info(f'  Using adaptive lookahead: {self.has_adaptive_lookahead}')
        if self.has_adaptive_lookahead:
            self.get_logger().info(f'  Lookahead range: [{np.min(self.ref_ld):.2f}, {np.max(self.ref_ld):.2f}] m')
        else:
            self.get_logger().info(f'  Fallback lookahead: {self.lookahead_fallback} m')
        self.get_logger().info(
            f'  Heading filter: {self.heading_filter} (max_err={self.heading_max_error * 180.0 / math.pi:.1f} deg)')
        self.get_logger().info(f'  Wheelbase: {self.wheelbase} m')
        self.get_logger().info(f'  Max steering: {self.delta_max * 180.0 / math.pi:.1f} deg')
        if self.lookahead_pub is not None:
            self.get_logger().info(f'  Lookahead topic: {lookahead_topic}')
    
    def load_raceline(self, csv_path: str):
        """Load raceline from CSV file (with ld_m column support).

        Supported formats:
        1) Header-based CSV with x_m/y_m and optional v_mps|vx_mps, ld_m
        2) Header-less numeric CSV used by rl_f1tenth (e.g., Budapest_optimal.csv)
        """
        if not os.path.exists(csv_path):
            raise FileNotFoundError(f'Raceline CSV not found: {csv_path}')

        has_v = False
        has_ld = False
        has_heading = False
        loaded_with_header = False

        # 1) Header-based CSV parsing (x_m/y_m + optional v_mps|vx_mps, ld_m)
        with open(csv_path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            fieldnames = reader.fieldnames or []
            if 'x_m' in fieldnames and 'y_m' in fieldnames:
                loaded_with_header = True
                x_list, y_list, v_list, ld_list = [], [], [], []
                heading_list = []
                speed_key = 'v_mps' if 'v_mps' in fieldnames else ('vx_mps' if 'vx_mps' in fieldnames else None)
                heading_key = (
                    'heading'
                    if 'heading' in fieldnames
                    else ('psi_rad' if 'psi_rad' in fieldnames else ('theta_rad' if 'theta_rad' in fieldnames else None))
                )
                has_v = speed_key is not None
                has_ld = 'ld_m' in fieldnames
                has_heading = heading_key is not None

                for row in reader:
                    x_raw = row.get('x_m', '')
                    y_raw = row.get('y_m', '')
                    if x_raw == '' or y_raw == '':
                        continue
                    x_list.append(float(x_raw))
                    y_list.append(float(y_raw))

                    if has_v:
                        v_raw = row.get(speed_key, '')
                        v_list.append(float(v_raw) if v_raw not in ('', None) else 3.0)
                    if has_ld:
                        ld_raw = row.get('ld_m', '')
                        ld_list.append(float(ld_raw) if ld_raw not in ('', None) else self.lookahead_fallback)
                    if has_heading:
                        h_raw = row.get(heading_key, '')
                        heading_list.append(float(h_raw) if h_raw not in ('', None) else 0.0)

                if len(x_list) == 0:
                    raise ValueError(f'No waypoint rows found in {csv_path}')

                self.ref_xy = np.column_stack([x_list, y_list]).astype(float)
                if has_v:
                    self.ref_v = np.array(v_list, dtype=float)
                else:
                    self.ref_v = np.ones(len(self.ref_xy), dtype=float) * 3.0

                if has_ld:
                    self.ref_ld = np.array(ld_list, dtype=float)
                else:
                    self.ref_ld = np.ones(len(self.ref_xy), dtype=float) * self.lookahead_fallback
                if has_heading:
                    self.ref_heading = np.array(heading_list, dtype=float)
                else:
                    self.ref_heading = None

        # 2) Header-less numeric CSV parsing (rl_f1tenth waypoint formats)
        if not loaded_with_header:
            data = np.loadtxt(csv_path, delimiter=',', comments='#')
            if data.ndim == 1:
                data = data.reshape(1, -1)
            if data.ndim != 2 or data.shape[1] < 2:
                raise ValueError(f'Invalid numeric waypoint CSV shape: {data.shape} ({csv_path})')

            n_cols = data.shape[1]

            # rl_f1tenth common formats:
            # - [x, y, psi, kappa, v] (5 cols)
            # - [s, x, y, psi, kappa, v, ax, heading, ld] (9 cols)
            if n_cols >= 9:
                x_col, y_col, heading_col, v_col, ld_col = 1, 2, 3, 5, 8
            elif n_cols >= 6:
                x_col, y_col, heading_col, v_col, ld_col = 1, 2, 3, 5, None
            elif n_cols >= 5:
                x_col, y_col, heading_col, v_col, ld_col = 0, 1, 2, 4, None
            else:
                x_col, y_col, heading_col, v_col, ld_col = 0, 1, None, None, None

            self.ref_xy = data[:, [x_col, y_col]].astype(float)

            if v_col is not None and v_col < n_cols:
                self.ref_v = data[:, v_col].astype(float)
                has_v = True
            else:
                self.ref_v = np.ones(len(self.ref_xy), dtype=float) * 3.0
                has_v = False

            if heading_col is not None and heading_col < n_cols:
                self.ref_heading = data[:, heading_col].astype(float)
                has_heading = True
            else:
                self.ref_heading = None
                has_heading = False

            if ld_col is not None and ld_col < n_cols:
                self.ref_ld = data[:, ld_col].astype(float)
                has_ld = True
            else:
                self.ref_ld = np.ones(len(self.ref_xy), dtype=float) * self.lookahead_fallback
                has_ld = False

        if not has_heading:
            # Fallback: estimate heading from local tangent of the raceline polyline.
            ref_next = np.roll(self.ref_xy, -1, axis=0)
            tangent = ref_next - self.ref_xy
            self.ref_heading = np.arctan2(tangent[:, 1], tangent[:, 0]).astype(float)

        if not has_v:
            self.get_logger().warn('Speed column not found, using default speed 3.0 m/s')
        if not has_ld:
            self.get_logger().warn(
                f'ld_m column not found in CSV. Using fallback lookahead: {self.lookahead_fallback} m')

        # Validate lookahead values
        self.has_adaptive_lookahead = has_ld
        if np.any(self.ref_ld <= 0):
            self.get_logger().warn('Found non-positive lookahead values. Replacing with fallback.')
            self.ref_ld = np.maximum(self.ref_ld, self.lookahead_fallback)

        self.get_logger().info(f'Loaded {len(self.ref_xy)} waypoints from {csv_path}')
        if loaded_with_header:
            self.get_logger().info('  Waypoint format: header-based')
        else:
            self.get_logger().info('  Waypoint format: numeric (rl_f1tenth-compatible)')
        self.get_logger().info(
            f'  Heading source: {"csv" if has_heading else "tangent (derived)"}')
        if has_ld:
            self.get_logger().info(f'  Adaptive lookahead: {len(self.ref_ld)} values')
    
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
    
    def apply_speed_constraints(self, v_target: float, delta: float, cte: float = 0.0) -> float:
        """Apply speed constraints — same as pp_fixed baseline (paper does not define online speed control).

        Steer-based slowdown + lateral acceleration cap only.
        CTE-based and curvature-based modifications removed to match paper scope.
        """
        # Steer-based slowdown (identical to pp_fixed)
        if self.use_steer_slowdown:
            steer_ratio = abs(delta) / max(1e-6, self.delta_max)
            v_target *= max(0.0, 1.0 - self.slowdown_gain * steer_ratio)

        # Lateral acceleration cap (identical to pp_fixed)
        v_lat_cap = lateral_speed_limit(self.wheelbase, delta, self.a_lat_max, v_cap=1e9)
        v_target = min(v_target, v_lat_cap)

        # Floor
        v_target = max(self.v_floor, v_target)
        return v_target
    
    def control_callback(self):
        """Main control loop"""
        if self.current_pose is None:
            return
        
        x, y, yaw = self.current_pose
        
        # 가장 가까운 웨이포인트 찾기
        # 루프 경로에서 반대편 waypoint 선택 방지를 위해 방향성 고려한 검색 사용
        pos = np.array([x, y], dtype=float)
        N = len(self.ref_xy)
        
        # 이전 waypoint 인덱스를 기준으로 방향성 고려한 검색
        if self.last_ref_idx is not None and self.last_ref_idx >= 0:
            # 속도에 따른 윈도우 크기 조정 (속도가 빠를수록 더 큰 윈도우)
            base_window = max(150, N // 3)  # 기본 윈도우 크기 증가
            current_speed = max(0.5, abs(self.current_velocity))
            if current_speed > 10.0:
                window_size = int(base_window * 2.5)  # 고속: 2.5배
            elif current_speed > 5.0:
                window_size = int(base_window * 2.0)  # 중속: 2배
            else:
                window_size = int(base_window * 1.5)  # 저속: 1.5배
            
            # 최대 윈도우 크기 제한 (전체의 60%)
            window_size = min(window_size, int(N * 0.6))
            
            # 윈도우 검색 (순방향 우선, 필요 시 heading filter 적용)
            if self.heading_filter:
                i_window = nearest_point_idx_window_heading(
                    pos, self.ref_xy, self.ref_heading, yaw,
                    self.last_ref_idx, window_size, self.heading_max_error)
            else:
                i_window = nearest_point_idx_window(pos, self.ref_xy, self.last_ref_idx, window_size)
            dist_window = float(np.linalg.norm(self.ref_xy[i_window] - pos))
            
            # Paper (Sukhil & Behl, 2021): "finds the nearest point to its base link"
            # Use window result; fall back to full search only when CTE is large.
            i = i_window
            if dist_window > 5.0:
                # Large CTE: full search to avoid stale index
                if self.heading_filter:
                    i_full = nearest_point_idx_full_heading(
                        pos, self.ref_xy, self.ref_heading, yaw, self.heading_max_error)
                else:
                    i_full = nearest_point_idx_full(pos, self.ref_xy)
                dist_full = float(np.linalg.norm(self.ref_xy[i_full] - pos))
                if dist_full < dist_window:
                    i = i_full
        else:
            # 첫 실행 시 전체 검색
            if self.heading_filter:
                i = nearest_point_idx_full_heading(
                    pos, self.ref_xy, self.ref_heading, yaw, self.heading_max_error)
            else:
                i = nearest_point_idx_full(pos, self.ref_xy)
        
        # 인덱스 점프가 너무 크면 제한 (안정성 향상)
        if self.last_ref_idx is not None and self.last_ref_idx >= 0:
            idx_jump = abs(i - self.last_ref_idx)
            idx_jump = min(idx_jump, N - idx_jump)  # 루프 경로 고려
            # 한 번에 전체의 30% 이상 점프하면 제한
            if idx_jump > N * 0.3:
                # 이전 인덱스에서 순방향으로만 진행
                forward_idx = (self.last_ref_idx + 1) % N
                backward_idx = (self.last_ref_idx - 1) % N
                dist_forward = float(np.linalg.norm(self.ref_xy[forward_idx] - pos))
                dist_backward = float(np.linalg.norm(self.ref_xy[backward_idx] - pos))
                if dist_forward < dist_backward:
                    i = forward_idx
                else:
                    i = backward_idx
        
        self.last_ref_idx = i
        
        # 현재 위치에서 선택한 waypoint까지의 거리 (cte)
        dist_to_waypoint = float(np.linalg.norm(self.ref_xy[i] - pos))
        
        # Paper Algorithm 1 (Sukhil & Behl, 2021):
        # Online controller uses the pre-computed offline label directly.
        # No runtime modification — the offline assignment already accounts for
        # map geometry, speed, and collision safety at each waypoint position.
        Ld = float(self.ref_ld[i])
        
        # Lookahead 포인트 찾기
        lookahead_pt, _ = lookahead_point_from_idx(self.ref_xy, i, Ld)
        
        # Pure Pursuit 조향각 계산
        delta, alpha = pure_pursuit_delta(
            x, y, yaw, lookahead_pt,
            self.wheelbase, Ld, self.delta_max
        )
        
        # 목표 속도 결정 (CSV의 v_mps 사용)
        v_target = float(self.ref_v[i])
        # CTE 기반 속도 제약 적용
        v_target = self.apply_speed_constraints(v_target, delta, cte=dist_to_waypoint)
        
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
        if self.lookahead_pub is not None:
            lookahead_msg = Float32()
            lookahead_msg.data = Ld
            self.lookahead_pub.publish(lookahead_msg)
        
        # 주기적 로깅 (1초마다)
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        
        if self._log_counter % int(self.get_parameter('control_hz').value) == 0:
            cte = float(np.linalg.norm(self.ref_xy[i] - pos))
            self.get_logger().info(
                f'idx={i:4d} v_tgt={v_target:5.2f} v_act={self.current_velocity:5.2f} '
                f'delta={delta*180/math.pi:6.2f}deg Ld={Ld:.2f}m cte={cte:.3f}m'
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
