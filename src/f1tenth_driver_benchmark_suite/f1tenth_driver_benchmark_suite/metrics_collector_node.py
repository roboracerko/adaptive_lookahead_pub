#!/usr/bin/env python3
"""
Metrics Collector Node for F1TENTH Driver Evaluation.

Collects real-time metrics during driving:
- Path tracking accuracy (RMS lateral error, max lateral error)
- Performance (lap time, average speed)
- Steering stability (RMS steering rate, lookahead variance)
- Safety (off-track ratio, e-stop count)

Publishes lap summaries and run summaries as JSON strings.
"""

import os
import json
import math
import time
import threading
from typing import Optional
import rclpy
from rclpy.node import Node
try:
    import psutil
except ImportError:
    psutil = None  # Fallback if psutil is not available
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, Bool, String
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point

from .online_stats import RunningRMS, RunningMaxAbs, WelfordVar, RunningMean, RunningMax, RunningMin
from .geometry import point_to_path_distance, compute_path_length, is_point_inside_track_boundary, check_track_boundary_violation, project_point_to_polyline
from .lap_manager import LapManager
from .grid_lap_manager import GridLapManager
from .progress_lap_manager import ProgressLapManager, ProgressLapConfig
from .track import Track
from .exporters import MetricsExporter


class MetricsCollector(Node):
    """Main metrics collection node."""
    
    def __init__(self):
        super().__init__('metrics_collector')
        
        # ========== Parameters ==========
        # Topic names
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.drive_topic = self.declare_parameter('drive_topic', '/drive').value
        self.lookahead_topic = self.declare_parameter('lookahead_topic', '/controller/lookahead').value
        self.estop_topic = self.declare_parameter('estop_topic', '/estop').value
        self.centerline_topic = self.declare_parameter('centerline_topic', '/track/centerline').value
        
        # Track configuration
        self.track_length_m = float(self.declare_parameter('track_length_m', 0.0).value)
        self.track_half_width_m = float(self.declare_parameter('track_half_width_m', 1.0).value)
        self.offtrack_margin_m = float(self.declare_parameter('offtrack_margin_m', 0.10).value)
        
        # Centerline file (optional, alternative to topic)
        centerline_file = self.declare_parameter('centerline_file', '').value
        
        # Start/finish line
        # If centerline is loaded, use first and last waypoints as start line
        # Otherwise use provided parameters
        p1 = self.declare_parameter('start_line_p1', [0.0, 0.0]).value
        p2 = self.declare_parameter('start_line_p2', [1.0, 0.0]).value
        start_p1 = (float(p1[0]), float(p1[1]))
        start_p2 = (float(p2[0]), float(p2[1]))
        
        # ========== State: Track ==========
        self.track = Track()
        
        if centerline_file and os.path.exists(centerline_file):
            self.get_logger().info(f"Loading centerline from file: {centerline_file}")
            if self.track.load_centerline_from_csv(centerline_file):
                self.get_logger().info(f"Loaded {len(self.track.centerline)} centerline points")
                if self.track.length > 0:
                    self.track_length_m = self.track.length
                
                # Auto-detect start line from raceline
                # Use first waypoint and create a perpendicular line segment
                if len(self.track.centerline) > 1:
                    # Get first waypoint
                    first_wp = self.track.centerline[0]
                    second_wp = self.track.centerline[1]
                    
                    # Compute direction from first to second waypoint
                    dx = second_wp[0] - first_wp[0]
                    dy = second_wp[1] - first_wp[1]
                    length = math.hypot(dx, dy)
                    if length > 1e-6:
                        # Normalize direction
                        dir_x = dx / length
                        dir_y = dy / length
                        # Perpendicular direction (rotate 90 degrees counter-clockwise)
                        perp_x = -dir_y
                        perp_y = dir_x
                        # Create start line segment perpendicular to track direction
                        # Length of start line: 5 meters (increased to avoid missing crossings)
                        line_length = 5.0
                        start_p1 = (first_wp[0] - perp_x * line_length / 2, first_wp[1] - perp_y * line_length / 2)
                        start_p2 = (first_wp[0] + perp_x * line_length / 2, first_wp[1] + perp_y * line_length / 2)
                        self.get_logger().info(f"Auto-detected start line from raceline:")
                        self.get_logger().info(f"  First waypoint: ({first_wp[0]:.2f}, {first_wp[1]:.2f})")
                        self.get_logger().info(f"  Start line: {start_p1} -> {start_p2}")
                        self.get_logger().info(f"  Start line length: {line_length:.2f}m")
                    else:
                        self.get_logger().warn("Failed to compute start line direction from raceline, using provided parameters")
                else:
                    self.get_logger().warn("Raceline has insufficient points, using provided start line parameters")
        
        self.min_lap_time_s = float(self.declare_parameter('min_lap_time_s', 3.0).value)
        
        # Lap detector type selection
        self.lap_detector_type = self.declare_parameter('lap_detector_type', 'progress').value  # 'progress', 'line', 'grid'
        
        # Progress-based lap detector parameters
        self.progress_start_zone_s = float(self.declare_parameter('progress_start_zone_s', 5.0).value)
        self.progress_wrap_margin_s = float(self.declare_parameter('progress_wrap_margin_s', 3.0).value)
        self.progress_cooldown_s = float(self.declare_parameter('progress_cooldown_s', 1.0).value)
        self.progress_min_progress_s = float(self.declare_parameter('progress_min_progress_s', 0.10).value)
        
        # Lap count configuration
        self.min_laps = int(self.declare_parameter('min_laps', 0).value)  # Minimum laps before evaluation (0 = no minimum)
        self.max_laps = int(self.declare_parameter('max_laps', 0).value)  # Maximum laps (0 = unlimited)
        self.auto_shutdown = self.declare_parameter('auto_shutdown', False).value  # Auto shutdown after max_laps
        
        # Output configuration
        output_dir = self.declare_parameter('output_dir', '/tmp/f1tenth_metrics').value
        self.exporter = MetricsExporter(output_dir)
        
        # Centerline file already declared above (line 52), no need to redeclare
        # centerline_file is already available from line 52
        
        # Frame configuration
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.use_tf = self.declare_parameter('use_tf', False).value
        
        # ========== State: Track ==========
        self.track = Track()
        
        if centerline_file and os.path.exists(centerline_file):
            self.get_logger().info(f"Loading centerline from file: {centerline_file}")
            if self.track.load_centerline_from_csv(centerline_file):
                self.get_logger().info(f"Loaded {len(self.track.centerline)} centerline points")
                if self.track.length > 0:
                    self.track_length_m = self.track.length
            else:
                self.get_logger().warn("Failed to load centerline from file")
        
        # ========== State: Lap Management ==========
        # Select lap detector based on type parameter
        if self.lap_detector_type == 'progress':
            # Progress-based lap detection (recommended)
            if self.track.is_valid() and len(self.track.centerline) >= 2:
                # Compute start_s from start line midpoint if available
                start_s = 0.0
                if start_p1 and start_p2:
                    start_mid = ((start_p1[0] + start_p2[0]) / 2.0, (start_p1[1] + start_p2[1]) / 2.0)
                    try:
                        proj = project_point_to_polyline(
                            start_mid, self.track.centerline, compute_signed=False
                        )
                        start_s = proj.s
                    except Exception as e:
                        self.get_logger().warn(f"Failed to compute start_s from start line: {e}, using 0.0")
                
                cfg = ProgressLapConfig(
                    min_lap_time_s=self.min_lap_time_s,
                    start_s=start_s,
                    start_zone_s=self.progress_start_zone_s,
                    wrap_margin_s=self.progress_wrap_margin_s,
                    cooldown_s=self.progress_cooldown_s,
                    min_progress_s=self.progress_min_progress_s,
                )
                self.lap_manager = ProgressLapManager(self.track.centerline, cfg, logger=self.get_logger())
                self.get_logger().info(f"Progress Lap Manager initialized:")
                self.get_logger().info(f"  Centerline points: {len(self.track.centerline)}")
                self.get_logger().info(f"  Track length: {self.lap_manager.length:.2f}m")
                self.get_logger().info(f"  Start s: {start_s:.2f}m")
                self.get_logger().info(f"  Start zone: ±{self.progress_start_zone_s:.2f}m")
                self.get_logger().info(f"  Wrap margin: {self.progress_wrap_margin_s:.2f}m")
                self.get_logger().info(f"  Min lap time: {self.min_lap_time_s}s")
            else:
                # Fallback to line crossing if centerline invalid
                self.get_logger().warn("Progress lap requested but centerline invalid; falling back to LapManager")
                self.lap_manager = LapManager(
                    start_p1, start_p2,
                    min_lap_time_s=self.min_lap_time_s,
                    direction_check=True,
                    crossing_threshold_m=0.5
                )
        elif self.lap_detector_type == 'grid':
            # Grid-based lap detection (legacy)
            if self.track.is_valid() and len(self.track.centerline) > 0:
                self.lap_manager = GridLapManager(
                    centerline=self.track.centerline,
                    start_line_p1=start_p1,
                    start_line_p2=start_p2,
                    grid_resolution=0.5,
                    start_line_width=2.0,
                    min_lap_time_s=self.min_lap_time_s,
                    coverage_threshold=0.7
                )
                self.get_logger().info(f"Grid Lap Manager initialized:")
                self.get_logger().info(f"  Start line: {start_p1} -> {start_p2}")
                self.get_logger().info(f"  Grid resolution: 0.5m")
                self.get_logger().info(f"  Start line width: 2.0m")
                self.get_logger().info(f"  Min lap time: {self.min_lap_time_s}s")
                self.get_logger().info(f"  Coverage threshold: 70%")
                self.get_logger().info(f"  Total track cells: {self.lap_manager.total_track_cells}")
            else:
                self.get_logger().warn("Grid lap requested but centerline invalid; falling back to LapManager")
                self.lap_manager = LapManager(
                    start_p1, start_p2,
                    min_lap_time_s=self.min_lap_time_s,
                    direction_check=True,
                    crossing_threshold_m=0.5
                )
        else:
            # 'line' - Line crossing based (fallback)
            self.lap_manager = LapManager(
                start_p1, start_p2,
                min_lap_time_s=self.min_lap_time_s,
                direction_check=True,
                crossing_threshold_m=0.5
            )
            self.get_logger().info(f"Line Crossing Lap Manager initialized (fallback)")
        
        # ========== State: Running Statistics (per lap) ==========
        self._reset_lap_stats()
        
        # ========== State: Global Counters ==========
        self.estop_count = 0
        self.all_laps = []  # Store all lap summaries
        
        # ========== State: Latest Control Values ==========
        self.last_delta: Optional[float] = None
        self.last_delta_stamp: Optional[float] = None
        self.last_lookahead: Optional[float] = None
        self.delta_history = []  # For steering rate calculation
        
        # ========== State: Pose History ==========
        self.prev_pos: Optional[tuple] = None
        self.prev_stamp: Optional[float] = None
        self.prev_yaw: Optional[float] = None
        self.total_dist = 0.0
        self.offtrack_dist = 0.0
        self.prev_offtrack = False
        
        # ========== State: Vehicle Trajectory Logging ==========
        self.trajectory = []  # List of (x, y, t, speed, lateral_error)
        self.trajectory_log_enabled = True
        
        # ========== State: Safety Monitoring ==========
        self.reverse_driving_detected = False
        self.reverse_detection_enabled = self.declare_parameter('reverse_detection_enabled', True).value
        self.reverse_dot_threshold = float(self.declare_parameter('reverse_dot_threshold', -0.5).value)
        self.reverse_min_movement_m = float(self.declare_parameter('reverse_min_movement_m', 0.02).value)
        self.reverse_min_speed_mps = float(self.declare_parameter('reverse_min_speed_mps', 0.5).value)
        self.reverse_grace_period_s = float(self.declare_parameter('reverse_grace_period_s', 2.0).value)
        self.reverse_consecutive_samples = int(self.declare_parameter('reverse_consecutive_samples', 8).value)
        if self.reverse_consecutive_samples < 1:
            self.reverse_consecutive_samples = 1
        self.reverse_violation_count = 0
        self.first_odom_time: Optional[float] = None
        self.is_shutting_down = False  # Emergency shutdown 플래그 (중복 호출 방지)
        self.offtrack_threshold_ratio = float(self.declare_parameter('offtrack_threshold_ratio', 0.3).value)  # 30% 이상 off-track 시 중지
        self.offtrack_duration_threshold = float(self.declare_parameter('offtrack_duration_threshold', 5.0).value)  # 5초 이상 off-track 시 중지
        self.offtrack_start_time: Optional[float] = None
        self.consecutive_offtrack_count = 0
        self.max_consecutive_offtrack = int(self.declare_parameter('max_consecutive_offtrack', 100).value)  # 연속 100 샘플 off-track 시 중지
        self.auto_shutdown_on_error = self.declare_parameter('auto_shutdown_on_error', True).value
        
        # Track boundary monitoring
        self.track_boundary_check_enabled = self.declare_parameter('track_boundary_check_enabled', True).value
        self.track_boundary_violation_threshold = float(self.declare_parameter('track_boundary_violation_threshold', 0.5).value)  # 0.5m 이상 경계 벗어나면 즉시 중지
        self.consecutive_boundary_violations = 0
        self.max_consecutive_boundary_violations = int(self.declare_parameter('max_consecutive_boundary_violations', 10).value)  # 연속 10 샘플 경계 위반 시 중지
        
        # ========== Publishers ==========
        self.pub_lap = self.create_publisher(String, '/metrics/lap_summary', 10)
        self.pub_run = self.create_publisher(String, '/metrics/run_summary', 10)
        self.pub_lat = self.create_publisher(Float32, '/metrics/debug/lateral_error', 10)
        self.pub_off = self.create_publisher(Bool, '/metrics/debug/offtrack', 10)
        
        # ========== Subscriptions ==========
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        self.create_subscription(AckermannDriveStamped, self.drive_topic, self.on_drive, 50)
        
        if self.lookahead_topic:
            self.create_subscription(Float32, self.lookahead_topic, self.on_lookahead, 50)
        
        if self.estop_topic:
            self.create_subscription(Bool, self.estop_topic, self.on_estop, 10)
        
        if self.centerline_topic:
            self.create_subscription(Path, self.centerline_topic, self.on_centerline, 10)
        
        self.get_logger().info("Metrics collector initialized")
        self.get_logger().info(f"  Odom topic: {self.odom_topic}")
        self.get_logger().info(f"  Drive topic: {self.drive_topic}")
        self.get_logger().info(f"  Output dir: {output_dir}")
        if self.min_laps > 0:
            self.get_logger().info(f"  Minimum laps: {self.min_laps}")
        if self.max_laps > 0:
            self.get_logger().info(f"  Maximum laps: {self.max_laps} (auto_shutdown: {self.auto_shutdown})")
        else:
            self.get_logger().info(f"  Lap limit: Unlimited")
    
    def _reset_lap_stats(self):
        """Reset statistics for new lap."""
        self.rms_lat = RunningRMS()
        self.max_lat = RunningMaxAbs()
        self.rms_heading_error = RunningRMS()
        self.max_heading_error = RunningMaxAbs()
        self.rms_steer_rate = RunningRMS()
        self.rms_steer_jerk = RunningRMS()
        self.rms_yaw_rate = RunningRMS()
        self.max_yaw_rate = RunningMaxAbs()
        self.rms_yaw_accel = RunningRMS()
        self.var_yaw_rate = WelfordVar()
        self.rms_longitudinal_accel = RunningRMS()
        self.rms_longitudinal_jerk = RunningRMS()
        self.var_lookahead = WelfordVar()
        self.mean_speed = RunningMean()
        self.max_speed = RunningMax()  # 최고 속도 추적
        self.min_speed = RunningMin()  # 최저 속도 추적
        self.lap_dist = 0.0
        self.lap_offtrack_dist = 0.0
        self.lap_start_stamp = None
        self.delta_history = []
        self.metric_prev_stamp: Optional[float] = None
        self.metric_prev_yaw: Optional[float] = None
        self.metric_prev_speed: Optional[float] = None
        self.metric_prev_yaw_rate: Optional[float] = None
        self.metric_prev_steer_rate: Optional[float] = None
        self.metric_prev_longitudinal_accel: Optional[float] = None
    
    def on_drive(self, msg: AckermannDriveStamped):
        """Callback for drive commands."""
        delta = float(msg.drive.steering_angle)
        stamp = msg.header.stamp
        msg_stamp = stamp.sec + stamp.nanosec * 1e-9
        recv_stamp = time.monotonic()

        # Prefer the message timestamp when it is sane and monotonic. Fall back
        # to local receive time when the driver clock is zeroed or jumps.
        sample_stamp = msg_stamp if msg_stamp > 0.0 else recv_stamp
        if self.last_delta_stamp is not None:
            dt_msg = sample_stamp - self.last_delta_stamp
            if not (1e-6 < dt_msg < 1.0):
                sample_stamp = recv_stamp

        prev_delta = self.last_delta
        prev_stamp = self.last_delta_stamp

        self.last_delta = delta
        self.last_delta_stamp = sample_stamp
        self.delta_history.append((self.last_delta_stamp, self.last_delta))
        # Keep only recent history (last 100 samples)
        if len(self.delta_history) > 100:
            self.delta_history.pop(0)

        if prev_delta is None or prev_stamp is None:
            return

        dt = self.last_delta_stamp - prev_stamp
        if not (1e-6 < dt < 1.0):
            return

        steer_rate = (delta - prev_delta) / dt
        self.rms_steer_rate.update(steer_rate)
        if self.metric_prev_steer_rate is not None:
            steer_jerk = (steer_rate - self.metric_prev_steer_rate) / dt
            self.rms_steer_jerk.update(steer_jerk)
        self.metric_prev_steer_rate = steer_rate
    
    def on_lookahead(self, msg: Float32):
        """Callback for lookahead distance."""
        self.last_lookahead = float(msg.data)
        if not math.isnan(self.last_lookahead):
            self.var_lookahead.update(self.last_lookahead)
    
    def on_estop(self, msg: Bool):
        """Callback for e-stop events."""
        if bool(msg.data):
            self.estop_count += 1
            self.get_logger().warn(f"E-stop detected! Total count: {self.estop_count}")
    
    def on_centerline(self, msg: Path):
        """Callback for centerline path."""
        if self.track.load_centerline_from_path_msg(msg):
            self.get_logger().info(f"Loaded centerline from topic: {len(self.track.centerline)} points")
            if self.track.length > 0:
                self.track_length_m = self.track.length
    
    def _compute_steering_rate(self, current_time: float) -> Optional[float]:
        """Compute steering rate from the two most recent drive messages.

        Uses drive-message timestamps only (same clock domain) to avoid
        cross-domain comparison failures when the gym bridge odom clock and
        the driver wall clock differ.
        """
        if len(self.delta_history) < 2:
            return None

        t1, d1 = self.delta_history[-2]
        t2, d2 = self.delta_history[-1]
        dt = t2 - t1
        if 1e-6 < dt < 1.0:
            return (d2 - d1) / dt

        return None
    
    def _nearest_lateral_error(self, x: float, y: float) -> tuple[float, int]:
        """Compute signed lateral error and nearest segment index."""
        if not self.track.is_valid():
            return float("nan"), -1
        
        p = (x, y)
        signed_dist, _, seg_idx = point_to_path_distance(p, self.track.centerline)
        return signed_dist, seg_idx

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def _heading_error(self, yaw: float, seg_idx: int) -> float:
        """Heading error between vehicle yaw and nearest raceline segment heading."""
        if not self.track.is_valid():
            return float("nan")
        if seg_idx < 0 or seg_idx >= len(self.track.centerline) - 1:
            return float("nan")
        p0 = self.track.centerline[seg_idx]
        p1 = self.track.centerline[seg_idx + 1]
        path_heading = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        return self._wrap_to_pi(yaw - path_heading)
    
    def on_odom(self, msg: Odometry):
        """Main callback for odometry updates."""
        # 이미 종료 중이면 처리하지 않음
        if self.is_shutting_down:
            return
        
        # Debug: Log first few calls to confirm callback is working
        if not hasattr(self, '_odom_call_count'):
            self._odom_call_count = 0
        self._odom_call_count += 1
        if self._odom_call_count <= 5 or self._odom_call_count % 100 == 0:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.get_logger().info(f"📥 on_odom 호출 #{self._odom_call_count}: Position: ({x:.2f}, {y:.2f})")

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        
        # Extract speed
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)
        
        # Timestamp
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if self.first_odom_time is None:
            self.first_odom_time = t
        
        # Initialize lap start time
        if self.lap_manager.lap_start_time is None:
            self.lap_manager.lap_start_time = t
        
        # ========== Lateral Error ==========
        ey, nearest_seg_idx = self._nearest_lateral_error(x, y)
        if not math.isnan(ey):
            self.rms_lat.update(ey)
            self.max_lat.update(ey)
            self.pub_lat.publish(Float32(data=float(ey)))
        
        # ========== Raceline Heading Fidelity ==========
        heading_error = self._heading_error(yaw, nearest_seg_idx)
        if not math.isnan(heading_error):
            self.rms_heading_error.update(heading_error)
            self.max_heading_error.update(heading_error)
        
        # ========== Trajectory Logging ==========
        if self.trajectory_log_enabled:
            self.trajectory.append((x, y, t, speed, ey if not math.isnan(ey) else 0.0))
        
        # ========== Speed ==========
        self.mean_speed.update(speed)
        self.max_speed.update(speed)  # 최고 속도 추적
        self.min_speed.update(speed)  # 최저 속도 추적

        # ========== Dynamic Stability (Yaw/Acceleration/Jerk) ==========
        dt_metric = (t - self.metric_prev_stamp) if self.metric_prev_stamp is not None else 0.0
        yaw_rate = None
        longitudinal_accel = None

        if dt_metric > 1e-4:
            if self.metric_prev_yaw is not None:
                yaw_rate = self._wrap_to_pi(yaw - self.metric_prev_yaw) / dt_metric
                self.rms_yaw_rate.update(yaw_rate)
                self.max_yaw_rate.update(yaw_rate)
                self.var_yaw_rate.update(yaw_rate)
                if self.metric_prev_yaw_rate is not None:
                    yaw_accel = (yaw_rate - self.metric_prev_yaw_rate) / dt_metric
                    self.rms_yaw_accel.update(yaw_accel)

            if self.metric_prev_speed is not None:
                longitudinal_accel = (speed - self.metric_prev_speed) / dt_metric
                self.rms_longitudinal_accel.update(longitudinal_accel)
                if self.metric_prev_longitudinal_accel is not None:
                    longitudinal_jerk = (
                        longitudinal_accel - self.metric_prev_longitudinal_accel
                    ) / dt_metric
                    self.rms_longitudinal_jerk.update(longitudinal_jerk)

        # ========== Track Boundary Check (가장 우선) ==========
        # 주의: 트랙 경계 감지는 항상 활성화되어야 안전합니다
        # 단, 시작 위치(start zone)에서는 트랙 경계 위반을 허용합니다 (시작 위치가 정확하지 않을 수 있음)
        # 시작 위치 문제는 트랙 경계 감지 임계값 조정이나 시작 위치 수정으로 해결해야 합니다
        if self.track_boundary_check_enabled and self.track.is_valid():
            # Start zone에 있는지 확인 (ProgressLapManager 사용 시)
            in_start_zone = False
            if isinstance(self.lap_manager, ProgressLapManager):
                try:
                    # ProgressLapManager의 s 좌표를 사용하여 start zone 확인
                    # 먼저 현재 위치를 centerline에 투영하여 s 좌표 얻기
                    from .geometry import project_point_to_polyline
                    proj = project_point_to_polyline((x, y), self.track.centerline)
                    s = proj.s if proj else 0.0
                    in_start_zone = self.lap_manager._is_in_start_zone_s(s)
                except Exception as e:
                    # 오류 발생 시 start zone이 아니라고 가정
                    pass
            
            # Start zone이 아닐 때만 트랙 경계 감지 수행
            if not in_start_zone:
                is_violation, violation_dist = check_track_boundary_violation(
                    (x, y),
                    self.track.centerline,
                    self.track_half_width_m + self.offtrack_margin_m  # 경계 = half_width + margin
                )
                
                if is_violation:
                    self.consecutive_boundary_violations += 1
                    
                    if violation_dist >= self.track_boundary_violation_threshold:
                        # 즉시 중지: 경계를 임계값 이상 벗어남
                        self.get_logger().error("=" * 70)
                        self.get_logger().error("🚨 트랙 경계 위반! 시뮬레이션을 즉시 중지합니다.")
                        self.get_logger().error(f"  위치: ({x:.2f}, {y:.2f})")
                        self.get_logger().error(f"  경계 이탈 거리: {violation_dist:.2f}m")
                        self.get_logger().error(f"  임계값: {self.track_boundary_violation_threshold:.2f}m")
                        self.get_logger().error("=" * 70)
                        if self.auto_shutdown_on_error:
                            self._emergency_shutdown("TRACK_BOUNDARY_VIOLATION", {
                                "violation_distance_m": violation_dist,
                                "position": [x, y]
                            })
                        return
                
                if self.consecutive_boundary_violations >= self.max_consecutive_boundary_violations:
                    # 연속 위반 시 중지
                    self.get_logger().error("=" * 70)
                    self.get_logger().error("🚨 연속 트랙 경계 위반! 시뮬레이션을 중지합니다.")
                    self.get_logger().error(f"  연속 위반 샘플 수: {self.consecutive_boundary_violations}")
                    self.get_logger().error(f"  위치: ({x:.2f}, {y:.2f})")
                    self.get_logger().error(f"  경계 이탈 거리: {violation_dist:.2f}m")
                    self.get_logger().error("=" * 70)
                    if self.auto_shutdown_on_error:
                        self._emergency_shutdown("TRACK_BOUNDARY_CONSECUTIVE", {
                            "consecutive_violations": self.consecutive_boundary_violations,
                            "violation_distance_m": violation_dist,
                            "position": [x, y]
                        })
                    return
            else:
                self.consecutive_boundary_violations = 0
        
        # ========== Distance Accumulation ==========
        cur_pos = (x, y)
        if self.prev_pos is not None:
            dx = cur_pos[0] - self.prev_pos[0]
            dy = cur_pos[1] - self.prev_pos[1]
            ds = math.hypot(dx, dy)
            self.total_dist += ds
            self.lap_dist += ds
            
            # ========== Reverse Driving Detection ==========
            if self.reverse_detection_enabled:
                dot_product = None
                dt = (t - self.prev_stamp) if self.prev_stamp is not None else 0.0
                in_grace = (
                    self.first_odom_time is not None and
                    (t - self.first_odom_time) < self.reverse_grace_period_s
                )

                if (not in_grace) and ds > self.reverse_min_movement_m and dt > 1e-4:
                    speed_est = ds / dt
                    if speed_est >= self.reverse_min_speed_mps:
                        # Compute direction vector
                        direction_x = dx / ds
                        direction_y = dy / ds
                        # Compute forward direction from yaw
                        forward_x = math.cos(yaw)
                        forward_y = math.sin(yaw)
                        # Dot product: negative means reverse
                        dot_product = direction_x * forward_x + direction_y * forward_y
                        if dot_product < self.reverse_dot_threshold:
                            self.reverse_violation_count += 1
                        else:
                            self.reverse_violation_count = 0
                    else:
                        self.reverse_violation_count = 0
                else:
                    self.reverse_violation_count = 0

                if self.reverse_violation_count >= self.reverse_consecutive_samples:
                    if not self.reverse_driving_detected:
                        self.reverse_driving_detected = True
                        self.get_logger().error("=" * 70)
                        self.get_logger().error("🚨 역주행 감지! 시뮬레이션을 중지합니다.")
                        self.get_logger().error(f"  위치: ({x:.2f}, {y:.2f})")
                        self.get_logger().error(f"  Yaw: {yaw*180/math.pi:.1f} deg")
                        if dot_product is not None:
                            self.get_logger().error(f"  Direction dot: {dot_product:.3f}")
                        self.get_logger().error(
                            f"  연속 역주행 샘플: {self.reverse_violation_count}/{self.reverse_consecutive_samples}"
                        )
                        self.get_logger().error("=" * 70)
                    if self.auto_shutdown_on_error:
                        self._emergency_shutdown("REVERSE_DRIVING", {
                            "position": [x, y],
                            "yaw_rad": yaw,
                            "direction_dot": dot_product,
                            "reverse_samples": self.reverse_violation_count
                        })
                        return
            
            # ========== Off-track Detection (즉시 종료) ==========
            # Lateral error 기반 감지 (ey가 유효한 경우)
            off = False
            if not math.isnan(ey):
                off = abs(ey) > (self.track_half_width_m + self.offtrack_margin_m)
                
                # 디버깅: 주기적으로 lateral error 로그 출력 (매 100번째 업데이트마다)
                if int(t * 10) % 100 == 0:
                    self.get_logger().info(f"📍 위치: ({x:.2f}, {y:.2f}), Lateral Error: {ey:.3f}m, 허용범위: ±{self.track_half_width_m + self.offtrack_margin_m:.2f}m")
            
            # ey가 NaN인 경우 track boundary check 결과 사용
            if math.isnan(ey) and self.track_boundary_check_enabled and self.track.is_valid():
                # Track boundary check에서 이미 위반을 확인했으므로 여기서는 추가 처리 불필요
                # 하지만 off 플래그는 False로 유지 (track boundary check에서 이미 처리됨)
                pass
            
            if off:
                # 트랙을 벗어나면 즉시 종료
                self.get_logger().error("=" * 70)
                self.get_logger().error("🚨 트랙 이탈 감지! 시뮬레이션을 즉시 중지합니다.")
                self.get_logger().error(f"  위치: ({x:.2f}, {y:.2f})")
                self.get_logger().error(f"  Lateral Error: {ey:.2f}m")
                self.get_logger().error(f"  트랙 반폭: {self.track_half_width_m:.2f}m")
                self.get_logger().error(f"  여유 마진: {self.offtrack_margin_m:.2f}m")
                self.get_logger().error(f"  허용 범위: ±{self.track_half_width_m + self.offtrack_margin_m:.2f}m")
                self.get_logger().error(f"  Track valid: {self.track.is_valid()}")
                self.get_logger().error("=" * 70)
                if self.auto_shutdown_on_error:
                    self._emergency_shutdown("OFFTRACK_IMMEDIATE", {
                        "position": [x, y],
                        "lateral_error_m": ey if not math.isnan(ey) else None,
                        "track_half_width_m": self.track_half_width_m,
                        "offtrack_margin_m": self.offtrack_margin_m
                    })
                    return
            else:
                # On track: reset counters and accumulate distance
                self.consecutive_offtrack_count = 0
                self.offtrack_start_time = None
            
            # Off-track distance accumulation (for metrics only, executed only if not off-track)
            # Note: If off-track, we already returned above, so this only runs when on-track
            if not off and self.prev_pos is not None:
                # Distance accumulation already done above, just publish status
                pass
            
            self.pub_off.publish(Bool(data=bool(off)))
            
            # ========== Lap Crossing Detection ==========
            # Update lap manager based on type
            if isinstance(self.lap_manager, ProgressLapManager):
                # Progress-based detection
                lap_crossed = self.lap_manager.update(cur_pos, t)
                
                # Log progress-based tracking info periodically
                if self._odom_call_count % 50 == 0:  # Every 50 calls
                    s = self.lap_manager.prev_s if self.lap_manager.prev_s is not None else 0.0
                    in_start_zone = self.lap_manager._is_in_start_zone_s(s)
                    state_name = self.lap_manager.state.name
                    self.get_logger().info(f"📍 Position: ({x:.2f}, {y:.2f}), s: {s:.2f}m/{self.lap_manager.length:.2f}m, State: {state_name}, Start zone: {in_start_zone}, Lap: {self.lap_manager.lap_count}")
            elif isinstance(self.lap_manager, GridLapManager):
                # Grid-based tracking
                lap_crossed = self.lap_manager.update(cur_pos, t)
                
                # Log grid-based tracking info periodically
                if self._odom_call_count % 50 == 0:  # Every 50 calls
                    coverage = self.lap_manager.get_track_coverage()
                    grid = self.lap_manager.world_to_grid(cur_pos[0], cur_pos[1])
                    in_start_zone = self.lap_manager.is_in_start_zone(grid)
                    self.get_logger().info(f"📍 Position: ({x:.2f}, {y:.2f}), Grid: {grid}, Start zone: {in_start_zone}, Coverage: {coverage:.1%}, Lap: {self.lap_manager.lap_count}, Time: {t:.2f}s")
                
                # Log when entering start zone
                if self.lap_manager.currently_in_start_zone and not self.lap_manager.was_in_start_zone:
                    coverage = self.lap_manager.get_track_coverage()
                    self.get_logger().info(f"🔄 Entered start zone: Position: ({x:.2f}, {y:.2f}), Coverage: {coverage:.1%}, Visited cells: {len(self.lap_manager.current_lap_cells)}/{self.lap_manager.total_track_cells}")
            else:
                # Line crossing based detection (LapManager)
                if self.lap_manager.prev_pos is not None:
                    signed_dist = self.lap_manager._compute_signed_distance_to_line(cur_pos)
                    prev_signed_dist = self.lap_manager._compute_signed_distance_to_line(self.lap_manager.prev_pos)
                    
                    # Log ALL crossing attempts with detailed information
                    if (prev_signed_dist <= 0.0 and signed_dist > 0.0) or (prev_signed_dist > 0.0 and signed_dist <= 0.0):
                        # Sign change detected
                        side_prev = "forward" if prev_signed_dist > 0 else "backward"
                        side_curr = "forward" if signed_dist > 0 else "backward"
                        self.get_logger().info(f"🔄 Start line crossing detected: {side_prev} → {side_curr}")
                        self.get_logger().info(f"   Position: ({x:.2f}, {y:.2f}), Prev dist: {prev_signed_dist:.3f}m, Curr dist: {signed_dist:.3f}m")
                
                lap_crossed = self.lap_manager.update(cur_pos, yaw, t)
            
            if lap_crossed:
                lap_num = self.lap_manager.get_lap_count()
                self.get_logger().info("=" * 70)
                self.get_logger().info(f"🏁 Lap {lap_num} detected at ({x:.2f}, {y:.2f})")
                self.get_logger().info(f"   Current time: {t:.2f}s")
                if isinstance(self.lap_manager, ProgressLapManager):
                    s = self.lap_manager.prev_s if self.lap_manager.prev_s is not None else 0.0
                    self.get_logger().info(f"   Progress s: {s:.2f}m / {self.lap_manager.length:.2f}m")
                    self.get_logger().info(f"   State: {self.lap_manager.state.name}")
                elif isinstance(self.lap_manager, GridLapManager):
                    coverage = self.lap_manager.get_track_coverage()
                    self.get_logger().info(f"   Track coverage: {coverage:.1%}")
                    self.get_logger().info(f"   Visited cells: {len(self.lap_manager.current_lap_cells)}/{self.lap_manager.total_track_cells}")
                else:
                    signed_dist = self.lap_manager._compute_signed_distance_to_line(cur_pos)
                    self.get_logger().info(f"   Signed distance: {signed_dist:.3f}m")
                self.get_logger().info(f"   Lap start time: {self.lap_manager.lap_start_time:.2f}s" if self.lap_manager.lap_start_time else "   Lap start time: N/A")
                # Handle different lap manager types
                if isinstance(self.lap_manager, ProgressLapManager):
                    last_crossing = self.lap_manager.last_lap_crossing_time
                else:
                    last_crossing = getattr(self.lap_manager, 'last_crossing_time', None)
                self.get_logger().info(f"   Last crossing time: {last_crossing:.2f}s" if last_crossing else "   Last crossing time: N/A")
                self.get_logger().info("=" * 70)
                self._close_lap(t)
        
        self.prev_pos = cur_pos
        self.prev_stamp = t
        self.prev_yaw = yaw
        self.metric_prev_stamp = t
        self.metric_prev_yaw = yaw
        self.metric_prev_speed = speed
        if yaw_rate is not None:
            self.metric_prev_yaw_rate = yaw_rate
        if longitudinal_accel is not None:
            self.metric_prev_longitudinal_accel = longitudinal_accel
    
    def _close_lap(self, t_now: float):
        """Close current lap and publish summary."""
        lap_num = self.lap_manager.get_lap_count()
        lap_time = self.lap_manager.last_lap_time
        completed_laps = len(self.all_laps)
        
        # Check max_laps BEFORE early return (important for auto-shutdown)
        # This ensures shutdown happens even if lap_time is invalid
        if self.max_laps > 0 and completed_laps >= self.max_laps:
            self.get_logger().info(f"  ✅ Maximum laps reached ({self.max_laps})")
            if self.auto_shutdown:
                self.get_logger().info("  🛑 Auto-shutdown enabled. Generating final summary and shutting down...")
                # Capture parent_pid BEFORE self.shutdown() — after shutdown the parent
                # (ros2 launch) may exit and ppid() would return 1 (init), causing
                # psutil to enumerate ALL system processes and wrongly kill unrelated
                # processes such as evaluation_runner.py.
                import threading
                import time
                import os
                import signal
                import subprocess
                _saved_parent_pid = os.getppid()
                # Export final summary immediately
                self.shutdown()
                # Signal main loop to exit by shutting down ROS 2
                self.get_logger().info("Shutting down metrics collector and ROS 2 node...")
                # Use a timer to shutdown after a brief delay to allow export to complete
                def delayed_shutdown():
                    time.sleep(2.0)  # Give time for export (increased to 2 seconds)

                    # SAFE SHUTDOWN: Only terminate processes that belong to this launch session
                    # This prevents killing unrelated system processes that could cause system crashes
                    # Note: We don't call rclpy.shutdown() here to avoid causing other nodes
                    # to exit with code 1. The launch system will handle ROS2 context shutdown.
                    self.get_logger().info("Terminating evaluation simulation processes safely...")

                    if psutil is not None:
                        # Use psutil for safe process management
                        try:
                            parent_pid = _saved_parent_pid
                            # Guard: if parent is init/systemd (ppid<=1), the launch process
                            # already exited — skip psutil to avoid killing unrelated processes.
                            if parent_pid <= 1:
                                self.get_logger().warn(
                                    f"Parent PID={parent_pid} (init/systemd), launch already "
                                    "exited. Skipping psutil-based process termination.")
                                rclpy.shutdown()
                                return
                            parent_process = psutil.Process(parent_pid)

                            self.get_logger().info(f"Parent process: {parent_process.name()} (PID: {parent_pid})")
                            
                            # Get all descendants of the parent process (launch file and its children)
                            descendants = parent_process.children(recursive=True)
                            
                            # Also include the parent process itself (launch file)
                            processes_to_terminate = descendants + [parent_process]
                            
                            terminated_count = 0
                            known_patterns = [
                                'rviz2', 'rviz', 'gym_bridge', 'pure_pursuit_driver',
                                'pure_pursuit', 'map_server', 'lifecycle_manager',
                                'robot_state_publisher', 'ego_robot_state_publisher',
                                'realtime_visualizer', 'metrics_collector',
                                'ros2', 'launch'
                            ]
                            
                            # First attempt: graceful shutdown with SIGTERM
                            for proc in processes_to_terminate:
                                try:
                                    proc_name = proc.name()
                                    cmdline = ' '.join(proc.cmdline()) if proc.cmdline() else ''
                                    
                                    # Convert to lowercase for case-insensitive matching
                                    proc_name_lower = proc_name.lower()
                                    cmdline_lower = cmdline.lower()
                                    
                                    # Check both process name and command line for patterns
                                    # Also explicitly check for GUI processes (rviz2, realtime_visualizer)
                                    should_terminate = False
                                    for pattern in known_patterns:
                                        if pattern.lower() in proc_name_lower or pattern.lower() in cmdline_lower:
                                            should_terminate = True
                                            break
                                    
                                    # Explicitly check for GUI processes (cmdline is more reliable)
                                    if 'rviz2' in cmdline_lower or 'rviz' in cmdline_lower:
                                        should_terminate = True
                                    if 'realtime_visualizer' in cmdline_lower:
                                        should_terminate = True
                                    
                                    if should_terminate:
                                        self.get_logger().info(f"Terminating: {proc_name} (PID: {proc.pid}) - cmdline: {cmdline[:100]}")
                                        proc.terminate()  # SIGTERM
                                        terminated_count += 1
                                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                                    pass  # Process already gone or inaccessible
                            
                            # Wait for graceful shutdown
                            # Don't call rclpy.shutdown() here - let processes exit naturally
                            # Calling rclpy.shutdown() causes other nodes to exit with code 1
                            # The launch system will handle ROS2 context shutdown
                            time.sleep(3.0)
                            
                            # Second attempt: force kill only processes that are still running
                            for proc in processes_to_terminate:
                                try:
                                    if proc.is_running():
                                        proc_name = proc.name()
                                        cmdline = ' '.join(proc.cmdline()) if proc.cmdline() else ''
                                        
                                        # Convert to lowercase for case-insensitive matching
                                        proc_name_lower = proc_name.lower()
                                        cmdline_lower = cmdline.lower()
                                        
                                        # Check both process name and command line for patterns
                                        should_kill = False
                                        for pattern in known_patterns:
                                            if pattern.lower() in proc_name_lower or pattern.lower() in cmdline_lower:
                                                should_kill = True
                                                break
                                        
                                        # Explicitly check for GUI processes
                                        if 'rviz2' in cmdline_lower or 'rviz' in cmdline_lower:
                                            should_kill = True
                                        if 'realtime_visualizer' in cmdline_lower:
                                            should_kill = True
                                        
                                        if should_kill:
                                            self.get_logger().warn(f"Force killing: {proc_name} (PID: {proc.pid}) - cmdline: {cmdline[:100]}")
                                            proc.kill()  # SIGKILL
                                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                                    pass
                            
                            self.get_logger().info(f"Terminated {terminated_count} processes safely")
                            
                        except (psutil.NoSuchProcess, psutil.AccessDenied) as e:
                            self.get_logger().warn(f"Could not access parent process: {e}")
                            # Fallback: Only log, don't kill anything
                            self.get_logger().info("Using fallback: letting ROS2 shutdown handle process termination")
                    else:
                        # Fallback if psutil is not available: Only shutdown ROS2, don't kill processes
                        self.get_logger().warn("psutil not available. Skipping process termination.")
                        self.get_logger().info("ROS2 shutdown should handle process cleanup.")
                    
                    self.get_logger().info("Shutdown complete")
                # Start shutdown thread
                shutdown_thread = threading.Thread(target=delayed_shutdown, daemon=False)
                shutdown_thread.start()
                # Store thread to prevent garbage collection
                self._shutdown_thread = shutdown_thread
                return  # Exit early if shutting down
        
        if lap_time is None or lap_time < self.min_lap_time_s:
            return
        
        # Compute lap metrics
        rms_ey = self.rms_lat.value()
        max_ey = self.max_lat.value()
        rms_heading_error = self.rms_heading_error.value()
        max_heading_error = self.max_heading_error.value()
        rms_steer_rate = self.rms_steer_rate.value()
        rms_steer_jerk = self.rms_steer_jerk.value()
        rms_yaw_rate = self.rms_yaw_rate.value()
        yaw_rate_std = self.var_yaw_rate.std()
        max_yaw_rate = self.max_yaw_rate.value()
        rms_yaw_accel = self.rms_yaw_accel.value()
        rms_longitudinal_accel = self.rms_longitudinal_accel.value()
        rms_longitudinal_jerk = self.rms_longitudinal_jerk.value()
        var_lookahead = self.var_lookahead.var()
        avg_speed = self.mean_speed.value()
        max_speed = self.max_speed.value()  # 최고 속도
        min_speed = self.min_speed.value()  # 최저 속도
        
        off_ratio = (self.lap_offtrack_dist / self.lap_dist) if self.lap_dist > 1e-6 else 0.0
        
        # Average speed from track length
        v_avg_from_track = (self.track_length_m / lap_time) if self.track_length_m > 1e-6 else avg_speed
        path_efficiency = (
            self.track_length_m / self.lap_dist
            if self.track_length_m > 1e-6 and self.lap_dist > 1e-6
            else float("nan")
        )
        path_excess_ratio = (
            (self.lap_dist - self.track_length_m) / self.track_length_m
            if self.track_length_m > 1e-6
            else float("nan")
        )
        
        lap_summary = {
            "lap": lap_num,
            "lap_time_s": lap_time,
            "avg_speed_mps": v_avg_from_track,
            "avg_speed_measured_mps": avg_speed,
            "max_speed_mps": max_speed,  # 최고 속도 추가
            "min_speed_mps": min_speed,  # 최저 속도 추가
            "rms_lateral_error_m": rms_ey,
            "max_lateral_error_m": max_ey,
            "rms_heading_error_rad": rms_heading_error,
            "max_heading_error_rad": max_heading_error,
            "rms_steering_rate_radps": rms_steer_rate,
            "rms_steering_jerk_radps2": rms_steer_jerk,
            "rms_yaw_rate_radps": rms_yaw_rate,
            "yaw_rate_std_radps": yaw_rate_std,
            "max_yaw_rate_radps": max_yaw_rate,
            "rms_yaw_accel_radps2": rms_yaw_accel,
            "rms_longitudinal_accel_mps2": rms_longitudinal_accel,
            "rms_longitudinal_jerk_mps3": rms_longitudinal_jerk,
            "lookahead_variance": var_lookahead,
            "offtrack_ratio": off_ratio,
            "lap_distance_m": self.lap_dist,
            "path_efficiency": path_efficiency,
            "path_excess_ratio": path_excess_ratio,
            "lap_offtrack_distance_m": self.lap_offtrack_dist,
            "estop_count_total": self.estop_count,
            "timestamp": t_now
        }
        
        # Publish
        self.pub_lap.publish(String(data=json.dumps(lap_summary)))
        
        # Export
        self.exporter.export_lap_summary(lap_num, lap_summary)
        self.exporter.export_lap_csv(lap_num, lap_summary)
        
        # Store
        self.all_laps.append(lap_summary)
        completed_laps = len(self.all_laps)
        
        self.get_logger().info(f"Lap {lap_num} completed:")
        self.get_logger().info(f"  Time: {lap_time:.2f}s")
        self.get_logger().info(f"  Avg Speed: {v_avg_from_track:.2f} m/s")
        self.get_logger().info(f"  Max Speed: {max_speed:.2f} m/s")
        self.get_logger().info(f"  Min Speed: {min_speed:.2f} m/s")
        self.get_logger().info(f"  RMS Lateral Error: {rms_ey:.3f} m")
        self.get_logger().info(f"  Max Lateral Error: {max_ey:.3f} m")
        self.get_logger().info(f"  RMS Heading Error: {rms_heading_error:.3f} rad")
        self.get_logger().info(f"  Yaw Rate Std: {yaw_rate_std:.3f} rad/s")
        self.get_logger().info(f"  Off-track Ratio: {off_ratio:.3f}")
        
        # Check if minimum laps reached
        if self.min_laps > 0 and completed_laps < self.min_laps:
            remaining = self.min_laps - completed_laps
            self.get_logger().info(f"  ⏳ Minimum laps not reached yet ({remaining} more needed)")
        
        # Check if maximum laps reached
        if self.max_laps > 0 and completed_laps >= self.max_laps:
            self.get_logger().info(f"  ✅ Maximum laps reached ({self.max_laps})")
            if self.auto_shutdown:
                self.get_logger().info("  🛑 Auto-shutdown enabled. Generating final summary and shutting down...")
                # Capture parent_pid BEFORE self.shutdown() — after shutdown the parent
                # (ros2 launch) may exit and ppid() would return 1 (init), causing
                # psutil to enumerate ALL system processes and wrongly kill unrelated
                # processes such as evaluation_runner.py.
                import threading
                import time
                import os
                import signal
                import subprocess
                _saved_parent_pid = os.getppid()
                # Export final summary immediately
                self.shutdown()
                # Signal main loop to exit by shutting down ROS 2
                self.get_logger().info("Shutting down metrics collector and ROS 2 node...")
                # Use a timer to shutdown after a brief delay to allow export to complete
                def delayed_shutdown():
                    time.sleep(2.0)  # Give time for export (increased to 2 seconds)

                    # SAFE SHUTDOWN: Only terminate processes that belong to this launch session
                    # This prevents killing unrelated system processes that could cause system crashes
                    # Note: We don't call rclpy.shutdown() here to avoid causing other nodes
                    # to exit with code 1. The launch system will handle ROS2 context shutdown.
                    self.get_logger().info("Terminating evaluation simulation processes safely...")

                    if psutil is not None:
                        # Use psutil for safe process management
                        try:
                            parent_pid = _saved_parent_pid
                            # Guard: if parent is init/systemd (ppid<=1), the launch process
                            # already exited — skip psutil to avoid killing unrelated processes.
                            if parent_pid <= 1:
                                self.get_logger().warn(
                                    f"Parent PID={parent_pid} (init/systemd), launch already "
                                    "exited. Skipping psutil-based process termination.")
                                rclpy.shutdown()
                                return
                            parent_process = psutil.Process(parent_pid)

                            self.get_logger().info(f"Parent process: {parent_process.name()} (PID: {parent_pid})")
                            
                            # Get all descendants of the parent process (launch file and its children)
                            descendants = parent_process.children(recursive=True)
                            
                            # Also include the parent process itself (launch file)
                            processes_to_terminate = descendants + [parent_process]
                            
                            terminated_count = 0
                            known_patterns = [
                                'rviz2', 'rviz', 'gym_bridge', 'pure_pursuit_driver',
                                'pure_pursuit', 'map_server', 'lifecycle_manager',
                                'robot_state_publisher', 'ego_robot_state_publisher',
                                'realtime_visualizer', 'metrics_collector',
                                'ros2', 'launch'
                            ]
                            
                            # First attempt: graceful shutdown with SIGTERM
                            for proc in processes_to_terminate:
                                try:
                                    proc_name = proc.name()
                                    cmdline = ' '.join(proc.cmdline()) if proc.cmdline() else ''
                                    
                                    # Convert to lowercase for case-insensitive matching
                                    proc_name_lower = proc_name.lower()
                                    cmdline_lower = cmdline.lower()
                                    
                                    # Check both process name and command line for patterns
                                    # Also explicitly check for GUI processes (rviz2, realtime_visualizer)
                                    should_terminate = False
                                    for pattern in known_patterns:
                                        if pattern.lower() in proc_name_lower or pattern.lower() in cmdline_lower:
                                            should_terminate = True
                                            break
                                    
                                    # Explicitly check for GUI processes (cmdline is more reliable)
                                    if 'rviz2' in cmdline_lower or 'rviz' in cmdline_lower:
                                        should_terminate = True
                                    if 'realtime_visualizer' in cmdline_lower:
                                        should_terminate = True
                                    
                                    if should_terminate:
                                        self.get_logger().info(f"Terminating: {proc_name} (PID: {proc.pid}) - cmdline: {cmdline[:100]}")
                                        proc.terminate()  # SIGTERM
                                        terminated_count += 1
                                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                                    pass  # Process already gone or inaccessible
                            
                            # Wait for graceful shutdown
                            # Don't call rclpy.shutdown() here - let processes exit naturally
                            # Calling rclpy.shutdown() causes other nodes to exit with code 1
                            # The launch system will handle ROS2 context shutdown
                            time.sleep(3.0)
                            
                            # Second attempt: force kill only processes that are still running
                            for proc in processes_to_terminate:
                                try:
                                    if proc.is_running():
                                        proc_name = proc.name()
                                        cmdline = ' '.join(proc.cmdline()) if proc.cmdline() else ''
                                        
                                        # Convert to lowercase for case-insensitive matching
                                        proc_name_lower = proc_name.lower()
                                        cmdline_lower = cmdline.lower()
                                        
                                        # Check both process name and command line for patterns
                                        should_kill = False
                                        for pattern in known_patterns:
                                            if pattern.lower() in proc_name_lower or pattern.lower() in cmdline_lower:
                                                should_kill = True
                                                break
                                        
                                        # Explicitly check for GUI processes
                                        if 'rviz2' in cmdline_lower or 'rviz' in cmdline_lower:
                                            should_kill = True
                                        if 'realtime_visualizer' in cmdline_lower:
                                            should_kill = True
                                        
                                        if should_kill:
                                            self.get_logger().warn(f"Force killing: {proc_name} (PID: {proc.pid}) - cmdline: {cmdline[:100]}")
                                            proc.kill()  # SIGKILL
                                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                                    pass
                            
                            self.get_logger().info(f"Terminated {terminated_count} processes safely")
                            
                        except (psutil.NoSuchProcess, psutil.AccessDenied) as e:
                            self.get_logger().warn(f"Could not access parent process: {e}")
                            # Fallback: Only log, don't kill anything
                            self.get_logger().info("Using fallback: letting ROS2 shutdown handle process termination")
                    else:
                        # Fallback if psutil is not available: Only shutdown ROS2, don't kill processes
                        self.get_logger().warn("psutil not available. Skipping process termination.")
                        self.get_logger().info("ROS2 shutdown should handle process cleanup.")
                    
                    self.get_logger().info("Shutdown complete")
                # Start shutdown thread
                shutdown_thread = threading.Thread(target=delayed_shutdown, daemon=False)
                shutdown_thread.start()
                # Store thread to prevent garbage collection
                self._shutdown_thread = shutdown_thread
        
        # Reset for next lap
        self._reset_lap_stats()
    
    def shutdown(self):
        """Export final run summary on shutdown and generate performance report."""
        if not self.all_laps:
            return
        
        def valid_values(key: str) -> list:
            vals = []
            for lap in self.all_laps:
                v = lap.get(key)
                if isinstance(v, (int, float)) and not (isinstance(v, float) and math.isnan(v)):
                    vals.append(float(v))
            return vals

        # Compute run summary
        lap_times = valid_values("lap_time_s")
        rms_errors = valid_values("rms_lateral_error_m")
        max_errors = valid_values("max_lateral_error_m")
        heading_errors = valid_values("rms_heading_error_rad")
        yaw_rate_rms = valid_values("rms_yaw_rate_radps")
        yaw_rate_std = valid_values("yaw_rate_std_radps")
        yaw_accel_rms = valid_values("rms_yaw_accel_radps2")
        steer_rate_rms = valid_values("rms_steering_rate_radps")
        steer_jerk_rms = valid_values("rms_steering_jerk_radps2")
        long_accel_rms = valid_values("rms_longitudinal_accel_mps2")
        long_jerk_rms = valid_values("rms_longitudinal_jerk_mps3")
        lookahead_variances = valid_values("lookahead_variance")
        path_efficiencies = valid_values("path_efficiency")
        path_excess_ratios = valid_values("path_excess_ratio")
        avg_speeds = valid_values("avg_speed_mps")
        max_speeds = valid_values("max_speed_mps")
        min_speeds = valid_values("min_speed_mps")
        
        run_summary = {
            "total_laps": len(self.all_laps),
            "total_distance_m": self.total_dist,
            "total_offtrack_distance_m": self.offtrack_dist,
            "overall_offtrack_ratio": (self.offtrack_dist / self.total_dist) if self.total_dist > 1e-6 else 0.0,
            "avg_lap_time_s": sum(lap_times) / len(lap_times) if lap_times else float("nan"),
            "min_lap_time_s": min(lap_times) if lap_times else float("nan"),
            "max_lap_time_s": max(lap_times) if lap_times else float("nan"),
            "avg_speed_mps": sum(avg_speeds) / len(avg_speeds) if avg_speeds else float("nan"),
            "max_speed_mps": max(max_speeds) if max_speeds else float("nan"),  # 전체 최고 속도
            "min_speed_mps": min(min_speeds) if min_speeds else float("nan"),  # 전체 최저 속도
            "avg_rms_lateral_error_m": sum(rms_errors) / len(rms_errors) if rms_errors else float("nan"),
            "max_lateral_error_m": max(max_errors) if max_errors else float("nan"),
            "avg_rms_heading_error_rad": sum(heading_errors) / len(heading_errors) if heading_errors else float("nan"),
            "avg_rms_yaw_rate_radps": sum(yaw_rate_rms) / len(yaw_rate_rms) if yaw_rate_rms else float("nan"),
            "avg_yaw_rate_std_radps": sum(yaw_rate_std) / len(yaw_rate_std) if yaw_rate_std else float("nan"),
            "avg_rms_yaw_accel_radps2": sum(yaw_accel_rms) / len(yaw_accel_rms) if yaw_accel_rms else float("nan"),
            "avg_rms_steering_rate_radps": sum(steer_rate_rms) / len(steer_rate_rms) if steer_rate_rms else float("nan"),
            "avg_rms_steering_jerk_radps2": sum(steer_jerk_rms) / len(steer_jerk_rms) if steer_jerk_rms else float("nan"),
            "avg_rms_longitudinal_accel_mps2": sum(long_accel_rms) / len(long_accel_rms) if long_accel_rms else float("nan"),
            "avg_rms_longitudinal_jerk_mps3": sum(long_jerk_rms) / len(long_jerk_rms) if long_jerk_rms else float("nan"),
            "avg_lookahead_variance": sum(lookahead_variances) / len(lookahead_variances) if lookahead_variances else float("nan"),
            "avg_path_efficiency": sum(path_efficiencies) / len(path_efficiencies) if path_efficiencies else float("nan"),
            "avg_path_excess_ratio": sum(path_excess_ratios) / len(path_excess_ratios) if path_excess_ratios else float("nan"),
            "estop_count_total": self.estop_count,
            "laps": self.all_laps
        }
        
        # Export
        run_id = f"run_{len(self.all_laps)}laps"
        self.exporter.export_run_summary(run_id, run_summary)
        self.exporter.export_all_laps_csv(self.all_laps)
        
        # Publish (only if context is still valid)
        try:
            self.pub_run.publish(String(data=json.dumps(run_summary)))
        except Exception as e:
            self.get_logger().warn(f"Failed to publish run summary (context may be invalid): {e}")
        
        self.get_logger().info("Run summary exported")
        
        # Generate performance report
        try:
            import subprocess
            import sys
            from pathlib import Path
            
            # Get the path to generate_performance_report.py
            # Try multiple possible locations:
            # 1. Source directory: src/f1tenth_driver_benchmark_suite/generate_performance_report.py
            # 2. Installed package: share/f1tenth_driver_benchmark_suite/generate_performance_report.py
            # 3. Relative to workspace root
            current_file = Path(__file__).resolve()
            
            # Try source directory first (most reliable)
            source_dir = current_file.parent.parent.parent  # f1tenth_driver_benchmark_suite/
            report_script = source_dir / 'generate_performance_report.py'
            
            # If not found, try workspace root relative path
            if not report_script.exists():
                # Try to find workspace root (look for src/ directory)
                workspace_root = current_file
                while workspace_root.parent != workspace_root:
                    if (workspace_root / 'src' / 'f1tenth_driver_benchmark_suite' / 'generate_performance_report.py').exists():
                        report_script = workspace_root / 'src' / 'f1tenth_driver_benchmark_suite' / 'generate_performance_report.py'
                        break
                    workspace_root = workspace_root.parent
            
                # If still not found, try installed package location
                if not report_script.exists():
                    try:
                        from ament_index_python.packages import get_package_share_directory
                        package_share = Path(get_package_share_directory('f1tenth_driver_benchmark_suite'))
                        report_script = package_share / 'generate_performance_report.py'
                    except Exception:
                        pass
            
            if report_script.exists():
                # Get output directory from exporter
                output_dir = self.exporter.output_dir if hasattr(self.exporter, 'output_dir') else '/tmp/f1tenth_metrics'
                report_path = os.path.join(output_dir, 'performance_report.txt')
                self.get_logger().info(f"Generating performance report: {report_path}")
                
                # Run report generation script
                result = subprocess.run(
                    [sys.executable, str(report_script), output_dir, report_path],
                    capture_output=True,
                    text=True,
                    timeout=30
                )
                
                if result.returncode == 0:
                    self.get_logger().info(f"✅ Performance report generated: {report_path}")
                    # Log first few lines of report for visibility
                    if os.path.exists(report_path):
                        with open(report_path, 'r', encoding='utf-8') as f:
                            lines = f.readlines()[:10]
                            self.get_logger().info("Report preview:")
                            for line in lines:
                                self.get_logger().info(line.rstrip())
                else:
                    self.get_logger().warn(f"Failed to generate report: {result.stderr}")
            else:
                self.get_logger().warn(f"Report script not found: {report_script}")
        except Exception as e:
            self.get_logger().error(f"Error generating performance report: {e}")
    
    def _emergency_shutdown(self, reason: str, extra_data: Optional[dict] = None):
        """
        Emergency shutdown due to safety violation.
        
        Args:
            reason: Error reason code (e.g., "TRACK_BOUNDARY_VIOLATION", "REVERSE_DRIVING")
            extra_data: Additional error data to include in error summary
        """
        # 중복 호출 방지
        if self.is_shutting_down:
            return
        
        self.is_shutting_down = True
        
        error_summary = {
            "error": reason,
            "error_code": self._get_error_code(reason),
            "timestamp": self.prev_stamp if self.prev_stamp else 0.0,
            "total_laps": len(self.all_laps),
            "final_position": self.prev_pos if self.prev_pos else None,
            "laps": self.all_laps
        }
        
        # Add extra data if provided
        if extra_data:
            error_summary.update(extra_data)
        
        # Export error summary
        error_file = os.path.join(self.exporter.output_dir, f"error_{reason.lower()}.json")
        with open(error_file, 'w') as f:
            json.dump(error_summary, f, indent=2)
        
        self.get_logger().error(f"오류 요약 저장: {error_file}")
        self.get_logger().error(f"에러 코드: {error_summary['error_code']}")
        
        # Export partial run summary if any laps completed
        if self.all_laps:
            self.shutdown()
        
        # Signal shutdown - use delayed_shutdown like max_laps reached
        self.get_logger().error("시뮬레이션을 종료합니다...")
        # Use the same delayed_shutdown mechanism as max_laps to ensure proper cleanup
        def delayed_shutdown():
            time.sleep(2.0)  # Give time for export
            
            # SAFE SHUTDOWN: Only terminate processes that belong to this launch session
            self.get_logger().info("Terminating evaluation simulation processes safely...")
            
            if psutil is not None:
                try:
                    current_process = psutil.Process(os.getpid())
                    parent_process = psutil.Process(current_process.ppid())
                    descendants = parent_process.children(recursive=True)
                    processes_to_terminate = descendants + [parent_process]
                    
                    known_patterns = [
                        'rviz2', 'rviz', 'gym_bridge', 'pure_pursuit_driver',
                        'pure_pursuit', 'map_server', 'lifecycle_manager',
                        'robot_state_publisher', 'ego_robot_state_publisher',
                        'realtime_visualizer', 'metrics_collector',
                        'ros2', 'launch'
                    ]
                    
                    # First attempt: graceful shutdown with SIGTERM
                    for proc in processes_to_terminate:
                        try:
                            proc_name_lower = proc.name().lower()
                            cmdline_lower = ' '.join(proc.cmdline()).lower()
                            if any(pattern.lower() in proc_name_lower or pattern.lower() in cmdline_lower for pattern in known_patterns):
                                self.get_logger().info(f"Terminating: {proc.name()} (PID: {proc.pid})")
                                proc.terminate()
                        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                            pass
                    
                    time.sleep(3.0)
                    
                    # Second attempt: force kill still running processes
                    for proc in processes_to_terminate:
                        try:
                            if proc.is_running():
                                proc_name_lower = proc.name().lower()
                                cmdline_lower = ' '.join(proc.cmdline()).lower()
                                if any(pattern.lower() in proc_name_lower or pattern.lower() in cmdline_lower for pattern in known_patterns):
                                    self.get_logger().warn(f"Force killing: {proc.name()} (PID: {proc.pid})")
                                    proc.kill()
                        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                            pass
                    self.get_logger().info("Shutdown complete")
                except (psutil.NoSuchProcess, psutil.AccessDenied) as e:
                    self.get_logger().warn(f"Could not access parent process: {e}")
            else:
                self.get_logger().warn("psutil not available. Skipping process termination.")
        
        shutdown_thread = threading.Thread(target=delayed_shutdown, daemon=False)
        shutdown_thread.start()
        self._shutdown_thread = shutdown_thread
    
    def _get_error_code(self, reason: str) -> int:
        """Get numeric error code for error reason."""
        error_codes = {
            "TRACK_BOUNDARY_VIOLATION": 1001,
            "TRACK_BOUNDARY_CONSECUTIVE": 1002,
            "REVERSE_DRIVING": 2001,
            "OFFTRACK_IMMEDIATE": 3001,  # 트랙 이탈 즉시 종료
            "OFFTRACK_RATIO": 3002,
            "OFFTRACK_DURATION": 3003,
            "OFFTRACK_CONSECUTIVE": 3004,
        }
        return error_codes.get(reason, 9999)


def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as e:
            # Ignore if shutdown was already called
            pass  # Already shutdown, ignore error


if __name__ == '__main__':
    main()
