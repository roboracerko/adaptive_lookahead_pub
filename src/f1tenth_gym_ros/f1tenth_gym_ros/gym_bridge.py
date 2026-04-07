# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import sys

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster
from interfaces.msg import Train

import gym
import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from waypoint_msgs.msg import Waypoint
import os
from transforms3d import euler

class GymBridge(Node):
    def __init__(self):
        super().__init__('gym_bridge')

        # Declare parameters with default values for ROS2 Humble compatibility
        self.declare_parameter('ego_namespace', 'ego_racecar')
        self.declare_parameter('ego_odom_topic', 'odom')
        self.declare_parameter('ego_opp_odom_topic', 'opp_odom')
        self.declare_parameter('ego_scan_topic', 'scan')
        self.declare_parameter('ego_drive_topic', 'drive')
        self.declare_parameter('opp_namespace', 'opp_racecar')
        self.declare_parameter('opp_odom_topic', 'odom')
        self.declare_parameter('opp_ego_odom_topic', 'opp_odom')
        self.declare_parameter('opp_scan_topic', 'opp_scan')
        self.declare_parameter('opp_drive_topic', 'opp_drive')
        self.declare_parameter('scan_distance_to_base_link', 0.275)
        self.declare_parameter('scan_fov', 4.7)
        self.declare_parameter('scan_beams', 1080)
        self.declare_parameter('map_path', '/home/jin/ros2_prj/sim_ws/src/f1tenth_gym_ros/maps/Budapest')
        self.declare_parameter('map_img_ext', '.png')
        self.declare_parameter('num_agent', 1)
        self.declare_parameter('max_laps', 2)
        self.declare_parameter('sx', 0.0)
        self.declare_parameter('sy', 0.0)
        self.declare_parameter('stheta', 0.0)
        self.declare_parameter('sx1', 2.0)
        self.declare_parameter('sy1', 0.5)
        self.declare_parameter('stheta1', 0.0)
        self.declare_parameter('kb_teleop', True)
        self.declare_parameter('train_rl', False)
        self.declare_parameter('debug_lookahead_m', -1.0)

        # check num_agents
        num_agents = self.get_parameter('num_agent').value
        if num_agents < 1 or num_agents > 2:
            raise ValueError('num_agents should be either 1 or 2.')
        elif type(num_agents) != int:
            raise ValueError('num_agents should be an int.')

        # env backend
        self.env = gym.make('f110_gym:f110-v0',
                            map=self.get_parameter('map_path').value,
                            map_ext=self.get_parameter('map_img_ext').value,
                            num_agents=num_agents,
                            lidar_dist=self.get_parameter("scan_distance_to_base_link").value,
                            max_laps=self.get_parameter('max_laps').value
                            )

        sx = self.get_parameter('sx').value
        sy = self.get_parameter('sy').value
        stheta = self.get_parameter('stheta').value
        self.ego_pose = [sx, sy, stheta]
        self.ego_speed = [0.0, 0.0, 0.0]
        self.ego_requested_speed = 0.0
        self.ego_steer = 0.0
        self.ego_collision = False
        self.info = dict()
        self.debug_lookahead_m = self.get_parameter('debug_lookahead_m').value
        # reference waypoint cache (must exist before _update_sim_state)
        self.ref_waypoints = None
        self.ref_waypoints_last_idx = 0
        ego_scan_topic = self.get_parameter('ego_scan_topic').value
        ego_drive_topic = self.get_parameter('ego_drive_topic').value
        scan_fov = self.get_parameter('scan_fov').value
        scan_beams = self.get_parameter('scan_beams').value
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        self.angle_inc = scan_fov / scan_beams
        self.ego_namespace = self.get_parameter('ego_namespace').value
        ego_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_odom_topic').value
        self.scan_distance_to_base_link = self.get_parameter('scan_distance_to_base_link').value
        
        if num_agents == 2:
            self.has_opp = True
            self.opp_namespace = self.get_parameter('opp_namespace').value
            sx1 = self.get_parameter('sx1').value
            sy1 = self.get_parameter('sy1').value
            stheta1 = self.get_parameter('stheta1').value
            self.opp_pose = [sx1, sy1, stheta1]
            self.opp_speed = [0.0, 0.0, 0.0]
            self.opp_requested_speed = 0.0
            self.opp_steer = 0.0
            self.opp_collision = False
            # Gym 0.25.2+ compatibility: reset() returns only observation
            self.obs = self.env.reset(poses=np.array([[sx, sy, stheta], [sx1, sy1, stheta1]]))
            self.get_logger().info(f"Reset with poses: sx={sx:.6f}, sy={sy:.6f}, stheta={stheta:.6f}")
            self._update_sim_state()  # Update ego_pose from observation including poses_theta
            self.done = False
            self.ego_scan = list(self.obs['scans'][0])
            self.opp_scan = list(self.obs['scans'][1])

            opp_scan_topic = self.get_parameter('opp_scan_topic').value
            opp_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_odom_topic').value
            opp_drive_topic = self.get_parameter('opp_drive_topic').value

            ego_opp_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_opp_odom_topic').value
            opp_ego_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_ego_odom_topic').value
        else:
            self.has_opp = False
            self.obs = self.env.reset(poses=np.array([[sx, sy, stheta]]))
            self.get_logger().info(f"Reset with poses: sx={sx:.6f}, sy={sy:.6f}, stheta={stheta:.6f}")
            self.done = False
            self._update_sim_state()  # Update ego_pose from observation including poses_theta
            self.ego_scan = list(self.obs['scans'][0])

        # sim physical step timer
        self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)
        # topic publishing timer
        self.timer = self.create_timer(0.04, self.timer_callback)

        # transform broadcaster
        self.br = TransformBroadcaster(self)

        # publishers
        self.ego_scan_pub = self.create_publisher(LaserScan, ego_scan_topic, 10)
        self.ego_odom_pub = self.create_publisher(Odometry, ego_odom_topic, 10)
        self.ego_drive_published = False
        if num_agents == 2:
            self.opp_scan_pub = self.create_publisher(LaserScan, opp_scan_topic, 10)
            self.ego_opp_odom_pub = self.create_publisher(Odometry, ego_opp_odom_topic, 10)
            self.opp_odom_pub = self.create_publisher(Odometry, opp_odom_topic, 10)
            self.opp_ego_odom_pub = self.create_publisher(Odometry, opp_ego_odom_topic, 10)
            self.opp_drive_published = False

        if self.get_parameter('train_rl').value:
            self.done_pub = self.create_publisher(Train, '/train', 1)

        # subscribers
        self.ego_drive_sub = self.create_subscription(
            AckermannDriveStamped,
            ego_drive_topic,
            self.drive_callback,
            10)
        self.ego_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.ego_reset_callback,
            10)
        if num_agents == 2:
            self.opp_drive_sub = self.create_subscription(
                AckermannDriveStamped,
                opp_drive_topic,
                self.opp_drive_callback,
                10)
            self.opp_reset_sub = self.create_subscription(
                PoseStamped,
                '/goal_pose',
                self.opp_reset_callback,
                10)

        if self.get_parameter('kb_teleop').value:
            self.teleop_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.teleop_callback,
                10)

        # reference waypoint subscription for debug (speed difference)
        self.declare_parameter('enable_reference_speed_log', True)
        self.declare_parameter('reference_waypoint_topic', '/reference_waypoint')
        self.ref_waypoints = None
        self.ref_waypoints_last_idx = 0
        if self.get_parameter('enable_reference_speed_log').value:
            ref_topic = self.get_parameter('reference_waypoint_topic').value
            ref_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.ref_waypoint_sub = self.create_subscription(
                Waypoint,
                ref_topic,
                self._ref_waypoint_callback,
                ref_qos,
            )

        if self.get_parameter('train_rl').value:
            self.init_sub = self.create_subscription(
                Train,
                '/train',
                self.reinitialize_pose,
                10)

    def reinitialize_pose(self, train_msg):
        if train_msg.init:
            self.__init__()

    def drive_callback(self, drive_msg):
        # MPPI v2: Forward-only constraint at simulation level
        speed = drive_msg.drive.speed
        if speed < 0.0:
            # ROS2 Humble compatibility: warn_throttle is not available, use warn instead
            self.get_logger().warn(
                f"[GymBridge] Forward-only: Negative speed {speed:.3f} m/s clamped to 0.0"
            )
            speed = 0.0
        
        self.ego_requested_speed = speed
        self.ego_steer = drive_msg.drive.steering_angle
        self.ego_drive_published = True

    def opp_drive_callback(self, drive_msg):
        self.opp_requested_speed = drive_msg.drive.speed
        self.opp_steer = drive_msg.drive.steering_angle
        self.opp_drive_published = True

    def ego_reset_callback(self, pose_msg):
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
        if self.has_opp:
            opp_pose = [self.obs['poses_x'][1], self.obs['poses_y'][1], self.obs['poses_theta'][1]]
            # Gym 0.25.2+ compatibility: reset() returns only observation
            self.obs = self.env.reset(poses=np.array([[rx, ry, rtheta], opp_pose]))
        else:
            # Gym 0.25.2+ compatibility: reset() returns only observation
            self.obs = self.env.reset(poses=np.array([[rx, ry, rtheta]]))
        self.done = False
        self._update_sim_state()

    def opp_reset_callback(self, pose_msg):
        if self.has_opp:
            rx = pose_msg.pose.position.x
            ry = pose_msg.pose.position.y
            rqx = pose_msg.pose.orientation.x
            rqy = pose_msg.pose.orientation.y
            rqz = pose_msg.pose.orientation.z
            rqw = pose_msg.pose.orientation.w
            _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
            # Gym 0.25.2+ compatibility: reset() returns only observation
            self.obs = self.env.reset(poses=np.array([list(self.ego_pose), [rx, ry, rtheta]]))
            self.done = False
            self._update_sim_state()

    def teleop_callback(self, twist_msg):
        if not self.ego_drive_published:
            self.ego_drive_published = True

        self.ego_requested_speed = twist_msg.linear.x

        if twist_msg.angular.z > 0.0:
            self.ego_steer = 0.3
        elif twist_msg.angular.z < 0.0:
            self.ego_steer = -0.3
        else:
            self.ego_steer = 0.0

    def drive_timer_callback(self):
        if self.ego_drive_published and not self.has_opp:
            self.obs, _, self.done, self.info = self.env.step(np.array([[self.ego_steer, self.ego_requested_speed]]))
        elif self.ego_drive_published and self.has_opp and self.opp_drive_published:
            self.obs, _, self.done, self.info = self.env.step(np.array([[self.ego_steer, self.ego_requested_speed], [self.opp_steer, self.opp_requested_speed]]))
        
        if self.get_parameter('train_rl').value:
            done_msg = Train()
            done_msg.done = self.done
            """
            if self.ego_drive_published:
                done_msg.collision = bool(self.info['collision']) 
            else:
                done_msg.collision = False
            """
            self.done_pub.publish(done_msg)
            
            if self.done:
                return self.__init__()
        
        self._update_sim_state()

    def timer_callback(self):
        ts = self.get_clock().now().to_msg()

        # pub scans
        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = self.ego_namespace + '/laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.
        scan.range_max = 30.
        scan.ranges = self.ego_scan
        self.ego_scan_pub.publish(scan)

        if self.has_opp:
            opp_scan = LaserScan()
            opp_scan.header.stamp = ts
            opp_scan.header.frame_id = self.opp_namespace + '/laser'
            opp_scan.angle_min = self.angle_min
            opp_scan.angle_max = self.angle_max
            opp_scan.angle_increment = self.angle_inc
            opp_scan.range_min = 0.
            opp_scan.range_max = 30.
            opp_scan.ranges = self.opp_scan
            self.opp_scan_pub.publish(opp_scan)

        # pub tf
        self._publish_odom(ts)
        self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        self._publish_wheel_transforms(ts)

    def _update_sim_state(self):
        self.ego_scan = list(self.obs['scans'][0])
        if self.has_opp:
            self.opp_scan = list(self.obs['scans'][1])
            self.opp_pose[0] = self.obs['poses_x'][1]
            self.opp_pose[1] = self.obs['poses_y'][1]
            self.opp_pose[2] = self.obs['poses_theta'][1]
            self.opp_speed[0] = self.obs['linear_vels_x'][1]
            self.opp_speed[1] = self.obs['linear_vels_y'][1]
            self.opp_speed[2] = self.obs['ang_vels_z'][1]
        poses_theta_val = self.obs["poses_theta"][0]
        self.ego_pose[0] = self.obs['poses_x'][0]
        self.ego_pose[1] = self.obs['poses_y'][0]
        self.ego_pose[2] = self.obs['poses_theta'][0]
        self.ego_speed[0] = self.obs['linear_vels_x'][0]
        self.ego_speed[1] = self.obs['linear_vels_y'][0]
        self.ego_speed[2] = self.obs['ang_vels_z'][0]
        v_x = float(self.ego_speed[0])
        v_y = float(self.ego_speed[1])
        v_mag = float(np.hypot(v_x, v_y))
        lookahead_str = f"{self.debug_lookahead_m:.3f}" if self.debug_lookahead_m >= 0.0 else "n/a"
        ref_info = ""
        if self.ref_waypoints:
            ref_v, ref_idx, ref_dist = self._get_reference_speed(self.ego_pose[0], self.ego_pose[1])
            if ref_v is not None:
                dv = ref_v - v_mag
                ref_info = f" ref_v={ref_v:.3f} dv={dv:+.3f} ref_d={ref_dist:.2f} idx={ref_idx}"
        self.get_logger().info(
            f"Updated ego_pose: x={self.ego_pose[0]:.6f}, y={self.ego_pose[1]:.6f}, "
            f"theta={self.ego_pose[2]:.6f} v={v_mag:.3f} (vx={v_x:.3f}, vy={v_y:.3f}) "
            f"cmd_v={self.ego_requested_speed:.3f} lookahead={lookahead_str} "
            f"from obs_poses_theta={poses_theta_val:.6f}{ref_info}"
        )

        

    def _publish_odom(self, ts):
        ego_odom = Odometry()
        ego_odom.header.stamp = ts
        ego_odom.header.frame_id = 'map'
        ego_odom.child_frame_id = self.ego_namespace + '/base_link'
        ego_odom.pose.pose.position.x = self.ego_pose[0]
        ego_odom.pose.pose.position.y = self.ego_pose[1]
        ego_quat = euler.euler2quat(0., 0., self.ego_pose[2], axes='sxyz')
        ego_odom.pose.pose.orientation.x = ego_quat[1]
        ego_odom.pose.pose.orientation.y = ego_quat[2]
        ego_odom.pose.pose.orientation.z = ego_quat[3]
        ego_odom.pose.pose.orientation.w = ego_quat[0]
        ego_odom.twist.twist.linear.x = self.ego_speed[0]
        ego_odom.twist.twist.linear.y = self.ego_speed[1]
        ego_odom.twist.twist.angular.z = self.ego_speed[2]
        self.ego_odom_pub.publish(ego_odom)

        if self.has_opp:
            opp_odom = Odometry()
            opp_odom.header.stamp = ts
            opp_odom.header.frame_id = 'map'
            opp_odom.child_frame_id = self.opp_namespace + '/base_link'
            opp_odom.pose.pose.position.x = self.opp_pose[0]
            opp_odom.pose.pose.position.y = self.opp_pose[1]
            opp_quat = euler.euler2quat(0., 0., self.opp_pose[2], axes='sxyz')
            opp_odom.pose.pose.orientation.x = opp_quat[1]
            opp_odom.pose.pose.orientation.y = opp_quat[2]
            opp_odom.pose.pose.orientation.z = opp_quat[3]
            opp_odom.pose.pose.orientation.w = opp_quat[0]
            opp_odom.twist.twist.linear.x = self.opp_speed[0]
            opp_odom.twist.twist.linear.y = self.opp_speed[1]
            opp_odom.twist.twist.angular.z = self.opp_speed[2]
            self.opp_odom_pub.publish(opp_odom)
            self.opp_ego_odom_pub.publish(ego_odom)
            self.ego_opp_odom_pub.publish(opp_odom)

    def _publish_transforms(self, ts):
        ego_t = Transform()
        ego_t.translation.x = self.ego_pose[0]
        ego_t.translation.y = self.ego_pose[1]
        ego_t.translation.z = 0.0
        ego_quat = euler.euler2quat(0.0, 0.0, self.ego_pose[2], axes='sxyz')
        ego_t.rotation.x = ego_quat[1]
        ego_t.rotation.y = ego_quat[2]
        ego_t.rotation.z = ego_quat[3]
        ego_t.rotation.w = ego_quat[0]

        ego_ts = TransformStamped()
        ego_ts.transform = ego_t
        ego_ts.header.stamp = ts
        ego_ts.header.frame_id = 'map'
        ego_ts.child_frame_id = self.ego_namespace + '/base_link'
        self.br.sendTransform(ego_ts)

        if self.has_opp:
            opp_t = Transform()
            opp_t.translation.x = self.opp_pose[0]
            opp_t.translation.y = self.opp_pose[1]
            opp_t.translation.z = 0.0
            opp_quat = euler.euler2quat(0.0, 0.0, self.opp_pose[2], axes='sxyz')
            opp_t.rotation.x = opp_quat[1]
            opp_t.rotation.y = opp_quat[2]
            opp_t.rotation.z = opp_quat[3]
            opp_t.rotation.w = opp_quat[0]

            opp_ts = TransformStamped()
            opp_ts.transform = opp_t
            opp_ts.header.stamp = ts
            opp_ts.header.frame_id = 'map'
            opp_ts.child_frame_id = self.opp_namespace + '/base_link'
            self.br.sendTransform(opp_ts)

    def _ref_waypoint_callback(self, msg: Waypoint):
        if not msg.poses or not msg.twists:
            return
        n = min(len(msg.poses), len(msg.twists))
        waypoints = []
        for i in range(n):
            p = msg.poses[i].pose.position
            v = msg.twists[i].twist.linear.x
            waypoints.append((float(p.x), float(p.y), float(v)))
        if waypoints:
            self.ref_waypoints = waypoints
            self.ref_waypoints_last_idx = min(self.ref_waypoints_last_idx, len(waypoints) - 1)

    def _get_reference_speed(self, x, y):
        if not self.ref_waypoints:
            return None, None, None
        min_dist = None
        best_idx = 0
        best_v = None
        for i, (wx, wy, wv) in enumerate(self.ref_waypoints):
            dx = wx - x
            dy = wy - y
            dist = dx * dx + dy * dy
            if min_dist is None or dist < min_dist:
                min_dist = dist
                best_idx = i
                best_v = wv
        if min_dist is None:
            return None, None, None
        self.ref_waypoints_last_idx = best_idx
        return best_v, best_idx, float(np.sqrt(min_dist))

    def _publish_wheel_transforms(self, ts):
        ego_wheel_ts = TransformStamped()
        ego_wheel_quat = euler.euler2quat(0., 0., self.ego_steer, axes='sxyz')
        ego_wheel_ts.transform.rotation.x = ego_wheel_quat[1]
        ego_wheel_ts.transform.rotation.y = ego_wheel_quat[2]
        ego_wheel_ts.transform.rotation.z = ego_wheel_quat[3]
        ego_wheel_ts.transform.rotation.w = ego_wheel_quat[0]
        ego_wheel_ts.header.stamp = ts
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_left_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_left_wheel'
        self.br.sendTransform(ego_wheel_ts)
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_right_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_right_wheel'
        self.br.sendTransform(ego_wheel_ts)

        if self.has_opp:
            opp_wheel_ts = TransformStamped()
            opp_wheel_quat = euler.euler2quat(0., 0., self.opp_steer, axes='sxyz')
            opp_wheel_ts.transform.rotation.x = opp_wheel_quat[1]
            opp_wheel_ts.transform.rotation.y = opp_wheel_quat[2]
            opp_wheel_ts.transform.rotation.z = opp_wheel_quat[3]
            opp_wheel_ts.transform.rotation.w = opp_wheel_quat[0]
            opp_wheel_ts.header.stamp = ts
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_left_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_left_wheel'
            self.br.sendTransform(opp_wheel_ts)
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_right_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_right_wheel'
            self.br.sendTransform(opp_wheel_ts)

    def _publish_laser_transforms(self, ts):
        ego_scan_ts = TransformStamped()
        ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
        # ego_scan_ts.transform.translation.z = 0.04+0.1+0.025
        ego_scan_ts.transform.rotation.w = 1.
        ego_scan_ts.header.stamp = ts
        ego_scan_ts.header.frame_id = self.ego_namespace + '/base_link'
        ego_scan_ts.child_frame_id = self.ego_namespace + '/laser'
        self.br.sendTransform(ego_scan_ts)

        if self.has_opp:
            opp_scan_ts = TransformStamped()
            opp_scan_ts.transform.translation.x = self.scan_distance_to_base_link
            opp_scan_ts.transform.rotation.w = 1.
            opp_scan_ts.header.stamp = ts
            opp_scan_ts.header.frame_id = self.opp_namespace + '/base_link'
            opp_scan_ts.child_frame_id = self.opp_namespace + '/laser'
            self.br.sendTransform(opp_scan_ts)

def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    rclpy.spin(gym_bridge)

if __name__ == '__main__':
    main()
