#!/usr/bin/env python3

import csv
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class RacelinePathPublisher(Node):
    def __init__(self):
        super().__init__('raceline_path_publisher')

        self.raceline_csv = self.declare_parameter('raceline_csv', '').value
        self.path_topic = self.declare_parameter('path_topic', '/benchmark/raceline_path').value
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        self.publish_hz = float(self.declare_parameter('publish_hz', 1.0).value)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(Path, self.path_topic, qos)

        self.waypoints = self._load_raceline(self.raceline_csv)
        self.path_msg = self._build_path_msg(self.waypoints)

        period = 1.0 / self.publish_hz if self.publish_hz > 0.0 else 1.0
        self.timer = self.create_timer(period, self._publish)

        self.get_logger().info(f'Loaded {len(self.waypoints)} raceline points from {self.raceline_csv}')
        self.get_logger().info(f'Publishing path to {self.path_topic}')

    def _load_raceline(self, path: str) -> List[Tuple[float, float]]:
        if not path:
            raise RuntimeError('Parameter raceline_csv is empty')

        points: List[Tuple[float, float]] = []
        with open(path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                if 'x_m' in row and 'y_m' in row:
                    points.append((float(row['x_m']), float(row['y_m'])))
                elif 'x' in row and 'y' in row:
                    points.append((float(row['x']), float(row['y'])))

        if not points:
            raise RuntimeError(f'No waypoint rows were parsed from: {path}')
        return points

    def _build_path_msg(self, points: List[Tuple[float, float]]) -> Path:
        msg = Path()
        msg.header.frame_id = self.frame_id

        for x, y in points:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def _publish(self):
        now = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = now
        for pose in self.path_msg.poses:
            pose.header.stamp = now
        self.pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RacelinePathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
