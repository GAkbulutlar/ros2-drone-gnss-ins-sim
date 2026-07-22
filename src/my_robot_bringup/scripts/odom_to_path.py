#!/usr/bin/env python3
"""
odom_to_path.py
Subscribes to three Odometry topics and re-publishes each as a nav_msgs/Path.
This lets RViz draw the full trajectory history as coloured lines, making
the drift of the IMU-only estimate vs. ground truth immediately visible.

Topics published:
  /path/ground_truth   ← /ground_truth/odom   (Gazebo perfect pose)
  /path/imu_only       ← /odometry/imu_only   (IMU dead-reckoning, drifts fast)
  /path/ekf_filtered   ← /odometry/filtered   (wheel + IMU fusion)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

TOPICS = {
    '/ground_truth/odom':          '/path/ground_truth',
    '/diff_drive_controller/odom': '/path/wheel_odom',
    '/odometry/filtered':          '/path/ekf_filtered',
}

# Thin out path poses: only append if the robot moved at least this far [m]
MIN_DISTANCE = 0.02


class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')

        self._paths: dict[str, Path] = {}
        self._pubs:  dict[str, rclpy.publisher.Publisher] = {}
        self._last:  dict[str, tuple[float, float]] = {}

        for odom_topic, path_topic in TOPICS.items():
            self._paths[odom_topic] = Path()
            self._pubs[odom_topic]  = self.create_publisher(Path, path_topic, 10)
            self._last[odom_topic]  = (None, None)
            self.create_subscription(
                Odometry, odom_topic,
                lambda msg, t=odom_topic: self._cb(msg, t),
                10,
            )
            self.get_logger().info(f'Bridging {odom_topic} -> {path_topic}')

    def _cb(self, msg: Odometry, topic: str) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        lx, ly = self._last[topic]

        # Skip if the robot hasn't moved enough (reduces path density)
        if lx is not None:
            if ((x - lx) ** 2 + (y - ly) ** 2) < MIN_DISTANCE ** 2:
                return

        self._last[topic] = (x, y)

        path = self._paths[topic]
        path.header = msg.header

        ps = PoseStamped()
        ps.header = msg.header
        ps.pose   = msg.pose.pose
        path.poses.append(ps)

        self._pubs[topic].publish(path)


def main():
    rclpy.init()
    node = OdomToPath()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
