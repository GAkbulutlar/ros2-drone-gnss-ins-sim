#!/usr/bin/env python3
"""
imu_dead_reckoning.py

Dead-reckons the robot's position using:
  - forward speed (vx)  from wheel encoders  (/diff_drive_controller/odom)
  - heading (yaw)        from raw IMU gyro     (/imu) -- angular_velocity.z integrated

KEY POINT: Gazebo publishes msg.orientation as ground truth (perfect world pose).
We must integrate msg.angular_velocity.z ourselves so that the gyro noise
(stddev 0.009 rad/s) and bias (0.03 rad/s) actually accumulate into heading error.
This produces a clearly drifting path compared to the EKF-fused estimate.

Publishes: /odometry/imu_heading  (nav_msgs/Odometry, frame: odom)
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class ImuDeadReckoning(Node):
    def __init__(self):
        super().__init__('imu_dead_reckoning')

        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0           # integrated from noisy gyro
        self._vx: float = 0.0            # forward speed from encoders

        self._last_imu_stamp = None      # for gyro integration dt
        self._last_odom_stamp = None     # for position integration dt

        self._pub = self.create_publisher(Odometry, '/odometry/imu_heading', 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, 10)
        self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self._odom_cb, 10)

        self.get_logger().info(
            'IMU dead-reckoning started: '
            'speed from wheels, heading by integrating noisy gyro z')

    # ── IMU callback: integrate angular_velocity.z to accumulate heading error ─
    def _imu_cb(self, msg: Imu) -> None:
        stamp = msg.header.stamp

        if self._last_imu_stamp is None:
            self._last_imu_stamp = stamp
            return

        dt = (stamp.sec  - self._last_imu_stamp.sec) + \
             (stamp.nanosec - self._last_imu_stamp.nanosec) * 1e-9
        self._last_imu_stamp = stamp

        if dt <= 0.0 or dt > 0.5:
            return

        # Integrate noisy gyro rate -> heading.  The noise + bias specified in
        # the URDF (stddev 0.009 rad/s, bias_mean 0.03 rad/s) are applied by
        # Gazebo to angular_velocity.z before publishing.
        self._yaw += msg.angular_velocity.z * dt

    # ── Odometry callback: integrate position along accumulated IMU heading ───
    def _odom_cb(self, msg: Odometry) -> None:
        stamp = msg.header.stamp

        if self._last_odom_stamp is None:
            self._last_odom_stamp = stamp
            return

        dt = (stamp.sec  - self._last_odom_stamp.sec) + \
             (stamp.nanosec - self._last_odom_stamp.nanosec) * 1e-9
        self._last_odom_stamp = stamp

        if dt <= 0.0 or dt > 0.5:
            return

        vx = msg.twist.twist.linear.x

        # Integrate: perfect encoder speed but noisy IMU heading
        self._x += vx * math.cos(self._yaw) * dt
        self._y += vx * math.sin(self._yaw) * dt

        out = Odometry()
        out.header.stamp    = stamp
        out.header.frame_id = 'odom'
        out.child_frame_id  = 'base_footprint'
        out.pose.pose.position.x  = self._x
        out.pose.pose.position.y  = self._y
        out.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(self._yaw / 2.0)
        out.twist.twist.linear.x   = vx
        self._pub.publish(out)


def main():
    rclpy.init()
    node = ImuDeadReckoning()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
