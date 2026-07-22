#!/usr/bin/env python3
"""
square_move.py
Drives the robot in a 2 m × 2 m square using EKF-filtered odometry feedback.
Vertices: (0,0) → (2,0) → (2,-2) → (0,-2) → (0,0)  [right-hand turns]

Run (after sourcing):
    python3 src/my_robot_bringup/scripts/square_move.py
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


# ── tuning parameters ────────────────────────────────────────────────────────
SIDE_M          = 2.0     # square side length  [m]
LINEAR_SPEED    = 0.3     # forward speed       [m/s]
ANGULAR_SPEED   = 0.4     # turning speed       [rad/s]
LINEAR_TOL      = 0.03    # distance tolerance  [m]
ANGULAR_TOL     = 0.03    # angle tolerance     [rad]
CMD_VEL_TOPIC   = "/diff_drive_controller/cmd_vel"
ODOM_TOPIC      = "/odometry/filtered"
# ─────────────────────────────────────────────────────────────────────────────


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _wrap(angle: float) -> float:
    """Wrap angle to [-π, π]."""
    while angle >  math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle


class SquareMover(Node):
    def __init__(self):
        super().__init__("square_mover")

        self._pub  = self.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)
        self._sub  = self.create_subscription(Odometry, ODOM_TOPIC,
                                              self._odom_cb, 10)

        # current pose (from EKF odometry)
        self._x = self._y = self._yaw = 0.0
        self._odom_ready = False

        # state machine
        self._state   = "wait"   # wait | move | turn | done
        self._side    = 0        # which side we're on (0-3)
        self._ref_x   = 0.0      # reference pose at segment start
        self._ref_y   = 0.0
        self._ref_yaw = 0.0

        self._timer = self.create_timer(0.05, self._step)   # 20 Hz loop
        self.get_logger().info(
            f"SquareMover ready – will trace a {SIDE_M} m² square."
        )

    # ── odometry callback ────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self._x   = msg.pose.pose.position.x
        self._y   = msg.pose.pose.position.y
        self._yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._odom_ready = True

    # ── control loop ─────────────────────────────────────────────────────────
    def _step(self):
        if not self._odom_ready:
            return

        if self._state == "wait":
            self._begin_segment()

        elif self._state == "move":
            dist = math.hypot(self._x - self._ref_x, self._y - self._ref_y)
            if dist < SIDE_M - LINEAR_TOL:
                self._cmd(linear=LINEAR_SPEED)
            else:
                self._cmd()
                self._ref_yaw = self._yaw
                self._state = "turn"
                self.get_logger().info(
                    f"  Side {self._side + 1}/4 done  "
                    f"(x={self._x:.2f}, y={self._y:.2f})  →  turning…"
                )

        elif self._state == "turn":
            delta = _wrap(self._yaw - self._ref_yaw)
            # right turn = negative yaw change of π/2
            if abs(delta) < math.pi / 2.0 - ANGULAR_TOL:
                self._cmd(angular=-ANGULAR_SPEED)
            else:
                self._cmd()
                self._side += 1
                if self._side >= 4:
                    self._state = "done"
                    self.get_logger().info(
                        "Square complete! "
                        f"Final pose: x={self._x:.3f} y={self._y:.3f} "
                        f"yaw={math.degrees(self._yaw):.1f}°"
                    )
                else:
                    self._begin_segment()

        elif self._state == "done":
            self._cmd()   # keep publishing zero to hold stop

    # ── helpers ──────────────────────────────────────────────────────────────
    def _begin_segment(self):
        self._ref_x   = self._x
        self._ref_y   = self._y
        self._ref_yaw = self._yaw
        self._state   = "move"
        self.get_logger().info(
            f"Side {self._side + 1}/4 start  "
            f"(x={self._ref_x:.2f}, y={self._ref_y:.2f}, "
            f"yaw={math.degrees(self._ref_yaw):.1f}°)"
        )

    def _cmd(self, linear: float = 0.0, angular: float = 0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x  = linear
        msg.twist.angular.z = angular
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = SquareMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._cmd()   # stop the robot on Ctrl-C (publishes zero TwistStamped)
        node.get_logger().info("Interrupted – robot stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
