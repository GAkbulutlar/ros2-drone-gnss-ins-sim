#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawtriangleNode(Node):

        def __init__(self):
            super().__init__("draw_triangle")
            self.cmd_vel_publish = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
            self.timer_ = self.create_timer(0.5, self.send_velocity_command)
            self.get_logger().info("Turtle Start drawing Triangle")


        def send_twist_command(self):
             msg = Twist()
             msg.linear.x = 1
             msg.angular.z = 1
             self.cmd_vel_publish.publish(msg)
               


def main(args=None):
    rclpy.init(args=args)
    node = DrawtriangleNode()
    rclpy.spin(node)
    rclpy.shutdown
    
