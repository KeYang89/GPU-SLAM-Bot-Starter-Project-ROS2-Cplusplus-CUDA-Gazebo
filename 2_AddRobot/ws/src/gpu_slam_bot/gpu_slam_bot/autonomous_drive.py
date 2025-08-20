#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AutonomousDrive(Node):
    def __init__(self):
        super().__init__('autonomous_drive')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.twist = Twist()
        # Move forward and turn slowly
        self.twist.linear.x = 0.2    # forward speed
        self.twist.angular.z = 0.3   # rotation speed

    def timer_callback(self):
        self.publisher_.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
