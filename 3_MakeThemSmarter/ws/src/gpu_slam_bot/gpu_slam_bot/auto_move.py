import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AutoMove(Node):
    def __init__(self):
        super().__init__('auto_move')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2  # forward speed
        msg.angular.z = 0.0 # no rotation
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutoMove()
    rclpy.spin(node)
    rclpy.shutdown()
