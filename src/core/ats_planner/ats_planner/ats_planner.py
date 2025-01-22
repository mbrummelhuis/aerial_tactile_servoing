import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import TwistStamped

class ATSNavigation(Node):
    '''
    Aerial Tactile Servoing Planner node. This node publishes pose references for the tactile sensor to follow.
    '''
    def __init__(self):
        super().__init__('ats_planner')

        # Parameters
        self.declare_parameter('frequency', 10)

        # Initialization log message
        self.get_logger().info("Navigation node initialized")

        # Publishers
        self.ee_velocity_publisher_ = self.create_publisher(TwistStamped, '/references/ee_pose', 10)

        self.period = 1.0/float(self.get_parameter('frequency').get_parameter_value().integer_value) # seconds

        self.timer = self.create_timer(self.period, self.timer_callback)

    '''
    Publish the reference end effector velocity
    '''
    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.ee_velocity_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ATSNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()