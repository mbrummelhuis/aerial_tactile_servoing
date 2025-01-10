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
        self.ee_linear_velocity_publisher_ = self.create_publisher(TwistStamped, '/references/ee_linear_velocity', 10)
        self.ee_angular_velocity_publisher_ = self.create_publisher(TwistStamped, '/references/ee_ _velocity', 10)

        # Subscribers
        self.subscriber_

        self.period = 1.0/float(self.get_parameter('frequency').get_parameter_value().integer_value) # seconds

        self.timer = self.create_timer(self.period, self.timer_callback)

    def timer_callback(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ATSNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()