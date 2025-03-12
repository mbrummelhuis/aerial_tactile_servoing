import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import TwistStamped

class ATSPlanner(Node):
    '''
    Aerial Tactile Servoing Planner node. This node publishes pose references for the tactile sensor to follow.
    The references are defined in the contact frame, with Z perpendicular to the interaction surface. THe surface_3d 
    Tactip feedback only allows tracking contact depth (z position), roll (around x axis), and pitch (around y axis).
    Setting the other references nonzero will induce a movement in that direction as the feedback is always zero.
    '''
    def __init__(self):
        super().__init__('ats_planner')

        # Parameters
        self.declare_parameter('frequency', 10.)

        # Initialization log message
        self.get_logger().info("Planner node initialized")

        # Publishers
        self.ee_velocity_publisher_ = self.create_publisher(TwistStamped, '/references/ee_pose', 10)

        self.period = 1.0/float(self.get_parameter('frequency').get_parameter_value().double_value) # seconds

        self.timer = self.create_timer(self.period, self.timer_callback)

    '''
    Publish the reference end effector velocity
    '''
    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0 # end effector x ref in contact frame (6d)
        msg.twist.linear.y = 0.0 # end effector y ref in contact frame (6d)
        msg.twist.linear.z = -1.5 # end effector z ref in contact frame (3d)
        msg.twist.angular.z = 0.0 # end effector yaw ref in contact frame (6d) TODO: UNITS?
        msg.twist.angular.y = 0.0 # end effector pitch ref in contact frame (3d)
        msg.twist.angular.x = 0.0 # end effector roll ref in contact frame (3d)
        self.ee_velocity_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ATSPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()