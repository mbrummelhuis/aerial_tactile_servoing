import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped

from tactip import TacTip

class TactipDriver(Node):
    def __init__(self):
        super().__init__('tactip_driver')
        self.get_logger().info("Tactip driver initialized")
        self.publisher_ = self.create_publisher(TwistStamped, '/sensors/tactip', 10)

        self.sensor = TacTip()

        self.period = 0.1 # seconds
        self.timer = self.create_timer(self.period, self.timer_callback)

    def timer_callback(self):
        # read the data
        data = self.sensor.read()

        self.get_logger().info(f"TacTip data: {data}")

        # publish the data
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = data[0]
        msg.twist.linear.y = data[1]
        msg.twist.linear.z = data[2]
        msg.twist.angular.x = data[3]
        msg.twist.angular.y = data[4]
        msg.twist.angular.z = data[5]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TactipDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()