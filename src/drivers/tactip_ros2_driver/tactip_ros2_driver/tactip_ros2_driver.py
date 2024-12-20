import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import TwistStamped

from .tactip import TacTip

from .dependencies.label_encoder import BASE_MODEL_PATH

class TactipDriver(Node):
    def __init__(self):
        super().__init__('tactip_driver')

        # Parameters
        self.declare_parameter('source', 4)
        self.declare_parameter('frequency', 10)
        self.declare_parameter('test_model_time', False)

        self.get_logger().info("Tactip driver initialized")
        self.get_logger().info(BASE_MODEL_PATH)
        self.publisher_ = self.create_publisher(TwistStamped, '/sensors/tactip', 10)

        self.sensor = TacTip()

        self.period = 1.0/float(self.get_parameter('frequency').get_parameter_value().integer_value) # seconds
        if self.get_parameter('test_model_time').get_parameter_value().bool_value:
            self.get_logger().info("Testing model execution time")
            self.test_model_execution_time()
        else:
            self.timer = self.create_timer(self.period, self.timer_callback)

    def timer_callback(self):
        # read the data
        start_time = time.time()
        processed_image = self.sensor.process()
        data = self.sensor.predict(processed_image)

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

    def test_model_execution_time(self, iterations = 1000):
        start_time = time.time()
        for i in range(iterations):
            processed_image = self.sensor.process()
            data = self.sensor.predict(processed_image)
        self.get_logger().info(f"Time taken for {iterations} iterations: {time.time() - start_time} [seconds]")
        self.get_logger().info(f"Average time taken: {(time.time() - start_time)/iterations} [seconds]")

def main(args=None):
    rclpy.init(args=args)
    node = TactipDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()