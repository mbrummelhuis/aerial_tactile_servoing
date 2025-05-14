import rclpy
from rclpy.node import Node
from math import pi
import time
import os
import cv2

from geometry_msgs.msg import TwistStamped

from .tactip import TacTip

from .dependencies.label_encoder import BASE_MODEL_PATH

class TactipDriver(Node):
    def __init__(self):
        super().__init__('tactip_driver')

        # Parameters
        self.declare_parameter('source', 4)
        self.declare_parameter('frequency', 10.)
        self.declare_parameter('verbose', False)
        self.declare_parameter('test_model_time', False)
        self.declare_parameter('save_debug_image', False)

        self.get_logger().info("Tactip driver initialized")
        self.get_logger().info(BASE_MODEL_PATH)

        if self.get_parameter('save_debug_image').get_parameter_value().bool_value:
            self.image_outfile_path = os.path.join('/home','martijn','aerial_tactile_servoing')
            self.get_logger().info(f'Saving debug image at {os.path.join(self.image_outfile_path,'sensor_image.png')}')

        # publishers
        self.publisher_ = self.create_publisher(TwistStamped, '/sensors/tactip', 10)

        self.sensor = TacTip(self.get_parameter('source').get_parameter_value().integer_value)
        self.get_logger().info(f"Reading from /dev/video{self.get_parameter('source').get_parameter_value().integer_value}" )

        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.period = 1.0/self.frequency # seconds
        if self.get_parameter('test_model_time').get_parameter_value().bool_value:
            self.get_logger().info("Testing model execution time. It will run 1000 predictions through the model and print the average time taken.")
            self.test_model_execution_time()
        self.timer = self.create_timer(self.period, self.timer_callback)

    def timer_callback(self):
        # read the data
        sensor_image = self.sensor.process()
        cv2.imwrite(os.path.join(self.image_outfile_path,'sensor_image.png'), sensor_image)

        #processed_image = process_image(sensor_image, **processed_image_params)
        data = self.sensor.predict(sensor_image)
        # The model outputs the sensor pose in the contact frame, but we want the contact frame pose in the sensor frame

        if self.get_parameter('verbose').get_parameter_value().bool_value:
            self.get_logger().info(f"Z (mm): {data[2]:.2f} \t Rx (deg): {data[3]:.2f} \t Ry (deg): {data[4]:.2f}")

        # publish the data
        # The model outputs are in mm and deg, so convert to SI
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = data[0]
        msg.twist.linear.y = data[1]
        msg.twist.linear.z = data[2]
        msg.twist.angular.x = data[3]
        msg.twist.angular.y = data[4]
        msg.twist.angular.z = data[5]
        self.publisher_.publish(msg)
        #self.get_logger().info(f"Published data: {msg}")

    def test_model_execution_time(self, iterations = 1000):
        start_time = time.time()
        capture_time = []
        predict_time = []
        for i in range(iterations):
            capture_start_time = time.time()
            processed_image = self.sensor.process()
            capture_end_time = time.time()
            _ = self.sensor.predict(processed_image)
            predict_end_time = time.time()
            capture_time.append(capture_end_time - capture_start_time)
            predict_time.append(predict_end_time - capture_end_time)
        end_time = time.time()
        self.get_logger().info(f"Time taken for {iterations} iterations: {end_time - start_time} [seconds]")
        self.get_logger().info(f"Average time taken: {(end_time - start_time)/iterations} [seconds/it]")
        self.get_logger().info(f"Iterations per second: {iterations/(end_time - start_time)} [it/s]")
        self.get_logger().info(f"Average capture time: {sum(capture_time)/iterations} [seconds/it]")
        self.get_logger().info(f"Average predict time: {sum(predict_time)/iterations} [seconds/it]")

def main(args=None):
    rclpy.init(args=args)
    node = TactipDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()