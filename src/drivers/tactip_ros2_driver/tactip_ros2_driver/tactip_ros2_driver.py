import rclpy
from rclpy.node import Node
from math import pi
import time
import os
import numpy as np
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
        self.declare_parameter('save_directory', 'Please set a save_directory in the launch file')

        # Instantiate Tactip sensor
        self.sensor = TacTip(self.get_parameter('source').get_parameter_value().integer_value)

        # Node feedback
        self.get_logger().info("Tactip driver initialized")
        self.get_logger().info(BASE_MODEL_PATH)
        self.get_logger().info(f"Reading from /dev/video{self.get_parameter('source').get_parameter_value().integer_value}" )


        # Set up period image saving if enabled in launch
        if self.get_parameter('save_debug_image').get_parameter_value().bool_value:
            self.image_outfile_path = self.get_parameter('save_directory').get_parameter_value().string_value
            self.get_logger().info(f'Saving debug images in {self.image_outfile_path}')
            self.img_counter = 0
            if len(os.listdir(self.image_outfile_path)) != 0: # If directory is not empty, exit to avoid overwriting data
                self.get_logger().error("Directory not empty. Please delete or move data before proceeding.")
                self.get_logger().error("Node will now exit. Please kill stack with  Ctrl+C.")
                return

        # publishers
        self.publisher_ = self.create_publisher(TwistStamped, '/sensors/tactip', 10)

        # Run testing model evaluation time functionality if enabled in Launch
        if self.get_parameter('test_model_time').get_parameter_value().bool_value:
            self.get_logger().info("Testing model execution time. It will run 1000 predictions through the model and print the average time taken.")
            self.test_model_execution_time()

        # Rotation between output in actual frame and the end-effector frame
        self.R_T = np.array([[1, 0, 0], [0, -1, 0],[0, 0, -1]])

        # Set up timer
        self.cycle_counter = 0
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.period = 1.0/self.frequency # seconds
        self.timer = self.create_timer(self.period, self.timer_callback)

    def timer_callback(self):
        # read the data
        # Save an image every second
        if self.get_parameter('save_debug_image').get_parameter_value().bool_value and self.cycle_counter%self.frequency == 0:
            raw_outfile = os.path.join(self.image_outfile_path,'raw_image'+str(self.img_counter)+'.png')
            proc_outfile = os.path.join(self.image_outfile_path,'sensor_image'+str(self.img_counter)+'.png')
            sensor_image = self.sensor.process(raw_outfile=raw_outfile, proc_outfile=proc_outfile)
            self.img_counter += 1
            self.cycle_counter = 0
        
        # Just read data without saving
        else:
            sensor_image = self.sensor.process()

        #processed_image = process_image(sensor_image, **processed_image_params)
        data = self.sensor.predict(sensor_image)
        # The model outputs the sensor pose in the contact frame

        # Rotation from actual output frame to desired (i.e. convention-wise) end-effector frame
        rot_pred_pos = np.matmul(self.R_T, data[:3]) # Rotate positions to end-effector frame
        rot_pred_ang = np.matmul(self.R_T, data[3:6]) # Rotate angles to end-effector frames
        rot_pred_pose = np.concatenate([rot_pred_pos, rot_pred_ang])

        if self.get_parameter('verbose').get_parameter_value().bool_value:
            self.get_logger().info(f"Z (mm): {rot_pred_pose[2]:.2f} \t Rx (deg): {rot_pred_pose[3]:.2f} \t Ry (deg): {rot_pred_pose[4]:.2f}")

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

        self.cycle_counter +=1

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