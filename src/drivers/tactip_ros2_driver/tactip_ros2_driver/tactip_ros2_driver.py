import rclpy
from rclpy.node import Node
from math import pi
import time
import os
import math
import numpy as np
from skimage.metrics import structural_similarity as ssim

from geometry_msgs.msg import TwistStamped, TransformStamped
from std_msgs.msg import Float64, Int8
from tf2_ros import TransformBroadcaster

from .tactip import TacTip

from .dependencies.label_encoder import BASE_MODEL_PATH


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class TactipDriver(Node):
    def __init__(self):
        super().__init__('tactip_driver')

        # Parameters
        self.declare_parameter('source', 4)
        self.declare_parameter('frequency', 10.)
        self.declare_parameter('verbose', False)
        self.declare_parameter('test_model_time', False)
        self.declare_parameter('save_debug_image', False)
        self.declare_parameter('save_interval', 1.)
        self.declare_parameter('ssim_contact_threshold', 0.7)
        self.declare_parameter('save_directory', 'Please set a save_directory in the launch file')

        # Image saving interval
        self.image_save_interval = self.get_parameter('save_interval').get_parameter_value().double_value

        # Instantiate Tactip sensor
        self.sensor = TacTip(self.get_parameter('source').get_parameter_value().integer_value)
        self.ssim_threshold = self.get_parameter('ssim_contact_threshold').get_parameter_value().double_value

        # Node feedback
        self.get_logger().info("Tactip driver initialized")
        self.get_logger().info(BASE_MODEL_PATH)
        self.get_logger().info(f"Reading from /dev/video{self.get_parameter('source').get_parameter_value().integer_value}" )
        self.get_logger().info(f"Sensor processing params: {self.sensor.sensor_params}")


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
        self.publisher_pose_ = self.create_publisher(TwistStamped, '/tactip/pose', 10)
        self.publisher_ssim_ = self.create_publisher(Float64, '/tactip/ssim', 10)
        self.publisher_contact_ = self.create_publisher(Int8, '/tactip/contact', 10)

        # Broadcaster TF
        self.broadcaster_tf = TransformBroadcaster(self)

        # Run testing model evaluation time functionality if enabled in Launch
        if self.get_parameter('test_model_time').get_parameter_value().bool_value:
            self.get_logger().info("Testing model execution time. It will run 1000 predictions through the model and print the average time taken.")
            self.test_model_execution_time()

        # Rotation between output in actual frame and the end-effector frame
        self.R_T = np.array([[1, 0, 0], [0, -1, 0],[0, 0, -1]])

        # Reference image
        self.ref_image_ssim = self.sensor.process().squeeze()

        # Set up timer
        self.cycle_counter = 0
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.period = 1.0/self.frequency # seconds
        self.timer = self.create_timer(self.period, self.timer_callback)

    def timer_callback(self):
        # read the data
        # Save an image every second
        if self.get_parameter('save_debug_image').get_parameter_value().bool_value and self.cycle_counter%int(self.frequency*self.image_save_interval) == 0:
            raw_outfile = os.path.join(self.image_outfile_path,'raw_image'+str(self.img_counter)+'.png')
            proc_outfile = os.path.join(self.image_outfile_path,'sensor_image'+str(self.img_counter)+'.png')
            sensor_image = self.sensor.process(raw_outfile=raw_outfile, proc_outfile=proc_outfile)
            self.img_counter += 1
            self.cycle_counter = 0
        
        # Just read data without saving
        else:
            sensor_image = self.sensor.process()
        
        # Get SSIM
        ssim_score = ssim(self.ref_image_ssim, sensor_image.squeeze())
        msg = Float64()
        msg.data = ssim_score
        self.publisher_ssim_.publish(msg)

        # Deduce contact
        if ssim_score < self.ssim_threshold:
            contact = True
        else:
            contact = False
        msg = Int8()
        msg.data = contact
        self.publisher_contact_.publish(msg)

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
        msg.twist.linear.x = rot_pred_pose[0]/1000.
        msg.twist.linear.y = rot_pred_pose[1]/1000.
        msg.twist.linear.z = rot_pred_pose[2]/1000.
        msg.twist.angular.x = rot_pred_pose[3]*0.0174532925 # deg2rad
        msg.twist.angular.y = rot_pred_pose[4]*0.0174532925 # deg2rad
        msg.twist.angular.z = rot_pred_pose[5]*0.0174532925 # deg2rad
        self.publisher_pose_.publish(msg)
        #self.get_logger().info(f"Published data: {msg}")

        # Broadcast the TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "contact_frame"
        t.child_frame_id = "sensor_frame"
        t.transform.translation.x = rot_pred_pose[0]/1000.
        t.transform.translation.y = rot_pred_pose[1]/1000.
        t.transform.translation.z = rot_pred_pose[2]/1000.
        q = quaternion_from_euler(rot_pred_pose[3]*0.0174532925, rot_pred_pose[4]*0.0174532925, rot_pred_pose[5]*0.0174532925)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.broadcaster_tf.sendTransform(t)

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