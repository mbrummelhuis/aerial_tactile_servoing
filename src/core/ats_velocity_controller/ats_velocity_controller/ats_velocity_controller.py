import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import TwistStamped

class ATSVelocityController(Node):
    '''
    Aerial Tactile Servoing end-effector velocity controller node. This node takes in the estimated sensor pose from the TacTip
    and the reference pose from the planner and computes virtual end-effector velocity commands to be sent to the IK node.
    In short, this is the PID node.
    '''
    def __init__(self):
        super().__init__('ats_velocity_controller')

        # Parameters
        self.declare_parameter('frequency', 10.)
        self.declare_parameter('kp', 1.0) # Proportional gain
        self.declare_parameter('ki', 0.0) # Integral gain
        self.declare_parameter('kd', 0.0) # Derivative gain
        self.declare_parameter('max_integral', 1.0) # Integrator saturation limit
        self.declare_parameter('ewma_alpha', 0.3) # Exponential moving average alpha for derivative smoothing

        self.gain_proportional = self.get_parameter('kp').get_parameter_value().double_value
        self.gain_integral = self.get_parameter('ki').get_parameter_value().double_value
        self.gain_derivative = self.get_parameter('kd').get_parameter_value().double_value
        self.max_integral = self.get_parameter('max_integral').get_parameter_value().double_value
        self.ewma_alpha = self.get_parameter('ewma_alpha').get_parameter_value().double_value

        # Initialization log message
        self.get_logger().info("Velocity controller node initialized")

        # Publishers
        self.ee_reference_velocity_subscriber_ = self.create_publisher(TwistStamped, '/references/ee_velocity', 10)

        # Subscribers
        self.ee_measured_pose_subscriber_ = self.create_subscription(TwistStamped, '/tactip/pose', self.ee_measured_pose_callback, 10)
        self.ee_reference_pose_subscriber_ = self.create_subscription(TwistStamped, '/references/ee_pose', self.ee_reference_pose_callback, 10)

        # data
        self.tactip_pose_ = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.reference_pose_ = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.integral = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.derivative = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.previous_error = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # timer
        self.period = 1.0/float(self.get_parameter('frequency').get_parameter_value().double_value) # seconds
        self.timer = self.create_timer(self.period, self.timer_callback)

    '''
    Execute control loop
    '''
    def timer_callback(self):
        # Compute the error
        error = self.reference_pose_ - self.tactip_pose_

        # Idk why but the depth is a bit weird
        error = error*-1.0

        # Convert to meters and rads (from mm and degs)
        error[0:3] = error[0:3]/1000.0
        error[3:6] = np.deg2rad(error[3:6])

        # integral with saturation
        self.integral = np.clip(self.integral + self.gain_integral*error*self.period, -self.max_integral, self.max_integral)

        # Derivative term with EWMA smoothing
        self.derivative = self.gain_derivative * (self.ewma_alpha*error + (1-self.ewma_alpha)*self.previous_error)
        self.previous_error = error
        self.get_logger().debug("Derivative: {}".format(self.derivative))

        # Compute the control signal (end-effector velocity)
        control_signal = self.gain_proportional*error + self.integral + self.derivative

        # Construct the message
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = control_signal[0] # [m/s] end effector x velocity in contact frame
        msg.twist.linear.y = control_signal[1] # [m/s] end effector y velocity in contact frame
        msg.twist.linear.z = control_signal[2] # [m/s] end effector z velocity in contact frame
        msg.twist.angular.z = control_signal[3] # [rad/s] end effector yaw rate in contact frame
        msg.twist.angular.y = control_signal[4] # [rad/s] end effector pitch rate in contact frame
        msg.twist.angular.x = control_signal[5] # [rad/s] end effector roll rate in contact frame
        self.ee_reference_velocity_subscriber_.publish(msg)

    def ee_measured_pose_callback(self, msg):
        self.tactip_pose_[0]=msg.twist.linear.x # [mm] end effector measured x in contact frame (6d)
        self.tactip_pose_[1]=msg.twist.linear.y # [mm] end effector measured y in contact frame (6d)
        self.tactip_pose_[2]=msg.twist.linear.z # [mm] end effector measured z in contact frame (3d)
        self.tactip_pose_[3]=msg.twist.angular.z # [deg] end effector measured yaw in contact frame (6d)
        self.tactip_pose_[4]=msg.twist.angular.y # [deg] end effector measured pitch in contact frame (3d)
        self.tactip_pose_[5]=msg.twist.angular.x # [deg] end effector measured roll in contact frame (3d)

    def ee_reference_pose_callback(self, msg):
        self.reference_pose_[0]=msg.twist.linear.x # [mm] end effector x ref in contact frame (6d) m
        self.reference_pose_[1]=msg.twist.linear.y # [mm] end effector y ref in contact frame (6d) m
        self.reference_pose_[2]=msg.twist.linear.z # [mm] end effector z ref in contact frame (3d) m
        self.reference_pose_[3]=msg.twist.angular.z # [deg] end effector yaw ref in contact frame (6d)
        self.reference_pose_[4]=msg.twist.angular.y # [deg] end effector pitch ref in contact frame (3d)
        self.reference_pose_[5]=msg.twist.angular.x # [deg] end effector roll ref in contact frame (3d) 

def main(args=None):
    rclpy.init(args=args)
    node = ATSVelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()