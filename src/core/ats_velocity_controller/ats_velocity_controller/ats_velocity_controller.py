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
        self.declare_parameter('ewma_alpha', 0.3) # Exponential moving average alpha for derivatieve smoothing

        # Initialization log message
        self.get_logger().info("Velocity controller node initialized")

        # Publishers
        self.ee_reference_velocity_subscriber_ = self.create_publisher(TwistStamped, '/references/ee_velocity', 10)

        # Subscribers
        self.ee_measured_pose_subscriber_ = self.create_subscription(TwistStamped, '/sensors/tactip', self.ee_measured_pose_callback, 10)
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

        # integral with saturation
        max_integral = self.get_parameter('max_integral').get_parameter_value().double_value
        integral_gain = self.get_parameter('ki').get_parameter_value().double_value
        self.integral = np.clip(self.integral + integral_gain*error*self.period, -max_integral, max_integral)

        # Derivative term with EWMA smoothing
        ewma_alpha = self.get_parameter('ewma_alpha').get_parameter_value().double_value
        derivative_gain = self.get_parameter('kd').get_parameter_value().double_value
        self.derivative = derivative_gain * (ewma_alpha*error + (1-ewma_alpha)*self.previous_error)
        self.previous_error = error

        # Compute the control signal (end-effector velocity)
        proportional_gain = self.get_parameter('kp').get_parameter_value().double_value
        control_signal = proportional_gain*error + self.integral + self.derivative

        # Construct the message
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = control_signal[0]
        msg.twist.linear.y = control_signal[1]
        msg.twist.linear.z = control_signal[2]
        msg.twist.angular.x = control_signal[3]
        msg.twist.angular.y = control_signal[4]
        msg.twist.angular.z = control_signal[5]
        self.ee_reference_velocity_subscriber_.publish(msg)

    def ee_measured_pose_callback(self, msg):
        self.tactip_pose_[0]=msg.twist.linear.x # end effector measured x in contact frame (6d)
        self.tactip_pose_[1]=msg.twist.linear.y # end effector measured y in contact frame (6d)
        self.tactip_pose_[2]=msg.twist.linear.z # end effector measured z in contact frame (3d)
        self.tactip_pose_[3]=msg.twist.angular.z # end effector measured yaw in contact frame (6d) TODO: UNITS?
        self.tactip_pose_[4]=msg.twist.angular.y # end effector measured pitch in contact frame (3d)
        self.tactip_pose_[5]=msg.twist.angular.x # end effector measured roll in contact frame (3d)

    def ee_reference_pose_callback(self, msg):
        self.reference_pose_[0]=msg.twist.linear.x # end effector x ref in contact frame (6d)
        self.reference_pose_[1]=msg.twist.linear.y # end effector y ref in contact frame (6d)
        self.reference_pose_[2]=msg.twist.linear.z # end effector z ref in contact frame (3d)
        self.reference_pose_[3]=msg.twist.angular.z # end effector yaw ref in contact frame (6d) TODO: UNITS?
        self.reference_pose_[4]=msg.twist.angular.y # end effector pitch ref in contact frame (3d)
        self.reference_pose_[5]=msg.twist.angular.x # end effector roll ref in contact frame (3d)

def main(args=None):
    rclpy.init(args=args)
    node = ATSVelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()