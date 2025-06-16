import rclpy
from rclpy.node import Node

import numpy as np

from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Int8, Int32, Float64
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from px4_msgs.msg import VehicleOdometry, TrajectorySetpoint

class PoseBasedATS(Node):
    def __init__(self):
        super().__init__('pose_based_ats')

        # Parameters
        self.declare_parameter('frequency', 10.)
        self.declare_parameter('reference_pose', [0., 0., 0.])
        self.declare_parameter('Kp', 3.0)
        self.declare_parameter('Ki', 0.1)
        self.declare_parameter('windup_clip', 10.)
        self.declare_parameter('publish_log', True)
        self.declare_parameter('regularization_weight', 0.001)
        self.declare_parameter('test_execution_time', False)
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value
        self.integrator = np.zeros(6)
        self.windup = self.get_parameter('windup_clip').get_parameter_value().double_value
        self.reg_weight = self.get_parameter('regularization_weight').get_parameter_value().double_value

        # Subscribers
        self.subscription_tactip = self.create_subscription(TwistStamped, '/tactip/pose', self.callback_tactip, 10)
        self.subscription_tactip_contact = self.create_subscription(Int8, '/tactip/contact', self.callback_tactip_contact, 10)
        self.subscription_servos = self.create_subscription(JointState, '/servo/out/state', self.callback_servo, 10)
        self.subscription_fmu = self.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.callback_fmu, 10)
        self.subscription_md = self.create_subscription(Int32, '/md/state', self.md_callback, 10)

        # Publishers (necessary)
        self.publisher_servo_positions = self.create_publisher(JointState, '/controller/out/servo_positions', 10)
        self.publisher_drone_ref = self.create_publisher(TrajectorySetpoint, '/controller/out/trajectory_setpoint', 10)

        # Publishers (for logging and debugging)
        self.publisher_reference_sensor_pose_inertial = self.create_publisher(TwistStamped, '/controller/out/reference_inertial_sensor_pose', 10)
        self.publisher_reference_sensor_pose_contact = self.create_publisher(TwistStamped, '/controller/out/reference_contact_sensor_pose', 10)
        self.publisher_forward_kinematics = self.create_publisher(TwistStamped, '/controller/out/forward_kinematics', 10)
        self.publisher_error = self.create_publisher(TwistStamped, '/controller/out/error', 10)
        self.publisher_ik_check = self.create_publisher(TwistStamped, '/controller/out/ik_check', 10)
        self.publisher_correction = self.create_publisher(TwistStamped, '/controller/out/correction', 10)
        self.publisher_drone_ref_twist = self.create_publisher(TwistStamped, '/controller/out/trajectory_setpoint_twist', 10)
        self.publisher_drone_actual_position = self.create_publisher(TwistStamped, '/controller/out/vehicle_visual_odometry', 10)

        self.publisher_ki_error = self.create_publisher(Float64, '/controller/optimizer/ki_error', 10)
        self.publisher_regularization = self.create_publisher(Float64, '/controller/optimizer/regularization', 10)

        # Data
        reference_pose = self.get_parameter('reference_pose').get_parameter_value().double_array_value
        if len(reference_pose) != 3:
            self.get_logger().error("Parameter 'reference_pose' must be a list of 3 elements.")
            return
        self.P_Cref = self.evaluate_P_CS(
            reference_pose[0],
            reference_pose[1],
            reference_pose[2])

        self.tactip = TwistStamped()
        self.tactip.twist.linear.x = 0.0
        self.tactip.twist.linear.y = 0.0
        self.tactip.twist.linear.z = 0.0
        self.tactip.twist.angular.x = 0.0
        self.tactip.twist.angular.y = 0.0
        self.tactip.twist.angular.z = 0.0
        self.servo_state = JointState()
        self.servo_state.position = [0., 0., 0.]
        self.vehicle_odometry = VehicleOdometry()
        self.contact = False
        self.md_state = 0

        # Custom weighting matrix for the regularization of the full kinematics case
        self.weighting_matrix =  np.eye(9)
        self.weighting_matrix[0,0] = 1
        self.weighting_matrix[1,1] = 1
        self.weighting_matrix[2,2] = 1
        self.weighting_matrix[3,3] = 10 # Roll - high penalty
        self.weighting_matrix[4,4] = 10 # Pitch - high penalty
        self.weighting_matrix[5,5] = 1
        self.weighting_matrix[6,6] = 1 # Q1 - low penalty
        self.weighting_matrix[7,7] = 10 # Q2 - high penalty
        self.weighting_matrix[8,8] = 1 # Q3 - low penalty

        # Timer
        self.period = 1./self.get_parameter('frequency').get_parameter_value().double_value
        self.timer = self.create_timer(self.period, self.callback_timer)

    def callback_timer(self):
        # Get state
        state = self.get_state()
        # Publish data on self
        # Own position in TwistStamped message
        twistmsg = TwistStamped()
        twistmsg.twist.linear.x = state[0]
        twistmsg.twist.linear.y = state[1]
        twistmsg.twist.linear.z = state[2]
        twistmsg.twist.angular.x = state[3]
        twistmsg.twist.angular.y = state[4]
        twistmsg.twist.angular.z = state[5]
        twistmsg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_drone_actual_position.publish(twistmsg)

        # Evaluate the error - TacTip output is in deg and mm
        P_SC = self.evaluate_P_SC(self.tactip.twist.angular.x/180.*np.pi, self.tactip.twist.angular.y/180.*np.pi, self.tactip.twist.linear.z/1000.)
        E_Sref = P_SC @ self.P_Cref
        self.publish_transform(self.P_Cref, self.publisher_reference_sensor_pose_contact)
        self.publish_transform(E_Sref, self.publisher_error)
        e_sr = self.transformation_to_vector(E_Sref)

        # Check for contact through SSIM
        if self.md_state == 8: # If contact, accumulate integrator
            self.integrator += self.Ki * e_sr
        else: # If not contact, reset integrator
            self.integrator = 0.

        u_ss = self.Kp*e_sr + np.clip(self.integrator,-self.windup, self.windup)

        # Control law
        U_SS = self.vector_to_transformation(u_ss)
        P_S = self.forward_kinematics(state)

        # Publish the forward kinematics for reference
        self.publish_transform(P_S, self.publisher_forward_kinematics)
        self.publish_transform(U_SS, self.publisher_correction)
        P_Sref = P_S @ U_SS # Transform adjustment from sensor frame to inertial frame
        #P_Sref = self.forward_kinematics([0.0, 0.0, -1.5, 0.0, 0.0, 0.0, np.pi/3, 0.0, np.pi/6])

        # Publish the corrected reference sensor pose in inertial frame in vector form
        self.publish_transform(P_Sref, self.publisher_reference_sensor_pose_inertial)

        # Inverse kinematics
        result = self.inverse_kinematics(P_Sref)
        state_reference = result[0]

        if result[1]==True:
            #self.get_logger().debug(f"IK optimization converged with value {result[3]}")
            msg = TrajectorySetpoint()
            msg.position = [state_reference[0], state_reference[1], state_reference[2]]
            msg.yaw = state_reference[5]
            self.publisher_drone_ref.publish(msg)

            msg = JointState()
            msg.name = ['q1', 'q2', 'q3']
            msg.position = [state_reference[6], state_reference[7], state_reference[8]]
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_servo_positions.publish(msg)

            # Twist for plotjuggler
            msg = TwistStamped()
            msg.twist.linear.x = state_reference[0]
            msg.twist.linear.y = state_reference[1]
            msg.twist.linear.z = state_reference[2]
            msg.twist.angular.x = state_reference[3]
            msg.twist.angular.y = state_reference[4]
            msg.twist.angular.z = state_reference[5]
            msg.header.stamp = self.get_clock().now().to_msg()

            self.publisher_drone_ref_twist.publish(msg)

            # FK for checking
            check = self.forward_kinematics(state_reference)
            self.publish_transform(check, self.publisher_ik_check)

            # Publish kinematic inversion error
            self.publish_ki_error(self.kinematic_inversion_error(state_reference, P_Sref, self.get_state()))

        elif result[1]!=True:
            self.get_logger().error(f"IK optimization failed to converge with error {result[2]}")

    def callback_tactip(self, msg):
        self.tactip = msg

    def callback_tactip_contact(self, msg):
        self.contact = msg.data

    def callback_servo(self, msg):
        self.servo_state = msg

    def callback_fmu(self, msg):
        self.vehicle_odometry = msg
    
    def md_callback(self, msg):
        self.md_state = msg.data

    ''' Evaluate transformation matrix of contact frame in sensor frame
    '''
    def evaluate_P_SC(self, alpha, beta, d):
        P_SC = np.zeros((4,4))
        P_SC[0,0] = np.cos(beta)
        P_SC[0,1] = 0
        P_SC[0,2] = -np.sin(beta)
        P_SC[0,3] = d*np.sin(beta)
        P_SC[1,0] = np.sin(alpha)*np.sin(beta)
        P_SC[1,1] = np.cos(alpha)
        P_SC[1,2] = np.sin(alpha)*np.cos(beta)
        P_SC[1,3] = -d*np.sin(alpha)*np.cos(beta)
        P_SC[2,0] = np.sin(beta)*np.cos(alpha)
        P_SC[2,1] = -np.sin(alpha)
        P_SC[2,2] = np.cos(alpha)*np.cos(beta)
        P_SC[2,3] = -d*np.cos(alpha)*np.cos(beta)
        P_SC[3,0] = 0
        P_SC[3,1] = 0
        P_SC[3,2] = 0
        P_SC[3,3] = 1

        return P_SC
    
    ''' Evaluate transformation matrix of sensor frame in contact frame
    This is the alpha and beta that is the output of the TacTip
    '''
    def evaluate_P_CS(self, alpha, beta, d):
        P_CS = np.zeros((4,4))
        P_CS[0,0] = np.cos(beta)
        P_CS[0,1] = np.sin(alpha)*np.sin(beta)
        P_CS[0,2] = np.sin(beta)*np.cos(alpha)
        P_CS[0,3] = 0
        P_CS[1,0] = 0
        P_CS[1,1] = np.cos(alpha)
        P_CS[1,2] = -np.sin(alpha)
        P_CS[1,3] = 0
        P_CS[2,0] = -np.sin(beta)
        P_CS[2,1] = np.sin(alpha)*np.cos(beta)
        P_CS[2,2] = np.cos(alpha)*np.cos(beta)
        P_CS[2,3] = d
        P_CS[3,0] = 0
        P_CS[3,1] = 0
        P_CS[3,2] = 0
        P_CS[3,3] = 1
        return P_CS

    def evaluate_P_C(self, state, alpha, beta, d):
        x_B = state[0]
        y_B = state[1]
        z_B = state[2]
        roll = state[3]
        pitch =state[4]
        yaw = state[5]
        q_1 = state[6]
        q_2 = state[7]
        q_3 = state[8]
        P_C = np.zeros((4,4))
        P_C[0,0] = np.sin(beta)*np.sin(pitch)*np.sin(alpha - q_3)*np.sin(q_1 + roll)*np.cos(yaw) + np.sin(beta)*np.sin(pitch)*np.cos(q_2)*np.cos(yaw)*np.cos(alpha - q_3)*np.cos(q_1 + roll) + np.sin(beta)*np.sin(q_2)*np.cos(pitch)*np.cos(yaw)*np.cos(alpha - q_3) - np.sin(beta)*np.sin(yaw)*np.sin(alpha - q_3)*np.cos(q_1 + roll) + np.sin(beta)*np.sin(yaw)*np.sin(q_1 + roll)*np.cos(q_2)*np.cos(alpha - q_3) - np.sin(pitch)*np.sin(q_2)*np.cos(beta)*np.cos(yaw)*np.cos(q_1 + roll) - np.sin(q_2)*np.sin(yaw)*np.sin(q_1 + roll)*np.cos(beta) + np.cos(beta)*np.cos(pitch)*np.cos(q_2)*np.cos(yaw)
        P_C[0,1] = -np.sin(pitch)*np.sin(alpha - q_3)*np.cos(q_2)*np.cos(yaw)*np.cos(q_1 + roll) + np.sin(pitch)*np.sin(q_1 + roll)*np.cos(yaw)*np.cos(alpha - q_3) - np.sin(q_2)*np.sin(alpha - q_3)*np.cos(pitch)*np.cos(yaw) - np.sin(yaw)*np.sin(alpha - q_3)*np.sin(q_1 + roll)*np.cos(q_2) - np.sin(yaw)*np.cos(alpha - q_3)*np.cos(q_1 + roll)
        P_C[0,2] = np.sin(beta)*np.sin(pitch)*np.sin(q_2)*np.cos(yaw)*np.cos(q_1 + roll) + np.sin(beta)*np.sin(q_2)*np.sin(yaw)*np.sin(q_1 + roll) - np.sin(beta)*np.cos(pitch)*np.cos(q_2)*np.cos(yaw) + np.sin(pitch)*np.sin(alpha - q_3)*np.sin(q_1 + roll)*np.cos(beta)*np.cos(yaw) + np.sin(pitch)*np.cos(beta)*np.cos(q_2)*np.cos(yaw)*np.cos(alpha - q_3)*np.cos(q_1 + roll) + np.sin(q_2)*np.cos(beta)*np.cos(pitch)*np.cos(yaw)*np.cos(alpha - q_3) - np.sin(yaw)*np.sin(alpha - q_3)*np.cos(beta)*np.cos(q_1 + roll) + np.sin(yaw)*np.sin(q_1 + roll)*np.cos(beta)*np.cos(q_2)*np.cos(alpha - q_3)
        P_C[0,3] = -0.11*np.sin(pitch)*np.cos(yaw)*np.cos(q_1 + roll) - 0.11*np.sin(yaw)*np.sin(q_1 + roll) - 0.311*np.sin(pitch)*np.cos(q_2)*np.cos(yaw)*np.cos(q_1 + roll) - 0.311*np.sin(q_2)*np.cos(pitch)*np.cos(yaw) - 0.311*np.sin(yaw)*np.sin(q_1 + roll)*np.cos(q_2) + 0.273*np.sin(pitch)*np.sin(q_3)*np.sin(q_1 + roll)*np.cos(yaw) - 0.273*np.sin(pitch)*np.cos(q_2)*np.cos(q_3)*np.cos(yaw)*np.cos(q_1 + roll) - 0.273*np.sin(q_2)*np.cos(pitch)*np.cos(q_3)*np.cos(yaw) - 0.273*np.sin(q_3)*np.sin(yaw)*np.cos(q_1 + roll) - 0.273*np.sin(yaw)*np.sin(q_1 + roll)*np.cos(q_2)*np.cos(q_3) + x_B - d*np.sin(beta)*np.sin(pitch)*np.sin(q_2)*np.cos(yaw)*np.cos(q_1 + roll) - d*np.sin(beta)*np.sin(q_2)*np.sin(yaw)*np.sin(q_1 + roll) + d*np.sin(beta)*np.cos(pitch)*np.cos(q_2)*np.cos(yaw) - d*np.sin(pitch)*np.sin(alpha - q_3)*np.sin(q_1 + roll)*np.cos(beta)*np.cos(yaw) - d*np.sin(pitch)*np.cos(beta)*np.cos(q_2)*np.cos(yaw)*np.cos(alpha - q_3)*np.cos(q_1 + roll) - d*np.sin(q_2)*np.cos(beta)*np.cos(pitch)*np.cos(yaw)*np.cos(alpha - q_3) + d*np.sin(yaw)*np.sin(alpha - q_3)*np.cos(beta)*np.cos(q_1 + roll) - d*np.sin(yaw)*np.sin(q_1 + roll)*np.cos(beta)*np.cos(q_2)*np.cos(alpha - q_3)
        P_C[1,0] = np.sin(beta)*np.sin(pitch)*np.sin(yaw)*np.sin(alpha - q_3)*np.sin(q_1 + roll) + np.sin(beta)*np.sin(pitch)*np.sin(yaw)*np.cos(q_2)*np.cos(alpha - q_3)*np.cos(q_1 + roll) + np.sin(beta)*np.sin(q_2)*np.sin(yaw)*np.cos(pitch)*np.cos(alpha - q_3) + np.sin(beta)*np.sin(alpha - q_3)*np.cos(yaw)*np.cos(q_1 + roll) - np.sin(beta)*np.sin(q_1 + roll)*np.cos(q_2)*np.cos(yaw)*np.cos(alpha - q_3) - np.sin(pitch)*np.sin(q_2)*np.sin(yaw)*np.cos(beta)*np.cos(q_1 + roll) + np.sin(q_2)*np.sin(q_1 + roll)*np.cos(beta)*np.cos(yaw) + np.sin(yaw)*np.cos(beta)*np.cos(pitch)*np.cos(q_2)
        P_C[1,1] = -np.sin(pitch)*np.sin(yaw)*np.sin(alpha - q_3)*np.cos(q_2)*np.cos(q_1 + roll) + np.sin(pitch)*np.sin(yaw)*np.sin(q_1 + roll)*np.cos(alpha - q_3) - np.sin(q_2)*np.sin(yaw)*np.sin(alpha - q_3)*np.cos(pitch) + np.sin(alpha - q_3)*np.sin(q_1 + roll)*np.cos(q_2)*np.cos(yaw) + np.cos(yaw)*np.cos(alpha - q_3)*np.cos(q_1 + roll)
        P_C[1,2] = np.sin(beta)*np.sin(pitch)*np.sin(q_2)*np.sin(yaw)*np.cos(q_1 + roll) - np.sin(beta)*np.sin(q_2)*np.sin(q_1 + roll)*np.cos(yaw) - np.sin(beta)*np.sin(yaw)*np.cos(pitch)*np.cos(q_2) + np.sin(pitch)*np.sin(yaw)*np.sin(alpha - q_3)*np.sin(q_1 + roll)*np.cos(beta) + np.sin(pitch)*np.sin(yaw)*np.cos(beta)*np.cos(q_2)*np.cos(alpha - q_3)*np.cos(q_1 + roll) + np.sin(q_2)*np.sin(yaw)*np.cos(beta)*np.cos(pitch)*np.cos(alpha - q_3) + np.sin(alpha - q_3)*np.cos(beta)*np.cos(yaw)*np.cos(q_1 + roll) - np.sin(q_1 + roll)*np.cos(beta)*np.cos(q_2)*np.cos(yaw)*np.cos(alpha - q_3)
        P_C[1,3] = -0.11*np.sin(pitch)*np.sin(yaw)*np.cos(q_1 + roll) + 0.11*np.sin(q_1 + roll)*np.cos(yaw) - 0.311*np.sin(pitch)*np.sin(yaw)*np.cos(q_2)*np.cos(q_1 + roll) - 0.311*np.sin(q_2)*np.sin(yaw)*np.cos(pitch) + 0.311*np.sin(q_1 + roll)*np.cos(q_2)*np.cos(yaw) + 0.273*np.sin(pitch)*np.sin(q_3)*np.sin(yaw)*np.sin(q_1 + roll) - 0.273*np.sin(pitch)*np.sin(yaw)*np.cos(q_2)*np.cos(q_3)*np.cos(q_1 + roll) - 0.273*np.sin(q_2)*np.sin(yaw)*np.cos(pitch)*np.cos(q_3) + 0.273*np.sin(q_3)*np.cos(yaw)*np.cos(q_1 + roll) + 0.273*np.sin(q_1 + roll)*np.cos(q_2)*np.cos(q_3)*np.cos(yaw) + y_B - d*np.sin(beta)*np.sin(pitch)*np.sin(q_2)*np.sin(yaw)*np.cos(q_1 + roll) + d*np.sin(beta)*np.sin(q_2)*np.sin(q_1 + roll)*np.cos(yaw) + d*np.sin(beta)*np.sin(yaw)*np.cos(pitch)*np.cos(q_2) - d*np.sin(pitch)*np.sin(yaw)*np.sin(alpha - q_3)*np.sin(q_1 + roll)*np.cos(beta) - d*np.sin(pitch)*np.sin(yaw)*np.cos(beta)*np.cos(q_2)*np.cos(alpha - q_3)*np.cos(q_1 + roll) - d*np.sin(q_2)*np.sin(yaw)*np.cos(beta)*np.cos(pitch)*np.cos(alpha - q_3) - d*np.sin(alpha - q_3)*np.cos(beta)*np.cos(yaw)*np.cos(q_1 + roll) + d*np.sin(q_1 + roll)*np.cos(beta)*np.cos(q_2)*np.cos(yaw)*np.cos(alpha - q_3)
        P_C[2,0] = -np.sin(beta)*np.sin(pitch)*np.sin(q_2)*np.cos(alpha - q_3) + np.sin(beta)*np.sin(alpha - q_3)*np.sin(q_1 + roll)*np.cos(pitch) + np.sin(beta)*np.cos(pitch)*np.cos(q_2)*np.cos(alpha - q_3)*np.cos(q_1 + roll) - np.sin(pitch)*np.cos(beta)*np.cos(q_2) - np.sin(q_2)*np.cos(beta)*np.cos(pitch)*np.cos(q_1 + roll)
        P_C[2,1] = np.sin(pitch)*np.sin(q_2)*np.sin(alpha - q_3) - np.sin(alpha - q_3)*np.cos(pitch)*np.cos(q_2)*np.cos(q_1 + roll) + np.sin(q_1 + roll)*np.cos(pitch)*np.cos(alpha - q_3)
        P_C[2,2] = np.sin(beta)*np.sin(pitch)*np.cos(q_2) + np.sin(beta)*np.sin(q_2)*np.cos(pitch)*np.cos(q_1 + roll) - np.sin(pitch)*np.sin(q_2)*np.cos(beta)*np.cos(alpha - q_3) + np.sin(alpha - q_3)*np.sin(q_1 + roll)*np.cos(beta)*np.cos(pitch) + np.cos(beta)*np.cos(pitch)*np.cos(q_2)*np.cos(alpha - q_3)*np.cos(q_1 + roll)
        P_C[2,3] = -0.11*np.cos(pitch)*np.cos(q_1 + roll) + 0.311*np.sin(pitch)*np.sin(q_2) - 0.311*np.cos(pitch)*np.cos(q_2)*np.cos(q_1 + roll) + 0.273*np.sin(pitch)*np.sin(q_2)*np.cos(q_3) + 0.273*np.sin(q_3)*np.sin(q_1 + roll)*np.cos(pitch) - 0.273*np.cos(pitch)*np.cos(q_2)*np.cos(q_3)*np.cos(q_1 + roll) + z_B - d*np.sin(beta)*np.sin(pitch)*np.cos(q_2) - d*np.sin(beta)*np.sin(q_2)*np.cos(pitch)*np.cos(q_1 + roll) + d*np.sin(pitch)*np.sin(q_2)*np.cos(beta)*np.cos(alpha - q_3) - d*np.sin(alpha - q_3)*np.sin(q_1 + roll)*np.cos(beta)*np.cos(pitch) - d*np.cos(beta)*np.cos(pitch)*np.cos(q_2)*np.cos(alpha - q_3)*np.cos(q_1 + roll)
        P_C[3,0] = 0
        P_C[3,1] = 0
        P_C[3,2] = 0
        P_C[3,3] = 1

        return P_C

    ''' Get HTM describing end-effector (sensor) pose in inertial frame -> P_S, evaluated at latest state
    '''
    def forward_kinematics(self, state):
        x_B = state[0]
        y_B = state[1]
        z_B = state[2]
        roll = state[3]
        pitch = state[4]
        yaw = state[5]
        q_1 = state[6]
        q_2 = state[7]
        q_3 = state[8]

        P_S = np.zeros((4,4))
        P_S[0,0] = -(np.sin(pitch)*np.cos(yaw)*np.cos(q_1 + roll) + np.sin(yaw)*np.sin(q_1 + roll))*np.sin(q_2) + np.cos(pitch)*np.cos(q_2)*np.cos(yaw)
        P_S[0,1] = -(-(np.sin(pitch)*np.cos(yaw)*np.cos(q_1 + roll) + np.sin(yaw)*np.sin(q_1 + roll))*np.cos(q_2) - np.sin(q_2)*np.cos(pitch)*np.cos(yaw))*np.sin(q_3) + (np.sin(pitch)*np.sin(q_1 + roll)*np.cos(yaw) - np.sin(yaw)*np.cos(q_1 + roll))*np.cos(q_3)
        P_S[0,2] = -(-(np.sin(pitch)*np.cos(yaw)*np.cos(q_1 + roll) + np.sin(yaw)*np.sin(q_1 + roll))*np.cos(q_2) - np.sin(q_2)*np.cos(pitch)*np.cos(yaw))*np.cos(q_3) - (np.sin(pitch)*np.sin(q_1 + roll)*np.cos(yaw) - np.sin(yaw)*np.cos(q_1 + roll))*np.sin(q_3)
        P_S[0,3] = -0.11*np.sin(pitch)*np.cos(yaw)*np.cos(q_1 + roll) - 0.11*np.sin(yaw)*np.sin(q_1 + roll) - 0.311*np.sin(pitch)*np.cos(q_2)*np.cos(yaw)*np.cos(q_1 + roll) - 0.311*np.sin(q_2)*np.cos(pitch)*np.cos(yaw) - 0.311*np.sin(yaw)*np.sin(q_1 + roll)*np.cos(q_2) + 0.273*np.sin(pitch)*np.sin(q_3)*np.sin(q_1 + roll)*np.cos(yaw) - 0.273*np.sin(pitch)*np.cos(q_2)*np.cos(q_3)*np.cos(yaw)*np.cos(q_1 + roll) - 0.273*np.sin(q_2)*np.cos(pitch)*np.cos(q_3)*np.cos(yaw) - 0.273*np.sin(q_3)*np.sin(yaw)*np.cos(q_1 + roll) - 0.273*np.sin(yaw)*np.sin(q_1 + roll)*np.cos(q_2)*np.cos(q_3) + x_B
        P_S[1,0] = (-np.sin(pitch)*np.sin(yaw)*np.cos(q_1 + roll) + np.sin(q_1 + roll)*np.cos(yaw))*np.sin(q_2) + np.sin(yaw)*np.cos(pitch)*np.cos(q_2)
        P_S[1,1] = -((-np.sin(pitch)*np.sin(yaw)*np.cos(q_1 + roll) + np.sin(q_1 + roll)*np.cos(yaw))*np.cos(q_2) - np.sin(q_2)*np.sin(yaw)*np.cos(pitch))*np.sin(q_3) + (np.sin(pitch)*np.sin(yaw)*np.sin(q_1 + roll) + np.cos(yaw)*np.cos(q_1 + roll))*np.cos(q_3)
        P_S[1,2] = -((-np.sin(pitch)*np.sin(yaw)*np.cos(q_1 + roll) + np.sin(q_1 + roll)*np.cos(yaw))*np.cos(q_2) - np.sin(q_2)*np.sin(yaw)*np.cos(pitch))*np.cos(q_3) - (np.sin(pitch)*np.sin(yaw)*np.sin(q_1 + roll) + np.cos(yaw)*np.cos(q_1 + roll))*np.sin(q_3)
        P_S[1,3] = -0.11*np.sin(pitch)*np.sin(yaw)*np.cos(q_1 + roll) + 0.11*np.sin(q_1 + roll)*np.cos(yaw) - 0.311*np.sin(pitch)*np.sin(yaw)*np.cos(q_2)*np.cos(q_1 + roll) - 0.311*np.sin(q_2)*np.sin(yaw)*np.cos(pitch) + 0.311*np.sin(q_1 + roll)*np.cos(q_2)*np.cos(yaw) + 0.273*np.sin(pitch)*np.sin(q_3)*np.sin(yaw)*np.sin(q_1 + roll) - 0.273*np.sin(pitch)*np.sin(yaw)*np.cos(q_2)*np.cos(q_3)*np.cos(q_1 + roll) - 0.273*np.sin(q_2)*np.sin(yaw)*np.cos(pitch)*np.cos(q_3) + 0.273*np.sin(q_3)*np.cos(yaw)*np.cos(q_1 + roll) + 0.273*np.sin(q_1 + roll)*np.cos(q_2)*np.cos(q_3)*np.cos(yaw) + y_B
        P_S[2,0] = -np.sin(pitch)*np.cos(q_2) - np.sin(q_2)*np.cos(pitch)*np.cos(q_1 + roll)
        P_S[2,1] = -(np.sin(pitch)*np.sin(q_2) - np.cos(pitch)*np.cos(q_2)*np.cos(q_1 + roll))*np.sin(q_3) + np.sin(q_1 + roll)*np.cos(pitch)*np.cos(q_3)
        P_S[2,2] = -(np.sin(pitch)*np.sin(q_2) - np.cos(pitch)*np.cos(q_2)*np.cos(q_1 + roll))*np.cos(q_3) - np.sin(q_3)*np.sin(q_1 + roll)*np.cos(pitch)
        P_S[2,3] = -0.11*np.cos(pitch)*np.cos(q_1 + roll) + 0.311*np.sin(pitch)*np.sin(q_2) - 0.311*np.cos(pitch)*np.cos(q_2)*np.cos(q_1 + roll) + 0.273*np.sin(pitch)*np.sin(q_2)*np.cos(q_3) + 0.273*np.sin(q_3)*np.sin(q_1 + roll)*np.cos(pitch) - 0.273*np.cos(pitch)*np.cos(q_2)*np.cos(q_3)*np.cos(q_1 + roll) + z_B
        P_S[3,0] = 0
        P_S[3,1] = 0
        P_S[3,2] = 0
        P_S[3,3] = 1

        return P_S

    ''' Returns the full 9DOF states
    '''    
    def get_state(self):
        euler = self.quaternion_to_euler(self.vehicle_odometry.q)
        current_state = np.array([
            self.vehicle_odometry.position[0],
            self.vehicle_odometry.position[1],
            self.vehicle_odometry.position[2],
            euler[0],
            euler[1],
            euler[2],
            self.servo_state.position[0],
            self.servo_state.position[1],
            self.servo_state.position[2]
        ])
        return current_state
    
    # Auxiliary functions
    ''' Get rotation matrix corresponding to wxyz quaternion
    '''
    def quaternion_to_rotmat(self, quat):
        # Below is from Automatic Addision - it seems not to correspond to most other sources
        # First row of the rotation matrix
        r00 = 2 * (quat[0] * quat[0] + quat[1] * quat[1]) - 1
        r01 = 2 * (quat[1] * quat[2] - quat[0] * quat[3])
        r02 = 2 * (quat[1] * quat[3] + quat[0] * quat[2])
        
        # Second row of the rotation matrix
        r10 = 2 * (quat[1] * quat[2] + quat[0] * quat[3])
        r11 = 2 * (quat[0] * quat[0] + quat[2] * quat[2]) - 1
        r12 = 2 * (quat[2] * quat[3] - quat[0] * quat[1])
        
        # Third row of the rotation matrix
        r20 = 2 * (quat[1] * quat[3] - quat[0] * quat[2])
        r21 = 2 * (quat[2] * quat[3] + quat[0] * quat[1])
        r22 = 2 * (quat[0] * quat[0] + quat[3] * quat[3]) - 1
        
        # 3x3 rotation matrix
        rotmat = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                
        return rotmat

    def rotmat_to_quaternion(self, rotmat):
        pass

    def rotmat_to_euler(self,rotmat):
        euler = np.zeros(3)
        if rotmat[2,0]!=1 and rotmat[2,0]!=-1:
            euler[1] = -np.arcsin(rotmat[2,0])

        euler[0] = np.arctan2(rotmat[2,1], rotmat[2,2])
        euler[1] = -np.arcsin(rotmat[2,0])
        euler[2] = np.arctan2(rotmat[1,0], rotmat[0,0])

        return euler

    def euler_to_rotmat(self, euler):
        pass

    def quaternion_to_euler(self, quat):
        # First quat to rotmat
        rotmat = self.quaternion_to_rotmat(quat)
        # Then rotmat to euler
        return self.rotmat_to_euler(rotmat)

    def transformation_to_vector(self, HTM):
        vector = np.zeros(6)
        vector[0] = HTM[0,3]
        vector[1] = HTM[1,3]
        vector[2] = HTM[2,3]
        vector[3] = np.arctan2(HTM[2,1], HTM[2,2])  # Rx, roll
        vector[4] = -np.arcsin(HTM[2,0])            # Ry, pitch
        vector[5] = np.arctan2(HTM[1,0], HTM[0,0])  # Rz, yaw

        return vector

    ''' Get homogeneous transformation matrix from pose vector (XYZ, RPY) with the (intrinsic) Z-Y'-X" or (extrinsic) XYZ convention. 
    '''    
    def vector_to_transformation(self, vector):
        roll = vector[3]
        pitch = vector[4]
        yaw = vector[5]
        HTM = np.array(
            [[np.cos(yaw)*np.cos(pitch), -np.sin(yaw)*np.cos(roll) + np.sin(pitch)*np.sin(roll)*np.cos(yaw), np.sin(yaw)*np.sin(roll) + np.sin(pitch)*np.cos(yaw)*np.cos(roll), vector[0]], 
             [np.sin(yaw)*np.cos(pitch), np.sin(yaw)*np.sin(pitch)*np.sin(roll) + np.cos(yaw)*np.cos(roll), np.sin(yaw)*np.sin(pitch)*np.cos(roll) - np.sin(roll)*np.cos(yaw), vector[1]], 
             [-np.sin(pitch), np.sin(roll)*np.cos(pitch), np.cos(pitch)*np.cos(roll), vector[2]],
             [0., 0., 0., 1.]])
        return HTM

    def kinematic_inversion_error(self, state, P_des, current_state):
        P_S = self.forward_kinematics(state)
        # Position error (Euclidean distance)
        pos_err = np.linalg.norm(P_S[:3, 3] - P_des[:3, 3])

        # Orientation error (rotation angle difference)
        R1 = P_S[:3, :3]
        R2 = P_des[:3, :3]
        delta_R = R.from_matrix(R1.T @ R2)
        ang_err = np.linalg.norm(delta_R.as_rotvec())

        ki_error = pos_err**2 + ang_err**2 # Total inverse kinematic error
        regularization = self.reg_weight * np.linalg.norm(state - current_state)**2
        return [ki_error, regularization]

    # Inverse kinematics stuff
    def ik_objective(self, state, P_des, current_state):
        P_S = self.forward_kinematics(state)

        # Position error (Euclidean distance)
        pos_err = np.linalg.norm(P_S[:3, 3] - P_des[:3, 3])

        # Orientation error (rotation angle difference)
        R1 = P_S[:3, :3]
        R2 = P_des[:3, :3]
        delta_R = R.from_matrix(R1.T @ R2)
        ang_err = np.linalg.norm(delta_R.as_rotvec())

        error = pos_err**2 + ang_err**2
        regularization = self.reg_weight * np.matmul((state-current_state), np.matmul(self.weighting_matrix, (state-current_state)))
        return error + regularization

    def inverse_kinematics(self, P_des, bounds=None):
        # Bounds
        lower_state_bounds = [None, None, None, -np.pi/4, -np.pi/4, -np.pi, -0.1, -np.pi/8, -np.pi/2]
        upper_state_bounds = [None, None, None, np.pi/4, np.pi/4, np.pi, np.pi, np.pi/8, np.pi/2]
        bounds = list(zip(lower_state_bounds, upper_state_bounds))

        # State
        current_state = self.get_state()

        result = minimize(
            fun=self.ik_objective,
            x0=current_state,
            args=(P_des, current_state),
            bounds=bounds,
            method='SLSQP',
            options={'ftol': 1e-6, 'disp': False}
        )
        return result.x, result.success, result.message, result.fun   
    
    def publish_transform(self, T, publisher):
        vec = self.transformation_to_vector(T)
        msg = TwistStamped()
        msg.twist.linear.x = vec[0]
        msg.twist.linear.y = vec[1]
        msg.twist.linear.z = vec[2]
        msg.twist.angular.x = vec[3]
        msg.twist.angular.y = vec[4]
        msg.twist.angular.z = vec[5]

        msg.header.stamp = self.get_clock().now().to_msg()
        publisher.publish(msg)

    def publish_ki_error(self, error):
        ki_msg = Float64()
        ki_msg.data = error[0]
        self.publisher_ki_error.publish(ki_msg)

        reg_msg = Float64()
        reg_msg.data = error[1]
        self.publisher_regularization.publish(reg_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseBasedATS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()