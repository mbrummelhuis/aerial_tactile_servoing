import rclpy
from rclpy.node import Node

import numpy as np

from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from px4_msgs.msg import VehicleOdometry, TrajectorySetpoint

class PoseBasedATS(Node):
    def __init__(self):
        super().__init__('pose_based_ats')

        # Parameters
        self.declare_parameter('frequency', 10.)
        self.declare_parameter('reference pose', [0., 0., 0.])
        self.declare_parameter('Kp', 3.0)
        self.declare_parameter('Ki', 0.1)
        self.declare_parameter('windup clip', 10.)
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value
        self.windup = self.get_parameter('windup clip').get_parameter_value().double_value

        # Subscribers
        self.subscription_tactip = self.create_subscription(TwistStamped, '/sensors/tactip', self.callback_tactip, 10)
        self.subscription_servos = self.create_subscription(JointState, '/servo/out/state', self.callback_servo, 10)
        self.subscription_fmu = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.callback_fmu, 10)

        # Publishers
        self.publisher_reference_sensor_pose = self.create_publisher(TwistStamped, '/references/ee_velocity', 10)
        self.publisher_servo_positions = self.create_publisher(JointState, '/servo/in/reference', 10)
        self.publisher_drone_ref = self.create_publisher(TrajectorySetpoint, '/controller/out/trajectory_setpoint', 10)

        # Data
        self.P_SC = np.zeros((4,4))
        self.P_Cref = self.evaluate_P_SC(
            self.get_parameter('reference pose').get_parameter_value().double_array_value[0],
            self.get_parameter('reference pose').get_parameter_value().double_array_value[1],
            self.get_parameter('reference pose').get_parameter_value().double_array_value[2])
        self.tactip = TwistStamped()
        self.servo_state = JointState()
        self.vehicle_odometry = VehicleOdometry()

        # Timer
        self.period = 1./self.get_parameter('frequency').get_parameter_value().double_value
        self.timer = self.create_timer(self.period, self.callback_timer)

    # Callbacks
    def callback_timer(self):
        # Evaluate the error
        self.evaluate_P_SC(self.tactip.twist.angular.x, self.tactip.twist.angular.y, self.tactip.twist.linear.z)
        E_Sref = self.P_SC@self.P_Cref
        e_sr = self.transformation_to_vector(E_Sref)

        # Control law TODO: add integral controller
        u_ss = self.Kp*np.eye(6)@e_sr 

        U_SS = self.vector_to_transformation(u_ss)

        P_S = self.forward_kinematics()
        P_Sref = P_S @ U_SS

        # Publish the corrected reference sensor pose in inertial frame in vector form
        p_Sref = self.transformation_to_vector(P_Sref)
        msg = TwistStamped()
        msg.twist.linear.x = p_Sref[0]
        msg.twist.linear.x = p_Sref[1]
        msg.twist.linear.x = p_Sref[2]
        msg.twist.angular.x = p_Sref[3]
        msg.twist.angular.x = p_Sref[4]
        msg.twist.angular.x = p_Sref[5]

        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_reference_sensor_pose.publish(msg)

        # Inverse kinematics
        result = self.inverse_kinematics(P_Sref)
        state_reference = result[0]
        if result[1]==True:
            self.get_logger().info(f"IK optimization converged with value {result[3]}")
            msg = TrajectorySetpoint()
            msg.position[0] = state_reference[0]
            msg.position[1] = state_reference[1]
            msg.position[2] = state_reference[2]
            msg.yaw = state_reference[5]
            self.publisher_drone_ref.publish(msg)

            msg = JointState()
            msg.name = ['q1', 'q2', 'q3']
            msg.position[0] = state_reference[6]
            msg.position[1] = state_reference[7]
            msg.position[2] = state_reference[8]
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_servo_positions.publish(msg)

        elif result[1]!=True:
            self.get_logger().error(f"IK optimization failed to converge with error {result[2]}")

    def callback_tactip(self, msg):
        self.tactip = msg

    def callback_servo(self, msg):
        self.servo_state = msg

    def callback_fmu(self, msg):
        self.vehicle_odometry = msg

    ''' Evaluate transformation matrix of contact frame in sensor frame
    '''
    def evaluate_P_SC(self, alpha, beta, d):
        self.P_SC[0,0] = np.cos(beta)
        self.P_SC[0,1] = 0
        self.P_SC[0,2] = -np.sin(beta)
        self.P_SC[0,3] = d*np.sin(beta)
        self.P_SC[1,0] = np.sin(alpha)*np.sin(beta)
        self.P_SC[1,1] = np.cos(alpha)
        self.P_SC[1,2] = np.sin(alpha)*np.cos(beta)
        self.P_SC[1,3] = -d*np.sin(alpha)*np.cos(beta)
        self.P_SC[2,0] = np.sin(beta)*np.cos(alpha)
        self.P_SC[2,1] = -np.sin(alpha)
        self.P_SC[2,2] = np.cos(alpha)*np.cos(beta)
        self.P_SC[2,3] = -d*np.cos(alpha)*np.cos(beta)
        self.P_SC[3,0] = 0
        self.P_SC[3,1] = 0
        self.P_SC[3,2] = 0
        self.P_SC[3,3] = 1

        return self.P_SC

    ''' Get HTM describing end-effector (sensor) pose in inertial frame, evaluated at latest state
    '''
    def forward_kinematics(self):
        x_B = self.vehicle_odometry.position[0]
        y_B = self.vehicle_odometry.position[1]
        z_B = self.vehicle_odometry.position[2]
        
        euler = self.quaternion_to_euler(self.vehicle_odometry.q)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        q_1 = self.servo_state.position[0]
        q_2 = self.servo_state.position[1]
        q_3 = self.servo_state.position[2]

        P_S = np.zeros((4,4))
        P_S[0,0] = -(np.sin(yaw)*np.sin(q_1 + roll) + np.sin(pitch)*np.cos(yaw)*np.cos(q_1 + roll))*np.sin(q_2) + np.cos(yaw)*np.cos(q_2)*np.cos(pitch)
        P_S[0,1] = -(-(np.sin(yaw)*np.sin(q_1 + roll) + np.sin(pitch)*np.cos(yaw)*np.cos(q_1 + roll))*np.cos(q_2) - np.sin(q_2)*np.cos(yaw)*np.cos(pitch))*np.sin(q_3) + (-np.sin(yaw)*np.cos(q_1 + roll) + np.sin(pitch)*np.sin(q_1 + roll)*np.cos(yaw))*np.cos(q_3)
        P_S[0,2] = -(-(np.sin(yaw)*np.sin(q_1 + roll) + np.sin(pitch)*np.cos(yaw)*np.cos(q_1 + roll))*np.cos(q_2) - np.sin(q_2)*np.cos(yaw)*np.cos(pitch))*np.cos(q_3) - (-np.sin(yaw)*np.cos(q_1 + roll) + np.sin(pitch)*np.sin(q_1 + roll)*np.cos(yaw))*np.sin(q_3)
        P_S[0,3] = -0.11*np.sin(yaw)*np.sin(q_1 + roll) - 0.11*np.sin(pitch)*np.cos(yaw)*np.cos(q_1 + roll) - 0.311*np.sin(yaw)*np.sin(q_1 + roll)*np.cos(q_2) - 0.311*np.sin(q_2)*np.cos(yaw)*np.cos(pitch) - 0.311*np.sin(pitch)*np.cos(yaw)*np.cos(q_2)*np.cos(q_1 + roll) - 0.273*np.sin(yaw)*np.sin(q_3)*np.cos(q_1 + roll) - 0.273*np.sin(yaw)*np.sin(q_1 + roll)*np.cos(q_2)*np.cos(q_3) - 0.273*np.sin(q_2)*np.cos(yaw)*np.cos(q_3)*np.cos(pitch) + 0.273*np.sin(q_3)*np.sin(pitch)*np.sin(q_1 + roll)*np.cos(yaw) - 0.273*np.sin(pitch)*np.cos(yaw)*np.cos(q_2)*np.cos(q_3)*np.cos(q_1 + roll) + x_B
        P_S[1,0] = (-np.sin(yaw)*np.sin(pitch)*np.cos(q_1 + roll) + np.sin(q_1 + roll)*np.cos(yaw))*np.sin(q_2) + np.sin(yaw)*np.cos(q_2)*np.cos(pitch)
        P_S[1,1] = -((-np.sin(yaw)*np.sin(pitch)*np.cos(q_1 + roll) + np.sin(q_1 + roll)*np.cos(yaw))*np.cos(q_2) - np.sin(yaw)*np.sin(q_2)*np.cos(pitch))*np.sin(q_3) + (np.sin(yaw)*np.sin(pitch)*np.sin(q_1 + roll) + np.cos(yaw)*np.cos(q_1 + roll))*np.cos(q_3)
        P_S[1,2] = -((-np.sin(yaw)*np.sin(pitch)*np.cos(q_1 + roll) + np.sin(q_1 + roll)*np.cos(yaw))*np.cos(q_2) - np.sin(yaw)*np.sin(q_2)*np.cos(pitch))*np.cos(q_3) - (np.sin(yaw)*np.sin(pitch)*np.sin(q_1 + roll) + np.cos(yaw)*np.cos(q_1 + roll))*np.sin(q_3)
        P_S[1,3] = -0.11*np.sin(yaw)*np.sin(pitch)*np.cos(q_1 + roll) + 0.11*np.sin(q_1 + roll)*np.cos(yaw) - 0.311*np.sin(yaw)*np.sin(q_2)*np.cos(pitch) - 0.311*np.sin(yaw)*np.sin(pitch)*np.cos(q_2)*np.cos(q_1 + roll) + 0.311*np.sin(q_1 + roll)*np.cos(yaw)*np.cos(q_2) - 0.273*np.sin(yaw)*np.sin(q_2)*np.cos(q_3)*np.cos(pitch) + 0.273*np.sin(yaw)*np.sin(q_3)*np.sin(pitch)*np.sin(q_1 + roll) - 0.273*np.sin(yaw)*np.sin(pitch)*np.cos(q_2)*np.cos(q_3)*np.cos(q_1 + roll) + 0.273*np.sin(q_3)*np.cos(yaw)*np.cos(q_1 + roll) + 0.273*np.sin(q_1 + roll)*np.cos(yaw)*np.cos(q_2)*np.cos(q_3) + y_B
        P_S[2,0] = -np.sin(q_2)*np.cos(pitch)*np.cos(q_1 + roll) - np.sin(pitch)*np.cos(q_2)
        P_S[2,1] = -(np.sin(q_2)*np.sin(pitch) - np.cos(q_2)*np.cos(pitch)*np.cos(q_1 + roll))*np.sin(q_3) + np.sin(q_1 + roll)*np.cos(q_3)*np.cos(pitch)
        P_S[2,2] = -(np.sin(q_2)*np.sin(pitch) - np.cos(q_2)*np.cos(pitch)*np.cos(q_1 + roll))*np.cos(q_3) - np.sin(q_3)*np.sin(q_1 + roll)*np.cos(pitch)
        P_S[2,3] = -0.11*np.cos(pitch)*np.cos(q_1 + roll) + 0.311*np.sin(q_2)*np.sin(pitch) - 0.311*np.cos(q_2)*np.cos(pitch)*np.cos(q_1 + roll) + 0.273*np.sin(q_2)*np.sin(pitch)*np.cos(q_3) + 0.273*np.sin(q_3)*np.sin(q_1 + roll)*np.cos(pitch) - 0.273*np.cos(q_2)*np.cos(q_3)*np.cos(pitch)*np.cos(q_1 + roll) + z_B
        P_S[3,0] = 0
        P_S[3,1] = 0
        P_S[3,2] = 0
        P_S[3,3] = 1

        return P_S
    
    # Auxiliary functions
    ''' Get rotation matrix corresponding to wxyz quaternion
    '''
    def quaternion_to_rotmat(self, quat):
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
        euler = np.array(3)
        euler[0] = np.arctan2(rotmat[1,0], rotmat[0,0])
        euler[1] = -np.arcsin(rotmat[2,0])
        euler[2] = np.arctan2(rotmat[2,1], rotmat[2,2])        

    def euler_to_rotmat(self, euler):
        pass

    def quaternion_to_euler(self, quat):
        # First quat to rotmat
        rotmat = self.quaternion_to_rotmat(quat)
        # Then rotmat to euler
        return self.rotmat_to_euler(rotmat)

    def transformation_to_vector(self, HTM):
        vector = np.array(6)
        vector[0] = HTM[0,3]
        vector[1] = HTM[1,3]
        vector[2] = HTM[2,3]
        vector[3] = np.arctan2(HTM[1,0], HTM[0,0])
        vector[4] = -np.arcsin(HTM[2,0])
        vector[5] = np.arctan2(HTM[2,1], HTM[2,2])

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

    # Inverse kinematics stuff
    ''' Returns a scalar pose error between two 4x4 homogeneous matrices.
    '''
    def pose_error(self, T1, T2, position_weight=1.0, orientation_weight=1.0):
        # Position error (Euclidean distance)
        pos_err = np.linalg.norm(T1[:3, 3] - T2[:3, 3])

        # Orientation error (rotation angle difference)
        R1 = T1[:3, :3]
        R2 = T2[:3, :3]
        delta_R = R.from_matrix(R1.T @ R2)
        ang_err = np.linalg.norm(delta_R.as_rotvec())

        return position_weight * pos_err**2 + orientation_weight * ang_err**2

    def ik_objective(self, state, P_des, current_state, reg_weight=1e-3):
        P_S = self.forward_kinematics(state)
        error = self.pose_error(P_S, P_des)
        regularization = reg_weight * np.linalg.norm(state - current_state)**2
        return error + regularization

    def inverse_kinematics(self, P_des, bounds=None):
        # Bounds
        lower_state_bounds = [None, None, None, -np.pi/4, -np.pi/4, -np.pi, -0.1, -np.pi/6, -np.pi/2]
        upper_state_bounds = [None, None, None, np.pi/4, np.pi/4, np.pi, np.pi, np.pi/6, np.pi/2]
        bounds = list(zip(lower_state_bounds, upper_state_bounds))

        # State
        euler = self.quaternion_to_euler(self.vehicle_odometry.q)
        current_state = np.array([
            self.vehicle_odometry.position[0],
            self.vehicle_odometry.position[0],
            self.vehicle_odometry.position[0],
            euler[0],
            euler[1],
            euler[2],
            self.servo_state.position[0],
            self.servo_state.position[1],
            self.servo_state.position[2]
        ])

        result = minimize(
            self.ik_objective,
            current_state,
            args=(self.forward_kinematics, P_des, current_state),
            bounds=bounds,
            method='SLSQP',
            options={'ftol': 1e-6, 'disp': True}
        )
        return result.x, result.success, result.message, result.fun

def main(args=None):
    rclpy.init(args=args)
    node = PoseBasedATS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()