import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import TwistStamped

from sensor_msgs.msg import JointState

from std_srvs.srv import SetBool

from inverse_differential_kinematics.jacobian import CentralisedJacobian, ManipulatorJacobian

class InverseDifferentialKinematics(Node):
    """
    This class implements the velocity controller for the aerial tactile servoing experiment.
    The controller is based on Closed-Loop Inverse Kinematics to go from a commanded end-effector pose to state velocity references.
    In short, this is the jacobian node.
    """
    def __init__(self):
        super().__init__('inverse_differential_kinematics')
        self.jacobian = CentralisedJacobian(mode='linear') # linear -> linear body velocities angular -> angular body velocities

        # Parameters
        self.declare_parameter('frequency', 10.)
        self.declare_parameter('mode', 'dry') # Set mode to 'dry' or 'flight' to use manipulator/centralised jacobian
        self.declare_parameter('verbose', False)
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value

        # Data
        self.state_body_angles_ = np.array([0., 0., 0.]) # YPR about z, y, x
        self.state_joint_positions_ = np.array([0., 0., 0.])
        self.state_body_angular_velocity_ = np.array([0., 0., 0.]) # YPR rates
        self.state_joint_velocity_ = np.array([0., 0., 0.])
        #self.virtual_end_effector_velocity_ = np.array([0., 0., 0., 0., 0., 0.]) # XYZ, YPR, virtual velocity driving end-effector to reference pose
        self.virtual_end_effector_velocity_inertial = np.array([0., 0., 0., 0., 0., 0.]) # XYZ, YPR, virtual velocity driving end-effector to reference pose in inertial frame
        self.forward_kinematics = np.array([0., 0., 0., 0., 0., 0.]) # XYZ, YPR, forward kinematics of the system
        # Subscribers
        # Create subscriptions related to flight
        self.get_logger().info(f"Inverse kinematics mode: {self.get_parameter('mode').get_parameter_value().string_value}")
        if self.get_parameter('mode').get_parameter_value().string_value == 'flight':
            self.get_logger().info(f'Creating body angle and velocity states')
            self.state_position_subscription = self.create_subscription(
                Vector3Stamped,
                '/state/body_angles', # Does not make sense (y and z not zero)
                self.state_body_angles_callback,
                10)
            self.state_linear_velocity_subscription = self.create_subscription(
                Vector3Stamped,
                '/state/body_velocity', # Does not exist, no publisher
                self.state_body_velocity_callback,
                10)
        # Create subscriptions related to manipulator
        self.joint_subscription = self.create_subscription(
            JointState,
            '/servo/out/state', # Publishes something, seems correct
            self.state_joint_callback,
            10)
        self.reference_ee_velocity_subscription = self.create_subscription(
            TwistStamped,
            '/references/ee_velocity', # Publishes something, seems correct
            self.reference_ee_velocity_callback,
            10)

        # Publishers -- output
        if self.get_parameter('mode').get_parameter_value().string_value == 'flight':
            self.reference_linear_velocity_publisher = self.create_publisher(
                TwistStamped,
                '/references/body_velocities', # published nothing on this topic
                10)
        self.reference_joint_velocity_publisher = self.create_publisher(
            JointState,
            '/servo/in/references/joint_references', # published nothing on this topic
            10)

        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.period = 1.0/self.frequency # seconds
        if self.get_parameter('mode').get_parameter_value().string_value == 'dry':
            self.get_logger().info("Dry version inverse kinematics")
            self.jacobian = ManipulatorJacobian()
            self.timer = self.create_timer(self.period, self.timer_callback_dry)
        elif self.get_parameter('mode').get_parameter_value().string_value == 'flight':
            self.get_logger().info("Flight version inverse kinematics")
            self.jacobian = CentralisedJacobian('linear')
            self.timer = self.create_timer(self.period, self.timer_callback_flight)
        else:
            raise ValueError('Invalid mode parameter. Set mode to "dry" or "flight')
    
    def timer_callback_dry(self):
        # Set the state
        self.state = {
            'q1': self.state_joint_positions_[0],
            'q2': self.state_joint_positions_[1],
            'q3': self.state_joint_positions_[2],
        }
        self.jacobian.set_state(self.state)
        J_pinv = self.jacobian.evaluate_pseudoinverse_jacobian()
        ref_state_velocities = J_pinv @ self.virtual_end_effector_velocity_inertial

        # Create message
        if self.verbose:
            self.get_logger().info(f"Reference velocities: {ref_state_velocities}")
        reference_joint_velocity = JointState()
        reference_joint_velocity.name = ['q1', 'q2', 'q3']
        reference_joint_velocity.position = [0.0, 0.0, 0.0]
        reference_joint_velocity.velocity = [ref_state_velocities[0], 0.0, ref_state_velocities[2]]
        reference_joint_velocity.effort = [0.0, 0.0, 0.0]

        # Get timestamp
        reference_joint_velocity.header.stamp = self.get_clock().now().to_msg()

        # Publish the references
        self.reference_joint_velocity_publisher.publish(reference_joint_velocity)

    def timer_callback_flight(self): # TODO finish implementation
        # Set the state
        self.state = {
            'yaw': self.state_body_angles_[0],
            'pitch': self.state_body_angles_[1],
            'roll': self.state_body_angles_[2],
            'q1': self.state_joint_positions_[0],
            'q2': self.state_joint_positions_[1],
            'q3': self.state_joint_positions_[2],
        }
        self.jacobian.set_state(self.state)

        # CLIK law
        J_con_pinv = self.jacobian.evaluate_pseudoinverse_controlled_jacobian()
        J_uncontrolled = self.jacobian.evaluate_uncontrolled_jacobian()
        ref_state_velocities = J_con_pinv @ self.virtual_end_effector_velocity_inertial #- \
        #    J_con_pinv @ J_uncontrolled @ self.state_body_velocity_[0:2] # XY velocities are uncontrolled
        # TODO: Add the uncontrolled velocities to the reference velocities --> Requires XY velocity estimate
        # I.e. current implementation assumes xdot and ydot to be zero.
        
        # Create messages
        reference_body_rates = TwistStamped()
        reference_body_rates.twist.linear.x = ref_state_velocities[0] # The x velocity is uncontrolled
        reference_body_rates.twist.linear.y = ref_state_velocities[1] # The y velocity is uncontrolled
        reference_body_rates.twist.linear.z = ref_state_velocities[2]  # Z velocity
        reference_body_rates.twist.angular.z = ref_state_velocities[4]
        reference_body_rates.twist.angular.y = 0.0 # Pitch rate
        reference_body_rates.twist.angular.x = 0.0 # Roll rate
        reference_joint_velocity = JointState()
        reference_joint_velocity.name = ['q1', 'q2', 'q3']
        reference_joint_velocity.position = [0.0, 0.0, 0.0]
        reference_joint_velocity.velocity = [ref_state_velocities[4], ref_state_velocities[5], ref_state_velocities[6]] # q3 velocity
        reference_joint_velocity.effort = [0.0, 0.0, 0.0]

        # Get timestamp
        timestamp = self.get_clock().now().to_msg()
        reference_body_rates.header.stamp = timestamp
        reference_joint_velocity.header.stamp = timestamp

        # Publish the references
        self.reference_linear_velocity_publisher.publish(reference_body_rates)
        self.reference_joint_velocity_publisher.publish(reference_joint_velocity)
    
    def rotate_to_world_frame(self, vector):
        """
        Rotate input vector from the end-effector frame to the world frame. A 3-vector is rotated by the rotation matrix
        and a 6-vector is rotated by the rotation matrix for the linear part and the angular part.
        """
        # Get the rotation matrix from body to world frame
                # Set the state
        self.state = {
            'yaw': self.state_body_angles_[0],
            'pitch': self.state_body_angles_[1],
            'roll': self.state_body_angles_[2],
            'q1': self.state_joint_positions_[0],
            'q2': self.state_joint_positions_[1],
            'q3': self.state_joint_positions_[2],
        }
        self.jacobian.set_state(self.state)
        R = self.jacobian.evaluate_rotation_matrix()
        if len(vector)==3:
            return R @ vector
        elif len(vector)==6:
            return np.concatenate((R @ vector[0:3], R @ vector[3:6]))
    
    # Subscriber callbacks
    def state_body_angles_callback(self, msg):
        self.state_body_angles_[0] = msg.vector.z # Yaw
        self.state_body_angles_[1] = msg.vector.y # Pitch
        self.state_body_angles_[2] = msg.vector.x # Roll
    
    def state_joint_callback(self, msg):
        self.state_joint_positions_[0] = msg.position[0] # q1
        self.state_joint_positions_[1] = msg.position[1] # q2
        self.state_joint_positions_[2] = msg.position[2] # q3

        self.state_joint_velocity_[0] = msg.velocity[0] # q1
        self.state_joint_velocity_[1] = msg.velocity[1] # q2
        self.state_joint_velocity_[2] = msg.velocity[2] # q3
    
    def state_body_velocity_callback(self, msg):
        self.state_body_angular_velocity_[0] = msg.vector.z # Yaw
        self.state_body_angular_velocity_[1] = msg.vector.y # Pitch
        self.state_body_angular_velocity_[2] = msg.vector.x # Roll
    
    ''' Receive EE velocity reference in contact frame 
    And rotate to world frame
    '''
    def reference_ee_velocity_callback(self, msg):
        virtual_end_effector_velocity_ = np.zeros(6)
        virtual_end_effector_velocity_[0] = msg.twist.linear.x # Body x
        virtual_end_effector_velocity_[1] = msg.twist.linear.y # Body y
        virtual_end_effector_velocity_[2] = msg.twist.linear.z # Body z
        virtual_end_effector_velocity_[3] = msg.twist.angular.z # Yaw
        virtual_end_effector_velocity_[4] = msg.twist.angular.y # Pitch
        virtual_end_effector_velocity_[5] = msg.twist.angular.x # Roll

        self.virtual_end_effector_velocity_inertial = self.rotate_to_world_frame(virtual_end_effector_velocity_)
    
def main(args=None):
    rclpy.init(args=args)
    node = InverseDifferentialKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()