import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import TwistStamped

from sensor_msgs.msg import JointState

from std_srvs.srv import SetBool

from jacobian import CentralisedJacobian, ManipulatorJacobian

class InverseDifferentialKinematics(Node):
    """
    This class implements the velocity controller for the aerial tactile servoing experiment.
    The controller is based on Closed-Loop Inverse Kinematics to go from a commanded end-effector pose to state velocity references.
    In short, this is the jacobian node.
    """
    def __init__(self):
        super().__init__('inverse_differential_kinematics')
        self.jacobian = CentralisedJacobian()

        # Parameters
        self.declare_parameter('frequency', 10.)
        self.declare_parameter('mode', 'dry') # Set mode to 'dry' or 'flight' to use manipulator/centralised jacobian

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
        if self.get_parameter('mode').get_parameter_value().string_value == 'flight':
            self.state_position_subscription = self.create_subscription(
                Vector3Stamped,
                '/state/body_angles',
                self.state_body_angles_callback,
                10)
            self.state_linear_velocity_subscription = self.create_subscription(
                Vector3Stamped,
                '/state/body_velocity',
                self.state_body_velocity_callback,
                10)
        # Create subscriptions related to manipulator
        self.state_joint_positions_subscription = self.create_subscription(
            Vector3Stamped,
            '/state/joint_positions',
            self.state_joint_position_callback,
            10)

        self.state_joint_velocity_subscription = self.create_subscription(
            Vector3Stamped,
            '/state/joint_velocities',
            self.state_joint_velocity_callback,
            10)
        self.reference_ee_velocity_subscription = self.create_subscription(
            TwistStamped,
            '/references/ee_velocity',
            self.reference_ee_velocity_callback,
            10)

        # Publishers
        if self.get_parameter('mode').get_parameter_value().string_value == 'flight':
            self.reference_linear_velocity_publisher = self.create_publisher(
                TwistStamped,
                '/references/body_velocities',
                10)
        self.reference_joint_velocity_publisher = self.create_publisher(
            Vector3Stamped,
            '/references/joint_velocities',
            10)

        self.period = 1.0/float(self.get_parameter('frequency').get_parameter_value().integer_value) # seconds
        if self.get_parameter('mode').get_parameter_value().string_value == 'dry':
            self.jacobian = ManipulatorJacobian()
            self.timer = self.create_timer(self.period, self.timer_callback_dry)
        elif self.get_parameter('mode').get_parameter_value().string_value == 'flight':
            self.jacobian = CentralisedJacobian()
            self.timer = self.create_timer(self.period, self.timer_callback_flight)
        else:
            raise ValueError('Invalid mode parameter. Set mode to "dry" or "flight')
        
        # Activation mechanism
        self.activation_srv = self.create_service(SetBool, 'activate', self.activate_callback)
        self.is_active = False
    
    def timer_callback_dry(self):
        if self.is_active:
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
            reference_joint_velocity = JointState()
            reference_joint_velocity.name = ['q1', 'q2', 'q3']
            reference_joint_velocity.position = [0.0, 0.0, 0.0]
            reference_joint_velocity.velocity = [ref_state_velocities[0], ref_state_velocities[1], ref_state_velocities[2]]
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
        ref_state_velocities = J_con_pinv * self.virtual_end_effector_velocity_inertial - \
            J_con_pinv*J_uncontrolled*self.state_body_velocity_[0:2] # XY velocities are uncontrolled
        
        # Create messages
        reference_body_rates = TwistStamped()
        reference_body_rates.twist.linear.x = 0.0 # The x velocity is uncontrolled
        reference_body_rates.twist.linear.y = 0.0 # The y velocity is uncontrolled
        reference_body_rates.twist.linear.z = ref_state_velocities[0]  # Z velocity
        reference_body_rates.twist.angular.z = ref_state_velocities[1] # Yaw rate
        reference_body_rates.twist.angular.y = ref_state_velocities[2] # Pitch rate
        reference_body_rates.twist.angular.x = ref_state_velocities[3] # Roll rate
        reference_joint_velocity = Vector3Stamped()
        reference_joint_velocity.vector.x = ref_state_velocities[6]
        reference_joint_velocity.vector.y = ref_state_velocities[7]
        reference_joint_velocity.vector.z = ref_state_velocities[8]

        # Get timestamp
        timestamp = self.get_clock().now().to_msg()
        reference_body_rates.header.stamp = timestamp
        reference_joint_velocity.header.stamp = timestamp

        # Publish the references
        self.reference_linear_velocity_publisher.publish(reference_body_rates)
        self.reference_joint_velocity_publisher.publish(reference_joint_velocity)
        
    def activate_callback(self, request, response):
        self.active = request.data
        response.success = True
        response.message = f"Publisher {'activated' if self.active else 'deactivated'}"
        self.get_logger().info(response.message)
        return response
    
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
    
    def state_joint_position_callback(self, msg):
        self.state_joint_positions_[0] = msg.vector.x # q1
        self.state_joint_positions_[1] = msg.vector.y # q2
        self.state_joint_positions_[2] = msg.vector.z # q3
    
    def state_body_velocity_callback(self, msg):
        self.state_body_angular_velocity_[0] = msg.twist.angular.z # Yaw
        self.state_body_angular_velocity_[1] = msg.twist.angular.y # Pitch
        self.state_body_angular_velocity_[2] = msg.twist.angular.x # Roll

    def state_joint_velocity_callback(self, msg):
        self.state_joint_velocity_[0] = msg.vector.x # q1
        self.state_joint_velocity_[1] = msg.vector.y # q2
        self.state_joint_velocity_[2] = msg.vector.z # q3
    
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