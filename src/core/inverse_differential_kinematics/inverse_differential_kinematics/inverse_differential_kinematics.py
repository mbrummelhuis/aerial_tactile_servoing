import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import TwistStamped

from jacobian import CentralisedJacobian

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
        self.declare_parameter('frequency', 10)

        # Data
        self.state_body_pose_ = np.array([0., 0., 0., 0., 0., 0.]) # XYZ, YPR
        self.state_joint_positions_ = np.array([0., 0., 0.])
        self.state_body_velocity_ = np.array([0., 0., 0., 0., 0., 0.])
        self.state_joint_velocity_ = np.array([0., 0., 0.])
        self.virtual_end_effector_velocity_ = np.array([0., 0., 0., 0., 0., 0.]) # XYZ, YPR, virtual velocity driving end-effector to reference pose

        # Subscribers
        self.state_position_subscription = self.create_subscription(
            TwistStamped,
            '/state/body_pose',
            self.state_body_pose_callback,
            10)
        self.state_joint_positions_subscription = self.create_subscription(
            Vector3Stamped,
            '/state/joint_positions',
            self.state_joint_position_callback,
            10)
        self.state_linear_velocity_subscription = self.create_subscription(
            TwistStamped,
            '/state/body_velocity',
            self.state_body_velocity_callback,
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
        self.reference_linear_velocity_publisher = self.create_publisher(
            TwistStamped,
            '/references/body_velocities',
            10)
        self.reference_joint_velocity_publisher = self.create_publisher(
            Vector3Stamped,
            '/references/joint_velocities',
            10)

        self.period = 1.0/float(self.get_parameter('frequency').get_parameter_value().integer_value) # seconds
        self.timer = self.create_timer(self.period, self.timer_callback)
     
    def timer_callback(self):
        # Set the state
        self.state = {
            'yaw': self.state_body_pose_[3],
            'pitch': self.state_body_pose_[4],
            'roll': self.state_body_pose_[5],
            'q1': self.state_joint_positions_[0],
            'q2': self.state_joint_positions_[1],
            'q3': self.state_joint_positions_[2],
        }
        self.jacobian.set_state(self.state)

        # CLIK law
        J_con_pinv = self.jacobian.evaluate_pseudoinverse_controlled_jacobian()
        J_uncontrolled = self.jacobian.evaluate_uncontrolled_jacobian()
        ref_state_velocities = J_con_pinv * self.virtual_end_effector_velocity_ - \
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
    
    # Subscriber callbacks
    def state_body_pose_callback(self, msg):
        self.state_body_pose_[0] = msg.twist.linear.x # Body x
        self.state_body_pose_[1] = msg.twist.linear.y # Body y
        self.state_body_pose_[2] = msg.twist.linear.z # Body z
        self.state_body_pose_[3] = msg.twist.angular.z # Yaw
        self.state_body_pose_[4] = msg.twist.angular.y # Pitch
        self.state_body_pose_[5] = msg.twist.angular.x # Roll
    
    def state_joint_position_callback(self, msg):
        self.state_joint_positions_[0] = msg.vector.x # q1
        self.state_joint_positions_[1] = msg.vector.y # q2
        self.state_joint_positions_[2] = msg.vector.z # q3
    
    def state_body_velocity_callback(self, msg):
        self.state_body_velocity_[0] = msg.twist.linear.x # Body x
        self.state_body_velocity_[1] = msg.twist.linear.y # Body y
        self.state_body_velocity_[2] = msg.twist.linear.z # Body z
        self.state_body_velocity_[3] = msg.twist.angular.z # Yaw
        self.state_body_velocity_[4] = msg.twist.angular.y # Pitch
        self.state_body_velocity_[5] = msg.twist.angular.x # Roll

    def state_joint_velocity_callback(self, msg):
        self.state_joint_velocity_[0] = msg.vector.x # q1
        self.state_joint_velocity_[1] = msg.vector.y # q2
        self.state_joint_velocity_[2] = msg.vector.z # q3
    
    def reference_ee_velocity_callback(self, msg):
        self.virtual_end_effector_velocity_[0] = msg.twist.linear.x # Body x
        self.virtual_end_effector_velocity_[1] = msg.twist.linear.y # Body y
        self.virtual_end_effector_velocity_[2] = msg.twist.linear.z # Body z
        self.virtual_end_effector_velocity_[3] = msg.twist.angular.z # Yaw
        self.virtual_end_effector_velocity_[4] = msg.twist.angular.y # Pitch
        self.virtual_end_effector_velocity_[5] = msg.twist.angular.x # Roll
    
def main(args=None):
    rclpy.init(args=args)
    node = InverseDifferentialKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()