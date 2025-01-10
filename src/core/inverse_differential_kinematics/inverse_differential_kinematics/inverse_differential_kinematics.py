import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Vector3Stamped

from jacobian import CentralisedJacobian

class InverseDifferentialKinematics(Node):
    """
    This class implements the velocity controller for the aerial tactile servoing experiment.
    The controller is based on Closed-Loop Inverse Kinematics to go from a commanded end-effector pose to state velocity references.
    """
    def __init__(self):
        super().__init__('inverse_differential_kinematics')
        self.jacobian = CentralisedJacobian()
        self.K = np.eye(6)

        self.desired_ee_velocities = np.zeros(6)

        # Data
        self.state_position = np.array([0., 0., 0.])
        self.state_orientation = np.array([0., 0., 0.])
        self.state_joint_positions = np.array([0., 0., 0.])
        self.state_linear_velocity = np.array([0., 0., 0.])
        self.state_angular_velocity = np.array([0., 0., 0.])
        self.state_joint_velocity = np.array([0., 0., 0.])
        self.reference_ee_linear_velocity = np.array([0., 0., 0.])
        self.reference_ee_angular_velocity = np.array([0., 0., 0.])

        # Subscribers
        self.state_position_subscription = self.create_subscription(
            Vector3Stamped,
            '/state/position',
            self.state_position_callback,
            10)
        self.state_orientation_subscription = self.create_subscription(
            Vector3Stamped,
            '/state/orientation',
            self.state_orientation_callback,
            10)
        self.state_joint_positions_subscription = self.create_subscription(
            Vector3Stamped,
            '/state/joint_positions',
            self.state_joint_position_callback,
            10)
        self.state_linear_velocity_subscription = self.create_subscription(
            Vector3Stamped,
            '/state/linear_velocity',
            self.state_linear_velocity_callback,
            10)
        self.state_angular_velocity_subscription = self.create_subscription(
            Vector3Stamped,
            '/state/angular_velocity',
            self.state_angular_velocity_callback,
            10)
        self.state_joint_velocity_subscription = self.create_subscription(
            Vector3Stamped,
            '/state/joint_velocities',
            self.state_joint_velocity_callback,
            10)
        self.reference_ee_linear_velocity_subscription = self.create_subscription(
            Vector3Stamped,
            '/references/ee_linear_velocity',
            self.reference_ee_linear_velocity_callback,
            10)
        self.reference_ee_angular_velocity_subscription = self.create_subscription(
            Vector3Stamped,
            '/references/ee_angular_velocity',
            self.reference_ee_angular_velocity_callback,
            10)

        # Publishers
        self.reference_linear_velocity_publisher = self.create_publisher(
            Vector3Stamped,
            '/references/linear_velocity',
            10)
        self.reference_angular_velocity_publisher = self.create_publisher(
            Vector3Stamped,
            '/references/angular_velocity',
            10)
        self.reference_joint_velocity_publisher = self.create_publisher(
            Vector3Stamped,
            '/references/joint_velocities',
            10)

        self.frequency = 40.0 # Hz
        self.period = 1/self.frequency
        self.timer = self.create_timer(self.period, self.timer_callback)
     
    def timer_callback(self):
        # CLIK law
        J_con_pinv = self.jacobian.evaluate_pseudoinverse_controlled_jacobian()
        J_uncontrolled = self.jacobian.evaluate_uncontrolled_jacobian()
        kinematic_error = self.ee_pose_reference - self.ee_pose_actual
        ref_state_velocities = J_con_pinv*(self.desired_ee_velocities + self.K*kinematic_error) - \
            J_con_pinv*J_uncontrolled*self.state_velocities[1:2]
        
        # Create messages
        reference_linear_velocity = Vector3Stamped()
        reference_linear_velocity.vector.x = ref_state_velocities[0]
        reference_linear_velocity.vector.y = ref_state_velocities[1]
        reference_linear_velocity.vector.z = ref_state_velocities[2]
        reference_angular_velocity = Vector3Stamped()
        reference_angular_velocity.vector.x = ref_state_velocities[3]
        reference_angular_velocity.vector.y = ref_state_velocities[4]
        reference_angular_velocity.vector.z = ref_state_velocities[5]
        reference_joint_velocity = Vector3Stamped()
        reference_joint_velocity.vector.x = ref_state_velocities[6]
        reference_joint_velocity.vector.y = ref_state_velocities[7]
        reference_joint_velocity.vector.z = ref_state_velocities[8]

        # Get timestamp
        timestamp = self.get_clock().now().to_msg()
        reference_linear_velocity.header.stamp = timestamp
        reference_angular_velocity.header.stamp = timestamp
        reference_joint_velocity.header.stamp = timestamp

        # Publish the references
        self.reference_linear_velocity_publisher.publish(reference_linear_velocity)
        self.reference_angular_velocity_publisher.publish(reference_angular_velocity)
        self.reference_joint_velocity_publisher.publish(reference_joint_velocity)
    
    # Subscriber callbacks
    def state_position_callback(self, msg):
        self.state_position[0] = msg.vector.x
        self.state_position[1] = msg.vector.y
        self.state_position[2] = msg.vector.z
    
    def state_orientation_callback(self, msg):
        self.state_orientation[0] = msg.vector.x
        self.state_orientation[1] = msg.vector.y
        self.state_orientation[2] = msg.vector.z
    
    def state_joint_position_callback(self, msg):
        self.state_joint_positions[0] = msg.vector.x
        self.state_joint_positions[1] = msg.vector.y
        self.state_joint_positions[2] = msg.vector.z
    
    def state_linear_velocity_callback(self, msg):
        self.state_linear_velocity[0] = msg.vector.x
        self.state_linear_velocity[1] = msg.vector.y
        self.state_linear_velocity[2] = msg.vector.z
    
    def state_angular_velocity_callback(self, msg):
        self.state_angular_velocity[0] = msg.vector.x
        self.state_angular_velocity[1] = msg.vector.y
        self.state_angular_velocity[2] = msg.vector.z

    def state_joint_velocity_callback(self, msg):
        self.state_joint_velocity[0] = msg.vector.x
        self.state_joint_velocity[1] = msg.vector.y
        self.state_joint_velocity[2] = msg.vector.z
    
    def reference_ee_linear_velocity_callback(self, msg):
        self.reference_ee_linear_velocity[0] = msg.vector.x
        self.reference_ee_linear_velocity[1] = msg.vector.y
        self.reference_ee_linear_velocity[2] = msg.vector.z

    def reference_ee_angular_velocity_callback(self, msg):
        self.reference_ee_angular_velocity[0] = msg.vector.x
        self.reference_ee_angular_velocity[1] = msg.vector.y
        self.reference_ee_angular_velocity[2] = msg.vector.z
    


def main(args=None):
    rclpy.init(args=args)
    node = InverseDifferentialKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()