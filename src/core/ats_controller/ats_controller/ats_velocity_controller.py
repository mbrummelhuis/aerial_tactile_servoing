import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Vector3Stamped

from jacobian import CentralisedJacobian

class ATSVelocityController(Node):
    def __init__(self):
        self.jacobian = CentralisedJacobian()
        self.K = np.eye(6)

        self.desired_ee_velocities = np.zeros(6)

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
        self.state_velocity_subscription = self.create_subscription(
            Vector3Stamped,
            '/state/velocity',
            self.state_velocity_callback,
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
        self.ee_pose_referece = np.zeros(6)

        # Publishers
        self.state_velocity_reference_publisher = self.create_publisher(
            TwistStamped,
            '/state/q1_ref',
            10)

        self.frequency = 40.0 # Hz
        self.period = 1/self.frequency
        self.timer = self.create_timer(self.period, self.timer_callback)

        # TO DO HERE
        # - Implement state subscribers that save the latest state on the node
        # - implement timer that evaluates the entire controller, outputting the state velocity references according to the control law by Arleo and Cataldi
        
    def timer_callback(self):
        J_con_pinv = self.jacobian.evaluate_pseudoinverse_controlled_jacobian()
        J_uncontrolled = self.jacobian.evaluate_uncontrolled_jacobian()
        kinematic_error = self.ee_pose_referece - self.ee_pose_actual
        self.ref_state_velocities = J_con_pinv*(self.desired_ee_velocities + self.K*kinematic_error) - \
            J_con_pinv*J_uncontrolled*self.state_velocities[1:2]
        # publish ref state velocities
        # TO do: build the message for the publisher
        # To do: implement a saturation function for the ref state velocities
        self.state_velocity_reference_publisher.publish(self.ref_state_velocities)


def main(args=None):
    rclpy.init(args=args)
    node = ATSVelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()