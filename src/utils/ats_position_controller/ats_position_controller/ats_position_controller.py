import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3Stamped

import numpy as np

from jacobian import ManipulatorJacobian


class ATSPositionController(Node):
    def __init__(self, link_lengths=[1., 1., 1.], ik_params={}, frequency=40.0):
        # data
        self.state_position = None
        self.state_orientation = None
        self.state_joint_angles = None
        self.reference_ee_position = None

        self.joint_positions = None
        
        # manipulator characteristics
        self.link_lengths = link_lengths

        # FABRIK parameters
        self.ik_params = ik_params

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
        self.state_joint_angles_subscription = self.create_subscription(
            Vector3Stamped,
            '/state/joint_angles',
            self.state_joint_angles_callback,
            10)
        self.reference_joint_angles_subscription = self.create_subscription(
            Vector3Stamped,
            '/references/ee_position',
            self.state_ee_position_reference_callback,
            10)

        # Publishers
        self.reference_joint_positions_publisher = self.create_publisher(
            Vector3Stamped,
            '/references/joint_angles',
            10)

        # Timer
        self.frequency = frequency
        self.period = 1./self.frequency
        self.timer = self.create_timer(self.period, self.timer_callback)
        
    
    def inverse_kinematics(self, target_position):
        pass
    
    def forward_kinematics(self):
        """
        Perform forward kinematics to find positions of nodes in the kinematic chain.
        """
        pos_1 = np.array([self.link_lengths[0]*np.cos(self.state_joint_angles[0]), 
                          self.link_lengths[0]*np.sin(self.state_joint_angles[0]), 
                          0.])
        pos_2 = np.array([self.link_lengths[0]*np.cos(self.state_joint_angles[0]) + self.link_lengths[1]*np.cos(self.state_joint_angles[0])*np.cos(self.state_joint_angles[1]), 
                          self.link_lengths[0]*np.sin(self.state_joint_angles[0]) + self.link_lengths[1]*np.sin(self.state_joint_angles[0])*np.cos(self.state_joint_angles[1]), 
                          -self.link_lengths[1]*np.sin(self.state_joint_angles[1])])
        pos_3 = np.array([self.link_lengths[0]*np.cos(self.state_joint_angles[0]) + self.link_lengths[1]*np.cos(self.state_joint_angles[0])*np.cos(self.state_joint_angles[1]) - self.link_lengths[2]*np.sin(self.state_joint_angles[0])*np.sin(self.state_joint_angles[3]) + self.link_lengths[2]*np.cos(self.state_joint_angles[0])*np.cos(self.state_joint_angles[1])*np.cos(self.state_joint_angles[3]), 
                          self.link_lengths[0]*np.sin(self.state_joint_angles[0]) + self.link_lengths[1]*np.sin(self.state_joint_angles[0])*np.cos(self.state_joint_angles[1]) + self.link_lengths[2]*np.sin(self.state_joint_angles[0])*np.cos(self.state_joint_angles[1])*np.cos(self.state_joint_angles[3]) + self.link_lengths[2]*np.sin(self.state_joint_angles[3])*np.cos(self.state_joint_angles[0]), 
                          -self.link_lengths[1]*np.sin(self.state_joint_angles[1]) - self.link_lengths[2]*np.sin(self.state_joint_angles[1])*np.cos(self.state_joint_angles[3])])

        self.joint_positions = np.array((4,3))
        self.joint_positions[0] = np.array([0., 0., 0.])
        self.joint_positions[1] = pos_1
        self.joint_positions[2] = pos_2
        self.joint_positions[3] = pos_3

    def timer_callback(self):
        # Do inverse kinematics
        reference_joint_positions = self.solve_FABRIK(self.state_joint_positions, self.reference_ee_position)

        # Construct and publish message
        reference_joint_positions = Vector3Stamped()
        reference_joint_positions.vector = reference_joint_positions
        reference_joint_positions.header.stamp = self.get_clock().now().to_msg()
        # Frame id assignment?
        self.reference_joint_positions_publisher.publish(reference_joint_positions)
    
    # Subscriber callbacks
    def state_position_callback(self, msg):
        self.state_position = msg.data.vector

    def state_orientation_callback(self, msg):
        self.state_orientation = msg.data.vector

    def state_joint_angles_callback(self, msg):
        self.state_joint_angles = msg.data.vector

    def state_ee_position_reference_callback(self, msg):
        self.reference_ee_position = msg.data.vector

    # Setters
    def set_tolerance(self, tolerance):
        self.tolerance = tolerance
    
    def set_max_iterations(self, max_iterations):
        self.max_iterations = max_iterations
    
    def set_link_lengths(self, link_lengths):
        self.link_lengths = link_lengths

def main(args=None):
    # Settings
    link_lengths = [0.11, 0.311, 0.273]
    ik_params = {"max_iterations": 100,
                "tolerance": 0.01}
    frequency = 20.0

    # Node
    rclpy.init(args=args)
    node = ATSPositionController(link_lengths=link_lengths, 
                                ik_params=ik_params,
                                 frequency=frequency)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()