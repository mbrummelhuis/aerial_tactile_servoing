import rclpy
from rclpy.node import Node
import datetime

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Int32

from geometry_msgs.msg import TwistStamped

from sensor_msgs.msg import JointState

from std_srvs.srv import SetBool

from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import OffboardControlMode


class MissionDirectorPy(Node):

    def __init__(self):
        super().__init__('mission_director_py')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to manual input
        self.subscriber_input_state = self.create_subscription(
            Int32, 
            '/md/input', 
            self.input_state_callback, 
            10)
        self.subscriber_tactip = self.create_subscription(
            TwistStamped,
            '/sensors/tactip',
            self.tactip_callback,
            10)
        # Initialize tactip data to zero
        self.tactip_data = TwistStamped()
        self.tactip_data.twist.linear.x = 0.0
        self.tactip_data.twist.linear.y = 0.0
        self.tactip_data.twist.linear.z = 0.0
        self.tactip_data.twist.angular.x = 0.0
        self.tactip_data.twist.angular.y = 0.0
        self.tactip_data.twist.angular.z = 0.0
        

        # Arms publishers
        self.publisher_arm = self.create_publisher(JointState, '/servo/in/reference_position', 10)
        self.declare_parameter('initial_joint_states', {0.0, 0.0, 0.0})

        # State transition times
        self.declare_parameter('entrypoint_time', 5.0)
        self.declare_parameter('position_arm_time', 5.0)
        self.declare_parameter('tactile_servoing_time', 5.0)

        # Timer
        self.declare_parameter('frequency', 10.)
        self.timer_period = 1.0 / self.get_parameter('frequency').get_parameter_value().double_value
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Activation server
        self.cli = self.create_client(SetBool, 'activate_inverse_kinematics')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

        # set initial state
        self.FSM_state = 'entrypoint'
        self.input_state = 0
        self.first_state_loop = True
        self.state_start_time = datetime.datetime.now()

    def timer_callback(self):
        match self.FSM_state:
            case('entrypoint'):
                if self.first_state_loop:
                    self.get_logger().info('Starting FSM')
                    self.first_state_loop = False

                # TODO How to know when servo driver is ready? Now we just wait
                # Transition
                if datetime.datetime.now() - self.state_start_time > datetime.timedelta(seconds=self.get_parameter('entrypoint_time').get_parameter_value().double_value):
                    self.get_logger().info(f"Waited for 5 seconds -- switching to disarmed")
                    self.FSM_state = 'position_arm'
                    self.state_start_time = datetime.datetime.now() # Reset state start time
                    self.first_state_loop = True # Reset first state loop flag

            case('position_arm'):
                if self.first_state_loop:
                    self.get_logger().info('Positioning arm')
                    self.first_state_loop = False
                    self.publish_arm_commands(0.0, 0.0, 0.0)

                # Transition
                if datetime.datetime.now() - self.state_start_time > datetime.timedelta(seconds=self.get_parameter('position_arm_time').get_parameter_value().double_value):
                    self.get_logger().info(f"Arm positioned -- switching to wait_for_contact")
                    self.FSM_state = 'wait_for_contact'
                    self.state_start_time = datetime.datetime.now() # Reset state start time
                    self.first_state_loop = True # Reset first state loop flag

            case('wait_for_contact'):
                if self.first_state_loop:
                    self.get_logger().info('Waiting for contact')
                    self.first_state_loop = False
                
                # Transition
                # If contact depth is greater than 1.0 mm, we assume contact.
                if (self.tactip_data.twist.linear.z)*1000<-1.0 and self.activate_ik() is not None:
                    self.get_logger().info('Contact detected and IK activated -- switching to tactile servoing')
                    self.FSM_state = 'tactile_servoing'
                    self.state_start_time = datetime.datetime.now()
                    self.first_state_loop = True # Reset first state loop flag

            case('tactile_servoing'):
                if self.first_state_loop:
                    self.get_logger().info('Tactile servoing')
                    self.first_state_loop = False

                # Transition    
                if datetime.datetime.now() - self.state_start_time > datetime.timedelta(seconds=self.get_parameter('tactile_servoing_time').get_parameter_value().double_value) and self.deactivate_ik() is not None:
                    self.get_logger().info(f"Waited for 5 seconds and IK deactivated -- switching to end")
                    self.FSM_state = 'end'
                    self.state_start_time = datetime.datetime.now()
                    self.first_state_loop = True # Reset first state loop flag
            
            case('end'):
                if self.first_state_loop:
                    self.get_logger().info('End')
                    self.first_state_loop = False
                    self.publish_arm_commands(0.0, 0.0, 0.0) # Arm to home position
    
    def publish_arm_commands(self, q1, q2, q3):
        self.get_logger().debug(f'Publishing position references: ({q1}, {q2}, {q3})')
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['q1', 'q2', 'q3']
        msg.position = [q1, q2, q3]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]

        self.publisher_arm.publish(msg)
    
    def input_state_callback(self, msg):
        self.get_logger().info(f'Got input state: {msg.data}')
        self.input_state = msg.data

    def tactip_callback(self, msg):
        self.get_logger().info(f'Tactip data: {msg}')
        self.tactip_data = msg

    def activate_ik(self):
        self.req.data = True
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info('Service call success')
        else:
            self.get_logger().info('Service call failed')
        return self.future.result()
        
    def deactivate_ik(self):
        self.req.data = False
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info('Service call success')
        else:
            self.get_logger().info('Service call failed')
        return self.future.result()

def main():
    rclpy.init(args=None)

    mission_director_py = MissionDirectorPy()

    rclpy.spin(mission_director_py)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mission_director_py.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()