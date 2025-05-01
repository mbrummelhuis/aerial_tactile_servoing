import rclpy
from rclpy.node import Node
import datetime

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Int32

from geometry_msgs.msg import TwistStamped

from sensor_msgs.msg import JointState

from std_srvs.srv import SetBool

from feetech_ros2.srv import SetMode

from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import OffboardControlMode


class MissionDirectorPy(Node):

    def __init__(self):
        super().__init__('mission_director_py')

        # Parameters
        self.declare_parameter('frequency', 10.)
        self.declare_parameter('takeoff_altitude', 2.0)
        self.declare_parameter('landing_velocity', -0.5)
        self.declare_parameter('hover_time', 10.0)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Position setpoint publishers
        self.publisher_vehicle_trajectory_setpoint = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.publisher_offboard_control_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode',10)

        # PX4 subscribers
        self.subscriber_vehicle_status = self.create_subscription(
            VehicleStatus, 
            '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, 
            qos_profile)
        self.subscriber_vehicle_local_position = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position', 
            self.vehicle_local_position_callback, 
            qos_profile)
        
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
        self.publisher_arm = self.create_publisher(JointState, '/servo/in/references/joint_references', 10)

        # IK activation server
        #self.ik_client = self.create_client(SetBool, 'activate_inverse_kinematics')
        #self.ik_req = SetBool.Request()
        #self.ik_active = False

        #while not self.ik_client.wait_for_service(timeout_sec=1.0): # and not self.mode_client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('services not available, waiting again...')

        # set initial state
        self.FSM_state = 'entrypoint'
        self.input_state = 0
        self.first_state_loop = True
        self.state_start_time = datetime.datetime.now()
        self.armed = False
        self.offboard = False
        self.hover_time = self.get_parameter('hover_time').get_parameter_value().double_value
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').get_parameter_value().double_value
        self.landing_velocity = self.get_parameter('landing_velocity').get_parameter_value().double_value

        # Initialize vehicle data
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()

        # Simulation stuff
        self.servo_position = True

        # Timer -- always last
        self.timer_period = 1.0 / self.get_parameter('frequency').get_parameter_value().double_value
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        match self.FSM_state:
            case('entrypoint'): # Entry point - wait for position fix
                self.x_setpoint = self.vehicle_local_position.x
                self.y_setpoint = self.vehicle_local_position.y
                self.get_logger().info("Waiting for position fix")
                if self.x_setpoint != 0.0 and self.y_setpoint != 0.0:
                    self.get_logger().info(f"Got position fix X: {self.x_setpoint} Y: {self.y_setpoint}")
                    self.FSM_state = 'disarmed'

            case('disarmed'): # Disarmed - Wait for arming and offboard
                self.publishOffboardControlMode()
                self.publishTrajectorySetpoint(0.0, 0.0, 0.0, 0.0) # Start publishing messages to px4

                if self.armed and self.offboard:
                    self.get_logger().info('Armed and offboard -- switching to takeoff')
                    self.FSM_state = 'takeoff'
                    self.state_start_time = datetime.datetime.now()
                elif self.armed and not self.offboard:
                    self.get_logger().info('Armed but not offboard -- waiting')
                elif not self.armed and self.offboard:
                    self.get_logger().info('Not armed but offboard -- waiting')
                else:
                    self.get_logger().info('Waiting for arming and offboard')

            case('takeoff'): # Takeoff - wait for takeoff altitude
                # get current vehicle altitude
                current_altitude = self.vehicle_local_position.z

                # create and publish setpoint message
                self.publishOffboardControlMode()
                # send takeoff command
                self.publishTrajectorySetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, 0.0)
                
                # check if vehicle has reached takeoff altitude
                if abs(current_altitude)+0.1 > abs(self.takeoff_altitude) and not self.input_state==1:
                    self.get_logger().info('Reached takeoff altitude -- switching to hover')
                    self.FSM_state = 'hover'
                    self.state_start_time = datetime.datetime.now()
                elif (self.input_state == 3):
                    self.get_logger().info(f'Input state is 3 -- manually transitioning to start_hover')
                    self.FSM_state = 'hover'
                    self.state_start_time = datetime.datetime.now()
            
            case('hover'):
                # create and publish setpoint message
                self.publishOffboardControlMode()
                self.publishTrajectorySetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, 0.0)

                if (datetime.datetime.now() - self.state_start_time).seconds > self.hover_time and not self.input_state==1:
                    self.get_logger().info(f'Hovered for {self.hover_time} seconds -- landing')
                    self.FSM_state = 'land'
                    self.state_start_time = datetime.datetime.now()

            case('land'):
                self.get_logger().debug('Landing')
                # Calculate next landing altitude (plus because z down positive)
                next_landing_altitude = self.vehicle_local_position.z + self.landing_velocity*self.timer_period
                # create and publish setpoint message
                self.publishOffboardControlMode()
                self.publishTrajectorySetpoint(self.x_setpoint, self.y_setpoint, next_landing_altitude, 0.0)
                if abs(self.previous_next_landing_altitude - next_landing_altitude) < 0.001:
                    self.get_logger().info('Going to landed state')
                    self.FSM_state = 'landed'

                self.previous_next_landing_altitude = next_landing_altitude
            
            case('landed'):
                self.get_logger().info('Landed -- please disarm')                



        
        # Small position PID loop for the arms to be used in velocity mode for the sim (because sim cannot easily change mode)
        if self.servo_position == True:
            # TODO Implement PID loop for the arms
            # TODO Implement velocity mode for the arms
            # TODO Implement position mode for the arms
            pass
    
    def publish_arm_position_commands(self, q1, q2, q3):
        self.get_logger().debug(f'Publishing position references: ({q1}, {q2}, {q3})')
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['q1', 'q2', 'q3']
        msg.position = [q1, q2, q3]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]

        self.publisher_arm.publish(msg)

    def publish_arm_velocity_commands(self, q1, q2, q3):
        self.get_logger().debug(f'Publishing velocity references: ({q1}, {q2}, {q3})')
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['q1', 'q2', 'q3']
        msg.position = [0.0, 0.0, 0.0]
        msg.velocity = [q1, q2, q3]
        msg.effort = [0.0, 0.0, 0.0]

        self.publisher_arm.publish(msg)
    
    def input_state_callback(self, msg):
        self.get_logger().info(f'Got input state: {msg.data}')
        self.input_state = msg.data

    def tactip_callback(self, msg):
        #self.get_logger().info(f'Tactip data: {msg}')
        self.tactip_data = msg

    def activate_ik(self):
        self.ik_req.data = True
        self.future = self.ik_client.call_async(self.ik_req)
        self.future.add_done_callback(self.activate_ik_callback)

        self.get_logger().info('Waiting for IK service call to complete')

    def activate_ik_callback(self, future):
        response = future.result()
        self.get_logger().info('Service call completed -- IK active')
        self.ik_active = True

    def deactivate_ik(self):
        self.ik_req.data = False
        self.future = self.ik_client.call_async(self.ik_req)
        self.future.add_done_callback(self.deactivate_ik_callback)

        self.get_logger().info('Waiting for IK service call to complete')

    def deactivate_ik_callback(self, future):
        response = future.result()
        self.get_logger().info('Service call completed -- IK deactivated')
        self.ik_active = False

    '''
    Request to set the servo driver mode. Mode 0 is default position mode, mode 1 is velocity mode, mode 4 is continuous position mode.
    '''
    def request_set_mode(self, mode):
        self.mode_req.operating_mode = mode
        self.future = self.mode_client.call_async(self.mode_req)
        self.future.add_done_callback(self.set_mode_callback)

        self.get_logger().info('Waiting for set mode service call to complete')

    def set_mode_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Service call completed')
        except Exception as e:
            self.get_logger().info(f'Service call failed {e}')

    # 
    def publishOffboardControlMode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.publisher_offboard_control_mode.publish(msg)
    
    def publishTrajectorySetpoint(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.position[0] = x
        msg.position[1] = y
        msg.position[2] = z
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.publisher_vehicle_trajectory_setpoint.publish(msg)

    def vehicle_status_callback(self, msg):
        self.get_logger().debug(f'Received vehicle_status')
        if (msg.arming_state == msg.ARMING_STATE_DISARMED):
            self.armed = False
        elif (msg.arming_state == msg.ARMING_STATE_ARMED):
            self.armed = True
        
        if (msg.nav_state == msg.NAVIGATION_STATE_OFFBOARD):
            self.offboard=True
        else:
            self.offboard=False

    def vehicle_local_position_callback(self, msg):
        self.get_logger().debug(f'Received vehicle_local_position: {msg.z}')
        self.vehicle_local_position = msg
    
    def input_state_callback(self, msg):
        self.get_logger().info(f'Got input state: {msg.data}')
        self.input_state = msg.data
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