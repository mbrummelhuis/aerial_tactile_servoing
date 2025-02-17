import rclpy
from rclpy.node import Node
import datetime

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Int32

from geometry_msgs.msg import Vector3Stamped

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
        
        self.subscriber_input_state = self.create_subscription(
            Int32, 
            '/fmu/in/input_state', 
            self.input_state_callback, 
            10)

        self.publisher_vehicle_trajectory_setpoint = self.create_publisher(TrajectorySetpoint, '/fmu/in/vehicle_trajectory_setpoint', 10)
        self.publisher_offboard_control_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode',10)

        # Arms publishers
        self.publisher_arm_1 = self.create_publisher(Vector3Stamped, '/servo1/in/reference_position', 10)
        self.publisher_arm_2 = self.create_publisher(Vector3Stamped, '/servo2/in/reference_position', 10)

        self.declare_parameter('frequency', 10.)
        timer_period = 1.0 / self.get_parameter('frequency').get_parameter_value().double_value
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # set initial state
        self.FSM_state = 'disarmed'

        # Initialize vehicle data
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()

        # Disarmed data
        self.armed = False
        self.offboard = False

        # Takeoff data
        self.takeoff_altitude = -1.0

        # Hover data
        self.hover_start_time = None
        self.declare_parameter('hover_duration', 10.)
        self.hover_time = self.get_parameter('hover_duration').get_parameter_value().double_value

        self.declare_parameter('landing_velocity', 0.1)
        self.landing_velocity = self.get_parameter('landing_velocity').get_parameter_value().double_value
        self.landing_start_altitude = None
        self.previous_next_landing_altitude = None


        self.input_state = 0

    def timer_callback(self):
        match self.FSM_state:
            case('disarmed'):
                self.publishOffboardControlMode()
                self.publishTrajectorySetpoint(0.0, 0.0, 0.0, 0.0) # Start publishing messages to px4
                if self.armed and self.offboard:
                    self.get_logger().info('Armed and offboard -- switching to takeoff')
                    self.FSM_state = 'takeoff'
                elif self.armed and not self.offboard:
                    self.get_logger().info('Armed but not offboard -- waiting')
                elif not self.armed and self.offboard:
                    self.get_logger().info('Not armed but offboard -- waiting')
                else:
                    self.get_logger().info('Waiting for arming and offboard')

            case('takeoff'):
                # get current vehicle altitude
                current_altitude = self.vehicle_local_position.z

                # create and publish setpoint message
                self.publishOffboardControlMode()
                self.publishTrajectorySetpoint(0.0, 0.0, self.takeoff_altitude, 0.0)
                
                # check if vehicle has reached takeoff altitude
                if abs(current_altitude) > abs(self.takeoff_altitude) and not self.input_state==1:
                    self.get_logger().info('Reached takeoff altitude -- switching to hover')
                    self.FSM_state = 'start_hover'
                    self.hover_start_time = datetime.datetime.now()
                elif (self.input_state == 1):
                    self.get_logger().info(f'Input state is 1 -- switching to end hover')
                    self.FSM_state = 'end_hover'
                    self.landing_start_altitude = self.vehicle_local_position.z

            case('start_hover'):
                # create and publish setpoint message
                self.publishOffboardControlMode()
                self.publishTrajectorySetpoint(0.0, 0.0, self.takeoff_altitude, 1.0)

                if (datetime.datetime.now() - self.hover_start_time).seconds > self.hover_time and not self.input_state==1:
                    self.get_logger().info(f'Hovered for {self.hover_time} seconds -- switching to Y')
                    self.FSM_state = 'Y'
                    self.landing_start_altitude = self.vehicle_local_position.z
                elif (self.input_state == 1):
                    self.get_logger().info(f'Input state is 1 -- switching to end hover')
                    self.FSM_state = 'end_hover'
                    self.landing_start_altitude = self.vehicle_local_position.z

            case('Y'):
                # create and publish setpoint message
                self.publishOffboardControlMode()
                self.publishTrajectorySetpoint(0.0, 0.0, self.takeoff_altitude, 1.0)

                # Angles associated with Y
                self.publisher_arm_1(0.0, 0.0, 0.0)
                self.publisher_arm_2(0.0, 0.0, 0.0)

                if (datetime.datetime.now() - self.hover_start_time).seconds > self.hover_time and not self.input_state==1:
                    self.get_logger().info(f'Y for {self.hover_time} seconds -- switching to M')
                    self.FSM_state = 'M'
                    self.landing_start_altitude = self.vehicle_local_position.z
                elif (self.input_state == 1):
                    self.get_logger().info(f'Input state is 1 -- switching to end hover')
                    self.FSM_state = 'end_hover'
                    self.landing_start_altitude = self.vehicle_local_position.z

            case('M'):
                # create and publish setpoint message
                self.publishOffboardControlMode()
                self.publishTrajectorySetpoint(0.0, 0.0, self.takeoff_altitude, 1.0)

                # Angles associated with M
                self.publisher_arm_1(0.0, 0.0, 0.0)
                self.publisher_arm_2(0.0, 0.0, 0.0)

                if (datetime.datetime.now() - self.hover_start_time).seconds > self.hover_time and not self.input_state==1:
                    self.get_logger().info(f'M for {self.hover_time} seconds -- switching to C')
                    self.FSM_state = 'C'
                    self.landing_start_altitude = self.vehicle_local_position.z
                elif (self.input_state == 1):
                    self.get_logger().info(f'Input state is 1 -- switching to end hover')
                    self.FSM_state = 'end_hover'
                    self.landing_start_altitude = self.vehicle_local_position.z

            case('C'):
                # create and publish setpoint message
                self.publishOffboardControlMode()
                self.publishTrajectorySetpoint(0.0, 0.0, self.takeoff_altitude, 1.0)

                # Angles associated with C
                self.publisher_arm_1(0.0, 0.0, 0.0)
                self.publisher_arm_2(0.0, 0.0, 0.0)

                if (datetime.datetime.now() - self.hover_start_time).seconds > self.hover_time and not self.input_state==1:
                    self.get_logger().info(f'C for {self.hover_time} seconds -- switching to A')
                    self.FSM_state = 'A'
                    self.landing_start_altitude = self.vehicle_local_position.z
                elif (self.input_state == 1):
                    self.get_logger().info(f'Input state is 1 -- switching to end hover')
                    self.FSM_state = 'end_hover'
                    self.landing_start_altitude = self.vehicle_local_position.z

            case('A'):
                # create and publish setpoint message
                self.publishOffboardControlMode()
                self.publishTrajectorySetpoint(0.0, 0.0, self.takeoff_altitude, 1.0)

                # Angles associated with A
                self.publisher_arm_1(0.0, 0.0, 0.0)
                self.publisher_arm_2(0.0, 0.0, 0.0)

                if (datetime.datetime.now() - self.hover_start_time).seconds > self.hover_time and not self.input_state==1:
                    self.get_logger().info(f'A for {self.hover_time} seconds -- switching to end hover')
                    self.FSM_state = 'hover_end'
                    self.landing_start_altitude = self.vehicle_local_position.z
                
                elif (self.input_state == 1):
                    self.get_logger().info(f'Input state is 1 -- switching to end hover')
                    self.FSM_state = 'end_hover'
                    self.landing_start_altitude = self.vehicle_local_position.z
                
                elif (self.input_state == 2):
                    self.get_logger().info(f'Input state is 2 -- repeating')
                    self.FSM_state = 'Y'
                    self.landing_start_altitude = self.vehicle_local_position.z

            case('end_hover'):
                # create and publish setpoint message
                self.publishOffboardControlMode()
                self.publishTrajectorySetpoint(0.0, 0.0, self.takeoff_altitude, 1.0)

                # Back to home position
                self.publisher_arm_1(0.0, 0.0, 0.0)
                self.publisher_arm_2(0.0, 0.0, 0.0)

                if (datetime.datetime.now() - self.hover_start_time).seconds > self.hover_time:
                    self.get_logger().info(f'End hover for {self.hover_time} seconds -- switching to land')
                    self.FSM_state = 'land'
                    self.landing_start_altitude = self.vehicle_local_position.z

            case('land'):
                self.get_logger().debug('Landing')
                # Calculate next landing altitude (plus because z down positive)
                next_landing_altitude = self.vehicle_local_position.z + self.landing_velocity*self.timer_period
                # create and publish setpoint message
                self.publishOffboardControlMode()
                self.publishTrajectorySetpoint(0.0, 0.0, next_landing_altitude, 0.0)
                if abs(self.previous_next_landing_altitude - next_landing_altitude) < 0.05:
                    self.get_logger().info('Going to landed state')
                    self.FSM_state = 'landed'

                self.previous_next_landing_altitude = next_landing_altitude
            
            case('landed'):
                self.get_logger().info('Landed -- please disarm')


    def publishOffboardControlMode(self):
        msg = OffboardControlMode()
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
        self.publisher_vehicle_trajectory_setpoint.publish(msg)
    
    def publishPositionReferencesArm1(self, q1, q2, q3):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = q1
        msg.vector.y = q2
        msg.vector.z = q3
        self.publisher_arm_1.publish(msg)
    
    def publishPositionReferencesArm2(self, q1, q2, q3):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = q1
        msg.vector.y = q2
        msg.vector.z = q3
        self.publisher_arm_2.publish(msg)

    def vehicle_status_callback(self, msg):
        self.get_logger().debug(f'Received vehicle_status: {msg.system_status}')
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