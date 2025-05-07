import rclpy
from rclpy.node import Node
import datetime

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Int32
from std_msgs.msg import Float64

from geometry_msgs.msg import TwistStamped

from sensor_msgs.msg import JointState

from std_srvs.srv import SetBool

from feetech_ros2.srv import SetMode

from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleCommand


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
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

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
        
        # GZ subscribers
        self.subscriber_joint_states = self.create_subscription(
            JointState,
            '/servo/out/state',
            self.joint_states_callback,
            10)
        
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
        self.subscriber_servo_velocity = self.create_subscription(
            JointState,
            '/servo/in/references/joint_references',
            self.servo_velocity_callback,
            10)
        self.subscriber_body_velocity_callback = self.create_subscription(
            TwistStamped,
            '/references/body_velocities',
            self.body_velocity_callback,
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
        self.publisher_pivot_vel = self.create_publisher(Float64, '/shoulder_1_vel_cmd', 10)
        self.publisher_shoulder_vel = self.create_publisher(Float64, '/elbow_1_vel_cmd', 10)
        self.publisher_elbow_vel = self.create_publisher(Float64, '/forearm_1_vel_cmd', 10)

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
        self.tactile_servoing = False
        self.hover_time = self.get_parameter('hover_time').get_parameter_value().double_value
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').get_parameter_value().double_value
        self.landing_velocity = self.get_parameter('landing_velocity').get_parameter_value().double_value
        self.previous_next_landing_altitude = abs(self.takeoff_altitude)+0.1
        self.kp = 2.
        self.kd = 0.3

        self.arm_positions = [0.0, 0.0, 0.0]
        self.arm_velocities = [0.0, 0.0, 0.0]

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
                if self.first_state_loop:
                    self.get_logger().info("Waiting for position fix")
                    self.first_state_loop = False

                self.x_setpoint = self.vehicle_local_position.x
                self.y_setpoint = self.vehicle_local_position.y
                self.publishOffboardPositionMode()
                if self.x_setpoint != 0.0 and self.y_setpoint != 0.0:
                    self.state_start_time = datetime.datetime.now()
                    self.first_state_loop = False
                    self.get_logger().info(f"Got position fix X: {self.x_setpoint} Y: {self.y_setpoint}")
                    self.FSM_state = 'move arm landed'
            case('move arm landed'):
                done = self.move_arm_to_position(1.578, 0.0, 2.10)
                if done:
                    done = False
                    self.get_logger().info("Done moving arm into takeoff position")
                    self.state_start_time = datetime.datetime.now()
                    self.FSM_state = 'wait'

            case('wait'):
                if self.first_state_loop:
                    self.get_logger().info(f"Waiting")
                    self.first_state_loop = False

                if (datetime.datetime.now() - self.state_start_time).seconds > 1:
                    self.first_state_loop = True
                    self.state_start_time = datetime.datetime.now()
                    self.FSM_state = 'disarmed'

            case('disarmed'): # Disarmed - Wait for arming and offboard
                self.engage_offboard_mode()
                self.armVehicle()
                self.get_logger().info("Arming and going to offboard")
                self.publishOffboardPositionMode()
                if self.armed and self.offboard:
                    self.get_logger().info("Taking off")
                    self.FSM_state = 'takeoff'
            
            case('takeoff'): # Takeoff - wait for takeoff altitude
                # get current vehicle altitude
                current_altitude = self.vehicle_local_position.z

                # create and publish setpoint message
                self.publishOffboardPositionMode()
                # send takeoff command
                self.publishTrajectoryPositionSetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, self.vehicle_local_position.heading)
                
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
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, self.vehicle_local_position.heading)

                if (datetime.datetime.now() - self.state_start_time).seconds > self.hover_time and not self.input_state==1:
                    self.get_logger().info(f'Hovered for {self.hover_time} seconds -- moving arm')
                    self.FSM_state = 'move arm hover'
                    self.state_start_time = datetime.datetime.now()

            case('move arm hover'):
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, self.vehicle_local_position.heading)
                done = self.move_arm_to_position(1.0, 0.0, -0.578)
                if done:
                    self.get_logger().info("Finished moving arm into tactile servoing position")
                    self.state_start_time = datetime.datetime.now()
                    self.FSM_state = 'wait_for_contact'

            case('wait_for_contact'):
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, self.vehicle_local_position.heading)
                if self.first_state_loop:
                    self.get_logger().info('Waiting for contact')
                    self.first_state_loop = False

                self.get_logger().info(f'Tactip depth: {self.tactip_data.twist.linear.z}')
                #self.get_logger().info(f'Tactip conditions: {self.tactip.data.twist.linear.z*1000 < -2.0}')
                # Transition
                # If contact depth is greater than 1.0 mm, we assume contact.
                if (self.tactip_data.twist.linear.z) < -2.0 or self.input_state == 1:
                    self.input_state = 0
                    self.get_logger().info(f"Contact detected -- activating IK")
                    self.tactile_servoing = True
                    self.FSM_state = 'tactile_servoing'
                    self.state_start_time = datetime.datetime.now()
                    self.first_state_loop = True # Reset first state loop flag

            case('tactile_servoing'):
                self.tactile_servoing = True
                self.publishOffboardVelocityMode()
                if self.first_state_loop:
                    self.get_logger().info('Tactile servoing')
                    self.first_state_loop = False

                # Transition    
                if datetime.datetime.now() - self.state_start_time > datetime.timedelta(seconds=600.):
                    self.tactile_servoing = False
                    self.get_logger().info(f"Waited for 30 seconds and IK deactivated -- switching to end")
                    self.FSM_state = 'land'
                    self.state_start_time = datetime.datetime.now()
                    self.first_state_loop = True # Reset first state loop flag

            case('land'):
                self.get_logger().debug('Landing')
                # Calculate next landing altitude (plus because z down positive)
                next_landing_altitude = self.vehicle_local_position.z + self.landing_velocity*self.timer_period
                # create and publish setpoint message
                self.publishOffboardPositionMode()
                self.land()
                if abs(self.previous_next_landing_altitude - next_landing_altitude) < 0.001:
                    self.get_logger().info('Landed')
                    self.FSM_state = 'landed'

                self.previous_next_landing_altitude = next_landing_altitude
            
            case('landed'):
                self.get_logger().info('Done')
                pass               

    
    def publish_arm_position_commands(self, q1, q2, q3):
        pass

    def publish_arm_velocity_commands(self, q1, q2, q3):
        msg = Float64()
        msg.data = q1
        self.publisher_pivot_vel.publish(msg)
        msg.data = q2
        self.publisher_shoulder_vel.publish(msg)
        msg.data = q3
        self.publisher_elbow_vel.publish(msg)
    
    def input_state_callback(self, msg):
        self.get_logger().info(f'Got input state: {msg.data}')
        self.input_state = msg.data

    def tactip_callback(self, msg):
        #self.get_logger().info(f'Tactip data: {msg}')
        self.tactip_data = msg

    def move_arm_to_position(self, pos1, pos2, pos3):
        error1 = pos1-self.arm_positions[0]
        error2 = pos2-self.arm_positions[1]
        error3 = pos3-self.arm_positions[2]
        vel1 = self.kp*(error1) - self.kd*self.arm_velocities[0]
        vel2 = self.kp*(error2) - self.kd*self.arm_velocities[1]
        vel3 = self.kp*(error3) - self.kd*self.arm_velocities[2]
        self.publish_arm_velocity_commands(vel1, vel2, vel3)

        if(abs(error1) < 0.001 and abs(error2) < 0.001 and abs(error3) < 0.001):
            self.publish_arm_velocity_commands(0.0, 0.0, 0.0)
            self.get_logger().info(f"Done moving arm to {pos1}, {pos2}, {pos3}")
            return True
        else:
            return False

    def publishOffboardPositionMode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.publisher_offboard_control_mode.publish(msg)
    def publishOffboardVelocityMode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.publisher_offboard_control_mode.publish(msg)

    def publishTrajectoryPositionSetpoint(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.position[0] = x
        msg.position[1] = y
        msg.position[2] = z
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.publisher_vehicle_trajectory_setpoint.publish(msg)
    def publishTrajectoryVelocitySetpoint(self, vx, vy, vz, yawspeed):
        msg = TrajectorySetpoint()
        msg.velocity[0] = vx
        msg.velocity[1] = vy
        msg.velocity[2] = vz
        msg.yawspeed = yawspeed
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.publisher_vehicle_trajectory_setpoint.publish(msg)

    def armVehicle(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')
        self.armed = True

    def disarmVehicle(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')
        self.armed = False

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")
        self.offboard = True

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher_vehicle_command.publish(msg)

    # Callbacks
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

    def joint_states_callback(self, msg):
        self.arm_positions[0] = msg.position[0] # Pivot
        self.arm_positions[1] = msg.position[1] # Shoulder
        self.arm_positions[2] = msg.position[2] # Elbow

        self.arm_velocities[0] = msg.velocity[0] # Pivot
        self.arm_velocities[1] = msg.velocity[1] # Shoulder
        self.arm_velocities[2] = msg.velocity[2] # Elbow
    
    def servo_velocity_callback(self, msg):
        # Forward the servo velocity message to the proper topic
        if self.tactile_servoing:
            self.get_logger().info(f'Got servo velocity: {msg.velocity[0]}, {msg.velocity[1]}, {msg.velocity[2]}')
            self.publish_arm_velocity_commands(msg.velocity[0], msg.velocity[1], msg.velocity[2])

    def body_velocity_callback(self, msg):
        if self.tactile_servoing:
            self.get_logger().info(f'Got body velocity: {msg.twist.linear.x}, {msg.twist.linear.y}, {msg.twist.linear.z}, {msg.twist.angular.z}')
            self.publishTrajectoryVelocitySetpoint(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.z)


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