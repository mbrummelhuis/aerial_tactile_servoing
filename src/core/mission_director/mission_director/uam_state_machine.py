import datetime

import numpy as np

from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState

from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleLocalPosition

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class UAMStateMachine(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.get_logger().info(f"StateMachine node '{node_name}' initialized.")

        # Parameters
        self.declare_parameter('sm.frequency', 10.0)
        self.frequency = self.get_parameter('sm.frequency').get_parameter_value().double_value
        self.timer_period = 1.0 / self.frequency
        self.declare_parameter('sm.position_clip', 0.0)
        self.position_clip = self.get_parameter('sm.position_clip').get_parameter_value().double_value
        self.declare_parameter('sm.takeoff_altitude', 1.0)
        self.takeoff_altitude = self.get_parameter('sm.takeoff_altitude').get_parameter_value().double_value
        self.declare_parameter('sm.dry_test', False)
        self.dry_test = self.get_parameter('sm.dry_test').get_parameter_value().bool_value

        # State machine publishers and subscribers
        self.pub_sm_state = self.create_publisher(Int32, '/md/state', 10)
        self.sub_sm_input = self.create_subscription(Int32, '/md/input', self.input_callback, 10)

        # Flight controller interfaces
        px4_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub_vehicle_trajectory_setpoint = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.pub_offboard_control_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode',10)
        self.pub_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.sub_vehicle_status = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, px4_qos_profile)
        self.sub_vehicle_odometry = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, px4_qos_profile)
        self.sub_vehicle_local_position = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, px4_qos_profile)

        # Manipulator interfaces
        self.pub_servo_references = self.create_publisher(JointState, '/servo/in/state', 10)
        self.sub_servo_states = self.create_subscription(JointState, '/servo/out/state', self.servo_state_callback, 10)

        # Attributes
        self.FSM_state = "entrypoint"
        self.input_state = 0
        self.first_state_loop = True
        self.armed = False
        self.offboard = False
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.state_start_time = datetime.datetime.now()
        self.servo_state = JointState()
        self.home_position = np.zeros(4)  # x, y, z, heading
        self.hover_position = np.zeros(4)  # x, y, z, heading
        self.land_position = np.zeros(4)  # x, y, z, heading
        self.running_position = np.zeros(4)  # x, y, z, heading

    def publish_offboard_position_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.pub_offboard_control_mode.publish(msg)

    def publish_trajectory_position_setpoint(self, x: float, y: float, z: float, yaw: float, yawspeed: float = 0.):
        # If clipping is not zero, clip the position
        if self.position_clip > 0.1:
            x_clipped = np.clip(x, -self.position_clip, self.position_clip)
            y_clipped = np.clip(y, -self.position_clip, self.position_clip)
            z_clipped = np.clip(z, -self.position_clip, 0.0) # Negative up
        else:
            x_clipped = x
            y_clipped = y
            z_clipped = z
        msg = TrajectorySetpoint()
        msg.position[0] = x_clipped
        msg.position[1] = y_clipped
        msg.position[2] = z_clipped
        msg.yaw = yaw
        msg.yawspeed=yawspeed
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.pub_vehicle_trajectory_setpoint.publish(msg)

    def transition_to_state(self, new_state='emergency'):
        if self.input_state != 0:
            self.get_logger().info('Manually triggered state transition')
            self.input_state = 0
        self.state_start_time = datetime.datetime.now() # Reset start time for next state
        self.first_state_loop = True # Reset first loop flag
        self.get_logger().info(f"Transition from {self.FSM_state} to {new_state}")
        self.FSM_state = new_state
        self.counter = 0

    def handle_state(self, state_number: int):
        self.publish_md_state(state_number)
        self.publish_offboard_position_mode()
        self.counter += 1

    def publish_md_state(self, state):
        msg = Int32()
        msg.data = state
        self.pub_sm_state.publish(msg)

    def publish_servo_position_references(self, q_list: list):
        msg = JointState()
        for q in q_list:
            msg.name.append('q'+str(len(msg.position)+1))
            msg.position.append(q)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_servo_references.publish(msg)

    #--------------------------------------------------------------------------
    # Callbacks
    #--------------------------------------------------------------------------
    def input_callback(self, msg: Int32):
        self.input_state = msg.data

    def vehicle_status_callback(self, msg: VehicleStatus):
        if msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.armed = True
        else:
            self.armed = False

        if msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.offboard = True
        else:
            self.offboard = False
        
        if msg.latest_disarming_reason == VehicleStatus.ARM_DISARM_REASON_KILL_SWITCH:
            self.killed = True
        else:
            self.killed = False

        self.vehicle_status = msg
    
    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        self.vehicle_odometry = msg

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        self.vehicle_local_position = msg
    
    def servo_state_callback(self, msg: JointState):
        self.servo_state = msg

    #--------------------------------------------------------------------------
    # State Machine state executors
    #--------------------------------------------------------------------------
    def state_entrypoint(self, next_state='emergency'):
        self.handle_state(state_number=0)
        self.home_position[0] = self.vehicle_local_position.x
        self.home_position[1] = self.vehicle_local_position.y
        self.home_position[2] = self.takeoff_altitude
        self.home_position[3] = self.vehicle_local_position.heading        
        
        if self.first_state_loop:
            self.get_logger().info("Waiting for position fix")
            self.first_state_loop = False

        # State transition
        if (self.home_position[0] != 0.0 and self.home_position[1] != 0.0):
            self.get_logger().info(f'Got position fix! \t x: {self.home_position[0]:.3f} [m] \t y: {self.home_position[1]:.3f} [m] \t {np.rad2deg(self.home_position[3]):.2f} [deg]')
            self.transition_to_state(new_state=next_state)
        elif self.input_state == 1:
            self.get_logger().info('Bypassing position fix wait')
            self.transition_to_state(new_state=next_state)

    def state_wait_for_arming(self, next_state='emergency'):
        self.handle_state(state_number=3)

        # State transition
        if self.armed and not self.offboard:
            self.get_logger().info('Armed but not offboard -- waiting', throttle_duration_sec=1)
        elif not self.armed and self.offboard:
            self.get_logger().info('Not armed but offboard -- waiting', throttle_duration_sec=1)
        elif (self.armed and self.offboard) or self.input_state == 1:
            self.transition_to_state(new_state=next_state)

    def state_takeoff(self, next_state='emergency'):
        self.handle_state(state_number=4)
        self.publish_trajectory_position_setpoint(*self.home_position)
        current_altitude = self.vehicle_local_position.z

        # First state loop
        if self.first_state_loop:
            self.get_logger().info(f'[4] Vehicle local position heading: {self.home_position[3]} rad')
            self.get_logger().info(f'[4] Takeoff altitude: {self.home_position[2]} m')
            self.first_state_loop = False

        # State transition
        if not self.offboard and not self.dry_test:
            self.transition_to_state('emergency')
        elif abs(current_altitude)+0.1 > abs(self.home_position[2]) or self.input_state==1:
            self.transition_to_state(new_state=next_state)

    def state_land(self, landing_speed=0.5, next_state='emergency'):
        self.handle_state(state_number=5)
        self.publish_trajectory_position_setpoint(*self.land_position)
        self.land_position[2] += landing_speed / self.frequency  # Increase z (down) at landing speed

        # First state loop
        if self.first_state_loop:
            self.land_position[0] = self.vehicle_local_position.x
            self.land_position[1] = self.vehicle_local_position.y
            self.land_position[2] = self.vehicle_local_position.z
            self.land_position[3] = self.vehicle_local_position.heading
            self.get_logger().info(f'[5] Landing from altitude: {self.land_position[2]} m at speed {landing_speed} m/s')
            self.first_state_loop = False

        # State transition
        if not self.offboard and not self.dry_test:
            self.transition_to_state('emergency')
        elif (self.vehicle_local_position.z >= -0.1 and self.land_position[2] > 0.5) or self.input_state==1:
            self.transition_to_state(new_state=next_state)

    def state_hover(self, duration_sec: float, next_state='emergency'):
        self.handle_state(state_number=10)

        # First state loop
        if self.first_state_loop:
            self.hover_position[0] = self.vehicle_local_position.x
            self.hover_position[1] = self.vehicle_local_position.y
            self.hover_position[2] = self.vehicle_local_position.z
            self.hover_position[3] = self.vehicle_local_position.heading
            self.get_logger().info(f'[5] Hovering at altitude: {self.home_position[2]} m for {duration_sec} seconds')
            self.first_state_loop = False

        self.get_logger().info(f'Hovering... {(datetime.datetime.now()-self.state_start_time).seconds:.1f}/{duration_sec} sec', throttle_duration_sec=1)

        # State transition
        if not self.offboard and not self.dry_test:
            self.transition_to_state('emergency')
        elif (datetime.datetime.now()-self.state_start_time).seconds > duration_sec or self.input_state==1:
            self.transition_to_state(new_state=next_state)

    def state_move_arms(self, q: list, next_state='emergency'):
        self.handle_state(state_number=11)

        # First state loop
        if self.first_state_loop:
            self.hover_position[0] = self.vehicle_local_position.x
            self.hover_position[1] = self.vehicle_local_position.y
            self.hover_position[2] = self.vehicle_local_position.z
            self.hover_position[3] = self.vehicle_local_position.heading
            self.publish_servo_position_references(q)
            self.get_logger().info(f'[6] Hovering at altitude: {self.home_position[2]} m while moving arms to states {q}')
            self.first_state_loop = False

        # Calculate euclidean distance between current and target servo positions
        current_q = np.array(self.servo_state.position)
        target_q = np.array(q)
        error = np.linalg.norm(current_q - target_q)

        # State transition
        if not self.offboard and not self.dry_test:
            self.transition_to_state('emergency')
        elif (error < 0.01) or self.input_state==1: # If error is small enough or input state is 1
            self.transition_to_state(new_state=next_state)