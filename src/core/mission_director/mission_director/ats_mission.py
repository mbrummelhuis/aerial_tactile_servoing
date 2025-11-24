import rclpy
import numpy as np
import datetime
from std_msgs.msg import Int8
from geometry_msgs.msg import TwistStamped
from px4_msgs.msg import TrajectorySetpoint
from sensor_msgs.msg import JointState

from mission_director.uam_state_machine import UAMStateMachine

class MissionDirector(UAMStateMachine):
    def __init__(self):
        super().__init__('mission_director')
        self.get_logger().info("MissionDirector node ats_mission initialized.")

        # Tactile servoing parameters
        self.contact_depth_threshold = 0.01  # Threshold for tactip z-velocity

        # Tactip interfaces
        self.sub_tactip = self.create_subscription(TwistStamped, '/tactip/pose', self.tactip_callback, 10)
        self.sub_tactip_contact = self.create_subscription(Int8, '/tactip/contact', self.tactip_contact_callback, 10)
        self.tactip_data = TwistStamped()
        self.contact = False

        # Controller interfaces
        self.sub_controller = self.create_subscription(TrajectorySetpoint, '/controller/out/trajectory_setpoint', self.controller_callback, 10)
        self.sub_controller_servo = self.create_subscription(JointState, '/controller/out/servo_positions', self.controller_servo_callback, 10)

        # Tactile servoing specific variables
        self.vehicle_trajectory_setpoint = TrajectorySetpoint()
        self.servo_reference = JointState()

        # Timer -- always last
        self.counter = 0
        self.timer = self.create_timer(self.timer_period, self.execute)


    def execute(self):
        match self.FSM_state:
            case "entrypoint":
                self.state_entrypoint(next_state="move_arms")

            case "move_arms":
                q_right = [0.0, 0.0, 0.0]
                self.state_move_arms(q=q_right.append(q_right[0]-np.pi), next_state="arms_takeoff_position")

            case "arms_takeoff_position":
                q_right = [1.57, 0.0, -1.57] # put some position here
                self.state_move_arms(q=q_right.append(q_right[0]-np.pi), next_state="wait_for_arm_offboard")

            case "wait_for_arm_offboard":
                self.state_wait_for_arming(next_state="takeoff")

            case "takeoff":
                self.state_takeoff(target_altitude=1.5, next_state="hover")

            case "hover":
                self.state_hover(duration_sec=10, next_state="pre_contact_position")

            case "pre_contact_position":
                q_right = [np.pi/3, 0.0, np.pi/6] # put some position here
                self.state_move_arms(q=q_right.append(q_right[0]-np.pi), next_state="land")

            case "approach":
                self.handle_state(state_number=21)
                approach_speed = 0.1  # m/s

                # First state loop
                if self.first_state_loop:
                    self.running_position[0] = self.vehicle_local_position.x
                    self.running_position[1] = self.vehicle_local_position.y
                    self.running_position[2] = self.vehicle_local_position.z
                    self.running_position[3] = self.vehicle_local_position.heading
                    self.get_logger().info(f'[21] Approaching contact surface at {approach_speed} m/s')
                    self.first_state_loop = False
                
                # Update position setpoint
                self.running_position[1] += approach_speed / self.frequency  # Increase y at approach
                self.get_logger().info(f'Approaching... Y setpoint: {self.running_position[1]} m', throttle_duration_sec=1)

                # State transition
                if not self.offboard and not self.dry_test:
                    self.transition_to_state('emergency')
                elif self.contact or self.input_state==1:
                    self.transition_to_state(new_state="tactile_servoing")
            
            case "tactile_servoing":
                self.handle_state(state_number=22)

                # First state loop
                if self.first_state_loop:
                    self.get_logger().info(f'[22] Tactile servoing initiated.')
                    self.first_state_loop = False
                
                # Pass through controller setpoints
                self.publish_trajectory_position_setpoint(
                    self.vehicle_trajectory_setpoint.position[0],
                    self.vehicle_trajectory_setpoint.position[1],
                    self.vehicle_trajectory_setpoint.position[2],
                    self.vehicle_trajectory_setpoint.yaw
                )
                servo_position_references = self.servo_reference.position.copy()
                np.append(servo_position_references, servo_position_references[0] - np.pi)
                self.publish_servo_position_references(servo_position_references) # TODO test this

                if self.tactip_data.twist.linear.z > self.contact_depth_threshold:
                   self.transition_to_state('approach')
                elif (datetime.datetime.now() - self.state_start_time).seconds > 150. or self.input_state==1:
                    self.transition_to_state('land')
                elif not self.offboard or self.killed or self.input_state == 2:
                    self.transition_to_state('emergency')

            case "land":
                self.state_land(next_state="disarm")

    # ------------------------------------------------------------------------------------
    # Tactile servoing specific callbacks
    # ------------------------------------------------------------------------------------
    def tactip_callback(self, msg):
        self.tactip_data = msg
    
    def tactip_contact_callback(self, msg):
        self.contact = bool(msg.data)

    def controller_callback(self, msg):
        self.vehicle_trajectory_setpoint = msg

    def controller_servo_callback(self, msg):
        self.servo_reference = msg

def main():
    rclpy.init(args=None)

    md = MissionDirector()

    rclpy.spin(md)
    md.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()