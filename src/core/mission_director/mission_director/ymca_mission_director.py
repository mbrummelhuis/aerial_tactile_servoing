import rclpy
import numpy as np

from mission_director.uam_state_machine import UAMStateMachine

class MissionDirector(UAMStateMachine):
    def __init__(self):
        super().__init__('mission_director', fcu_on=False)
        self.get_logger().info("MissionDirector node uam_control_test initialized.")

        # Timer -- always last
        self.counter = 0
        self.timer = self.create_timer(self.timer_period, self.execute)

    def execute(self):
        match self.FSM_state:
            case "entrypoint":
                self.state_entrypoint(next_state="move_arms")

            case "move_arms":
                q = [1.0, 0.1, 1.3,
                     -1.0, -0.1, -1.3]
                self.state_move_arms(q=q, next_state="arms_takeoff_position")

            case "arms_takeoff_position":
                q = [1.57, 0.0, -1.57,
                     -1.57, 0.0, 1.57]
                self.state_move_arms(q=q, next_state="sim_arm_vehicle")
            
            case "sim_arm_vehicle": # TODO set to wait for arm state for flight test
                self.state_sim_arm(next_state="takeoff")

            case "takeoff":
                self.state_takeoff(target_altitude = 1.5, next_state="hover")

            case "hover":
                self.state_hover(duration_sec=5, next_state="Y")

            case "Y":
                q = [1.0, 0.0, -0.2,
                    -1.0, 0.0, 0.2]
                self.state_move_arms(q=q, next_state="M")
            
            case "M":
                q = [0.7, 0.0, -1.8,
                    -0.7, 0.0, 1.8]
                self.state_move_arms(q=q, next_state="C")
            
            case "C":
                q = [1.3, 0.0, 0.3, # 0.7, 0.0, -0.8
                    -4.5, 0.0, -0.3]
                self.state_move_arms(q=q, next_state="A")   
            
            case "A":
                q = [0.8, 0.0, -1.6,
                    0.8, 0.0, 1.6]
                self.state_move_arms(q=q, next_state="wave_left")

            case "wave_left":
                q = [1.4, 0.5, -1.5,
                    -1.4, 0.0, -1.5]
                self.state_move_arms(q=q, next_state="wave_right")
            
            case "wave_right":
                q = [1.4, 0.0, 1.5,
                    -1.4, 0.5, 1.5]
                self.state_move_arms(q=q, next_state="land")

            case "land":
                self.state_land(next_state="disarm")

            # --- Do not remove these states ---
            case "emergency":
                self.state_emergency()

            case _:
                self.get_logger().error(f"Unknown state: {self.FSM_state}")
                self.transition_to_state(new_state="emergency")
def main():
    rclpy.init(args=None)

    md = MissionDirector()

    rclpy.spin(md)
    md.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()