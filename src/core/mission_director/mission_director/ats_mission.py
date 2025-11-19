import datetime
import rclpy

from mission_director.uam_state_machine import UAMStateMachine

class MissionDirector(UAMStateMachine):
    def __init__(self):
        super().__init__('mission_director')
        self.get_logger().info("MissionDirector node initialized.")


        # Timer -- always last
        self.counter = 0
        self.timer = self.create_timer(self.timer_period, self.execute)


    def execute(self):
        match self.FSM_state:
            case "entrypoint":
                self.state_entrypoint(next_state="move_arms")

            case "move_arms":
                self.state_move_arms(q=[0.0, 0.0, 0.0, 0.0], next_state="takeoff")

            case "takeoff":
                self.state_takeoff(target_altitude=1.5, next_state="hover")

            case "hover":
                self.state_hover(duration_sec=10, next_state="hover_move_arms")

            case "hover_move_arms":
                self.state_move_arms(q=[1.0, 0.5, -0.5, -1.0], next_state="land")
            
            case "land":
                self.state_land(next_state="disarm")

                

def main():
    rclpy.init(args=None)

    md = MissionDirector()

    rclpy.spin(md)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    md.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()