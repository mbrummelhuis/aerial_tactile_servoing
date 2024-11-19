#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <mission_director/mission_director.hpp>
#include <mission_director/state.hpp>

//using namespace std::chrono_literals;


/* @brief 
*/
MissionDirector::MissionDirector() : Node("mission_director") {
    RCLCPP_INFO(this->get_logger(), "Mission Director started");
    setState(std::make_shared<StateDisarmed>());

    // publishers


    // subscribers

    // timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&MissionDirector::timer_callback, this));
}

void MissionDirector::SetState(std::shared_ptr<State> new_state) {
    current_state_ = new_state;
    RCLCPP_INFO(this->get_logger(), "State changed to %s", current_state_->getStateName().c_str());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionDirector>());
  rclcpp::shutdown();
  return 0;
}