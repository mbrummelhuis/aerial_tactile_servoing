#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <mission_director/md_ros_wrapper.hpp>
#include <mission_director/mission_director.hpp>
#include <mission_director/state.hpp>
#include <mission_director/state_disarmed.hpp>
//using namespace std::chrono_literals;


/* @brief 
*/
MissionDirector::MissionDirector() {
  // Set the backreference to the ROS2 wrapper
    setWrapper(std::make_shared<MDROSWrapper>());
    wrapper_->logInfo("Mission director initiated.");

    // Set initial state
    SetState(std::make_shared<StateDisarmed>());
}

void MissionDirector::setWrapper(std::shared_ptr<MDROSWrapper> wrapper) {
    wrapper_ = wrapper;
    wrapper_->logInfo("Setting Mission Director on ROS2 wrapper");
}

void MissionDirector::SetState(std::shared_ptr<State> new_state) {
    current_state_ = new_state;
    wrapper_->logInfo("State changed to: " + current_state_->getStateName());
}

void MissionDirector::execute() {
    current_state_->runState();
}