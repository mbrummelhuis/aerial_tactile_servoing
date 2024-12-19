#include <string>
#include <chrono>

#include "state_hover.hpp"
#include "state_land.hpp"

using namespace std::chrono;

/* @brief Hover state
            Keep current position and altitude
 */
StateHover::StateHover() : State() {
    start_time_ = std::chrono::steady_clock::now(); 
}

/* @brief Execute the state logic: Set offboard position mode and publish position setpoint
*/
void StateHover::execute() {
    OffboardControlMode offboard_msg{};
	offboard_msg.position = true;
	offboard_msg.velocity = false;
	offboard_msg.acceleration = false;
	offboard_msg.attitude = false;
	offboard_msg.body_rate = false;
    context_.lock()->publishOffboardControlmode(std::make_shared<OffboardControlMode>(offboard_msg));

	TrajectorySetpoint setpoint_msg{};
	setpoint_msg.position = {vehicle_local_position_.x, vehicle_local_position_.y, vehicle_local_position_.z};
	setpoint_msg.yaw = vehicle_local_position_.heading; // [-PI:PI]
    context_.lock()->publishTrajectorySetpoint(std::make_shared<TrajectorySetpoint>(setpoint_msg));

    checkTransition();
}

// Check for and initiate transition if conditions are met
void StateHover::checkTransition() {
    // condition: after x seconds
    auto current_time = std::chrono::steady_clock::now();
    if (current_time - start_time_ > std::chrono::seconds(wait_time_)) {
        context_.lock()->setState(std::make_shared<StateLand>());
    }
}