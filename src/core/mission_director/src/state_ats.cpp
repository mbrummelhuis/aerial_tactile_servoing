#include <string>
#include <chrono>

#include "state_ats.hpp"
#include "state_hover.hpp"


using namespace std::chrono;

/* @brief Tactile servoing state
            Executes the experimental tactile servoing controller.
 */
StateATS::StateATS() : State() {
	start_time_ = std::chrono::steady_clock::now(); 
}

/* @brief Execute the state logic: Set offboard position mode and publish position setpoint
*/
void StateATS::execute() {
    OffboardControlMode offboard_msg{};
	offboard_msg.position = false;
	offboard_msg.velocity = true;
	offboard_msg.acceleration = false;
	offboard_msg.attitude = false;
	offboard_msg.body_rate = false;
    context_.lock()->publishOffboardControlmode(std::make_shared<OffboardControlMode>(offboard_msg));

	TrajectorySetpoint setpoint_msg{};
	setpoint_msg.velocity = {0.0, 0.0, -2.0};
	setpoint_msg.yawspeed = -3.14; // [-PI:PI]
    context_.lock()->publishTrajectorySetpoint(std::make_shared<TrajectorySetpoint>(setpoint_msg));

    checkTransition();
}

void StateATS::checkTransition() {
    // condition: after x seconds
    auto current_time = std::chrono::steady_clock::now();
    if (current_time - start_time_ > std::chrono::seconds(wait_time_)) {
        context_.lock()->setState(std::make_shared<StateHover>());
    }
}
