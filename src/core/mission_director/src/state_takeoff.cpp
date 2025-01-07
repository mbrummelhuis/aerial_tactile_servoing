#include <string>
#include <chrono>

#include "state_takeoff.hpp"
#include "state_hover.hpp"

using namespace std::chrono;

/* @brief Takeoff state
            Lets drone take off to reference altitude.
 */
StateTakeoff::StateTakeoff() : State() {
    reference_altitude_ = -2.0; // [m] positive down, NED
}

/* @brief Execute the state logic: Set offboard position mode and publish position setpoint at desired altitude. 
            Then check state transition.
*/
void StateTakeoff::execute() {
    OffboardControlMode offboard_msg{};
	offboard_msg.position = true;
	offboard_msg.velocity = false;
	offboard_msg.acceleration = false;
	offboard_msg.attitude = false;
	offboard_msg.body_rate = false;
    context_.lock()->publishOffboardControlmode(std::make_shared<OffboardControlMode>(offboard_msg));

	TrajectorySetpoint setpoint_msg{};
	setpoint_msg.position = {0.0, 0.0, reference_altitude_};
	setpoint_msg.yaw = -3.14; // [-PI:PI]
    context_.lock()->publishTrajectorySetpoint(std::make_shared<TrajectorySetpoint>(setpoint_msg));

    checkTransition();
}

// Check for and initiate transition if conditions are met
void StateTakeoff::checkTransition() {
    // condition
    if (vehicle_local_position_.z - reference_altitude_ > margin_altitude_) {
        context_.lock()->setState(std::make_shared<StateHover>());
    }
}