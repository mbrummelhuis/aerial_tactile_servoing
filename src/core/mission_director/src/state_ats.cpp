#include <string>
#include <chrono>

#include "state_ats.hpp"


using namespace std::chrono;

/* @brief Tactile servoing state
            Executes the experimental tactile servoing controller.
 */
StateATS::StateATS() : State() {
}

/* @brief Execute the state logic: Set offboard position mode and publish position setpoint
*/
void StateATS::execute() {
    OffboardControlMode offboard_msg{};
	offboard_msg.position = true;
	offboard_msg.velocity = false;
	offboard_msg.acceleration = false;
	offboard_msg.attitude = false;
	offboard_msg.body_rate = false;
    context_.lock()->publishOffboardControlmode(std::make_shared<OffboardControlMode>(offboard_msg));

	TrajectorySetpoint setpoint_msg{};
	setpoint_msg.position = {0.0, 0.0, -2.0};
	setpoint_msg.yaw = -3.14; // [-PI:PI]
    context_.lock()->publishTrajectorySetpoint(std::make_shared<TrajectorySetpoint>(setpoint_msg));

    checkTransition();
}
