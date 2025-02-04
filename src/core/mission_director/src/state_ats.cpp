#include <string>
#include <chrono>

#include "state_ats.hpp"
#include "state_hover.hpp"

#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>


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
    publishBodyRateMode();
    publishVehicleRatesSetpoint();

    checkTransition();
}

void StateATS::publishBodyRateMode()
{
    OffboardControlMode offboard_msg{};
	offboard_msg.position = false;
	offboard_msg.velocity = false;
	offboard_msg.acceleration = false;
	offboard_msg.attitude = false;
	offboard_msg.body_rate = true;
    offboard_msg.thrust_and_torque = false;
    offboard_msg.direct_actuator = false;
    context_.lock()->publishOffboardControlmode(std::make_shared<OffboardControlMode>(offboard_msg));
}

void StateATS::publishVehicleRatesSetpoint()
{
    VehicleRatesSetpoint rates_setpoint_msg{};
    rates_setpoint_msg.roll = reference_body_velocity_.twist.angular.x; // roll rate
    rates_setpoint_msg.pitch = reference_body_velocity_.twist.angular.y; // pitch rate
    rates_setpoint_msg.yaw = reference_body_velocity_.twist.angular.z; // yaw rate
    rates_setpoint_msg.thrust_body[0] = 0.0;
    rates_setpoint_msg.thrust_body[1] = 0.0;
    rates_setpoint_msg.thrust_body[2] = reference_body_velocity_.twist.linear.z; // negative throttle demand
    context_.lock()->publishVehicleRatesSetpoint(std::make_shared<VehicleRatesSetpoint>(rates_setpoint_msg));

}

void StateATS::checkTransition() {
    // condition: after x seconds
    auto current_time = std::chrono::steady_clock::now();
    if (current_time - start_time_ > std::chrono::seconds(wait_time_)) {
        context_.lock()->setState(std::make_shared<StateHover>());
    }
}
