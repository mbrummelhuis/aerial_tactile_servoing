
#include "state_land.hpp"
#include "state_disarmed.hpp"

StateLand::StateLand() : State() {
    is_landed_ = false;
    previous_altitude_ref_ = 0.0;
    fixed_land_x_ = vehicle_local_position_.x;
    fixed_land_y_ = vehicle_local_position_.y;
    frequency_ = context_.lock()->getFrequency();
}

void StateLand::execute() {
    // calculate new altitude reference for descent
    float new_altitude_ref = current_altitude_ - downward_speed_ / frequency_;
    
    // set and publish offboard control position mode
    OffboardControlMode offboard_msg{};
	offboard_msg.position = true;
	offboard_msg.velocity = false;
	offboard_msg.acceleration = false;
	offboard_msg.attitude = false;
	offboard_msg.body_rate = false;
    context_.lock()->publishOffboardControlmode(std::make_shared<OffboardControlMode>(offboard_msg));

	TrajectorySetpoint setpoint_msg{};
	setpoint_msg.position = {fixed_land_x_, fixed_land_y_, -new_altitude_ref};
	setpoint_msg.yaw = -3.14; // [-PI:PI]
    context_.lock()->publishTrajectorySetpoint(std::make_shared<TrajectorySetpoint>(setpoint_msg));

    checkTransition();
}

void StateLand::checkTransition() {
    // condition
    if (is_landed_ == true) {
        context_.lock()->setState(std::make_shared<StateDisarmed>());
    }
}