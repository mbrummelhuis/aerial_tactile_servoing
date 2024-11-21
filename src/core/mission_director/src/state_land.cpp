
#include "state_land.hpp"
#include "state_disarmed.hpp"

StateLand::StateLand() : State() {
    is_landed_ = false;
    previous_altitude_ref_ = 0.0;
    fixed_land_x_ = current_local_x_;
    fixed_land_y_ = current_local_y_;
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

void StateLand::setVehicleAltitude(const DistanceSensor::SharedPtr msg) {
    current_altitude_ = msg->current_distance;
}

void StateLand::setVehicleLocalPosition(const VehicleLocalPosition::SharedPtr msg) {
    current_local_x_ = msg->x;
    current_local_y_ = msg->y;
}

void StateLand::setVehicleLandDetected(const VehicleLandDetected::SharedPtr msg) {
    if (msg->landed == true) {
        is_landed_ = true;
    }
    else {
        is_landed_ = false;
    }
}

void StateLand::checkTransition() {
    // condition
    if (is_landed_ == true) {
        context_.lock()->setState(std::make_shared<StateDisarmed>());
    }
}