

#include "state.hpp"

State::State() {
    // Initializer code goes here
}

// Any general state code goes heres

void State::setVehicleStatus(const VehicleStatus::SharedPtr msg) {
    vehicle_status_ = *msg;
}

void State::setVehicleAltitude(const DistanceSensor::SharedPtr msg) {
    vehicle_altitude_ = *msg;
}

void State::setVehicleLocalPosition(const VehicleLocalPosition::SharedPtr msg) {
    vehicle_local_position_ = *msg;
}

void State::setVehicleLandDetected(const VehicleLandDetected::SharedPtr msg) {
    vehicle_land_detected_ = *msg;
}

void State::setTactileSensorPose(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    tactile_sensor_pose_ = *msg;
}

void State::setVehicleAngularVelocity(const VehicleAngularVelocity::SharedPtr msg) {
    vehicle_angular_velocity_ = *msg;
}

void State::setVehicleAttitude(const VehicleAttitude::SharedPtr msg) {
    vehicle_attitude_ = *msg;
}

void State::publishVehicleState() {
    geometry_msgs::msg::Vector3Stamped body_angle_msg;
    std::vector<double> q = {vehicle_attitude_.q[0], vehicle_attitude_.q[1], vehicle_attitude_.q[2], vehicle_attitude_.q[3]};
    std::vector<double> angles = QuatToEuler(q);

    body_angle_msg.vector.x = angles[2]; // roll
    body_angle_msg.vector.y = angles[1]; // roll
    body_angle_msg.vector.z = angles[0]; // yaw

    geometry_msgs::msg::Vector3Stamped body_angular_velocity_msg;
    body_angular_velocity_msg.vector.x = vehicle_angular_velocity_.xyz[0]; // roll rate
    body_angular_velocity_msg.vector.y = vehicle_angular_velocity_.xyz[1]; // pitch rate
    body_angular_velocity_msg.vector.z = vehicle_angular_velocity_.xyz[2]; // yaw rate

    context_.lock()->publishBodyAngles(std::make_shared<geometry_msgs::msg::Vector3Stamped>(body_angle_msg));
    context_.lock()->publishBodyAngularVelocity(std::make_shared<geometry_msgs::msg::Vector3Stamped>(body_angular_velocity_msg));
}

std::vector<double> State::QuatToEuler(std::vector<double> q) {
    std::vector<double> angles(3); // order yaw pitch roll
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
    double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
    angles[2] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q[0] * q[2] - q[1] * q[3]));
    double cosp = std::sqrt(1 - 2 * (q[0] * q[2] - q[1] * q[3]));
    angles[1] = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
    double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    angles[0] = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}