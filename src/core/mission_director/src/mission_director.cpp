#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <mission_director/mission_director.hpp>
#include <mission_director/state.hpp>
#include <mission_director/state_disarmed.hpp>

using namespace px4_msgs::msg;

/* @brief 
*/
MissionDirector::MissionDirector() : Node("mission_director") {
    RCLCPP_INFO(this->get_logger(),"Mission director initiated.");

    // Set initial state
    setState(std::make_shared<StateDisarmed>());

    // subscribers
    subscriber_vehicle_status_ = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status", 10, std::bind(&MissionDirector::vehicleStatusCallback, this, std::placeholders::_1));
    subscriber_distance_sensor_ = this->create_subscription<DistanceSensor>("/fmu/out/distance_sensor", 10, std::bind(&MissionDirector::vehicleDistanceSensorCallback, this, std::placeholders::_1));
    subscriber_vehicle_local_position_ = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", 10, std::bind(&MissionDirector::vehicleLocalPositionCallback, this, std::placeholders::_1));
    
    // publishers
    publisher_trajectory_setpoint_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    publisher_vehicle_command_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    publisher_offboard_control_mode_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

    // make timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / frequency_), std::bind(&MissionDirector::execute, this));
}

void MissionDirector::logInfo(std::string message) {
    RCLCPP_INFO(this->get_logger(), message.c_str());
}

void MissionDirector::setState(std::shared_ptr<State> new_state) {
    current_state_ = new_state;
}

void MissionDirector::execute() {
    current_state_->execute();
}

void MissionDirector::vehicleStatusCallback(const VehicleStatus::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: [%d]", msg->nav_state);
    current_state_->setVehicleStatus(msg);
}

void MissionDirector::vehicleDistanceSensorCallback(const DistanceSensor::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: [%f]", msg->current_distance);
    current_state_->setVehicleAltitude(msg);
}

void MissionDirector::vehicleLocalPositionCallback(const VehicleLocalPosition::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: [%f]", msg->x);
    current_state_->setVehicleLocalPosition(msg);
}

void MissionDirector::vehicleLandDetectedCallback(const VehicleLandDetected::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: [%i]", msg->landed);
    current_state_->setVehicleLandDetected(msg);
}

void MissionDirector::publishTrajectorySetpoint(TrajectorySetpoint::SharedPtr msg) {
	msg->timestamp = this->get_clock()->now().nanoseconds() / 1000; // add the timestamp
    publisher_trajectory_setpoint_->publish(*msg); // publish the message
}

void MissionDirector::publishVehicleCommand(VehicleCommand::SharedPtr msg) {
    msg->timestamp = this->get_clock()->now().nanoseconds() / 1000; // add the timestamp
    publisher_vehicle_command_->publish(*msg); // publish the message
}

void MissionDirector::publishOffboardControlmode(OffboardControlMode::SharedPtr msg) {
    msg->timestamp = this->get_clock()->now().nanoseconds() / 1000; // add the timestamp
    publisher_offboard_control_mode_->publish(*msg); // publish the message
}

int MissionDirector::getFrequency() const {
    return frequency_;
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MissionDirector>());
	rclcpp::shutdown();
	return 0;
}