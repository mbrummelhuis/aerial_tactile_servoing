#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <mission_director/mission_director.hpp>
#include <mission_director/state.hpp>
#include <mission_director/state_disarmed.hpp>

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
    subscriber_vehicle_angular_velocity_ = this->create_subscription<VehicleAngularVelocity>("/fmu/out/vehicle_angular_velocity", 10, std::bind(&MissionDirector::vehicleAngularVelocityCallback, this, std::placeholders::_1));
    subscriber_vehicle_attitude_ = this->create_subscription<VehicleAttitude>("/fmu/out/vehicle_attitude", 10, std::bind(&MissionDirector::vehicleAttitudeCallback, this, std::placeholders::_1));
    
    subscriber_reference_body_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/references/body_velocities", 10, std::bind(&MissionDirector::referenceBodyVelocityCallback, this, std::placeholders::_1));

    // publishers
    publisher_trajectory_setpoint_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    publisher_vehicle_command_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    publisher_offboard_control_mode_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    publisher_vehicle_rates_setpoint_ = this->create_publisher<VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", 10);

    publisher_body_angles_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/state/body_angles", 10);
    publisher_body_angular_velocity_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/state/body_angular_velocity", 10);


    // make timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / frequency_), std::bind(&MissionDirector::execute, this));
}

void MissionDirector::logInfo(std::string message) {
    RCLCPP_INFO(this->get_logger(), message.c_str());
}

void MissionDirector::setState(std::shared_ptr<State> new_state) {
    RCLCPP_INFO(this->get_logger(), "State changed to: %s", new_state->getStateName().c_str());
    current_state_ = new_state;
}

void MissionDirector::execute() {
    publishVehicleState();
    current_state_->execute();
}

void MissionDirector::vehicleStatusCallback(const VehicleStatus::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "I heard: [%d]", msg->nav_state);
    current_state_->setVehicleStatus(msg);
}

void MissionDirector::vehicleDistanceSensorCallback(const DistanceSensor::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "I heard: [%f]", msg->current_distance);
    current_state_->setVehicleAltitude(msg);
}

void MissionDirector::vehicleLocalPositionCallback(const VehicleLocalPosition::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "I heard: [%f]", msg->x);
    current_state_->setVehicleLocalPosition(msg);
}

void MissionDirector::vehicleLandDetectedCallback(const VehicleLandDetected::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "I heard: [%i]", msg->landed);
    current_state_->setVehicleLandDetected(msg);
}

void MissionDirector::tactileSensorPoseCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "I heard: [%f]", msg->twist.linear.x);
    current_state_->setTactileSensorPose(msg);
}

void MissionDirector::vehicleAngularVelocityCallback(const VehicleAngularVelocity::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "I heard: [%f]", msg->xyz[0]);
    current_state_->setVehicleAngularVelocity(msg);
}

void MissionDirector::vehicleAttitudeCallback(const VehicleAttitude::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "I heard: [%f]", msg->q[0]);
    current_state_->setVehicleAttitude(msg);
}

void MissionDirector::referenceBodyVelocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "I heard: [%f]", msg->twist.linear.x);
    current_state_->setReferenceBodyVelocity(msg);
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

void MissionDirector::publishVehicleState() {
    current_state_->publishVehicleState();
}

void MissionDirector::publishBodyAngles(geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
    msg->header.stamp = this->get_clock()->now(); // add the timestamp
    publisher_body_angles_->publish(*msg); // publish the message
}

void MissionDirector::publishBodyAngularVelocity(geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
    msg->header.stamp = this->get_clock()->now(); // add the timestamp
    publisher_body_angular_velocity_->publish(*msg); // publish the message
}

void MissionDirector::publishVehicleRatesSetpoint(VehicleRatesSetpoint::SharedPtr msg) {
    msg->timestamp = this->get_clock()->now().nanoseconds() / 1000; // add the timestamp
    publisher_vehicle_rates_setpoint_->publish(*msg); // publish the message
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