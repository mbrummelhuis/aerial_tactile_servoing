#ifndef MISSION_DIRECTOR_NODE_HPP
#define MISSION_DIRECTOR_NODE_HPP

// includes
#include "rclcpp/rclcpp.hpp"


#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

#include "mission_director/state.hpp"

using namespace px4_msgs::msg;

class MissionDirector : public rclcpp::Node {
    public:
        MissionDirector();

        void logInfo(std::string message);

        void setState(std::shared_ptr<State> new_state);

        void runState();

        void execute();

        std::string getCurrentState() const;

        int getFrequency() const;

        void publishTrajectorySetpoint(TrajectorySetpoint::SharedPtr msg);
        void publishVehicleCommand(VehicleCommand::SharedPtr msg);
        void publishOffboardControlmode(OffboardControlMode::SharedPtr msg);
    
    private:
        int frequency_ = 100; // Hz
        std::shared_ptr<State> current_state_;
        rclcpp::TimerBase::SharedPtr timer_;

        // subscribers
        rclcpp::Subscription<VehicleStatus>::SharedPtr subscriber_vehicle_status_;
        rclcpp::Subscription<DistanceSensor>::SharedPtr subscriber_distance_sensor_;
        rclcpp::Subscription<VehicleLocalPosition>::SharedPtr subscriber_vehicle_local_position_;
        rclcpp::Subscription<VehicleLandDetected>::SharedPtr subscriber_vehicle_land_detected_;
        // publishers
        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr publisher_trajectory_setpoint_;
        rclcpp::Publisher<VehicleCommand>::SharedPtr publisher_vehicle_command_;
        rclcpp::Publisher<OffboardControlMode>::SharedPtr publisher_offboard_control_mode_;

        // callbacks
        void vehicleStatusCallback(const VehicleStatus::SharedPtr msg);
        void vehicleDistanceSensorCallback(const DistanceSensor::SharedPtr msg);
        void vehicleLocalPositionCallback(const VehicleLocalPosition::SharedPtr msg);
        void vehicleLandDetectedCallback(const VehicleLandDetected::SharedPtr msg);
        // data
        TrajectorySetpoint trajectory_setpoint_;
};


#endif // MISSION_DIRECTOR_NODE_HPP