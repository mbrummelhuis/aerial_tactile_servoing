#ifndef STATE_HOVER_HPP
#define STATE_HOVER_HPP

#include "rclcpp/rclcpp.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>

#include "state.hpp"

class StateHover : public State {
public:
    StateHover() : State() {}

    void execute() override;

    void setVehicleLocalPosition(const VehicleLocalPosition::SharedPtr msg) override;

    void checkTransition();

private:
    std::string state_name_ = "Hover";

    std::chrono::time_point<std::chrono::steady_clock> start_time_;

    float current_position_ = 0.0;


    const int wait_time_ = 5; // seconds
};

#endif // STATE_HOVER_HPP