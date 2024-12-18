#ifndef STATE_ATS_HPP
#define STATE_ATS_HPP

#include "rclcpp/rclcpp.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>

#include "state.hpp"

class StateATS : public State {
public:
    StateATS() : State() {}

    void execute() override;

    void checkTransition() override;

private:
    std::string state_name_ = "ATS";
};

#endif // STATE_ATS_HPP