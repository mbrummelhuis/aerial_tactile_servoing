#ifndef STATE_ATS_HPP
#define STATE_ATS_HPP

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>

#include "mission_director/state.hpp"

class StateATS : public State {
public:
    StateATS() : State() {}

    void execute() override;

    void checkTransition() override;

private:
    std::string state_name_ = "ATS";

    std::chrono::time_point<std::chrono::steady_clock> start_time_;

    const int wait_time_ = 60; // seconds

    // functions
    void publishBodyRateMode();
    void publishVehicleRatesSetpoint();
};

#endif // STATE_ATS_HPP