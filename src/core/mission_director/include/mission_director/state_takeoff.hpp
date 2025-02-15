#ifndef STATE_TAKEOFF_HPP
#define STATE_TAKEOFF_HPP

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>

#include "mission_director/state.hpp"

class StateTakeoff : public State {
public:
    StateTakeoff() : State() {}

    void execute() override;

    void checkTransition() override;

private:
    std::string state_name_ = "Takeoff";

    float reference_altitude_;
    float current_altitude_;
    const float margin_altitude_ = 0.2;
};

#endif // STATE_TAKEOFF_HPP