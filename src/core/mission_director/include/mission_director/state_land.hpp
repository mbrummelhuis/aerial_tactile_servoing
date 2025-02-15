#ifndef STATE_LAND_HPP
#define STATE_LAND_HPP

#include "mission_director/state.hpp"

class StateLand : public State {
public:
    StateLand() : State() {}

    void execute() override;

    void checkTransition();
private:
    std::string state_name_ = "Land";

    int frequency_;

    float previous_altitude_ref_;

    float fixed_land_x_;
    float fixed_land_y_;

    float current_altitude_;

    bool is_landed_;

    const float downward_speed_ = 0.5; // m/s
    
};

#endif // STATE_LAND_HPP
