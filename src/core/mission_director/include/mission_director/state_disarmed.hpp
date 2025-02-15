#ifndef STATE_DISARMED_HPP
#define STATE_DISARMED_HPP

#include "mission_director/mission_director.hpp"
#include "mission_director/state_takeoff.hpp"

#include "mission_director/state.hpp"

class StateDisarmed : public State {
public:
    StateDisarmed() : State() {}

    void execute() override;

    void checkTransition() override;

private:
    std::string state_name_ = "Disarmed";
    bool is_armed_;
    bool is_offboard_;
};


#endif // STATE_DISARMED_HPP
