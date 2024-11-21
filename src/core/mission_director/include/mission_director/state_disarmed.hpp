#ifndef STATE_DISARMED_HPP
#define STATE_DISARMED_HPP

#include "state.hpp"
#include "mission_director.hpp"

class StateDisarmed : public State {
public:
    StateDisarmed() : State() {}

    void runState() override;

    std::string getStateName() const override;
};


#endif // STATE_DISARMED_HPP
