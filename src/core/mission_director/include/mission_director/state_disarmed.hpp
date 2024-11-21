#ifndef STATE_DISARMED_HPP
#define STATE_DISARMED_HPP

#include "mission_director.hpp"
#include "state_takeoff.hpp"

class StateDisarmed : public State {
public:
    StateDisarmed() : State() {}

    void execute() override;

    void setVehicleStatus(const VehicleStatus::SharedPtr msg) override;

    void checkTransition();

private:
    std::string state_name_ = "Disarmed";
    bool is_armed_;
    bool is_offboard_;
};


#endif // STATE_DISARMED_HPP
