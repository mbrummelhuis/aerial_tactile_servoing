#include <string>

#include "state_disarmed.hpp"

/* @brief Disarmed state
            Waits until the drone is armed and put in offboard mode through the remote
 */
StateDisarmed::StateDisarmed() : State() {
    is_armed_ = false;
    is_offboard_ = false;
}

void StateDisarmed::execute() {
    // just check for armed and offboard status
    checkTransition();
}

void StateDisarmed::setVehicleStatus(const VehicleStatus::SharedPtr msg) {
    if (msg->arming_state == VehicleStatus::ARMING_STATE_ARMED) {
        is_armed_ = true;
    }
    else {
        is_armed_ = false;
    }
    if (msg->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        is_offboard_ = true;
    }
    else {
        is_offboard_ = false;
    }
}

void StateDisarmed::checkTransition() {
    if(is_armed_ && is_offboard_) {
        context_.lock()->setState(std::make_shared<StateTakeoff>());
    }
}