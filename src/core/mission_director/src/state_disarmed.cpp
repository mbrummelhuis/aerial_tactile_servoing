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

void StateDisarmed::checkTransition() {
    // Condition is armed and offboard
    if (vehicle_status_.arming_state == VehicleStatus::ARMING_STATE_ARMED) {
        if (is_armed_ = false) {
            context_.lock()->logInfo("[State Disarmed] Arm detected.");
        }
        is_armed_ = true;
    }
    else {        
        if (is_armed_ = true) {
            context_.lock()->logInfo("[State Disarmed] Disarm detected.");
        }
        is_armed_ = false;
    }
    if (vehicle_status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        if (is_offboard_ = false) {
            context_.lock()->logInfo("[State Disarmed] To offboard mode.");
        }
        is_offboard_ = true;
    }
    else {
        if (is_armed_ = true) {
            context_.lock()->logInfo("[State Disarmed] Out of offboard mode.");
        }
        is_offboard_ = false;
    }
    if(is_armed_ && is_offboard_) {
        context_.lock()->setState(std::make_shared<StateTakeoff>());
    }
}