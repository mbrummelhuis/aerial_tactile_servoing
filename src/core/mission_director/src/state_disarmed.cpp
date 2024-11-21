#include <string>

#include "state_disarmed.hpp"

/* @brief Disarmed state
            Waits until the drone is armed and put in offboard mode through the remote
 */
StateDisarmed::StateDisarmed() : State() {
    // Initializer code goes here
}

void StateDisarmed::runState() {
    // Check if drone is armed and in offboard mode. If so, change state to StateTakeoff
}

void StateDisarmed::

std::string StateDisarmed::getStateName() const {
    return "Disarmed";
}
