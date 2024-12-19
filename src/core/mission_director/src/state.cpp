

#include "state.hpp"

State::State() {
    // Initializer code goes here
}

// Any general state code goes heres

void State::setVehicleStatus(const VehicleStatus::SharedPtr msg) {
    vehicle_status_ = *msg;
}

void State::setVehicleAltitude(const DistanceSensor::SharedPtr msg) {
    vehicle_altitude_ = *msg;
}

void State::setVehicleLocalPosition(const VehicleLocalPosition::SharedPtr msg) {
    vehicle_local_position_ = *msg;
}

void State::setVehicleLandDetected(const VehicleLandDetected::SharedPtr msg) {
    vehicle_land_detected_ = *msg;
}