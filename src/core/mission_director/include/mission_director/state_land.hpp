#ifndef STATE_LAND_HPP
#define STATE_LAND_HPP

#include "rclcpp/rclcpp.hpp"
#include "state.hpp"

class StateLand : public State {
public:
    StateLand() : State() {}

    void execute() override;

    void setVehicleAltitude(const DistanceSensor::SharedPtr msg) override;
    void setVehicleLocalPosition(const VehicleLocalPosition::SharedPtr msg) override;
    void setVehicleLandDetected(const VehicleLandDetected::SharedPtr msg) override;
    void checkTransition();
private:
    std::string state_name_ = "Land";

    int frequency_;

    float previous_altitude_ref_;

    float fixed_land_x_;
    float fixed_land_y_;

    float current_altitude_;
    float current_local_x_;
    float current_local_y_;

    bool is_landed_;

    const float downward_speed_ = 0.5; // m/s
    
};

#endif // STATE_LAND_HPP
