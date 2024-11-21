#ifndef STATE_HPP
#define STATE_HPP

// includes
#include "rclcpp/rclcpp.hpp"

#include "mission_director/mission_director.hpp"
#include "mission_director/state.hpp"

class State{
public:
    State();
    virtual ~State() = default;

    virtual void transition();

    void setContext(std::weak_ptr<MissionDirector> context);

    virtual void execute();

    // data setters
    virtual void setVehicleStatus(const VehicleStatus::SharedPtr msg);
    virtual void setVehicleAltitude(const DistanceSensor::SharedPtr msg);
    virtual void setVehicleLocalPosition(const VehicleLocalPosition::SharedPtr msg);
    virtual void setVehicleLandDetected(const VehicleLandDetected::SharedPtr msg);

    std::string getStateName() const {
        return state_name_;
    }

    std::string state_name_;


    std::weak_ptr<MissionDirector> context_; // backreference to the mission director
};


#endif // STATE_HPP