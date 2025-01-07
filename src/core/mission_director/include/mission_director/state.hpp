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

    virtual void checkTransition();

    void setContext(std::weak_ptr<MissionDirector> context);

    virtual void execute();

    // data setters
    void setVehicleStatus(const VehicleStatus::SharedPtr msg);
    void setVehicleAltitude(const DistanceSensor::SharedPtr msg);
    void setVehicleLocalPosition(const VehicleLocalPosition::SharedPtr msg);
    void setVehicleLandDetected(const VehicleLandDetected::SharedPtr msg);
    void setTactileSensorPose(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    std::string getStateName() const {
        return state_name_;
    }

    std::string state_name_;


    std::weak_ptr<MissionDirector> context_; // backreference to the mission director

protected:
    VehicleStatus vehicle_status_;
    DistanceSensor vehicle_altitude_;
    VehicleLocalPosition vehicle_local_position_;
    VehicleLandDetected vehicle_land_detected_;
    geometry_msgs::msg::TwistStamped tactile_sensor_pose_;
};


#endif // STATE_HPP