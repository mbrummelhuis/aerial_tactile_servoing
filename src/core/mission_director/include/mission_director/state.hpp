#ifndef STATE_HPP
#define STATE_HPP

// includes
#include "rclcpp/rclcpp.hpp"
#include "mission_director/state.hpp"

class State{
public:
    State();
    virtual ~State() = default;

    virtual void runState();

    virtual std::string getStateName() const = 0;

    void setContext(std::shared_ptr<MissionDirector> context);

private:
    std::shared_ptr<MissionDirector> context_; // backreference to the mission director
};


#endif // STATE_HPP