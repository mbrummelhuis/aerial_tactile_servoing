#ifndef MISSION_DIRECTOR_NODE_HPP
#define MISSION_DIRECTOR_NODE_HPP

// includes
#include "rclcpp/rclcpp.hpp"
#include "mission_director/state.hpp"


class MissionDirector : public rclcpp::Node {
    public:
        MissionDirector();

        void setState(std::shared_ptr<State> new_state);


        void runState();
    
    private:
        std::shared_ptr<State> current_state_;
        rclcpp::TimerBase::SharedPtr timer_;
};


#endif // MISSION_DIRECTOR_NODE_HPP