#ifndef MISSION_DIRECTOR_NODE_HPP
#define MISSION_DIRECTOR_NODE_HPP

// includes
#include "rclcpp/rclcpp.hpp"
#include "mission_director/md_ros_wrapper.hpp"
#include "mission_director/state.hpp"


class MissionDirector {
    public:
        MissionDirector();

        void SetState(std::shared_ptr<State> new_state);

        void runState();

        void execute();

        std::string getCurrentState() const;
    
    private:
        std::shared_ptr<State> current_state_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<MDROSWrapper> wrapper_;

        void setWrapper(std::shared_ptr<MDROSWrapper> wrapper);
};


#endif // MISSION_DIRECTOR_NODE_HPP