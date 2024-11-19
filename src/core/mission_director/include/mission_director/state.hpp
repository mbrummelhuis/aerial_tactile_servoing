#ifndef STATE_HPP
#define STATE_HPP

// includes
#include "rclcpp/rclcpp.hpp"
#include "mission_director/state.hpp"


class State : public rclcpp::Node {
    public:
        State();

        void runState();
    
    private:
};


#endif // STATE_HPP