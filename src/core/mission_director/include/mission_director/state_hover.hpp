#ifndef STATE_HOVER_HPP
#define STATE_HOVER_HPP

#include "mission_director/state.hpp"

class StateHover : public State {
public:
    StateHover() : State() {}

    void execute() override;

    void checkTransition() override;

private:
    std::string state_name_ = "Hover";

    std::chrono::time_point<std::chrono::steady_clock> start_time_;

    const int wait_time_ = 5; // seconds
};

#endif // STATE_HOVER_HPP