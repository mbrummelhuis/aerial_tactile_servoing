
#include "rclcpp/rclcpp.hpp"
#include "mission_director/mission_director.hpp"

class MDROSWrapper : public rclcpp::Node {
    public:
        MDROSWrapper();
        void setMissionDirector(std::shared_ptr<MissionDirector> MD);

        void logInfo(std::string msg);
    
    private:
        int frequency_ = 100; // Hz
        std::shared_ptr<MissionDirector> MD_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

        void execute();
        void timer_callback();
        void subscriber_callback(const std_msgs::msg::String::SharedPtr msg);
};