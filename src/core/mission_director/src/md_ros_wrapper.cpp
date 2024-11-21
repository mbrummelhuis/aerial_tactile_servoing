
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "mission_director/md_ros_wrapper.hpp"
#include "mission_director/mission_director.hpp"

MDROSWrapper::MDROSWrapper() : Node("mission_director") {
    setMissionDirector(std::make_shared<MissionDirector>());

    // publishers
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    // subscribers
    subscriber_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&MDROSWrapper::subscriber_callback, this, std::placeholders::_1));
    
    // create timer 100 Hz -> 10 ms
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / frequency_), std::bind(&MDROSWrapper::timer_callback, this));
}

/* @brief Log a message to the console with RCL logger
 */
void MDROSWrapper::logInfo(std::string msg) {
    RCLCPP_INFO(this->get_logger(),msg.c_str());
}

void MDROSWrapper::setMissionDirector(std::shared_ptr<MissionDirector> MD) {
    RCLCPP_INFO(this->get_logger(), "Setting Mission Director on ROS2 wrapper");
    MD_ = MD;
}

void MDROSWrapper::timer_callback() {
    MD_->execute(); // Execute the mission director
}

void MDROSWrapper::subscriber_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDROSWrapper>());
  rclcpp::shutdown();
  return 0;
}