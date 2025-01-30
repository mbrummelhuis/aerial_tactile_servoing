// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DRIVER_FEETECH_SERVO_HPP_
#define DRIVER_FEETECH_SERVO_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"


class DriverFeetechServo : public rclcpp::Node
{
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;

  DriverFeetechServo();
  virtual ~DriverFeetechServo();

private:
  // functions
  int InitializeServos();
  int Home();

  // services
  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr current_servo_position_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr current_servo_velocity_publisher_;

  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr reference_servo_position_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr reference_servo_velocity_subscriber_;

  // callbacks
  void referenceServoPositionCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void referenceServoVelocityCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  // Handlers
  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;

  // servo data getters
  void getPresentPosition(const int id, int &position);
  void getPresentVelocity(const int id, int &velocity);
  void getPresentCurrent(const int id, int &current);

  // servo data setters
  void setTorqueEnable(const int id, const int &enable);
  int setMode(const int id, const int &mode);
  int setReference(const int id, const int &reference);

  // data
  int present_position_;
  int present_velocity_;
  int control_mode_;

  // ??
  int dxl_error = 0;
};

#endif  // DRIVER_FEETECH_SERVO_HPP_
