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

// Limit switch input pins -- Check gpio readall
#define LIMIT_PIVOT_1 10
#define LIMIT_PIVOT_2 9
#define LIMIT_SHOULDER_1 6
#define LIMIT_SHOULDER_2 13

// Control table addresses for Feetech STS
#define ADDR_OPERATING_MODE 33 // 0: position, 1: velocity
#define ADDR_TORQUE_ENABLE 40 // 0: torque disable, 1: torque enable
#define ADDR_GOAL_POSITION 42
#define ADDR_PRESENT_POSITION 56
#define ADDR_GOAL_SPEED 46
#define ADDR_PRESENT_SPEED 58
#define ADDR_PRESENT_CURRENT 69

// Controller gains
#define ADDR_POSITION_P 21
#define ADDR_POSITION_D 22
#define ADDR_POSITION_I 23

// Protocol version
#define PROTOCOL_VERSION 1.0  // Feetech uses Dynamixel protocol v1.0

// Default setting
#define BAUDRATE 1000000  // Servo Baudrate
#define DEVICE_NAME "/dev/ttyUSB1"  // [Linux]: "/dev/ttyUSB*"

// Modes
#define POSITION_MODE 0
#define VELOCITY_MODE 1
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

// Servo IDs
#define PIVOT_1_ID 1
#define PIVOT_2_ID 11
#define SHOULDER_1_ID 2
#define SHOULDER_2_ID 12
#define ELBOW_1_ID 3
#define ELBOW_2_ID 13

class DriverFeetechServo : public rclcpp::Node
{
public:
  DriverFeetechServo();
  ~DriverFeetechServo();

  struct ServoState {
    int id;
    int position;
    int velocity;
    int current;

    ServoState(int id = 0, float position = 0.0f, float velocity = 0.0f, float current = 0.0f)
        : id(id), position(position), velocity(velocity), current(current) {}
  };

  struct ServoData {
    // unordered map allows accessing servos by ID: servo_data.servos[PIVOT_1_ID]
    std::unordered_map<int, ServoState> servos;
    ServoData() { // Set servos on servodata
      servos[PIVOT_1_ID] = ServoState(PIVOT_1_ID);
      servos[PIVOT_2_ID] = ServoState(PIVOT_2_ID);
      servos[SHOULDER_1_ID] = ServoState(SHOULDER_1_ID);
      servos[SHOULDER_2_ID] = ServoState(SHOULDER_2_ID);
      servos[ELBOW_1_ID] = ServoState(ELBOW_1_ID);
      servos[ELBOW_2_ID] = ServoState(ELBOW_2_ID);
    }
  } servo_data;

private:
  // functions
  int InitializeServos();
  int Home();

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
  void getPresentPositions();
  void getPresentVelocities();
  void getPresentCurrents();

  // general servo data setters
  int setMode(const int &mode);

  // individual servo data setters
  void setTorqueEnable(const int id, const int &enable);
  int setReference(const int id, const int &reference);

  // data
  int present_position_;
  int present_velocity_;
  int control_mode_;

  // ??
  int dxl_error = 0;
};

#endif  // DRIVER_FEETECH_SERVO_HPP_
