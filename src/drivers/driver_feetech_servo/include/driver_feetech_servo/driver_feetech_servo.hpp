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
#define ADDR_VELOCITY_P 37
#define ADDR_VELOCITY_I 39

// Protocol version
#define PROTOCOL_VERSION 1.0  // Feetech uses Dynamixel protocol v1.0

// Default setting
#define BAUDRATE 1000000  // Servo Baudrate
#define DEVICE_NAME "/dev/ttyUSB1"  // [Linux]: "/dev/ttyUSB*"

// Modes
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

// Servo IDs
#define PIVOT_1_ID 1
#define PIVOT_2_ID 11
#define SHOULDER_1_ID 2
#define SHOULDER_2_ID 12
#define ELBOW_1_ID 3
#define ELBOW_2_ID 13

enum ControlMode {
  POSITION_MODE = 0,
  VELOCITY_MODE = 1
};

enum HomingMode {
  LOAD_BASED = 0,   // Home based on load increase
  SWITCH_BASED = 1  // Home based on limit switch
};

enum TorqueEnable {
  ENABLED = 1,
  DISABLED = 0
};

class DriverFeetechServo : public rclcpp::Node
{
public:
  DriverFeetechServo();
  ~DriverFeetechServo();

  // Define struct for saving state of single servo
  struct ServoState {
    int id;
    int position;
    int velocity;
    int current;
    HomingMode homing_mode;
    ControlMode control_mode;

    ServoState(
      int id = 0, 
      float position = 0.0f, 
      float velocity = 0.0f, 
      float current = 0.0f, 
      HomingMode homing_mode = LOAD_BASED, 
      ControlMode control_mode = POSITION_MODE)
        : id(id), 
        position(position), 
        velocity(velocity), 
        current(current),
        homing_mode(LOAD_BASED),
        control_mode(POSITION_MODE) {}
  };

  // Define struct for saving state of all servos
  struct ServoData {
    // unordered map allows accessing servos by ID: servo_data.servos[PIVOT_1_ID]
    std::unordered_map<int, ServoState> servo_map;

    ServoData() {}
    void AddServo(ServoState servo) {
      servo_map[servo.id] = servo;
    }
    
  } mServoData;

private:
  // functions
  int InitializeServos();
  void HomeSingleServo(const int id);
  void HomeAll();

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
  int getSinglePresentPosition(const int id);
  int getSinglePresentVelocity(const int id);
  int getSinglePresentCurrent(const int id);
  void getAllPresentPositions();
  void getAllPresentVelocities();
  void getAllPresentCurrents();

  // general servo data setters
  void setSingleMode(const int id, const ControlMode &mode);
  void setAllMode(const ControlMode &mode);

  // individual servo data setters
  void setSingleEnable(const int id, const TorqueEnable &enable);
  void setAllEnable(const TorqueEnable &enable);
  void setPositionReference(const int id, const int &reference);
  void setVelocityReference(const int id, const int &reference);

  // ??
  uint8_t mErrorCode;
  int mCommResult;
};

#endif  // DRIVER_FEETECH_SERVO_HPP_
