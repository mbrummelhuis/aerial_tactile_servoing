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

/*******************************************************************************
// This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
// For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
// To test this example, please follow the commands below.
//
// Open terminal #1
// $ ros2 run dynamixel_sdk_examples read_write_node
//
// Open terminal #2 (run one of below commands at a time)
// $ ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 1000}"
// $ ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 1"
//
// Author: Will Son
*******************************************************************************/

#include <cstdio>
#include <memory>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <wiringPi.h>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "driver_feetech_servo/driver_feetech_servo.hpp"

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

// Controller gains
#define ADDR_POSITION_P 21
#define ADDR_POSITION_D 22
#define ADDR_POSITION_I 23

// Protocol version
#define PROTOCOL_VERSION 1.0  // Feetech uses Dynamixel protocol v1.0

// Default setting
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB1"  // [Linux]: "/dev/ttyUSB*"

// Modes
#define POSITION_MODE 0
#define VELOCITY_MODE 1
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

// Servo IDs
#define PIVOT_1 1
#define PIVOT_2 11
#define SHOULDER_1 2
#define SHOULDER_2 12
#define ELBOW_1 3
#define ELBOW_2 13

uint8_t error_code = 0;
int comm_result = COMM_TX_FAIL;

using std::placeholders::_1;

DriverFeetechServo::DriverFeetechServo()
: Node("driver_feetech_servo")
{
  RCLCPP_INFO(this->get_logger(), "Started Feetech servo driver node");

  // QoS settings
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);
  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  // Subscriptions
  // Subscribe to topic to set mode and to topic to set reference (i.e. reference--> one messag for all servos)
  reference_servo_position_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("/servo/in/reference_position", 10, std::bind(&DriverFeetechServo::referenceServoPositionCallback, this, _1));
  reference_servo_velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("/servo/in/reference_velocity", 10, std::bind(&DriverFeetechServo::referenceServoVelocityCallback, this, _1));

  // Publishers
  // Publish relevant information i.e. servo position (calculate in this node?), velocity, torque etc.
  current_servo_position_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/servo/out/current_position", 10);
  current_servo_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/servo/out/current_velocity", 10);

  // Initialize Servos
  InitializeServos();
}

int DriverFeetechServo::Home()
{
  RCLCPP_INFO(this->get_logger(), "Homing servos");
}

/*
 * Initialize Servos
 */
int DriverFeetechServo::InitializeServos()
{
  // Set PortHandler and PacketHandler
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  comm_result = portHandler->openPort();
  if (comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("driver_feetech_servo"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("driver_feetech_servo"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port
  comm_result = portHandler->setBaudRate(BAUDRATE);
  if (comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("driver_feetech_servo"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("driver_feetech_servo"), "Succeeded to set the baudrate.");
  }

  // homing

  // Set all servos to position mode
  setMode(PIVOT_1, POSITION_MODE);
  setMode(PIVOT_2, POSITION_MODE);
  setMode(SHOULDER_1, POSITION_MODE);
  setMode(SHOULDER_2, POSITION_MODE);

  // Set all servos to torque enable
  setTorqueEnable(PIVOT_1, TORQUE_ENABLE);
  setTorqueEnable(PIVOT_2, TORQUE_ENABLE);
  setTorqueEnable(SHOULDER_1, TORQUE_ENABLE);
  setTorqueEnable(SHOULDER_2, TORQUE_ENABLE);




};

void DriverFeetechServo::getPresentPosition(const int id, int &position)
{
  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  comm_result = packetHandler->read2ByteTxRx(
    portHandler,
    (uint8_t) id,
    ADDR_PRESENT_POSITION,
    reinterpret_cast<uint16_t *>(&position),
    &error_code
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Get [ID: %d] [Present Position: %d ticks]",
    id,
    position
  );
};

void DriverFeetechServo::getPresentVelocity(const int id, int &velocity)
{
  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  comm_result = packetHandler->read2ByteTxRx(
    portHandler,
    (uint8_t) id,
    ADDR_PRESENT_SPEED,
    reinterpret_cast<uint16_t *>(&velocity),
    &error_code
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Get [ID: %d] [Present velocity: %d ticks/s]",
    id,
    velocity
  );
};

void DriverFeetechServo::getPresentCurrent(const int id, int &current)
{
  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  comm_result = packetHandler->read2ByteTxRx(
    portHandler,
    (uint8_t) id,
    ADDR_PRESENT_SPEED,
    reinterpret_cast<uint16_t *>(&current),
    &error_code
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Get [ID: %d] [Present current: %d mA]",
    id,
    current*6.5
  );
};

/* Set servo control mode
 * @param servo_id: ID of the servo
 * @param mode: 0 for position mode, 1 for velocity mode
 */
int DriverFeetechServo::setMode(const int id, const int &mode)
{
  // Set all servos to position mode
  comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    id,
    ADDR_OPERATING_MODE,
    mode,
    &error_code
  );

  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set Position Control Mode.Error code {error_code}");
    return -1;
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set control mode to {mode}.");
    control_mode_ = mode;
    return 0;
  }
};

int DriverFeetechServo::setReference(const int id, const int &reference)
{
  // Write goal position
  if (control_mode_ == POSITION_MODE)
  {
    comm_result = packetHandler->write2ByteTxRx(
      portHandler,
      id,
      ADDR_GOAL_POSITION,
      reference,
      &error_code
    );
  }
  else if (control_mode_ == VELOCITY_MODE)
  {
    comm_result = packetHandler->write2ByteTxRx(
      portHandler,
      id,
      ADDR_GOAL_SPEED,
      reference,
      &error_code
    );
  }
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set reference. Error code {error_code}");
    return -1;
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set reference.");
    return 0;
  }
};

void DriverFeetechServo::setTorqueEnable(const int id, const int &enable)
{
  // Write torque enable (length : 1 byte)
  comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    (uint8_t) id,
    ADDR_TORQUE_ENABLE,
    enable,
    &error_code
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Set [ID: %d] [Torque enable: %d]",
    id,
    enable
  );
};


int main(int argc, char * argv[])
{
  // Set up the gpio using wiringPi for limit switches
  wiringPiSetup();
  pinMode(LIMIT_PIVOT_1, INPUT);
  pinMode(LIMIT_PIVOT_2, INPUT);
  pinMode(LIMIT_SHOULDER_1, INPUT);
  pinMode(LIMIT_SHOULDER_2, INPUT);

  // Initialize ROS node
  rclcpp::init(argc, argv);
  auto driverfeetechservo = std::make_shared<DriverFeetechServo>();
  rclcpp::spin(driverfeetechservo);
  rclcpp::shutdown();

  return 0;
}
