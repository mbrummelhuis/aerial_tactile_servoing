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

void DriverFeetechServo::HomeAll()
{
  
}

void DriverFeetechServo::HomeSingleServo(const int id)
{
  if (mServoData.servo_map[id].homing_mode==LOAD_BASED)
  {
    // move arm one way 

    // take load and save

    // move servo further

    // take load again. if higher than threshold, set home and go other way

  }
  else if (mServoData.servo_map[id].homing_mode==SWITCH_BASED)
  {
    // read limit switch

    // if pressed: move arm one way until unpressed

    // if not pressed: move other way until pressed

    // set this as home position
  }
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
  mCommResult = portHandler->openPort();
  if (mCommResult == false) {
    RCLCPP_ERROR(rclcpp::get_logger("driver_feetech_servo"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("driver_feetech_servo"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port
  mCommResult = portHandler->setBaudRate(BAUDRATE);
  if (mCommResult == false) {
    RCLCPP_ERROR(rclcpp::get_logger("driver_feetech_servo"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("driver_feetech_servo"), "Succeeded to set the baudrate.");
  }

  // homing procedure

  // Set all servos to position mode
  setAllMode(POSITION_MODE);

  // Set all servos to torque enable
  setAllEnable(ENABLED);

  // Check all the limit switches
  int limit_pivot_1 = digitalRead(LIMIT_PIVOT_1);
  int limit_pivot_2 = digitalRead(LIMIT_PIVOT_2);
  int limit_shoulder_1 = digitalRead(LIMIT_SHOULDER_1);
  int limit_shoulder_2 = digitalRead(LIMIT_SHOULDER_2);

  bool homed = false;
  int position_increment = 10;
  while (homed==false)
  {
    getAllPresentPositions();
    // For the pivot and shoulder, servos are mounted mirrored
    if (limit_pivot_1==LOW) // Pivot 1 is above the limit switch
    {
      setPositionReference(PIVOT_1_ID, mServoData.servo_map[0].position+position_increment);
    }
    else
    {
      setPositionReference(PIVOT_1_ID, 0);
      homed = true;
    }

  }


};

/* Get present position for servo ID and set on servo data struct
*/
int DriverFeetechServo::getSinglePresentPosition(const int id)
{
  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  mCommResult = packetHandler->read2ByteTxRx(
    portHandler,
    (uint8_t) mServoData.servo_map[id].id,
    ADDR_PRESENT_POSITION,
    reinterpret_cast<uint16_t *>(&mServoData.servo_map[id].position),
    &mErrorCode
  );

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get present position. Error code %c", mErrorCode);
  } 
  else {
    RCLCPP_INFO(this->get_logger(), "Get [ID: %d] [Present position: %d ticks]",
    mServoData.servo_map[id].id,
    mServoData.servo_map[id].position);
  }
};

int DriverFeetechServo::getSinglePresentVelocity(const int id)
{
  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  mCommResult = packetHandler->read2ByteTxRx(
    portHandler,
    (uint8_t) mServoData.servo_map[id].id,
    ADDR_PRESENT_SPEED,
    reinterpret_cast<uint16_t *>(&mServoData.servo_map[id].velocity),
    &mErrorCode
  );

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get present velocity. Error code %c", mErrorCode);
    return -1;
  } 
  else {
    RCLCPP_INFO(this->get_logger(), "Get [ID: %d] [Present velocity: %d ticks/s]",
    mServoData.servo_map[id].id,
    mServoData.servo_map[id].velocity);
    return mServoData.servo_map[id].velocity;
  }
};

int DriverFeetechServo::getSinglePresentCurrent(const int id)
{
  // Read present current (length: 2 bytes)
  mCommResult = packetHandler->read2ByteTxRx(
    portHandler,
    (uint8_t) mServoData.servo_map[id].id,
    ADDR_PRESENT_CURRENT,
    reinterpret_cast<uint16_t *>(&mServoData.servo_map[id].current),
    &mErrorCode
  );

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get present current. Error code %c", mErrorCode);
    return -1;
  } 
  else {
    RCLCPP_INFO(this->get_logger(), "Get [ID: %d] [Present current: %d mA]",
    mServoData.servo_map[id].id,
    mServoData.servo_map[id].current*6.5);
    return mServoData.servo_map[id].current;
  }
}

void DriverFeetechServo::getAllPresentPositions()
{
  for (auto& [id, servo] : mServoData.servo_map) {  // Use non-const reference
      getSinglePresentPosition(id);
  }
};

void DriverFeetechServo::getAllPresentVelocities()
{
  for (auto& [id, servo] : mServoData.servo_map) {  // Use non-const reference
      getSinglePresentVelocity(id);
  }
};

void DriverFeetechServo::getAllPresentCurrents()
{
  for (auto& [id, servo] : mServoData.servo_map) {  // Use non-const reference
      getSinglePresentCurrent(id);
  }
};

void DriverFeetechServo::setSingleMode(const int id, const ControlMode &mode)
{
  // Set all servos to position mode
  mCommResult = packetHandler->write1ByteTxRx(
    portHandler,
    (uint8_t) mServoData.servo_map[id].id,
    ADDR_OPERATING_MODE,
    mode,
    &mErrorCode
  );

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set Position Control Mode.Error code %c", mErrorCode);
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set control mode to %d.", mode);
    mServoData.servo_map[id].control_mode = mode;
  }
}

/* Set servo control mode
 * @param servo_id: ID of the servo
 * @param mode: 0 for position mode, 1 for velocity mode
 */
void DriverFeetechServo::setAllMode(const ControlMode &mode)
{
  for (auto& [id, servo] : mServoData.servo_map) {  // Use non-const reference
    setSingleMode(id, mode);
  }
};

void DriverFeetechServo::setPositionReference(const int id, const int &reference)
{
  mCommResult = packetHandler->write2ByteTxRx(
    portHandler,
    id,
    ADDR_GOAL_POSITION,
    reference,
    &mErrorCode
  );

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set reference. Error code %c", mErrorCode);
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set reference.");
  }
};

void DriverFeetechServo::setVelocityReference(const int id, const int &reference)
{
  mCommResult = packetHandler->write2ByteTxRx(
    portHandler,
    id,
    ADDR_GOAL_SPEED,
    reference,
    &mErrorCode
  );

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set reference. Error code %c", mErrorCode);
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set reference.");
  }
}

void DriverFeetechServo::setSingleEnable(const int id, const TorqueEnable &enable)
{
  mCommResult = packetHandler->write1ByteTxRx(
    portHandler,
    (uint8_t) id,
    ADDR_TORQUE_ENABLE,
    enable,
    &mErrorCode
  );

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set torque enable. Error code %c", mErrorCode);
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set torque enable.");
  }
}

void DriverFeetechServo::setAllEnable(const TorqueEnable &enable)
{
  for (auto& [id, servo] : mServoData.servo_map) {  // Use non-const reference
    setSingleEnable(id, enable);
  }
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
