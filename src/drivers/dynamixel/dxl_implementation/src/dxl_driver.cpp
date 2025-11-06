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
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "sensor_msgs/msg/joint_state.hpp"

#include "dxl_driver.hpp"

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

DXLDriver::DXLDriver()
: Node("dxl_driver")
{
    // Declare all parameters
    // Driver parameters
    this->declare_parameter("driver.port_name", "/dev/ttyUSB0");
    this->declare_parameter("driver.baud_rate", 115200);
    this->declare_parameter("driver.frequency", 100.);
    this->declare_parameter("driver.protocol_version", 2.0);

    // Servo parameters
    this->declare_parameter("servos.ids", std::vector<int>{1});
    this->declare_parameter("servos.operating_modes", std::vector<int>{4});
    this->declare_parameter("servos.homing_modes", std::vector<int>{0});
    this->declare_parameter("servos.directions", std::vector<int>{1});
    this->declare_parameter("servos.max_speeds", std::vector<double>{0.1});
    this->declare_parameter("servos.gear_ratios", std::vector<double>{1.0});
    this->declare_parameter("servos.start_offsets", std::vector<double>{0.0});

    // Generate uint8_t vector of ids
    std::vector<long> int_ids = this->get_parameter("servos.ids").as_integer_array();
    ids_.resize(int_ids.size());
    std::transform(int_ids.begin(), int_ids.end(), ids_.begin(),
                    [](int val) { return static_cast<uint8_t>(val); });
    num_servos = int_ids.size();
    check_parameter_sizes(num_servos); // Check if param sizes match



    settings_.port = this->get_parameter("driver.port_name").as_string();
    settings_.baud_rate = this->get_parameter("driver.baud_rate").as_int();
    settings_.frequency = this->get_parameter("driver.frequency").as_double();
    settings_.protocol_version = this->get_parameter("driver.protocol_version").as_double();

    // ROS2 servo interfaces
    sub_servo_reference = this->create_subscription<sensor_msgs::msg::JointState>(
      "/servo/in/state", 10, 
      std::bind(&DXLDriver::servoReferenceCallback, this, std::placeholders::_1));
    pub_servo_state = this->create_publisher<sensor_msgs::msg::JointState>("/servo/out/state", 10);
    
    // Initialize Dynamixels
    portHandler = dynamixel::PortHandler::getPortHandler(settings_.port.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(settings_.protocol_version);
    
    // Initialize GroupSyncRead and GroupSyncWrite objects
    gsrPosition = dynamixel::GroupSyncRead(portHandler, packetHandler, DXLREGISTER::PRESENT_POSITION, num_servos_);
    gsrVelocity = dynamixel::GroupSyncRead(portHandler, packetHandler, DXLREGISTER::PRESENT_VELOCITY, num_servos_);
    gsrCurrent = dynamixel::GroupSyncRead(portHandler, packetHandler, DXLREGISTER::PRESENT_CURRENT, num_servos_);
    gsrPWM = dynamixel::GroupSyncRead(portHandler, packetHandler, DXLREGISTER::PRESENT_PWM, num_servos_);

    gswPosition = dynamixel::GroupSyncWrite(portHandler, packetHandler, DXLREGISTER::GOAL_POSITION, num_servos_);
    gswVelocity = dynamixel::GroupSyncWrite(portHandler, packetHandler, DXLREGISTER::GOAL_VELOCITY, num_servos_);

    // Open Serial Port
    dxl_comm_result = portHandler->openPort();
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxl_comm_result = portHandler->setBaudRate(settings_.baud_rate);
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
    }

    setupDynamixel(BROADCAST_ID);

    RCLCPP_INFO(this->get_logger(), "Dynamixel driver node initialized");    

    // Timer for publishing servo state
    double node_frequency_ = this->get_parameter("driver.frequency").as_double();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000./node_frequency_)), std::bind(&DXLDriver::loop, this));
}

DXLDriver::~DXLDriver()
{
    // Disable Torque of DYNAMIXEL
    packetHandler->write1ByteTxRx(
        portHandler,
        BROADCAST_ID,
        DXLREGISTER::TORQUE_ENABLE,
        0,
        &dxl_error
    );
}

void DXLDriver::loop()
{
    set_all_position_references(); // set servo references on servo
    get_all_servo_data(); // get data from the servos
    publishAllServoData(); // publish data to ROS topic
}

void DXLDriver::set_all_position_references()
{
    for(int i = 0; i++; i<num_servos_)
    {
        int dxl_addparam_result = false;
        uint8_t param_goal_position[4];

        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(servodata_[i].goal_position));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(servodata_[i].goal_position));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(servodata_[i].goal_position));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(servodata_[i].goal_position));
        dxl_addparam_result = gswPosition.addParam(ids_[i], param_goal_position);
        if (dxl_addparam_result != true) {
            RCLCPP_ERROR(this->get_logger(), "Failed to addparam to groupSyncWrite for Dynamixel ID %d", servodata_[i].id);
        }
    }

    dxl_comm_result = gswPosition.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
    }
    gswPosition.clearParam(); 
}

void DXLDriver::get_all_servo_data()
{
    int dxl_addparam_result = false;
    int32_t present_position;
    for(int i =0; i++; i<num_servos_)
    {
        dxl_addparam_result = gsrPosition.addParam(ids_[i]);
        if (dxl_addparam_result != true) {
            RCLCPP_ERROR(this->get_logger(), "Failed to addparam to groupSyncWrite for Dynamixel ID %d", servodata_[i].id);
        }
    }

    dxl_comm_result = gsrPosition.txRxPacket();
    if (dxl_comm_result == COMM_SUCCESS)
    {
        for (int i=0; i++; i<num_servos_)
        {
            servodata_[i].present_position = int2rad(gsrPosition.getData(ids_[i], DXLREGISTER::PRESENT_POSITION, 4));
        }
    }

}

void DXLDriver::

void DXLDriver::servoReferenceCallback()
{

}

void DXLDriver::setupDynamixel(uint8_t dxl_id)
{
    dxl_comm_result = portHandler->openPort();
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxl_comm_result = portHandler->setBaudRate(settings_.baud_rate);
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
    }

  // Use Position Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    DXLREGISTER::OPERATING_MODE,
    3,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Position Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    DXLREGISTER::TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
  }
}

void DXLDriver::check_parameter_sizes(size_t num_servos) const
{
    if (this->get_parameter("servos.operating_modes").as_integer_array().size() != num_servos)
    {
        RCLCPP_ERROR(this->get_logger(), "servos.operating_modes not the same size as number of servos!");
        exit(-1);
    }
    if (this->get_parameter("servos.homing_modes").as_integer_array().size() != num_servos)
    {
        RCLCPP_ERROR(this->get_logger(), "servos.homing_modes not the same size as number of servos!");
        exit(-1);
    }
    if (this->get_parameter("servos.directions").as_integer_array().size() != num_servos)
    {
        RCLCPP_ERROR(this->get_logger(), "servos.directions not the same size as number of servos!");
        exit(-1);
    }
    if (this->get_parameter("servos.max_speeds").as_double_array().size() != num_servos)
    {
        RCLCPP_ERROR(this->get_logger(), "servos.max_speeds not the same size as number of servos!");
        exit(-1);
    }
    if (this->get_parameter("servos.max_currents").as_double_array().size() != num_servos)
    {
        RCLCPP_ERROR(this->get_logger(), "servos.max_currents not the same size as number of servos!");
        exit(-1);
    }
    if (this->get_parameter("servos.gear_ratios").as_double_array().size() != num_servos)
    {
        RCLCPP_ERROR(this->get_logger(), "servos.gear_ratios not the same size as number of servos!");
        exit(-1);
    }
    if (this->get_parameter("servos.start_offsets").as_double_array().size() != num_servos)
    {
        RCLCPP_ERROR(this->get_logger(), "servos.start_offsets not the same size as number of servos!");
        exit(-1);
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto dxl_driver = std::make_shared<DXLDriver>();
  rclcpp::spin(dxl_driver);
  rclcpp::shutdown();

  return 0;
}
