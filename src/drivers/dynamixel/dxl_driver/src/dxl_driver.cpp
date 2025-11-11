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
// $ ros2 run dynamixel_sdk_examples dxl_driver
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

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "dxl_driver.hpp"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

// dynamixel::GroupSyncRead *gsrPosition;
// dynamixel::GroupSyncRead *gsrVelocity;
// dynamixel::GroupSyncRead *gsrCurrent;
// dynamixel::GroupSyncRead *gsrPWM;


uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

DXLDriver::DXLDriver(dynamixel::GroupSyncRead *positionReader, dynamixel::GroupSyncRead *velocityReader,
    dynamixel::GroupSyncRead *currentReader, dynamixel::GroupSyncRead *PWMReader, 
    dynamixel::GroupSyncWrite *positionWriter, dynamixel::GroupSyncWrite *velocityWriter)
: Node("dxl_driver")
{
    gsrPosition = positionReader;
    gsrVelocity = velocityReader;
    gsrCurrent = currentReader;
    gsrPWM = PWMReader;

    gswPosition = positionWriter;
    gswVelocity = velocityWriter;

    // Declare all parameters
    this->declare_parameter("driver.frequency", 1.);
    // Servo parameters
    this->declare_parameter("servos.ids", std::vector<int>{1});
    this->declare_parameter("servos.operating_modes", std::vector<int>{4});
    this->declare_parameter("servos.homing_modes", std::vector<int>{0});
    this->declare_parameter("servos.directions", std::vector<int>{1});
    this->declare_parameter("servos.max_speeds", std::vector<double>{0.1});
    this->declare_parameter("servos.gear_ratios", std::vector<double>{1.0});

    // Generate uint8_t vector of ids
    std::vector<long> int_ids = this->get_parameter("servos.ids").as_integer_array();
    ids_.resize(int_ids.size());
    std::transform(int_ids.begin(), int_ids.end(), ids_.begin(),
                    [](int val) { return static_cast<uint8_t>(val); });
    num_servos_ = static_cast<int>(int_ids.size());
    check_parameter_sizes(num_servos_); // Check if param sizes match
    std::vector<int64_t> operating_modes = this->get_parameter("servos.operating_modes").as_integer_array();
    std::vector<int64_t> directions = this->get_parameter("servos.directions").as_integer_array();
    std::vector<double> gear_ratios = this->get_parameter("servos.gear_ratios").as_double_array();
    std::vector<int64_t> homing_modes = this->get_parameter("servos.homing_modes").as_integer_array();
    std::vector<double> max_speeds = this->get_parameter("servos.max_speeds").as_double_array();
    for (int i = 0; i < num_servos_; i++)
    {
        id2index_[ids_[i]] = i;

        ServoData current_servo_data;

        current_servo_data.operating_mode = static_cast<DXLMode>(operating_modes[i]);
        current_servo_data.direction = static_cast<int>(directions[i]);
        current_servo_data.gear_ratio = gear_ratios[i];
        current_servo_data.goal_position = current_servo_data.present_position;
        current_servo_data.goal_velocity = current_servo_data.present_velocity;
        current_servo_data.max_velocity = max_speeds[i];
        write_max_velocities();
        if (homing_modes[i] == 0) // Don't update homing
        {

        }
        else if (homing_modes[i] == 1) // Set current
        {

        }
        // ...
        // Assign servo settings here
        servodata_.push_back(current_servo_data);

        int dxl_addparam_result = false;
        dxl_addparam_result = gsrPosition->addParam(ids_[i]);
        if (dxl_addparam_result != true) {
            RCLCPP_ERROR(this->get_logger(), "Failed to addparam to groupSyncRead Position for Dynamixel ID %d", servodata_[i].id);
        }
        dxl_addparam_result = gsrVelocity->addParam(ids_[i]);
        if (dxl_addparam_result != true) {
            RCLCPP_ERROR(this->get_logger(), "Failed to addparam to groupSyncRead Velocit for Dynamixel ID %d", servodata_[i].id);
        }
        dxl_addparam_result = gsrCurrent->addParam(ids_[i]);
        if (dxl_addparam_result != true) {
            RCLCPP_ERROR(this->get_logger(), "Failed to addparam to groupSyncRead Current for Dynamixel ID %d", servodata_[i].id);
        }
        dxl_addparam_result = gsrPWM->addParam(ids_[i]);
        if (dxl_addparam_result != true) {
            RCLCPP_ERROR(this->get_logger(), "Failed to addparam to groupSyncRead PWMS for Dynamixel ID %d", servodata_[i].id);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Succeeded assigning servodata");

    // ROS2 servo interfaces
    sub_servo_reference = this->create_subscription<sensor_msgs::msg::JointState>(
      "/servo/in/state", 10, 
      std::bind(&DXLDriver::servo_reference_callback, this, std::placeholders::_1));
    pub_servo_state = this->create_publisher<sensor_msgs::msg::JointState>("/servo/out/state", 10);
    RCLCPP_INFO(this->get_logger(), "Succeeded creating ROS2 interfaces");

    // Read current positions and set as reference
    get_all_servo_data();
    

    // Set up the servos and engage torque
    setup_dynamixel(BROADCAST_ID); // 
    RCLCPP_INFO(this->get_logger(), "Succeeded Configuring driver");

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
    publish_all_servo_data(); // publish data to ROS topic
}

void DXLDriver::set_all_position_references()
{
    for(int i = 0; i<num_servos_; i++)
    {
        int dxl_addparam_result = false;
        uint8_t param_goal_position[4];
        int32_t goal_pos_ticks = pos_rad2int(servodata_[i].id, servodata_[i].goal_position);
        RCLCPP_INFO(this->get_logger(), "goal position %f", servodata_[i].goal_position);
        RCLCPP_INFO(this->get_logger(), "goal position ticks %i", goal_pos_ticks);

        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_pos_ticks));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_pos_ticks));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_pos_ticks));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_pos_ticks));
        dxl_addparam_result = gswPosition->addParam(ids_[i], param_goal_position);
        if (dxl_addparam_result != true) {
            RCLCPP_ERROR(this->get_logger(), "Failed to addparam to groupSyncWrite for Dynamixel ID %d with error code %i", servodata_[i].id, dxl_addparam_result);
        }
    }

    dxl_comm_result = gswPosition->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
    }
    gswPosition->clearParam(); 
}

void DXLDriver::get_all_servo_data()
{
    get_present_positions();
    get_present_velocities();
    get_present_currents();
    get_present_pwms();
}

void DXLDriver::publish_all_servo_data()
{
    std::vector<double> present_positions(num_servos_);
    std::vector<double> present_velocities(num_servos_);
    std::vector<double> present_currents(num_servos_);
    std::vector<std::string> names(num_servos_);
    for (int i=0; i<num_servos_; i++)
    {
        present_positions[i] = servodata_[i].present_position;
        present_velocities[i] = servodata_[i].present_velocity;
        present_currents[i] = servodata_[i].present_current;
        names[i] = "q" + std::to_string(i);
    }

    auto servo_state_msg = sensor_msgs::msg::JointState();
    servo_state_msg.header.stamp = this->get_clock()->now();
    servo_state_msg.name = names;
    servo_state_msg.position = present_positions; // In rad at output shaft
    servo_state_msg.velocity = present_velocities; // In rad/s at output shaft
    servo_state_msg.effort = present_currents; // In mA
    this->pub_servo_state->publish(servo_state_msg);
}

void DXLDriver::get_present_positions()
{
    dxl_comm_result = gsrPosition->txRxPacket();
    if (dxl_comm_result == COMM_SUCCESS)
    {
        for (int i=0; i<num_servos_; i++)
        {
            servodata_[i].present_position = pos_int2rad(servodata_[i].id, gsrPosition->getData(ids_[i], DXLREGISTER::PRESENT_POSITION, 4));
        }
    }
    gsrPosition->clearParam();
}

void DXLDriver::get_present_velocities()
{
    dxl_comm_result = gsrVelocity->txRxPacket();
    if (dxl_comm_result == COMM_SUCCESS)
    {
        for (int i=0; i<num_servos_; i++)
        {
            servodata_[i].present_velocity = vel_int2rad(servodata_[i].id, gsrVelocity->getData(ids_[i], DXLREGISTER::PRESENT_VELOCITY, 4));
        }
    }
    gsrVelocity->clearParam();
}

void DXLDriver::get_present_currents()
{
    dxl_comm_result = gsrCurrent->txRxPacket();
    if (dxl_comm_result == COMM_SUCCESS)
    {
        for (int i=0; i<num_servos_; i++)
        {
            servodata_[i].present_current = cur_int2amp(gsrCurrent->getData(ids_[i], DXLREGISTER::PRESENT_CURRENT, 4));
        }
    }
    gsrCurrent->clearParam();
}

void DXLDriver::get_present_pwms()
{
    dxl_comm_result = gsrPWM->txRxPacket();
    if (dxl_comm_result == COMM_SUCCESS)
    {
        for (int i=0; i<num_servos_; i++)
        {
            servodata_[i].present_pwm = gsrPWM->getData(ids_[i], DXLREGISTER::PRESENT_PWM, 4);
        }
    }
}

void DXLDriver::write_max_velocities()
{
    for (int i = 0; i < num_servos_; i++)
    {
        // Enable Torque of DYNAMIXEL
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            servodata_[i].id,
            DXLREGISTER::TORQUE_ENABLE,
            vel_rad2int(servodata_[i].id, servodata_[i].max_velocity),
            &dxl_error
        );
        if (dxl_comm_result == COMM_SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "[ID: %i] Set max velocity %f rad/s | %d ticks", 
                servodata_[i].id, 
                servodata_[i].max_velocity, 
                vel_rad2int(servodata_[i].id, servodata_[i].max_velocity));
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "[ID: %i] Failed to set max velocity, result %i, error %i", servodata_[i].id, dxl_comm_result, dxl_error);
        }
    }
}
void DXLDriver::set_home_position_at_current_position()
{
    // Update servo current positions
    int32_t homing_offset;
    get_present_positions();

    for (int i = 0; i < num_servos_; i++)
    {
        // Read current homing offset
        dxl_comm_result = packetHandler->read4ByteTxRx(
            portHandler,
            servodata_[i].id,
            DXLREGISTER::HOMING_OFFSET,
            reinterpret_cast<uint32_t *>(&homing_offset),
            &dxl_error
        );
        if (dxl_comm_result!=COMM_SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "[ID: %i] Read error, comm result %i, error %i", servodata_[i].id, dxl_comm_result, dxl_error);
        }
        // Update homing offset with current position
        int32_t new_homing_offset = pos_rad2int(servodata_[i].id, servodata_[i].present_position) - homing_offset;
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            servodata_[i].id,
            DXLREGISTER::HOMING_OFFSET,
            static_cast<uint32_t>(new_homing_offset),
            &dxl_error
        );
        if (dxl_comm_result == COMM_SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "[ID: %i] Set homing offset to %d", servodata_[i].id, new_homing_offset);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "[ID: %i] Failed to set homing offset, result %i, error %i", servodata_[i].id, dxl_comm_result, dxl_error);
        }
    }
}

void DXLDriver::write_torque_enable(int8_t torque_enable)
{
    for (int i = 0; i < num_servos_; i++)
    {
        // Enable Torque of DYNAMIXEL
        dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler,
            servodata_[i].id,
            DXLREGISTER::TORQUE_ENABLE,
            torque_enable,
            &dxl_error
        );
        if (dxl_comm_result == COMM_SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "[ID: %i] Set torque enable %i", servodata_[i].id, torque_enable);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "[ID: %i] Failed to set torque enable, result %i, error %i", servodata_[i].id, dxl_comm_result, dxl_error);
        }
    }
}

double DXLDriver::pos_int2rad(uint8_t id, uint32_t position_ticks)
{
    int i = id2index_[id];
    double position_rads = static_cast<double>(servodata_[i].direction) * static_cast<double>(position_ticks) * servodata_[i].gear_ratio / 4096.;
    return position_rads;
}

int32_t DXLDriver::pos_rad2int(uint8_t id, double position_rads)
{
    int i = id2index_[id];
    int32_t position_ticks = static_cast<int32_t>(servodata_[i].direction) * static_cast<int32_t>(position_rads / servodata_[i].gear_ratio* 4096);
    return position_ticks;
}

double DXLDriver::vel_int2rad(uint8_t id, uint32_t velocity_ticks)
{
    return static_cast<double>(velocity_ticks * RAD_PER_SECOND_PER_TICK / servodata_[id2index_[id]].gear_ratio);
}

uint32_t DXLDriver::vel_rad2int(uint8_t id, double velocity_rads)
{
    return static_cast<uint32_t>(velocity_rads / RAD_PER_SECOND_PER_TICK * servodata_[id2index_[id]].gear_ratio);
}

double DXLDriver::cur_int2amp(uint16_t current_ticks)
{
    return static_cast<double>(current_ticks*MA_PER_TICK);
}

void DXLDriver::servo_reference_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (int i = 0; i < num_servos_; i++)
    {
        servodata_[i].goal_position = msg->position[i];
    }
}

void DXLDriver::setup_dynamixel(uint8_t dxl_id)
{
    // Use Position Control Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        DXLREGISTER::OPERATING_MODE,
        4,
        &dxl_error
    );
    RCLCPP_INFO(this->get_logger(), "Error %i", dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("dxl_driver"), "Failed to set Position Control Mode.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("dxl_driver"), "Succeeded to set Position Control Mode.");
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
        RCLCPP_ERROR(rclcpp::get_logger("dxl_driver"), "Failed to enable torque.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("dxl_driver"), "Succeeded to enable torque.");
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
    if (this->get_parameter("servos.gear_ratios").as_double_array().size() != num_servos)
    {
        RCLCPP_ERROR(this->get_logger(), "servos.gear_ratios not the same size as number of servos!");
        exit(-1);
    }
}

void setup_port()
{
    dxl_comm_result = portHandler->openPort();
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("dxl_driver"), "Failed to open the port!");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("dxl_driver"), "Succeeded to open the port.");
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("dxl_driver"), "Failed to set the baudrate!");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("dxl_driver"), "Succeeded to set the baudrate.");
    } 
}

int main(int argc, char * argv[])
{
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    dynamixel::GroupSyncRead gsrPosition(portHandler, packetHandler, DXLREGISTER::PRESENT_POSITION, 4);
    dynamixel::GroupSyncRead gsrVelocity(portHandler, packetHandler, DXLREGISTER::PRESENT_VELOCITY, 4);
    dynamixel::GroupSyncRead gsrCurrent(portHandler, packetHandler, DXLREGISTER::PRESENT_CURRENT, 2);
    dynamixel::GroupSyncRead gsrPWM(portHandler, packetHandler, DXLREGISTER::PRESENT_PWM, 2);
    
    dynamixel::GroupSyncWrite gswPosition(portHandler, packetHandler, DXLREGISTER::GOAL_POSITION, 4);
    dynamixel::GroupSyncWrite gswVelocity(portHandler, packetHandler, DXLREGISTER::GOAL_VELOCITY, 4);

    setup_port();

    rclcpp::init(argc, argv);

    auto dxl_driver = std::make_shared<DXLDriver>(
        &gsrPosition,
        &gsrVelocity,
        &gsrCurrent,
        &gsrPWM,
        &gswPosition,
        &gswVelocity);
    rclcpp::spin(dxl_driver);
    rclcpp::shutdown();

    return 0;
}
