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

#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include <sensor_msgs/msg/joint_state.hpp>

#define DEVICE_NAME "/dev/ttyUSB0"
#define BAUDRATE 57600
#define PROTOCOL_VERSION 2.0

namespace DXLREGISTER
{
    uint8_t const MODEL_NUMBER      = 0x00; // Size: 2 bytes
    uint8_t const ID                = 0x02; // Size: 4 bytes
    uint8_t const BAUD_RATE         = 0x08; // Size; 1 byte
    uint8_t const RETURN_DELAY      = 0x09; // Size: 1 byte
    uint8_t const DRIVE_MODE        = 0x0A; // Size: 1 byte
    uint8_t const OPERATING_MODE    = 0x0B; // Size; 1 byte
    uint8_t const SHADOW_ID         = 0x0C; // Size: 1 byte
    uint8_t const HOMING_OFFSET     = 0x14; // Size: 4 bytes
    uint8_t const MOVING_THRESHOLD  = 0x18; // Size: 4 bytes
    uint8_t const CURRENT_LIMIT     = 0x26; // Size: 2 bytes
    uint8_t const VELOCITY_LIMIT    = 0x2C; // Size: 4 bytes
    uint8_t const MAX_POSITION      = 0x30; // Size: 4 bytes
    uint8_t const MIN_POSITION      = 0x34; // Size: 4 bytes
    uint8_t const TORQUE_ENABLE     = 0x40; // Size: 1 byte
    uint8_t const LED               = 0x41; // Size: 1 byte
    uint8_t const HARDWARE_ERROR    = 0x46; // Size: 1 byte
    uint8_t const GOAL_PWM          = 0x64; // Size: 2 bytes
    uint8_t const GOAL_CURRENT      = 0x66; // Size: 2 bytes
    uint8_t const GOAL_VELOCITY     = 0x68; // Size: 4 bytes
    uint8_t const GOAL_POSITION     = 0x74; // Size: 4 bytes
    uint8_t const PRESENT_PWM       = 0x7C; // Size: 2 bytes, read-only
    uint8_t const PRESENT_CURRENT   = 0x7E; // Size: 2 bytes, read-only
    uint8_t const PRESENT_VELOCITY  = 0x80; // Size: 4 bytes, read-only
    uint8_t const PRESENT_POSITION  = 0x84; // Size: 4 bytes, read-only
};

enum DXLMode{
    CURRENT             = 0, // Current control mode
    VELOCITY            = 1, // Velocity control mode
    POSITION            = 3, // Joint control mode, position within limits
    EXTENDED_POSITION   = 4, // Multi-turn position control mode
    POSITION_CURRENT    = 5, // Position/current control mode
    PWM                 = 16 // PWM control mode
};

enum UNIT
{
    COUNTS = 0,
    RAD = 1,
    DEG = 2
};

struct ServoData
{
    uint8_t id;
    double gear_ratio;
    double max_velocity; // In ticks/s
    int direction;
    DXLMode operating_mode;
    float goal_position; // In ticks
    float goal_velocity; // In ticks/s
    double present_position;
    double present_velocity;
    double present_current;
    uint32_t present_pwm;
    int16_t home_position; // In ticks at the servo horn
};


class DXLDriver : public rclcpp::Node
{
public:

    DXLDriver(dynamixel::GroupSyncRead *positionReader, dynamixel::GroupSyncRead *velocityReader,
                dynamixel::GroupSyncRead *currentReader, dynamixel::GroupSyncRead *PWMReader, 
                dynamixel::GroupSyncWrite *positionWriter, dynamixel::GroupSyncWrite *velocityWriter);
    virtual ~DXLDriver();

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_servo_reference;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_servo_state;

    dynamixel::GroupSyncRead *gsrPosition;
    dynamixel::GroupSyncRead *gsrVelocity;
    dynamixel::GroupSyncRead *gsrCurrent;
    dynamixel::GroupSyncRead *gsrPWM;

    dynamixel::GroupSyncWrite *gswPosition;
    dynamixel::GroupSyncWrite *gswVelocity;

    /// @brief Callback for the timer, execute interface loop
    void loop();
    void set_all_position_references();
    void get_all_servo_data();
    void servo_reference_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publish_all_servo_data();
    //void setup_port();
    void setup_dynamixel(uint8_t dxl_id);
    double pos_int2rad(uint8_t id, uint32_t position_ticks); // return the rad position with gear ratio and direction
    uint32_t pos_rad2int(uint8_t id, double position_rads);
    double vel_int2rad(uint8_t id, uint32_t velocity_ticks);
    double cur_int2amp(uint32_t current_ticks);

    void get_present_positions();
    void get_present_velocities();
    void get_present_currents();
    void get_present_pwms();

    void set_home_position_at_current_position();
    void enable_torque(int8_t torque_enable);

    void check_parameter_sizes(size_t num_servos) const;

    // Data
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<uint8_t> ids_;
    std::unordered_map<int, size_t> id2index_; 
    int num_servos_;
    std::vector<ServoData> servodata_;


};

#endif  // READ_WRITE_NODE_HPP_
