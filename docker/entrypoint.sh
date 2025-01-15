#!/bin/bash

# Enable error signals
set -e

# Source ROS2 installation at runtime
source /opt/ros/humble/setup.bash

# Echo the argument -- the argument is the launch file
echo "Launching ROS2 from launch file: $@"

# Launch the ROS2 software stack
exec ros2 launch $@