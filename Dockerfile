# Use an official ROS 2 base image (e.g., Humble Hawksbill on Ubuntu 22.04)
FROM ros:humble-ros-base-jammy

# Set environment variables
ENV ROS_DISTRO humble

# Update and install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace directory in the container
WORKDIR /ros2_ws

# Copy the workspace source code into the container
COPY src ./src

# Install any ROS 2 package dependencies
RUN apt-get update && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN colcon build