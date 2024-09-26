# Use an official ROS 2 base image (e.g., Humble Hawksbill on Ubuntu 22.04)
FROM osrf/ros:humble-desktop-full-jammy

# Set environment variables
ENV ROS_DISTRO=humble

# Update and install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    git \
    nano \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Copy the entrypoint into the container root
COPY /docker/entrypoint.sh /entrypoint.sh

# Set up the workspace directory in the container
WORKDIR /ros2_ws

# Copy the workspace source code into the container
COPY src ./src

# Install any ROS 2 package dependencies
RUN apt-get update && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN colcon build

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

CMD ["bash"]