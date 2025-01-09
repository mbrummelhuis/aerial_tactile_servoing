# Use an official ROS 2 base image (e.g., Humble Hawksbill on Ubuntu 22.04)
FROM osrf/ros:humble-desktop-full-jammy

# Set the shell to bash (required for ROS 2 sourcing)
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Set environment variables
ENV ROS_DISTRO=humble

# Update and install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-argcomplete \
    bash-completion \ 
    git \
    nano \
    terminator \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Configure user
ARG USERNAME=ats-devcontainer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/ros2_ws && chown $USER_UID:$USER_GID /home/$USERNAME/ros2_ws

# Set up sudo
RUN apt-get update && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Copy the entrypoint into the container root
COPY /docker/entrypoint.sh /entrypoint.sh
COPY /docker/bashrc /home/$USERNAME/.bashrc

# Switch user from root to $USERNAME
USER $USERNAME

# Install python dependencies (for TacTip driver)
RUN pip3 install -U \
    numpy==2.2.0 \
    opencv-python==4.10.0.84 \
    scikit-image==0.24.0 \
    torch==2.5.1 \
    pytorch-model-summary==0.1.2 \
    vit-pytorch==1.8.8 \
    pandas==2.2.3 \
    matplotlib==3.9.2 \
    seaborn==0.13.2



# Change working directory to /ros2_ws
WORKDIR /ros2_ws

# Copy the workspace source code into the container
#COPY src src

# Define entrypoint bash script
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

# Set the default command to bash (delete this line if you want to run something else)
CMD ["terminator"]

# Install any ROS 2 package dependencies
#RUN apt-get update && rosdep update \
#    && rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
#RUN colcon build --symlink-install