# Aerial Tactile Servoing
Repository containing everything needed to run the companion computer component of the Aerial Tactile Servoing project.

### Installation
1. Clone this repository where you want your ROS2 workspace. The root of the repository is also the workspace root. Also update the submodules.
```
git clone https://github.com/mbrummelhuis/aerial_tactile_servoing.git
git submodule update --init --recursive
```
2. Make a virtual environment in this and install all the requirements
```
python3 -m venv venv-ats
source venv-ats/activate
pip install -r requirements.txt
```
3. Build
```
colcon build
source install/setup.bash
```

### Running
1. Run the package stack via the launch file
```
ros2 launch ats_bringup your-launch-file.launch.py
```

### Docker instructions
The aim is that we can deploy the repository by installing docker, pulling the image from Dockerhub and then run the container, pulling the src from this repo into a ROS2 workspace, and finally colcon building.

To run the container, follow the following steps. First, install Docker and Docker compose on the machine.
```
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Install docker
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
Clone this repository recursively (needed if you want to build natively, for the [px4_msgs](https://github.com/PX4/px4_msgs) and [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK), which are part of the ROS2 package stack).
```
git clone --recursive https://github.com/mbrummelhuis/aerial_tactile_servoing.git
```
Navigate to the docker folder and compose.
```
cd aerial_tactile_servoing/docker
docker compose up
```
This will pull the built Docker image from Dockerhub, which has the built ROS2 package stack contained in this repository, along with its dependencies.