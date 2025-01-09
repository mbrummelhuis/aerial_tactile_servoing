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