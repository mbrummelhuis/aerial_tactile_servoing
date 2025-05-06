from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os

"""
Launch simulation with one arm.

The package can be launched with 'ros2 launch ats_bringup gz_sim_one_arm.launch.py'
"""

def generate_launch_description():
    # Add the paths to the simulation and controller launch files
    sim_launch_path = os.path.join(get_package_share_directory('px4_uam_sim'), 'launch', 'gz_martijn_one_arm.launch.py')
    sim_controller_path = os.path.join(get_package_share_directory('ats_bringup'), 'launch', 'sim_controller.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch_path),
            launch_arguments={'logging': 'false', 'tactip_enable': 'false', 'major_frequency': '25.0'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_controller_path)),
    ])
