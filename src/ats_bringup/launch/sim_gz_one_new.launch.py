from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os
import datetime

"""
Launch simulation with one arm. Corresponds to the implementation after refactoring.

The package can be launched with 'ros2 launch ats_bringup sim_gz_one_new.launch.py'
"""
logging = False
config_name = 'dxl_ros2_ats.yaml'

def generate_launch_description():
    ld = LaunchDescription()

    # Add the paths to the simulation and controller launch files
    sim_launch_path = os.path.join(get_package_share_directory('px4_uam_sim'), 'launch', 'gz_martijn_one_arm.launch.py')
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path),
        launch_arguments={
            'logging': 'false', 
            'tactip_enable': 'true', 
            'major_frequency': '25.0'}.items())
        )
    # Add sim remapper node
    sim_remapper = Node(
        package='sim_remapper',
        executable='sim_remapper',
        name='sim_remapper',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    ld.add_action(sim_remapper)

    # Add the logging
    if logging:
        rosbag_name = 'ros2bag_sim_'+datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        rosbag_path = f'/home/martijn/aerial_tactile_servoing/rosbags/{rosbag_name}'
        rosbag_record = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', rosbag_path, '-a'], 
            output='screen', 
            log_cmd=True,
        )
        ld.add_action(rosbag_record)

    # Mission director node
    mission_director = Node(
        package="mission_director",
        executable="uam_control_test",
        name="md_uam_control_test",
        output="screen",
        parameters=[
            {'sm.frequency': 10.0},
            {'sm.position_clip': 3.0},
            {'sm.takeoff_altitude': 1.5},
            {'sm.dry_test': False}
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )
    ld.add_action(mission_director)

    # Dynamixel servo driver node
    param_file = os.path.join(get_package_share_directory('ats_bringup'), 'config', config_name)
    servo_driver = Node(
        package="dxl_driver",
        executable="dxl_driver_node",
        name="dxl_driver",
        output="screen",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "info"]
    )
    ld.add_action(servo_driver)

    return ld
