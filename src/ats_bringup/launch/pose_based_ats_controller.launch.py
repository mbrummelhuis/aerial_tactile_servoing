from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory
import os
import datetime

logging = True
md_name = 'flight_mission_director_contact'

"""
Launch simulation with one arm.

The package can be launched with 'ros2 launch ats_bringup gz_sim_one_arm.launch.py'
"""

def generate_launch_description():
    ld = LaunchDescription()

    arg_logging = DeclareLaunchArgument(
        'logging',
        default_value='false',
        description='Enable logging'
    )
    arg_tactip_enable = DeclareLaunchArgument(
        'tactip_enable',
        default_value='false',
        description='Enable tactip driver node'
    )
    arg_major_frequency = DeclareLaunchArgument(
        'major_frequency',
        default_value='25.0',
        description='Main frequency of all nodes'
    )
    ld.add_action(arg_logging)
    ld.add_action(arg_tactip_enable)
    ld.add_action(arg_major_frequency)

    if logging:
        rosbag_name = 'ros2bag_'+datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        ros2bag = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', '/ros2_ws/aerial_tactile_servoing/rosbags/'+rosbag_name, '-a'], 
            output='screen', 
            log_cmd=True,
        )
        ld.add_action(ros2bag)

    mission_director = Node(
        package='mission_director_py',
        executable=md_name,
        name='mission_director',
        output='screen',
        parameters=[
            {'frequency': LaunchConfiguration('major_frequency')},
            {'takeoff_altitude': -1.5},
            {'landing_velocity': -0.5},
            {'search_velocity': 0.05},
            {'ssim_contact_threshold': 0.7},
            {'hover_time': 3.0},
            {'position_clip': 3.0}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    ld.add_action(mission_director)

    controller = Node(
        package='pose_based_ats',
        executable='pose_based_ats',
        name='ats_controller',
        output='screen',
        parameters=[
            {'frequency': LaunchConfiguration('major_frequency')},
            {'reference_pose': [0., 0., -0.002]},
            {'Kp': 1.5},
            {'Ki': 0.1},
            {'windup_clip': 1.},
            {'regularization_weight': 0.001},
            {'ssim_contact_threshold': 0.7}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    ld.add_action(controller)

    # Ctrl + / to comment/uncomment a code block
    param_file = os.path.join(get_package_share_directory('ats_bringup'), 'config', 'feetech_ros2.yaml')
    servo_driver = Node(
        package="feetech_ros2",
        executable="feetech_ros2_interface",
        name="feetech_ros2_interface",
        output="screen",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "fatal"] # Suppress most logs
    )
    ld.add_action(servo_driver)

    tactip_driver = Node(
        package='tactip_ros2_driver',
        executable='tactip_ros2_driver',
        name='tactip_driver',
        output='screen',
        parameters=[
            {'source': 0},
            {'frequency': LaunchConfiguration('major_frequency')},
            {'verbose': False},
            {'test_model_time': False},
            {'save_debug_image': False},
            {'save_directory': os.path.join('/home','martijn','aerial_tactile_servoing','data','tactip_images')}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    ld.add_action(tactip_driver)
    
    

    return ld