import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import datetime
"""
Launch file for testing ATS controller with only manipulator

The package can be launched with 'ros2 launch ats_bringup ats_test_launch.launch.py'
"""

logging = False
tactip_enable = False
servo_enable = True

major_frequency = 25.0


def generate_launch_description():
    ld = LaunchDescription()

    mission_director = Node(
        package='mission_director_py',
        executable='dry_ats_md',
        name='mission_director',
        output='screen',
        parameters=[
            {'initial_joint_states': [0.0, 0.0, 0.0]},
            {'entrypoint_time': 10.0},
            {'position_arm_time': 20.0},
            {'tactile_servoing_time': 30.0},
            {'frequency': major_frequency}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    ats_planner = Node(
        package='ats_planner',
        executable='ats_planner',
        name='ats_planner',
        output='screen',
        parameters=[
            {'frequency': major_frequency}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    ats_velocity_controller = Node(
        package='ats_velocity_controller',
        executable='ats_velocity_controller',
        name='ats_velocity_controller',
        output='screen',
        parameters=[
            {'frequency': major_frequency},
            {'kp': 5.0},
            {'ki': 0.0},
            {'kd': 0.0},
            {'max_integral': 1.0},
            {'ewma_alpha': 0.3}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    inverse_dif_kinematics = Node(
        package='inverse_differential_kinematics',
        executable='inverse_differential_kinematics',
        name='inverse_differential_kinematics',
        output='screen',
        parameters=[
            {'frequency': major_frequency},
            {'mode': 'dry'}, # Set to 'flight to enable flight testing'
            {'verbose': False},
            {'start_active': False}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    if tactip_enable:
        tactip_driver = Node(
            package='tactip_ros2_driver',
            executable='tactip_ros2_driver',
            name='tactip_driver',
            output='screen',
            parameters=[
                {'source': 0},
                {'frequency': major_frequency},
                {'verbose': False},
                {'test_model_time': True}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
        ld.add_action(tactip_driver)
    if servo_enable:
        param_file = os.path.join(get_package_share_directory('ats_bringup'), 'config', 'feetech_ros2.yaml')
        servo_driver = Node(
            package="feetech_ros2",
            executable="feetech_ros2_interface",
            name="feetech_ros2_interface",
            output="screen",
            parameters=[param_file],
            arguments=["--ros-args", "--log-level", "info"]
        )
        
        ld.add_action(servo_driver)

    if logging:
        rosbag_name = 'ros2bag_'+datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        ros2bag = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', '/ros2_ws/aerial_tactile_servoing/rosbags/'+rosbag_name, '-a'], 
            output='screen', 
            log_cmd=True,
        )
        ld.add_action(ros2bag)

    ld.add_action(mission_director)
    ld.add_action(ats_planner)
    ld.add_action(ats_velocity_controller)
    ld.add_action(inverse_dif_kinematics)
    
    return ld
