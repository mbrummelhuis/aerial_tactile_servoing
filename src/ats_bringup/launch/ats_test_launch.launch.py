from launch import LaunchDescription
from launch_ros.actions import Node

"""
Launch file for testing ATS controller

The package can be launched with 'ros2 launch ats_bringup ats_test_launch.launch.py'
"""

def generate_launch_description():
    ld = LaunchDescription()

    ats_planner = Node(
        package='ats_planner',
        executable='ats_planner',
        name='ats_planner',
        output='screen',
        parameters=[
            {'frequency': 15.}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    ats_velocity_controller = Node(
        package='ats_velocity_controller',
        executable='ats_velocity_controller',
        name='ats_velocity_controller',
        output='screen',
        parameters=[
            {'frequency': 15.},
            {'kp': 1.0},
            {'ki': 0.01},
            {'kd': 0.0},
            {'max_integral': 1.5},
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
            {'frequency': 15.}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    ld.add_action(ats_planner)
    ld.add_action(ats_velocity_controller)
    ld.add_action(inverse_dif_kinematics)
    
    return ld