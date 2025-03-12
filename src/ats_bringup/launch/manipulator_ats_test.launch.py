from launch import LaunchDescription
from launch_ros.actions import Node

"""
Launch file for testing ATS controller with only manipulator

The package can be launched with 'ros2 launch ats_bringup ats_test_launch.launch.py'
"""

def generate_launch_description():
    ld = LaunchDescription()

    mission_director = Node(
        package='mission_director_py',
        executable='dry_ats_md',
        name='mission_director',
        output='screen',
        parameters=[
            {'initial_joint_states': [0.0, 0.0, 0.0]},
            {'entrypoint_time': 5.0},
            {'position_arm_time': 5.0},
            {'tactile_servoing_time': 5.0},
            {'frequency': 15.}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

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
            {'frequency': 15.},
            {'mode': 'dry'} # Set to 'flight to enable flight testing'
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    if False:
        tactip_driver = Node(
            package='tactip_ros2_driver',
            executable='tactip_ros2_driver',
            name='tactip_driver',
            output='screen',
            parameters=[
                {'source': 4},
                {'frequency': 10},
                {'test_model_time': False}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )

        servo_driver = Node(
            package="feetech_ros2_driver",
            executable="feetech_ros2_driver",
            name="feetech_driver",
            output="screen",
            parameters=[
                {"frequency": 10},

            ],
            arguments=["--ros-args", "--log-level", "info"]
        )
        ld.add_action(tactip_driver)
        ld.add_action(servo_driver)

    ld.add_action(mission_director)
    ld.add_action(ats_planner)
    ld.add_action(ats_velocity_controller)
    ld.add_action(inverse_dif_kinematics)
    
    return ld