from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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


    mission_director = Node(
        package='mission_director_py',
        executable='sim_mission_director',
        name='mission_director',
        output='screen',
        parameters=[
            {'frequency': LaunchConfiguration('major_frequency')},
            {'takeoff_altitude': -1.5},
            {'landing_velocity': -0.5},
            {'hover_time': 5.0}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    ld.add_action(mission_director)

    ats_planner = Node(
        package='ats_planner',
        executable='ats_planner',
        name='ats_planner',
        output='screen',
        parameters=[
            {'frequency': LaunchConfiguration('major_frequency')},
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    ld.add_action(ats_planner)

    ats_velocity_controller = Node(
        package='ats_velocity_controller',
        executable='ats_velocity_controller',
        name='ats_velocity_controller',
        output='screen',
        parameters=[
            {'frequency': LaunchConfiguration('major_frequency')},
            {'kp': 5.0},
            {'ki': 0.0},
            {'kd': 0.0},
            {'max_integral': 1.0},
            {'ewma_alpha': 0.3}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    ld.add_action(ats_velocity_controller)

    inverse_dif_kinematics = Node(
        package='inverse_differential_kinematics',
        executable='inverse_differential_kinematics',
        name='inverse_differential_kinematics',
        output='screen',
        parameters=[
            {'frequency': LaunchConfiguration('major_frequency')},
            {'mode': 'flight'}, # Set to 'flight to enable flight testing'
            {'verbose': False},
            {'start_active': False}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    ld.add_action(inverse_dif_kinematics)
    if LaunchConfiguration('tactip_enable') == 'true':
        tactip_driver = Node(
            package='tactip_ros2_driver',
            executable='tactip_ros2_driver',
            name='tactip_driver',
            output='screen',
            parameters=[
                {'source': 0},
                {'frequency': LaunchConfiguration('major_frequency')},
                {'verbose': False},
                {'test_model_time': True}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
        ld.add_action(tactip_driver)

    return ld