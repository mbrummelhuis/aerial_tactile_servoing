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
        executable='pbats_mission_director',
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

    controller = Node(
        package='pose_based_ats',
        executable='pose_based_ats',
        name='ats_controller',
        output='screen',
        parameters=[
            {'frequency': LaunchConfiguration('major_frequency')},
            {'reference_pose': [0., 0., -0.003]},
            {'Kp': 3.0},
            {'Ki': 0.1},
            {'windup_clip': 5.}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    ld.add_action(controller)


    if LaunchConfiguration('tactip_enable') == 'true':
        tactip_driver = Node(
            package='tactip_ros2_driver',
            executable='tactip_ros2_driver',
            name='tactip_driver',
            output='screen',
            parameters=[
                {'source': 4},
                {'frequency': LaunchConfiguration('major_frequency')},
                {'verbose': False},
                {'test_model_time': False},
                {'save_debug_image': False}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
        ld.add_action(tactip_driver)
    
    

    return ld