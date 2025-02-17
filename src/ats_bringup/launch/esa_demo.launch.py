from launch import LaunchDescription
from launch_ros.actions import Node

"""
Launch file for the ESA demo. This launch file starts the mission director node and two servo driver nodes to hover the drone and control the 
servos doing the YMCA.

The package can be launched with 'ros2 launch ats_bringup esa_demo.launch.py'
"""

def generate_launch_description():
    ld = LaunchDescription()

    '''
    mission_director = Node(
        package='mission_director_py',
        executable='mission_director_py',
        name='mission_director_py',
        output='screen',
        parameters=[
            {'frequency': 20.},
            {'hover_duration': 10.},
            {'landing_velocity': 0.1}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    '''
    servo_driver = Node(
        package='feetech_ros2_driver',
        executable='feetech_ros2_driver',
        name='feetech_ros2_driver_1',
        output='screen',
        parameters=[
            {'pivot_1_id': 11},
            {'shoulder_1_id':32},
            {'elbow_1_id': 3},
            {'pivot_2_id': 1},
            {'shoulder_2_id':2},
            {'elbow_2_id': 13},
            {'limit_pivot': 4},
            {'limit_shoulder': 10},
            {'home_velocity': 50},
            {'port': '/dev/ttyUSB0'},
            {'frequency': 20},
            {'namespace': 'servo'},
            {'qos_depth': 10}
        ],
        arguments=['--ros-args', '--log-level', 'debug']
    )

    ld.add_action(servo_driver)
    
    return ld