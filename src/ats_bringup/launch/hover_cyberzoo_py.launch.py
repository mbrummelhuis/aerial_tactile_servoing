from launch import LaunchDescription
from launch_ros.actions import Node

"""
Example launch file. You can create launch files for subtests in this package in the same way as this file. 
This file launches the cv_aruco_detector node as an example.
To add nodes to be launched, copy the code here and add the package containing the node as a dependency in the package.xml file.

The package can be launched with 'ros2 launch tucan_bringup example.launch.py'
"""

def generate_launch_description():
    ld = LaunchDescription()

    mission_director = Node(
        package='mission_director_py',
        executable='mission_director_py',
        name='mission_director_py',
        output='screen',
        parameters=[
            {'frequency': 10.},
            {'hover_duration': 10.},
            {'landing_velocity': 0.1}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    ld.add_action(mission_director)
    
    return ld