from launch import LaunchDescription
from launch_ros.actions import Node

import os

"""
Launch file for testing the TacTip ROS2 driver. 

The package can be launched with 'ros2 launch ats_bringup test_tactip.launch.py'
"""

def generate_launch_description():
    ld = LaunchDescription()
    
    tactip_ros2_node = Node(
        package='tactip_ros2_driver',
        executable='tactip_ros2_driver',
        name='tactip_ros2_driver',
        output='screen',
        parameters=[
            {'source': 4},
            {'frequency': 10.},
            {'verbose': True},
            {'test_model_time': False},
            {'save_debug_image': True},
            {'save_directory': os.path.join('/home','martijn','aerial_tactile_servoing','data','tactip_images')}
        ],
    )
    
    ld.add_action(tactip_ros2_node)
    
    return ld