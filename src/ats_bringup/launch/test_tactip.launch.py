from launch import LaunchDescription
from launch_ros.actions import Node

"""
Launch file for testing the TacTip ROS2 driver. 

The package can be launched with 'ros2 launch ats_bringup test_tactip.launch.py'
"""

def generate_launch_description():
    ld = LaunchDescription()
    
    ar_detection_node = Node(
        package='tactip_ros2_driver',
        executable='tactip_ros2_driver',
    )
    
    ld.add_action(ar_detection_node)
    
    return ld