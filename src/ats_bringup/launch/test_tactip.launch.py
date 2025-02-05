from launch import LaunchDescription
from launch_ros.actions import Node

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
            {'source': 0},
            {'frequency': 10},
            {'test_model_time': True}
        ],
    )
    
    ld.add_action(tactip_ros2_node)
    
    return ld