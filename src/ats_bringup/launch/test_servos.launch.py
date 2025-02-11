from launch import LaunchDescription
from launch_ros.actions import Node

"""
Launch file for testing the ROS2 Dynamixel driver (for Feetech servos). 

The package can be launched with 'ros2 launch ats_bringup test_servos.launch.py'
"""

def generate_launch_description():
    ld = LaunchDescription()
    
    feetech_servo_driver = Node(
        package='feetech_ros2_driver',
        executable='feetech_ros2_driver',
        name='feetech_ros2_driver',
        output='screen',
        parameters=[
            {'pivot_id': 1},
            {'shoulder_id': 12},
            {'elbow_id': 3},
            {'limit_pivot': 10},
            {'limit_shoulder': 6},
            {'frequency': 1},
            {'qos_depth': 10}
        ],
    )
    
    ld.add_action(feetech_servo_driver)
    
    return ld
