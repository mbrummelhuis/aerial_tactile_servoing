from launch import LaunchDescription
from launch_ros.actions import Node
import launch

"""
Launch file for testing the ROS2 Dynamixel driver (for Feetech servos). 

The package can be launched with 'ros2 launch ats_bringup test_servos.launch.py'
"""

def generate_launch_description():
    ld = LaunchDescription()


    # SERVO IDS
    # Pivot FCU side: 1
    # Shoulder FCU side: 12
    # Elbow FCU side:??
    # Pivot OPI side: 11
    # Shoulder OPI side : 2
    # Elbow OPI side: ??

    # PINS:
    # Green: 3
    # Blue: 4
    # White: 9
    # Yellow: 10
    
    feetech_servo_driver = Node(
        package='feetech_ros2_driver',
        executable='feetech_ros2_driver',
        name='feetech_ros2_driver',
        output='screen',
        parameters=[
            {'pivot_id': 11},
            {'shoulder_id':2},
            {'elbow_id': 13},
            {'limit_pivot': 4},
            {'limit_shoulder': 10},
            {'home_velocity': 50},
            {'port': '/dev/ttyUSB0'},
            {'frequency': 20},
            {'namespace': 'servo1'},
            {'qos_depth': 10}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    ld.add_action(feetech_servo_driver)
    
    return ld
