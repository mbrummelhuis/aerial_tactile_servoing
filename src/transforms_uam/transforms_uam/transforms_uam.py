import math
import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class TransformUAM(Node):
    def __init__(self):
        super().__init__('transforms_uam')

        # Publishers
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        # --- World to base transform (floating base) ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'ned'
        t.child_frame_id = 'body'

        # Example position (e.g. drone hovering with sinusoidal bobbing)
        time_elapsed = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = -1.0

        # Identity orientation (no rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

        # --- Joint states ---
        joint_msg = JointState()
        joint_msg.header.stamp = now
        joint_msg.name = ['pivot', 'shoulder', 'elbow']

        # Example joint motion
        joint_msg.position = [
            np.pi/3, # pivot
            0.0,  # shoulder
            np.pi/6 # elbow
        ]

        self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TransformUAM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()