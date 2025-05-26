import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from rosbags.typesys import register_types, get_types_from_msg
import os

def register_px4_msgs():
    msg_dir = '/home/martijn/aerial_tactile_servoing/src/px4_msgs/msg'
    msg_defs = {}

    for filename in os.listdir(msg_dir):
        if filename.endswith('.msg'):
            path = os.path.join(msg_dir, filename)
            with open(path, 'r') as f:
                msg_defs[filename] = f.read()

    # px4_msgs is the name of the message package
    parsed = get_types_from_msg(msg_defs, 'px4_msgs')
    register_types(parsed)


def extract_field(msg, field_path):
    """Recursively access nested fields from a message using dot-separated path, with support for list indexing."""
    for attr in field_path.split('.'):
        if attr.isdigit():
            msg = msg[int(attr)]
        else:
            msg = getattr(msg, attr)
    return msg

def main(bag_path, topic_name, field_path):
    #register_px4_msgs()
    bag_path = Path(bag_path)

    timestamps = []
    values = []

    with Reader(bag_path) as reader:
        for connection in reader.connections:
            if connection.topic == topic_name:
                msgtype = connection.msgtype
                break
        else:
            raise ValueError(f"Topic {topic_name} not found in bag.")

        for conn, timestamp, rawdata in reader.messages(connections=[connection]):
            msg = deserialize_cdr(rawdata, connection.msgtype)
            value = extract_field(msg, field_path)
            time_sec = timestamp * 1e-9  # Convert from nanoseconds
            timestamps.append(time_sec)
            values.append(value)
    print("Plotting")
    # Plot
    plt.plot(timestamps, values)
    plt.xlabel('Time (s)')
    plt.ylabel(field_path)
    plt.title(f'{field_path} from {topic_name}')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot field from ROS 2 bag.')
    parser.add_argument('bag_path', help='Path to ROS 2 bag folder')
    parser.add_argument('topic_name', help='Topic name (e.g., /odom)')
    parser.add_argument('field_path', help='Field path (e.g., pose.pose.position.x)')
    args = parser.parse_args()
    main(args.bag_path, args.topic_name, args.field_path)
