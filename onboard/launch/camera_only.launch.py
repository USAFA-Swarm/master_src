"""
camera_only.launch.py — Minimal camera-only launch for live feed testing

Starts camera_ros only. No MAVROS, no AprilTag, no TF.
Use this for camera checkout and ground-station live feed testing.

IMPORTANT: set ROS_LOCALHOST_ONLY=1 before launching to prevent the
Humble/Jazzy DDS incompatibility from crashing camera_ros.

Usage (on Pi):
  export ROS_LOCALHOST_ONLY=1
  ros2 launch onboard camera_only.launch.py

Then on the ground laptop (separate terminal, keep SSH session open):
  ssh -X hare@192.168.168.102
  ... run view_camera.sh, or manually:
  export ROS_DOMAIN_ID=13 && export ROS_LOCALHOST_ONLY=1
  source /opt/ros/jazzy/setup.bash
  ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw \\
    --qos-reliability best_effort
"""

import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('onboard')
    config_path = os.path.join(pkg_share, 'config', 'onboard.yaml')

    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    cam = cfg['camera']
    tf  = cfg['camera_tf']

    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'camera':   cam['camera'],
                'format':   cam['format'],
                'width':    cam['width'],
                'height':   cam['height'],
                'frame_id': tf['child_frame'],
            }],
        ),
    ])
