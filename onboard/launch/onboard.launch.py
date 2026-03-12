"""
onboard.launch.py — Full Pi 4 onboard stack

Starts (in order):
  1. mavros_node      — MAVROS bridge to ArduPilot FCU
  2. camera_node      — libcamera CSI camera (camera_ros)
  3. apriltag_node    — AprilTag detection with TF2 (apriltag_ros)
  4. camera_tf_static — Static TF: base_link → camera_optical_frame

All parameters are read from config/onboard.yaml.
No values are hardcoded in this file.

Usage (on Pi):
  ros2 launch onboard onboard.launch.py
"""

import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Load the single config file — only place parameters are defined
    pkg_share = get_package_share_directory('onboard')
    config_path = os.path.join(pkg_share, 'config', 'onboard.yaml')

    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    mav = cfg['mavros']
    cam = cfg['camera']
    tag = cfg['apriltag']
    tf  = cfg['camera_tf']

    return LaunchDescription([

        # ------------------------------------------------------------------
        # 1. MAVROS — bridges FCU ↔ ROS2 topics/services
        #    Namespace matches the drone name typed into controller.py
        # ------------------------------------------------------------------
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros_node',
            namespace=mav['drone_name'],
            output='screen',
            parameters=[{
                'fcu_url': mav['fcu_url'],
            }],
        ),

        # ------------------------------------------------------------------
        # 2. Camera (libcamera via camera_ros)
        #    Node is named 'camera' so ~/image_raw → /camera/image_raw
        #                              ~/camera_info → /camera/camera_info
        # ------------------------------------------------------------------
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'camera':    cam['camera'],
                'width':     cam['width'],
                'height':    cam['height'],
                'framerate': cam['framerate'],
                'frame_id':  tf['child_frame'],
            }],
        ),

        # ------------------------------------------------------------------
        # 3. AprilTag detection (apriltag_ros)
        #    Subscribes to /camera/image_raw and /camera/camera_info
        #    Publishes  /apriltag/detections (AprilTagDetectionArray)
        #    Publishes  TF2: camera_optical_frame → tag_<id>
        # ------------------------------------------------------------------
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            parameters=[{
                'family':      tag['tag_family'],
                'size':        tag['tag_size'],
                'max_hamming': tag['max_hamming'],
            }],
            remappings=[
                # camera_ros publishes to /camera/image_raw and /camera/camera_info
                # apriltag_ros subscribes by default to image_rect + camera_info
                ('image_rect',  '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
                # publish detections under /apriltag/ namespace
                ('detections',  '/apriltag/detections'),
            ],
        ),

        # ------------------------------------------------------------------
        # 4. Static TF: base_link → camera_optical_frame
        #    Tells tf2 where the camera sits relative to the drone body.
        #    Values must be measured from the physical mount.
        # ------------------------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_static',
            arguments=[
                '--x',              str(tf['x']),
                '--y',              str(tf['y']),
                '--z',              str(tf['z']),
                '--roll',           str(tf['roll']),
                '--pitch',          str(tf['pitch']),
                '--yaw',            str(tf['yaw']),
                '--frame-id',       tf['parent_frame'],
                '--child-frame-id', tf['child_frame'],
            ],
        ),

    ])
