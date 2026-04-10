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
    ctf = cfg['camera_tf']
    pl  = cfg['precision_landing']

    return LaunchDescription([

        # ------------------------------------------------------------------
        # 1. MAVROS — bridges FCU ↔ ROS2 topics/services
        #    Namespace matches the drone name typed into controller.py
        # ------------------------------------------------------------------
        Node(
            package='mavros',
            executable='mavros_node',
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
                'format':    cam['format'],
                'width':     cam['width'],
                'height':    cam['height'],
                'framerate': cam['framerate'],
                'frame_id':  ctf['child_frame'],
            }],
        ),

        # ------------------------------------------------------------------
        # 3. Image rotate — corrects upside-down camera (Rotate180 physical mount)
        #    Subscribes /camera/image_raw, publishes /camera/image_rotated
        #    AprilTag and any viewer should use /camera/image_rotated
        # ------------------------------------------------------------------
        Node(
            package='image_rotate',
            executable='image_rotate',
            name='image_rotate',
            output='screen',
            parameters=[{'rotation_angle': 3.14159265}],
            remappings=[
                ('image',         '/camera/image_raw'),
                ('rotated/image', '/camera/image_rotated'),
            ],
        ),

        # ------------------------------------------------------------------
        # 4. AprilTag detection (apriltag_ros)
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
                'size':        tag['tag_size'],        # default for unlisted tags
                'max_hamming': tag['max_hamming'],
                'tag.ids':     tag['tag_ids'],         # per-tag size overrides
                'tag.sizes':   tag['tag_sizes'],
            }],
            remappings=[
                # apriltag_ros subscribes by default to image_rect + camera_info
                # Use image_rotated so AprilTag sees the corrected (right-side-up) image
                ('image_rect',  '/camera/image_rotated'),
                ('camera_info', '/camera/camera_info'),
                # publish detections under /apriltag/ namespace
                ('detections',  '/apriltag/detections'),
            ],
        ),

        # ------------------------------------------------------------------
        # 5. Static TF: base_link → camera
        #    Tells tf2 where the camera sits relative to the drone body.
        #    Values must be measured from the physical mount.
        # ------------------------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_static',
            arguments=[
                '--x',              str(ctf['x']),
                '--y',              str(ctf['y']),
                '--z',              str(ctf['z']),
                '--roll',           str(ctf['roll']),
                '--pitch',          str(ctf['pitch']),
                '--yaw',            str(ctf['yaw']),
                '--frame-id',       ctf['parent_frame'],
                '--child-frame-id', ctf['child_frame'],
            ],
        ),

        # ------------------------------------------------------------------
        # 6. Identity TF: camera → camera_rotated
        #    image_rotate appends "_rotated" to the frame_id in image headers.
        #    apriltag_ros may use that modified frame_id as the TF parent.
        #    This zero-offset TF ensures the chain base_link→camera→camera_rotated
        #    is always complete regardless of which frame apriltag uses.
        # ------------------------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_rotated_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id',       'camera',
                '--child-frame-id', 'camera_rotated',
            ],
        ),

        # ------------------------------------------------------------------
        # 7. Precision landing — monitors /apriltag/detections at all times;
        #    takes setpoint control when target tag is confirmed visible,
        #    centers laterally, descends, hands off to ArduCopter LAND
        # ------------------------------------------------------------------
        Node(
            package='onboard',
            executable='precision_landing',
            name='precision_landing',
            output='screen',
            parameters=[{
                'drone_name':       pl['drone_name'],
                'landing_tag_id':   pl['landing_tag_id'],
                'high_alt_tag_id':  pl['high_alt_tag_id'],
                'tag_switch_alt':   pl['tag_switch_alt'],
                'confirm_frames':   pl['confirm_frames'],
                'tag_loss_timeout': pl['tag_loss_timeout'],
                'descent_step':     pl['descent_step'],
                'land_final_alt':   pl['land_final_alt'],
                'lateral_gain':     pl['lateral_gain'],
                'loop_rate':        pl['loop_rate'],
            }],
        ),

    ])
