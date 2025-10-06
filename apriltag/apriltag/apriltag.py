#!/usr/bin/env python3

# BSD 3-Clause License
#
# Copyright (c) 2025, Stan Baek
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Stan Baek
# Date Created: April 7, 2025


import rclpy  # Core ROS2 Python library for initialization and management
from rclpy.node import Node  # Base class for all ROS2 nodes
from sensor_msgs.msg import Image  # ROS2 message type for camera images
from geometry_msgs.msg import PoseStamped  # ROS2 message type for 3D poses
from cv_bridge import CvBridge  # Converts ROS2 Image messages to OpenCV images
import cv2
import os  # For file and path operations
import numpy as np
from pupil_apriltags import Detector  # AprilTag detector library
import yaml  # Library for parsing YAML files
from scipy.spatial.transform import Rotation  # Converts rotation matrices to quaternions
from ament_index_python.packages import get_package_share_directory  # Utility to locate package directories


class AprilTagNode(Node):
    """
    A ROS2 Node that detects AprilTags in camera images and publishes their poses (position and orientation).

    Attributes:
        bridge (CvBridge): Converts between ROS2 Image messages and OpenCV images.
        K (ndarray): Camera matrix (intrinsic parameters).
        D (ndarray): Distortion coefficients for the camera lens.
        fx, fy, cx, cy (float): Focal lengths and principal point coordinates from the camera matrix.
        tag_size (float): Physical size of the AprilTags in meters.
        detector (Detector): AprilTag detector instance.
        pose_pub (Publisher): ROS2 publisher for AprilTag poses.
    """
    def __init__(self):
        """Initialize the AprilTag detection node."""
        super().__init__('apriltag_node')  # Name the node "apriltag_node"
        
        # Initialize CvBridge to convert ROS Image messages to OpenCV format
        self.bridge = CvBridge()

        # Placeholder for camera parameters (to be loaded from calibration files)
        self.K = None
        self.D = None
        self.fx = self.fy = self.cx = self.cy = None

        # Define the physical size of AprilTags in meters
        self.tag_size = 0.166

        # Load camera calibration parameters from a YAML file
        calib_file = self.declare_parameter('camera_info_file', '').get_parameter_value().string_value
        calib_file = os.path.expanduser(calib_file)  # Expand paths like "~" to full paths

        # Determine calibration file path
        if calib_file and os.path.isfile(calib_file):
            camera_info_path = calib_file
        else:
            # Fallback to default calibration file within the package
            pkg_share = get_package_share_directory('lab11_apriltag')
            camera_info_path = os.path.join(pkg_share, 'config', 'default_cam.yaml')
            if not os.path.isfile(camera_info_path):  # If the default file is missing, raise an error
                self.get_logger().error(f"Default calibration file not found: {camera_info_path}")
                raise FileNotFoundError(camera_info_path)

        # Load camera calibration data
        self.load_camera_info(camera_info_path)

        # TODO: Create an image subscriber to receive camera images
        self.subscription = self.create_subscription(Image, 'image_raw', self.image_callback, 10)

        # Initialize the AprilTag detector
        # API: https://pupil-apriltags.readthedocs.io/en/stable/api.html
        self.detector = Detector(families='tag36h11')

        # TODO: Create a publisher to publish AprilTag pose data (PoseStamped) using a topic named "/apriltag_pose".
        self.apriltag_pose_pub = self.create_publisher(PoseStamped, 'apriltag_pose', 10)        

    def load_camera_info(self, filepath) -> None:
        """
        Load camera calibration parameters from a YAML file.

        Args:
            filepath (str): Path to the camera calibration file.

        Raises:
            FileNotFoundError: If the calibration file is not found.
        """
        with open(filepath, 'r') as file:
            calib_data = yaml.safe_load(file)

        # Parse intrinsic camera matrix and distortion coefficients
        self.K = np.array(calib_data['camera_matrix']['data']).reshape((3, 3))
        self.D = np.array(calib_data['distortion_coefficients']['data'])

        # Extract focal lengths and principal point from the matrix
        self.fx = self.K[0, 0]
        self.fy = self.K[1, 1]
        self.cx = self.K[0, 2]
        self.cy = self.K[1, 2]

        self.get_logger().info(f"Loaded camera calibration from: {filepath}")

    def image_callback(self, image: Image) -> None:
        """
        Process incoming camera images to detect AprilTags and publish their poses.

        Args:
            image (Image): ROS2 Image message from the camera.
        """
        # Convert ROS2 Image message to OpenCV format
        yuyv_img = self.bridge.imgmsg_to_cv2(image, desired_encoding='yuv422_yuy2')

        # Convert YUYV image to grayscale for tag detection
        gray_img = cv2.cvtColor(yuyv_img, cv2.COLOR_YUV2GRAY_YUY2)

        # Rectify the image using camera parameters to remove lens distortion
        h, w = gray_img.shape
        new_K, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), 1)
        rectified_gray = cv2.undistort(gray_img, self.K, self.D, None, new_K)

        # Prepare camera parameters for the AprilTag detector
        camera_params = [self.fx, self.fy, self.cx, self.cy]

        # Detect AprilTags in the rectified image
        tags = self.detector.detect(rectified_gray, estimate_tag_pose=True, 
                                    camera_params=camera_params, tag_size=self.tag_size)

        for tag in tags:
            # Extract position (translation vector) and orientation (rotation matrix) of the detected tag
            t = tag.pose_t.flatten()  # (x, y, z) position
            R = tag.pose_R  # Rotation matrix
            
            # Convert rotation matrix to quaternion for ROS Pose messages
            rot = Rotation.from_matrix(tag.pose_R)
            quat = rot.as_quat()  # Returns [x, y, z, w]

            # TODO: Create a PoseStamped message for the detected tag's pose
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = str(tag.tag_id)
            pose_msg.pose.position.x = t[0]
            pose_msg.pose.position.y = t[1]
            pose_msg.pose.position.z = t[2]
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

            # TODO: Publish the pose message to the ROS2 topic
            self.apriltag_pose_pub.publish(pose_msg)

            # TODO: Log the detected tag's ID, position, and orientation
            self.get_logger().info(f"Detected tag ID: {tag.tag_id}, Position: {t}, Orientation (quaternion): {quat}")

            # Draw a polygon around the detected tag and display its ID
            corners = np.int32(tag.corners)
            cv2.polylines(rectified_gray, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
            center = tuple(np.int32(tag.center))
            
             # TODO: Label the tag ID at the center of the tag.
            cv2.putText(rectified_gray, str(tag.tag_id), center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Display the rectified image with drawn tags
        cv2.imshow("AprilTag Detection", rectified_gray)
        cv2.waitKey(1)


def main(args=None):
    """
    Entry point for the AprilTag detection node.
    Initializes the ROS2 environment, starts the node, and handles clean shutdown.
    """
    rclpy.init(args=args)
    node = AprilTagNode()  # Create an instance of the AprilTag detection node
    rclpy.spin(node)  # Keep the node active and listening for incoming messages
    node.destroy_node()  # Clean up when the node is shut down
    rclpy.shutdown()  # Shutdown the ROS2 environment
    cv2.destroyAllWindows()  # Close any OpenCV windows