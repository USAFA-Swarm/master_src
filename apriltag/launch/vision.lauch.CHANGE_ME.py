import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('apriltag')
    config_dir = os.path.join(pkg_share, 'config')

    stop_detector_model = os.path.join(config_dir, 'CHANGE_ME.svm')
    camera_info_path = os.path.join(config_dir, 'default_cam.yaml')
    camera_info_url = 'file://' + camera_info_path

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            # TODO: Complete this function call
            package='apriltag',
            executable='stop_detector',
            name='stop_detector_node',
            output='screen',
            arguments=['-d', stop_detector_model],
        ),
        launch_ros.actions.Node(
            # TODO: Complete this function call
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'framerate': 15.0,
                'camera_info_url': camera_info_url
            }]
        ),
        launch_ros.actions.Node(
            # TODO: Complete this function call
            package='apriltag',
            executable='apriltag',
            name='apriltag_node',
            output='screen',
            parameters=[{
                'camera_info_file': camera_info_path
            }]
        ),
    ])