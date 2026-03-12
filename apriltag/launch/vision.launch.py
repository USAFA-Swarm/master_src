import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('apriltag')
    camera_info_path = os.path.join(pkg_share, 'config', 'default_cam.yaml')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'framerate': 15.0,
                'camera_info_url': 'file://' + camera_info_path,
            }]
        ),
        launch_ros.actions.Node(
            package='apriltag',
            executable='apriltag',
            name='apriltag_node',
            output='screen',
            parameters=[{
                'camera_info_file': camera_info_path,
            }]
        ),
    ])
