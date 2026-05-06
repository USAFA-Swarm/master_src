"""
sitl.launch.py — Ground-laptop SITL launch (MAVROS only, no camera/apriltag)

Usage:
  1. Start ArduPilot SITL (in a separate terminal):
       sim_vehicle.py -v ArduCopter --out udp:127.0.0.1:14551

  2. Launch MAVROS bridge:
       ros2 launch capstone sitl.launch.py

     Or with a custom UDP address:
       ros2 launch capstone sitl.launch.py fcu_url:=udp://@127.0.0.1:14551

  3. Run the controller (separate terminal):
       cd /home/dfec/master_src && source install/setup.bash
       python3 capstone/capstone/controller.py
       Enter drone name: drone1

Notes:
  - precision_landing/takeover topic never fires without the Pi — controller
    stays out of PRECISION_LANDING state automatically.
  - Circle command now hovers at center after completing the loop.
  - Namespace 'drone1' must match what you type into controller.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://@127.0.0.1:14551',
        description='MAVROS FCU URL — default expects SITL outputting to UDP 14551',
    )

    drone_name_arg = DeclareLaunchArgument(
        'drone_name',
        default_value='drone1',
        description='Drone ROS namespace — must match controller.py input',
    )

    return LaunchDescription([
        fcu_url_arg,
        drone_name_arg,

        Node(
            package='mavros',
            executable='mavros_node',
            namespace=LaunchConfiguration('drone_name'),
            output='screen',
            parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
            }],
        ),
    ])
