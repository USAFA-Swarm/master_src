# master_ws/src/master/comms_interface/comms_interface/telemetry_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, BatteryState

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry')

        self.create_subscription(Imu, '/smaug2/imu/data', self.imu_callback, 10)
        self.create_subscription(NavSatFix, '/smaug2/global_position/raw/fix', self.gps_callback, 10)
        self.create_subscription(BatteryState, '/smaug2/battery', self.battery_callback, 10)

    def imu_callback(self, msg):
        self.get_logger().info(f'IMU received: Orientation={msg.orientation}')

    def gps_callback(self, msg):
        self.get_logger().info(f'GPS: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}')

    def battery_callback(self, msg):
        self.get_logger().info(f'Battery voltage={msg.voltage:.2f}V, remaining={msg.percentage*100:.1f}%')


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
