# master_ws/src/master/comms_interface/comms_interface/telemetry.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, BatteryState

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry')

        self.declare_parameter('drone_name', 'smaug2')
        drone = self.get_parameter('drone_name').get_parameter_value().string_value

        self.create_subscription(Imu, f'/{drone}/imu/data', self.imu_callback, 10)
        self.create_subscription(NavSatFix, f'/{drone}/global_position/raw/fix', self.gps_callback, 10)
        self.create_subscription(BatteryState, f'/{drone}/battery', self.battery_callback, 10)

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
