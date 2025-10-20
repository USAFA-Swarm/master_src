#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import BatteryState, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, RCIn, RCOut, ManualControl, SetMode
from mavros_msgs.srv import CommandBool

class GroundStationController(Node):
    def __init__(self):
        super().__init__('ground_station_controller')

        # ----------------------
        # 1. Subscriptions (listen)
        # ----------------------
        self.create_subscription(State, '/smaug2/state', self.state_callback, 10)
        self.create_subscription(BatteryState, '/smaug2/battery', self.battery_callback, 10)
        self.create_subscription(Imu, '/smaug2/imu/data', self.imu_callback, 10)
        self.create_subscription(PoseStamped, '/smaug2/local_position/pose', self.pose_callback, 10)
        self.create_subscription(TwistStamped, '/smaug2/local_position/velocity_local', self.vel_callback, 10)
        self.create_subscription(RCIn, '/smaug2/rc/in', self.rc_in_callback, 10)

        # ----------------------
        # 2. Publishers (send)
        # ----------------------
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/smaug2/setpoint_velocity/cmd_vel', 10)
        self.manual_pub = self.create_publisher(ManualControl, '/smaug2/manual_control/send', 10)
        self.mode_pub = self.create_publisher(SetMode, '/smaug2/set_mode', 10)  # optional
        self.arming_client = self.create_client(CommandBool, '/smaug2/cmd/arming')

        # ----------------------
        # 3. Internal state
        # ----------------------
        self.current_state = None
        self.current_pose = None
        self.battery_level = None
        self.remote_input = None

        self.get_logger().info("✅ Ground station controller initialized and listening...")

    # ----------------------
    # Callbacks
    # ----------------------
    def state_callback(self, msg):
        self.current_state = msg
        self.get_logger().debug(f"Drone mode: {msg.mode}, armed: {msg.armed}")

    def battery_callback(self, msg):
        self.battery_level = msg.percentage
        self.get_logger().debug(f"Battery: {msg.percentage * 100:.1f}%")

    def imu_callback(self, msg):
        # could log or filter data
        pass

    def pose_callback(self, msg):
        self.current_pose = msg
        pass

    def vel_callback(self, msg):
        pass

    def rc_in_callback(self, msg):
        self.remote_input = msg
        # Example: detect if pilot takes manual control
        if any(ch > 1800 for ch in msg.channels):
            self.get_logger().warn("Manual RC input detected — switching to manual control!")

    # ----------------------
    # Command methods
    # ----------------------
    def send_velocity_command(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.linear.z = vz
        cmd.twist.angular.z = yaw_rate
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f"Sent velocity command: vx={vx}, vy={vy}, vz={vz}, yaw_rate={yaw_rate}")

    def arm_drone(self, arm=True):
        if not self.arming_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Arming service not available")
            return
        req = CommandBool.Request()
        req.value = arm
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info("Drone armed!" if arm else "Drone disarmed!")
        else:
            self.get_logger().error("Failed to arm/disarm drone")

def main(args=None):
    rclpy.init(args=args)
    node = GroundStationController()

    # Example usage loop
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            # send commands periodically (testing)
            # node.send_velocity_command(vx=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
