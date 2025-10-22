import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, RCIn
from mavros_msgs.srv import CommandBool, SetMode


class GroundStationController(Node):
    def __init__(self):
        super().__init__('ground_station_controller')

        self.drones = ['smaug2']
        self.state = {}

        for name in self.drones:
            ns = f'/{name}'

            # Store last known states
            self.state[name] = {
                'pose': None,
                'velocity': None,
                'imu': None,
                'battery': None,
                'state': None,
                'rc': None
            }

            # Subscriptions
            self.create_subscription(State, f'{ns}/state', lambda msg, n=name: self.state_callback(msg, n), 10)
            self.create_subscription(PoseStamped, f'{ns}/local_position/pose', lambda msg, n=name: self.pose_callback(msg, n), 10)
            self.create_subscription(TwistStamped, f'{ns}/local_position/velocity_local', lambda msg, n=name: self.vel_callback(msg, n), 10)
            self.create_subscription(Imu, f'{ns}/imu/data', lambda msg, n=name: self.imu_callback(msg, n), 10)
            self.create_subscription(BatteryState, f'{ns}/battery', lambda msg, n=name: self.battery_callback(msg, n), 10)
            self.create_subscription(RCIn, f'{ns}/rc/in', lambda msg, n=name: self.rc_callback(msg, n), 10)

            # Publishers
            self.cmd_vel_pub = self.create_publisher(TwistStamped, f'{ns}/setpoint_velocity/cmd_vel', 10)
            self.pose_target_pub = self.create_publisher(PoseStamped, f'{ns}/setpoint_position/local', 10)

            # Services
            self.arming_client = self.create_client(CommandBool, f'{ns}/cmd/arming')
            self.mode_client = self.create_client(SetMode, f'{ns}/set_mode')

    # -------------------------
    # Callbacks
    # -------------------------
    def state_callback(self, msg, drone):
        self.state[drone]['state'] = msg

    def pose_callback(self, msg, drone):
        self.state[drone]['pose'] = msg

    def vel_callback(self, msg, drone):
        self.state[drone]['velocity'] = msg

    def imu_callback(self, msg, drone):
        self.state[drone]['imu'] = msg

    def battery_callback(self, msg, drone):
        self.state[drone]['battery'] = msg

    def rc_callback(self, msg, drone):
        self.state[drone]['rc'] = msg

    # -------------------------
    # Commands
    # -------------------------
    def send_velocity_command(self, drone, vx, vy, vz):
        msg = TwistStamped()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self.cmd_vel_pub.publish(msg)

    def send_position_target(self, drone, x, y, z):
        msg = PoseStamped()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.pose_target_pub.publish(msg)

    def arm(self, drone):
        req = CommandBool.Request(value=True)
        self.arming_client.call_async(req)

    def set_mode(self, drone, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.mode_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)

    # Example: controlling 4 drones named smaug1–smaug4
    drone_names = [f'smaug{i}' for i in range(1, 5)]
    node = GroundStationController(drone_names)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Ground Station Controller.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()