import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ManualControl, RCIn
from mavros_msgs.srv import CommandBool, SetMode


class GroundStationController(Node):
    def __init__(self):
        super().__init__('ground_station_controller')

        # List of all drone namespaces
        self.drones = ['smaug2']

        # Store publishers/subscribers per drone
        self.subscriptions = {}
        self.publishers = {}
        self.clients = {}
        self.drone_state = {}

        for name in self.drones:
            ns = f'/{name}'

            # 🛰️ Store drone state
            self.drone_state[name] = {
                'pose': None,
                'velocity': None,
                'imu': None,
                'battery': None,
                'state': None
            }

            # ----------------------
            # Subscriptions
            # ----------------------
            self.subscriptions[name] = [
                self.create_subscription(State, f'{ns}/state', lambda msg, n=name: self.state_callback(msg, n), 10),
                self.create_subscription(BatteryState, f'{ns}/battery', lambda msg, n=name: self.battery_callback(msg, n), 10),
                self.create_subscription(Imu, f'{ns}/imu/data', lambda msg, n=name: self.imu_callback(msg, n), 10),
                self.create_subscription(PoseStamped, f'{ns}/local_position/pose', lambda msg, n=name: self.pose_callback(msg, n), 10),
                self.create_subscription(TwistStamped, f'{ns}/local_position/velocity_local', lambda msg, n=name: self.vel_callback(msg, n), 10),
                self.create_subscription(RCIn, f'{ns}/rc/in', lambda msg, n=name: self.rc_in_callback(msg, n), 10)
            ]

            # ----------------------
            # Publishers
            # ----------------------
            self.publishers[name] = {
                'cmd_vel': self.create_publisher(TwistStamped, f'{ns}/setpoint_velocity/cmd_vel', 10),
                'manual': self.create_publisher(ManualControl, f'{ns}/manual_control/send', 10),
                'mode': self.create_publisher(SetMode, f'{ns}/set_mode', 10)
            }

            # ----------------------
            # Service Clients
            # ----------------------
            self.clients[name] = {
                'arming': self.create_client(CommandBool, f'{ns}/cmd/arming')
            }

        self.get_logger().info(f'Initialized controller for drones: {", ".join(self.drones)}')

    # ----------------------
    # Callbacks
    # ----------------------
    def state_callback(self, msg, drone):
        self.drone_state[drone]['state'] = msg

    def battery_callback(self, msg, drone):
        self.drone_state[drone]['battery'] = msg

    def imu_callback(self, msg, drone):
        self.drone_state[drone]['imu'] = msg

    def pose_callback(self, msg, drone):
        self.drone_state[drone]['pose'] = msg

    def vel_callback(self, msg, drone):
        self.drone_state[drone]['velocity'] = msg

    def rc_in_callback(self, msg, drone):
        # Optional: process RC input per drone
        pass

    # ----------------------
    # Command functions
    # ----------------------
    def send_velocity_command(self, drone, vx, vy, vz):
        msg = TwistStamped()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self.publishers[drone]['cmd_vel'].publish(msg)

    def arm_drone(self, drone):
        req = CommandBool.Request(value=True)
        future = self.clients[drone]['arming'].call_async(req)
        return future
    

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
