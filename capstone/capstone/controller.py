import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState, Imu
from mavros_msgs.msg import State, RCIn
from mavros_msgs.srv import CommandBool, SetMode
import threading
import time

class DroneState:
    """
    Class for drone states.
    """
    ARM = 'ARM'
    WAYPOINT = 'WAYPOINT'
    HOVER = 'HOVER'
    LANDING_ZONE = 'LANDING_ZONE'
    LAND = 'LAND'
    RC_TAKEOVER = 'RC_TAKEOVER'


class Controller(Node):
    """
    A RO2 node that controls multiple drone's decision trees.
    
    This class subscribes to various topics to monitor drone states including position,
    velocity, IMU data, battery status, and RC inputs. The node relies on a state machine
    approach:
    1. Move to waypoints.
    2. Make decisions based on mission parameters.
    3. Hover.
    4. Look for landing platforms.
    5. Land when prompted, when battery is low, or when mission is complete.

    Publishes velocity commands and position targets to the drones.
    """
    def __init__(self, drone_names):
        super().__init__('controller')   # Initialize the ROS2 node with the name 'controller'
        
        qos = QoSProfile(depth=10)       # Sets message queue depth
        self.drones = drone_names
        self.state = {}                  # Tracks per-drone state
        self.target_waypoints = {}       # Target waypoints per drone

        # Initialize per-drone state and topics
        for name in self.drones:
            ns = f'/{name}'

            # Store state information in a dictionary
            self.state[name] = {
                'pose': None,
                'velocity': None,
                'imu': None,
                'battery': None,
                'state': None,
                'rc': None,
                'current_state': DroneState.ARM,
                'armed': False,
                'manual_override': False
            }

            # -----------------------------
            # MAVROS TOPIC SUBSCRIPTIONS (ROS → MAVLINK telemetry)
            # -----------------------------
            self.create_subscription(State, f'{ns}/state', lambda msg, n=name: self.state_callback(msg, n), qos)
            self.create_subscription(PoseStamped, f'{ns}/local_position/pose', lambda msg, n=name: self.pose_callback(msg, n), qos)
            self.create_subscription(TwistStamped, f'{ns}/local_position/velocity_local', lambda msg, n=name: self.vel_callback(msg, n), qos)
            self.create_subscription(Imu, f'{ns}/imu/data', lambda msg, n=name: self.imu_callback(msg, n), qos)
            self.create_subscription(BatteryState, f'{ns}/battery', lambda msg, n=name: self.battery_callback(msg, n), qos)
            self.create_subscription(RCIn, f'{ns}/rc/in', lambda msg, n=name: self.rc_callback(msg, n), qos)

            # -----------------------------
            # PUBLISHERS (Commands → Drone)
            # -----------------------------
            # Position commands
            self.pose_target_pub = self.create_publisher(PoseStamped, f'{ns}/setpoint_position/local', qos)
            # Velocity commands
            self.cmd_vel_pub = self.create_publisher(TwistStamped, f'{ns}/setpoint_velocity/cmd_vel', qos)

            # -----------------------------
            # SERVICES (Commands that require a response)
            # -----------------------------
            # Arming motors
            self.arming_client = self.create_client(CommandBool, f'{ns}/cmd/arming')
            # Set flight mode
            self.mode_client = self.create_client(SetMode, f'{ns}/set_mode')

        # -----------------------------
        # THREADS
        # -----------------------------
        # Launch state thread
        self.state_thread = threading.Thread(target=self.state_machine_loop, daemon=True)
        self.state_thread.start()

        # Launch user input thread
        self.input_thread = threading.Thread(target=self.user_input_loop, daemon=True)
        self.input_thread.start()

        self.get_logger().info("Ground Station Controller initialized for drones: " + ', '.join(self.drones))

    # ===========================================================
    # CALLBACKS — triggered whenever a subscribed topic updates
    # ===========================================================
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
        # Fail-safe: Land if low battery
        if msg.percentage < 0.20 and not self.state[drone]['manual_override']:
            self.get_logger().warn(f"[{drone}] Low battery! Initiating landing.")
            self.state[drone]['current_state'] = DroneState.LAND

    def rc_callback(self, msg, drone):
        self.state[drone]['rc'] = msg
        # Detect RC takeover if throttle channel is above threshold
        if any(ch > 1500 for ch in msg.channels[:4]):  # example: first 4 channels active
            self.get_logger().warn(f"[{drone}] RC signal detected! Giving manual control.")
            self.state[drone]['current_state'] = DroneState.RC_TAKEOVER
            self.state[drone]['manual_override'] = True

    # ===========================================================
    # MAVROS COMMAND FUNCTIONS
    # ===========================================================
    def arm(self, drone):
        """
        Send arming command to the drone.
        """
        if not self.arming_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"[{drone}] Arming service unavailable.")
            return
        req = CommandBool.Request(value=True)
        self.arming_client.call_async(req)
        self.get_logger().info(f"[{drone}] Sending arming request...")

    def set_mode(self, drone, mode):
        """
        Set flight mode.
        """
        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"[{drone}] Mode service unavailable.")
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self.mode_client.call_async(req)
        self.get_logger().info(f"[{drone}] Setting mode to {mode}...")

    def send_position_target(self, drone, x, y, z):
        """
        Send position target to the drone.
        """
        msg = PoseStamped()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.pose_target_pub.publish(msg)

    def send_velocity_command(self, drone, vx, vy, vz):
        """
        Send velocity command to the drone.
        """
        msg = TwistStamped()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self.cmd_vel_pub.publish(msg)

    # ===========================================================
    # USER INPUT LOOP (runs in its own thread)
    # ===========================================================
    def user_input_loop(self):
        """
        Allows manual input in the terminal.
        Example Commands:
          smaug1 5 5 3    → send smaug1 to (x=5, y=5, z=3)
          land smaug2     → land drone smaug2
        """
        while True:
            try:
                text = input("\nEnter waypoint (drone_name x y z) or 'land <drone>': ").strip()
                if text.lower().startswith('land'):
                    _, drone = text.split()
                    if drone in self.drones:
                        self.state[drone]['current_state'] = DroneState.LAND
                        self.get_logger().info(f"[{drone}] Manual landing triggered.")
                else:
                    drone, x, y, z = text.split()
                    self.target_waypoints[drone] = (float(x), float(y), float(z))
                    self.state[drone]['current_state'] = DroneState.WAYPOINT
                    self.get_logger().info(f"[{drone}] New waypoint set to ({x}, {y}, {z}).")
            except Exception as e:
                print(f"Input error: {e}")

    # ===========================================================
    # MAIN STATE MACHINE LOOP (runs continuously)
    # ===========================================================
    def state_machine_loop(self):
        """
        Each drone independently transitions between states.
        This loop runs continuously and publishes commands accordingly.
        """
        rate = 2.0  # Hz
        while rclpy.ok():
            for drone in self.drones:
                s = self.state[drone]
                current = s['current_state']

                # Skip drones that have been taken over by RC
                if s['manual_override']:
                    continue
                
                # -------------------------
                # STATE LOGIC
                # -------------------------

                # Arm state
                if current == DroneState.ARM:
                    self.arm(drone)
                    self.set_mode(drone, "OFFBOARD")
                    s['current_state'] = DroneState.HOVER

                # Waypoint state
                elif current == DroneState.WAYPOINT:
                    if drone in self.target_waypoints:
                        x, y, z = self.target_waypoints[drone]
                        self.send_position_target(drone, x, y, z)
                        self.get_logger().info(f"[{drone}] Moving to waypoint ({x}, {y}, {z}).")
                        s['current_state'] = DroneState.HOVER

                # Hover state
                elif current == DroneState.HOVER:
                    # Send small velocity command to stay in place
                    self.send_velocity_command(drone, 0.0, 0.0, 0.0)

                # Landing zone state
                elif current == DroneState.LANDING_ZONE:
                    # Example: fly to known landing zone coordinates
                    self.send_position_target(drone, 0.0, 0.0, 2.0)
                    s['current_state'] = DroneState.LAND

                # Land state
                elif current == DroneState.LAND:
                    self.set_mode(drone, "AUTO.LAND")
                    self.get_logger().info(f"[{drone}] Landing initiated.")

                # RC Takeover state
                elif current == DroneState.RC_TAKEOVER:
                    self.get_logger().warn(f"[{drone}] Manual RC control. ROS2 disengaged.")
                    continue

            time.sleep(1.0 / rate)      # Control loop rate


def main(args=None):
    """
    Main entrt point of the node. Initializes and runs the Controller node.
    """
    rclpy.init(args=args)   # Initialize ROS2
    drone_names = [f'smaug{i}' for i in range(1, 5)]
    node = Controller(drone_names)  # Create Controller node instance

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Ground Station Controller.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  # Execute the script