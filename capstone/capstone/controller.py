import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
import math
from sensor_msgs.msg import BatteryState, Imu
from mavros_msgs.msg import State, RCIn
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import threading
import time

class DroneState:
    ARM = 'ARM'
    WAYPOINT = 'WAYPOINT'
    HOVER = 'HOVER'
    LAND = 'LAND'
    RC_TAKEOVER = 'RC_TAKEOVER'

class Controller(Node):
    def __init__(self, drone_names):
        super().__init__('controller')

        # Use BEST_EFFORT QoS to match MAVROS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.drones = drone_names
        self.state = {}  
        self.target_waypoints = {}
        self.last_status_check = {}  # Track when we last checked drone status

        # Initialize drones
        for name in self.drones:
            ns = f'/{name}'
            self.state[name] = {
                'pose': None,
                'velocity': None,
                'imu': None,
                'battery': None,
                'state': None,
                'rc': None,
                'current_state': DroneState.ARM,
                'armed': False,
                'manual_override': False,
                'target_heading': None,
                'ready_to_arm': False,
                'landing_attempts': 0,
                'landing_initiated': False,
                'last_mode_warn': 0.0,
                'low_battery_warned': False  # Track if we've already warned about low battery
            }

            # Subscriptions
            self.create_subscription(State, f'{ns}/state', lambda msg, n=name: self.state_callback(msg, n), qos)
            self.create_subscription(PoseStamped, f'{ns}/local_position/pose', lambda msg, n=name: self.pose_callback(msg, n), qos)
            self.create_subscription(TwistStamped, f'{ns}/local_position/velocity_local', lambda msg, n=name: self.vel_callback(msg, n), qos)
            self.create_subscription(Imu, f'{ns}/imu/data', lambda msg, n=name: self.imu_callback(msg, n), qos)
            self.create_subscription(BatteryState, f'{ns}/battery', lambda msg, n=name: self.battery_callback(msg, n), qos)
            self.create_subscription(RCIn, f'{ns}/rc/in', lambda msg, n=name: self.rc_callback(msg, n), qos)

            # Publishers
            # Create publishers on common topic variants so setpoints reach MAVROS whatever remapping is used
            pose_pubs = []
            vel_pubs = []
            try:
                pose_pubs.append(self.create_publisher(PoseStamped, f'{ns}/setpoint_position/local', qos))
            except Exception:
                pass
            try:
                pose_pubs.append(self.create_publisher(PoseStamped, f'{ns}/mavros/setpoint_position/local', qos))
            except Exception:
                pass
            try:
                vel_pubs.append(self.create_publisher(TwistStamped, f'{ns}/setpoint_velocity/cmd_vel', qos))
            except Exception:
                pass
            try:
                vel_pubs.append(self.create_publisher(TwistStamped, f'{ns}/mavros/setpoint_velocity/cmd_vel', qos))
            except Exception:
                pass
            self.state[name]['pose_pubs'] = pose_pubs
            self.state[name]['vel_pubs'] = vel_pubs

            # Services: try to locate the correct service name among common remappings
            def _create_client_from_candidates(srv_type, candidates):
                for cand in candidates:
                    try:
                        client = self.create_client(srv_type, cand)
                        # quick non-blocking check if service exists
                        if client.wait_for_service(timeout_sec=0.2):
                            self.get_logger().info(f"[{name}] Using service: {cand}")
                            return client
                    except Exception:
                        continue
                # fallback: create client with first candidate (will be waited on later)
                return self.create_client(srv_type, candidates[0])

            arming_candidates = [f'{ns}/cmd/arming', f'{ns}/mavros/cmd/arming', f'/mavros{ns}/cmd/arming', f'/mavros/cmd/arming']
            mode_candidates = [f'{ns}/set_mode', f'{ns}/mavros/set_mode', f'/mavros{ns}/set_mode', f'/mavros/set_mode']
            takeoff_candidates = [f'{ns}/cmd/takeoff', f'{ns}/mavros/cmd/takeoff', f'/mavros{ns}/cmd/takeoff', f'/mavros/cmd/takeoff']

            self.state[name]['arming_client'] = _create_client_from_candidates(CommandBool, arming_candidates)
            self.state[name]['mode_client'] = _create_client_from_candidates(SetMode, mode_candidates)
            self.state[name]['takeoff_client'] = _create_client_from_candidates(CommandTOL, takeoff_candidates)

        # Threads
        threading.Thread(target=self.state_machine_loop, daemon=True).start()
        threading.Thread(target=self.user_input_loop, daemon=True).start()

        self.get_logger().info("Controller initialized for drones: " + ", ".join(self.drones))

    # ------------------- Callbacks -------------------
    def state_callback(self, msg, drone):
        if self.state[drone].get('state') is None:
            self.get_logger().info(f"[{drone}] FCU state connection established")
        self.state[drone]['state'] = msg
        
    def pose_callback(self, msg, drone):
        if self.state[drone].get('pose') is None:
            self.get_logger().info(f"[{drone}] Position data received")
        self.state[drone]['pose'] = msg
        
    def vel_callback(self, msg, drone): 
        self.state[drone]['velocity'] = msg
        
    def imu_callback(self, msg, drone): 
        if self.state[drone].get('imu') is None:
            self.get_logger().info(f"[{drone}] IMU data received")
        self.state[drone]['imu'] = msg
    def battery_callback(self, msg, drone):
        # Store previous battery state for change detection
        old_battery = self.state[drone].get('battery')
        self.state[drone]['battery'] = msg
        
        # Log initial battery reading
        if old_battery is None:
            self.get_logger().info(f"[{drone}] Initial battery level: {msg.percentage*100:.1f}%")
        
        # Check for low battery
        if msg.percentage < 0.20:
            # If flying, initiate emergency landing
            if self.state[drone]['current_state'] in [DroneState.HOVER, DroneState.WAYPOINT]:
                if not self.state[drone].get('low_battery_warned'):
                    self.get_logger().warn(f"[{drone}] Low battery ({msg.percentage*100:.1f}%)! Initiating emergency landing.")
                    self.state[drone]['low_battery_warned'] = True
                self.state[drone]['current_state'] = DroneState.LAND
                self.state[drone]['emergency_landing'] = True
            else:
                # If we're not flying, warn once about low battery preventing takeoff
                if not self.state[drone].get('low_battery_warned'):
                    self.get_logger().warn(f"[{drone}] Battery level too low for takeoff ({msg.percentage*100:.1f}%)")
                    self.get_logger().warn(f"[{drone}] Minimum 20% battery required for flight operations")
                    self.state[drone]['low_battery_warned'] = True
        else:
            # Reset warning flag if battery is back above 20%
            self.state[drone]['low_battery_warned'] = False
    def rc_callback(self, msg, drone):
        self.state[drone]['rc'] = msg
        if any(ch > 1500 for ch in msg.channels[:4]):
            self.get_logger().warn(f"[{drone}] RC takeover detected.")
            self.state[drone]['current_state'] = DroneState.RC_TAKEOVER
            self.state[drone]['manual_override'] = True

    # ------------------- MAVROS Commands -------------------
    def arm(self, drone):
        client = self.state[drone]['arming_client']
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"[{drone}] Arming service unavailable")
            return False
        req = CommandBool.Request(value=True)
        future = client.call_async(req)
        # Wait for future in a thread-safe manner (avoid spin_until_future_complete from threads)
        if not self._wait_for_future(future, timeout_sec=5.0):
            self.get_logger().error(f"[{drone}] Arming service call timed out")
            return False
        try:
            res = future.result()
            if res and getattr(res, 'success', False):
                self.get_logger().info(f"[{drone}] Armed")
                return True
            else:
                self.get_logger().error(f"[{drone}] Failed to arm: {res}")
                return False
        except Exception as e:
            self.get_logger().error(f"[{drone}] Exception while arming: {e}")
            return False

    def set_mode(self, drone, mode):
        # First check current mode to avoid unnecessary changes
        current_mode = None
        if self.state[drone].get('state') is not None:
            current_mode = getattr(self.state[drone]['state'], 'mode', None)
        if current_mode is not None and mode.upper() in str(current_mode).upper():
            self.get_logger().debug(f"[{drone}] Already in {mode} mode")
            return True

        client = self.state[drone]['mode_client']
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"[{drone}] Mode service unavailable")
            return False
        req = SetMode.Request()
        # be explicit: leave base_mode as 0 and set custom_mode string
        req.custom_mode = mode
        future = client.call_async(req)
        # Wait for future in a thread-safe manner (avoid spin_until_future_complete from threads)
        if not self._wait_for_future(future, timeout_sec=5.0):
            self.get_logger().error(f"[{drone}] set_mode service call timed out")
            return False
        try:
            res = future.result()
            # Log the raw response for debugging why mode change might fail
            self.get_logger().debug(f"[{drone}] set_mode response: {res}")
            if res and getattr(res, 'mode_sent', False):
                self.get_logger().info(f"[{drone}] Mode set to {mode}")
                return True
            else:
                self.get_logger().error(f"[{drone}] Failed to set mode {mode}: {res}")
                return False
        except Exception as e:
            self.get_logger().error(f"[{drone}] Exception while setting mode {mode}: {e}")
            return False

    def takeoff(self, drone, altitude):
        """Use mavros CommandTOL service to request a takeoff to `altitude` meters.

        Returns True on success, False otherwise.
        """
        client = self.state[drone].get('takeoff_client')
        if client is None:
            self.get_logger().error(f"[{drone}] No takeoff service client available")
            return False
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"[{drone}] Takeoff service unavailable")
            return False
        req = CommandTOL.Request()
        # For most autopilots providing a takeoff service, latitude/longitude of 0.0 means "current position"
        try:
            req.altitude = float(altitude)
        except Exception:
            req.altitude = altitude
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw = 0.0

        future = client.call_async(req)
        if not self._wait_for_future(future, timeout_sec=5.0):
            self.get_logger().error(f"[{drone}] Takeoff service call timed out")
            return False
        try:
            res = future.result()
            # Response shape may vary; check for success-like fields
            if res and (getattr(res, 'success', False) or getattr(res, 'result', False)):
                self.get_logger().info(f"[{drone}] Takeoff requested to {altitude} m: {res}")
                return True
            else:
                self.get_logger().error(f"[{drone}] Takeoff failed: {res}")
                return False
        except Exception as e:
            self.get_logger().error(f"[{drone}] Exception during takeoff: {e}")
            return False
    def ascend_fallback(self, drone, target_z, vz=0.6, timeout=15.0):
        """Publish an upward velocity until target_z is reached or timeout.

        Returns True if target_z reached, False otherwise.
        """
        start = time.time()
        self.get_logger().info(f"[{drone}] Starting ascend fallback to {target_z} m (vz={vz} m/s)")
        current_z = 0.0
        while time.time() - start < timeout:
            pose = self.state[drone].get('pose')
            if pose is not None:
                current_z = getattr(pose.pose.position, 'z', 0.0)
            if current_z >= target_z - 0.1:
                self.get_logger().info(f"[{drone}] Reached target altitude: {current_z:.2f} m")
                # stop vertical motion
                self.send_velocity(drone, 0.0, 0.0, 0.0)
                return True
            # command upward velocity
            self.send_velocity(drone, 0.0, 0.0, -vz)  # mavros velocity z is often NED; negative vz => up for some setups
            # Also publish a position setpoint at same altitude to help PX4 offboard controllers (best-effort)
            try:
                self.send_position(drone, self.target_waypoints.get(drone, (0, 0, target_z))[0],
                                   self.target_waypoints.get(drone, (0, 0, target_z))[1], current_z + 0.5)
            except Exception:
                pass
            time.sleep(0.2)

        # timeout
        self.get_logger().warn(f"[{drone}] Ascend fallback timed out after {timeout} s (last z={current_z:.2f})")
        # stop motion
        self.send_velocity(drone, 0.0, 0.0, 0.0)
        return False

    def _wait_for_future(self, future, timeout_sec=5.0):
        """Wait for an rclpy future to complete in a thread-safe way.

        Returns True if the future completed before timeout, False otherwise.
        """
        start = time.time()
        # Poll the future and spin once to process events
        while rclpy.ok() and not future.done():
            try:
                rclpy.spin_once(self, timeout_sec=0.1)
            except Exception:
                # In rare cases spin_once may throw if executor is busy; ignore and continue polling
                pass
            if time.time() - start > timeout_sec:
                return False
        return future.done()

    def send_position(self, drone, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # or local_origin
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        # Keep current orientation if available to avoid sudden heading changes
        pose = self.state[drone].get('pose')
        if pose is not None and hasattr(pose.pose, 'orientation'):
            msg.pose.orientation = pose.pose.orientation
        else:
            # Default to no rotation if we don't have current orientation
            msg.pose.orientation.w = 1.0
        pubs = self.state[drone].get('pose_pubs') or []
        if not pubs:
            self.get_logger().warn(f"[{drone}] No pose publishers available to send setpoint")
            return
        for pub in pubs:
            try:
                pub.publish(msg)
            except Exception:
                # ignore publish errors for individual pubs
                pass

    def _yaw_to_quaternion(self, yaw_rad):
        """Return quaternion (x,y,z,w) for a yaw angle in radians (rotation about Z)."""
        half = yaw_rad * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def set_heading(self, drone, yaw_deg):
        """Publish a PoseStamped with orientation set to yaw_deg so the vehicle faces that heading.

        This uses existing position (or target waypoint) to avoid moving position.
        """
        yaw_rad = math.radians(float(yaw_deg))
        qx, qy, qz, qw = self._yaw_to_quaternion(yaw_rad)

        # Use current pose if available, otherwise use target waypoint or zero
        pose = self.state[drone].get('pose')
        if pose is not None:
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
        else:
            wp = self.target_waypoints.get(drone, (0.0, 0.0, 0.0))
            x, y, z = wp

        msg = PoseStamped()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        pubs = self.state[drone].get('pose_pubs') or []
        if not pubs:
            self.get_logger().warn(f"[{drone}] No pose publishers available to set heading")
            return
        for pub in pubs:
            try:
                pub.publish(msg)
            except Exception:
                pass

    def send_velocity(self, drone, vx, vy, vz):
        msg = TwistStamped()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        pubs = self.state[drone].get('vel_pubs') or []
        if not pubs:
            self.get_logger().warn(f"[{drone}] No velocity publishers available to send setpoint")
            return
        for pub in pubs:
            try:
                pub.publish(msg)
            except Exception:
                pass

    # ------------------- User Input -------------------
    def display_command_options(self):
        print("\nController ready. Available commands:")
        print(f"Available drones: {', '.join(self.drones)}")
        print("\nCommands:")
        print("  Waypoint:  <drone> <x> <y> <z>      (Example: sim1 1.0 2.0 3.0)")
        print("  Heading:   heading <drone> <deg>     (Example: heading sim1 90)")
        print("  Landing:   land <drone>              (Example: land sim1)")
        print("\nWaiting for commands...")

    def user_input_loop(self):
        while True:
            try:
                # Add decoration and newlines to make prompt more visible
                text = input("\n>>> Command> ").strip()
                parts = text.split()
                if not parts:
                    self.display_command_options()
                    continue

                # For waypoint commands, first part is the drone name
                # For other commands (land, heading), second part is the drone name
                cmd = parts[0].lower()
                if cmd in ['land', 'heading']:
                    drone = parts[1] if len(parts) > 1 else None
                else:
                    # Waypoint command - first part is drone name
                    drone = parts[0]
                    
                if not drone or drone not in self.drones:
                    self.get_logger().error(f"Unknown drone name '{drone}'. Valid drones: {', '.join(self.drones)}")
                    self.display_command_options()
                    continue

                # Check if drone is in emergency landing
                if self.state[drone].get('emergency_landing', False):
                    self.get_logger().warn(f"[{drone}] Cannot accept commands - emergency landing in progress")
                    self.display_command_options()
                    continue

                # Check if we have received necessary status information
                if not all([self.state[drone].get(key) for key in ['state', 'battery', 'pose']]):
                    self.get_logger().error(f"[{drone}] Cannot process command - waiting for drone to initialize")
                    self.display_command_options()
                    continue

                # Process commands
                if cmd == 'land' and len(parts) == 2:
                    self.state[drone]['current_state'] = DroneState.LAND
                    self.get_logger().info(f"[{drone}] Landing command received.")
                elif cmd == 'heading' and len(parts) == 3:
                    try:
                        degf = float(parts[2])
                    except ValueError:
                        self.get_logger().error("Invalid heading value")
                        self.display_command_options()
                        continue
                    self.state[drone]['target_heading'] = degf
                    # immediately publish a heading setpoint burst so it takes effect
                    for _ in range(10):
                        self.set_heading(drone, degf)
                        time.sleep(0.05)
                    self.get_logger().info(f"[{drone}] Heading set to {degf} deg.")
                    self.display_command_options()
                elif len(parts) == 4:  # waypoint command format: drone x y z
                    try:
                        # Skip drone name validation as it's already done above
                        x, y, z = map(float, parts[1:4])  # get coordinates from parts 1-3
                        self.get_logger().info(f"[{drone}] Processing waypoint command: ({x}, {y}, {z})")
                    except ValueError:
                        self.get_logger().error(f"[{drone}] Invalid coordinates. Must be numbers (x y z)")
                        self.display_command_options()
                        continue

                    # Check battery level before allowing arming
                    battery = self.state[drone].get('battery')
                    if battery and battery.percentage < 0.20:  # 20% battery threshold
                        self.get_logger().error(f"[{drone}] Cannot execute command - battery level too low ({battery.percentage*100:.1f}%)")
                        self.display_command_options()
                        continue
                    
                    self.target_waypoints[drone] = (x, y, z)
                    self.state[drone]['ready_to_arm'] = True
                    self.state[drone]['current_state'] = DroneState.ARM  # Ensure we're in ARM state
                    self.get_logger().info(f"[{drone}] Waypoint set to ({x}, {y}, {z}). Ready to arm.")
                else:
                    self.get_logger().error('Invalid input format')
                    self.display_command_options()
            except Exception as e:
                print("Input error:", e)

    # ------------------- State Machine -------------------
    def state_machine_loop(self):
        rate = 20.0  # Hz
        while rclpy.ok():
            for drone in self.drones:
                s = self.state[drone]

                # Periodically check and report drone status if we're waiting for initialization
                now = time.time()
                if not all([s.get(key) for key in ['state', 'battery', 'pose']]):
                    if now - self.last_status_check.get(drone, 0.0) > 5.0:  # Check every 5 seconds
                        missing = []
                        if not s.get('state'): missing.append('FCU state')
                        if not s.get('battery'): missing.append('battery status')
                        if not s.get('pose'): missing.append('position data')
                        self.get_logger().warn(f"[{drone}] Still waiting for: {', '.join(missing)}")
                        self.last_status_check[drone] = now

                if s['manual_override']:
                    continue

                # ensure we have a connection/state before trying to arm or change mode
                if s['state'] is None or not getattr(s['state'], 'connected', True):
                    # no valid state yet
                    self.get_logger().debug(f"[{drone}] Waiting for FCU connection/state...")
                    continue

                # Arm + OFFBOARD after waypoint
                if s['current_state'] == DroneState.ARM and s['ready_to_arm']:
                    # Double-check battery level before arming
                    battery = s.get('battery')
                    if battery and battery.percentage < 0.20:  # 20% battery threshold
                        self.get_logger().error(f"[{drone}] Aborting takeoff - battery too low ({battery.percentage*100:.1f}%)")
                        self.get_logger().error(f"[{drone}] Battery must be above 20% for safe flight operations")
                        s['ready_to_arm'] = False
                        s['current_state'] = DroneState.ARM
                        if drone in self.target_waypoints:
                            del self.target_waypoints[drone]
                        self.display_command_options()
                        continue

                    x, y, z = self.target_waypoints[drone]

                    # First ensure we're in STABILIZE mode for arming
                    if not self.set_mode(drone, "STABILIZE"):
                        self.get_logger().error(f"[{drone}] Could not set STABILIZE mode, retrying...")
                        continue

                    # Wait for mode change to take effect
                    start = time.time()
                    mode_changed = False
                    while time.time() - start < 3.0:  # Wait up to 3 seconds
                        fcu_mode = None
                        if s.get('state') is not None:
                            fcu_mode = getattr(s['state'], 'mode', None)
                        if fcu_mode is not None and 'STABILIZE' in str(fcu_mode).upper():
                            mode_changed = True
                            self.get_logger().info(f"[{drone}] Verified mode change to STABILIZE")
                            break
                        time.sleep(0.1)
                    
                    if not mode_changed:
                        self.get_logger().warn(f"[{drone}] Failed to verify STABILIZE mode (current={fcu_mode})")
                        continue

                    # Try to arm and verify arming state
                    armed = self.arm(drone)
                    if not armed:
                        self.get_logger().error(f"[{drone}] Could not arm, retrying...")
                        continue

                    # Verify arming state
                    start = time.time()
                    arm_verified = False
                    while time.time() - start < 3.0:  # Wait up to 3 seconds
                        if s.get('state') is not None and getattr(s['state'], 'armed', False):
                            arm_verified = True
                            self.get_logger().info(f"[{drone}] Verified armed state")
                            break
                        time.sleep(0.1)

                    if not arm_verified:
                        self.get_logger().error(f"[{drone}] Failed to verify armed state")
                        continue

                    # Go directly to GUIDED mode for flight
                    if self.set_mode(drone, "GUIDED"):
                        # Request takeoff to waypoint altitude
                        tookoff = self.takeoff(drone, z)
                        if tookoff:
                            self.get_logger().info(f"[{drone}] Takeoff command accepted, waiting for altitude increase")
                            
                            # Wait for actual altitude change (up to 10 seconds)
                            start = time.time()
                            initial_z = None
                            takeoff_successful = False
                            
                            while time.time() - start < 10.0:
                                pose = s.get('pose')
                                if pose is not None:
                                    current_z = getattr(pose.pose.position, 'z', 0.0)
                                    if initial_z is None:
                                        initial_z = current_z
                                        self.get_logger().info(f"[{drone}] Initial altitude: {initial_z:.2f} m")
                                    elif current_z > initial_z + 0.5:  # Require 0.5m increase
                                        takeoff_successful = True
                                        self.get_logger().info(f"[{drone}] Takeoff verified - altitude increased to {current_z:.2f} m")
                                        break
                                time.sleep(0.2)
                            
                            if takeoff_successful:
                                s['current_state'] = DroneState.WAYPOINT  # Go directly to WAYPOINT state
                                self.get_logger().info(f"[{drone}] Takeoff complete, starting movement to waypoint")
                                # Calculate and set initial heading to waypoint
                                pose = s.get('pose')
                                if pose is not None:
                                    current_x = getattr(pose.pose.position, 'x', 0.0)
                                    current_y = getattr(pose.pose.position, 'y', 0.0)
                                    dx = x - current_x
                                    dy = y - current_y
                                    target_heading = math.degrees(math.atan2(dy, dx))
                                    try:
                                        self.set_heading(drone, target_heading)
                                        self.get_logger().info(f"[{drone}] Setting initial heading to {target_heading:.1f} degrees")
                                    except Exception as e:
                                        self.get_logger().warn(f"[{drone}] Could not set initial heading: {e}")
                            else:
                                self.get_logger().warn(f"[{drone}] Takeoff command accepted but no altitude increase detected. Trying fallback...")
                                if self.ascend_fallback(drone, z):
                                    s['current_state'] = DroneState.HOVER
                                else:
                                    self.get_logger().error(f"[{drone}] Takeoff failed - could not achieve altitude")
                        else:
                            self.get_logger().error(f"[{drone}] Takeoff command failed")
                    else:
                        self.get_logger().error(f"[{drone}] Mode change to GUIDED failed. Staying in {s['current_state']}")

                # Move to waypoint
                elif s['current_state'] == DroneState.HOVER:
                    if drone in self.target_waypoints:
                        x, y, z = self.target_waypoints[drone]
                        s['current_state'] = DroneState.WAYPOINT  # Switch to WAYPOINT state for movement
                        self.get_logger().info(f"[{drone}] Starting movement to waypoint ({x}, {y}, {z})")
                        # Ensure mode is correct at the start of movement
                        if not self.set_mode(drone, "GUIDED"):
                            self.get_logger().error(f"[{drone}] Could not set GUIDED mode for waypoint control")

                # Handle waypoint movement
                elif s['current_state'] == DroneState.WAYPOINT:
                    if drone in self.target_waypoints:
                        x, y, z = self.target_waypoints[drone]
                        
                        # Verify GUIDED mode
                        current_mode = None
                        if s.get('state') is not None:
                            current_mode = getattr(s['state'], 'mode', '')
                        if current_mode is None or 'GUIDED' not in str(current_mode).upper():
                            if not self.set_mode(drone, "GUIDED"):
                                self.get_logger().error(f"[{drone}] Lost GUIDED mode, retrying...")
                                continue

                        # Get current position
                        pose = s.get('pose')
                        if pose is not None:
                            current_x = getattr(pose.pose.position, 'x', 0.0)
                            current_y = getattr(pose.pose.position, 'y', 0.0)
                            current_z = getattr(pose.pose.position, 'z', 0.0)

                            # Calculate distance and direction
                            dx = x - current_x
                            dy = y - current_y
                            dz = z - current_z
                            dist = math.sqrt(dx*dx + dy*dy + dz*dz)

                            # Use both position and velocity control for smoother movement
                            self.send_position(drone, x, y, z)  # Send target position

                            # More aggressive velocity control
                            max_speed = 8.0  # m/s - Significantly increased
                            min_speed = 2.0  # m/s - Higher minimum speed
                            
                            # Proportional control with distance zones
                            if dist > 20.0:  # Far from target
                                speed = max_speed
                            elif dist > 5.0:  # Medium distance
                                speed = max(min_speed * 2, dist * 0.5)  # Faster approach
                            else:  # Close to target
                                speed = max(min_speed, dist * 0.8)  # Gentler deceleration
                            
                            if dist > 0.1:  # Only send velocity if we're not too close
                                # Calculate velocity components
                                vx = (dx/dist) * speed
                                vy = (dy/dist) * speed
                                vz = (dz/dist) * speed
                                
                                # Send both velocity and position setpoints for better control
                                self.send_velocity(drone, vx, vy, vz)
                                # More frequent position updates
                                for _ in range(2):
                                    self.send_position(drone, x, y, z)
                                    
                                # Debug output for velocities
                                if now - s.get('last_debug_log', 0) > 2.0:  # Every 2 seconds
                                    self.get_logger().info(f"[{drone}] Commanded velocity: vx={vx:.1f}, vy={vy:.1f}, vz={vz:.1f} m/s")
                                    s['last_debug_log'] = now
                            
                            # Calculate distance to target
                            dist = math.sqrt((x - current_x)**2 + (y - current_y)**2 + (z - current_z)**2)
                            
                            # Log progress periodically
                            now = time.time()
                            if now - s.get('last_progress_log', 0) > 2.0:  # Log every 2 seconds
                                self.get_logger().info(f"[{drone}] Distance to waypoint: {dist:.2f} m")
                                s['last_progress_log'] = now
                            
                            # Check both position and velocity for arrival
                            vel = s.get('velocity')
                            current_speed = 0.0
                            if vel:
                                vx = getattr(vel.twist.linear, 'x', 0.0)
                                vy = getattr(vel.twist.linear, 'y', 0.0)
                                vz = getattr(vel.twist.linear, 'z', 0.0)
                                current_speed = math.sqrt(vx*vx + vy*vy + vz*vz)

                            # If we're close enough and nearly stopped
                            if dist < 0.5 and current_speed < 0.1:  # within 0.5 meters and almost stopped
                                # Send zero velocity to ensure we stop
                                self.send_velocity(drone, 0.0, 0.0, 0.0)
                                self.get_logger().info(f"[{drone}] Successfully reached waypoint ({x}, {y}, {z})")
                                s['current_state'] = DroneState.HOVER  # Return to HOVER state
                                self.get_logger().info(f"[{drone}] Ready for next command")
                                self.display_command_options()
                            else:
                                # Calculate and update heading while moving
                                dx = x - current_x
                                dy = y - current_y
                                target_heading = math.degrees(math.atan2(dy, dx))
                                try:
                                    self.set_heading(drone, target_heading)
                                except Exception:
                                    pass
                                # Log movement progress more frequently
                                now = time.time()
                                if now - s.get('last_progress_log', 0) > 1.0:  # Log every second
                                    self.get_logger().info(
                                        f"[{drone}] Moving to waypoint: distance={dist:.2f}m, "
                                        f"speed={current_speed:.2f}m/s, "
                                        f"heading={target_heading:.1f}°"
                                    )
                    # maintain explicit heading if requested
                    elif s.get('target_heading') is not None:
                        try:
                            self.set_heading(drone, s['target_heading'])
                        except Exception:
                            pass

                # Landing
                elif s['current_state'] == DroneState.LAND:
                    # Try common land mode names (PX4: AUTO.LAND, ArduPilot: LAND)
                    if not s.get('landing_initiated'):
                        max_attempts = 3
                        now = time.time()
                        # allow attempts spaced by 2 seconds
                        if s.get('landing_attempts', 0) >= max_attempts and now - s.get('last_landing_time', 0.0) < 10.0:
                            self.get_logger().warn(f"[{drone}] Landing attempts exhausted; will not retry immediately.")
                        else:
                            # attempt landing modes in order
                            landed = False
                            for land_mode in ("LAND", "RTL", "AUTO.LAND"):  # Try LAND first as it's more direct
                                if self.set_mode(drone, land_mode):
                                    self.get_logger().info(f"[{drone}] Land mode set to {land_mode}")
                                    landed = True
                                    s['landing_initiated'] = True
                                    break
                                else:
                                    # log current FCU mode for debugging (rate-limited)
                                    current_mode = None
                                    if s.get('state') is not None:
                                        current_mode = getattr(s['state'], 'mode', None)
                                    self.get_logger().warn(f"[{drone}] Failed to set mode {land_mode}. FCU mode: {current_mode}")

                            s['landing_attempts'] = s.get('landing_attempts', 0) + 1
                            s['last_landing_time'] = now
                            if not landed:
                                self.get_logger().error(f"[{drone}] Could not initiate landing with any common mode.")
                    
                    # Check if we've landed
                    if s.get('landing_initiated'):
                        pose = s.get('pose')
                        if pose is not None:
                            current_z = getattr(pose.pose.position, 'z', 0.0)
                            if current_z < 0.1:  # Consider landed when below 0.1m
                                self.get_logger().info(f"[{drone}] Landing complete")
                                # Reset all state for next command
                                s['current_state'] = DroneState.ARM
                                s['ready_to_arm'] = False
                                s['landing_attempts'] = 0
                                s['landing_initiated'] = False
                                s['last_mode_warn'] = 0.0
                                s['target_heading'] = None
                                s['emergency_landing'] = False  # Clear emergency flag
                                # Clear any existing waypoint
                                if drone in self.target_waypoints:
                                    del self.target_waypoints[drone]
                                # Display commands again
                                self.display_command_options()

            time.sleep(1.0 / rate)

def main(args=None):
    try:
        rclpy.init(args=args)
        
        while True:
            drone_names = input("Enter drone names (space separated): ").strip().split()
            if not drone_names:
                print("No drones entered. Exiting.")
                return
            
            # Validate drone names: should start with letters, not numbers
            invalid_names = [name for name in drone_names if name[0].isdigit()]
            if invalid_names:
                print("\nError: Invalid drone names. Names must not start with numbers:")
                print("  Invalid names:", ", ".join(invalid_names))
                print("\nExample valid names: sim1, drone2, uav3")
                continue
            break

        node = Controller(drone_names)
        # Give ROS logger time to print startup messages
        time.sleep(0.5)
        node.display_command_options()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print("\nShutdown requested by user")
        finally:
            node.destroy_node()

    except Exception as e:
        print(f"Error during execution: {e}")
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Ignore shutdown errors as ROS might already be shutdown

if __name__ == "__main__":
    main()
