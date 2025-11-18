import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from geographic_msgs.msg import GeoPoseStamped
import math
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from mavros_msgs.msg import State, RCIn
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import threading
import time

class DroneState:
    ARM = 'ARM'
    WAYPOINT = 'WAYPOINT'
    HOVER = 'HOVER'
    LAND = 'LAND'
    CIRCLE = 'CIRCLE'
    RC_TAKEOVER = 'RC_TAKEOVER'

class Controller(Node):
    def __init__(self, drone_names):
        super().__init__('controller')

        # Use BEST_EFFORT QoS to match MAVROS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.drones = drone_names
        self.state = {}
        self.target_waypoints = {}
        self.circle_waypoints = {}  # Queue of waypoints for circle flight
        self.last_status_check = {}  # Track when we last checked drone status        # Initialize drones
        for name in self.drones:
            ns = f'/{name}'
            self.state[name] = {
                'pose': None,
                'velocity': None,
                'imu': None,
                'battery': None,
                'state': None,
                'rc': None,
                'gps': None,
                'current_state': DroneState.ARM,
                'armed': False,
                'manual_override': False,
                'target_heading': None,
                'ready_to_arm': False,
                'landing_attempts': 0,
                'landing_initiated': False,
                'last_mode_warn': 0.0,
                'low_battery_warned': False
            }

            # Subscriptions
            self.create_subscription(State, f'{ns}/state', lambda msg, n=name: self.state_callback(msg, n), qos)
            self.create_subscription(PoseStamped, f'{ns}/local_position/pose', lambda msg, n=name: self.pose_callback(msg, n), qos)
            self.create_subscription(NavSatFix, f'{ns}/global_position/global', lambda msg, n=name: self.gps_callback(msg, n), qos)
            self.create_subscription(TwistStamped, f'{ns}/local_position/velocity_local', lambda msg, n=name: self.vel_callback(msg, n), qos)
            self.create_subscription(Imu, f'{ns}/imu/data', lambda msg, n=name: self.imu_callback(msg, n), qos)
            self.create_subscription(BatteryState, f'{ns}/battery', lambda msg, n=name: self.battery_callback(msg, n), qos)
            self.create_subscription(RCIn, f'{ns}/rc/in', lambda msg, n=name: self.rc_callback(msg, n), qos)

            # Publishers
            pose_pubs = []
            gps_pubs = []
            try:
                pose_pubs.append(self.create_publisher(PoseStamped, f'{ns}/setpoint_position/local', qos))
            except Exception:
                pass
            try:
                pose_pubs.append(self.create_publisher(PoseStamped, f'{ns}/mavros/setpoint_position/local', qos))
            except Exception:
                pass
            try:
                gps_pubs.append(self.create_publisher(GeoPoseStamped, f'{ns}/setpoint_position/global', qos))
            except Exception:
                pass
            try:
                gps_pubs.append(self.create_publisher(GeoPoseStamped, f'{ns}/mavros/setpoint_position/global', qos))
            except Exception:
                pass
            self.state[name]['pose_pubs'] = pose_pubs
            self.state[name]['gps_pubs'] = gps_pubs

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
        
    def gps_callback(self, msg, drone):
        if self.state[drone].get('gps') is None:
            self.get_logger().info(f"[{drone}] GPS data received")
        self.state[drone]['gps'] = msg
        
    def battery_callback(self, msg, drone):
        self.state[drone]['battery'] = msg
        # Only warn once about low battery
        if msg.percentage < 0.20 and not self.state[drone].get('low_battery_warned'):
            self.get_logger().warn(f"[{drone}] Low battery: {msg.percentage*100:.1f}%")
            self.state[drone]['low_battery_warned'] = True
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

    def send_position(self, drone, x, y, z, yaw_deg=None):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        # Set orientation based on yaw if provided, otherwise keep current
        if yaw_deg is not None:
            yaw_rad = math.radians(float(yaw_deg))
            qx, qy, qz, qw = self._yaw_to_quaternion(yaw_rad)
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
        else:
            pose = self.state[drone].get('pose')
            if pose is not None and hasattr(pose.pose, 'orientation'):
                msg.pose.orientation = pose.pose.orientation
            else:
                msg.pose.orientation.w = 1.0
            
        pubs = self.state[drone].get('pose_pubs') or []
        if not pubs:
            self.get_logger().warn(f"[{drone}] No pose publishers available")
            return
            
        for pub in pubs:
            try:
                pub.publish(msg)
            except Exception:
                pass

    def _yaw_to_quaternion(self, yaw_rad):
        """Return quaternion (x,y,z,w) for a yaw angle in radians (rotation about Z)."""
        half = yaw_rad * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def set_heading(self, drone, yaw_deg):
        """Set drone heading to yaw_deg using existing position."""
        yaw_rad = math.radians(float(yaw_deg))
        qx, qy, qz, qw = self._yaw_to_quaternion(yaw_rad)

        # Use current pose if available, otherwise use target waypoint
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
        for pub in pubs:
            try:
                pub.publish(msg)
            except Exception:
                pass

    def send_gps_waypoint(self, drone, lat, lon, alt, yaw_deg=None):
        """Send GPS waypoint. Allows multiple drones to fly same GPS pattern."""
        msg = GeoPoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.latitude = lat
        msg.pose.position.longitude = lon
        msg.pose.position.altitude = alt
        
        if yaw_deg is not None:
            yaw_rad = math.radians(float(yaw_deg))
            qx, qy, qz, qw = self._yaw_to_quaternion(yaw_rad)
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
        else:
            msg.pose.orientation.w = 1.0
            
        pubs = self.state[drone].get('gps_pubs') or []
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
        print("  Circle:    circle <drone1> [drone2 ...] <alt> <radius>")
        print("             Single:    circle sim1 5.0 10.0")
        print("             Formation: circle sim1 sim2 sim3 5.0 10.0")
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
                # For other commands (land, heading, circle), second part is the drone name
                cmd = parts[0].lower()
                if cmd in ['land', 'heading', 'circle']:
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
                elif cmd == 'circle' and len(parts) >= 4:
                    # Parse circle command: circle <drone1> [drone2 ...] <altitude> <radius>
                    # Find where altitude/radius start (last 2 numeric args)
                    try:
                        altitude = float(parts[-2])
                        radius = float(parts[-1])
                        # All parts between 'circle' and last 2 are drone names
                        formation_drones = parts[1:-2]
                    except ValueError:
                        self.get_logger().error("Invalid circle parameters - expected: circle <drone1> [drone2 ...] <altitude> <radius>")
                        self.display_command_options()
                        continue
                    
                    # Validate all drones exist
                    invalid_drones = [d for d in formation_drones if d not in self.drones]
                    if invalid_drones:
                        self.get_logger().error(f"Unknown drone(s): {', '.join(invalid_drones)}. Valid drones: {', '.join(self.drones)}")
                        self.display_command_options()
                        continue
                    
                    # Check battery for all drones
                    low_battery_drones = []
                    for d in formation_drones:
                        battery = self.state[d].get('battery')
                        if battery and battery.percentage < 0.20:
                            low_battery_drones.append(f"{d} ({battery.percentage*100:.1f}%)")
                    if low_battery_drones:
                        self.get_logger().error(f"Battery too low for: {', '.join(low_battery_drones)}")
                        self.display_command_options()
                        continue
                    
                    # Check initialization for all drones
                    uninitialized = []
                    for d in formation_drones:
                        if not all([self.state[d].get(key) for key in ['state', 'battery', 'pose']]):
                            uninitialized.append(d)
                    if uninitialized:
                        self.get_logger().error(f"Drones not initialized: {', '.join(uninitialized)}")
                        self.display_command_options()
                        continue
                    
                    # Determine GPS vs local mode from first drone
                    first_drone = formation_drones[0]
                    gps = self.state[first_drone].get('gps')
                    use_gps = gps and hasattr(gps, 'latitude') and gps.latitude != 0
                    
                    # If GPS mode, verify all drones have GPS
                    if use_gps:
                        no_gps_drones = []
                        for d in formation_drones:
                            d_gps = self.state[d].get('gps')
                            if not (d_gps and hasattr(d_gps, 'latitude') and d_gps.latitude != 0):
                                no_gps_drones.append(d)
                        if no_gps_drones:
                            self.get_logger().error(f"GPS not available for: {', '.join(no_gps_drones)}")
                            self.display_command_options()
                            continue
                    
                    num_drones = len(formation_drones)
                    
                    # Generate waypoints for each drone with phase offset
                    for idx, d in enumerate(formation_drones):
                        # Calculate phase offset for even spacing
                        phase_offset = 2 * math.pi * idx / num_drones
                        
                        if use_gps:
                            # GPS mode - generate lat/lon waypoints
                            d_gps = self.state[d].get('gps')
                            lat0, lon0 = d_gps.latitude, d_gps.longitude
                            # Convert radius to degrees (approx 111km per degree)
                            r_lat = radius / 111000.0
                            r_lon = radius / (111000.0 * math.cos(math.radians(lat0)))
                            waypoints = [(lat0 + r_lat * math.sin(i * math.pi / 8 + phase_offset),
                                         lon0 + r_lon * math.cos(i * math.pi / 8 + phase_offset), altitude, True)
                                        for i in range(17)]
                            waypoints.append((lat0, lon0, altitude, True))
                            self.get_logger().info(f"[{d}] Circle in GPS mode at ({lat0:.6f}, {lon0:.6f}), phase offset {math.degrees(phase_offset):.1f}°")
                        else:
                            # Local mode - generate x/y waypoints
                            pose = self.state[d].get('pose')
                            cx, cy = (0.0, 0.0) if not pose else (pose.pose.position.x, pose.pose.position.y)
                            waypoints = [(cx + radius * math.cos(i * math.pi / 8 + phase_offset), 
                                         cy + radius * math.sin(i * math.pi / 8 + phase_offset), altitude, False) 
                                        for i in range(17)]
                            waypoints.append((cx, cy, altitude, False))
                            self.get_logger().info(f"[{d}] Circle in local mode at ({cx:.2f}, {cy:.2f}), phase offset {math.degrees(phase_offset):.1f}°")
                        
                        self.circle_waypoints[d] = waypoints
                        self.target_waypoints[d] = waypoints[0]
                        
                        # Check if flying
                        is_flying = self.state[d].get('pose') and self.state[d]['pose'].pose.position.z > 0.5
                        is_armed = self.state[d].get('state') and self.state[d]['state'].armed
                        
                        if is_flying and is_armed:
                            self.state[d]['current_state'] = DroneState.CIRCLE
                            self.get_logger().info(f"[{d}] Starting circle: alt={altitude}m, radius={radius}m")
                        else:
                            self.state[d]['ready_to_arm'] = True
                            self.state[d]['current_state'] = DroneState.ARM
                            self.get_logger().info(f"[{d}] Circle queued. Taking off to {altitude}m")
                    
                    mode_str = "GPS" if use_gps else "local"
                    self.get_logger().info(f"Formation circle started for {num_drones} drones in {mode_str} mode")
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
                    
                    # Store the waypoint
                    self.target_waypoints[drone] = (x, y, z)
                    self.get_logger().info(f"[{drone}] Waypoint stored: ({x}, {y}, {z})")
                    
                    # Check if drone is already flying
                    pose = self.state[drone].get('pose')
                    is_flying = False
                    if pose is not None:
                        current_z = getattr(pose.pose.position, 'z', 0.0)
                        is_flying = current_z > 0.5  # Consider flying if above 0.5m
                    
                    fcu_state = self.state[drone].get('state')
                    is_armed = fcu_state and getattr(fcu_state, 'armed', False)
                    
                    if is_flying and is_armed:
                        # Already flying - go directly to waypoint
                        self.state[drone]['current_state'] = DroneState.WAYPOINT
                        self.get_logger().info(f"[{drone}] Already flying - moving to waypoint ({x}, {y}, {z})")
                    else:
                        # Not flying - need to arm and takeoff first
                        self.state[drone]['ready_to_arm'] = True
                        self.state[drone]['current_state'] = DroneState.ARM
                        self.get_logger().info(f"[{drone}] Waypoint set to ({x}, {y}, {z}). Preparing for takeoff.")
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
                                s['current_state'] = DroneState.CIRCLE if drone in self.circle_waypoints else DroneState.WAYPOINT
                                self.get_logger().info(f"[{drone}] Takeoff complete, starting movement to waypoint")
                            else:
                                self.get_logger().error(f"[{drone}] Takeoff failed - no altitude increase detected")
                        else:
                            self.get_logger().error(f"[{drone}] Takeoff command failed")
                    else:
                        self.get_logger().error(f"[{drone}] Mode change to GUIDED failed")

                # Move to waypoint
                elif s['current_state'] == DroneState.HOVER:
                    if drone in self.target_waypoints:
                        x, y, z = self.target_waypoints[drone]
                        s['current_state'] = DroneState.WAYPOINT  # Switch to WAYPOINT state for movement
                        self.get_logger().info(f"[{drone}] Starting movement to waypoint ({x}, {y}, {z})")
                        # Ensure we're in GUIDED mode for waypoint navigation
                        if not self.set_mode(drone, "GUIDED"):
                            self.get_logger().error(f"[{drone}] Could not set GUIDED mode for waypoint control")

                # Handle waypoint movement
                elif s['current_state'] == DroneState.WAYPOINT:
                    if drone in self.target_waypoints:
                        x, y, z = self.target_waypoints[drone]
                        
                        # Verify we're in a flight mode (GUIDED or GUIDED_NOGPS)
                        current_mode = None
                        if s.get('state') is not None:
                            current_mode = getattr(s['state'], 'mode', '')
                        if current_mode is None or ('GUIDED' not in str(current_mode).upper()):
                            if not self.set_mode(drone, "GUIDED"):
                                self.get_logger().error(f"[{drone}] Lost flight mode, retrying...")
                                continue

                        # Get current position
                        pose = s.get('pose')
                        if pose is not None:
                            current_x = getattr(pose.pose.position, 'x', 0.0)
                            current_y = getattr(pose.pose.position, 'y', 0.0)
                            current_z = getattr(pose.pose.position, 'z', 0.0)

                            # Calculate distance to target
                            dx = x - current_x
                            dy = y - current_y
                            dz = z - current_z
                            dist = math.sqrt(dx*dx + dy*dy + dz*dz)

                            # Check if reached waypoint
                            if dist < 0.6:
                                self.get_logger().info(f"[{drone}] Reached waypoint ({x:.1f}, {y:.1f}, {z:.1f})")
                                s['current_state'] = DroneState.HOVER
                                # Clear the waypoint so we don't immediately re-enter WAYPOINT state
                                if drone in self.target_waypoints:
                                    del self.target_waypoints[drone]
                                self.get_logger().info(f"[{drone}] Hovering. Ready for next command.")
                                self.display_command_options()
                            else:
                                # Send target position with heading toward waypoint
                                target_heading = math.degrees(math.atan2(dy, dx))
                                self.send_position(drone, x, y, z, yaw_deg=target_heading)
                    # maintain explicit heading if requested
                    elif s.get('target_heading') is not None:
                        try:
                            self.set_heading(drone, s['target_heading'])
                        except Exception:
                            pass

                # Handle circle flight
                elif s['current_state'] == DroneState.CIRCLE:
                    if drone in self.circle_waypoints and self.circle_waypoints[drone]:
                        wp = self.circle_waypoints[drone][0]
                        is_gps = len(wp) == 4 and wp[3]  # Check if GPS waypoint
                        
                        if is_gps:
                            # GPS mode
                            lat, lon, alt = wp[0], wp[1], wp[2]
                            gps = s.get('gps')
                            if gps and hasattr(gps, 'latitude'):
                                # Calculate distance using haversine approximation
                                dlat = math.radians(lat - gps.latitude)
                                dlon = math.radians(lon - gps.longitude)
                                a = math.sin(dlat/2)**2 + math.cos(math.radians(gps.latitude)) * math.cos(math.radians(lat)) * math.sin(dlon/2)**2
                                dist = 6371000 * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
                                
                                if dist < 1.5:  # Reached waypoint
                                    self.circle_waypoints[drone].pop(0)
                                    if not self.circle_waypoints[drone]:
                                        self.get_logger().info(f"[{drone}] Circle complete")
                                        s['current_state'] = DroneState.LAND
                                        del self.circle_waypoints[drone]
                                else:
                                    heading = math.degrees(math.atan2(math.sin(dlon) * math.cos(math.radians(lat)),
                                                          math.cos(math.radians(gps.latitude)) * math.sin(math.radians(lat)) -
                                                          math.sin(math.radians(gps.latitude)) * math.cos(math.radians(lat)) * math.cos(dlon)))
                                    self.send_gps_waypoint(drone, lat, lon, alt, yaw_deg=heading)
                        else:
                            # Local mode
                            x, y, z = wp[0], wp[1], wp[2]
                            pose = s.get('pose')
                            if pose is not None:
                                current_x = getattr(pose.pose.position, 'x', 0.0)
                                current_y = getattr(pose.pose.position, 'y', 0.0)
                                dist = math.sqrt((x - current_x)**2 + (y - current_y)**2)

                                if dist < 1.0:
                                    self.circle_waypoints[drone].pop(0)
                                    if not self.circle_waypoints[drone]:
                                        self.get_logger().info(f"[{drone}] Circle complete")
                                        s['current_state'] = DroneState.LAND
                                        del self.circle_waypoints[drone]
                                else:
                                    target_heading = math.degrees(math.atan2(y - current_y, x - current_x))
                                    self.send_position(drone, x, y, z, yaw_deg=target_heading)

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
