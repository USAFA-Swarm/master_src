# Drone Controller — Documentation

This document explains what the controller does, how it works internally, and how to operate it. 

---

## Table of Contents

1. [What It Does](#1-what-it-does)
2. [System Requirements](#2-system-requirements)
3. [How to Run](#3-how-to-run)
4. [Command Reference](#4-command-reference)
5. [Architecture Overview](#5-architecture-overview)
6. [State Machine](#6-state-machine)
7. [Internal Data Structures](#7-internal-data-structures)
8. [ROS2 Topics and Services](#8-ros2-topics-and-services)
9. [Key Design Decisions and Gotchas](#9-key-design-decisions-and-gotchas)

---

## 1. What It Does

The controller is a ROS2 Python node that provides autonomous flight control for one or more ArduPilot-based drones via MAVROS. It exposes a text-based command interface in the terminal and handles all low-level arming, mode switching, takeoff, waypoint navigation, circle flight patterns, and landing automatically.

**Supported operations:**
- Fly to a single (x, y, z) waypoint in local frame
- Fly a clockwise circle pattern around the current position at a given altitude and radius
- Form a circle pattern with multiple drones simultaneously (evenly spaced around the circle)
- Land on command
- Set a fixed heading while hovering
- Low battery warning and abort
- RTL (Return to Launch) detection — exits autonomous control if the FCU switches to RTL mode

---

## 2. System Requirements

| Dependency | Notes |
|---|---|
| ROS2 (Humble or later) | Tested on Ubuntu 22.04 |
| MAVROS | Must be running for each drone namespace (e.g. `/sim1`) |
| MAVProxy | Required in simulation to bridge ArduPilot SITL ↔ MAVROS |
| ArduPilot SITL or real vehicle | Tested with ArduCopter |
| Python packages | `rclpy`, `geometry_msgs`, `geographic_msgs`, `sensor_msgs`, `mavros_msgs` |

**For simulation**, three processes must be running before the controller is started:

```
1. ArduPilot SITL         (provides the simulated vehicle)
2. MAVProxy               (bridges SITL to MAVROS)
3. MAVROS                 (bridges MAVProxy to ROS2 topics/services)
```

Without MAVProxy, MAVROS will connect but will not receive GPS or position data.

---

## 3. How to Run

### Simulation

**Must be on ECE wifi on both computers**

# Step 1 — ArduPilot SITL - (SITL computer)
Go to 10.1.119.44:8081 in a web browser once the SITL is running. Kill all SITL instances, click on the map to drop a point for the simulated drone, start SITL instances. This only works with one drone at a time. Multiple will show up on QGroundControl 'QGC', but the ros2 topics only identify one drone.

# Terminal 1 — MAVProxy (bridge to MAVROS) - (Ubuntu computer)
mavproxy.py --master=udpout:10.1.119.44:11101 --out=udp:10.1.119.44:15101

# Terminal 2 — MAVROS (with namespace sim1) - (Ubuntu computer)
ros2 run mavros mavros_node --ros-args  -p fcu_url:=udp://@10.1.119.44:11201 -r __ns:=/sim1

# Terminal 3 — Controller - (Ubuntu computer)
cd /home/dfec/master_src/capstone/capstone
python3 controller.py

# QGroundControl/Mission Planner - (Windows computer)
QGC - Application Settings -> Comm links -> Links -> Add -> Enter whatever name you want -> Type = UDP -> Port = 11101 -> Server Addresses = 10.1.119.44:11101

Mission Planner - Set type to UDP -> Connect -> Enter Local port = 11101 -> OK


### Real Drone Connection/Operation

# Microhard and Communication Setup
Must use a switch to split data to both Ubuntu and Windows ground computers. The master microhard should be connected to the switch via the LAN port to one of the numbered ports on the switch (NOT the Internet port on the switch). The two computers should be connected to the other numbered ports on the switch. If using an ethernet -> USB-C connecter, ensure USBPcap is installed (easy to do via WireShark installation) or else ethernet data will not be relayed to the computer. For the drone microhard, ensure the ethernet is connected on the port opposite to the power connector on the microhard. No other special considerations needed for the Pi setup on the drone.

# Connecting the computers to the drone

# 1 SSH into the Pi 
- Open a terminal on the Ubuntu computer, run the ssh code to connect to the Pi on the drone. For our Pi it was: 'ssh pi@192.168.168.102'. Then enter the Pi's password. This process can be sped up by making an alias and using a passkey. The correct ssh programs must be installed on the Pi first. Both mavproxy and mavros commands must be run on the Pi

# 2. Run Mavproxy 
'mavproxy.py --master=/dev/ttyACM0 --baudrate 921600 --out=udp:192.168.168.233:14550 --out=udp:192.168.168.103:14551' 

- The two IPs should correspond to the two ground computers. Connecting to QGC and Mission Planner should be the same as the simulation. You must ensure that the port matches with the IP of the computer. For this case, our Windows computer ends in .233, so we must use the 14550 port. Adding a server address is unneccesary for QGC.

# 3. Run Mavros
'ros2 run mavros mavros_node --ros-args -r __ns:=/drone1 -p fcu_url:=serial:///dev/ttyACM0:921600'

- You may rename it to something other than drone1 as long as it doesn't start with a number. Occasionally this won't connect because the route will change (ttyACM0 will be incorrect). If this happens, reboot the drone until it connects (probably a better solution somewhere here).



When the controller starts, it prompts:

```
Enter drone names (space separated):
```

Type the namespace name(s) that match your MAVROS launch, e.g. `sim1` or `sim1 sim2`.

> **Important:** Drone names must **not** start with a number. `sim1` is valid; `1sim` is not. This is because the name is used directly as a ROS2 namespace prefix.

### Verifying data is flowing before issuing commands

The controller will print warnings every 5 seconds if it hasn't received FCU state, battery, or position data:

```
[sim1] Still waiting for: battery status, position data
```

Wait until these messages stop before issuing flight commands. Once all data arrives you will see:

```
[sim1] FCU state connection established
[sim1] Position data received
[sim1] GPS data received
```

### Checking topics manually

```bash
# Check battery data is publishing (note: requires --qos-reliability flag)
ros2 topic echo --qos-reliability best_effort /sim1/battery

# Check position data
ros2 topic echo --qos-reliability best_effort /sim1/local_position/pose

# Check FCU state (connected, mode, armed)
ros2 topic echo /sim1/state
```

> The `--qos-reliability best_effort` flag is required because MAVROS publishes with `BEST_EFFORT` QoS. The default `ros2 topic echo` uses `RELIABLE` and will silently receive nothing without this flag.

---

## 4. Command Reference

Commands are typed at the `>>> Command>` prompt after the controller starts.

### Waypoint

```
<drone> <x> <y> <z>
```

Fly to a position in the **local ENU frame** (metres, relative to the vehicle's starting position).

- If the drone is on the ground, it will arm, take off to `z` metres, then navigate to `(x, y, z)`.
- If the drone is already flying, it navigates directly.
- Once the waypoint is reached (within 0.6 m), the drone hovers and waits for the next command.

**Example:**
```
sim1 0.0 0.0 5.0     # Arm, take off to 5m, hover
sim1 10.0 0.0 5.0    # Fly 10m north at 5m altitude
```

### Circle

```
circle <drone1> [drone2 ...] <altitude> <radius>
```

Take off to `altitude` metres and fly a **clockwise** circle with the given `radius` (metres) centred on the drone's current position. The drone faces tangentially along the circle as it flies.

- If GPS is available, the pattern is generated in **GPS coordinates** (latitude/longitude), which allows multiple drones to fly the exact same geographic circle.
- If GPS is not available (or GPS coordinates are zero), the pattern uses **local frame** coordinates.
- 17 evenly-spaced waypoints are generated around the circle, plus a return to the start point (18 total).
- After completing the circle, the drone **automatically lands**.

For **multiple drones**, each drone is placed at an evenly-spaced phase offset so they are spread around the circle from the start, flying in formation.

**Examples:**
```
circle sim1 10.0 20.0              # Single drone, 10m altitude, 20m radius
circle sim1 sim2 10.0 20.0         # Two drones, 180° apart
circle sim1 sim2 sim3 10.0 20.0    # Three drones, 120° apart
```

### Land

```
land <drone>
```

Immediately begin landing. Tries `LAND` mode first, then `RTL`, then `AUTO.LAND`. After landing is confirmed (altitude < 0.1 m), the controller resets and waits for the next command.

**Example:**
```
land sim1
```

### Heading

```
heading <drone> <degrees>
```

Rotate the drone to face the given compass bearing (0° = East in local frame, 90° = North). The drone holds its current position while rotating.

**Example:**
```
heading sim1 90
```

---

## 5. Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                     controller.py                        │
│                                                         │
│  Main thread                                            │
│  └─ rclpy.spin()  ──── processes all ROS2 callbacks     │
│                                                         │
│  Thread 1: state_machine_loop()                         │
│  └─ runs at 50 Hz                                       │
│  └─ reads self.state[drone] written by callbacks        │
│  └─ drives each drone through ARM→flight→LAND           │
│                                                         │
│  Thread 2: user_input_loop()                            │
│  └─ blocks on input()                                   │
│  └─ writes commands into self.state[drone]              │
│  └─ sets ready_to_arm, target_waypoints, etc.           │
└─────────────────────────────────────────────────────────┘
         │ ROS2 subscriptions (callbacks)
         ▼
   MAVROS topics: /sim1/state, /sim1/local_position/pose,
                  /sim1/battery, /sim1/global_position/global,
                  /sim1/rc/in
         │ ROS2 service calls (arm, set_mode, takeoff)
         ▼
   MAVROS services: /sim1/cmd/arming, /sim1/set_mode,
                    /sim1/cmd/takeoff
         │ ROS2 publications (position setpoints)
         ▼
   MAVROS topics: /sim1/setpoint_position/local  (local frame)
                  /sim1/setpoint_position/global (GPS frame)
```

### Threading model

The state machine and user input run in **daemon threads** so they die automatically when the main process exits. The main thread runs `rclpy.spin()`, which is the only thread allowed to drive the ROS2 executor.

Service calls (arm, set_mode, takeoff) are called from the state machine thread. Because `rclpy.spin_until_future_complete` cannot be called from a non-main thread, a custom `_wait_for_future()` helper polls the future and calls `rclpy.spin_once()` in a loop until the service responds or times out.

---

## 6. State Machine

Each drone has an independent state machine. The current state is stored in `self.state[drone]['current_state']` and is one of:

```
ARM  →  CIRCLE
          or
ARM  →  WAYPOINT  →  HOVER  (waits for next command)
                          ↓
                        WAYPOINT (if new waypoint issued)
                          ↓
                         LAND

CIRCLE → LAND (auto after completing the circle)

Any state → LAND (on `land` command)
```

### State descriptions

| State | What happens |
|---|---|
| `ARM` | Waits for `ready_to_arm` flag. Sets STABILIZE mode, arms the vehicle, switches to GUIDED, calls `CommandTOL` takeoff to target altitude. Transitions to CIRCLE or WAYPOINT after altitude is reached. |
| `WAYPOINT` | Repeatedly sends a position setpoint toward `target_waypoints[drone]`. Calculates distance every loop tick. Transitions to HOVER when within 0.6 m of target. |
| `HOVER` | Drone has arrived at a waypoint. If a new waypoint is stored, transitions immediately back to WAYPOINT. Otherwise waits. |
| `CIRCLE` | Pops waypoints off `circle_waypoints[drone]` queue one at a time, sending position setpoints. Transitions to LAND when the queue is empty. |
| `LAND` | Sets `LAND` mode (or `RTL` / `AUTO.LAND` as fallback). Monitors altitude. Resets all state when altitude < 0.1 m. |
| `RC_TAKEOVER` | Set if `manual_override` is True. State machine skips this drone. Cleared when any new command (land/waypoint/circle) is issued. |

### Takeoff sequence (ARM state detail)

1. Set mode → `STABILIZE` (required to arm in ArduPilot)
2. Verify STABILIZE is confirmed in FCU state (up to 3 s)
3. Call arming service
4. Verify armed state confirmed in FCU state (up to 3 s)
5. Set mode → `GUIDED`
6. Call `CommandTOL` takeoff service with target altitude
7. Monitor altitude for up to 15 s:
   - Once climb of 0.3 m detected: send position hold setpoint at `(current_x, current_y, target_z)` to maintain position while climbing
   - Once altitude reaches `target_z - 0.5 m`: transition to CIRCLE or WAYPOINT
   - If 15 s elapses without reaching altitude: transition to LAND

---

## 7. Internal Data Structures

### `self.state[drone]` — per-drone state dict

| Key | Type | Description |
|---|---|---|
| `pose` | `PoseStamped` or None | Latest local position from `/local_position/pose` |
| `battery` | `BatteryState` or None | Latest battery message |
| `state` | `mavros_msgs/State` or None | Latest FCU state (connected, armed, mode) |
| `rc` | `RCIn` or None | Latest raw RC input message (stored but not used for control decisions) |
| `gps` | `NavSatFix` or None | Latest GPS fix from `/global_position/global` |
| `current_state` | `DroneState` string | Current state machine state |
| `manual_override` | bool | When True, state machine skips this drone |
| `target_heading` | float or None | Heading to hold in degrees (set by `heading` command) |
| `ready_to_arm` | bool | Flag set by user commands to trigger ARM state processing |
| `landing_attempts` | int | Number of landing mode attempts made |
| `landing_initiated` | bool | True once a landing mode was successfully set |
| `low_battery_warned` | bool | Prevents repeated low battery warnings |
| `arming_client` | service client | ROS2 client for arming service |
| `mode_client` | service client | ROS2 client for mode-change service |
| `takeoff_client` | service client | ROS2 client for takeoff service |
| `pose_pubs` | list | Publishers for local position setpoints |
| `gps_pubs` | list | Publishers for GPS position setpoints |

### `self.target_waypoints[drone]`

Stores the current target as a **3-tuple** `(x, y, z)` for waypoint commands, or a **4-tuple** `(x, y, z, is_gps)` for the current circle waypoint. Deleted when the waypoint is reached or when a new command is issued.

### `self.circle_waypoints[drone]`

A **list of 4-tuples** `(x, y, z, is_gps)` representing the full circle path. The state machine pops from the front of this list as each waypoint is reached. Deleted on landing or when a new waypoint/circle command is issued.

---

## 8. ROS2 Topics and Services

All topics and services are namespaced under `/<drone_name>`. For a drone named `sim1`, the prefix is `/sim1`.

### Subscriptions (controller reads)

| Topic | Message Type | Purpose |
|---|---|---|
| `/<ns>/state` | `mavros_msgs/State` | FCU connection status, armed state, flight mode |
| `/<ns>/local_position/pose` | `geometry_msgs/PoseStamped` | Current local position (ENU frame, metres) |
| `/<ns>/global_position/global` | `sensor_msgs/NavSatFix` | Current GPS position (lat/lon/alt) |
| `/<ns>/battery` | `sensor_msgs/BatteryState` | Battery voltage and percentage |
| `/<ns>/rc/in` | `mavros_msgs/RCIn` | Raw RC input (stored for future use) |

All subscriptions use `BEST_EFFORT` / `VOLATILE` / `KEEP_LAST` QoS to match the MAVROS publisher QoS profile. Using `RELIABLE` QoS here would cause silent subscription failure.

### Publications (controller writes)

| Topic | Message Type | Purpose |
|---|---|---|
| `/<ns>/setpoint_position/local` | `geometry_msgs/PoseStamped` | Local frame position setpoints during waypoint/circle flight |
| `/<ns>/setpoint_position/global` | `geographic_msgs/GeoPoseStamped` | GPS frame position setpoints during GPS-mode circle flight |

Two publisher topic names are attempted for each drone at startup to handle different MAVROS configurations.

### Services (controller calls)

| Service | Type | Purpose |
|---|---|---|
| `/<ns>/cmd/arming` | `mavros_msgs/srv/CommandBool` | Arm/disarm the vehicle |
| `/<ns>/set_mode` | `mavros_msgs/srv/SetMode` | Change flight mode (STABILIZE, GUIDED, LAND, RTL) |
| `/<ns>/cmd/takeoff` | `mavros_msgs/srv/CommandTOL` | Request automated takeoff to altitude |

Multiple candidate service name patterns are tried at startup (e.g. `/<ns>/cmd/arming`, `/<ns>/mavros/cmd/arming`, `/mavros<ns>/cmd/arming`) to handle variations in MAVROS namespace configurations.

---

## 9. Key Design Decisions and Gotchas

### MAVROS QoS must be BEST_EFFORT

MAVROS publishes sensor data with `BEST_EFFORT` reliability. If you subscribe with `RELIABLE` (the ROS2 default), the subscription silently fails — no error, no data. The controller explicitly sets `BEST_EFFORT` / `VOLATILE` / `KEEP_LAST` on all subscriptions.

This also affects manual topic inspection: `ros2 topic echo /sim1/battery` gives no output without `--qos-reliability best_effort`.

### MAVProxy is required in simulation

ArduPilot SITL does not send all telemetry (especially GPS and battery data) unless MAVProxy is running as an intermediary. MAVROS will report `connected: true` even without MAVProxy, but battery and position topics will have publishers with zero messages.

### Waypoint tuples can be 3-tuple or 4-tuple

`target_waypoints[drone]` can hold either a `(x, y, z)` 3-tuple (from the waypoint command) or a `(x, y, z, is_gps)` 4-tuple (the first waypoint of a circle). Any code that unpacks this must handle both forms. Use `wp[0], wp[1], wp[2]` rather than `x, y, z = wp`.

### Takeoff sequence requires STABILIZE before arming

ArduPilot will refuse to arm in GUIDED mode. The sequence must be: STABILIZE → arm → GUIDED → takeoff. The controller verifies each step by watching the FCU state topic before proceeding.

### Service calls from background threads

`rclpy.spin_until_future_complete()` cannot safely be called from a thread other than the one running `rclpy.spin()`. The `_wait_for_future()` helper works around this by polling `future.done()` and calling `rclpy.spin_once()` in a loop, which processes callbacks temporarily from the background thread.

### RC takeover detection — mode-based only

RC takeover is detected **only** by watching the FCU mode field in the `/state` topic. If the mode changes to `RTL`, the state machine exits. There is no RC channel-based detection. This is intentional: in simulation, MAVROS reports unpopulated RC channels as `0`, which is `1500 - 1500 = 1500` away from neutral — a channel-based check with any reasonable deadzone would always trigger a false takeover.

On a real vehicle with a physical RC transmitter, this means you can take over by switching to RTL on the transmitter. Switching to STABILIZE/LOITER/etc. will not trigger the detection but will interrupt autonomous flight because the vehicle will leave GUIDED mode.

### Circle pattern generation

The circle is approximated by 17 evenly-spaced waypoints at `i * π/8` radians apart (every 22.5°), plus a return to the start, giving 18 total. Waypoints are generated **clockwise** by negating the angle in the parametric circle equation. The drone's yaw setpoint along the circle is set to face tangentially (perpendicular to the radial direction) so the nose always points in the direction of travel.
