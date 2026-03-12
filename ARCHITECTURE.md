# ARCHITECTURE.md — Node Graph, Topics, and Coordinate Frames

## Node Graph (Current State)

```
┌─────────────────────────────────────────────────────────────────────┐
│                        EXTERNAL PROCESSES                           │
│                                                                     │
│   ArduPilot FCU ◄──MAVLink──► mavros_node (/<drone>)               │
│   usb_cam_node ──────────────────────────────────────────────────  │
└─────────────────────────────────────────────────────────────────────┘
         │                                   │
         │ MAVROS topics                     │ /image_raw
         ▼                                   ▼
┌─────────────────────┐           ┌─────────────────────┐
│  controller         │           │  apriltag_node      │
│  (capstone pkg)     │           │  (apriltag pkg)     │
│                     │           │                     │
│  Subscribes:        │           │  Subscribes:        │
│  /<d>/state         │           │  /image_raw         │
│  /<d>/local_pos/pose│           │                     │
│  /<d>/global_pos/.. │           │  Publishes:         │
│  /<d>/battery       │           │  /apriltag_pose     │
│  /<d>/rc/in         │           │  (PoseStamped)      │
│                     │           └─────────────────────┘
│  Publishes:         │                    │
│  /<d>/setpoint_pos/ │                    │  ← NOT YET CONSUMED
│    local            │                    │    by any node
│  /<d>/setpoint_pos/ │
│    global           │           ┌─────────────────────┐
│                     │           │  image_saver        │
│  Calls Services:    │           │  (apriltag pkg)     │
│  /<d>/cmd/arming    │           │  (utility only)     │
│  /<d>/set_mode      │           └─────────────────────┘
│  /<d>/cmd/takeoff   │
└─────────────────────┘
         │
         │  (separate, incompatible architecture)
         ▼
┌─────────────────────┐
│  move_to_goal       │
│  (move2hover pkg)   │
│  (NOT MAVROS-based) │
│                     │
│  Subscribes:        │
│  /odom              │
│  /imu               │
│  /ctrl_relinq       │
│                     │
│  Publishes:         │
│  /cmd_vel           │
└─────────────────────┘

┌─────────────────────┐   ┌─────────────────────┐
│  telemetry          │   │  status_node        │
│  (comms_interface)  │   │  (comms_interface)  │
│  (logging only)     │   │  (logging only)     │
│                     │   │                     │
│  Subscribes:        │   │  Subscribes:        │
│  /smaug2/imu/data   │   │  /smaug2/state      │
│  /smaug2/global_... │   │  /smaug2/ext_state  │
│  /smaug2/battery    │   │  /smaug2/sys_status │
│  (HARDCODED)        │   │  /diagnostics       │
└─────────────────────┘   └─────────────────────┘
```

---

## Topic Reference

### capstone/controller — Subscriptions

| Topic | Message Type | Package | QoS | Notes |
|---|---|---|---|---|
| `/<drone>/state` | `mavros_msgs/State` | mavros_msgs | BEST_EFFORT | Armed, mode, connected |
| `/<drone>/local_position/pose` | `geometry_msgs/PoseStamped` | geometry_msgs | BEST_EFFORT | ENU local frame |
| `/<drone>/global_position/global` | `sensor_msgs/NavSatFix` | sensor_msgs | BEST_EFFORT | GPS lat/lon/alt |
| `/<drone>/battery` | `sensor_msgs/BatteryState` | sensor_msgs | BEST_EFFORT | Voltage + percentage |
| `/<drone>/rc/in` | `mavros_msgs/RCIn` | mavros_msgs | BEST_EFFORT | RC channel values |

### capstone/controller — Publications

| Topic | Message Type | Package | QoS | Notes |
|---|---|---|---|---|
| `/<drone>/setpoint_position/local` | `geometry_msgs/PoseStamped` | geometry_msgs | default | ENU local setpoint |
| `/<drone>/setpoint_position/global` | `geographic_msgs/GeoPoseStamped` | geographic_msgs | default | GPS setpoint (circle mode) |

### capstone/controller — Service Clients

| Service | Type | Package | Notes |
|---|---|---|---|
| `/<drone>/cmd/arming` | `mavros_msgs/srv/CommandBool` | mavros_msgs | Arm/disarm |
| `/<drone>/set_mode` | `mavros_msgs/srv/SetMode` | mavros_msgs | STABILIZE, GUIDED, etc. |
| `/<drone>/cmd/takeoff` | `mavros_msgs/srv/CommandTOL` | mavros_msgs | Takeoff to altitude |

### apriltag/apriltag_node

| Direction | Topic | Message Type | Notes |
|---|---|---|---|
| Subscribe | `/image_raw` | `sensor_msgs/Image` | YUYV format from usb_cam |
| Publish | `/apriltag_pose` | `geometry_msgs/PoseStamped` | One message per detected tag |

`frame_id` on `/apriltag_pose` is set to `str(tag.tag_id)` — the tag's integer ID, not a TF frame.

### move2hover/move_to_goal

| Direction | Topic | Message Type | Notes |
|---|---|---|---|
| Subscribe | `/odom` | `nav_msgs/Odometry` | Position source |
| Subscribe | `/imu` | `sensor_msgs/Imu` | Yaw source |
| Subscribe | `/ctrl_relinq` | `std_msgs/Bool` | Enable flag |
| Publish | `/cmd_vel` | `geometry_msgs/Twist` | Direct velocity command |

### comms_interface/telemetry (hardcoded to smaug2)

| Direction | Topic | Message Type |
|---|---|---|
| Subscribe | `/smaug2/imu/data` | `sensor_msgs/Imu` |
| Subscribe | `/smaug2/global_position/raw/fix` | `sensor_msgs/NavSatFix` |
| Subscribe | `/smaug2/battery` | `sensor_msgs/BatteryState` |

### comms_interface/status_node (hardcoded to smaug2)

| Direction | Topic | Message Type |
|---|---|---|
| Subscribe | `/smaug2/state` | `mavros_msgs/State` |
| Subscribe | `/smaug2/extended_state` | `mavros_msgs/ExtendedState` |
| Subscribe | `/smaug2/sys_status` | `mavros_msgs/SysStatus` |
| Subscribe | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |

---

## Coordinate Frames

### Local Position Frame (ENU — East/North/Up)

Used by MAVROS `local_position/pose` and `setpoint_position/local`.

```
Z (Up)
│
│   Y (North)
│  /
│ /
└────── X (East)
```

- Origin: arming location (or home position reset)
- Units: meters
- The capstone controller uses this frame for all waypoint and circle setpoints

### Global Position Frame

Used by MAVROS `global_position/global` and `setpoint_position/global`.
- Latitude (degrees), Longitude (degrees), Altitude (meters, AMSL)
- Used in circle mode when GPS coordinates are required

### AprilTag Camera Frame

The `apriltag_node` outputs poses in the **camera optical frame**:
```
Z (into scene / forward)
│
│   Y (down)
│  /
│ /
└────── X (right)
```

- Origin: camera optical center
- Units: meters
- `t[0]` = lateral offset, `t[1]` = vertical offset, `t[2]` = distance to tag
- **No TF frame is published** — `frame_id` is set to the tag's integer ID as a string

### Frame Transforms Needed (Not Yet Implemented)

To use AprilTag poses for landing guidance, the following transforms will be needed:
1. `camera_optical_frame → body_frame` (fixed mount offset/rotation)
2. `body_frame → base_link` (standard ArduPilot body frame)
3. Mapping from tag-relative position to ENU setpoint correction

---

## State Machine (capstone/controller)

```
┌───────────────────────────────────────────────────────────┐
│                      RC_TAKEOVER                          │
│  (any state → RC_TAKEOVER when RC ch8 > 1500)            │
│  (RC_TAKEOVER → ARM when mode returns to GUIDED/RTL ends) │
└───────────────────────────────────────────────────────────┘

                    ┌─────────┐
               ┌───►│   ARM   │◄──────────────────────┐
               │    └────┬────┘                        │
               │         │ STABILIZE → arm → GUIDED    │
               │         │ → takeoff                   │
               │    ┌────▼────┐                        │
               │    │WAYPOINT │◄──────────────────────┐│
               │    └────┬────┘  user: <d> x y z      ││
               │         │ within 0.6m tolerance       ││
               │    ┌────▼────┐                        ││
               │    │  HOVER  │────────────────────────┘│
               │    └────┬────┘  user: land <d>         │
               │         │ user: circle ...              │
               │    ┌────▼────┐                         │
               │    │ CIRCLE  │                         │
               │    └────┬────┘                         │
               │         │ waypoints exhausted           │
               │    ┌────▼────┐                         │
               └────┤  LAND   │                         │
                    └────┬────┘                         │
                         │ altitude < 0.1m              │
                         └─────────────────────────────►┘
```

---

## Threading Model (capstone/controller)

```
Main thread:          rclpy.spin()  ← processes all ROS2 callbacks
Thread 1 (daemon):    state_machine_loop() at 50 Hz
Thread 2 (daemon):    user_input_loop()   ← blocks on input()
```

All shared state accessed via `self.state[drone_name]` dict. No explicit locks — relies on Python GIL.

---

## Service Namespace Discovery

The controller tries multiple MAVROS namespace patterns:
```
1. /<ns>/cmd/arming          ← standard ROS2 MAVROS
2. /<ns>/mavros/cmd/arming   ← if mavros is sub-namespace
3. /mavros<ns>/cmd/arming    ← legacy patterns
```

---

## Battery State Estimation

For real drones (not SITL), battery percentage is estimated from voltage:
- Full: `cell_count × 4.2V`
- Empty: `cell_count × 3.5V`
- Default cell count: 6S
- Constructor parameter: `cell_count=6`

---

## Missing Connections (Gaps in the Node Graph)

```
apriltag_node  ──/apriltag_pose──►  [NOTHING]
                                     ↑
                                     Landing guidance node
                                     needed here
```

The AprilTag node publishes `/apriltag_pose` but no node subscribes to it. The capstone controller's `LAND` state performs blind altitude descent only.
