# ARCHITECTURE.md — Node Graph, Topics, and Coordinate Frames

## Node Graph (Current State)

```
┌──────────────────────────────────────────────────────────────────────┐
│                         EXTERNAL PROCESSES                           │
│                                                                      │
│   ArduPilot FCU ◄──MAVLink──► mavros_node (/<drone>)                │
│   usb_cam_node_exe ───────────────────────────────────────────────── │
└──────────────────────────────────────────────────────────────────────┘
          │                                    │
          │ MAVROS topics                      │ /image_raw  (relative,
          ▼                                    ▼   root namespace)
┌──────────────────────┐           ┌──────────────────────┐
│  controller          │           │  apriltag_node       │
│  (capstone pkg)      │           │  (apriltag pkg)      │
│                      │           │                      │
│  Subscribes:         │           │  Parameter:          │
│  /<d>/state          │           │  camera_info_file    │
│  /<d>/local_position/│           │                      │
│    pose              │           │  Subscribes:         │
│  /<d>/global_position│           │  /image_raw          │
│    /global           │           │                      │
│  /<d>/battery        │           │  Publishes:          │
│  /<d>/rc/in          │           │  /apriltag_pose      │
│                      │           │  (PoseStamped)       │
│  Publishes:          │           └──────────────────────┘
│  /<d>/setpoint_      │                    │
│    position/local    │                    │  ← NOT YET CONSUMED
│  /<d>/setpoint_      │                       by any node
│    position/global   │
│                      │           ┌──────────────────────┐
│  Calls Services:     │           │  image_capture.py    │
│  /<d>/cmd/arming     │           │  (apriltag pkg)      │
│  /<d>/set_mode       │           │  standalone utility, │
│  /<d>/cmd/takeoff    │           │  no ros2 run entry   │
└──────────────────────┘           └──────────────────────┘
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

| Direction | Topic | Message Type | QoS | Notes |
|---|---|---|---|---|
| Subscribe | `/image_raw` | `sensor_msgs/Image` | default (depth 10) | YUYV format from usb_cam; relative topic name — root namespace |
| Publish | `/apriltag_pose` | `geometry_msgs/PoseStamped` | default (depth 10) | One message per detected tag per frame; relative topic name — root namespace |

**Parameters:**

| Parameter | Type | Default | Notes |
|---|---|---|---|
| `camera_info_file` | string | `''` | Path to camera calibration YAML; falls back to `apriltag/config/default_cam.yaml` if empty or missing |

**`frame_id` on `/apriltag_pose`** is set to `str(tag.tag_id)` — the integer tag ID cast to string. This is not a TF frame name.

---

## Launch File

`apriltag/launch/vision.launch.py` starts both required nodes:

```
usb_cam_node_exe  →  /image_raw  →  apriltag_node
  video_device: /dev/video0
  framerate: 15.0
  camera_info_url: file://<pkg>/config/default_cam.yaml
                                  ↓
                          camera_info_file param
```

No launch file exists for the `capstone` package. The controller is launched manually:
```bash
ros2 run capstone controller
```

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
- **No TF frame is published** — `frame_id` is the tag's integer ID as a string

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

The controller tries multiple MAVROS namespace patterns in order:
```
1. /<ns>/cmd/arming          ← standard ROS2 MAVROS
2. /<ns>/mavros/cmd/arming   ← if mavros is a sub-namespace
3. /mavros<ns>/cmd/arming    ← legacy pattern
```

---

## Battery State Estimation

For real drones (not SITL), battery percentage is estimated from voltage:
- Full: `cell_count × 4.2V`
- Empty: `cell_count × 3.5V`
- Default cell count: 6S (set in `main()` as `cell_count=6`)
- Abort threshold: 20% — controller logs a warning and transitions to LAND

---

## Missing Connections (Gap in Node Graph)

```
apriltag_node  ──/apriltag_pose──►  [NOTHING]
                                     ↑
                                     Landing guidance node
                                     needed here
```

The AprilTag node publishes `/apriltag_pose` but no node subscribes to it.
The capstone controller's `LAND` state descends blindly on altitude alone.
