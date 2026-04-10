# ARCHITECTURE.md — Node Graph, Topics, and Coordinate Frames

## System Split

```
GROUND LAPTOP (192.168.168.103)          PI 4 (192.168.168.102)
────────────────────────────────         ──────────────────────────────────────
ros2 run capstone controller             ros2 launch onboard onboard.launch.py
  (python3 controller.py)                  mavros_node        ← ttyACM0 @ 921600
                                           camera_node        ← CSI IMX296
  Microhard radio link                     image_rotate       ← /camera/image_raw
  192.168.168.x subnet                     apriltag_node      ← /camera/image_rotated
                                           precision_landing  ← /apriltag/detections
  /<d>/precision_landing/takeover          camera_tf_static
  (Pi→Ground, Bool) ──────────────────────────────────────────────────────────
────────────────────────────────         ──────────────────────────────────────
```

---

## Node Graph

```
ArduPilot FCU
    │ serial ttyACM0 @ 921600 baud
    ▼
┌───────────────────────┐
│  mavros_node          │  namespace: /<drone_name>  (default: /drone1)
│  (mavros pkg)         │
│                       │  Publishes (BEST_EFFORT QoS):
│  fcu_url from         │  /<d>/state                   mavros_msgs/State
│  onboard.yaml         │  /<d>/local_position/pose     geometry_msgs/PoseStamped
│                       │  /<d>/global_position/global  sensor_msgs/NavSatFix
│                       │  /<d>/battery                 sensor_msgs/BatteryState
│                       │  /<d>/rc/in                   mavros_msgs/RCIn
│                       │
│                       │  Services:
│                       │  /<d>/cmd/arming              mavros_msgs/srv/CommandBool
│                       │  /<d>/set_mode                mavros_msgs/srv/SetMode
│                       │  /<d>/cmd/takeoff             mavros_msgs/srv/CommandTOL
└───────────────────────┘
    │ (Microhard link)
    ▼
┌───────────────────────┐
│  controller           │  node name: controller
│  (capstone pkg)       │  runs on: ground laptop
│  ground laptop        │
│                       │  Subscribes (BEST_EFFORT):
│                       │  /<d>/state, local_position/pose,
│                       │  global_position/global, battery, rc/in
│                       │  /<d>/precision_landing/takeover  Bool (Pi→Ground)
│                       │
│                       │  Publishes:
│                       │  /<d>/setpoint_position/local   PoseStamped
│                       │  /<d>/setpoint_position/global  GeoPoseStamped
│                       │
│                       │  Interface: stdin terminal commands
└───────────────────────┘


── CAMERA / VISION SUBSYSTEM ─────────────────────────────────────────────────

CSI ribbon camera (IMX296 global shutter, 1280×720, calibrated 2026-04-08)
    │
    ▼
┌───────────────────────┐
│  camera_node          │  node name: camera → topics at /camera/*
│  (camera_ros pkg)     │  frame_id published: "camera" (node name, not param)
│                       │
│                       │  Publishes:
│                       │  /camera/image_raw    sensor_msgs/Image  (upside-down)
│                       │  /camera/camera_info  sensor_msgs/CameraInfo
└───────────────────────┘
    │
    ▼
┌───────────────────────┐
│  image_rotate         │  rotation_angle: π — corrects upside-down mount
│  (image_rotate pkg)   │
│                       │  /camera/image_raw → /camera/image_rotated
└───────────────────────┘
    │
    ▼
┌───────────────────────┐
│  apriltag_node        │
│  (apriltag_ros pkg)   │  Subscribes:
│                       │  /camera/image_rotated  (correct orientation)
│                       │  /camera/camera_info
│                       │
│                       │  Publishes:
│                       │  /apriltag/detections  apriltag_msgs/AprilTagDetectionArray
│                       │
│                       │  TF2 (dynamic, per detection):
│                       │  camera → tag<id>
└───────────────────────┘
    │
    ▼
┌───────────────────────┐
│  precision_landing    │  Subscribes:
│  (onboard pkg)        │  /apriltag/detections  — always watching
│                       │  /<d>/local_position/pose
│                       │  /<d>/precision_landing/abort  (Bool, from ground)
│                       │
│                       │  Publishes:
│                       │  /<d>/setpoint_position/local   (when CENTERING)
│                       │  /<d>/precision_landing/takeover  Bool (Pi→Ground)
│                       │
│                       │  Service client:
│                       │  /<d>/set_mode  (LAND on handoff)
│                       │
│                       │  State machine: IDLE → CENTERING → HANDOFF → IDLE
└───────────────────────┘

┌───────────────────────┐
│  camera_tf_static     │  static_transform_publisher
│  (tf2_ros)            │  base_link → camera
│                       │  x=0, y=0, z=-0.05, roll=π, pitch=0, yaw=0
│                       │  (confirmed measurements 2026-04-09)
└───────────────────────┘
```

---

## Topic Reference

### capstone/controller — Subscriptions (all BEST_EFFORT QoS)

| Topic | Message Type | Notes |
|---|---|---|
| `/<d>/state` | `mavros_msgs/State` | Armed, mode, connected |
| `/<d>/local_position/pose` | `geometry_msgs/PoseStamped` | ENU frame, metres |
| `/<d>/global_position/global` | `sensor_msgs/NavSatFix` | GPS lat/lon/alt |
| `/<d>/battery` | `sensor_msgs/BatteryState` | Voltage + percentage |
| `/<d>/rc/in` | `mavros_msgs/RCIn` | Raw RC channels |

### capstone/controller — Publications

| Topic | Message Type | Notes |
|---|---|---|
| `/<d>/setpoint_position/local` | `geometry_msgs/PoseStamped` | Local ENU setpoint |
| `/<d>/setpoint_position/global` | `geographic_msgs/GeoPoseStamped` | GPS setpoint (circle mode) |

### capstone/controller — Service Clients

| Service | Type | Notes |
|---|---|---|
| `/<d>/cmd/arming` | `mavros_msgs/srv/CommandBool` | Tries 4 namespace patterns |
| `/<d>/set_mode` | `mavros_msgs/srv/SetMode` | STABILIZE → GUIDED → LAND/RTL |
| `/<d>/cmd/takeoff` | `mavros_msgs/srv/CommandTOL` | Called after arming |

### onboard — camera_ros (PENDING)

| Direction | Topic | Message Type | Notes |
|---|---|---|---|
| Publish | `/camera/image_raw` | `sensor_msgs/Image` | YUYV from CSI sensor |
| Publish | `/camera/camera_info` | `sensor_msgs/CameraInfo` | Calibration params |

### onboard — apriltag_ros (PENDING)

| Direction | Topic | Message Type | Notes |
|---|---|---|---|
| Subscribe | `/camera/image_raw` | `sensor_msgs/Image` | Remapped from `image_rect` |
| Subscribe | `/camera/camera_info` | `sensor_msgs/CameraInfo` | Remapped from `camera_info` |
| Publish | `/apriltag/detections` | `apriltag_msgs/AprilTagDetectionArray` | Remapped from `detections` |
| TF2 | `camera_optical_frame → tag_<id>` | — | Per detected tag, dynamic |

---

## Config: onboard/config/onboard.yaml

Single source of truth for all onboard parameters. Nothing hardcoded in launch file.

| Section | Key fields |
|---|---|
| `mavros` | `drone_name`, `fcu_url` |
| `camera` | `camera` (index), `width`, `height`, `framerate`, `frame_id` |
| `apriltag` | `tag_family`, `tag_size`, `max_hamming` |
| `camera_tf` | `parent_frame`, `child_frame`, `x`, `y`, `z`, `roll`, `pitch`, `yaw` |

---

## Coordinate Frames

### TF2 Tree (when camera is working)

```
base_link  (body: X-forward, Y-left, Z-up)
    │  static — onboard.yaml camera_tf  (x=0, y=0, z=-0.05, roll=π)
    ▼
camera  (frame_id published by camera_ros; X-right, Y-down, Z-into-scene)
    │  dynamic — apriltag_ros, one per detection
    ▼
tag<id>  (origin at tag centre, Z pointing out of tag face)
```

Full chain for precision landing TF lookup: `map → base_link → camera → tag<id>`
(MAVROS publishes dynamic `map → base_link`)

### Local ENU (MAVROS local_position)
- X = East, Y = North, Z = Up
- Origin = arming location
- Used by controller for all waypoint and circle setpoints

### Global (MAVROS global_position)
- WGS84 lat/lon/alt (AMSL)
- Used by controller in GPS-mode circle flight

---

## State Machine (capstone/controller)

```
ARM → WAYPOINT → HOVER → WAYPOINT (loop)
                       → CIRCLE → LAND → ARM (reset)
                       → LAND
                       → PRECISION_LANDING (on takeover=True from Pi)
PRECISION_LANDING → HOVER (on takeover=False from Pi — landing complete or aborted)
Any state → RC_TAKEOVER (on RTL mode detect) → ARM
```

Takeoff sequence: `STABILIZE → arm → GUIDED → CommandTOL`

---

## Threading Model (capstone/controller)

```
Main thread:       rclpy.spin()           — ROS2 callbacks
Thread 1 (daemon): state_machine_loop()   — 50 Hz state transitions
Thread 2 (daemon): user_input_loop()      — blocks on stdin input()
```

---

## Service Namespace Discovery

Controller tries in order per drone:
```
/<ns>/cmd/arming
/<ns>/mavros/cmd/arming
/mavros<ns>/cmd/arming
/mavros/cmd/arming
```

---

## Battery Monitoring

Battery monitoring is delegated to Mission Planner / ArduCopter's onboard failsafe.
The controller displays raw voltage at the command prompt for operator awareness only.
No abort logic exists in the controller — Mission Planner handles failsafe actions.
