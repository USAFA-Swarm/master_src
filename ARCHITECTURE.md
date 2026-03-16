# ARCHITECTURE.md — Node Graph, Topics, and Coordinate Frames

## System Split

```
GROUND LAPTOP (192.168.168.103)          PI 4 (192.168.168.102)
────────────────────────────────         ──────────────────────────────────────
ros2 run capstone controller             ros2 launch onboard onboard.launch.py
  (python3 controller.py)                  mavros_node     ← ttyACM0 @ 921600
                                           camera_node     ← CSI [PENDING]
  Microhard radio link                     apriltag_node   ← /camera/* [PENDING]
  192.168.168.x subnet                     camera_tf_static
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
│                       │
│                       │  Publishes:
│                       │  /<d>/setpoint_position/local   PoseStamped
│                       │  /<d>/setpoint_position/global  GeoPoseStamped
│                       │
│                       │  Interface: stdin terminal commands
└───────────────────────┘


── CAMERA SUBSYSTEM — PENDING (CSI hardware issue) ──────────────────────────

CSI ribbon camera
    │ [NOT CONNECTED — cable/connector fault]
    │  sudo dmesg shows no unicam/imx entries
    ▼
┌───────────────────────┐
│  camera_node          │  [PENDING]
│  (camera_ros pkg)     │  node name: camera → topics at /camera/*
│                       │
│                       │  Publishes:
│                       │  /camera/image_raw    sensor_msgs/Image
│                       │  /camera/camera_info  sensor_msgs/CameraInfo
└───────────────────────┘
    │
    ▼
┌───────────────────────┐
│  apriltag_node        │  [PENDING — blocked on camera]
│  (apriltag_ros pkg)   │
│                       │  Subscribes:
│                       │  /camera/image_raw    (remapped from image_rect)
│                       │  /camera/camera_info  (remapped from camera_info)
│                       │
│                       │  Publishes:
│                       │  /apriltag/detections  apriltag_msgs/AprilTagDetectionArray
│                       │
│                       │  TF2 (dynamic, per detection):
│                       │  camera_optical_frame → tag_<id>
└───────────────────────┘

┌───────────────────────┐
│  camera_tf_static     │  static_transform_publisher
│  (tf2_ros)            │  base_link → camera_optical_frame
│                       │  values from onboard.yaml camera_tf section
│                       │  [PLACEHOLDER values — not yet measured]
└───────────────────────┘

    /apriltag/detections ──► [NOT YET CONSUMED]
                              Precision landing node needed here (Tier 2)
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
    │  static — onboard.yaml camera_tf  [PLACEHOLDER values]
    ▼
camera_optical_frame  (X-right, Y-down, Z-into-scene)
    │  dynamic — apriltag_ros, one per detection
    ▼
tag_<id>  (origin at tag centre, Z pointing out of tag face)
```

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

## Battery State Estimation

- Full: `cell_count × 4.2 V`  |  Empty: `cell_count × 3.5 V`
- Default: 6S (`cell_count=6`, hardcoded in `main()`)
- Abort at 20% → LAND state
