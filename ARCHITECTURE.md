# ARCHITECTURE.md — Node Graph, Topics, and Coordinate Frames

## System Split

```
┌─────────────────────────────┐        ┌────────────────────────────────────────┐
│      GROUND LAPTOP          │        │          PI 4  (onboard)               │
│                             │        │                                        │
│  ros2 run capstone          │        │  ros2 launch onboard onboard.launch.py │
│         controller          │        │                                        │
│                             │        │  ┌──────────────┐                      │
│  (controller.py)            │        │  │  mavros_node │ ← FCU ttyACM0        │
│                             │        │  │  /<drone>/   │   921600 baud        │
│  SSH workflow:              │        │  └──────────────┘                      │
│  ssh pi@192.168.168.102     │        │                                        │
│  mavproxy.py --master=...   │        │  ┌──────────────┐                      │
│  ros2 run mavros ...        │        │  │  camera_node │ ← CSI ribbon camera  │
│  (previously manual)        │        │  │  (camera_ros)│   libcamera backend  │
│                             │        │  └──────────────┘                      │
│                             │        │                                        │
│                             │        │  ┌──────────────┐                      │
│                             │        │  │apriltag_node │                      │
│                             │        │  │(apriltag_ros)│                      │
│                             │        │  └──────────────┘                      │
│                             │        │                                        │
│                             │        │  ┌──────────────┐                      │
│                             │        │  │camera_tf_    │ static TF publisher  │
│                             │        │  │static        │                      │
│                             │        │  └──────────────┘                      │
└─────────────────────────────┘        └────────────────────────────────────────┘
            │  Microhard link (192.168.168.x)  │
            └──────────────────────────────────┘
```

---

## Node Graph (Pi onboard + Ground)

```
  CSI camera
      │ libcamera
      ▼
┌─────────────────┐
│  camera_node    │  name='camera' → topics resolve to /camera/*
│  (camera_ros)   │
│                 │  Publishes:
│  Params:        │  /camera/image_raw   (sensor_msgs/Image, YUYV)
│  camera: 0      │  /camera/camera_info (sensor_msgs/CameraInfo)
│  width: 1280    │
│  height: 720    │
│  framerate: 15  │
└────────┬────────┘
         │ /camera/image_raw
         │ /camera/camera_info
         ▼
┌─────────────────┐         TF2 tree
│  apriltag_node  │─────────────────────────────────────────────────────►
│  (apriltag_ros) │  camera_optical_frame → tag_<id>  (per detection)
│                 │
│  Params:        │  Publishes:
│  family: 36h11  │  /apriltag/detections  (apriltag_msgs/AprilTagDetectionArray)
│  size: 0.166 m  │
│  max_hamming: 0 │
└─────────────────┘

┌─────────────────┐         TF2 tree
│ camera_tf_static│─────────────────────────────────────────────────────►
│ (tf2_ros)       │  base_link → camera_optical_frame  (static, from config)
└─────────────────┘

ArduPilot FCU
      │ serial ttyACM0 @ 921600
      ▼
┌─────────────────┐
│  mavros_node    │  namespace: /<drone_name>
│  (mavros)       │
│                 │  Publishes:  (BEST_EFFORT QoS)
│  Param:         │  /<d>/state                    mavros_msgs/State
│  fcu_url:       │  /<d>/local_position/pose      geometry_msgs/PoseStamped
│  serial://      │  /<d>/global_position/global   sensor_msgs/NavSatFix
│  /dev/ttyACM0   │  /<d>/battery                  sensor_msgs/BatteryState
│  :921600        │  /<d>/rc/in                    mavros_msgs/RCIn
│                 │
│                 │  Services:
│                 │  /<d>/cmd/arming               mavros_msgs/srv/CommandBool
│                 │  /<d>/set_mode                 mavros_msgs/srv/SetMode
│                 │  /<d>/cmd/takeoff              mavros_msgs/srv/CommandTOL
└─────────────────┘
         │
         │ (Microhard link across network)
         ▼
┌─────────────────┐
│  controller     │  (ground laptop — capstone package)
│  (capstone)     │
│                 │  Subscribes: /<d>/state, local_position/pose,
│                 │             global_position/global, battery, rc/in
│                 │  Publishes:  /<d>/setpoint_position/local
│                 │             /<d>/setpoint_position/global
└─────────────────┘

         /apriltag/detections ──► [NOT YET CONSUMED]
                                   Landing guidance node needed here
```

---

## Topic Reference

### camera_ros / camera_node

| Direction | Topic | Message Type | Notes |
|---|---|---|---|
| Publish | `/camera/image_raw` | `sensor_msgs/Image` | Raw YUYV from CSI sensor; node named `camera` so `~/image_raw` → `/camera/image_raw` |
| Publish | `/camera/camera_info` | `sensor_msgs/CameraInfo` | Calibration parameters; populated after calibration |

### apriltag_ros / apriltag_node

| Direction | Topic | Message Type | Notes |
|---|---|---|---|
| Subscribe | `/camera/image_raw` | `sensor_msgs/Image` | Remapped from `image_rect` |
| Subscribe | `/camera/camera_info` | `sensor_msgs/CameraInfo` | Remapped from `camera_info` |
| Publish | `/apriltag/detections` | `apriltag_msgs/AprilTagDetectionArray` | Remapped from `detections`; one entry per detected tag |
| Publish (TF2) | `camera_optical_frame → tag_<id>` | tf2 transform | Published for each detected tag; frame persists until tag lost |

### capstone / controller (ground, unchanged)

| Direction | Topic | Message Type | QoS |
|---|---|---|---|
| Subscribe | `/<drone>/state` | `mavros_msgs/State` | BEST_EFFORT |
| Subscribe | `/<drone>/local_position/pose` | `geometry_msgs/PoseStamped` | BEST_EFFORT |
| Subscribe | `/<drone>/global_position/global` | `sensor_msgs/NavSatFix` | BEST_EFFORT |
| Subscribe | `/<drone>/battery` | `sensor_msgs/BatteryState` | BEST_EFFORT |
| Subscribe | `/<drone>/rc/in` | `mavros_msgs/RCIn` | BEST_EFFORT |
| Publish | `/<drone>/setpoint_position/local` | `geometry_msgs/PoseStamped` | default |
| Publish | `/<drone>/setpoint_position/global` | `geographic_msgs/GeoPoseStamped` | default |

### capstone / controller — Service Clients

| Service | Type | Notes |
|---|---|---|
| `/<drone>/cmd/arming` | `mavros_msgs/srv/CommandBool` | Arm/disarm |
| `/<drone>/set_mode` | `mavros_msgs/srv/SetMode` | STABILIZE, GUIDED, LAND, RTL |
| `/<drone>/cmd/takeoff` | `mavros_msgs/srv/CommandTOL` | Takeoff to altitude |

---

## Launch File

### Onboard Pi stack: `onboard/launch/onboard.launch.py`

All parameters flow from `onboard/config/onboard.yaml`:

```
onboard.yaml
    │
    ├─ mavros.drone_name      → mavros_node namespace
    ├─ mavros.fcu_url         → mavros_node fcu_url param
    │
    ├─ camera.{width,height,  → camera_node params
    │          framerate}
    │
    ├─ apriltag.{family,      → apriltag_node params
    │            size,
    │            max_hamming}
    │
    └─ camera_tf.{x,y,z,      → static_transform_publisher args
                  roll,pitch,
                  yaw}
```

**To run on Pi:**
```bash
ros2 launch onboard onboard.launch.py
```

**No launch file exists for the ground `capstone` package** — controller is still run manually:
```bash
cd /home/dfec/master_src/capstone/capstone
python3 controller.py
```

---

## Coordinate Frames

### TF2 Tree (after onboard launch)

```
base_link  (drone body: X-forward, Y-left, Z-up)
    │
    │  static transform from onboard.yaml camera_tf section
    │  (PLACEHOLDER VALUES — must be measured from physical mount)
    ▼
camera_optical_frame  (X-right, Y-down, Z-into-scene)
    │
    │  dynamic, published per detection by apriltag_ros
    ▼
tag_<id>  (one frame per detected tag, origin at tag centre)
```

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
- Units: metres
- Controller uses this for all waypoint and circle setpoints

### Global Position Frame

Used by MAVROS `global_position/global` and `setpoint_position/global`.
- Latitude (°), Longitude (°), Altitude (m AMSL)
- Used in circle mode when GPS is available

### Camera Optical Frame (`camera_optical_frame`)

Per ROS REP-103 optical convention:

```
Z (into scene / forward through lens)
│
│   Y (down in image)
│  /
│ /
└────── X (right in image)
```

- Origin: camera optical centre
- Units: metres
- Pose of each detected tag is expressed in this frame by `apriltag_ros`
- Static transform `base_link → camera_optical_frame` defined in `onboard.yaml`

---

## State Machine (capstone/controller) — unchanged

```
┌───────────────────────────────────────────────────────────┐
│                      RC_TAKEOVER                          │
│  (any state → RC_TAKEOVER when mode changes to RTL)      │
└───────────────────────────────────────────────────────────┘

                    ┌─────────┐
               ┌───►│   ARM   │◄──────────────────────┐
               │    └────┬────┘                        │
               │    ┌────▼────┐                        │
               │    │WAYPOINT │◄──────────────────────┐│
               │    └────┬────┘  user: <d> x y z      ││
               │    ┌────▼────┐                        ││
               │    │  HOVER  │────────────────────────┘│
               │    └────┬────┘                         │
               │    ┌────▼────┐                         │
               │    │ CIRCLE  │                         │
               │    └────┬────┘                         │
               │    ┌────▼────┐                         │
               └────┤  LAND   │─────────────────────────┘
                    └─────────┘
                    (altitude < 0.1 m → reset to ARM)
```

---

## Threading Model (capstone/controller)

```
Main thread:          rclpy.spin()  ← processes all ROS2 callbacks
Thread 1 (daemon):    state_machine_loop() at 50 Hz
Thread 2 (daemon):    user_input_loop()   ← blocks on input()
```

---

## Service Namespace Discovery (capstone/controller)

Tries in order:
```
1. /<ns>/cmd/arming
2. /<ns>/mavros/cmd/arming
3. /mavros<ns>/cmd/arming
4. /mavros/cmd/arming
```

---

## Battery State Estimation (capstone/controller)

- Full: `cell_count × 4.2 V`
- Empty: `cell_count × 3.5 V`
- Default: 6S (`cell_count=6`)
- Abort threshold: 20% → LAND

---

## Missing Connections (Gaps)

```
apriltag_ros  ──/apriltag/detections──►  [NOT YET CONSUMED]
                                          ↑
                                          Landing guidance node
                                          needed here (Tier 3)
```

AprilTag detections and TF2 tag frames are published but nothing
subscribes to them. The controller's LAND state still descends blindly.
The next major work item is a precision landing node that consumes
`/apriltag/detections` and adjusts `setpoint_position/local` during descent.
