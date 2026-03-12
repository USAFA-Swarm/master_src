# TASKS.md — What Is Done, In Progress, and Needs Building

Tasks are ordered by dependency — complete earlier tasks before later ones.

---

## TIER 0 — Completed (Production Ready)

### [DONE] Multi-drone MAVROS controller
- **File:** `capstone/capstone/controller.py`
- ARM → WAYPOINT → HOVER → CIRCLE → LAND → RC_TAKEOVER state machine
- Multi-drone namespace support; battery abort at 20%; RC takeover detection
- Circle formation flight (17+1 waypoints, phase-offset multi-drone)
- GPS and local ENU setpoints; full service namespace discovery
- **Docs:** `capstone/CONTROLLER_DOCUMENTATION.md`

### [DONE] Package metadata (package.xml)
- All packages have correct format-3 `package.xml` with proper dependencies
- `capstone`, `apriltag`, `onboard` — all complete

### [DONE] Bug fixes (from initial analysis)
- `apriltag.py:88` — wrong package name `lab11_apriltag` → `apriltag`
- `apriltag.py:188` — indentation inconsistency fixed
- `apriltag/config/default_cam.yaml` — populated with valid-structure template
- `comms_interface` — removed (passive stubs, never used operationally)
- `move2hover` — removed (cmd_vel architecture, incompatible with MAVROS)
- `drone_interface` — removed (empty stub, no implementation)
- Runtime artifacts (`eeprom.bin`, `mav.tlog*`) — removed from repo

### [DONE] Camera subsystem — onboard Pi stack
- **Package:** `onboard/`
- **Launch:** `onboard/launch/onboard.launch.py`
- **Config:** `onboard/config/onboard.yaml` — single YAML for all parameters
- Starts `mavros_node` (namespace from config), `camera_node` (libcamera/CSI),
  `apriltag_node` (apriltag_ros with TF2), `camera_tf_static`
- Replaces the manual SSH + individual `ros2 run` workflow
- **Run on Pi:** `ros2 launch onboard onboard.launch.py`

### [DONE] AprilTag detection (apriltag_ros, with TF2)
- Uses `ros-humble-apriltag-ros` (christianrauch) — full TF2 support
- Subscribes `/camera/image_raw` + `/camera/camera_info`
- Publishes `/apriltag/detections` (`apriltag_msgs/AprilTagDetectionArray`)
- Publishes TF2: `camera_optical_frame → tag_<id>` per detected tag

### [DONE] Static TF: body → camera
- `static_transform_publisher` launched by `onboard.launch.py`
- `base_link → camera_optical_frame`
- **Values are PLACEHOLDERS** — must be measured from physical mount (see Tier 1)

### [DONE] Image capture utility (calibration tool)
- **File:** `apriltag/apriltag/image_capture.py`
- Run directly with `python3 image_capture.py -o /path/to/save`
- Saves images on 's' or Enter keypress for use with `camera_calibration`

---

## TIER 1 — Must Do Before First Flight (Physical Steps)

### [TODO] Camera calibration — replace placeholder values in default_cam.yaml

The `apriltag/config/default_cam.yaml` has valid structure but placeholder
intrinsic values (fx=fy=600, zero distortion). AprilTag pose estimates will be
wrong until real calibration data is substituted.

**Procedure:**

```bash
# 1. On Pi: start the camera
ros2 launch onboard onboard.launch.py

# 2. On ground laptop: run the calibration tool
#    Use a 8x6 checkerboard with 25mm squares (adjust --size/--square to match yours)
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  image:=/camera/image_raw \
  camera:=/camera

# 3. Move the checkerboard: fill all X/Y/Size/Skew bars to green, then click CALIBRATE
# 4. Click SAVE — writes calibration to /tmp/calibrationdata.tar.gz
# 5. Extract and copy camera_matrix + distortion_coefficients into:
#      apriltag/config/default_cam.yaml
```

**What to check:** After calibration, `reprojection error < 0.5 px` is acceptable.
Larger error means the checkerboard wasn't fully covered; redo it.

### [TODO] Measure and set camera mount transform

`onboard/config/onboard.yaml` section `camera_tf` has placeholder x/y/z/roll/pitch/yaw.
These define the static TF from `base_link` (body centre) to `camera_optical_frame`.

**Procedure:**
1. Measure the physical offset of the camera lens from the drone centre of mass:
   - `x` = forward offset in metres
   - `y` = left offset in metres
   - `z` = upward offset in metres (negative if camera is below body)
2. Determine the camera rotation relative to body:
   - Downward-facing, ribbon forward: `roll=3.14159, pitch=0, yaw=0`
   - Downward-facing, ribbon aft: `roll=3.14159, pitch=0, yaw=3.14159`
   - Adjust if the camera is tilted or rotated in the mount
3. Update `onboard/config/onboard.yaml` `camera_tf` section
4. Rebuild and redeploy: `colcon build --packages-select onboard`

### [TODO] Verify tag_size matches printed tags

`onboard/config/onboard.yaml` has `apriltag.tag_size: 0.166` (166 mm).
Measure your actual printed tags edge-to-edge (outer black border).
Update the config if different. Pose estimation distance scales linearly with this value.

### [TODO] Install dependencies on Pi

On the Pi, install required ROS2 packages before first launch:

```bash
sudo apt update
sudo apt install \
  ros-humble-camera-ros \
  ros-humble-apriltag-ros \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-tf2-ros \
  python3-yaml

# ArduPilot GeographicLib data (required by MAVROS)
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

---

## TIER 2 — Next Development Work

### [TODO] Precision landing node

The core unimplemented capability. `/apriltag/detections` is published but
nothing subscribes to it. The controller's LAND state descends blindly.

**What's needed:**
1. New node `capstone/capstone/landing.py` (or integrated into controller)
2. Subscribe to `/apriltag/detections`
3. When a landing tag is visible:
   - Extract tag pose from detection (in `camera_optical_frame`)
   - Use TF2 to transform tag position into ENU body frame
   - Compute lateral offset: `(tag_x_body, tag_y_body)` = how far drone is from tag centre
   - During LAND state: publish corrected `setpoint_position/local` to close the offset
   - Reduce altitude setpoint at a controlled rate (e.g. 0.2 m/s)
4. When tag is lost: hold position, log warning, continue blind descent after timeout

**Key architecture decision:** standalone node vs controller integration
- Standalone: cleaner separation, can be disabled without modifying controller
- Integrated: direct access to state machine, avoids extra topic latency

**New topic consumed:** `/apriltag/detections` (`apriltag_msgs/AprilTagDetectionArray`)
**Existing topic written:** `/<drone>/setpoint_position/local` (`geometry_msgs/PoseStamped`)
**TF2 needed:** `camera_optical_frame → base_link` (already published by onboard stack)

### [TODO] Assign tag IDs per drone for multi-drone landing

Each drone needs a unique landing pad with a unique AprilTag ID.
The precision landing node must filter detections by expected tag ID for that drone.

**Config addition needed in `onboard.yaml`:**
```yaml
apriltag:
  landing_tag_id: 0    # tag ID assigned to this drone's landing pad
```

### [TODO] Capstone launch file (ground side)

Currently the controller is started manually with `python3 controller.py`.
A launch file would make this cleaner and support parameter passing.

**File to create:** `capstone/launch/capstone.launch.py`
```python
Node(package='capstone', executable='controller', output='screen')
```

### [TODO] SITL launch file (simulation)

A launch file that starts MAVROS with SITL FCU URL for development without hardware.

```yaml
# sitl.yaml
mavros:
  drone_name: "sim1"
  fcu_url: "udp://@10.1.119.44:11201"
```

**File to create:** `onboard/launch/sitl.launch.py` (or separate `sim` package)

---

## TIER 3 — Polish and Robustness

### [TODO] Landing detection using ExtendedState
- Currently LAND state checks `pose.z < 0.1 m`
- Better: subscribe `mavros_msgs/ExtendedState` → `landed_state == LANDED`

### [TODO] Position hold on RC_TAKEOVER exit
- Currently: returns to ARM state and re-arms on any new command
- Better: resume HOVER at current position if already airborne

### [TODO] Configurable cell_count parameter
- `cell_count=6` hardcoded in `controller.py` main()
- Declare as ROS2 parameter so 4S/6S can be selected without editing code

### [TODO] Integration tests
- No tests exist in any package
- Minimum: smoke test that all nodes initialise without crashing

---

## Dependency Graph

```
[Tier 1 — Physical]
    ├── Camera calibration (default_cam.yaml)
    ├── Camera mount transform (onboard.yaml camera_tf)
    ├── Verify tag_size
    └── Install Pi dependencies
            │
            ▼
[Tier 2 — Development]
    ├── Precision landing node       ← main effort
    ├── Per-drone tag ID config
    ├── Capstone launch file
    └── SITL launch file
            │
            ▼
[Tier 3 — Polish]
    ├── ExtendedState landing detection
    ├── RC_TAKEOVER position hold
    ├── cell_count parameter
    └── Integration tests
```

---

## Quick Reference: Running the System

### Real drone (Pi + ground laptop)

**Pi** (via SSH: `ssh pi@192.168.168.102`):
```bash
ros2 launch onboard onboard.launch.py
```
This replaces the previous manual sequence of:
```bash
mavproxy.py --master=/dev/ttyACM0 --baudrate 921600 --out=udp:...
ros2 run mavros mavros_node --ros-args -r __ns:=/drone1 -p fcu_url:=...
```

**Ground laptop:**
```bash
cd /home/dfec/master_src/capstone/capstone
python3 controller.py
# Enter drone names: drone1
```

### Simulation (SITL)

**Ground laptop — Terminal 1:**
```bash
mavproxy.py --master=udpout:10.1.119.44:11101 --out=udp:10.1.119.44:15101
```
**Ground laptop — Terminal 2:**
```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://@10.1.119.44:11201 -r __ns:=/sim1
```
**Ground laptop — Terminal 3:**
```bash
cd /home/dfec/master_src/capstone/capstone
python3 controller.py
# Enter drone names: sim1
```

---

## Removed

- `drone_interface/` — empty stub, entry point referenced nonexistent module
- `move2hover/` — cmd_vel architecture incompatible with MAVROS, not integrated
- `apriltag/launch/vision.lauch.CHANGE_ME.py` — typo, incomplete, referenced nonexistent stop_detector
- `apriltag/config/CHANGE_ME.svm` — placeholder for nonexistent stop_detector node
- `apriltag/setup.py` dead CHANGE_ME.svm data_files entry — removed
- `eeprom.bin` — ArduPilot EEPROM runtime dump, not source code
- `mav.tlog` — MAVLink telemetry log, not source code
- `mav.tlog.raw` — MAVLink telemetry log (raw), not source code
- `comms_interface/` — passive logging stubs, never used operationally

---

## Fixed

- `capstone/package.xml` — was empty; wrote full manifest with 5 deps
- `apriltag/package.xml` — wrong opencv2 dep, spurious ament_cmake, missing 4 deps; corrected
- `comms_interface/package.xml` — was empty; written (then package removed)
- `comms_interface/setup.py` — entry point typo `telemtry` → `telemetry` (then package removed)
- `apriltag/apriltag/apriltag.py:88` — `lab11_apriltag` → `apriltag` package name
- `apriltag/apriltag/apriltag.py:188` — extra leading space in indentation removed
- `apriltag/config/default_cam.yaml` — was empty/crash; replaced with valid-structure template
- `apriltag/launch/vision.launch.py` — created correctly-named replacement (old had typo, missing stop_detector)
