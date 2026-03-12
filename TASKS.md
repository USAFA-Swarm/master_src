# TASKS.md — What Is Done, In Progress, and Needs Building

Tasks are ordered by dependency — complete earlier tasks before later ones.

---

## TIER 0 — Completed (Production Ready)

### [DONE] Multi-drone MAVROS controller
- **File:** `capstone/capstone/controller.py`
- **Features:** ARM, WAYPOINT, HOVER, CIRCLE, LAND, RC_TAKEOVER states
- **Features:** Multi-drone support with namespaces
- **Features:** Battery monitoring with 20% abort threshold
- **Features:** Circle pattern (17+1 waypoints, clockwise, phase-offset formation)
- **Features:** RC takeover detection and safe handoff
- **Features:** GPS and local ENU frame setpoints
- **Features:** Threading model (50Hz state machine + input loop)
- **Documentation:** Full CONTROLLER_DOCUMENTATION.md exists

### [DONE] AprilTag detection node (core logic)
- **File:** `apriltag/apriltag/apriltag.py`
- **Features:** YUYV image input, undistortion, pupil_apriltags detection
- **Features:** Pose estimation (position + quaternion), publishes `/apriltag_pose`
- **Status:** Code is complete and correct; blocked by config/naming bugs (see Tier 1)

### [DONE] Image capture utility
- **File:** `apriltag/apriltag/image_capture.py`
- **Features:** Live camera feed, save images on keypress for calibration

---

## TIER 1 — Bug Fixes (Must Do Before Any Testing)

These are broken things that prevent existing code from running.

### [TODO] Fix apriltag package name bug
- **File:** `apriltag/apriltag/apriltag.py:88`
- **Fix:** Change `get_package_share_directory('lab11_apriltag')` → `get_package_share_directory('apriltag')`
- **Impact:** Node crashes on startup without a `camera_info_file` parameter
- **Effort:** 1 line

### [TODO] Populate camera calibration YAML
- **File:** `apriltag/config/default_cam.yaml`
- **Fix:** Run camera calibration (ros2 run camera_calibration cameracalibrator) and populate:
  ```yaml
  camera_matrix:
    data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
  distortion_coefficients:
    data: [k1, k2, p1, p2, k3]
  ```
- **Impact:** AprilTag node cannot load camera parameters; crashes immediately
- **Effort:** Physical calibration procedure required

### [TODO] Fix launch file filename typo
- **File:** `apriltag/launch/vision.lauch.CHANGE_ME.py`
- **Fix:** Rename to `vision.launch.py`
- **Effort:** 1 git mv

### [TODO] Fix comms_interface entry point typo
- **File:** `comms_interface/setup.py`
- **Fix:** Change `telemtry` → `telemetry` in console_scripts
- **Effort:** 1 character

### [TODO] Write all package.xml files
- **Files:** All 5 packages have empty `package.xml`
- **Fix:** Each needs `<package>`, `<name>`, `<version>`, `<description>`, `<maintainer>`, `<license>`, and `<depend>` entries
- **Impact:** `colcon build` will fail or warn; rosdep cannot resolve dependencies
- **Effort:** ~20 lines per package

**Minimum required dependencies per package:**

| Package | Key depends |
|---|---|
| `capstone` | `rclpy`, `geometry_msgs`, `geographic_msgs`, `sensor_msgs`, `mavros_msgs` |
| `apriltag` | `rclpy`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`, `python3-opencv`, `python3-pupil-apriltags`, `python3-scipy` |
| `move2hover` | `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `python3-tf-transformations` |
| `comms_interface` | `rclpy`, `sensor_msgs`, `mavros_msgs`, `diagnostic_msgs` |
| `drone_interface` | TBD (no implementation yet) |

---

## TIER 2 — Completing Partial Implementations

### [TODO] Complete the apriltag launch file
- **File:** `apriltag/launch/vision.launch.py` (after rename)
- **Fix:** Remove TODOs, verify node names and parameters, confirm topic remappings match
- **Note:** `stop_detector` node referenced in launch does not exist — remove or implement
- **Effort:** Small — review and clean existing structure

### [TODO] Parameterize comms_interface topics
- **Files:** `comms_interface/comms_interface/telemetry.py`, `status.py`
- **Fix:** Replace hardcoded `/smaug2/*` with ROS2 parameters or namespace remapping
- **Effort:** Small refactor

### [TODO] Parameterize move2hover goals and topics
- **File:** `move2hover/move2hover/move2hover.py`
- **Fix:** Declare `goal_x`, `goal_y`, `goal_z` as ROS2 node parameters
- **Effort:** Small refactor

### [TODO] Create capstone launch file
- **File:** `capstone/launch/capstone.launch.py` (create new)
- **Content:** Launch `mavros_node` for each drone + `controller` node with drone list parameter
- **Effort:** ~30 lines

### [TODO] Implement drone_interface package
- **Files:** `drone_interface/drone_interface/controller.py` (create)
- **Note:** Role of this package vs `capstone` is currently undefined — clarify intent before implementing
- **Options:** (a) remove it, (b) make it a thin wrapper/interface class, (c) give it a distinct role

---

## TIER 3 — New Features (Core Mission Capability)

These depend on Tier 1 and Tier 2 being completed first.

### [TODO] AprilTag-guided precision landing
- **Depends on:** Working apriltag node (Tier 1 fixes), capstone controller LAND state
- **What's needed:**
  1. A new node (or addition to controller) that subscribes to `/apriltag_pose`
  2. Converts camera-frame tag pose to ENU body correction
  3. Feeds corrected setpoints to `/<drone>/setpoint_position/local` during LAND state
  4. Defines descent rate and centering tolerance
- **Architectural decision:** New standalone node vs integrated into controller
- **New topics needed:**
  - Input: `/apriltag_pose` (geometry_msgs/PoseStamped) — already published
  - Output: lateral correction setpoints to MAVROS
- **Frame transforms needed:** camera optical → body → ENU

### [TODO] TF frame publication for camera mount
- **Depends on:** AprilTag landing node design
- **What's needed:** Static transform from body frame to camera frame (based on physical mount)
- **Implementation:** `static_transform_publisher` in launch file or `tf2_ros` in code

### [TODO] Multi-drone AprilTag landing (one tag per drone)
- **Depends on:** Single-drone AprilTag landing
- **What's needed:** Tag ID assignment per drone, per-drone pose subscriptions
- **Consideration:** `/apriltag_pose` currently publishes all tags on one topic without drone namespace — may need remapping or filtering by tag ID

### [TODO] Mission command interface (replace stdin)
- **Current state:** Commands typed via stdin in `user_input_loop()`
- **Goal:** ROS2 action server or topic-based mission commanding
- **Options:**
  - `std_msgs/String` command topic
  - Custom action for each maneuver type
  - ROS2 mission planning integration
- **Effort:** Medium — requires protocol design

---

## TIER 4 — Polish and Robustness

### [TODO] Add position hold during RC_TAKEOVER exit
- Currently: on exit from RC_TAKEOVER, controller returns to ARM state and must re-arm
- Better: resume HOVER at current position

### [TODO] Add configurable cell_count parameter
- Currently: `cell_count=6` hardcoded in `main()` call in controller.py
- Fix: declare as ROS2 parameter

### [TODO] Landing detection using ExtendedState
- Currently: LAND state checks `pose.z < 0.1` for landing confirmation
- Better: subscribe to `mavros_msgs/ExtendedState` and check `landed_state == LANDED`

### [TODO] Simulation / SITL launch file
- A launch file that starts multiple MAVROS instances for multi-drone SITL
- Each drone gets its own FCU URL and namespace

### [TODO] Integration tests
- No tests exist in any package
- Minimum: smoke tests that nodes initialize without crashing

---

## Dependency Graph Summary

```
[Tier 1 Fixes]
    ├── Fix apriltag pkg name bug
    ├── Camera calibration YAML
    ├── Rename launch file
    ├── Fix comms entry point typo
    └── Write package.xml files
            │
            ▼
[Tier 2 Completions]
    ├── Complete launch file
    ├── Parameterize comms
    ├── Capstone launch file
    └── (drone_interface removed)
            │
            ▼
[Tier 3 New Features]
    ├── TF camera→body transform
    ├── AprilTag-guided landing (single drone)
    ├── Multi-drone AprilTag landing
    └── Mission command interface
            │
            ▼
[Tier 4 Polish]
    ├── RC_TAKEOVER position hold
    ├── cell_count parameter
    ├── ExtendedState landing detection
    ├── SITL launch file
    └── Integration tests
```

---

## Quick Reference: What to Build Next

If starting fresh on the AprilTag landing feature, the minimum path is:

1. Fix `apriltag.py:88` package name (5 min)
2. Populate `default_cam.yaml` with real calibration (requires calibration session)
3. Rename launch file (1 min)
4. Write `package.xml` for both `apriltag` and `capstone` (15 min)
5. Design and implement precision landing node (main effort)
6. Add TF static transform for camera mount (10 min)
7. Integrate landing node trigger into capstone LAND state

---

## Removed

- `drone_interface/` — entire package deleted; only contained an empty `__init__.py` and an entry point referencing a nonexistent `controller.py` module; no implementation existed
- `move2hover/` — entire package deleted; uses `cmd_vel`/odometry architecture incompatible with MAVROS/ArduPilot; hardcoded goals and topics; no connection to any other package in the workspace
- `apriltag/launch/vision.lauch.CHANGE_ME.py` — deleted; filename typo ("lauch"), unresolved TODO stubs throughout, and references a `stop_detector` executable that does not exist in this workspace
- `apriltag/config/CHANGE_ME.svm` — deleted; placeholder file for the nonexistent `stop_detector` node; zero real content
- `apriltag/setup.py` line referencing `config/CHANGE_ME.svm` — removed dead `data_files` entry pointing to the deleted placeholder
- `eeprom.bin` — deleted; ArduPilot EEPROM runtime dump, not source code, should not be in version control
- `mav.tlog` — deleted; MAVLink telemetry runtime log, not source code, should not be in version control
- `mav.tlog.raw` — deleted; MAVLink telemetry runtime log (raw), not source code, should not be in version control

---

## Fixed

Fixes applied in dependency order (foundational → code-level → config → launch).

### package.xml — capstone
- `capstone/package.xml` was empty; wrote full format-3 manifest declaring `rclpy`, `geometry_msgs`, `geographic_msgs`, `sensor_msgs`, `mavros_msgs`; without this `colcon build` cannot resolve dependencies

### package.xml — apriltag
- `apriltag/package.xml` had wrong maintainer, wrong `opencv2` dep (not a rosdep key), spurious `buildtool_depend ament_cmake` (package is ament_python), and was missing `geometry_msgs`, `python3-numpy`, `python3-yaml`, `python3-scipy`; corrected all entries

### package.xml — comms_interface
- `comms_interface/package.xml` was empty; wrote full format-3 manifest declaring `rclpy`, `sensor_msgs`, `mavros_msgs`, `diagnostic_msgs`

### comms_interface/setup.py — entry point typo
- Entry point name was `telemtry` and module path was `comms_interface.telemtry:main`; corrected both to `telemetry` / `comms_interface.telemetry:main`; the actual source file is `telemetry.py` so the old entry point would fail at `ros2 run`

### apriltag/apriltag/apriltag.py:88 — wrong package name
- `get_package_share_directory('lab11_apriltag')` changed to `get_package_share_directory('apriltag')`; the old name references a non-existent package causing `PackageNotFoundError` on node startup whenever `camera_info_file` param is not explicitly set

### apriltag/apriltag/apriltag.py:188 — indentation inconsistency
- Extra leading space before `# TODO: Label the tag ID` comment removed; was breaking visual alignment and would cause `W191`/`E101` lint warnings

### comms_interface/telemetry.py — hardcoded drone namespace
- Replaced hardcoded `/smaug2/` topic prefixes with a `drone_name` ROS2 parameter (default `smaug2`); topics built as `f'/{drone}/...'`; node was silently subscribing to the wrong drone on any multi-drone setup

### comms_interface/status.py — hardcoded drone namespace
- Same fix as telemetry.py; `drone_name` parameter controls the `/state`, `/extended_state`, and `/sys_status` topic prefixes; `/diagnostics` is left as an absolute topic since it is system-wide

### apriltag/config/default_cam.yaml — empty calibration file
- File was 0 bytes; `load_camera_info()` would crash with a `KeyError` trying to read `camera_matrix.data`; replaced with a valid-structure YAML template using placeholder values (fx=fy=600, cx=320, cy=240, zero distortion); comments explain how to run `camera_calibration` to produce real values; poses will be inaccurate until real calibration data is substituted

### apriltag/launch/vision.launch.py — missing launch file (was deleted typo version)
- Created `vision.launch.py` (correct spelling) replacing the deleted `vision.lauch.CHANGE_ME.py`; launches `usb_cam_node_exe` and `apriltag_node` with `camera_info_file` parameter pointing to the package config; removed the nonexistent `stop_detector` node that was in the original
