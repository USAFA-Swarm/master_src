# CLAUDE.md — Project Context for master_src

## Project Overview

**Robot Teaming Capstone** — ROS2-based autonomous multi-drone control system using ArduPilot flight controllers over MAVROS. The primary goal is coordinated drone formation flight, including circle patterns and waypoint navigation, with emerging work toward AprilTag-guided precision landing.

Maintained by: Dr. Baek (USAFA Capstone, CE/CS 499)
Contact: c26ty.hubert@afacademy.af.edu

---

## Technology Stack

| Layer | Technology |
|---|---|
| OS | Ubuntu (Linux 6.8.0-90-generic) |
| ROS2 distro | Humble (assumed, Humble/Iron era) |
| FCU middleware | MAVROS (ArduPilot dialect) |
| Flight controller | ArduPilot (GUIDED mode) |
| Vision | pupil_apriltags, OpenCV, cv_bridge |
| Language | Python 3 (all packages) |
| Build system | ament_python (setup.py + setup.cfg) |
| Camera driver | usb_cam (ROS2 package) |

---

## Workspace Structure

```
/home/dfec/master_src/          ← ROS2 workspace src/
├── capstone/                   ← MAIN: multi-drone MAVROS controller
│   ├── capstone/
│   │   ├── __init__.py
│   │   └── controller.py       ← PRIMARY NODE (production-ready)
│   ├── CONTROLLER_DOCUMENTATION.md
│   ├── package.xml             ← EMPTY — needs content
│   ├── setup.py
│   └── setup.cfg
│
├── apriltag/                   ← AprilTag vision detection
│   ├── apriltag/
│   │   ├── __init__.py
│   │   ├── apriltag.py         ← AprilTag detector node (functional, has bugs)
│   │   └── image_capture.py    ← Calibration image capture utility
│   ├── config/
│   │   └── default_cam.yaml    ← EMPTY — camera calibration missing
│   ├── launch/
│   │   └── vision.lauch.CHANGE_ME.py  ← Incomplete launch file (filename typo)
│   ├── package.xml             ← EMPTY — needs content
│   ├── setup.py
│   └── setup.cfg
│
├── move2hover/                 ← Simple velocity-based hover controller
│   ├── move2hover/
│   │   └── move2hover.py       ← cmd_vel controller (not MAVROS)
│   ├── README_FIXES.md
│   ├── package.xml             ← EMPTY — needs content
│   ├── setup.py
│   └── setup.cfg
│
├── comms_interface/            ← Diagnostic/telemetry monitor stubs
│   ├── comms_interface/
│   │   ├── __init__.py
│   │   ├── telemetry.py        ← IMU/GPS/battery logger
│   │   └── status.py           ← State/sys_status logger
│   ├── package.xml             ← EMPTY — needs content
│   ├── setup.py
│   └── setup.cfg
│
└── drone_interface/            ← EMPTY STUB (no implementation)
    ├── drone_interface/
    │   └── __init__.py
    ├── package.xml             ← EMPTY — needs content
    └── setup.py
```

---

## Package Summary

### `capstone` — Primary Controller
- **Node:** `controller` (class `Controller(Node)`)
- **Entry point:** `capstone.controller:main`
- **Status:** Production-ready, fully functional
- **Multi-drone:** Yes — accepts list of drone namespaces at construction
- **State machine states:** `ARM → WAYPOINT → HOVER → CIRCLE → LAND → RC_TAKEOVER`

### `apriltag` — Vision Detection
- **Node:** `apriltag_node` (class `AprilTagNode(Node)`)
- **Entry point:** `apriltag.apriltag:main`
- **Status:** Code functional but broken by package name bug and missing camera config
- **Also has:** `image_saver` utility node for capturing calibration images

### `move2hover` — Simple Hover Controller
- **Node:** `move_to_goal` (class `MoveToGoal(Node)`)
- **Entry point:** `move2hover.move2hover:main`
- **Status:** Functional but uses `cmd_vel` (not MAVROS), hardcoded goals
- **Note:** Different architecture from capstone; likely a lab exercise artifact

### `comms_interface` — Telemetry Stubs
- **Nodes:** `telemetry`, `status_node`
- **Status:** Logging-only stubs, hardcoded to `/smaug2/*` topics

### `drone_interface` — Empty Stub
- **Status:** No implementation; entry point references nonexistent file

---

## Drone Namespace Convention

Drones are identified by a name string (e.g., `smaug1`, `smaug2`, `sim1`).
All MAVROS topics follow the pattern: `/<drone_name>/<topic>`

Example for `smaug2`:
- `/smaug2/state`
- `/smaug2/local_position/pose`
- `/smaug2/battery`

The capstone controller auto-discovers the correct service namespace by trying three patterns:
1. `/<ns>/cmd/arming`
2. `/<ns>/mavros/cmd/arming`
3. `/mavros<ns>/cmd/arming`

**MAVROS launch:** Connect to SITL via:
```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://@10.1.119.44:11101 -r __ns:=/sim1
```

---

## ROS2 QoS Convention

The capstone controller uses MAVROS-compatible QoS on all subscriptions to MAVROS topics:
```python
QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)
```
Publishers to MAVROS setpoint topics use default QoS.

---

## Key Parameters & Constants

| Parameter | Value | Location |
|---|---|---|
| Waypoint tolerance | 0.6 m | controller.py |
| Battery abort threshold | 20% (0.20) | controller.py |
| Circle waypoints per loop | 17 + 1 close | controller.py |
| Circle direction | Clockwise | controller.py |
| AprilTag family | tag36h11 | apriltag.py |
| AprilTag physical size | 0.166 m | apriltag.py |
| Camera pixel format | YUYV (yuv422_yuy2) | apriltag.py |
| move2hover loop rate | 10 Hz | move2hover.py |
| State machine loop rate | 50 Hz | controller.py |

---

## Known Bugs & Issues

| Severity | Package | Issue |
|---|---|---|
| HIGH | `apriltag` | `apriltag.py:88` uses `get_package_share_directory('lab11_apriltag')` — wrong package name, should be `'apriltag'` |
| HIGH | `apriltag` | `config/default_cam.yaml` is empty — node will crash without a valid camera calibration file |
| HIGH | `drone_interface` | Entry point references `drone_interface.controller:main` which does not exist |
| MEDIUM | `apriltag` | Launch file named `vision.lauch.CHANGE_ME.py` — typo "lauch", needs rename |
| MEDIUM | `apriltag` | Launch file references `stop_detector` executable that doesn't exist in this package |
| MEDIUM | `apriltag` | `config/CHANGE_ME.svm` referenced in setup.py but file doesn't exist |
| MEDIUM | `comms_interface` | Entry point typo: `telemtry` instead of `telemetry` |
| LOW | `comms_interface` | All topics hardcoded to `/smaug2/*` |
| LOW | `move2hover` | Topics hardcoded; goal position hardcoded |
| LOW | all packages | `package.xml` files are empty (no declared dependencies, maintainer, or license) |
| LOW | `apriltag` | `apriltag.py:188` has extra leading space in indentation |

---

## What Does NOT Exist Yet

- **AprilTag-guided precision landing** — the capstone controller has a `LAND` state but it descends blindly; no node consumes `/apriltag_pose` to guide landing
- **Any launch file for `capstone`** — controller is launched manually with `ros2 run`
- **Complete package.xml files** — all are empty
- **Camera calibration data** — `default_cam.yaml` is empty
- **Any inter-node communication** between apriltag and capstone
- **Action servers or services** for mission commanding (all commanding is via stdin)

---

## Git Conventions

- Branch: `main`
- Remote: `https://github.com/USAFA-Swarm/master_src`
- Commit style: short imperative messages (e.g., "Working battery readings", "Working code with RC relinquish")
