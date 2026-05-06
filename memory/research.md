# Research — Flight Visualization Graphs (2026-04-30)

## Task 1: SITL Position Logger + Visualizer

### Approach (updated — real SITL telemetry, not synthetic data)
- A ROS2 subscriber node runs on the ground laptop during a SITL session
- Subscribes to `/<drone>/local_position/pose` (PoseStamped, BEST_EFFORT QoS per ARCHITECTURE.md)
- Writes every pose message to a timestamped CSV
- After the flight, a separate standalone Python script reads the CSV and generates plots
- QGC connects to SITL via UDP 14550 for live display; this logger runs alongside it independently

### Topic
| Topic | Type | QoS | Notes |
|---|---|---|---|
| `/<d>/local_position/pose` | `geometry_msgs/PoseStamped` | BEST_EFFORT / VOLATILE | ENU frame, metres, published by MAVROS at ~10–50 Hz |

### CSV Schema
```
timestamp_sec, x_m, y_m, z_m
```
- `timestamp_sec`: elapsed seconds since logging started (float)
- `x_m`: East (metres)
- `y_m`: North (metres)
- `z_m`: Up / altitude (metres)

### Logger Node Design
- Package: `capstone` (runs on ground laptop, same machine as controller)
- File: `capstone/capstone/flight_logger.py`
- Entry point: `flight_logger`
- Parameters: `drone_name` (str, default `drone1`), `log_dir` (str, default `~/flight_logs`)
- Runs standalone: `ros2 run capstone flight_logger --ros-args -p drone_name:=drone1`
- Filename: `flight_YYYYMMDD_HHMMSS.csv` created at node start
- Prints: `[logger] t=12.3s  x=+1.23  y=+0.45  z=+4.98` at ~2 Hz to confirm it's running

### Visualizer Script
- Pure Python/matplotlib, no ROS2: `scripts/plot_flight_log.py`
- CLI: `python3 scripts/plot_flight_log.py <path/to/csv>`
- Detects phase boundaries from the data (altitude thresholds + plateau detection)
  OR accepts optional `--phases` argument with manual timestamps if auto-detection is ambiguous
- Produces 4 plots saved to `graphs/`:
  1. X (East) vs time
  2. Y (North) vs time
  3. Z (altitude) vs time
  4. 3D trajectory with phase labels and directional arrows

### Phase Auto-detection Logic (for plot annotations)
| Phase | Heuristic |
|---|---|
| TAKEOFF | z rising from ~0 |
| CRUISE/SEARCH | z stable at altitude, circular x/y motion |
| TRANSIT | z stable, linear x/y displacement |
| LAND | z falling to ~0 |

---

## Task 2: AprilTag Relative Position Logger + Visualizer

### Context
- SITL has no camera — this logger is for real hardware flights on the Pi
- Runs on Pi (Jazzy) alongside the full onboard stack
- No SITL use case for this task

### Topic
- **Subscribe**: `/apriltag/detections` — `apriltag_msgs/AprilTagDetectionArray`
- **Note confirmed in previous session**: `apriltag_msgs` on this Pi has no `.pose` field
- Must use `tf2_ros.Buffer.lookup_transform('camera', 'tag36h11:{id}', ...)` for 3D position
- Camera optical frame: X=right, Y=down, Z=depth (range into scene)

### CSV Schema
```
timestamp_sec, tag_id, x_m, y_m, z_m, lateral_m
```
- `timestamp_sec`: elapsed seconds since first detection (float)
- `tag_id`: int — which tag ID detected
- `x_m`: metres right of center in camera frame
- `y_m`: metres down from center in camera frame
- `z_m`: range/depth to tag (altitude proxy)
- `lateral_m`: sqrt(x²+y²) — total lateral offset

### Node Design
- Package: `onboard`, file: `onboard/onboard/apriltag_logger.py`, entry point: `apriltag_logger`
- Runs standalone on Pi: `ros2 run onboard apriltag_logger`
- Parameters: `tag_family` (str, default `36h11`), `log_dir` (str, default `~/apriltag_logs`), `warn_timeout` (float, default 2.0)
- After flight: `scp hare@192.168.168.105:~/apriltag_logs/apriltag_*.csv .`

### Visualizer Script
- Pure Python/matplotlib, no ROS2: `scripts/plot_apriltag_log.py`
- CLI: `python3 scripts/plot_apriltag_log.py <path/to/csv>`
- Produces 5 plots saved to `graphs/`:
  1. X offset vs time
  2. Y offset vs time
  3. Z (range) vs time
  4. 2D top-down scatter with time-gradient colormap
  5. Combined 2×2 subplot (report-ready)

---

## File Structure
```
master_src/
├── scripts/
│   ├── plot_flight_log.py          # Task 1 visualizer — standalone
│   └── plot_apriltag_log.py        # Task 2 visualizer — standalone
├── capstone/
│   └── capstone/
│       └── flight_logger.py        # Task 1 logger — ROS2, ground laptop
├── capstone/setup.py               # add flight_logger entry point
├── onboard/
│   └── onboard/
│       └── apriltag_logger.py      # Task 2 logger — ROS2, Pi
├── onboard/setup.py                # add apriltag_logger entry point
└── graphs/                         # output directory (gitignored)
```

---

## Key Constraints
- Visualizer scripts: zero ROS2 dependency, runnable on ground laptop offline
- All plots: labeled axes with units, titles, phase markers, tight_layout, saved as PNG
- Timestamped filenames prevent overwriting between runs
- Logger QoS must match MAVROS: BEST_EFFORT / VOLATILE / KEEP_LAST
- `flight_logger` runs on ground laptop alongside controller.py (same ROS domain)
- `apriltag_logger` runs on Pi (ROS_LOCALHOST_ONLY=1 — only sees local Pi topics)
