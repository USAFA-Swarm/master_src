# Plan: Flight Visualization — Logger + Graphs (2026-04-30)

## Overview

Two deliverables:
- **Task 1**: `flight_logger` ROS2 node (ground laptop, `capstone` pkg) logs MAVROS pose during SITL → `plot_flight_log.py` generates 4 trajectory plots
- **Task 2**: `apriltag_logger` ROS2 node (Pi, `onboard` pkg) logs TF-derived tag offsets during real hardware flight → `plot_apriltag_log.py` generates 5 offset plots

---

## Files to Create or Modify

| # | File | Action | Runs on |
|---|---|---|---|
| 1 | `capstone/capstone/flight_logger.py` | CREATE | Ground laptop |
| 2 | `capstone/setup.py` | MODIFY — add `flight_logger` entry point | Ground laptop |
| 3 | `scripts/plot_flight_log.py` | CREATE | Ground laptop (offline) |
| 4 | `onboard/onboard/apriltag_logger.py` | CREATE | Pi |
| 5 | `onboard/setup.py` | MODIFY — add `apriltag_logger` entry point | Pi |
| 6 | `scripts/plot_apriltag_log.py` | CREATE | Ground laptop (offline) |

No changes to `controller.py`, `precision_landing.py`, `onboard.launch.py`, or any flight logic.

---

## Step 1 — `capstone/capstone/flight_logger.py`

ROS2 node. Subscribes to `/<drone_name>/local_position/pose`.

**Parameters:**
- `drone_name` (str, default `drone1`)
- `log_dir` (str, default `~/flight_logs`)

**Behavior:**
- Creates `<log_dir>/flight_YYYYMMDD_HHMMSS.csv` on startup
- Writes header row: `timestamp_sec,x_m,y_m,z_m`
- On every pose callback: compute elapsed seconds since node start, write one CSV row
- Prints `[flight_logger] Recording to <path>` on start
- Prints `[flight_logger] t=12.3s  x=+1.23  y=+0.45  z=+4.98` throttled at 2 Hz so terminal is readable
- On Ctrl-C / shutdown: closes CSV file cleanly, prints total rows written

**QoS:** BEST_EFFORT / VOLATILE / KEEP_LAST(10) — must match MAVROS publications per ARCHITECTURE.md

**CSV row example:**
```
12.345,1.230,0.450,4.980
```

---

## Step 2 — `capstone/setup.py`

Add one entry to `console_scripts`:
```python
'flight_logger = capstone.flight_logger:main',
```

---

## Step 3 — `scripts/plot_flight_log.py`

Standalone Python script. No ROS2. Uses only `csv`, `matplotlib`, `numpy`, `argparse`, `pathlib`.

**CLI:** `python3 scripts/plot_flight_log.py <path/to/csv> [--output-dir graphs]`

**Phase auto-detection** (operates on z_m column):
- TAKEOFF: first segment where z rises from < 0.3 m to within 0.5 m of max altitude
- CRUISE/SEARCH: z stable at altitude (std dev of z in rolling window < 0.3 m), x/y changing
- TRANSIT: z stable, large net displacement in x/y (detected as linear motion)
- LAND: z falling from altitude back to < 0.5 m
- Phase boundaries stored as time indices → vertical lines on each time-series plot

**Output — 4 PNG files saved to `--output-dir`:**

| File | Plot |
|---|---|
| `flight_x_TIMESTAMP.png` | X (East, m) vs time (s) — phase boundary vlines + labels |
| `flight_y_TIMESTAMP.png` | Y (North, m) vs time (s) — phase boundary vlines + labels |
| `flight_z_TIMESTAMP.png` | Z (altitude, m) vs time (s) — phase boundary vlines + labels |
| `flight_3d_TIMESTAMP.png` | 3D trajectory, each phase a different color, directional arrows every 20 points, legend |

**TIMESTAMP** = taken from the input CSV filename so plots are linked to their source run.

**Plot style requirements:**
- `figsize=(10, 5)` for 2D plots, `figsize=(10, 8)` for 3D
- Axis labels with units: `"Time (s)"`, `"East (m)"`, `"North (m)"`, `"Altitude (m)"`
- Phase labels on vertical lines (rotated text)
- `tight_layout()` before save
- DPI 150

---

## Step 4 — `onboard/onboard/apriltag_logger.py`

ROS2 node. Subscribes to `/apriltag/detections`, uses tf2 for 3D pose.

**Parameters:**
- `tag_family` (str, default `36h11`)
- `log_dir` (str, default `~/apriltag_logs`)
- `warn_timeout` (float, default `2.0`)

**Behavior:**
- Creates `<log_dir>/apriltag_YYYYMMDD_HHMMSS.csv` on first detection (not on startup — avoids empty files)
- Writes header: `timestamp_sec,tag_id,x_m,y_m,z_m,lateral_m`
- On each detection callback: for each detected tag, attempt `lookup_transform('camera', 'tag36h11:{id}', ...)` with 0.1 s timeout; on success, write CSV row + print to terminal
- Prints one line per detected tag per callback: `[apriltag_logger] Tag 0 | z=0.842 m  lateral=0.031 m  x=+0.028 m  y=+0.014 m`
- If no detection for `warn_timeout` seconds and CSV has been started: prints warning

**TF frame naming:** `f'tag{family}:{tid}'` — e.g. `tag36h11:0` — matches what apriltag_ros publishes (confirmed working in tag_offset_display.py)

**QoS on subscription:** default (10 depth) — apriltag publishes best-effort but subscriber depth 10 is fine for logging

---

## Step 5 — `onboard/setup.py`

Add one entry to `console_scripts`:
```python
'apriltag_logger = onboard.apriltag_logger:main',
```

---

## Step 6 — `scripts/plot_apriltag_log.py`

Standalone Python script. No ROS2. Uses `csv`, `matplotlib`, `numpy`, `argparse`, `pathlib`.

**CLI:** `python3 scripts/plot_apriltag_log.py <path/to/csv> [--tag-id 0] [--output-dir graphs]`

`--tag-id` filters to a single tag ID (default: plot all tags, different colors).

**Output — 5 PNG files saved to `--output-dir`:**

| File | Plot |
|---|---|
| `apriltag_x_TIMESTAMP.png` | X offset (m) vs time (s) — horizontal zero line |
| `apriltag_y_TIMESTAMP.png` | Y offset (m) vs time (s) — horizontal zero line |
| `apriltag_z_TIMESTAMP.png` | Z / range (m) vs time (s) |
| `apriltag_topdown_TIMESTAMP.png` | 2D scatter: X vs Y, colormap = time, colorbar, zero crosshairs |
| `apriltag_combined_TIMESTAMP.png` | 2×2 subplot: x(t), y(t), z(t), top-down — report-ready |

**Top-down plot details:**
- X axis: "Right offset (m)", Y axis: "Forward offset (m)" (or "Down offset (m)" — label clearly)
- Scatter colored by `timestamp_sec` using `plt.cm.viridis`
- Colorbar labeled "Time (s)"
- Cross-hair lines at x=0, y=0 (dashed gray) showing tag center
- Title: "Drone Position Relative to Tag Center (top-down)"

**Combined subplot:** `figsize=(12, 10)`, shared x-axis on time plots, `tight_layout()`

---

## Success Criteria

### Task 1
1. `colcon build --packages-select capstone` succeeds on ground laptop
2. `ros2 run capstone flight_logger` starts, prints CSV path, subscribes without error
3. During SITL flight: CSV grows in real-time (`wc -l` increases while drone moves)
4. `python3 scripts/plot_flight_log.py <csv>` runs without error on ground laptop with no ROS2
5. 4 PNG files appear in `graphs/` with correct axis labels and at least one detectable phase boundary

### Task 2
1. `colcon build --packages-select onboard` succeeds on Pi
2. `ros2 run onboard apriltag_logger` starts without error
3. With tag in camera view: CSV row written per detection, terminal output visible
4. `python3 scripts/plot_apriltag_log.py <csv>` runs without error
5. 5 PNG files in `graphs/`; top-down plot shows approach path converging toward (0, 0)

---

## Usage Workflow

### Task 1 (SITL)
```bash
# Terminal 1 — SITL + MAVROS
sim_vehicle.py -v ArduCopter --out udp:127.0.0.1:14551
ros2 launch capstone sitl.launch.py

# Terminal 2 — logger (start before flying)
source install/setup.bash
ros2 run capstone flight_logger --ros-args -p drone_name:=drone1

# Terminal 3 — controller
python3 capstone/capstone/controller.py

# After flight: Ctrl-C logger, then plot
python3 scripts/plot_flight_log.py ~/flight_logs/flight_*.csv
```

### Task 2 (Real hardware, on Pi)
```bash
# On Pi — run alongside full stack
ros2 run onboard apriltag_logger

# After flight: SCP to ground laptop then plot
scp hare@192.168.168.105:~/apriltag_logs/apriltag_*.csv .
python3 scripts/plot_apriltag_log.py apriltag_*.csv
```
