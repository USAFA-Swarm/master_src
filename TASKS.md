# TASKS.md — Work Queue

Ordered by dependency. Complete earlier tiers before later ones.

---

## DONE

- **Multi-drone MAVROS controller** (`capstone/capstone/controller.py`)
  ARM/WAYPOINT/HOVER/CIRCLE/LAND/RC_TAKEOVER state machine;
  multi-drone namespace support; circle formation flight; RC takeover detection.
  Docs: `capstone/CONTROLLER_DOCUMENTATION.md`

- **Battery abort removed from controller** — `_voltage_to_percentage`, `battery_pct`,
  `low_battery_warned`, and dead commented-out abort block removed. Battery monitoring
  delegated entirely to Mission Planner / ArduCopter failsafe. Controller still displays
  raw voltage at the command prompt for operator awareness.

- **Package metadata** — all `package.xml` files complete with correct deps

- **Bug fixes** — wrong package name in apriltag.py, indentation, empty YAML
  template, comms_interface and move2hover and drone_interface removed,
  runtime artifacts removed from git, .gitignore fixed for ROS2

- **Onboard Pi launch package** (`onboard/`)
  `onboard.launch.py` starts mavros + camera_ros + apriltag_ros + static TF.
  `onboard.yaml` is single config source — no hardcoded values in launch file.
  Replaces manual three-terminal SSH workflow.
  MAVROS `name=` conflict fixed (removed explicit node name).

- **apriltag_ros integration** (code complete, pending hardware)
  Subscribed to `/camera/image_raw` + `/camera/camera_info`.
  Publishes `/apriltag/detections` with TF2. Config in `onboard.yaml`.

- **Static TF publisher** `base_link → camera_optical_frame` in launch file.
  Values are placeholders — must be measured from physical mount.

- **Precision landing node** (`onboard/onboard/precision_landing.py`)
  Monitors `/apriltag/detections` at all times. On target tag confirmation, publishes
  `/<d>/precision_landing/takeover=True`; controller yields. Node centers laterally
  using `map → tag<id>` TF2 lookup, descends, hands off to ArduCopter LAND below
  `land_final_alt`. Config in `onboard.yaml precision_landing:` section.

- **Image rotate** (`image_rotate` node in onboard.launch.py)
  Corrects upside-down IMX296 mount. Publishes `/camera/image_rotated`.
  AprilTag now receives corrected image; poses are intuitive.

- **Frame_id fix** — camera_ros ignores `frame_id` param, publishes `frame_id: camera`.
  Static TF `child_frame` updated to `"camera"` to match. TF chain now complete:
  `base_link → camera → tag<id>`.

- **Camera-only launch file** (`onboard/launch/camera_only.launch.py`)
  Starts camera_ros only — no MAVROS, no AprilTag, no TF.
  Use for camera checkout and live feed testing without the full stack.

- **Ground-station live feed script** (`scripts/view_camera.sh`)
  SSH -X into Pi, runs `image_tools showimage` on Pi, forwards window to ground laptop.
  Bypasses Humble/Jazzy DDS incompatibility — no direct cross-machine ROS2 subscription.

- **`format: RGB888` added to onboard.yaml** camera section.
  Was missing — `onboard.launch.py` had never been passing the format parameter that
  the manual `ros2 run` command requires. Both launch files now pass it.

---

## TIER 1 — Current Blockers (Hardware)

### [DONE] CSI camera detected on Pi

Pi upgraded to Ubuntu 24.04 + ROS2 Jazzy. Camera sensor confirmed working:
- Sensor: **IMX296 global shutter** (`/base/soc/i2c0mux/i2c@1/imx296@1a`)
- Max resolution: 1456x1088. Working resolution: 1280x720
- `camera_ros` publishes `/camera/image_raw` successfully
- Known issue: `camera_ros` logs a framerate parameter error (non-fatal, still publishes)
- `rpicam-apps` / `libcamera-hello` not available on this setup — use `camera_ros` directly

### [DONE] Live video feed

Camera feed viewable on Pi monitor directly via `image_tools showimage`.
SSH -X forwarding also confirmed working (latency ~30s over radio — use Pi monitor instead).
`ros-jazzy-image-tools` installed on Pi.
`~/.bashrc` on Pi configured with all env vars — no manual exports needed in new terminals.

**Start camera + viewer (both in tmux on Pi):**
```bash
# Window 0 — camera
ros2 run camera_ros camera_node --ros-args -p camera:=0 -p format:=RGB888 -p width:=1280 -p height:=720

# Window 1 — viewer (Ctrl-B c for new tmux window)
ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw -p reliability:=best_effort
```

### [DONE] Camera calibration

Calibrated 2026-04-08. IMX296 at 1280×720, 8×5 interior corners, 24mm squares.
Real intrinsics written to:
- Pi: `~/.ros/camera_info/imx296__base_soc_i2c0mux_i2c_1_imx296_1a_1280x720.yaml`
- Repo: `apriltag/config/default_cam.yaml`

fx=1551.157, fy=1547.374, cx=663.415, cy=341.480
k1=-0.459, k2=0.235, p1=0.004, p2=0.001

**Note:** Camera is mounted Rotate180 (upside-down per libcamera warning).
Image feed is inverted. Fix physically or add a flip node before AprilTag detection.

Checkerboard on hand: **9×6 squares, 24 mm each** → `--size 8x5 --square 0.024`

**Procedure used (for reference / recalibration):**
```bash
# Terminal 1 on Pi — camera
ros2 run camera_ros camera_node --ros-args -p camera:=0 -p format:=RGB888 -p width:=1280 -p height:=720

# Terminal 2 on Pi — calibrator (GUI opens on Pi monitor)
ros2 run camera_calibration cameracalibrator \
  --size 8x5 --square 0.024 \
  --ros-args -r image:=/camera/image_raw -r camera_info:=/camera/camera_info

# Move board until X/Y/Size/Skew bars all green → CALIBRATE → SAVE
# NOTE: COMMIT button is broken (runtime error) — use SAVE then manual copy instead:
mkdir -p /tmp/cal && tar -xzf /tmp/calibrationdata.tar.gz -C /tmp/cal
mkdir -p ~/.ros/camera_info
cp /tmp/cal/ost.yaml \
  ~/.ros/camera_info/imx296__base_soc_i2c0mux_i2c_1_imx296_1a_1280x720.yaml
```

**Calibration file locations:**
- Pi runtime: `~/.ros/camera_info/imx296__base_soc_i2c0mux_i2c_1_imx296_1a_1280x720.yaml`
- Repo: `apriltag/config/default_cam.yaml` (real values committed 2026-04-08)

### [TODO] Measure camera mount transform

`onboard/config/onboard.yaml` `camera_tf` section has placeholder offsets.
Measure physical position of camera lens from drone centre of mass and update.

### [TODO] Verify tag_size matches printed tags

`onboard.yaml` has `tag_size: 0.166` (166 mm outer border).
Measure actual printed tags and update if different.

### [TODO] Build and deploy onboard package

After pulling latest code to Pi:
```bash
cd ~/master_src
colcon build --packages-select onboard
source install/setup.bash
```

Then install new apt dependency:
```bash
sudo apt install -y ros-jazzy-image-rotate
```

### [TODO] Confirm Pi apt dependencies installed

Pi runs **Ubuntu 24.04 / ROS2 Jazzy** — use `ros-jazzy-*`, not `ros-humble-*`.

```bash
sudo apt install -y \
  ros-jazzy-camera-ros \
  ros-jazzy-image-tools \
  ros-jazzy-apriltag-ros \
  ros-jazzy-mavros ros-jazzy-mavros-extras \
  ros-jazzy-tf2-ros python3-yaml
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

---

## TIER 1.5 — Circle Hard Landing Bug

### [TODO] Diagnose circle command hard landing

Symptom: `circle` command takes off correctly, begins moving toward first waypoint,
then initiates LAND shortly after. Waypoint commands work fine through the same
takeoff path, so the failure is specific to CIRCLE state execution.

**Suspected Python causes to investigate with live logs:**
- `circle_waypoints[drone]` being emptied or missing when CIRCLE handler runs
  → triggers fallback LAND at `controller.py:831`
- `_in_guided_mode` failing repeatedly during CIRCLE, leaving drone without setpoints

**ArduCopter parameter to check (Mission Planner Full Parameter List):**
- `BATT_FS_LOW_ACTION` — should not be set to LAND (1) if voltage threshold can trip
  during spin-up; recommend WARN (0) or RTL (2) and let MP operator decide
- `BATT_LOW_VOLT` — verify threshold is below minimum expected loaded voltage
- `BATT_CRT_VOLT` — same

**Test procedure:**
1. Run `circle drone1 5.0 10.0`
2. Capture full controller log (`python3 controller.py 2>&1 | tee circle_log.txt`)
3. Look for "Circle waypoints not available" or "Lost flight mode" messages
4. Cross-reference with Mission Planner battery voltage trace at moment of landing

---

## TIER 2 — Next Development (after Tier 1 resolved)

### [TODO] Verify full onboard launch works end-to-end

After camera hardware fix:
1. `ros2 launch onboard onboard.launch.py`
2. Check topics exist: `ros2 topic list | grep -E 'camera|apriltag|drone1'`
3. Check image rate: `ros2 topic hz /camera/image_raw`
4. View image on ground: `ros2 run rqt_image_view rqt_image_view`
   (requires matching `ROS_DOMAIN_ID` on both machines)
5. Hold tag in front of camera, check detections:
   `ros2 topic echo /apriltag/detections`

### [DONE] Precision landing node — see DONE section above

### [TODO] Per-drone tag ID assignment

Each drone needs a unique landing pad tag ID.
Add `landing_tag_id` to `onboard.yaml` and filter detections in landing node.

### [TODO] Capstone launch file (ground side)

`capstone/launch/capstone.launch.py` to replace `python3 controller.py`.

---

## TIER 3 — Polish

- Landing detection via `mavros_msgs/ExtendedState` instead of altitude threshold
- RC_TAKEOVER exit: resume HOVER at current position instead of re-arming
- Remove vestigial `cell_count` parameter from `Controller.__init__` (was only used by
  the now-deleted `_voltage_to_percentage`; currently harmless but dead weight)
- SITL launch file (`onboard/launch/sitl.launch.py` with UDP fcu_url)
- Integration smoke tests

---

## Removed (reference)

- `drone_interface/` — empty stub
- `move2hover/` — cmd_vel architecture, incompatible with MAVROS
- `comms_interface/` — passive logging stubs, never used operationally
- `apriltag/launch/vision.lauch.CHANGE_ME.py` — typo, incomplete
- `apriltag/config/CHANGE_ME.svm` — placeholder for nonexistent node
- `eeprom.bin`, `mav.tlog`, `mav.tlog.raw` — runtime artifacts
