# TASKS.md — Work Queue

Ordered by dependency. Complete earlier tiers before later ones.

---

## DONE

- **Multi-drone MAVROS controller** (`capstone/capstone/controller.py`)
  ARM/WAYPOINT/HOVER/CIRCLE/LAND/RC_TAKEOVER state machine; battery abort;
  multi-drone namespace support; circle formation flight; RC takeover detection.
  Docs: `capstone/CONTROLLER_DOCUMENTATION.md`

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

---

## TIER 1 — Current Blockers (Hardware)

### [BLOCKED] CSI camera not detected on Pi

**Symptom:** `sudo dmesg | grep -iE 'unicam|imx|ov5647'` returns nothing.
The Pi kernel sees no camera sensor at all — not a software issue.

**Fix procedure:**
1. Power off Pi completely
2. Reseat CSI ribbon cable at both ends:
   - Metal contacts face **toward the PCB** (away from the black locking tab)
   - Black locking tab must be pressed fully flat — feels firmer than expected
   - Check for any crease or fold in the ribbon — replace cable if found
3. Power on, then verify:
   ```bash
   sudo dmesg | grep -iE 'unicam|imx|ov5647|csi'
   # Should show: unicam 3f801000.csi: ... and imx219 (or your sensor)
   ```
4. If still nothing, add explicit overlay to `/boot/firmware/config.txt`:
   ```bash
   # Pi Camera v2 (IMX219): echo "dtoverlay=imx219"
   # Pi Camera v3 (IMX708): echo "dtoverlay=imx708"
   echo "dtoverlay=imx219" | sudo tee -a /boot/firmware/config.txt
   sudo reboot
   ```

**Once hardware is confirmed working:**
```bash
sudo apt install -y libcamera-apps
libcamera-hello --list-cameras    # should show sensor name and modes
```

### [BLOCKED] Camera calibration — placeholder values in default_cam.yaml

`apriltag/config/default_cam.yaml` has valid structure but fake intrinsics
(fx=fy=600, zero distortion). AprilTag pose estimates will be wrong until
replaced with real calibration data.

**Procedure (after camera hardware is working):**
```bash
# On Pi — start camera
ros2 launch onboard onboard.launch.py

# On ground laptop — run calibration tool
# Use a printed checkerboard; adjust --size and --square to match yours
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  image:=/camera/image_raw camera:=/camera

# Move board until X/Y/Size/Skew bars are all green → click CALIBRATE → SAVE
# Copy camera_matrix and distortion_coefficients into:
#   apriltag/config/default_cam.yaml
```

Target reprojection error: < 0.5 px.

### [TODO] Measure camera mount transform

`onboard/config/onboard.yaml` `camera_tf` section has placeholder offsets.
Measure physical position of camera lens from drone centre of mass and update.

### [TODO] Verify tag_size matches printed tags

`onboard.yaml` has `tag_size: 0.166` (166 mm outer border).
Measure actual printed tags and update if different.

### [TODO] Confirm Pi apt dependencies installed

```bash
sudo apt install -y \
  ros-humble-camera-ros \
  ros-humble-apriltag-ros \
  ros-humble-mavros ros-humble-mavros-extras \
  ros-humble-tf2-ros python3-yaml
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

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

### [TODO] Precision landing node

Core unimplemented capability. `/apriltag/detections` is published but nothing
consumes it. Controller LAND state descends blindly.

High-level design:
- New node subscribes `/apriltag/detections`
- During LAND state: uses TF2 to transform tag pose to ENU frame
- Publishes corrected lateral `setpoint_position/local` to close offset to tag
- Controlled descent rate; hold position and warn if tag lost mid-descent
- Config: `landing_tag_id` per drone in `onboard.yaml`

**Depends on:** working camera + calibration + measured camera TF.
**Architectural decision:** standalone node vs integrated into controller.

### [TODO] Per-drone tag ID assignment

Each drone needs a unique landing pad tag ID.
Add `landing_tag_id` to `onboard.yaml` and filter detections in landing node.

### [TODO] Capstone launch file (ground side)

`capstone/launch/capstone.launch.py` to replace `python3 controller.py`.

---

## TIER 3 — Polish

- Landing detection via `mavros_msgs/ExtendedState` instead of altitude threshold
- RC_TAKEOVER exit: resume HOVER at current position instead of re-arming
- `cell_count` as ROS2 parameter (currently hardcoded as 6 in `main()`)
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
