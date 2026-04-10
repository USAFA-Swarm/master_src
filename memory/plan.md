---
name: Precision landing implementation plan
description: Numbered plan for image flip, precision landing node, controller takeover protocol
type: project
---

# Plan: Precision Landing with Autonomous Takeover

## Design Decisions (approved)

- **Image flip via `image_rotate` package** — no new Python file. Add node to launch file,
  `rotation_angle: π`. Apriltag gets corrected image; camera_tf stays roll=π, pitch=0, yaw=0.
- **Frame_id fix** — camera_ros ignores `frame_id` param, always publishes `frame_id: camera`.
  Fix: change `camera_tf.child_frame` to `"camera"` in onboard.yaml (one line).
- **Camera TF values** — x=0, y=0, z=-0.05 confirmed correct. Roll=π already in yaml. No TF math changes.
- **Precision landing triggers on tag detection at ANY time during flight.**
  Node publishes `/<d>/precision_landing/takeover` Bool. Controller subscribes; yields when True,
  resumes HOVER when False.
- **All precision landing logic on Pi** — closed loop, survives comms drop.

---

## Files To Create or Modify

| # | File | Action |
|---|---|---|
| 1 | `onboard/config/onboard.yaml` | MODIFY — fix child_frame, add precision_landing section |
| 2 | `onboard/launch/onboard.launch.py` | MODIFY — add image_rotate node; remap apriltag to rotated image; add precision_landing node |
| 3 | `onboard/package.xml` | MODIFY — add image_rotate, rclpy, std_msgs, geometry_msgs, tf2_geometry_msgs, apriltag_msgs, mavros_msgs |
| 4 | `onboard/setup.py` | MODIFY — add precision_landing to console_scripts |
| 5 | `onboard/onboard/precision_landing.py` | CREATE — landing node |
| 6 | `capstone/capstone/controller.py` | MODIFY — takeover subscription + PRECISION_LANDING state |
| 7 | `ARCHITECTURE.md` | UPDATE |
| 8 | `TASKS.md` | UPDATE |

Pi apt install needed (once): `sudo apt install -y ros-jazzy-image-rotate`

---

## Step-by-Step Implementation

### Step 1 — `onboard/config/onboard.yaml`

**Change A:** Fix frame_id mismatch. In `camera_tf` section:
```yaml
child_frame: "camera"   # was "camera_optical_frame" — camera_ros ignores frame_id param
                         # and always publishes frame_id: camera (node name)
```

Also update `camera.frame_id` for consistency (though camera_ros ignores it):
```yaml
frame_id: "camera"   # was "camera_optical_frame"
```

**Change B:** Add `precision_landing` section at end of file:
```yaml
# ---------------------------------------------------------------------------
# Precision landing node
# ---------------------------------------------------------------------------
precision_landing:
  drone_name: "drone1"        # must match mavros.drone_name
  landing_tag_id: 0           # tag ID to land on
  confirm_frames: 5           # consecutive detections required before takeover
  tag_loss_timeout: 3.0       # seconds without detection before holding position
  descent_step: 0.05          # metres descent per control cycle
  land_final_alt: 0.4         # metres AGL — hand off to ArduCopter LAND below this
  lateral_gain: 0.6           # lateral correction scale (0.0–1.0)
  loop_rate: 10.0             # Hz
```

---

### Step 2 — `onboard/launch/onboard.launch.py`

**Change A:** Load `pl` config section at top of `generate_launch_description()`:
```python
pl = cfg['precision_landing']
```

**Change B:** Add `image_rotate` node after camera_node, before apriltag_node:
```python
Node(
    package='image_rotate',
    executable='image_rotate',
    name='image_rotate',
    output='screen',
    parameters=[{'rotation_angle': 3.14159265}],
    remappings=[
        ('image',         '/camera/image_raw'),
        ('rotated/image', '/camera/image_rotated'),
    ],
),
```

**Change C:** Change apriltag image remapping from `/camera/image_raw` to `/camera/image_rotated`:
```python
('image_rect', '/camera/image_rotated'),   # was /camera/image_raw
```

**Change D:** Add `precision_landing` node at end of LaunchDescription:
```python
Node(
    package='onboard',
    executable='precision_landing',
    name='precision_landing',
    output='screen',
    parameters=[config_path],
),
```

---

### Step 3 — `onboard/package.xml`

Add dependencies for new node and image_rotate:
```xml
<exec_depend>image_rotate</exec_depend>
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>tf2_ros</exec_depend>
<exec_depend>tf2_geometry_msgs</exec_depend>
<exec_depend>apriltag_msgs</exec_depend>
<exec_depend>mavros_msgs</exec_depend>
```

---

### Step 4 — `onboard/setup.py`

Add to `console_scripts`:
```python
'precision_landing = onboard.precision_landing:main',
```

---

### Step 5 — `onboard/onboard/precision_landing.py`

New node. State machine: IDLE → CENTERING → HANDOFF → IDLE.

**Subscriptions:**
- `/apriltag/detections` — always active, triggers takeover
- `/<drone>/local_position/pose` — altitude tracking
- `/<drone>/precision_landing/abort` (Bool) — operator abort from ground

**Publications:**
- `/<drone>/precision_landing/takeover` (Bool) — True=we have control, False=released
- `/<drone>/setpoint_position/local` (PoseStamped) — corrective setpoints

**TF2:** `buffer.lookup_transform('base_link', 'tag_<id>', rclpy.time.Time())`

**State logic:**

```
IDLE
  On every detection message:
    Filter for landing_tag_id.
    If found: increment confirm_counter.
    If confirm_counter >= confirm_frames:
      Publish takeover=True.
      Store current pose as hold_pose.
      → CENTERING

CENTERING (runs at loop_rate Hz in timer callback)
  TF lookup: base_link → tag_<landing_tag_id>
  If TF succeeds:
    Reset tag_loss_timer.
    lateral_error = (tag.x, tag.y) in base_link frame
    new_setpoint.x = current_pose.x - lateral_error.x * lateral_gain
    new_setpoint.y = current_pose.y - lateral_error.y * lateral_gain
    new_setpoint.z = current_pose.z - descent_step   (descend one step)
    Clamp new_setpoint.z >= land_final_alt (don't go below handoff altitude)
    Publish setpoint.
    hold_pose = new_setpoint (update hold in case tag lost next cycle)
  If TF fails (tag not visible):
    Increment tag_loss_timer.
    If tag_loss_timer > tag_loss_timeout:
      Publish setpoint = hold_pose (hold position, stop descending)
      Log WARN "Tag lost — holding position"
    Else:
      Continue publishing last setpoint (hold)
  If current_pose.z <= land_final_alt:
    → HANDOFF
  If abort received:
    Publish takeover=False.
    → IDLE

HANDOFF
  Call set_mode service: LAND
  Publish takeover=False immediately (controller resumes, drone is in LAND mode)
  → IDLE
```

**Note on abort:** The abort topic `/<drone>/precision_landing/abort` can be published from the
ground by adding a controller command `abort_landing` — or simply by the operator switching
to STABILIZE mode (which triggers RC_TAKEOVER in controller already).

---

### Step 6 — `capstone/capstone/controller.py`

**Change A:** Add `PRECISION_LANDING` to `DroneState` enum:
```python
PRECISION_LANDING = 'precision_landing'
```

**Change B:** Import `Bool` from `std_msgs.msg` (add to existing imports).

**Change C:** In `_setup_drone`, add takeover subscription:
```python
self.create_subscription(
    Bool,
    f'/{drone}/precision_landing/takeover',
    lambda msg, d=drone: self._takeover_callback(msg, d),
    qos_profile
)
```

**Change D:** Add `_takeover_callback`:
```python
def _takeover_callback(self, msg, drone):
    s = self.state[drone]
    if msg.data:
        prev = s['current_state']
        if prev not in (DroneState.ARM, DroneState.PRECISION_LANDING):
            self.get_logger().warn(
                f'[{drone}] Precision landing takeover — was {prev.value}')
            s['current_state'] = DroneState.PRECISION_LANDING
    else:
        if s['current_state'] == DroneState.PRECISION_LANDING:
            self.get_logger().info(f'[{drone}] Precision landing released — HOVER')
            s['current_state'] = DroneState.HOVER
```

**Change E:** In state machine loop, add handler (do nothing — Pi owns setpoints):
```python
elif current_state == DroneState.PRECISION_LANDING:
    pass  # Pi precision_landing node has control
```

**Change F:** In `display_command_options`, add display string for new state:
```python
DroneState.PRECISION_LANDING: 'PRECISION LANDING (Pi has control)',
```

---

### Step 7 — ARCHITECTURE.md

Update:
- Camera subsystem: add `image_rotate` node between camera_node and apriltag_node
- Add `precision_landing` node box consuming `/apriltag/detections`, publishing setpoints + takeover topic
- Topic table: add `/<d>/precision_landing/takeover` and `/<d>/precision_landing/abort`
- State machine diagram: add PRECISION_LANDING state, transition from any state on takeover
- Remove "[PENDING]" markers from camera/apriltag (they're working)
- Fix TF tree: `base_link → camera → tag_<id>`

---

### Step 8 — TASKS.md

Move to DONE: precision landing node, camera orientation fix, frame_id fix.
Add to TIER 1: `colcon build --packages-select onboard capstone` on Pi and ground.
Add to TIER 3: Add `abort_landing` command to controller for manual abort of precision landing.

---

## Success Criteria

1. `ros2 topic echo /camera/image_rotated` — image appears right-side up
2. `ros2 topic echo /camera/camera_info --once | grep frame_id` → `frame_id: camera`
3. `ros2 topic echo /tf_static | grep child` → one entry shows `child_frame_id: camera`
4. `ros2 topic echo /apriltag/detections` shows detections when tag in view
5. Hold tag under camera during HOVER → `/<d>/precision_landing/takeover` publishes True
6. Controller log shows "Precision landing takeover"
7. Drone corrects laterally toward tag center, descends
8. At `land_final_alt`, flight mode switches to LAND
9. After landing, takeover publishes False
10. `colcon build --packages-select onboard capstone` succeeds

---

## Known Limitations at Implementation Time

- camera_tf x/y/z are now confirmed correct (0, 0, -0.05). Roll=π confirmed correct for
  downward-facing camera + 180° image rotation. Yaw=0 assumes image right = drone nose —
  if corrections go in the wrong lateral direction, add yaw=3.14159 to onboard.yaml camera_tf.
- tag_loss abort only holds position; does not automatically return to original mission.
  Operator must type `hover` or `land` after a failed precision landing attempt.
