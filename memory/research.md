---
name: Precision landing architecture research
description: Compute split (Pi vs ground), network failure modes, and design decisions for precision landing node
type: project
---

# Precision Landing — Architecture Research

## 1. What is Currently Running Where

### Pi (192.168.168.102 — Jazzy)
| Node | Package | Topics |
|---|---|---|
| `mavros_node` | `mavros` | Pub: `/<d>/state`, `local_position/pose`, `battery`, `rc/in`; Svc: arming, set_mode, takeoff |
| `camera_node` | `camera_ros` | Pub: `/camera/image_raw`, `/camera/camera_info` |
| `apriltag_node` | `apriltag_ros` | Sub: `/camera/image_raw`, `/camera/camera_info`; Pub: `/apriltag/detections`; TF2: `camera_optical_frame → tag_<id>` |
| `camera_tf_static` | `tf2_ros` | TF2: `base_link → camera_optical_frame` (static, placeholder values) |

### Ground (192.168.168.103 — Humble)
| Node | Package | Notes |
|---|---|---|
| `controller` | `capstone` | All state machine logic, all setpoints, all arming/mode commands; user stdin interface |

### Currently NOT running anywhere
- Precision landing node (the gap we're filling)
- `/apriltag/detections` is published by `apriltag_node` on Pi but nothing subscribes

---

## 2. What MUST Run Onboard (Pi)

These components have hard latency or physical connection requirements:

| Component | Reason it must be onboard |
|---|---|
| `mavros_node` | Serial to FCU (ttyACM0) — cannot be bridged to ground without MAVProxy; any latency here causes GUIDED mode to exit setpoint hold |
| `camera_node` | CSI ribbon cable to IMX296 — physical, no alternative |
| `apriltag_ros` | Image processing at 720p: ~500 KB/frame raw. Streaming raw images over Microhard at 30fps is impractical (~120 Mbps uncompressed). Detection on Pi emits only tiny pose messages. Also: ROS_LOCALHOST_ONLY=1 means camera topics never leave Pi anyway |
| `camera_tf_static` | TF2 lookup must be in same context as apriltag_ros TF broadcasts |
| **Precision landing node (proposed)** | Must close the control loop locally: detection → lateral correction setpoint. If this runs on ground, every correction cycle adds one full round-trip over Microhard (~10–50ms nominal, potentially much worse). At low altitude, drift during a missed cycle matters. Also: if comms drop mid-descent, a Pi-local node can hold last known setpoint or abort; a ground node just vanishes. |

**Summary: everything camera-related and the precision landing correction loop MUST be on Pi.**

---

## 3. What CAN Run on the Ground Station

| Component | Notes |
|---|---|
| `controller.py` state machine | Mission-level commands (ARM, WAYPOINT, CIRCLE, LAND). Latency is tolerable because these are coarse maneuvers, not sub-second corrections |
| Operator interface (stdin) | Must be on ground by design |
| Battery/state monitoring display | Cosmetic — already on ground |
| `rqt_image_view` or `showimage` for monitoring | Via SSH -X from Pi (due to DDS incompatibility — do NOT subscribe from ground) |
| Logging / rosbag | Can record from ground for MAVROS topics; camera/detection logs must be taken from Pi |

**Note on `/apriltag/detections` cross-machine:** This is an `apriltag_msgs/AprilTagDetectionArray` message — a small pose message, NOT a large image. The Humble/Jazzy DDS crash is specific to large messages (Image). Small messages like detection arrays *may* cross versions without crashing, but this has NOT been tested on this hardware. To be safe, keep detection consumers on Pi.

---

## 4. Network Failure Modes

### If Microhard link drops during precision landing

| Component | What happens | Safe? |
|---|---|---|
| `controller.py` on ground | Loses MAVROS telemetry; cannot send new state commands | Depends on what's onboard |
| MAVROS setpoint stream | If ground was streaming setpoints → stream stops → ArduCopter exits GUIDED mode after ~1s, triggers RTL or failsafe | **BAD if ground holds the landing loop** |
| Pi-local precision landing node | Continues running; can continue sending corrective setpoints until tag lost or touchdown | **SAFE — no dependency on ground** |
| ArduCopter GCS failsafe | `FS_GCS_ENABLE` — if enabled, loss of GCS heartbeat triggers RTL after timeout. This fires regardless of where landing node runs. Should be set to a reasonable timeout (e.g., 5s) to allow brief drops. | Operator should configure in Mission Planner |

**Conclusion:** If the precision landing correction loop runs on the Pi, a comms drop during landing is survivable — the drone continues its descent with Pi-local tag tracking. If it runs on the ground, a comms drop immediately stops all corrections and the drone drifts.

### If Pi crashes or apriltag_node dies during landing

Either way (onboard or ground node), detections stop. Both cases need identical tag-loss fallback:
- Hold current horizontal position, stop descent
- Warn operator
- If tag not reacquired within N seconds → abort to HOVER at current altitude

This fallback is implemented the same way regardless of where the node runs.

---

## 5. Proposed Architecture

### New node: `precision_landing` — runs on Pi

**Location:** `onboard/onboard/precision_landing.py` (new file in onboard package)

**State machine (within the node, active only when `controller.py` sends LAND command):**

```
IDLE
 └─ on LAND command from controller → SEARCH
SEARCH  (drone descending slowly or hovering, scanning for tag)
 └─ tag detected for N consecutive frames → DESCEND
DESCEND  (tag visible, actively correcting horizontal, descending)
 └─ tag lost → CENTER (hold horizontal, stop descent, wait)
 └─ altitude < land_final_alt → LAND (let ArduCopter handle touchdown)
CENTER  (centering on last known position, waiting for tag reacquire)
 └─ tag reacquired → DESCEND
 └─ timeout → HOVER_HOLD (abort descent, warn operator)
LAND    (final few cm — release to ArduCopter LAND mode, node becomes passive)
```

**Activation:** The node needs a signal from `controller.py` that LAND state is active.
Options:
1. Subscribe to `/drone1/state` — check mode is GUIDED and controller is in LAND (can't tell mode from state alone)
2. Separate activation topic: `controller.py` publishes `/<d>/precision_landing/activate` (std_msgs/Bool) when LAND state is entered
3. Node monitors altitude continuously and activates below a threshold

**Recommendation:** Option 2 — explicit activation topic. Clean separation, no altitude guessing.

**Corrective setpoints:** Use `/<d>/setpoint_position/local` (PoseStamped, ENU).
TF2 lookup: `base_link → tag_<id>` using full chain through `camera_optical_frame`.
Lateral correction = negative of tag's x,y offset in ENU, scaled by a gain.
Descent rate = fixed step per cycle while tag visible, zero while centering.

**Config in `onboard.yaml` (new `precision_landing` section):**
```yaml
precision_landing:
  landing_tag_id: 0           # tag ID to land on
  confirm_frames: 5           # consecutive detections before DESCEND
  tag_loss_timeout: 3.0       # seconds before abort
  descent_step: 0.05          # metres per cycle to descend
  land_final_alt: 0.3         # metres AGL — hand off to ArduCopter LAND below this
  lateral_gain: 0.5           # scale factor on lateral correction
  loop_rate: 10.0             # Hz
```

---

## 6. Key Open Questions Before Implementation

1. **camera_tf values are placeholders** — lateral corrections will be wrong until real offsets are measured. Can test logic with zeros (camera approximately at CoM) but real values needed for actual precision landing.

2. **Camera upside-down (Rotate180)** — apriltag_ros may handle this transparently through the TF chain, or tag poses may be inverted. Test needed.

3. **`frame_id` in camera_info** — currently publishing as `"camera"` not `"camera_optical_frame"`. TF2 lookup will fail if frame IDs don't match. Must verify or fix.

4. **Activation mechanism** — need to add `/<d>/precision_landing/activate` publisher to `controller.py` (ground side) OR choose threshold-based activation.

5. **DDS cross-machine for activation topic** — if activation comes from ground controller, it crosses the Microhard link. The topic is a small Bool message — should be safe across Humble/Jazzy. The response (setpoints) goes Pi → MAVROS locally, never crosses to ground.

---

## 7. Files to Create or Modify

| File | Action |
|---|---|
| `onboard/onboard/precision_landing.py` | CREATE — new node |
| `onboard/config/onboard.yaml` | MODIFY — add `precision_landing:` section |
| `onboard/launch/onboard.launch.py` | MODIFY — add precision_landing node |
| `capstone/capstone/controller.py` | MODIFY — publish activation Bool on LAND state entry/exit |
| `onboard/package.xml` | VERIFY — check deps include tf2_ros, geometry_msgs, apriltag_msgs, std_msgs |
| `ARCHITECTURE.md` | UPDATE — add precision_landing node to diagram |
| `TASKS.md` | UPDATE — mark items done / add new items |
