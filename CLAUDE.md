# CLAUDE.md — Project Context and Working Rules

## Project Context

| Item | Detail |
|---|---|
| Project | USAFA Robot Teaming Capstone — autonomous multi-drone control |
| ROS2 distro | Pi: Jazzy (Ubuntu 24.04) / Ground: Humble (Ubuntu 22.04) |
| Flight controller | ArduPilot (ArduCopter), GUIDED mode |
| FC bridge | MAVROS (`ros-jazzy-mavros` on Pi) |
| Onboard computer | Raspberry Pi 4, Ubuntu 24.04 LTS (user: `hare`) |
| Ground computer | Ubuntu 22.04 laptop (`/home/dfec/master_src`) |
| Camera | IMX296 global shutter CSI camera — **WORKING** |
| Link | Microhard radio → switch → ground laptops (192.168.168.x subnet) |
| Repo | https://github.com/USAFA-Swarm/master_src |

**Packages in workspace:**

| Package | Where it runs | Purpose |
|---|---|---|
| `capstone` | Ground laptop | Multi-drone MAVROS controller (stdin commands) |
| `apriltag` | Pi (future) | Custom AprilTag detection node (pupil_apriltags) |
| `onboard` | Pi | Launch file + config for full Pi stack |

---

## Workflow — Follow This Every Task

### 1. RESEARCH
Read all source files, configs, and docs relevant to the task.
Write findings to `memory/research.md`. **Stop and wait for review.**

### 2. PLAN
Write a numbered implementation plan to `memory/plan.md`:
- Files to create or modify
- Exact changes per file
- Success criteria (how to verify it worked)

**Stop and wait for explicit approval before writing any code.**

### 3. IMPLEMENT
Work through the approved plan step by step.
Note completion of each numbered step as you go.

### 4. VALIDATE
Check each success criterion from the plan.
Run `colcon build --packages-select <pkg>` if applicable.

### 5. UPDATE DOCS
Update `TASKS.md` and `ARCHITECTURE.md` to reflect what changed.

---

## Rules

- Never push directly to main. Use a feature branch for any non-trivial change.
- No hardcoded IPs, paths, topic names, or credentials. Parameterize everything.
- All tunable values in `onboard/config/onboard.yaml`, not in launch files or source.
- After two failed correction attempts on the same issue, stop and ask.
- Keep this file under 150 lines. Move long specs to `memory/` files.

---

## Current Blockers

- **Humble/Jazzy DDS incompatibility** — ground laptop (Humble) subscribing to
  Pi (Jazzy) topics causes DDS deserialization errors that crash `camera_ros`.
  **Workaround:** keep `ROS_LOCALHOST_ONLY=1` on Pi always for camera ops.
  View feed via SSH -X (`scripts/view_camera.sh`) — do NOT subscribe from ground.
  Note: `ROS_LOCALHOST_ONLY` is deprecated in Jazzy but still honored.
- **Camera mounted upside-down** — libcamera reports Rotate180 on IMX296.
  Raw image feed is inverted. Remount camera physically, or add a flip node
  before AprilTag detection. Does not affect calibration validity.
- **Circle command hard landing** — drone takes off, begins moving toward first
  circle waypoint, then lands unexpectedly. Waypoint commands work fine.
  See TASKS.md Tier 1.5 for diagnosis procedure.
- **Next session start:** SSH to Pi (IP may need checking — set static if not done),
  run `tmux attach -t camera` or start fresh. See Pi SSH Workflow below.

---

## Pi SSH Workflow

**Network:** Microhard radio → switch. Pi IP: `192.168.168.102`.
Ground Ubuntu: `192.168.168.103`. Ground Windows: `192.168.168.233`.

**SSH (passwordless — run `ssh-keygen` then `ssh-copy-id hare@192.168.168.102` once):**
```bash
ssh hare@192.168.168.102
```

**Pi env vars (already added to `~/.bashrc` — auto-set in every new terminal):**
```bash
export ROS_DOMAIN_ID=13
export ROS_LOCALHOST_ONLY=1
source /opt/ros/jazzy/setup.bash
source ~/master_src/install/setup.bash
```

**Persistent sessions — use tmux so SSH disconnects don't kill nodes:**
```bash
tmux new -s camera        # new session
tmux attach -t camera     # reattach after disconnect
# Ctrl-B c  = new window | Ctrl-B 0/1/2 = switch window
```

**Camera only (live feed / testing):**
```bash
ros2 run camera_ros camera_node --ros-args -p camera:=0 -p format:=RGB888 -p width:=1280 -p height:=720
# or via launch file (after colcon build on Pi):
ros2 launch onboard camera_only.launch.py
```

**View feed on Pi monitor (open in second tmux window or Pi terminal):**
```bash
ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw -p reliability:=best_effort
```

**Full onboard launch:**
```bash
ros2 launch onboard onboard.launch.py
```

**OLD manual sequence (reference only — superseded by launch file):**
```bash
# Terminal 1 on Pi — MAVProxy (bridge for QGC/Mission Planner)
mavproxy.py --master=/dev/ttyACM0 --baudrate 921600 \
  --out=udp:192.168.168.233:14550 \
  --out=udp:192.168.168.103:14551

# Terminal 2 on Pi — MAVROS
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/drone1 -p fcu_url:=serial:///dev/ttyACM0:921600
```

**Ground laptop — controller:**
```bash
cd /home/dfec/master_src
source install/setup.bash
python3 capstone/capstone/controller.py
# Enter drone names: drone1
```

**Ground laptop — view camera feed (SSH -X to Pi):**
```bash
./scripts/view_camera.sh
# Opens showimage window forwarded from Pi — do NOT subscribe to /camera topics directly
```

**QGroundControl (Windows):** UDP, port 14550.
**Mission Planner (Windows):** UDP, local port 14550.

**Note:** If MAVROS won't connect, `ttyACM0` may have changed.
Check with `ls /dev/ttyACM*` on the Pi and update `onboard.yaml`.

---

## Key Conventions

- Drone namespace: `/<drone_name>/` — must match what you type into controller.py
- MAVROS QoS: BEST_EFFORT / VOLATILE / KEEP_LAST on all subscriptions
- `ros2 topic echo` needs `--qos-reliability best_effort` for MAVROS topics
- Local position frame: ENU (X=East, Y=North, Z=Up), origin at arm point
- Camera frame: optical convention (X=right, Y=down, Z=into scene)
- All params live in `onboard/config/onboard.yaml`
- See `ARCHITECTURE.md` for full topic/frame reference
- See `TASKS.md` for prioritized work queue
- See `capstone/CONTROLLER_DOCUMENTATION.md` for controller command reference
