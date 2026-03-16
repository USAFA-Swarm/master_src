# CLAUDE.md — Project Context and Working Rules

## Project Context

| Item | Detail |
|---|---|
| Project | USAFA Robot Teaming Capstone — autonomous multi-drone control |
| ROS2 distro | Humble (Ubuntu 22.04) |
| Flight controller | ArduPilot (ArduCopter), GUIDED mode |
| FC bridge | MAVROS (`ros-humble-mavros`) |
| Onboard computer | Raspberry Pi 4, Ubuntu 22.04.5 LTS |
| Ground computer | Ubuntu 22.04 laptop (`/home/dfec/master_src`) |
| Camera | CSI ribbon camera on Pi (model TBD — hardware issue pending) |
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

- **CSI ribbon cable hardware issue** — camera sensor not detected by Pi kernel
  (`sudo dmesg | grep unicam` returns nothing). Cable or connector fault.
  Camera subsystem (`camera_ros`, `apriltag_ros`) is on hold until resolved.
- **Next task once resolved:** verify `libcamera-hello --list-cameras` shows
  the sensor, then re-run `ros2 launch onboard onboard.launch.py` and confirm
  `/camera/image_raw` publishes. See TASKS.md Tier 1.

---

## Pi SSH Workflow

**Network:** Microhard radio → switch. Pi IP: `192.168.168.102`.
Ground Ubuntu: `192.168.168.103`. Ground Windows: `192.168.168.233`.

**NEW — single launch (replaces the three manual steps below):**
```bash
ssh pi@192.168.168.102
cd ~/master_src && source install/setup.bash
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
cd /home/dfec/master_src/capstone/capstone
python3 controller.py
# Enter drone names: drone1
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
