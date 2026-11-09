# Phase 1 — Status & Reference

Companion to [`plan.md`](plan.md), [`hardware.md`](hardware.md),
[`architecture.md`](architecture.md), and [`phase0-status.md`](phase0-status.md).
This is the practical "what we plan to do for Phase 1, what we actually
did, and what we learned" document.

**Mapping does not run `cat_patrol_robot`.** That package is the patrol app
(Phase 4+). Phase 1 uses only `yahboomcar_bringup`, `sllidar_ros2`, and
`slam_toolbox`. Saved maps go under `cat_patrol_robot/maps/` as project
artifacts.

Phase 1 = build a 2D map of one room with `slam_toolbox`, save it to
disk, and write a single-command launch file that reproduces the SLAM
session. **Scope deliberately small** — one room only. Multi-room
mapping is deferred to Phase 7.

When this doc was written, Phase 1 had not yet started; section 11
now records Session 1 (in progress). Use this doc as the canonical
command reference — especially `/scan_filtered`, the lighter SLAM
params, and host-side RViz.

> **⚑ STATUS 2026-06-26 — Phase 1 mapping WORKS.** Two things changed since the
> early sessions in §11, both documented in the new **§13–§15**:
> 1. **Root cause found & fixed** for the long-running map "collapse/bowtie":
>    the **RPLiDAR A1 was mounted ~180° rotated vs the URDF**, so the scan filter
>    was keeping the robot's *rear* (antennas/wires) and discarding the clean
>    front. Fixed via a 180° yaw in the URDF + a rewritten scan filter (§13a).
> 2. **Architecture moved:** `slam_toolbox` **and** RViz now run on the **host PC
>    inside a Humble Docker container** — not on the Jetson. The Jetson-side
>    `slam_toolbox` steps in §4/§10 are **superseded by §13e–§13h**. The Jetson
>    now runs sensors + odom + TF only.
>
> New readers: read **§13 (architecture & connectivity)** and **§15 (lessons
> learned)** first, then use §4/§10 only for the Jetson sensor/odom stack.

---

## 1. Purpose and scope

**Goal.** Map ONE room with `slam_toolbox` (online async mode), save the
resulting occupancy grid as a `.pgm` + `.yaml` pair, and capture the
session as a reproducible `slam.launch.py` so future sessions are a
single command.

**Why this phase exists.** SLAM is the gateway phase. Once we have a
trustworthy map, every later phase becomes possible:
- Phase 2 (AMCL) needs a saved map to localize against.
- Phase 3 (Nav2) needs a map for the global costmap.
- Phase 4 (patrol manager) needs a map to record waypoints in.
- Phase 6 (cat detection in the map frame) needs map-frame geometry.

**Scope.** One room. The user has chosen to focus on a single space
rather than the whole flat. This avoids the full-flat complications:
travelling through narrow doorways while building a single map, mecanum
behaviour transitioning between carpet and smooth floor (Phase 0 finding
in [`phase0-status.md`](phase0-status.md) section 4c #19), and the time
it takes to drive a large area at safe SLAM speeds.

**Cross-references.**
- [`plan.md`](plan.md) — Phase 1 section, the original arc.
- [`hardware.md`](hardware.md) section 7 — RPLiDAR A1 specs and topics.
- [`architecture.md`](architecture.md) — where SLAM sits in the data flow
  (the dashed-border block in the diagram, currently inactive).
- [`phase0-status.md`](phase0-status.md) — Phase 0 findings, especially
  the surface-dependence of mecanum rotation (section 4c lessons #19,
  #20) and the calibration record (section 5).

---

## 2. Acceptance criteria (the "done when…" list)

Phase 1 is done when ALL of these are true:

- [ ] A clean `.pgm` + `.yaml` map of the chosen room, committed under
      `cat_patrol_robot/maps/<room_name>.pgm` and `<room_name>.yaml`.
- [ ] A `slam.launch.py` in `cat_patrol_robot/launch/` that brings up
      bringup + lidar + `slam_toolbox` + RViz in one command, accepting
      a `room:=<name>` arg if multiple rooms get mapped later.
- [ ] The map covers the chosen room without major holes, smearing, or
      duplicated walls. "No major holes" = if I look at the saved
      `.pgm`, I can identify the room shape unambiguously.
- [ ] Loop closure has fired at least once during a mapping session
      (visible in the `slam_toolbox` console output as a log line and
      in RViz as a visible "snap" of the map).
      **Note:** our Jetson-8GB params file sets `do_loop_closing: false`
      to avoid OOM kills — loop closure is deferred until we have headroom
      or a beefier machine. A clean perimeter map without loop closure
      is acceptable for Phase 1 on this hardware.
- [ ] All sessions documented in section 11 below.

Stretch (nice-to-have, not required):
- A floor-type annotation overlay on the map (carpet vs smooth-floor
  area), in preparation for Phase 4 surface-aware patrol planning.
- Map serialization (`.posegraph` files) saved alongside the `.pgm` so
  future sessions can resume the SLAM graph rather than start fresh.

---

## 3. Pre-flight checklist

Two sub-lists: things you do once, and things you do at the start of
every mapping session.

### 3a. One-time setup

- Confirm `slam_toolbox` is installed:
      `ros2 pkg prefix slam_toolbox`
  → expect `/opt/ros/humble`. Already verified during planning.
- Confirm `nav2_map_server` is installed:
      `ros2 pkg prefix nav2_map_server`
  → expect `/opt/ros/humble`. Already verified.
- Pick the room. Note approximate dimensions on paper so you can later
  sanity-check the saved map's extent.
- Clear trip hazards along the routes you'll drive. The robot will be
  going slowly, but it can still get hung up on cables or rugs.
- Decide on a tape mark for the starting position. Put a piece of tape
  on the floor at the front-center of where the robot will sit at
  session start. This makes "did I return home?" answerable.
- Create the maps directory:
      `mkdir -p ~/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/maps`
- Build `yahboomcar_bringup` after pulling mapping updates:
      `cd ~/yahboomcar_ros2_ws/yahboomcar_ws && colcon build --packages-select yahboomcar_bringup --symlink-install`
- **RViz runs on the host PC (Noble + ROS Jazzy)**, not on the Jetson.
  Install `ros-jazzy-rviz2` on the host. Save your working RViz layout
  there (Displays: Map `/map` with **Transient Local** durability,
  LaserScan `/scan_filtered`, Fixed Frame `map`, Views Target Frame
  `base_footprint`).
- SLAM params for this robot live at
  [`yahboomcar_bringup/param/mapper_params_online_async.yaml`](../yahboomcar_bringup/param/mapper_params_online_async.yaml)
  (lighter than upstream defaults — required on 8 GB Jetson).

### 3b. Every-session checklist

- Battery > ~11.5 V:
      `ros2 topic echo /voltage --once`
  Below ~11.0 V the chassis will start to misbehave under load; near
  empty the robot moves very slowly — charge before mapping.
- Trim active in bringup launch args:
      `trim_vy_per_vx:=0.012`
  (The Phase 0 calibration value. Without it the chassis will curve
  right and SLAM will work harder.)
- Robot at the tape-mark start.
- LiDAR motor spinning. If it was paused with `/stop_motor`:
      `ros2 service call /start_motor std_srvs/srv/Empty`
- All terminals freshly sourced if you've rebuilt anything since the
  last session:
      `source /opt/ros/humble/setup.bash`
      `source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash`
- A clear path from the start position to the perimeter of the room.

---

## 4. Mapping session procedure (step-by-step recipe)

### Tomorrow — start here (exact order)

**Packages used:** `yahboomcar_bringup` + `sllidar_ros2` + `slam_toolbox` only.
**Not used:** `cat_patrol_robot` (patrol app — only the `maps/` folder for saving).

#### A. Before any terminals (5 min)

On **Jetson**:

```bash
# 1) Clean memory floor — reboot if swap was used in a prior OOM crash
free -h
# If "Swap" used > 512 MiB → reboot the Jetson first

# 2) Optional but recommended — frees ~100+ MB
sudo systemctl stop ollama

# 3) Find chassis USB port (CH340 7523 — NOT the LiDAR port)
for d in /dev/ttyUSB*; do
  pid=$(udevadm info -q property -n "$d" 2>/dev/null | grep '^ID_MODEL_ID=' | cut -d= -f2)
  [ "$pid" = "7523" ] && echo "CHASSIS=$d"
done
# Example output: CHASSIS=/dev/ttyUSB0  ← use this in T1 below

# 4) Confirm LiDAR by-id symlink (should NOT be the same as CHASSIS)
readlink -f /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
# Example: /dev/ttyUSB1
```

On **host** (peter-pen): no need to start anything yet.

#### B. Jetson — four terminals, in order

**Every Jetson terminal — paste first:**

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
```

**T1 — bringup** (replace `/dev/ttyUSB0` if step A found a different `CHASSIS=`):

```bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py \
    chassis_serial_port:=/dev/ttyUSB0 \
    use_joystick:=true \
    trim_vy_per_vx:=0.012
```

Wait for: `cmd_vel trim: trim_vy_per_vx=0.0120`

Quick check (same or new Jetson terminal, sourced):

```bash
ros2 node list | grep driver_node
ros2 topic echo /voltage --once
```

**T2 — LiDAR** (only after T1 is healthy):

```bash
ros2 launch sllidar_ros2 sllidar_launch.py \
    serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
    serial_baudrate:=115200 \
    frame_id:=laser_link
```

Wait for: `SLLidar health status : OK`

If you see `cannot retrieve SLLidar health code: 80008002` → wrong port conflict.
Ctrl+C T1 and T2; re-run step A; chassis and lidar must be **different** `/dev/ttyUSB*`.

**T2b — scan filter** (wait ~5 s after T2 OK):

```bash
ros2 run yahboomcar_bringup scan_front_filter
```

Wait for: `Keeping scan angles [-90, 90] deg -> /scan_filtered`

**T3 — SLAM** (wait ~5 s after T2b):

```bash
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=false \
    slam_params_file:=/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_bringup/param/mapper_params_online_async.yaml
```

Wait **30 seconds**. Ignore early `Message Filter dropping message` lines.
Success: `Registering sensor: [Custom Described Lidar]` and **no** `exit code -9`.

Before RViz, on Jetson:

```bash
free -h
# need ≥ 2 GiB in "available" column
```

#### C. Host — RViz last (terminal 4)

**Only after T3 has run 30 s and `free -h` looks OK.**

```bash
export ROS_DOMAIN_ID=28
export ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash
ros2 run rviz2 rviz2 -d /home/bots/all/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_rviz/rviz/mapping.rviz
```

Use **your saved host config** if you exported one instead of `mapping.rviz`.

**RViz displays (minimum — saves Jetson RAM):**

| Display | Setting |
|---|---|
| Fixed Frame | `map` |
| Views → Target Frame | `base_footprint` |
| Map | topic `/map`, Durability **Transient Local** |
| LaserScan | topic `/scan_filtered` |
| TF | **disable** (reduces OOM risk over WiFi) |

#### D. Drive and save

- Joystick on **Jetson** USB; Start = buzzer only; sticks drive at ~0.15 m/s max.
- Drive slowly along walls; arcing turns on smooth floor (no spin-in-place).
- When map looks good:

```bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/maps
ros2 run nav2_map_server map_saver_cli -f my_room
```

#### E. If anything dies with `exit -9` or `Bad alloc`

1. Ctrl+C **all** Jetson terminals + close host RViz.
2. Wait 10 s (or reboot if swap is high).
3. Start again from **A**.

#### F. Optional — one Jetson terminal instead of T1–T3

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
sudo systemctl stop ollama   # optional
ros2 launch yahboomcar_bringup slam_mapping_X3_launch.py \
    chassis_serial_port:=/dev/ttyUSB0 \
    trim_vy_per_vx:=0.012
```

Then wait 30 s → `free -h` → host RViz (step C). EKF + IMU stay enabled.

---

### Reference detail (same session, more explanation)

A SLAM session uses **four Jetson terminals** (or optional single launch above)
plus **RViz on the host**. Each Jetson stack on its own terminal so any one
can be killed without taking down the others.

**Topic chain (do not mix these up):**

| Topic | Publisher | Consumer |
|---|---|---|
| `/scan` | `sllidar_node` | `scan_front_filter` only |
| `/scan_filtered` | `scan_front_filter` | `slam_toolbox`, RViz LaserScan |
| `/map` | `slam_toolbox` | RViz Map (QoS: **Transient Local**) |

**Startup order:** T1 bringup → wait for `/driver_node` → T2 lidar →
T2b scan filter → wait ~5 s → T3 SLAM → wait **~30 s** → check memory →
host RViz **last**.

Before opening RViz on the host, on the Jetson run:

```bash
free -h
```

You want **≥ 2 GiB** in the `available` column. If lower, wait 10 s and
retry; do not open RViz yet. Host RViz joining the DDS network adds
load on the Jetson — starting it too early (or with SLAM still settling)
can trigger OOM kills (`exit code -9`, `Bad alloc` in bringup).

### Terminal 1 — bringup (chassis + EKF + IMU + joystick)

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py \
    chassis_serial_port:=/dev/ttyUSB0 \
    use_joystick:=true \
    trim_vy_per_vx:=0.012
```

Joystick max speeds default to **0.15 m/s** linear and **0.5 rad/s** turn
(full stick = that speed). To go even slower: `joy_xspeed_limit:=0.10`.
To restore Yahboom defaults: `joy_xspeed_limit:=1.0 joy_yspeed_limit:=1.0 joy_angular_speed_limit:=5.0`.

Wait for `cmd_vel trim: trim_vy_per_vx=0.0120` log line before
continuing.

Verify (new Jetson terminal):

```bash
ros2 node list | grep driver_node
```

Joystick: driving is **enabled by default** (Start toggles **buzzer** on
this USB pad, not drive enable). Full stick ≈ **0.15 m/s** with defaults
below.

### Terminal 2 — LiDAR

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_launch.py \
    serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
    serial_baudrate:=115200 \
    frame_id:=laser_link
```

### Terminal 2b — front-half scan filter (required on this robot)

The RPLidar sits low; upper chassis structure blocks the **rear 180°**.
Those hits look like phantom walls and ruin SLAM. This node keeps only the
forward ±90° arc and publishes `/scan_filtered`.

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 run yahboomcar_bringup scan_front_filter
```

In RViz set **LaserScan → Topic** to `/scan_filtered` (not `/scan`).

If front/rear look swapped, tune angles (degrees behind the robot kept out):

```bash
ros2 run yahboomcar_bringup scan_front_filter --ros-args \
    -p trusted_angle_min:=1.5708 -p trusted_angle_max:=-1.5708
```

### Terminal 3 — slam_toolbox (online async, **lighter params — required**)

Always use our params file (not bare `online_async_launch.py`). It
points SLAM at `/scan_filtered`, caps memory use on the 8 GB Jetson, and
sets `do_loop_closing: false`.

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=false \
    slam_params_file:=/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_bringup/param/mapper_params_online_async.yaml
```

Wait **30 seconds** after start. You will see many `Message Filter
dropping message` lines — **normal at startup**. Success signs:

- `Registering sensor: [Custom Described Lidar]`
- Drop messages **stop** after ~20–30 s
- **No** `process has died … exit code -9` (OOM — see section 7 recovery)
- **No** `ekf_node Failed to meet update rate` spam (symptom of CPU/RAM pressure)

Do **not** Ctrl+C during the first 30 s thinking it failed. Do **not**
open host RViz until SLAM has been stable for 30 s.

### Terminal 4 — RViz on **host PC** (not Jetson)

On **peter-pen** (Ubuntu Noble, ROS Jazzy):

```bash
export ROS_DOMAIN_ID=28
export ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash
ros2 run rviz2 rviz2 -d /home/bots/all/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_rviz/rviz/mapping.rviz
```

Or your saved config from the host if you exported one.

**RViz display settings (critical):**

| Display | Topic | Notes |
|---|---|---|
| **Global Options → Fixed Frame** | `map` | |
| **Views → Target Frame** | `base_footprint` | camera follows robot |
| **Map** | `/map` | **Durability: Transient Local** |
| **LaserScan** | `/scan_filtered` | half-circle forward arc |
| **TF** | disabled on host | enable only if needed; costs Jetson RAM |

If Map says **"no map received"**: check Durability is Transient Local;
drive until `/map` width/height > 0; confirm host sees `/map` with
`ros2 topic list --no-daemon`.

Host `ros2` CLI: append `--no-daemon` if the daemon misbehaves, e.g.
`ros2 --no-daemon topic list`.

### Terminal 5 — driving (joystick on Jetson, default)

Use the joystick plugged into the **Jetson** (not the host). Press
**Start** only for buzzer — sticks drive without an enable toggle.

Optional keyboard fallback on Jetson:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Slow with `x` / `c` before driving.

### Reset map mid-session

Ctrl+C **Terminal 3 (SLAM) only**, restart T3 command above. For a fully
clean map also Ctrl+C T1, reposition robot at tape mark, restart T1 → T2
→ T2b → T3.

### Terminal 6 — (optional) inspect / save

Use spare terminals for `ros2 topic echo`, `map_saver_cli`, etc. (see
section 10).

### Driving rules

These are non-negotiable for a clean map:

- **Speed: 0.10–0.15 m/s linear.** Faster causes scan-matching to lag
  behind actual motion → map smearing.
- **Avoid in-place rotations on smooth floor.** Phase 0 finding: ~190 cm
  of unwanted translation per attempted in-place spin (see
  [`phase0-status.md`](phase0-status.md) section 4c #19). Use ARCING
  turns instead — keep wheels rolling forward, with small angular.z
  added.
- **In-place rotations are fine on carpet.** Mecanum rollers grip
  carpet pile, the chassis pivots cleanly. Use this when scanning a
  corner that's hard to approach via arcing.
- **Drive perimeter first.** Go around the room close to the walls
  (~50 cm offset) so the LiDAR sees them clearly. Then sweep through
  the middle.
- **Only trust the forward 180° of LiDAR.** Rear hemisphere hits the
  robot superstructure — always run `scan_front_filter` (T2b). Map the
  full room by **turning the robot** so the forward arc sees each wall.
- **Loop back to start at least once.** With `do_loop_closing: false`
  you won't get an automatic snap, but revisiting the start still helps
  scan matching refine locally.
- **If you scrape something or the chassis jolts, drive a small loop
  near where you are** before continuing. Sudden physical disruptions
  introduce localization spikes that scan-matching will smooth out
  given another nearby observation.

---

## 5. Saving the map

Once the room looks good in RViz, save the map without taking down the
SLAM session.

In a new terminal (don't use one of the existing ones):

```bash
source /opt/ros/humble/setup.bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/maps
ros2 run nav2_map_server map_saver_cli -f my_room
```

Replace `my_room` with whatever name fits (e.g. `living_room_v1`).
The trailing `-f <name>` is the **base name** of the output files; the
saver appends `.pgm` and `.yaml` extensions itself.

You'll get two files:

| File | Format | Contents |
|---|---|---|
| `<name>.pgm` | Plain grayscale PGM image | The occupancy grid as pixels: white = free, black = occupied, gray = unknown. Open in any image viewer. |
| `<name>.yaml` | YAML | Metadata: image filename, resolution (m/pixel, default 0.05), origin in world coords, free/occupied thresholds, negate flag. |

**The `.yaml` is what tools like `nav2_map_server` actually load**; it
points at the `.pgm`. Both must travel together.

Optional — also save the SLAM session itself (the pose graph):

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/maps/my_room'}"
```

This produces `.data` and `.posegraph` files that `slam_toolbox` can
later deserialize to *resume* mapping (vs starting a fresh SLAM run).
Useful when you want to add more area to an existing map across
multiple sessions.

After saving, commit the map files:

```bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot
git add maps/
git status
```

(Don't commit yet — wait until Phase 1 acceptance criteria are met.)

---

## 6. The `slam.launch.py` we'll write next (outline only)

Once a manual session has produced a usable map, we'll capture the
recipe as a single launch file so future runs are one command. This
section outlines its shape; the actual file lives at
`cat_patrol_robot/launch/slam.launch.py` and gets written as a
follow-up to this doc.

**Outline:**

- Argument: `use_sim_time` (default false).
- Argument: `rviz` (default true; pass `rviz:=false` for headless mode).
- Argument: `slam_params_file` (default points at our own copy of
  `mapper_params_online_async.yaml`, copied from
  `/opt/ros/humble/share/slam_toolbox/config/` so we can tweak it).
- Argument: `chassis_serial_port` and `lidar_serial_port` (forwarded to
  the bringup and lidar includes; same defaults as Phase 0).
- Include `yahboomcar_bringup_X3_launch.py` with `trim_vy_per_vx:=0.012`
  and `use_joystick:=true`.
- Include `sllidar_launch.py` with our by-id symlink and
  `frame_id:=laser_link`.
- Include `scan_front_filter` node (publishes `/scan_filtered`).
- Include `slam_toolbox`'s `online_async_launch.py` with
  [`config/mapper_params_online_async.yaml`](../yahboomcar_bringup/param/mapper_params_online_async.yaml).
- RViz on host only (document env vars); do not start RViz on Jetson.

**Why a separate launch file vs reusing `cat_patrol.launch.py`?**
`cat_patrol.launch.py` brings up the existing `patrol_node` and the
mail node — Phase 0/4 plumbing. `slam.launch.py` is a separate
single-purpose composition for mapping work. Keeping them separate
matches the "one launch file per task" principle: bringup is
hardware-only, slam is mapping-only, cat_patrol is application-only.
You compose them by including, not by accumulating into one file.


---

## 7. Pitfalls and how to recover

Likely failures, sorted by how often they bite, with one-line fixes.

| Symptom | Probable cause | Fix |
|---|---|---|
| Map display "no map received" in RViz | QoS mismatch (`/map` is Transient Local) or empty 0×0 map before driving | Map display: Topic `/map`, Durability **Transient Local**. Drive 30 s; check `width`/`height` on Jetson. |
| LaserScan "could not transform laser_link to map" | TF not ready, SLAM not up, or host not receiving `/tf` | Wait 20 s after SLAM start; host `ROS_DOMAIN_ID=28`; `ros2 run tf2_ros tf2_echo map laser_link` on host. |
| `slam_toolbox` dies `exit code -9` | OOM on 8 GB Jetson | Use our lighter params file; close host RViz while SLAM starts; don't run default upstream params. |
| `yahboom_joy_X3` dies `exit code -9` + `Bad alloc` in bringup terminals | Full stack + host RViz exceeded 8 GB RAM; DDS graph corrupted | **Full restart** (all Jetson terminals + close host RViz). Wait 10 s. Restart T1→T2→T2b→T3; wait 30 s; check `free -h` ≥ 2 GiB; then RViz. Joystick dead until T1 restarted. |
| `cmd_vel timeout 0.64s > 0.60s, forcing stop` | Safety watchdog: no `/cmd_vel` for 0.6 s | **Normal when idle** (not touching stick). Also appears after `joy_node` OOM kill — consequence, not cause. |
| `ekf_node Failed to meet update rate` | EKF configured faster than CPU can run under full stack | Default now **10 Hz** (was 30). Harmless if occasional; spam + `exit -9` = OOM → full restart. |
| Joystick no motion | `driver_node` crashed, or old enable-button workflow | `ros2 node list \| grep driver`; sticks work without Start (Start = buzzer on this pad). |
| Robot very slow | Low battery | Charge; check `/voltage`. |
| Host `ros2` daemon errors | Stale Jazzy daemon on peter-pen | `pkill -9 -f ros2cli.daemon`; use `ros2 --no-daemon topic list`. |
| Chaotic map / phantom walls near robot | Rear lidar hits robot superstructure | Run `scan_front_filter`; SLAM + RViz use `/scan_filtered`. |
| Map looks "smeared" — walls have ghost copies offset by a few cm | Driving too fast → scan matcher can't track | Slow to 0.10 m/s. Restart SLAM (no need to restart bringup or lidar). |
| Walls drift / duplicate the longer you drive | Odom drifting; loop closure hasn't fired | Drive back to a previously-seen area. Loop closure should fire and snap things into place. |
| `/scan` shows nothing in RViz | Either lidar motor stopped, or `frame_id` mismatch | `ros2 service call /start_motor std_srvs/srv/Empty`; verify `/scan`'s frame_id is `laser_link` not `laser`. |
| LiDAR sees beyond windows, sees mirror-reflections | Laser triangulation passes through glass and reflects off mirrors | Expected. Either accept the artefacts or edit them out of the saved `.pgm` later in GIMP. |
| Cats walk through scan plane | Moving obstacles get added to the map | SLAM tries to filter dynamic obstacles, but if it's bad, restart with cats elsewhere. |
| Smooth-floor in-place rotation produces a long arc | Mecanum rollers slip on smooth floor (Phase 0 finding) | Avoid in-place rotation on smooth floor — use arcing turns. |
| TF tree errors: "frame X does not exist" | Usually a node crashed (commonly EKF) | Check terminal 1 for crashed nodes. Restart bringup. |
| Map cells look noisy / speckled | LiDAR bouncing off shiny floor or returning bogus near-range | Slow down further. Increase `min_laser_range_meters` in slam params from 0 to 0.15. |
| `slam_toolbox` logs "Message Filter dropping message" | Normal at startup while TF tree settles | Ignore unless it persists more than ~5 seconds. |
| Mapping seems to do nothing after a long drive | TF chain broken or SLAM not getting scans | `ros2 run tf2_ros tf2_echo map odom`; `ros2 topic hz /scan_filtered`; confirm T2b filter running. |
| Robot doesn't move on `i` press | cmd_vel watchdog has stopped motors | Press `i` more firmly. Watchdog re-arms on first cmd_vel. |

### Why OOM keeps happening (8 GB Jetson budget)

**Measured on this robot (EKF + IMU ON):**

| Stage | RAM used | Notes |
|---|---|---|
| T1 + T2 + filter (no SLAM) | **~250 MiB** | Not 2 GB — bringup is cheap |
| `slam_toolbox` starting | **~1.0–1.5 GiB** | This is the cliff |
| Host RViz joining network | **+300–500 MiB** on Jetson | DDS + `/tf` fan-out |
| ollama + Cursor + swap thrash | **+0.5–1.5 GiB** | Same Jetson you SSH from |

So if you had **~2 GiB free** before SLAM, SLAM alone can take most of it; opening RViz at the same time pushes over the edge → `exit -9`.

**We keep EKF + IMU.** Other mitigations:
- **Reboot** if swap > 512 MiB (your Jetson often has ~1.2 GiB swapped after failed runs)
- `sudo systemctl stop ollama` before mapping (not robot nodes)
- Fast-DDS profile (`fastdds_mapping.xml`) — set by `slam_mapping.launch.py`
- **Host RViz last**, 30 s after SLAM; **disable TF display** (Map + LaserScan only)
- Lighter SLAM yaml (already applied)

**Mitigations (no `cat_patrol_robot` required for mapping):**
- Manual T1→T2→T2b→T3 (see section 10) — uses `yahboomcar_bringup` + `sllidar_ros2` + `slam_toolbox` only
- Optional one-liner: `ros2 launch yahboomcar_bringup slam_mapping_X3_launch.py`
- Reboot if swap > 512 MiB; stop ollama before mapping
- Host RViz last; Map + LaserScan only (no TF display)
- Lighter SLAM yaml in `yahboomcar_bringup/param/mapper_params_online_async.yaml`

### OOM recovery (when bringup shows `Bad alloc` or joy `exit -9`)

Once `Bad alloc` appears, **do not** keep running — DDS state is unreliable.

1. Ctrl+C **all** Jetson terminals (T1–T3) and **close** host RViz.
2. Wait 10 s; optionally `pkill -9 -f ros2` on Jetson if anything stuck.
3. Confirm memory recovered: `free -h` → `available` should be **> 5 GiB**.
4. **Restart:** T1→T2→T2b→T3 (section 10), or `ros2 launch yahboomcar_bringup slam_mapping_X3_launch.py`
5. In RViz use **minimal displays**: Map + LaserScan only — **disable TF display** (TF over WiFi is a major Jetson RAM/CPU hit).

The first `cmd_vel timeout` lines right after T1 start are **normal** (no stick input yet).

---

## 8. Concept anchors (the study guide)

This is the "ask me later" section. Each bullet is a concept I (the
project's AI assistant) am ready to teach in detail when you query.
Don't try to learn them all up-front — pick whichever feels fuzzy
during a session.

### 8a. ROS 2 concepts to query me about

- **`nav_msgs/OccupancyGrid` message structure.** Header, MapMetaData
  (resolution, width, height, origin), and the int8 `data[]` array.
  Why is `data` a flat int8 not a 2D array? How does row-major indexing
  work?
- **The `/map` topic — why it's latched and low-frequency.** Latched
  topics in ROS 2 (transient_local QoS), the difference between "every
  subscriber gets the latest cached message" vs "subscribers only see
  future messages".
- **The `map → odom` transform as drift correction.** How does it
  *compose* with `odom → base_footprint` (from EKF) and the static
  TFs? Why don't we just have a `map → base_footprint` directly?
- **Launch-file composition.** `IncludeLaunchDescription`,
  `LaunchConfiguration`, `OpaqueFunction`. We touched the surface in
  Phase 0; in Phase 1's `slam.launch.py` we use them more.
- **YAML parameter files for slam_toolbox.** The
  `mapper_params_online_async.yaml` template, what each parameter does
  (`solver_plugin`, `ceres_*`, `correlation_search_space_*`,
  `loop_match_*`, `link_match_*`).
- **The `map_server` and `map_saver_cli` tools.** How does
  `map_saver_cli` ask `slam_toolbox` for the current map? Subscribe vs
  service call. Why does the saver need to be in the map's "save
  directory" or accept absolute paths?
- **`sensor_msgs/LaserScan` deeper dive.** `angle_min`,
  `angle_increment`, `range_min`, `range_max`, `ranges[]`,
  `intensities[]`, `scan_time`, `time_increment`. How
  `slam_toolbox` actually uses each.
- **TF lookup at a specific message timestamp.** In Phase 0 the drift
  checker used `tf2::TimePointZero`. The educational alternative is
  `tf2_ros::fromMsg(msg.header.stamp)` — when do you want which?
- **slam_toolbox modes.** Online async vs online sync vs offline vs
  lifelong vs localization. Which one we picked (online async) and
  why.
- **slam_toolbox localization-only mode** as a Phase 2 preview. It's
  a competing alternative to `nav2_amcl` — same job, different
  algorithm.
- **transient_local vs reliable QoS** specifically as it applies to
  the `/map` topic and why `slam_toolbox` chose what it did.

### 8b. SLAM theory to query me about

- **Occupancy grid representation.** free=0, occupied=100, unknown=-1.
  Cell coordinates vs world coordinates. The world↔grid conversion
  (origin + resolution + row-major flatten).
- **Bayesian log-odds update of cell occupancy.** Why log-odds
  instead of plain probability — additive updates, numerical
  stability. The Bayesian formulation of "I see a wall here" vs "I
  see clear space here".
- **Scan matching.** `slam_toolbox` uses Karto's correlative scan
  matcher under the hood. What that means: brute-force search of (x,
  y, yaw) offsets to align a new scan with an existing local map.
  Why it works at all (geometry of indoor environments). Where it
  fails (long featureless corridors, symmetric rooms, glass).
- **Pose graph SLAM.** Front-end (graph building: each scan becomes a
  node, scan-matches between nearby scans become edges) vs back-end
  (graph optimization: solve the joint configuration that minimizes
  edge errors). `slam_toolbox` uses Ceres Solver as the back-end.
- **Loop closure detection.** How `slam_toolbox` decides "I've been
  here before". The candidate-search radius, the match-quality
  threshold, why false loop closures are catastrophic.
- **Pose graph optimization on loop closure.** When a loop closes,
  what gets re-distributed? Every node in the cycle. Why does the
  whole map "snap" visibly when a loop closes — what's actually being
  recomputed.
- **Why `slam_toolbox` is graph-based and not particle-filter-based.**
  The contrast vs `gmapping` (the old ROS 1 default) and `nav2_amcl`
  (Phase 2's localization). Trade-offs: graph SLAM is more accurate
  for mapping; particle filters are better for localization on a
  fixed map.
- **The free-space rays.** Each LiDAR ray contributes a "free" update
  along its length and an "occupied" update at its endpoint. Why
  this is more useful than just marking endpoints.
- **Map drift vs odom drift.** Both exist. Which one the EKF fights,
  which one `slam_toolbox` fights, and why both layers are needed.

### 8c. Existing C++ code to read (no writing required)

A curated list of files to study, all already accessible. Each entry
is what's there to learn.

- **`slam_toolbox` source** at
  [github.com/SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox).
  Specific files:
  - `src/slam_toolbox_async.cpp` — the async-mode node entry-point.
    Compare to Phase 0's `odom_drift_checker_node.cpp` to see how a
    real production ROS 2 node is structured (lifecycle, callback
    groups, parameter callbacks, multi-threaded executor).
  - The Karto wrapper code — how a third-party C++ library gets
    integrated into a ROS 2 node.
  - Loop-closure detection logic — the actual search-and-match
    algorithm.
- **`nav2_map_server` source** at
  [github.com/ros-planning/navigation2/tree/main/nav2_map_server](https://github.com/ros-planning/navigation2/tree/main/nav2_map_server).
  How `nav_msgs/OccupancyGrid` is serialized to/from `.pgm` + `.yaml`.
  Concrete example of file I/O alongside ROS message handling.
- **`robot_localization`'s `ekf_filter_node`** — already touched in
  Phase 0; revisit how it composes with the TF tree once
  `slam_toolbox` adds the `map → odom` link.
- **`base_node_X3.cpp`** in this workspace at
  [`yahboomcar_ws/src/yahboomcar_base_node/src/base_node_X3.cpp`](../yahboomcar_base_node/src/base_node_X3.cpp).
  Reread as warm-up; the dead-reckoning math from Phase 0 will look
  closer to scan-matching geometry once you've read 8b.
- **`odom_drift_checker_node.cpp`** at
  [`odom_drift_checker/src/odom_drift_checker_node.cpp`](../odom_drift_checker/src/odom_drift_checker_node.cpp).
  Re-read with the new context: the TF lookup, the std::optional, the
  parameter idiom — they all reappear in `slam_toolbox`'s code at a
  larger scale.


---

## 9. ROS 2 idioms exercised in this phase

These are practical idioms the phase will exercise hands-on (vs the
deeper concepts in section 8 you can ask me about):

- **Composing one launch file from multiple existing ones.** Phase 0
  used `OpaqueFunction` lightly; Phase 1's `slam.launch.py` uses
  multiple `IncludeLaunchDescription`s.
- **Latched topic semantics.** `/map` is published with
  `transient_local` reliability and the cached most-recent message —
  RViz subscribes any time and immediately gets the current map.
  Compare with the streaming behaviour of `/scan` and `/odom`.
- **TF tree growth.** Phase 0's TF tree had 4 contributors. Phase 1
  adds `slam_toolbox` as a 5th, publishing `map → odom`. View it with
  `ros2 run tf2_tools view_frames`.
- **Service calls.** New ones in this phase: `/start_motor` and
  `/stop_motor` from the lidar driver, `/slam_toolbox/serialize_map`
  and `/slam_toolbox/deserialize_map`, `/slam_toolbox/save_map`.
- **Visual debugging in RViz vs terminal-only debugging.** Phase 0
  was almost entirely terminal-driven (`ros2 topic echo`, log lines).
  Phase 1 you really do need RViz — the map is geometric and the
  fastest debugger is your eyes.

---

## 10. Commands cheat-sheet

**Tomorrow:** follow **section 4 → "Tomorrow — start here"** first.
This section is the same commands in compact form.

Copy-paste-ready. On the **Jetson**, always:

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
```

On the **host**: Jazzy + `ROS_DOMAIN_ID=28` + `ROS_LOCALHOST_ONLY=0`.

### 0. Pre-flight (Jetson, once per session)

```bash
free -h
sudo systemctl stop ollama    # optional, recommended
for d in /dev/ttyUSB*; do
  pid=$(udevadm info -q property -n "$d" 2>/dev/null | grep '^ID_MODEL_ID=' | cut -d= -f2)
  [ "$pid" = "7523" ] && echo "CHASSIS=$d"
done
readlink -f /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

### Verify installation (Jetson)

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 pkg prefix slam_toolbox
ros2 pkg prefix nav2_map_server
ros2 pkg executables yahboomcar_bringup | grep scan_front_filter
```

### Optional all-in-one (Jetson, replaces T1+T2+T2b+T3)

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch yahboomcar_bringup slam_mapping_X3_launch.py \
    chassis_serial_port:=/dev/ttyUSB0 \
    trim_vy_per_vx:=0.012
```

### Bringup — Jetson terminal 1

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py \
    chassis_serial_port:=/dev/ttyUSB0 \
    use_joystick:=true \
    trim_vy_per_vx:=0.012
```

Joystick caps (defaults): `joy_xspeed_limit=0.15`, `joy_yspeed_limit=0.15`,
`joy_angular_speed_limit=0.5`. Slower: add `joy_xspeed_limit:=0.10`.

### LiDAR — Jetson terminal 2

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_launch.py \
    serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
    serial_baudrate:=115200 \
    frame_id:=laser_link
```

### Scan filter — Jetson terminal 2b (**required**)

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 run yahboomcar_bringup scan_front_filter
```

Fallback if `No executable found`: rebuild `yahboomcar_bringup`

### slam_toolbox — Jetson terminal 3 (**always use lighter params**)

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=false \
    slam_params_file:=/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_bringup/param/mapper_params_online_async.yaml
```

### RViz — **host** terminal 4

```bash
export ROS_DOMAIN_ID=28
export ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash
ros2 run rviz2 rviz2 -d /home/bots/all/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_rviz/rviz/mapping.rviz
```

Map: `/map`, Durability **Transient Local**. LaserScan: `/scan_filtered`.

### Teleop fallback — Jetson terminal 5 (optional)

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Inspect during the session (Jetson)

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 topic hz /scan_filtered
ros2 topic hz /odom
ros2 topic hz /map
ros2 topic echo /map --once | grep -E 'width|height'
ros2 run tf2_ros tf2_echo map laser_link
ros2 node list | grep -E 'driver|slam|scan_front'
ros2 topic echo /voltage --once
```

### Inspect from host

```bash
export ROS_DOMAIN_ID=28
export ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash
ros2 --no-daemon topic list | grep -E 'map|scan_filtered|tf'
ros2 run tf2_ros tf2_echo map laser_link
```

### Save the map

```bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/maps
ros2 run nav2_map_server map_saver_cli -f my_room
```

### Serialize the SLAM session (optional, for resuming later)

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/maps/my_room'}"
```

### Pause / resume the lidar motor

```bash
ros2 service call /stop_motor std_srvs/srv/Empty
ros2 service call /start_motor std_srvs/srv/Empty
```

---

## 11. Phase 1 status log (filled in over time)

Empty for now. Fill in as you run sessions, the same way Phase 0's
calibration record filled in. Keep these terse and factual — surprises
and findings, not narration.

### Session 1 — 2026-06-20 (interrupted — battery)

- Room: _(fill in)_
- Start position (tape mark): yes
- Battery at start / end: low at end → robot very slow
- Trim used: `0.012`
- Stack: bringup + lidar + `scan_front_filter` + lighter SLAM params + host RViz (Jazzy)
- Findings:
  - Rear 180° LiDAR untrustworthy (robot structure) → `/scan_filtered` required
  - Chaotic map before filter; SLAM OOM with default params → lighter yaml
  - Host RViz: Map needs Transient Local QoS; LaserScan `/scan_filtered`
  - Joystick: Start = buzzer; drive enable defaulted on in `yahboom_joy_X3`
  - Joy speed capped at 0.15 m/s via bringup launch args
- Outcome: map not saved yet; resume after charge

### Session 2 — 2026-06-21 (OOM when host RViz started)

- Stack: T1–T3 running; opened host RViz → bringup T1 showed cascade
- Symptoms: `yahboom_joy_X3 exit -9`, `Bad alloc` in multiple nodes,
  `ekf_node Failed to meet update rate` spam
- Cause: 8 GB Jetson RAM exhausted (bringup + lidar + filter + SLAM + DDS
  load when host RViz joined)
- Recovery: full restart all terminals; wait 30 s after SLAM before RViz;
  check `free -h` ≥ 2 GiB available first
- Outcome: _(resume after full restart)_

### Session 3 — 2026-06-21 (stopped for day — resume tomorrow)

- Status: mapping not completed; no map saved yet
- Next run: section 4 **"Tomorrow — start here"** (verify USB ports first)
- Key fixes since session 2:
  - `scan_front_filter` moved to `yahboomcar_bringup` (not `cat_patrol_robot`)
  - Chassis port may be `/dev/ttyUSB0` (verify with CH340 7523 loop)
  - LiDAR stays on CP2102 by-id symlink
  - EKF + IMU kept; OOM mitigated by reboot/stop ollama/Rviz last/no TF display
- Outcome: _(pending)_

### Session 4 — _date_

(copy template above)

### Final results

- Map file: `maps/<name>.pgm` + `.yaml`
- Resolution: ___ m/cell
- Map dimensions: ___ × ___ m
- Total drive distance during mapping: ___ m
- Drift observed (visual): ___
- Loop-closure events: ___
- Anything unusual:

### Calibration changes

- (record any time `trim_vy_per_vx` or other Phase 0 calibration
  values were re-measured or re-tuned because of Phase 1 observations)

---

## 12. Open questions / forward links

- **Phase 2 (AMCL).** The saved `.pgm` + `.yaml` from this phase
  becomes AMCL's input map. AMCL is a particle-filter-based
  localizer that estimates the robot's pose in the saved map at
  runtime. Same `map → odom` TF, different algorithm producing it.
- **Phase 7 (multi-room).** The plan's Phase 7 includes a
  multi-room item; that would extend Phase 1's per-room map approach
  to a set of saved maps with a "go to map X" command. Don't try to
  do this in Phase 1.
- **Surface-aware patrol planning.** Phase 0 added this as an open
  question in [`phase0-status.md`](phase0-status.md) section 6. The
  Phase 1 map is the substrate — once we have it, you can hand-paint
  a `floor_type.yaml` overlay (carpet vs smooth) and Phase 4's
  patrol manager can use that overlay to route in-place rotations
  onto carpet.
- **Map editing.** It's normal to clean up the saved `.pgm` in GIMP
  before using it for navigation: delete moving-obstacle artefacts
  (cats), delete phantom returns from glass and mirrors, fill in
  small holes. Considered "manual map curation" — not cheating; this
  is what professional navigation pipelines also do.
- **Multiple SLAM runs on the same room.** Each run produces a
  slightly different map even with identical driving (different
  starting noise, different random tie-breaks in the optimizer).
  Run 2-3 sessions and pick the cleanest one.
- **`slam_toolbox` lifelong mode.** Continuously update the map
  across sessions. Mentioned for context; Phase 1 uses online async
  for simplicity. Lifelong is a Phase 7 polish item.

---

## 13. UPDATE 2026-06-26 — Phase 1 solved; SLAM moved to the host

Phase 1 mapping now produces correct, non-smeared maps. This section supersedes
the Jetson-side `slam_toolbox` instructions in §4/§10.

### 13a. Root cause of the map "collapse": the LiDAR was mounted ~180° rotated

**Symptom.** Driving a rectangle produced a map that **collapsed into a diagonal
"bowtie"** — the four perpendicular legs rotated onto one axis, walls smeared. It
happened with scan matching ON at *any* field of view (even full 360°), yet
**pure-odometry mapping** (`use_scan_matching: false`) kept correct perpendicular
geometry.

**Diagnosis path** (each step eliminated a suspect):
1. **Odometry is good** — a driven 90° turn moved `/odom` yaw ~90° (IMU-sourced),
   a 1.0 m drive read 1.03 m. Not odom.
2. **Filter doesn't re-stamp** — `scan_front_filter` copies the header
   (`out.header = msg.header`), preserving capture time.
3. **No clock skew** — Jetson/host agree in UTC epoch (NTP). Timezones were
   mismatched but that's display-only (fixed anyway: both Asia/Jerusalem, RTC UTC).
4. **FOV is not it** — collapse happens at full 360° too.
5. **Known-object test (smoking gun)** — an object at the robot's *true front*
   showed up in raw `/scan` at **~180°**, not 0°. → **laser-0° points to the
   robot's REAR.**

**Why that single fact explained every symptom:**
- `scan_front_filter` kept laser ±110° (around laser-0°) thinking it was the front,
  but that is the robot's **rear** — exactly where the **antennas + wires** sit. So
  SLAM was fed the antenna-contaminated rear and the clean front was discarded.
- Those antenna/wire returns are **body-fixed** (move with the robot) → the
  scan-matcher anchors on them and won't register rotation → **collapse**.
- The identity `base_link→laser_link` TF (claiming laser-0° = front) also placed
  points **reflected**: a fixed wall lands at `2·sensor − wall`, so it appears to
  move at ~2× as the robot drives → smear.

**The fix (two parts — the filter runs in raw device angles, *before* TF):**
1. **URDF:** `laser_joint` rpy yaw = **π** in `yahboomcar_X3.urdf` (+ `.xacro`) so
   kept points are placed in the correct direction.
2. **Filter:** rewritten to a wrap-safe *trusted arc* — params `trusted_center`,
   `trusted_halfwidth`; `t2.5.sh` now passes `trusted_center:=π`,
   `trusted_halfwidth:=1.92` → keep clean front 220°, clear rear ~140° (antennas).

Scan matching re-enabled → rectangle maps cleanly, no collapse.

### 13b. Who runs where (current architecture)

| Machine | Runs | Role |
|---|---|---|
| **Jetson** (Ubuntu 22.04, ROS 2 **Humble**) | `yahboomcar_bringup` (base_node, IMU filter, EKF→`/odom`, joy), `robot_state_publisher` (URDF→TF), `sllidar_ros2` (`/scan`), `scan_front_filter` (`/scan_filtered`) | sensors + odom + TF only |
| **Host `peter-pen`** (Ubuntu 24.04, ROS 2 **Jazzy**) | Humble **Docker** containers: `humble-docker-slam-1` (`slam_toolbox`), `humble-docker-rviz-1` (`rviz2`) | mapping + visualization |

### 13c. Why SLAM runs in a Humble *Docker* on a Jazzy host

The host OS ships ROS 2 **Jazzy**; the robot runs **Humble**. They use **different
DDS wire encodings (XCDR1 vs XCDR2) and are NOT wire-compatible** — running Jazzy
`slam_toolbox` directly against the robot floods `serdata.cpp "invalid data size"`
errors and drops every string-bearing message (`/rosout`, `/parameter_events`). So
the host runs SLAM + RViz inside **Humble Docker containers** that speak the robot's
wire format. Compose project: `/home/bots/humble-docker/` (services `slam`, `rviz`;
both `network_mode: host`, `ipc: host`).

### 13d. How cross-machine connectivity is enabled (DDS)

- **One ROS graph across two machines:** every process exports
  `ROS_DOMAIN_ID=28`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
  `CYCLONEDDS_URI=file://…/cyclonedds.xml`, `ROS_LOCALHOST_ONLY=0`. Mismatch any one
  and `ros2 node list` shows nothing across the link (this is the #1 "why can't I
  see the topics" trap).
- **CycloneDDS hardened for WiFi** (`/home/jetson/cyclonedds.xml` &
  `/home/bots/cyclonedds.xml`): `<AllowMulticast>spdp</AllowMulticast>` (multicast
  for discovery only, unicast for data — kills a retransmit storm that OOM-killed
  bringup when a remote subscriber joined), bounded `<WhcHigh>500kB`, larger kernel
  socket buffers via sysctl. **Read once at process start → after editing the xml,
  restart all nodes** (mixed configs cause silent TF dropouts).
- Containers bind-mount the same xml: `-v /home/bots/cyclonedds.xml:/cfg/...:ro`.

### 13e. The TF chain (who publishes each link, on which machine)

| Transform | Published by | Machine |
|---|---|---|
| `map → odom` | `slam_toolbox` (drift correction) | **Host** container |
| `odom → base_footprint` | `ekf_filter_node` (robot_localization) | Jetson |
| `base_footprint → base_link → laser_link` | `robot_state_publisher` (from URDF) | Jetson |

**Key insight:** `slam_toolbox` consumes the `…→laser_link` transform from `/tf` /
`/tf_static` over DDS — **it never reads a URDF**. So the lidar-flip fix lives
entirely in the Jetson URDF and the TF it broadcasts; nothing on the host needs the
URDF for SLAM to be correct (the host RViz robot *mesh* is cosmetic). Because
`laser_joint` is *fixed*, it's on **`/tf_static`** (latched/transient_local) — so
**restart slam AFTER restarting bringup** so it picks up the corrected static TF.

### 13f. Host Docker — inspect / edit params / restart

- SLAM params: **`/home/bots/slam_params/mapper_params_host.yaml`**, bind-mounted
  read-only to `/slam_params` in the container; launched
  `ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/slam_params/mapper_params_host.yaml`.
- Inspect live params (from any domain-28 cyclonedds shell): `ros2 param dump /slam_toolbox`
- Edit + apply: change the host yaml → `cd ~/humble-docker && sudo docker compose restart slam`
  (restart also **clears the accumulated map** — good for a fresh run).
- Containers/mounts: `sudo docker ps`,
  `sudo docker inspect humble-docker-slam-1 --format '{{json .Mounts}}'`.

### 13g. Screenshot / debug workflow (Jetson has no display)

- The map/RViz live on the host; capture screenshots there and copy to the Jetson:
  `scp /home/bots/Pictures/Screenshots/imNN.png jetson@192.168.0.120:~/viz_images/`.
- Reusable diagnostics on the Jetson at **`/home/jetson/slam_debug/`**:
  `raw_closest.py` (closest raw-scan returns by angle — the known-object test),
  `yaw_mon.py` (compare `/odom` vs `/imu` vs wheel yaw), `pos_mon.py` (odom vs wheel
  position), `diag.py` (scan/odom stamp ages + scan content). Run with the
  domain-28 cyclonedds env; pipe with `python3 -u … > file` (ROS stdout buffers).

### 13h. Updated startup order (supersedes §4/§10 Jetson-SLAM)

Scripts in `~/myscripts2/` already carry the domain-28 + cyclonedds env.

1. **Jetson `t1.sh`** — bringup (chassis, IMU, EKF→`/odom`, joy, robot_state_publisher).
   Restarting **zeroes odom** so the map starts centered.
2. **Jetson `t2.sh`** — lidar (`/scan`).
3. **Jetson `t2.5.sh`** — `scan_front_filter` (`/scan_filtered`, `trusted_center=π`).
4. **Host** — `cd ~/humble-docker && sudo docker compose restart slam`
   (and `docker compose up rviz` after `xhost +local:root`).

---

## 14. Phase 1 status log — Session 2026-06-26 (RESOLVED)

- **Architecture:** SLAM + RViz on host (Humble Docker); Jetson = sensors/odom/TF.
- **Root cause** of the long-running map collapse/bowtie: **LiDAR mounted ~180° vs
  the URDF** → `scan_front_filter` kept the rear antenna/wire cone → scan-matcher
  anchored on body-fixed antennas → couldn't register rotation → collapse.
- **Fix:** URDF `laser_joint` yaw = π + `scan_front_filter` rewritten to a wrap-safe
  trusted arc (`trusted_center=π`, `trusted_halfwidth=1.92`). Scan matching ON again.
- **Also earlier this session:** EKF retuned to take **heading from IMU only** (X3
  wheel rotation is unreliable); verified driven 90° turn ≈ 90° in `/odom`, 1 m drive
  ≈ 1.03 m. Jetson timezone aligned to host (Asia/Jerusalem, RTC UTC).
- **Outcome:** rectangle drives map **cleanly, no smear.**
- **Next:** re-enable `do_loop_closing: true`, loosen the conservative matcher
  params (`correlation_search_space_dimension` 0.3→0.5, `coarse_search_angle_offset`
  0.2→0.349, penalties back to defaults), drive a full room, save the `.pgm`/`.yaml`
  to meet the §2 acceptance criteria.

---

## 15. Lessons learned — study tables

A curriculum distilled from what Phase 1 actually exercised. Columns: **Topic** ·
**What we hit / why it matters** · **Go deeper**. Pick a row, ask me to teach it.

### 15a. ROS 2 core

| Topic | What we hit / why it matters | Go deeper |
|---|---|---|
| Nodes, topics, pub/sub | The whole stack is nodes exchanging messages; `ros2 node list`/`topic list` were the first debugging reflex | `ros2 topic info -v`, message_filters |
| QoS profiles | `/map` is **transient_local** (latched) → late subscribers still get it; `/scan`,`/odom` are volatile/best-effort. A QoS mismatch = silent "no data" | Reliability, Durability, History, depth; RMW QoS compatibility matrix |
| Parameters | `slam_toolbox` is tuned entirely via params; we read them live with `ros2 param dump`, changed behavior without code | declare/get, param files (YAML), `ros2 param set`, param callbacks |
| Launch files | `xacro` is run on the URDF at launch; bringup composes many nodes | `IncludeLaunchDescription`, `LaunchConfiguration`, `OpaqueFunction`, `Command` substitution |
| `ros2 run` vs `ros2 launch` | We ran the filter via `ros2 run … --ros-args -p k:=v`; bringup via launch | console_scripts entry points, `--ros-args` remap/param syntax |
| Time | message **header.stamp** (epoch) vs wall clock; `use_sim_time` | `rclpy`/`rclcpp` Time & Duration, sim time |
| ament/colcon build | `--symlink-install` made Python edits live; we rebuilt one package; egg-link editable install | overlay vs underlay, `--packages-select`, build/install/log layout |

### 15b. TF2 & coordinate frames

| Topic | What we hit / why it matters | Go deeper |
|---|---|---|
| The TF tree | `map→odom→base_footprint→base_link→laser_link` spans two machines; one wrong link broke SLAM | `tf2_tools view_frames`, `tf2_echo` |
| Static vs dynamic TF | `laser_joint` is fixed → `/tf_static` (latched); needed careful restart ordering | `static_transform_publisher`, transient_local on `/tf_static` |
| TF at a timestamp | scan-matching needs the pose at the *scan's* stamp; WiFi lag vs `transform_timeout` dropped scans | tf2 buffer, interpolation, extrapolation, `lookupTransform` |
| Frame conventions (REP-103/105) | x-forward, y-left, z-up, CCW yaw; **the lidar 180° flip was a frame-convention bug** | REP-103, REP-105, right-hand rule |
| Mounting transforms in URDF | the fix was literally a joint `rpy` yaw=π | URDF `<joint>` origin, xacro macros |

### 15c. Sensors & messages

| Topic | What we hit / why it matters | Go deeper |
|---|---|---|
| `sensor_msgs/LaserScan` | `angle_min/max/increment` (sign = handedness), `ranges[]`, `scan_time`; the known-object test read the angle of a return | how slam_toolbox consumes each field; intra-scan motion distortion |
| Scan handedness/orientation | positive `angle_increment` = CCW; raw angle 0 turned out to be robot-rear | A1 spins clockwise physically; driver maps to ROS CCW |
| `nav_msgs/Odometry` | compared `/odom` (EKF) vs `/odom_raw` (wheel); quaternion→yaw | pose vs twist, covariance, child_frame_id |
| `sensor_msgs/Imu` | gyro `angular_velocity.z` drove heading; bias at rest tiny | orientation vs angular_velocity, madgwick filter |
| Quaternions ↔ yaw | converted constantly while debugging headings | `tf_transformations`, atan2 form, gimbal-free reasoning |

### 15d. SLAM (slam_toolbox / Karto)

| Topic | What we hit / why it matters | Go deeper |
|---|---|---|
| Online async mapping | our mode; builds occupancy grid + pose graph live | sync vs async vs lifelong vs localization modes |
| Scan matching | the part that **collapsed** the map; it rotates/translates scans to fit — and locks onto body-fixed points | Karto correlative scan matcher; `correlation_search_space_*`, `*_search_angle_offset` |
| `use_scan_matching` on/off | turning it OFF (pure odom) isolated the bug — a key diagnostic | when to trust odom vs matcher |
| Pose graph + back-end | nodes = scans, edges = matches; Ceres optimizes | front-end vs back-end, `ceres_*` params |
| Loop closure | left OFF (8 GB + false-closure risk in repetitive space) | candidate search radius, match thresholds, why false closures are catastrophic |
| OccupancyGrid | `/map`: free/occupied/unknown, resolution, origin, row-major `data[]` | world↔grid conversion, log-odds updates, free-space rays |
| `map→odom` as correction | how SLAM corrects EKF drift without touching odom | composition with the TF tree |
| map_saver / serialize | `.pgm`+`.yaml` for nav; `.posegraph` to resume | `nav2_map_server`, SerializePoseGraph |

### 15e. Sensor fusion & odometry (robot_localization EKF)

| Topic | What we hit / why it matters | Go deeper |
|---|---|---|
| EKF config vectors | `odom0_config`/`imu0_config` 15-bool masks decided we fuse wheel **velocity** + IMU **yaw** only | robot_localization params, `two_d_mode` |
| Why heading from IMU | X3 wheel rotation is unreliable (~dead in-place); IMU gyro is accurate | dead-reckoning error sources, mecanum slip |
| Verifying odom empirically | driven-turn & 1 m-drive tests (`yaw_mon.py`, `pos_mon.py`) proved odom before blaming it | controlled experiments, ground-truth checks |
| `reset_on_time_jump` | why the Jetson "RTC in local TZ" was a latent hazard | clock jumps & filter resets |

### 15f. DDS / middleware / multi-machine

| Topic | What we hit / why it matters | Go deeper |
|---|---|---|
| RMW abstraction | nodes were invisible until shell used `rmw_cyclonedds_cpp` + domain 28 | rmw layer, FastDDS vs CycloneDDS |
| `ROS_DOMAIN_ID` | the stack lives on **28**, not 0 — wrong domain = empty graph | domain → UDP port mapping |
| CycloneDDS config | hardened xml fixed WiFi OOM (spdp multicast, WhcHigh, sysctl buffers) | CycloneDDS XML, discovery vs data, writer history cache |
| Cross-distro wire (in)compat | Humble↔Jazzy not wire-compatible → Humble Docker on host | XCDR1 vs XCDR2, type hashes |
| Discovery & services | `ros2 param dump` succeeding **proved** the remote node was alive | SPDP/SEDP discovery, request/response services |

### 15g. Docker & cross-distro ops

| Topic | What we hit / why it matters | Go deeper |
|---|---|---|
| Containerizing ROS | host runs SLAM/RViz in Humble images to match the robot | `network_mode: host`, `ipc: host` for DDS/shared-mem |
| Bind mounts | params + cyclonedds.xml mounted read-only into the container | `docker inspect … Mounts`, `:ro` |
| Compose lifecycle | `docker compose restart slam` applies params + clears map | services, restart vs up/down |
| GUI from container | RViz needs `DISPLAY` + X11 socket + `xhost +local:root` | X forwarding, `/tmp/.X11-unix` |

### 15h. Debugging methodology (the meta-lessons)

| Lesson | What happened here |
|---|---|
| **Isolate variables one at a time** | scan-matching ON vs OFF split "placement" from "matching" and located the failure |
| **Verify assumptions with a measurement, not a guess** | the known-object test beat three plausible theories (FOV, timestamps, odom) |
| **Don't anchor on the first plausible cause** | "limited FOV" looked right and was wrong; the user's "is front actually back?" cracked it |
| **Rule things out cheaply first** | odom, clock, re-stamping checked before touching the matcher |
| **Trust the data over the model** | TF *said* identity; reality was 180° off — the URDF only encodes an assumption |
| **Make diagnostics reusable** | the `slam_debug/` scripts are kept for next time |

### 15i. C++ for ROS (reading real nodes — no writing required)

| Target | What to learn |
|---|---|
| `slam_toolbox` `src/slam_toolbox_async.cpp` | production node shape: lifecycle, callback groups, param callbacks, multi-threaded executor |
| Karto wrapper in slam_toolbox | integrating a third-party C++ lib into a ROS 2 node |
| `nav2_map_server` | OccupancyGrid ↔ `.pgm`/`.yaml` file I/O alongside ROS messages |
| `robot_localization` `ekf_filter_node` | how the EKF composes with the TF tree once `map→odom` is added |
| this workspace `base_node_X3.cpp` | the dead-reckoning math that explains why X3 wheel yaw is unreliable |
| this workspace `scan_front_filter.py` | smallest possible sub→process→pub node; the trusted-arc + wrap-normalize logic |
