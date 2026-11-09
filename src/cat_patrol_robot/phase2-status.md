# Phase 2 — Localization Status & Reference

Companion to [`plan.md`](plan.md), [`phase1-status.md`](phase1-status.md),
[`hardware.md`](hardware.md), and [`architecture.md`](architecture.md).

Phase 2 = localize on the saved map using `nav2_amcl`. The robot knows the
room; now it needs to know **where it is in the room** at runtime.

> **STATUS: PASSED.** AMCL + map_server + lifecycle_manager running on Jetson.
> Particle cloud converges, `map → odom` TF is stable, all acceptance criteria met.
> See §13 for the deep-dive learning guide — topics, components, RViz views, and verification commands.

---

## 1. Purpose and scope

**Goal.** Bring up `nav2_amcl` + `nav2_map_server` + `nav2_lifecycle_manager`
on the Jetson, set an initial pose in RViz, drive the robot, and watch the
particle cloud converge to a tight cluster.

**Why this phase matters (from plan.md).**
- This is your first hands-on encounter with **lifecycle nodes** — one of the
  more interesting design patterns in ROS 2. Every Nav2 component uses it.
- A working `map → odom` TF from AMCL is the prerequisite for Phase 3 (Nav2
  path planning) — the planner needs to know where the robot is in the map.

**What changes vs Phase 1.**
- The host Docker **slam container is no longer needed**. SLAM is done.
- AMCL runs on the **Jetson** (not the host) — it's ~10× lighter than
  slam_toolbox mapping mode. The 8 GB Jetson handles it fine alongside bringup
  and lidar.
- The host runs **RViz only** (same Humble Docker `rviz` container as Phase 1).
- The map is **static** — `map_server` loads it once and never changes it.
  The robot's live sensor data goes into Nav2's costmap layers (Phase 3), not
  the static map.

**Scope.** One room, `living_room_v1`. Single initial pose set by hand in
RViz. No automatic relocalization, no multi-map. Those are Phase 7 items.

---

## 2. Acceptance criteria

Phase 2 is done when ALL of these are true:

- [x] `map_server`, `amcl`, and `lifecycle_manager` start cleanly on the
      Jetson with no `WARN`/`ERROR` about missing topics or TF.
- [x] After clicking "2D Pose Estimate" in RViz at the robot's actual
      position, the `/particle_cloud` (PoseArray) appears as a spread of
      arrows centered on the estimate.
- [x] After driving 2–3 m the particle cloud converges to a tight cluster
      (≤ ~30 cm spread) and **stays** tight during further driving.
- [x] `ros2 run tf2_ros tf2_echo map odom` (on Jetson) shows a stable,
      sensible transform — not wildly drifting.
- [x] `ros2 run tf2_tools view_frames` (on Jetson) shows the complete chain
      `map → odom → base_footprint → base_link → laser_link`.
- [x] The robot can be driven to several points in the room and AMCL keeps
      up — no "lost" state (particle cloud spreading back out).

Stretch (nice-to-have):
- AMCL auto-recovers if you pick it up and put it down elsewhere (kidnapped
  robot problem). This requires `recovery_alpha_fast` > 0.
- Save the initial pose to the params file so the next session starts near
  the correct location without a manual "2D Pose Estimate" click.

---

## 3. What we need to create

Two new files in `cat_patrol_robot`, and one convenience script:

| File | Purpose |
|---|---|
| `config/amcl_params.yaml` | AMCL + map_server + lifecycle_manager params tuned for this robot |
| `launch/localization.launch.py` | Brings up map_server + amcl + lifecycle_manager in one command |
| `~/myscripts2/t3_amcl.sh` | Convenience script (domain-28 + cyclonedds env, mirrors t1/t2/t2.5) |

These are created in §4 below. The CMakeLists already installs `config/` and
`launch/` into the ament share directory, so no CMakeLists changes needed.

---

## 4. Files to create (do this before the first session)

### 4a. `config/amcl_params.yaml`

Key differences from the existing `yahboomcar_nav/params/dwa_nav_params.yaml`
AMCL section:

| Param | Old value | Our value | Reason |
|---|---|---|---|
| `scan_topic` | `/scan` (default) | `/scan_filtered` | Only the filtered forward arc is trustworthy on this robot |
| `robot_model_type` | `DifferentialMotionModel` | `OmnidirectionalMotionModel` | X3 is mecanum — can strafe; diff model under-represents possible motions |
| `laser_min_range` | `-1.0` | `0.15` | Matches our SLAM filter; avoids chassis near-field hits |
| `laser_max_range` | `100.0` | `8.0` | RPLiDAR A1 reliable range; avoid bogus long returns |

Create this file:

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/config/amcl_params.yaml << 'EOF'
amcl:
  ros__parameters:
    use_sim_time: false

    # --- Motion model ---
    # OmnidirectionalMotionModel because X3 is mecanum (can strafe).
    robot_model_type: "nav2_amcl::OmnidirectionalMotionModel"
    # alpha1-5: noise coefficients (rotation-from-rotation, rotation-from-translation,
    # translation-from-translation, translation-from-rotation, strafe-from-translation).
    # Start conservative (0.2); tighten if the cloud is too spread after a straight drive.
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2

    # --- Laser model ---
    scan_topic: /scan_filtered
    laser_model_type: "likelihood_field"
    laser_min_range: 0.15
    laser_max_range: 8.0
    max_beams: 60
    laser_likelihood_max_dist: 2.0

    # Beam weights (must sum to 1.0).
    # z_hit: fraction explained by expected map wall.
    # z_rand: random / unexplained returns (cats, glass, virtual walls we painted).
    # z_max: beam went beyond max range (sensor missed).
    # z_short: shorter than expected (moving obstacle in the way).
    # Raise z_rand slightly vs defaults because our painted virtual walls
    # produce real "through-wall" laser returns that don't match the map.
    z_hit: 0.5
    z_rand: 0.4
    z_max: 0.05
    z_short: 0.05
    sigma_hit: 0.2
    lambda_short: 0.1

    # --- Particle filter ---
    min_particles: 500
    max_particles: 2000
    pf_err: 0.05
    pf_z: 0.99
    resample_interval: 1

    # --- Update thresholds ---
    # Only update the filter after the robot has moved this much.
    update_min_d: 0.25     # metres
    update_min_a: 0.2      # radians (~11°)

    # --- Frame ids (must match our TF tree) ---
    global_frame_id: "map"
    odom_frame_id: "odom"
    base_frame_id: "base_footprint"
    tf_broadcast: true     # AMCL publishes map → odom

    # --- Recovery ---
    # Both 0.0 = no random particle injection (safe start).
    # Set recovery_alpha_fast: 0.1 if the robot gets "lost" too easily.
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0

    # --- Beam skip (filter out moving obstacles in scan) ---
    do_beamskip: false
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3

    transform_tolerance: 1.0
    save_pose_rate: 0.5

map_server:
  ros__parameters:
    use_sim_time: false
    yaml_filename: "/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/maps/living_room_v1.yaml"
    topic_name: "map"
    frame_id: "map"

lifecycle_manager_localization:
  ros__parameters:
    use_sim_time: false
    autostart: true
    node_names: ["map_server", "amcl"]
EOF
```

### 4b. `launch/localization.launch.py`

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/launch/localization.launch.py << 'EOF'
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/config/amcl_params.yaml',
        description='Full path to AMCL/map_server params YAML',
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
    ])
EOF
```

### 4c. `~/myscripts2/t3_amcl.sh`

```bash
cat > ~/myscripts2/t3_amcl.sh << 'EOF'
#!/usr/bin/env bash
export ROS_DOMAIN_ID=28
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds.xml
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch cat_patrol_robot localization.launch.py
EOF
chmod +x ~/myscripts2/t3_amcl.sh
```

### 4d. Build

After creating the files above:

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
colcon build --symlink-install --packages-select cat_patrol_robot
source install/setup.bash
```

---

## 5. Architecture

```
Jetson (Humble)                          Host peter-pen (Humble Docker)
────────────────────────────────         ────────────────────────────────
t1.sh   yahboomcar_bringup              humble-docker-rviz-1
        ├── base_node (MCU)                 rviz2
        ├── imu_filter                         Map       /map  [Transient Local]
        ├── ekf_filter  → /odom               PoseArray /particle_cloud
        ├── robot_state_publisher             LaserScan /scan_filtered
        └── yahboom_joy_X3                    Pose      /amcl_pose
                                              TF        (enable for debugging)
t2.sh   sllidar_ros2  → /scan
                                         (slam Docker NOT needed in Phase 2)
t2.5.sh scan_front_filter
        /scan → /scan_filtered

t3_amcl.sh   localization.launch.py
        ├── map_server   → /map [Transient Local]
        ├── amcl         → /particle_cloud, /amcl_pose
        │                  map → odom TF
        └── lifecycle_manager (autostart: configures + activates both)

TF chain (same structure as Phase 1, different publisher for map→odom):
  map → odom          AMCL (was slam_toolbox in Phase 1)
  odom → base_footprint   EKF (same as Phase 1)
  base_footprint → ...    robot_state_publisher (same as Phase 1)
```

---

## 6. Pre-flight checklist

### 6a. One-time (before first session)

- [ ] Create the three files in §4 and run the colcon build.
- [ ] Verify build: `ros2 pkg executables cat_patrol_robot | grep -v patrol_node`
      should show nothing new (localization.launch.py is a launch file, not
      an executable, but build must succeed cleanly).
- [ ] Verify launch file is reachable:
      `ros2 launch cat_patrol_robot localization.launch.py --show-args`
- [ ] Stop the host slam Docker container (no longer needed):
      On host: `cd ~/humble-docker && sudo docker compose stop slam`
- [ ] Ensure host RViz Docker is available:
      On host: `cd ~/humble-docker && sudo docker compose up -d rviz`
      (or `docker compose start rviz` if already created).

### 6b. Every session

- [ ] Robot at the tape mark (starting position matters — you'll click the
      initial pose in RViz to match it).
- [ ] Battery > 11.5 V: `ros2 topic echo /voltage --once`
- [ ] Reboot Jetson if swap > 512 MiB: `free -h`
- [ ] `sudo systemctl stop ollama` (optional, frees ~100 MB)
- [ ] Verify USB ports (chassis = CH340 7523, lidar = CP2102 by-id symlink).

---

## 7. Session procedure

### Step A — Jetson terminals (in order)

**Every terminal — paste first:**
```bash
export ROS_DOMAIN_ID=28
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds.xml
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
```

Or use the scripts in `~/myscripts2/`:

**T1 — bringup:**
```bash
bash ~/myscripts2/t1.sh
```
Wait for: `cmd_vel trim: trim_vy_per_vx=0.0120`

**T2 — lidar:**
```bash
bash ~/myscripts2/t2.sh
```
Wait for: `SLLidar health status : OK`

**T2b — scan filter:**
```bash
bash ~/myscripts2/t2.5.sh
```
Wait for: `Keeping scan angles` log line.

**T3 — localization (new for Phase 2):**
```bash
bash ~/myscripts2/t3_amcl.sh
```

Success signs from T3:
- `[lifecycle_manager_localization] Configuring map_server`
- `[lifecycle_manager_localization] Configuring amcl`
- `[lifecycle_manager_localization] Activating map_server`
- `[lifecycle_manager_localization] Activating amcl`
- `[amcl] Received a 290 X 141 map` (or similar size)
- `[lifecycle_manager_localization] Managed nodes are active`

**No `exit code -9`** — if AMCL OOMs (unlikely, it's light), see §8.

Check memory after T3 is active:
```bash
free -h
# expect ≥ 3 GiB available (AMCL is much lighter than slam_toolbox)
```

### Step B — Host RViz

On **host** (after T3 is stable):

```bash
export ROS_DOMAIN_ID=28
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/bots/cyclonedds.xml
export ROS_LOCALHOST_ONLY=0
# inside the rviz Docker container, or directly if Jazzy RViz is OK for display:
sudo docker exec -it humble-docker-rviz-1 bash
# then inside:
export DISPLAY=:0
ros2 run rviz2 rviz2
```

**RViz displays for Phase 2:**

| Display | Topic | Notes |
|---|---|---|
| Fixed Frame | `map` | |
| Map | `/map` | **Durability: Transient Local** |
| LaserScan | `/scan_filtered` | |
| PoseArray | `/particle_cloud` | The AMCL particle cloud — set color to distinctive |
| Pose (arrow) | `/amcl_pose` | The best-estimate pose |
| TF | optional | Enable only to debug; costs Jetson RAM |

### Step C — Set initial pose

1. In RViz click **"2D Pose Estimate"** (toolbar at top).
2. Click on the map at the robot's **actual physical location** (where it is
   on the tape mark).
3. Drag the arrow to set the heading (which way the robot faces).
4. Release — you should see a spread of arrows (the particle cloud) appear
   centered on your estimate.

If no particle cloud appears:
- Check `/particle_cloud` topic exists: `ros2 topic list | grep particle`
- Check AMCL is active: `ros2 lifecycle get /amcl`

### Step D — Drive and verify convergence

- Drive slowly using the joystick (~0.10 m/s).
- After 1–2 m of driving, the particle cloud should tighten significantly.
- After a turn, it tightens further (rotation is very informative for AMCL).
- **Convergence test:** drive a small loop and return to start. The `/amcl_pose`
  arrow should be close to where you actually are.

### Step E — Verify TF chain

On Jetson (new terminal, sourced):
```bash
ros2 run tf2_ros tf2_echo map odom
# expect: steady transform, not wildly jumping

ros2 run tf2_tools view_frames
# produces /tmp/frames_XXXX.pdf — copy to host to view
```

---

## 8. Pitfalls and recovery

| Symptom | Cause | Fix |
|---|---|---|
| `[lifecycle_manager] Failed to configure map_server` | yaml_filename wrong or map file missing | Check path: `ls /home/jetson/yahboomcar_ros2_ws/.../maps/living_room_v1.*`; verify absolute path in amcl_params.yaml |
| `[amcl] Received map with 0 cells` | Map loaded but empty; yaml origin/size issue | Check living_room_v1.yaml has the absolute image path (not relative) |
| `[amcl] WARN: Failed to transform laser scan` | TF not ready; bringup not up, or AMCL started before bringup | Restart T3 after T1/T2/T2b are all healthy |
| No `/particle_cloud` topic in RViz | AMCL not active (lifecycle not reached Active state) | Check T3 for `Managed nodes are active`; `ros2 lifecycle get /amcl` |
| Particle cloud doesn't converge after driving | Wrong initial pose; wrong motion model; scan_topic still `/scan` not `/scan_filtered` | Re-click 2D Pose Estimate at the correct location; verify `scan_topic: /scan_filtered` in params |
| Particle cloud diverges / robot gets "lost" | Driving too fast; localization can't keep up; or hit a dynamic obstacle | Slow down; stop; re-click 2D Pose Estimate |
| `map → odom` TF jumpy | AMCL struggling; too many dynamic obstacles in scan | Reduce `max_beams` to 30; enable `do_beamskip: true` |
| AMCL exits -9 | OOM (unlikely but possible with large particle counts) | Reduce `max_particles` to 1000; `sudo systemctl stop ollama` |
| `[amcl] WARN: Cannot call set_parameters on node` | lifecycle not yet in configure state | Wait 5 s; check lifecycle_manager log for errors |
| `/map` in RViz shows "no map received" | Durability not Transient Local; or map_server not active | Set Map display Durability to **Transient Local**; check T3 log |
| Robot model in wrong position in RViz | Wrong initial pose estimate | Re-click 2D Pose Estimate |
| Virtual walls (painted in GIMP) cause AMCL to drift | Laser sees through painted walls → beam weights wrong | Slightly raise `z_rand` (to 0.5) and lower `z_hit` (to 0.4); painted walls are a small fraction of the scan |

---

## 9. Concept anchors (lifecycle nodes — the Phase 2 learning goal)

From plan.md: *"This is your first encounter with lifecycle nodes — nav2_amcl is
a managed node, and Nav2 itself is built on the same pattern."*

The pattern:

```
Unconfigured  →  Configured  →  Active  →  Deactivated  →  Finalized
     ↑                ↑             ↑
  on_configure()  on_activate()  on_deactivate()
```

- `on_configure()`: allocate resources, load params, subscribe to topics — but
  don't start publishing yet.
- `on_activate()`: start publishing, start timers. The node is now "running."
- `on_deactivate()`: stop publishing but keep resources — can re-activate fast.
- `on_cleanup()`: free resources, return to Unconfigured.

`lifecycle_manager` with `autostart: true` drives this automatically at
startup: it calls configure then activate on each node in `node_names` in
order. That's why you see the sequence in T3's log.

**Where to read the code:**
- `rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_node.hpp` — the base
  class interface. Look at the virtual `on_*` signatures.
- `nav2_amcl/src/amcl_node.cpp` — a real production lifecycle node. Find the
  `on_configure`, `on_activate`, `on_deactivate` methods and read what they
  do at each state.
- `nav2_lifecycle_manager/src/lifecycle_manager.cpp` — the manager itself.
  How it issues `change_state` service calls to each managed node.

---

## 10. Commands cheat-sheet

**Every Jetson terminal:**
```bash
export ROS_DOMAIN_ID=28
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds.xml
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
```

**Start stack (Jetson):**
```bash
bash ~/myscripts2/t1.sh       # T1: bringup
bash ~/myscripts2/t2.sh       # T2: lidar
bash ~/myscripts2/t2.5.sh     # T2b: scan filter
bash ~/myscripts2/t3_amcl.sh  # T3: localization
```

**Verify AMCL is active:**
```bash
ros2 lifecycle get /amcl
ros2 lifecycle get /map_server
```

**Check TF chain:**
```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_tools view_frames
```

**Inspect topics:**
```bash
ros2 topic hz /particle_cloud
ros2 topic echo /amcl_pose --once
ros2 topic hz /scan_filtered
```

**Manually trigger AMCL update (if it seems stuck):**
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}'
```
(Better: use RViz "2D Pose Estimate" button instead.)

**If AMCL loses localization — reset without restarting:**
Click "2D Pose Estimate" in RViz again at the robot's current position.

**Stop the slam Docker on host (one-time):**
```bash
cd ~/humble-docker && sudo docker compose stop slam
```

---

## 11. Phase 2 session log

### Session 1 — _date_

- Stack versions / build SHA: _(colcon build completed on?)_
- Battery at start / end:
- Particle cloud appeared after pose estimate: yes/no
- Convergence: tight after ___ m of driving
- `map → odom` TF: stable / jumpy
- Anything unusual:
- Outcome:

### Session 2 — _date_

_(copy template above)_

### Final results

- AMCL params that worked: alpha1-5 = ___, max_particles = ___
- Convergence time from cold estimate: ___ seconds / ___ metres driven
- Localization stable during full room drive: yes/no
- Phase 2 acceptance criteria: all met / blockers: ___

---

## 12. Open questions / forward links

- **Phase 3 (Nav2 full stack).** Phase 2's `map_server` + `amcl` +
  `lifecycle_manager` become the localization half of Phase 3. Phase 3 adds
  the planner, controller, costmaps, behavior server, and BT navigator — all
  as additional lifecycle nodes managed by the same (or an extended)
  `lifecycle_manager`.
- **AMCL vs slam_toolbox localization mode.** Phase 1 notes mention
  slam_toolbox has a localization-only mode that can use the `.posegraph`
  files saved alongside the `.pgm`. We chose nav2_amcl per the plan — but
  slam_toolbox localization is worth knowing about as an alternative.
- **Initial pose automation.** After Phase 2 is tuned, `set_initial_pose: true`
  + `initial_pose: {x, y, yaw}` in the params file can be used to seed AMCL
  at the tape-mark position automatically. Useful once the robot always starts
  at the same spot.
- **Kidnapped robot.** If `recovery_alpha_fast: 0.1` is set, AMCL injects
  random particles when the filter's total weight drops below a threshold,
  allowing recovery from being picked up and moved. Useful for a cat-patrol
  robot that might get nudged by a cat.

---

## 13. Learning Guide — Phase 2 deep dive

This section is the study reference for the next day. It covers every component,
every topic, every RViz panel, and how to verify them from the command line.

---

### 13a. Full topic data-flow diagram

```
HARDWARE / DISK
───────────────────────────────────────────────────────────────────────────────
disk: living_room_v1.pgm + .yaml
         │
    [map_server]
         │ publishes (once, Transient Local QoS)
         ▼
    /map   (nav_msgs/OccupancyGrid)
         │
         ├──► RViz: Map display
         └──► amcl  (map input — static background)

/dev/ttyUSB0 (RPLiDAR A1, CP2102)
         │
    [sllidar_ros2]
         │ publishes ~10 Hz
         ▼
    /scan   (sensor_msgs/LaserScan, 360° full ring)
         │
    [scan_front_filter]   ← keeps only ~220° front wedge, trusted_center=π
         │ publishes ~10 Hz
         ▼
    /scan_filtered   (sensor_msgs/LaserScan, ~220° arc)
         │
         ├──► RViz: LaserScan display
         └──► amcl  (sensor update input)

/dev/ttyUSB1 (Yahboom MCU, CH340)
         │
    [base_node]   → /cmd_vel (subscribed), wheel encoder ticks, IMU raw
         │
    [imu_filter_madgwick]   → /imu/data  (filtered orientation)
         │
    [robot_localization EKF]
         │ fuses /imu/data + wheel odometry
         ▼
    /odom   (nav_msgs/Odometry)
    TF: odom → base_footprint   (published by EKF)

    [robot_state_publisher]
         │ reads URDF + /joint_states
         ▼
    TF: base_footprint → base_link → laser_link → camera_link → ...

USER (RViz "2D Pose Estimate" button)
         │
         ▼
    /initialpose   (geometry_msgs/PoseWithCovarianceStamped)
         │
         └──► amcl  (seeds the particle cloud)

AMCL  (reads /map, /scan_filtered, /initialpose, TF odom→base_footprint)
         │
         ├──► /particle_cloud   (geometry_msgs/PoseArray, ~2 Hz)
         │         └──► RViz: PoseArray display
         │
         ├──► /amcl_pose   (geometry_msgs/PoseWithCovarianceStamped, ~2 Hz)
         │         └──► RViz: Pose display
         │
         └──► TF: map → odom   (published continuously)
                      └──► completes the chain: map→odom→base_footprint→base_link→laser_link
```

---

### 13b. Component reference — what each node does, inputs, outputs

#### `map_server`  (package: `nav2_map_server`)

| | |
|---|---|
| **What it does** | Reads a `.pgm` image + `.yaml` metadata from disk, converts to `OccupancyGrid`, and latches it on `/map`. After that it does nothing — the map is static. |
| **Input** | `yaml_filename` in params → `.pgm` grayscale image on disk. White pixels = free (0), black = occupied (100), grey = unknown (-1). |
| **Output** | `/map` topic (Transient Local durability — subscribers joining late still get the map) |
| **Key params** | `yaml_filename`, `frame_id: "map"`, `topic_name: "map"` |
| **Lifecycle** | Loads map on `on_configure()`; starts publishing on `on_activate()`. Until activated, `/map` is silent. |

#### `nav2_amcl`  (package: `nav2_amcl`)

| | |
|---|---|
| **What it does** | Adaptive Monte-Carlo Localization. Maintains N particles, each = one hypothesis `(x, y, θ)` of where the robot is in the map. Uses the laser scan to score hypotheses, resamples, and converges to the true pose. |
| **Inputs** | `/scan_filtered` (LaserScan) — sensor evidence; `/map` (OccupancyGrid) — map to compare scan against; `/initialpose` — user-supplied seed; TF `odom → base_footprint` — how much the robot has moved since last update |
| **Outputs** | `/particle_cloud` (PoseArray) — all N particles; `/amcl_pose` (PoseWithCovarianceStamped) — weighted mean + covariance; TF `map → odom` — correction that makes `map → base_footprint` match reality |
| **Key params** | `robot_model_type`, `scan_topic`, `min/max_particles`, `alpha1-5`, `z_hit/z_rand/z_max/z_short`, `update_min_d`, `update_min_a` |
| **Lifecycle** | Subscribes to topics on `on_configure()`; starts publishing TF and topics on `on_activate()`. Nothing is published until activated. |

#### `lifecycle_manager`  (package: `nav2_lifecycle_manager`)

| | |
|---|---|
| **What it does** | Drives the lifecycle state machine of managed nodes in order. With `autostart: true` it calls `configure` then `activate` on `map_server` then `amcl` at startup. Also provides a health-check watchdog. |
| **Inputs** | None (no subscribed topics). Issues ROS2 lifecycle service calls (`/map_server/change_state`, `/amcl/change_state`). |
| **Outputs** | None (no published topics). Only indirectly causes `map_server` and `amcl` to reach Active state. |
| **Key params** | `node_names: ["map_server", "amcl"]`, `autostart: true` |

#### `scan_front_filter`  (custom node, `cat_patrol_robot`)

| | |
|---|---|
| **What it does** | Crops the full 360° lidar ring to a trusted arc (~220° centred on the robot's front). Drops rear readings behind the antenna mast (which caused SLAM collapse in Phase 1). |
| **Input** | `/scan` (sensor_msgs/LaserScan, 360°) |
| **Output** | `/scan_filtered` (sensor_msgs/LaserScan, ~220° arc) — `angle_min`/`angle_max` match the trusted arc; out-of-arc beams set to `NaN` or `inf`. |

#### `robot_localization EKF`  (package: `robot_localization`)

| | |
|---|---|
| **What it does** | Extended Kalman Filter that fuses wheel odometry + IMU to produce a smooth, drift-corrected odometry estimate. Publishes the `odom → base_footprint` TF that AMCL reads. |
| **Inputs** | `/imu/data` (filtered IMU), raw wheel velocity or `/odom_raw` from base_node |
| **Outputs** | `/odom` (nav_msgs/Odometry), TF `odom → base_footprint` |
| **Note for this robot** | Wheel encoders give ~0° rotation accuracy for a 90° turn (Phase 1 finding). EKF is tuned to trust IMU heading, not wheel yaw. |

#### `robot_state_publisher`  (package: `robot_state_publisher`)

| | |
|---|---|
| **What it does** | Reads the robot URDF and `/joint_states`, publishes all fixed and movable joint transforms as TF. Provides the `base_footprint → ... → laser_link` portion of the chain. |
| **Input** | URDF (loaded at startup), `/joint_states` topic |
| **Output** | TF subtree: `base_footprint → base_link → laser_link`, plus camera and wheel frames |

---

### 13c. The TF chain — why every link matters

```
map ──(AMCL)──► odom ──(EKF)──► base_footprint ──(RSP)──► base_link ──(RSP)──► laser_link
```

| Link | Publisher | Meaning |
|---|---|---|
| `map → odom` | AMCL | Correction offset: how far the raw odometry frame has drifted from the true map |
| `odom → base_footprint` | EKF (robot_localization) | Dead-reckoning estimate of robot motion since startup; accumulates drift over time |
| `base_footprint → base_link` | robot_state_publisher (URDF) | Lift from floor plane to robot body centre |
| `base_link → laser_link` | robot_state_publisher (URDF) | Where the lidar is mounted relative to the body |

**Why `base_footprint` and not `base_link`?**
Nav2 and AMCL use `base_footprint` as the robot's "floor projection" frame. It is the same x,y as `base_link` but always at z=0 (no roll/pitch). Planning and cost maps operate in 2D, so this convention avoids numerical issues from IMU-induced roll/pitch jitter.

**What AMCL actually corrects:**
AMCL does not change `odom → base_footprint` (the EKF owns that). Instead it adjusts `map → odom` so that the full chain `map → odom → base_footprint` correctly describes the robot's true position in the map. If the EKF accumulates 10 cm of drift, AMCL's `map → odom` offset absorbs it.

---

### 13d. AMCL algorithm — particle filter step by step

Understanding this helps diagnose convergence problems.

**1. Initialization (after `/initialpose` received)**
- Spawn `min_particles` to `max_particles` particles as a Gaussian blob centred on the given pose with covariance from the message. Large initial covariance = wide spread = robot could be anywhere nearby. Small = tight initial guess.

**2. Motion update (every time robot moves)**
- AMCL reads the TF change `odom → base_footprint` since the last update (triggered when robot has moved `update_min_d` metres or rotated `update_min_a` radians).
- Each particle is propagated forward by the same Δx, Δy, Δθ **plus noise** sampled from the motion model.
- Noise parameters `alpha1`–`alpha5` control how much each type of motion smears the particles:
  - `alpha1`: rotation noise from rotation (gyro drift)
  - `alpha2`: rotation noise from translation (slip during straight drive)
  - `alpha3`: translation noise from translation (wheel slip)
  - `alpha4`: translation noise from rotation (pivot slip)
  - `alpha5`: strafe noise from translation (mecanum slip)
- After this step the cloud is wider — uncertainty grew because we moved.

**3. Sensor update (every scan arriving at `/scan_filtered`)**
- For each particle, score the current laser scan from that hypothetical pose using the **likelihood field model**:
  - For each beam endpoint, look up the distance to the nearest occupied cell in the pre-computed map (not ray-casting — faster).
  - Weight = product over beams of `z_hit * Gaussian(dist, 0, sigma_hit) + z_rand * (1/max_range)`.
  - High weight = scan matches the map well from that pose.
- Particles near walls that the laser actually sees get high weight; particles in open space (where the laser would pass through walls) get low weight.

**4. Resampling**
- Draw `max_particles` particles with replacement, proportional to weight (stochastic universal sampling in practice).
- High-weight particles are duplicated; zero-weight particles die.
- After resampling, the cloud clusters at poses that explain the sensor data.
- `resample_interval: 1` means resample after every update (default). Higher values reduce noise at the cost of slower convergence.

**5. Adaptive particle count**
- `pf_err` and `pf_z` control the KLD-sampling algorithm: the particle count is reduced when the distribution is tight (low entropy) and increased when it is spread (high entropy). This is why you see the count drop from `max_particles` to near `min_particles` after convergence.

**Key insight:** rotation is much more informative than translation. A 90° turn with a distinctive room corner visible will snap the cloud tightly. Driving straight in a long corridor (perceptual aliasing) may not converge — the robot looks the same at every point along the hall.

---

### 13e. Beam weight parameters — practical guide

Our values (`z_hit=0.5, z_rand=0.4, z_max=0.05, z_short=0.05`) differ from Nav2 defaults because of the **painted virtual walls** in the map (GIMP-drawn boundaries that do not exist as physical walls). The real laser passes through them and returns farther hits — those are "random" from AMCL's perspective.

| Param | Nav2 default | Our value | Effect of increasing |
|---|---|---|---|
| `z_hit` | 0.5 | 0.5 | More weight on beams matching map walls. Tighter convergence when room matches map well. Brittle if map is imperfect. |
| `z_rand` | 0.5 | 0.4 | Tolerance for unexplained returns (glass, cats, painted walls). Too high = slow convergence; too low = crashes on clutter. |
| `z_max` | 0.05 | 0.05 | Tolerance for beams that exceeded max range (missed returns). |
| `z_short` | 0.05 | 0.05 | Tolerance for beams shorter than expected (moving obstacle in front). Increase if there are moving people/cats. |
| `sigma_hit` | 0.2 | 0.2 | Width of the Gaussian around a map-matching beam. Wider = more forgiving of map inaccuracies. |

**Must sum to 1.0.** If you change one, adjust another.

---

### 13f. RViz displays — what to look for in each panel

#### Fixed Frame = `map`
All displays are rendered in the `map` coordinate frame. If this is wrong (e.g. `odom`) everything drifts as the robot moves, because `odom` accumulates drift relative to `map`.

---

#### Map display — topic `/map`
- **What you see:** Grey-scale occupancy grid. White = free space (value 0). Black = walls and obstacles (value 100). Grey = unknown (value -1, typically transparent).
- **What to check:** The room outline should be recognizable. Virtual painted walls appear as thin black lines cutting across open space.
- **Common problem:** Map shows "no map received." Cause: Map display Durability set to **Volatile** instead of **Transient Local**. `map_server` publishes once at startup; a volatile subscriber that connects later misses it.
- **Expected rate:** `/map` publishes once (or very rarely — only if the map changes, which it does not in Phase 2).

---

#### PoseArray display — topic `/particle_cloud`
- **What you see:** A field of small arrows on the map, each representing one particle = one hypothesis of where the robot could be.
- **At startup (before pose estimate):** Particles spread uniformly across the entire map (global localization mode, if enabled) or absent (waiting for `/initialpose`).
- **Right after "2D Pose Estimate":** Dense cluster of arrows centred at the clicked pose, spread in a Gaussian blob. The spread reflects your uncertainty — wide if you are unsure.
- **During driving:** The blob tightens as laser scan matches the map. After a turn near distinctive walls it should collapse to a tight cluster (< 30 cm radius).
- **Well-converged:** The arrows form a tight rosette, all pointing the same direction. You can barely see individual arrows — they overlap.
- **Lost/diverged:** Arrows spread back out, often bimodally (two candidate locations). This means the sensor update stopped resolving ambiguity (perceptual aliasing or dynamic obstacles overwhelming the map).

---

#### Pose display — topic `/amcl_pose`
- **What you see:** A single large arrow showing AMCL's weighted-mean best estimate of the robot's pose. This is what downstream Nav2 components (planner, controller) will use.
- **What to check:** The arrow should match where the physical robot actually is in the room. Drive to a corner, stop, compare the arrow's position to the map corner.
- **Note:** This jumps when AMCL does a large correction (common right after setting initial pose). During normal driving it updates smoothly at ~2 Hz.

---

#### LaserScan display — topic `/scan_filtered`
- **What you see:** A ring of coloured dots showing the current laser returns in the robot's laser frame, projected onto the map. Points land where the beam hit an obstacle.
- **What to check for localization quality:** When AMCL is well-converged, the laser dots should **land on the black occupied cells** in the Map display. If they are offset (dots 30 cm off from walls) the localization is wrong or the TF chain has a problem.
- **Expected arc:** Only ~220° front of the robot — the rear wedge (behind the antenna mast) is filtered. Spinning the robot shows which half is missing.
- **Rate:** ~10 Hz (same as lidar).

---

#### TF display (optional, enable for debugging)
- **What you see:** Coloured axis tripods at each TF frame origin. Red=X, Green=Y, Blue=Z.
- **What to look for:** `map` frame should be anchored stationary. `odom` frame drifts slowly relative to `map` (AMCL corrects it). `base_footprint` moves with the robot.
- **Cost:** TF rendering is expensive in RViz. Enable it only when debugging, then disable it to save CPU/RAM on the Jetson.

---

### 13g. Lifecycle node deep dive

Nav2 wraps all its nodes in the ROS 2 **Managed Node** (lifecycle node) pattern. Understanding it is required for Phase 3, which adds six more lifecycle nodes.

```
           ┌──────────────────────────────────────────────────────────────────┐
           │                   Lifecycle State Machine                        │
           │                                                                  │
           │  [create]                                                        │
           │     │                                                            │
           │     ▼                                                            │
           │  Unconfigured ──on_configure()──► Inactive ──on_activate()──► Active
           │       ▲                              │                          │
           │       │                   on_cleanup()│             on_deactivate()│
           │       └──────────────────────────────┘                          │
           │                                                            on_deactivate()
           │                                               Inactive ◄────────┘
           │                                                  │
           │                                        on_cleanup() or on_shutdown()
           │                                                  │
           │                                             Finalized
           └──────────────────────────────────────────────────────────────────┘
```

**State meanings:**

| State | Resources allocated? | Publishing? |
|---|---|---|
| Unconfigured | No | No |
| Inactive (Configured) | Yes (params loaded, TF/topics subscribed) | No |
| Active | Yes | **Yes** |
| Finalized | No | No |

**Why two states before Active?**
`on_configure()` sets up all subscriptions and parameters — expensive work done once. `on_activate()` is deliberately cheap (flip a flag, start timers). This lets you reconfigure a node without destroying it and rebuilding from scratch.

**The `lifecycle_manager` sequence (what you see in T3 log):**
1. Sends `configure` to `map_server` → `[map_server] Configuring...` → map loaded into memory.
2. Sends `activate` to `map_server` → `[map_server] Activating...` → `/map` starts publishing.
3. Sends `configure` to `amcl` → `[amcl] Configuring...` → allocates particle array, subscribes to `/scan_filtered`.
4. Sends `activate` to `amcl` → `[amcl] Activating...` → starts publishing `/particle_cloud`, `/amcl_pose`, TF `map→odom`.
5. All nodes active → `Managed nodes are active`.

**The map must be published (step 2) before AMCL activates (step 4)** because AMCL reads the map during activation. The `node_names` order in the params file matters.

**Checking lifecycle state from CLI:**
```bash
ros2 lifecycle get /amcl
# → active
ros2 lifecycle get /map_server
# → active
```

**Reading the source:**
- `nav2_amcl/src/amcl_node.cpp` — find `on_configure`, `on_activate`, `on_deactivate`.
  - `on_configure`: loads params, creates particle filter, subscribes to `/map` and `/scan_filtered`.
  - `on_activate`: starts `bond` to lifecycle_manager, activates pose publishers, starts TF broadcast.
- `nav2_lifecycle_manager/src/lifecycle_manager.cpp` — find `manageLifecycleNodes()`.
  - Issues `change_state` service calls in sequence; checks return codes.
  - Has a bond/heartbeat mechanism: if a managed node crashes, lifecycle_manager detects it and can bring the whole stack down cleanly.

---

### 13h. QoS — why `/map` needs Transient Local

ROS 2 uses DDS quality-of-service policies. Two that matter here:

| Policy | Volatile (default) | Transient Local |
|---|---|---|
| **What happens to old messages** | Discarded as soon as sent | Publisher keeps them in memory |
| **Late subscriber** | Gets nothing (missed the message) | Gets the buffered message immediately |
| **Use case** | Sensor streams (laser, IMU) — always current | `/map` — published once, subscribers join later |

`map_server` uses **Transient Local** because it publishes `/map` once at activation. RViz and AMCL may connect seconds later. With Volatile, they would never receive the map. With Transient Local, they immediately get the latched copy.

**Critical:** Publisher and subscriber QoS must be compatible. In RViz:
- Map display → set **Durability** to **Transient Local**.
- If left at Volatile, the display shows "no map received" even though `map_server` is running fine.

---

### 13i. Verify every topic from the command line

Run these on the Jetson (with domain ID + source) to confirm the stack is healthy:

```bash
# ── Map ──────────────────────────────────────────────────────────────────────
ros2 topic info /map --verbose
# Expect: Type nav_msgs/msg/OccupancyGrid
#         Publisher count: 1  (map_server)
#         QoS: Durability TRANSIENT_LOCAL

ros2 topic echo /map --once | head -20
# Expect: header.frame_id: map, info.width: NNN, info.height: NNN

# ── AMCL outputs ─────────────────────────────────────────────────────────────
ros2 topic hz /particle_cloud
# Expect: ~2 Hz when robot is moving or AMCL is updating

ros2 topic echo /amcl_pose --once
# Expect: header.frame_id: map, pose.pose.position.x/y: current position

ros2 topic info /particle_cloud
# Expect: Type geometry_msgs/msg/PoseArray, Publisher count: 1

# ── Scan pipeline ─────────────────────────────────────────────────────────────
ros2 topic hz /scan
# Expect: ~10 Hz (lidar running)

ros2 topic hz /scan_filtered
# Expect: ~10 Hz (filter running, same rate as input)

ros2 topic echo /scan_filtered --once | grep -E "angle_min|angle_max|range_max"
# Expect: angle_min close to -1.92 rad, angle_max close to +1.92 rad (i.e. ~220°)

# ── Odometry / EKF ───────────────────────────────────────────────────────────
ros2 topic hz /odom
# Expect: ~50 Hz (EKF output)

ros2 topic echo /odom --once | grep -A5 pose
# Expect: position x/y/z near 0 at startup (EKF resets on launch)

# ── Lifecycle states ──────────────────────────────────────────────────────────
ros2 lifecycle get /amcl
# → active
ros2 lifecycle get /map_server
# → active
ros2 lifecycle get /lifecycle_manager_localization
# → active

# ── TF chain ─────────────────────────────────────────────────────────────────
ros2 run tf2_ros tf2_echo map odom
# Expect: steady translation and rotation; should NOT be "No common time found"

ros2 run tf2_ros tf2_echo map base_footprint
# Expect: the robot's estimated global pose (same as /amcl_pose essentially)

ros2 run tf2_ros tf2_echo odom base_footprint
# Expect: EKF odometry pose; this should grow as robot drives

ros2 run tf2_tools view_frames
# Produces /tmp/frames_XXXX.pdf — copy to host: scp jetson@192.168.0.120:/tmp/frames_*.pdf .
# Must show: map → odom → base_footprint → base_link → laser_link

# ── Topic list sanity check ───────────────────────────────────────────────────
ros2 topic list | sort
# Phase 2 minimum required topics:
#   /amcl_pose
#   /initialpose
#   /map
#   /odom
#   /particle_cloud
#   /scan
#   /scan_filtered

# ── Node list sanity check ────────────────────────────────────────────────────
ros2 node list
# Must include:
#   /amcl
#   /map_server
#   /lifecycle_manager_localization
#   /scan_front_filter  (or whatever the filter node is named)
#   /ekf_filter_node
#   /robot_state_publisher
```

---

### 13j. Connections between phases — where Phase 2 ends and Phase 3 begins

Phase 2 leaves the robot with a running localization stack:

```
Phase 2 provides:
  map_server   → /map  (static background for cost maps)
  amcl         → /amcl_pose  (where the robot is)
               → TF map→odom  (global pose for planners)
  lifecycle_manager_localization  (keeps the above alive)

Phase 3 adds on top:
  nav2_planner       → reads /map (+ costmap layers), /amcl_pose
                     → produces /plan (nav_msgs/Path)
  nav2_controller    → reads /plan, /odom, /scan_filtered (for local avoidance)
                     → produces /cmd_vel (geometry_msgs/Twist)
  nav2_costmap_2d    → layers: static (/map) + inflation + obstacle (/scan_filtered)
                     → provides global_costmap and local_costmap
  bt_navigator       → orchestrates planner + controller via behavior tree
                     → accepts /navigate_to_pose action goals
  lifecycle_manager_navigation  → manages all the above nodes
```

The `localization.launch.py` you wrote in Phase 2 gets imported unchanged into the Phase 3 launch file. Nothing in Phase 2's config needs to change for Phase 3 to work.
