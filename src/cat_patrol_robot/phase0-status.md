# Phase 0 — Status & Reference

Companion to `plan.md`. This is the practical, "what did we actually do
today" document — file inventory, command cheat-sheet, observed topics, and
lessons learned. Read alongside `plan.md` (which has the project arc) and
the actual code (which has the truth).

---

## 1. Files we created or modified

Three packages were touched. Each lives in a different repo (or should).

### 1a. `cat_patrol_robot/` — your existing repo

| File | Status | Purpose |
|------|--------|---------|
| `launch/cat_patrol.launch.py` | modified | (a) Fixed `lidar_frame_id` default `'laser'` → `'laser_link'` to match the URDF static TF chain. (b) Corrected the inline comment that wrongly attributed `odom→base_footprint` TF to the chassis driver — it's actually published by the EKF. |
| `plan.md` | modified | Added a "Phase 0 findings on this robot" subsection with the TF tree, frame-name fix, mechanical bias, encoder asymmetry, and trim calibration value. |
| `scripts/wheel_balance_diagnostic.py` | **new** | Standalone diagnostic that reads per-wheel encoder counts via `Rosmaster_Lib.get_motor_encoder()`. Two modes: `discover` (manually rotate each wheel to map encoder index → physical wheel) and `measure` (commanded forward motion, reports per-wheel rate deviation from the mean). |
| `phase0-status.md` | **new** | This document. |

### 1b. `odom_drift_checker/` — new standalone package

| File | Status | Purpose |
|------|--------|---------|
| `package.xml` | **new** | ament_cmake C++ package manifest. Deps: `rclcpp`, `nav_msgs`, `geometry_msgs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`. |
| `CMakeLists.txt` | **new** | Builds a single executable `odom_drift_checker_node`. C++17. |
| `src/odom_drift_checker_node.cpp` | **new** | Single-file `rclcpp::Node`: subscribes to `/odom`, integrates path length, prints a STATUS line every 2 s with `(x, y, yaw)` + path / drift, and prints a single `RETURN HOME` line each time the robot re-enters a 30 cm circle around its starting pose. Also runs an independent TF lookup (`tf_buffer_.lookupTransform("odom", "base_footprint", TimePointZero)`) as a cross-check against the topic. |

### 1c. `yahboomcar_bringup/` — Yahboom vendor package, modified locally

| File | Status | Purpose |
|------|--------|---------|
| `yahboomcar_bringup/Mcnamu_driver_X3.py` | modified | Added two ROS parameters `trim_vy_per_vx` and `trim_w_per_vx` (default 0.0 — no behaviour change unless explicitly set). In `cmd_vel_callback`, the commanded `vy` and `angular.z` are corrected by `trim * vx` before being sent to `set_car_motion`. This is the chassis-level mechanical-asymmetry trim. |
| `launch/yahboomcar_bringup_X3_launch.py` | modified | Declared `trim_vy_per_vx` and `trim_w_per_vx` as launch arguments (defaults `'0.0'`) and wired them into the `Mcnamu_driver_X3` Node parameters. |

### 1d. Suggested git layout

You currently track only `cat_patrol_robot`. You now have changes in three
distinct categories. Recommended repo structure:

#### Recommended (option A — three repos, one per category)

```
~/yahboomcar_ros2_ws/yahboomcar_ws/src/
  cat_patrol_robot/                ← repo 1 (you already have a remote for this)
  odom_drift_checker/              ← repo 2 (NEW — create now)
  yahboomcar_bringup/              ← Yahboom upstream; do NOT git-init in place
```

For repo 2 (`odom_drift_checker`), create it now:

```bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src/odom_drift_checker
git init
echo -e "build/\ninstall/\nlog/\n__pycache__/\n*.pyc\n.ros_inspect_logs/" > .gitignore
git add .
git commit -m "Phase 0: odom_drift_checker — diagnostic node for wheel-odometry drift"
# Create empty repo on GitHub/GitLab named odom_drift_checker, then:
git remote add origin <YOUR_REMOTE_URL>
git push -u origin main
```

For the Yahboom modifications, **do not commit them inside the Yahboom
package** (you'd lose them on a fresh clone of their repo). Instead, keep
patches in `cat_patrol_robot/patches/`:

```bash
mkdir -p ~/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/patches
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src
diff -u yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver_X3.py.orig \
        yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver_X3.py \
   > cat_patrol_robot/patches/0001-trim-params-Mcnamu_driver_X3.patch
diff -u yahboomcar_bringup/launch/yahboomcar_bringup_X3_launch.py.orig \
        yahboomcar_bringup/launch/yahboomcar_bringup_X3_launch.py \
   > cat_patrol_robot/patches/0002-trim-launch-args.patch
```

(You'll need to keep the `.orig` files around — easiest way is to save
fresh copies of the files *before* you modify them, in a sibling
`upstream/` directory, the first time you touch a vendor package.)

Then write a tiny `cat_patrol_robot/scripts/apply_patches.sh` that the next
time you set up the workspace runs:

```bash
#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/../../"
for p in cat_patrol_robot/patches/*.patch; do
  patch -p0 < "$p"
done
```

#### Alternative (option B — fork the Yahboom repo)

If you prefer a single source of truth, fork
`https://github.com/YahboomTechnology/Yahboomcar_ros2` (or wherever this
package came from), apply your patches in your fork, and clone your fork
in place of the upstream. Cleaner for big modifications, more overhead for
small ones.

**For now I'd suggest option A.** Two patches is small enough that it
doesn't justify a fork.

---

## 2. Topics that produced data during testing

These appeared in `ros2 topic list` once both `bringup` and `sllidar_ros2`
were running. Bold = ones we actually used.

| Topic | Hz | Source node | What it carries |
|-------|----|-------------|-----------------|
| **`/odom`** | 10.0 | `ekf_filter_node` | EKF-fused odometry (frame_id=odom, child=base_footprint). The "right" pose to subscribe to from any consumer. |
| `/odom_raw` | 10.0 | `base_node_X3` | Raw wheel-only integration of `/vel_raw`. |
| `/imu/data_raw` | 10.0 | `Mcnamu_driver_X3` | Raw IMU from the Rosmaster MCU (no orientation, just gyro+accel). |
| **`/imu/data`** | 10.0 | `imu_filter_madgwick` | Filtered IMU (gyro-fused orientation; mag disabled). Consumed by EKF. |
| `/imu/mag` | 10.0 | `Mcnamu_driver_X3` | Raw magnetometer. |
| `/vel_raw` | 10.0 | `Mcnamu_driver_X3` | Chassis-frame velocity reported by the MCU firmware. NOT raw encoders — the firmware does its own kinematics. |
| `/joint_states` | 10.0 | `Mcnamu_driver_X3` | Joint names; doesn't currently populate position/velocity. Mostly cosmetic. |
| `/voltage` | 10.0 | `Mcnamu_driver_X3` | Battery voltage (you'll use this for the low-battery return-home in Phase 7). |
| **`/tf`** | 20.0 | `ekf_filter_node` (odom→base_footprint), `joint_state_publisher` (wheel joints) | Live transforms. |
| **`/tf_static`** | latched | `robot_state_publisher` | URDF static transforms (base_footprint→base_link, base_link→{laser_link, camera_link, imu_link}). |
| **`/scan`** | 7.4 | `sllidar_node` | LiDAR scan, frame_id=`laser_link` (after our fix; was `laser` before). |
| **`/cmd_vel`** | 1–20 (varies) | `teleop_twist_keyboard` (or `yahboom_joy_X3`) | Commanded velocity. After our trim work, the chassis driver applies a small leftward strafe correction internally. |
| `/joy`, `/JoyState` | varies | `joy_node`, `yahboom_joy_X3` | Joystick events (only when use_joystick=true). |
| `/edition` | 10.0 | `Mcnamu_driver_X3` | Firmware version (debug). |

Plus **per-motor encoder counts**, which are NOT a ROS topic — they're
read by `wheel_balance_diagnostic.py` via direct `Rosmaster_Lib.Rosmaster.
get_motor_encoder()` calls. Mapping discovered empirically:

| Index | Wheel |
|-------|-------|
| `m1` | FL (front left) |
| `m2` | BL (back left) |
| `m3` | FR (front right) |
| `m4` | BR (back right) |

---

## 3. Commands we ran (cheat-sheet)

Grouped by purpose. Re-source `setup.bash` in any terminal where you need
ROS executables to be discoverable, especially after a `colcon build`.

### 3a. Common shell prelude (every fresh terminal)

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
# ROS_DOMAIN_ID is set by the system shell init (=28 on this Jetson).
```

### 3b. Bringup (chassis + odom + EKF + joystick) — terminal 1

```bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py \
    chassis_serial_port:=/dev/ttyUSB1 \
    use_joystick:=true \
    trim_vy_per_vx:=0.012        # calibrated value for this chassis
```

Watch for the line confirming the trim took effect:
```
[Mcnamu_driver_X3-3] [INFO] [...]: cmd_vel trim: trim_vy_per_vx=0.0120, trim_w_per_vx=0.0000
```

### 3c. LiDAR (separate launch) — terminal 3

```bash
ros2 launch sllidar_ros2 sllidar_launch.py \
    serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
    serial_baudrate:=115200 \
    frame_id:=laser_link
```

### 3d. Drift checker — terminal 5

```bash
ros2 run odom_drift_checker odom_drift_checker_node
```

Optional parameter overrides (drove default scope of "near home"):

```bash
ros2 run odom_drift_checker odom_drift_checker_node --ros-args \
    -p return_threshold_m:=0.15 \
    -p away_threshold_m:=0.50
```

### 3e. Teleop — terminal 6

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Keys: `i` forward, `,` backward, `j` rotate left, `l` rotate right,
`u`/`o` forward+turn, `m`/`.` back+turn. `q`/`z` increase/decrease all
speeds; `w`/`x` increase/decrease linear only; `e`/`c` increase/decrease
angular only. **Calibrated trim is for ~0.15 m/s — slow it down with
`x` until you're around there.**

### 3f. /cmd_vel inspection — terminal 7

```bash
ros2 topic echo /cmd_vel
```

### 3g. Wheel-balance diagnostic — needs bringup STOPPED first

Stop the bringup (terminal 1, Ctrl-C), put the robot on blocks for
discover and on-blocks measurements:

```bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/scripts
python3 wheel_balance_diagnostic.py discover                  # one-time, find m{1..4}->wheel mapping
python3 wheel_balance_diagnostic.py measure --on-blocks       # commanded forward, encoders only
python3 wheel_balance_diagnostic.py measure --duration 2.0    # same on the floor (DRIVES forward!)
```

Restart bringup afterward.

### 3h. One-shot inspection commands

```bash
# Topic list
ros2 topic list | sort

# What's /odom carrying right now?
ros2 topic echo /odom --once

# Publish rates
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 topic hz /tf

# Frame headers (just enough to verify frame_id correctness)
ros2 topic echo /scan --once --field header
ros2 topic echo /imu/data --once --field header

# TF tree as a PDF (output: frames_<timestamp>.pdf in CWD)
cd /tmp && ros2 run tf2_tools view_frames

# Specific transform
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link laser_link
```

### 3i. Build commands

```bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws

# Build only the package you changed:
colcon build --symlink-install --packages-select odom_drift_checker
colcon build --symlink-install --packages-select yahboomcar_bringup
colcon build --symlink-install --packages-select cat_patrol_robot

# After ANY build that changes the install layout, re-source:
source install/setup.bash
```

---

## 4. Lessons learned

In rough order of "I wish I'd known this before".

### 4a. About this robot specifically

1. **The chassis driver does NOT publish `/odom` or any TF.** `Mcnamu_driver_X3.py` only publishes `/vel_raw`, raw IMU, joint_states, voltage. The full odom pipeline is `Mcnamu_driver_X3` → `base_node_X3` (integrates `/vel_raw` → `/odom_raw`) → `imu_filter_madgwick` (filters IMU) → `ekf_filter_node` (fuses both → `/odom` AND publishes `odom→base_footprint` TF). If `/odom` looks wrong, the EKF is the most likely culprit, not the chassis driver.
2. **`/scan`'s `frame_id` defaulted to `'laser'`, but the URDF defines the static TF as `'laser_link'`.** This means SLAM/AMCL would have silently failed to project scans into the robot frame in Phase 1. Fixed in `cat_patrol.launch.py`. Always verify `frame_id` matches a frame that actually exists in `/tf` or `/tf_static` before relying on it.
3. **The X3 has mecanum wheels** (not differential drive). `set_car_motion(vx, vy, w)` accepts independent `vy`, and the firmware does the four-wheel kinematics internally. Mecanum wheels are *handed*: same-side wheels share roller orientation on this unit (left side = `\`, right side = `/`). You cannot freely swap wheels — only same-handedness positions can be exchanged.
4. **`/odom` was honest about the chassis curving even when the chassis curved physically.** That's a *good* outcome — it means the EKF (gyro-fused) caught the bias. SLAM/AMCL/Nav2 only need `/odom` to reflect reality, not the robot to drive straight.
5. **Per-wheel encoder rates were balanced within ±5%** when commanded forward, but the chassis still curved by ~16% on the floor. Most of the "extra" drift came from cumulative noise (transients, gyro drift over 13 s drives, teleop wobble), not steady-state bias. The right calibration coefficient for `trim_vy_per_vx` was **14× smaller** than the naive ratio Δy/Δx would have predicted.
6. **`Rosmaster_Lib` is a single-serial-port library.** Only one process at a time can hold `/dev/ttyUSB1`. Diagnostics that talk directly to the firmware (like `wheel_balance_diagnostic.py`) require stopping the bringup driver first.
7. **The chassis MCU has a built-in cmd_vel watchdog** (visible as the `cmd_vel timeout 0.61s` warning at startup). It stops the motors if no command arrives for `cmd_timeout_sec` (default 0.6 s in our driver). Healthy safety property, not a bug.

### 4b. About ROS 2 / colcon / sandboxing

8. **`colcon build --symlink-install` for Python packages uses egg-links.** After such a rebuild, *any shell that sourced `install/setup.bash` before the rebuild* has stale `PYTHONPATH` entries pointing to paths that no longer exist. Re-source after every build.
9. **`tf2_echo` against a static TF can transiently fail with "frame does not exist"** until `/tf_static` is consumed by the listener. It's not an error; just call again or wait a couple seconds.
10. **`view_frames`'s output filename includes a timestamp.** Newer versions write `frames_<date>.pdf`/`.gv`; the `.gv` (Graphviz source) is more grep-friendly than the PDF.
11. **The Cursor sandbox blocks DDS multicast by default.** ROS topic discovery from inside the sandbox returns empty topic lists even when nodes are running. Fix: rerun the command with `required_permissions: ["all"]` (or move it to a real terminal).
12. **The Cursor sandbox blocks writes to `/home/jetson/.ros/log/`.** Some `ros2 cli` and `tf2_*` tools fail to start because spdlog can't open the log file. Fix: `export ROS_LOG_DIR=<somewhere-writable>` before running.

### 4c. About C++ / robotics craft

13. **`std::optional<>` beats sentinel-and-bool flag pairs** for "value not yet captured". `home_pose_.has_value()` is cleaner than `home_initialized && home_x == ...`.
14. **`tf2_ros::Buffer` must be declared before `tf2_ros::TransformListener`** in the class member list — the listener's constructor takes a reference to the buffer, and C++ initializes members in *declaration order*, regardless of the order they appear in the constructor's initializer list. Get this wrong and you get an undefined-behaviour reference to an uninitialized buffer.
15. **A naive `path_length / euclidean_distance_from_start ≈ 1.018` does NOT mean "robot went straight".** For a smooth gentle arc with small lateral drift, the ratio is very close to 1. Don't read the chassis curve out of that ratio alone — read the (x, y, yaw) directly.
16. **Calibration is empirical and should always be bisected, never extrapolated from a single measurement.** Single drives include noise that swamps the steady-state bias.
17. **`linear_velocity_x = vx * linear_scale_x_` in `base_node_X3.cpp` affects how `/odom_raw` is INTEGRATED, not how the wheels actually move.** If you want to change the actual chassis behaviour, you have to act on the cmd_vel side (before `set_car_motion`), not on the odom side.
18. **`/odom` is observation-only at this stage of the project.** Nothing reads `/odom` and writes back to `/cmd_vel`. The closed loop appears in Phase 3 with `nav2_controller`. Until then, "the robot doesn't drive straight" is purely a teleop annoyance, not a navigation blocker.
19. **Mecanum behaviour is heavily floor-dependent.** Tested with a 12-second `j` (rotate left) command at 1.0 rad/s commanded:
    - **On carpet**: ~360° in-place rotation, ~5 cm unwanted translation. /odom matches reality. The mecanum rollers grip carpet pile, the kinematics work as designed.
    - **On smooth floor (laminate / tile)**: ~190 cm of arcing translation (165 cm back + 95 cm left) with only ~90° of physical rotation. /odom only sees ~3 cm of translation — the wheel-derived chassis Twist is blind to lateral roller-slip. The IMU-derived yaw is roughly correct (~113° in /odom).
    - **Practical implication**: position estimates from /odom are unreliable on smooth floors during rotation. Yaw is reliable on both surfaces. Phases 1–2 (SLAM, AMCL) compensate via lidar; until then, prefer arcing turns (forward + small angular.z) on smooth floors.
20. **The robot's actual environment has BOTH surfaces** — carpet in one area, smooth flooring in the rest. The patrol manager (Phase 4 onward) should be aware of which area is which, and prefer to schedule in-place rotations on the carpet area when possible. The map built in Phase 1 won't directly know about floor type, so this is information you'd add manually (or via a future "floor segmentation" extension).

---

## 5. Calibration record (this specific X3 unit)

| Parameter / metric | Value | Notes |
|-----------|-------|-------|
| `trim_vy_per_vx` | **0.012** | Calibrated at vx ~0.15 m/s on hardwood floor. Empirical bisection from initial 0.04. |
| `trim_w_per_vx` | 0.0 | Yaw bias at this speed was small enough to ignore. |
| Chassis curve at trim=0 | ~16 cm/m rightward | Most of which was cumulative noise, not steady-state. |
| Per-wheel encoder spread (on blocks) | ±3.2% from mean | FL fastest, BL slowest, FR slow, BR fastest. |
| Per-wheel encoder spread (on floor) | ±4.5% from mean | FL gets faster under load; BL converges to mean. |
| Approximate baseline path rate | 587 ticks/s on floor / 704 ticks/s on blocks @ 0.15 m/s commanded | The 17% drop on the floor is normal motor-load behaviour. |
| Closed-loop drift (out-and-back, 4.69 m total path, with trim active) | **14.4 cm position, 4.7° yaw** | Phase 0 acceptance gate. Within plan's "10–20 cm position, 5–10° yaw" healthy band. Forward leg was ≈2 mm lateral over 2.31 m (clean); most drift came from the uncalibrated backward leg. |
| In-place rotation on carpet (12 s `j`, 1.0 rad/s commanded) | ~360° physical, ~5 cm translation, +12.5° final yaw in /odom | Mecanum kinematics work as designed; /odom matches reality. |
| In-place rotation on smooth floor (12 s `j`, 1.0 rad/s commanded) | ~90° physical rotation, ~190 cm translation; /odom shows ~3 cm translation, ~113° yaw | Mecanum rollers slip; chassis arcs instead of pivoting. /odom is blind to the lateral sliding (yaw is roughly right via IMU). |

If you re-build the robot or service the wheels, **re-run
`wheel_balance_diagnostic.py measure` and re-do the trim bisection**.
The numbers above are unit-and-floor specific.

---

## 6. Open questions / ideas to pursue later

Not blocking; just noted for later.

- The `joint_states` published by `Mcnamu_driver_X3` doesn't populate
  `position` or `velocity` — only names. Fixing it would let you visualize
  wheel rotation in RViz from URDF.
- `base_node_X3.cpp` initializes `last_vel_time_` without a value — first
  iteration's `vel_dt_` is huge (epoch start to current time). EKF probably
  rejects the resulting outlier, but it's worth fixing.
- Yaw drift over long stationary periods (gyro bias) was visible in early
  drift-checker runs as `yaw=-6.9 deg → -6.5 deg → +7.3 deg → -5.8 deg`
  oscillation. Worth investigating before Phase 2 (AMCL) if it persists.
- The chassis trim only fixes forward motion. Strafe and rotation aren't
  compensated. They probably don't need to be (Nav2 will close the loop)
  but worth checking once we get there.
- **Surface-aware patrol planning (Phase 4+).** The patrol manager could
  benefit from knowing which areas of the map are carpet vs smooth floor,
  and preferring to put in-place rotations on the carpet. Implementation
  ideas, ordered cheap-to-expensive:
  - **Manual annotation**: hand-paint a `floor_type.yaml` (or PNG) over the
    SLAM map after Phase 1, encoding "carpet here, smooth there".
  - **Slip detector**: at runtime, watch the disagreement between IMU yaw
    rate and wheel-derived yaw rate. When the chassis is commanded to
    rotate in place but the lidar/AMCL pose isn't actually rotating in
    place (i.e., the chassis has slid), tag the current location as
    "slippery" in a slip-cost map. Over many patrols the map fills in.
  - **Behaviour selection**: when planning a patrol leg that requires a
    heading change, the patrol manager picks the route that lets the
    rotation happen on carpet, even if the carpet route is geometrically
    longer.
