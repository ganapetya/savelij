# Cat Patrol Robot — Step-by-Step Execution Plan

> **Purpose of this project:** learn modern **C++** and **ROS 2** by building a
> real, useful robot end-to-end. Hardware: Yahboom X3 (Jetson Orin NX, RPLidar
> A1, Orbbec Astra depth/color camera).
>
> Each phase has a clear deliverable, an explicit **C++ / ROS 2 learning goal**,
> and a "done when…" checklist. Phases are sized so that finishing one feels
> like a real milestone, not a chore.

---

## Where this plan stands relative to the existing code

There is already a working monolithic `patrol_node` in this package:

- One C++ node, FSM at 20 Hz, Strategy pattern for patrol behaviors
  (`ClassicPattern`, `TillObstacleBackPattern`).
- Direct `/cmd_vel` + TF odometry navigation (no Nav2).
- LiDAR front-cone + depth-camera obstacle stop.
- Bluetooth speaker bring-up, buzzer, JPEG capture, JSON mail request to a
  Python `mail_node`.

This plan **replaces the navigation core** (raw `/cmd_vel` + odom) with Nav2,
and **reuses everything else** (mail pipeline, BT speaker scripts, buzzer
wrapper, image-save helpers, the Strategy-pattern idea). Concretely:

- The existing `patrol_node` becomes a reference implementation; it stays
  buildable through Phase 3, then is gradually superseded by the new
  `patrol_manager` package starting in Phase 4.
- New work goes into **separate packages** under
  `yahboomcar_ws/src/`: `patrol_manager`, `cat_detector`,
  `odom_drift_checker`, etc. Mirrors how a professional repo would be
  structured.
- The existing Python `mail_node` and BT scripts are kept as-is and called by
  the new `patrol_manager` once it exists.

---

## Phase 0 — Foundation check  (~1 week)

**Why this phase matters.** SLAM, AMCL, and Nav2 all assume a sane TF tree
and trustworthy odometry. If `odom → base_link` lies to you, *every* later
phase will be debugged in the wrong layer. This phase exists so you never
have to ask "is it the planner, the localization, or the wheels?" again.

### Tasks
- [ ] Read the Yahboom-provided URDF for the X3; adapt frame names to match
      what the chassis driver actually publishes.
- [ ] Confirm `odom → base_link` (or `base_footprint`) is being published by
      the chassis driver. Run `ros2 topic echo /odom` while teleoperating.
- [ ] Verify the TF tree is connected, and that the LiDAR frame is in it:
      `ros2 run tf2_tools view_frames` → open the generated PDF.
- [ ] **Square test.** Teleop the robot in a 2 m square, return to start.
      Acceptable drift: position within ~10–20 cm, heading within ~10°.
- [ ] **C++ exercise — `odom_drift_checker` node.** Subscribes to `/odom`,
      integrates absolute distance traveled, logs accumulated error each
      time the robot is "near home" again. Single-file node, plain
      `rclcpp::Node`, one subscription, one timer.

### C++ / ROS 2 learning goal
Subscriptions, callbacks, basic `rclcpp::Node` structure, message types,
`tf2_ros::Buffer` + `TransformListener` lookup at a specific time.

### Done when
TF tree is connected end-to-end (LiDAR → base → odom), the square test
closes within tolerance, and the drift-checker node prints sensible numbers
during a 5-minute teleop session.

> **Pitfall.** Pure wheel odometry on a small robot will drift surprisingly
> fast on carpet. If the square test is way off, that's a *real* finding —
> note it now, because it'll explain weirdness later in Phase 2.

### Phase 0 findings on this robot (2026-06-11)

- TF tree clean: `odom → base_footprint → base_link → {wheels, imu_link, camera_link, laser_link}`. The EKF (`robot_localization`) is what publishes `odom → base_footprint`, **not** the chassis driver — `Mcnamu_driver_X3.py` only publishes raw IMU/`vel_raw`/joint_states, and `base_node_X3` integrates `vel_raw` into `/odom_raw` without broadcasting TF.
- LiDAR `frame_id` mismatch fixed in `cat_patrol.launch.py`: was `'laser'`, now `'laser_link'` to match the URDF static TF chain.
- **Mechanical bias.** Commanded `linear.x=0.15 m/s` straight forward, `angular.z=0`, the X3 physically curves to the right (initial measurement: ≈51 cm over 3.13 m). The EKF (gyro-fused IMU) sees this curve correctly: `/odom` reports the same ~51 cm Δy, ~−4.8° Δyaw. So odom is honest about the drift; the robot itself just doesn't drive straight on command. AMCL/SLAM should be unaffected since they don't rely on the robot driving straight, only on `/odom` reflecting reality.
- **Per-wheel encoder asymmetry (`scripts/wheel_balance_diagnostic.py`).** With `set_car_motion(0.15, 0, 0)`:
  - On blocks: m1=FL +1.6%, m2=BL −2.5%, m3=FR −2.2%, m4=BR +3.2% from the 4-wheel mean (~5% diagonal {FL,BR} > {BL,FR} pattern).
  - On floor (under load): same diagonal pattern shrinks slightly but a *new* left-vs-right asymmetry appears (left side ~3.8% faster), most likely from lateral CG offset + load-dependent motor saturation. FR is consistently the slowest wheel; FL the fastest, on both surfaces. This is below the ~2% "all balanced" threshold for both pure motor and pure mechanical fixes, so the cause is partly motor balance and partly wheel/floor coupling.
- **Software trim in `Mcnamu_driver_X3.py`.** Two new chassis-level parameters added (`trim_vy_per_vx`, `trim_w_per_vx`, both default 0.0). They inject a small leftward strafe / yaw correction proportional to commanded forward velocity, before calling `set_car_motion`. The firmware's per-wheel PID then implements the corrected command. Calibrated empirically on this unit:
  - **`trim_vy_per_vx = 0.012`** at cruise speed ~0.15 m/s.
  - `trim_w_per_vx = 0.0` (yaw bias was small enough not to need correction).
  - Naive trim from the raw single-drive ratio (Δy/Δx ≈ −0.163) is **14× too large** because the cumulative drift includes a lot of transient noise. The right calibration is bisection by repeated trial, not extrapolation from one drive.
  - The trim is speed-dependent. Calibrated at 0.15 m/s; expect somewhat worse straight-line behavior at very different speeds. Nav2's controller (Phase 3) closes the loop on `/odom` and will compensate for any residual bias, so this trim is mainly for nicer teleop.
- `tf_vs_topic` disagreement stays at 0.0000 m — `/odom` topic and `odom→base_footprint` TF are perfectly consistent (both come from the same EKF instance).
- **Closed-loop drift test (out-and-back, 4.69 m total path).** After the chassis trim was calibrated, drove forward 2.31 m at 0.15 m/s, stopped, drove backward to start. `/odom` resting pose:
  - Position drift: **14.4 cm** (5.6 cm behind start + 13.2 cm to the left). Within plan's "10–20 cm healthy" band.
  - Yaw drift: **−4.7°**. Within plan's "5–10° healthy" band.
  - The forward leg was clean (≈2 mm lateral over 2.31 m). Most of the drift accumulated during backward motion (the trim is calibrated only for forward; backward direction would need a separate `trim_vy_per_vx_reverse`). The yaw drift accumulated steadily across the whole drive.
  - **Phase 0 acceptance criteria met. `/odom` is trustworthy enough for Phases 1–3.**
- **Mecanum on mixed floor surfaces — surface-dependence of in-place rotation.** This robot operates in an environment with **carpet in one area and smooth flooring (laminate / tile) in the rest**. The two surfaces produce dramatically different behaviour for in-place rotation commands (`angular.z != 0`, `linear.x = 0`), tested with a 12-second `j` press at 1.0 rad/s commanded:
  - **On carpet**: physical motion was ~360° in place with ~5 cm of unwanted translation. `/odom` agreed (yaw integrated ~372°, position drifted ~4.3 cm). Mecanum rollers grip carpet pile, lateral pivot forces are real, kinematics work as designed.
  - **On smooth floor**: physical motion was ~190 cm of arc-shaped translation (165 cm back + 95 cm left) with only ~90° of physical rotation. `/odom` reported ~3 cm of translation and ~113° of yaw — the IMU caught the rotation roughly correctly, but the wheel-derived chassis Twist saw none of the lateral sliding. Mecanum rollers slip on smooth floors; lateral force becomes friction-limited; the chassis drifts in an arc rather than pivoting.
  - **Implication**: position estimates from `/odom` alone are **not reliable on the smooth-floor area during rotation**. The IMU yaw is reliable on both surfaces. Phases 1–2 (SLAM and AMCL) will partly compensate via lidar-based localization (`map → odom` correction); Phase 3 (Nav2 controller) will compensate fully via closed-loop control over scan-matched pose. Until then, prefer **arcing turns** (`forward + small angular.z`) over in-place rotation on smooth floors — the wheels stay rolling forward, where mecanum is honest.
  - **Behavioural takeaway for the patrol manager (Phase 4 onward)**: design waypoint paths so that in-place rotations happen on carpet whenever possible, and arcing turns are used on the smooth-floor area. The patrol manager's strategy can choose, given a map of which floor type is where.
  - **No simple software trim fixes this.** Roller slip is highly non-linear in velocity and surface. The fix path is the lidar-feedback loop in Phases 2–3, not a constant coefficient.

---

## Phase 1 — Map the room  (1–2 weeks)

**Why this phase matters.** You need a known-good occupancy grid before any
of Phase 2 onward makes sense. This is also the phase where you'll
experience first-hand why occupancy grids are noisy: cats walking through
the LiDAR plane, glass tables, dark furniture legs, mirrors.

### Tasks
- [ ] Bring up `slam_toolbox` (online async mode) while teleoperating
      slowly around the room.
- [ ] Drive a loop, then close it — watch the loop closure event in RViz.
- [ ] Save the map: `ros2 run nav2_map_server map_saver_cli -f my_room`
      (produces `my_room.pgm` + `my_room.yaml`).
- [ ] Commit the map files into a new package or `cat_patrol_robot/maps/`.
- [ ] Write a small `slam.launch.py` that brings up chassis + LiDAR +
      slam_toolbox + RViz with a sensible config preloaded.

### C++ / ROS 2 learning goal
Mostly conceptual: occupancy grids, scan matching, loop closure. Practical:
launch-file Python, YAML parameter files, RViz configs.

### Done when
You have a clean PGM/YAML of the room, saved into the repo, and a launch
file that reproduces the SLAM session in one command.

> **Tip.** Drive *slowly* and avoid sharp turns during your first map run.
> Aggressive motion + a small LiDAR like the A1 is the #1 reason maps come
> out smeared.

---

## Phase 2 — Localize on the saved map  (~1 week)

**Why this phase matters.** This is your first encounter with **lifecycle
nodes** — `nav2_amcl` is a managed node, and Nav2 itself is built on the
same pattern. The lifecycle pattern (configure → activate → deactivate →
cleanup → shutdown) is one of the more interesting C++ designs you'll meet
in ROS 2.

### Tasks
- [ ] Bring up `nav2_amcl` with your saved map and a `nav2_lifecycle_manager`.
- [ ] In RViz: load the map, drop an initial pose estimate, drive around,
      watch the particle cloud converge.
- [ ] Tune `min_particles`, `max_particles`, `laser_model_type`, and the
      various `_z_*` weights enough that the cloud is tight after a couple
      meters of driving.

### C++ / ROS 2 learning goal
**Lifecycle nodes**: read the `rclcpp_lifecycle` headers and trace through
how `on_configure` / `on_activate` / `on_deactivate` work. You don't have
to write one yet (you will, much later, if you push the patrol manager to a
lifecycle node), but understanding the contract pays off all through Nav2.

### Done when
Click "2D Pose Estimate" in RViz once at startup, drive the robot, the
particle cloud converges and stays converged, and the `map → odom`
transform looks sane in `view_frames`.

> **Pitfall.** AMCL in a small home with cats walking through the laser
> plane can wobble. If localization is constantly jumpy, raise the
> measurement noise and lower the expected weights — don't chase it with
> tighter parameters.

---

## Phase 3 — Autonomous navigation with Nav2  (2–3 weeks)

**Why this phase matters.** This is where the **action client architecture**
you've studied becomes real. `NavigateToPose` is a ROS 2 action with
goals, feedback, and results — and writing the C++ client for it exercises
goal handles, feedback callbacks, and `std::shared_future`. The conceptual
rhyme with `std::future` / `std::packaged_task` from *C++ Concurrency in
Action* is exactly the point.

### Tasks
- [ ] Full Nav2 bringup: `nav2_planner`, `nav2_controller`, global +
      local costmaps, behavior server, `nav2_lifecycle_manager`.
- [ ] Tune costmap **inflation radius** for your robot footprint. Start
      conservative — a tight inflation will make Nav2 plan paths through
      walls; a generous inflation will make it refuse to plan in narrow
      doorways. Iterate.
- [ ] **Milestone 1.** Click a goal in RViz → robot navigates there,
      avoids obstacles, stops near the goal pose.
- [ ] **Milestone 2 — C++ action client (the meat of this phase).**
      Write `nav_goal_client_node.cpp`: a standalone node that
      programmatically sends `NavigateToPose` goals, prints feedback
      (distance remaining), and prints the final result. Exercise:
      - Goal handle (`ClientGoalHandle::SharedPtr`).
      - Feedback callback signature.
      - Result future and how to wait on it without blocking the executor.
      - Goal cancellation API (you'll need this in Phase 6).

### C++ / ROS 2 learning goal
Action clients (`rclcpp_action::Client`), goal handles, feedback
callbacks, futures, lambdas as callbacks, `std::bind` vs lambdas. This is
the single most valuable phase for C++/ROS 2 fluency.

### Done when
Both milestones pass: clicked goals work in RViz, and your standalone
`nav_goal_client_node` can drive the robot to a hard-coded pose and print
"goal succeeded" / "goal canceled" / "goal aborted" correctly.

> **Tip.** Tether the robot to a monitor or use RViz over your TP-Link
> robot network during all of Phase 3. Debugging Nav2 blind is misery.

---

## Phase 4 — Patrol behavior  (2–3 weeks)

**Why this phase matters.** This is the *heart* of the project and almost
pure C++ design work. It's also where the existing `patrol_node`'s FSM and
Strategy pattern are reborn on top of Nav2 instead of raw `/cmd_vel`.

### Tasks
- [ ] Create a new package `patrol_manager` (C++, ament_cmake).
- [ ] Define a top-level FSM (similar to the existing one but Nav2-native):
      ```
      SLEEPING → WAKING → PATROLLING(waypoint i) → CAPTURE(i) → … → RETURNING → SLEEPING
                                                                ↘ INVESTIGATING (Phase 6)
      ```
- [ ] Record 4–6 waypoints by driving the robot to each spot and capturing
      the pose. Save them in a YAML file in `patrol_manager/config/`.
- [ ] Use the action client from Phase 3 to drive between waypoints.
- [ ] At each waypoint, take a photo (reuse the JPEG-save logic from the
      existing `patrol_node`) and append it to the cycle's photo list.
- [ ] After the last waypoint, send a goal to the recorded home pose,
      then go to `SLEEPING`.
- [ ] Wake-up scheduling: start with a `rclcpp::Timer` that fires every
      `patrol_period_sec`. Defer the systemd-timer-launches-stack option
      to Phase 7.
- [ ] **C++ design exercises**:
      - Composable node vs. standalone executable — try both.
      - `MultiThreadedExecutor` with **two callback groups**:
        - *MutuallyExclusive* group for the FSM tick (so it never
          re-enters itself).
        - *Reentrant* group for action feedback + sensor callbacks (so
          they don't starve the FSM tick).
      - Strategy pattern (carry over from `patrol_node`): `PatrolStrategy`
        base class, `WaypointPatrolStrategy` derived. Leaves room for
        future strategies (random walk, perimeter sweep, etc.).

### C++ / ROS 2 learning goal
This phase exercises *most* of *C++ Concurrency in Action* in practice:
executors, callback groups, mutexes (or atomics), `std::function`,
ownership with `unique_ptr` vs `shared_ptr`, RAII for the action goal
handle. You'll feel the difference between a single-threaded executor
that starves and a multi-threaded executor with proper callback groups.

### Done when
The robot wakes up on schedule, drives a full waypoint cycle via Nav2,
takes a photo at each, returns home, sends one email with all photos,
and goes back to sleep. End-to-end, unattended.

> **Tip.** Keep `patrol_manager` strictly Nav2-action-driven — never
> publish raw `/cmd_vel` from it. If you find yourself reaching for
> `/cmd_vel`, that's a sign the behavior belongs in a Nav2 behavior
> plugin, not in your manager.

---

## Phase 5 — Cat detection  (4–6 weeks, parallel-friendly)

**Why this phase matters.** Two sub-stages (generic detection, then per-cat
identification) and a producer-consumer pipeline that's a textbook C++
concurrency exercise.

> **Honest timeline note.** The original sketch said 2–4 weeks. On a Jetson
> Orin NX, going from "fresh OS" to "TensorRT-exported per-cat classifier
> running at real-time" tends to take 4–6 weeks once dataset collection,
> labeling, and re-training cycles are accounted for.

### Tasks (sub-stage 5a — generic cat detection)
- [ ] Create new package `cat_detector` (Python is fine here — `ultralytics`
      is dramatically easier than C++ here, and it doesn't cost you C++
      practice because the *interesting* concurrency is in 5b).
- [ ] Run YOLOv8n (or `jetson-inference detectnet`) on the Astra color
      stream. "cat" is a COCO class, so this works zero-shot.
- [ ] ROS 2 node: subscribes to the camera topic via `cv_bridge`,
      publishes `vision_msgs/Detection2DArray` and an annotated image.
- [ ] Export the model to TensorRT (`.engine`) for real-time inference.
      Measure FPS before and after.

### Tasks (sub-stage 5b — per-cat classifier)
- [ ] Use the patrol photos from Phase 4 as a *flywheel*: every cycle,
      photos get saved, you label them, the dataset grows.
- [ ] Once you have a few hundred per cat, fine-tune either:
      - A small classifier head over the YOLO crop, OR
      - YOLO with two custom classes ("cat_A", "cat_B").
- [ ] **C++ exercise — producer-consumer image pipeline.** If you want
      C++ here, write a small C++ node that:
      - Camera callback **produces** frames into a thread-safe bounded
        queue (the one you built reading the concurrency book).
      - A worker thread **consumes** frames, drops stale ones under load
        (queue capacity = 1, latest-wins), and forwards them to the
        Python detector via a service / topic.
      - This is a real exercise in `std::mutex`, `std::condition_variable`,
        and bounded-queue semantics.

### C++ / ROS 2 learning goal
**5a:** `cv_bridge`, vision messages, custom message types.
**5b:** producer/consumer concurrency in C++, thread-safe queue, drop-stale
under load.

### Done when
The detector publishes detections at ≥ 10 FPS during patrol, distinguishes
your two cats with reasonable accuracy on a held-out test set, and the
pipeline drops frames gracefully when inference can't keep up (no
unbounded memory growth).

---

## Phase 6 — Integration: see cat → capture → bark  (1–2 weeks)

**Why this phase matters.** This is where the patrol manager's FSM gains
a new state and you finally exercise **action cancellation** — an API
plenty of people never touch.

### Tasks
- [ ] Subscribe `patrol_manager` to the detector's topic.
- [ ] On a confident detection during `PATROLLING`:
      - **Cancel** the current `NavigateToPose` goal (`async_cancel_goal`).
      - Transition FSM to `INVESTIGATING`.
      - Save annotated image, send mail with the cat's name in the subject,
        play the bark via the existing BT speaker pipeline.
      - Resume patrol from the *interrupted* waypoint (not the next one).
- [ ] **Stretch goal — nav-toward-cat.** Compute the cat's approximate
      position by combining the detection bearing (from camera intrinsics)
      with the lidar range or depth camera at that bearing, transform
      camera frame → map frame at the detection timestamp, and send a Nav2
      goal *in front of* the cat (offset for camera framing, snapped out of
      any obstacle). This is its own mini-phase; budget another 1–2 weeks
      if you take it on.

### C++ / ROS 2 learning goal
Action *cancellation*, TF lookups at a specific message timestamp,
multi-source FSM input (timer ticks, action results, detection messages —
all funneled through the right callback group from Phase 4).

### Done when
Mid-patrol cat sighting reliably triggers: cancel current nav goal → save
annotated photo → email with cat name → bark → resume from interrupted
waypoint. End-to-end, unattended.

> **Pitfall.** It's tempting to use the cat-detected callback to directly
> mutate FSM state. Don't — push an event into a queue and let the FSM
> tick consume it. Otherwise you'll fight race conditions for a week.

---

## Phase 7 — Polish (ongoing)

Rough priority order. Pick whichever sounds most fun next; none are
required for "the project works."

- [ ] **Battery monitoring.** Subscribe to `/voltage`, return-home-when-low,
      systemd-stop the stack at critical voltage. The existing
      `patrol_node` already has the voltage subscription and warn
      threshold — port that over.
- [ ] **systemd-launches-stack.** Replace the in-process wake-up timer
      with a systemd `OnCalendar=` timer that starts the whole stack via
      `cat-patrol.service`, runs one patrol cycle, then shuts down.
      More power-friendly on a Jetson that's mostly idle.
- [ ] **Web dashboard.** Small Flask/FastAPI app showing live map +
      robot pose (subscribe via `rosbridge`), latest cat sightings, and
      patrol history. Plotting + last-N JPEGs.
- [ ] **Patrol logs.** "Murka detected 3 times near the couch" — keep a
      sqlite log of every detection with timestamp + map pose + cat name.
- [ ] **Multi-room.** Multiple maps, "go to kitchen" command, optionally a
      docking station with charging contacts.
- [ ] **Lifecycle-ize the patrol manager.** Convert to
      `LifecycleNode`. Now Nav2's lifecycle manager can bring up your node
      together with the rest of the stack. This is a clean way to revisit
      the lifecycle concept from Phase 2 with a node you fully understand.

---

## Conventions

### Repo layout
Each new piece of work lives in its **own package** under
`yahboomcar_ws/src/`:

- `cat_patrol_robot/` — current monolith (kept buildable through Phase 3,
  superseded from Phase 4 on, eventually deleted).
- `odom_drift_checker/` — Phase 0.
- `patrol_manager/` — Phase 4 onward.
- `cat_detector/` — Phase 5 (mostly Python; small C++ producer/consumer if
  you want the exercise).
- `cat_patrol_msgs/` — shared custom messages (per-cat detection, patrol
  status, etc.) once the second package needs them.

### Workflow per phase
1. Branch off `main`: `feat/phaseN-<name>`.
2. Build with `colcon build --symlink-install --packages-select <pkg>`.
3. Source `install/setup.bash`.
4. Smoke test on the bench (robot on blocks if it might run away).
5. Real test on the floor.
6. Tick the boxes in this file, write a short note in the phase section
   about anything surprising, merge.

### Practical tips (carried forward from the original plan, still good)
- Tether the robot to a monitor (or RViz over the TP-Link robot network)
  during all of Phases 1–3. Debugging Nav2 blind is genuinely miserable.
- Tune costmap inflation conservatively at first — a patrol robot that
  hugs walls *will* eventually kiss one.
- Keep package boundaries clean. It mirrors how you'd structure this
  professionally and makes the "delete the monolith" step in Phase 4
  painless.
- Commit your config + map files. They are *part of* the robot. Treating
  them as throwaway is the #1 reason hobby ROS projects rot.
