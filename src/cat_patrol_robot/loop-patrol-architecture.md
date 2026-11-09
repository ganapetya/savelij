# Cat Patrol — Loop Patrol Architecture

Autonomous Nav2 waypoint patrol for **Savelij** (Yahboom ROSMASTER X3, Jetson
Orin NX, ROS 2 Humble). One command brings up the whole stack, drives a fixed
waypoint loop on a known map, photographs each waypoint, recognizes the
household cats, plays a per-cat voice line, and emails snapshots.

> Single entry point:
> ```bash
> patrol start        # preflight self-checks, then launch everything
> patrol status       # nodes + patrol_manager FSM state
> patrol stop         # kill the whole process group
> ```
> `patrol` = `cat_patrol_robot/scripts/patrol` (also symlinked in `~/myscripts2/`).
> It replaces the manual `t1..t10` script sequence documented at the end.

---

## 1. Runtime environment (applies to every node)

The whole graph must share one DDS configuration or nodes silently fail to see
each other. The `patrol` script exports these before launching:

| Variable | Value | Why |
|---|---|---|
| `ROS_DOMAIN_ID` | **28** | The domain the working stack uses (NOT 0). |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | CycloneDDS, not the default FastDDS. |
| `CYCLONEDDS_URI` | `file:///home/jetson/cyclonedds.xml` | Interface/buffer tuning (DDS OOM fix). |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | `…/yahboomcar_bringup/param/fastdds_mapping.xml` | Fallback if FastDDS is ever used. |

**Overlays sourced (in order):** `/opt/ros/humble` → `software/library_ws`
(contains **astra_camera**) → `yahboomcar_ws` (everything else). Missing the
`library_ws` overlay means the camera driver isn't found.

---

## 2. Node inventory

Five logical layers. All start from `patrol_system.launch.py`, staggered by time.

### Layer 1 — Sensors + base bringup  (`cat_patrol.launch.py`, `start_patrol_node:=false`)

| Node | Package | Subscribes | Publishes |
|---|---|---|---|
| `astra_camera` | astra_camera (library_ws) | — | `/camera/color/image_raw`, `/camera/depth/image_raw`, `camera_info` |
| `Mcnamu_driver_X3` | yahboomcar_bringup | `/cmd_vel`, `/Buzzer`, `/RGBLight` | `/vel_raw`, `/imu/data_raw`, `/imu/mag`, `/voltage`, `/edition` |
| `base_node_X3` | yahboomcar_base_node | `/vel_raw` | `/odom_raw` |
| `imu_filter_madgwick` | imu_filter_madgwick | `/imu/data_raw`, `/imu/mag` | `/imu/data` |
| `ekf_filter_node` | robot_localization | `/odom_raw`, `/imu/data` | `/odom`, **TF `odom→base_footprint`** |
| `robot_state_publisher` | robot_state_publisher | `/robot_description` | **TF static** (`base_footprint→base_link→laser_link/camera_link…`) |
| `sllidar_node` | sllidar_ros2 | — | `/scan` (frame `laser_link`) |
| `scan_front_filter` | yahboomcar_bringup | `/scan` | **`/scan_filtered`** (trusted arc: center `π`, half-width `1.92`) |
| `mail_node` | cat_patrol_robot (Python) | `/cat_patrol/mail_request` | → SMTP email |

> **`/scan_filtered` is critical.** The RPLidar A1 is mounted ~180° vs the URDF;
> the filter keeps only the clean front ~220° arc and drops the rear
> antenna/wire cone that otherwise wrecks scan matching. **AMCL and both Nav2
> costmaps consume `/scan_filtered`, never `/scan`.** Companion fix:
> `laser_joint` yaw = π in `yahboomcar_X3.urdf`.

### Layer 2 — Localization  (`localization.launch.py`)

| Node | Subscribes | Publishes |
|---|---|---|
| `map_server` | — | `/map` (`living_room_v1`) |
| `amcl` | `/scan_filtered`, `/map`, TF | `/amcl_pose`, `/particle_cloud`, **TF `map→odom`** |
| `lifecycle_manager_localization` | manages `map_server`, `amcl` | — |

AMCL **seeds itself at home** (`set_initial_pose: true`, `initial_pose =
0.331, -0.086, 0.043` in `amcl_params.yaml`) — no RViz "2D Pose Estimate"
needed. RViz can still override anytime by publishing `/initialpose`.

### Layer 3 — Navigation (Nav2)  (`navigation.launch.py`)

| Node | Role | Key I/O |
|---|---|---|
| `controller_server` | local planner (MPPI/Omni) | `/scan_filtered`, `/odom` → **`/cmd_vel`** |
| `planner_server` | global planner (NavFn) | `/map`, `/scan_filtered` |
| `behavior_server` | recoveries (spin/backup/wait) | — |
| `bt_navigator` | serves **`navigate_to_pose`** action | — |
| `waypoint_follower` | multi-goal helper | — |
| `lifecycle_manager_navigation` | manages all Nav2 nodes | — |

### Layer 4 — Brain  (`patrol_manager.launch.py`)

| Node | Subscribes | Publishes / Acts |
|---|---|---|
| `patrol_manager` | `/camera/color/image_raw`, `/cmd_vel`, `/vel_raw`, `/cat_detector_cpp/detections`, **`/cat_patrol/audio_ready`** (when voice on) | `/cat_patrol/mail_request`, `/patrol_manager/state`, `/patrol_manager/waypoints` (latched markers); **action client → `navigate_to_pose`** |

This is **the brain**. (The older odom `patrol_node` in `cat_patrol_robot`
still exists and has the same mail features, but is NOT started here — two
brains must never both drive `/cmd_vel`. Hence `start_patrol_node:=false`.)

### Layer 5 — Detection + voice  (`cat_voice.launch.py`)

| Node / process | Toggle | Subscribes | Publishes / Effect |
|---|---|---|---|
| `cat_detector_cpp` | `use_detector` | `/camera/color/image_raw` | `/cat_detector_cpp/detections`, `/cat_detector_cpp/image_annotated`, `/Buzzer` (beep) |
| `cat_voice` (voice_node) | `use_voice` | `/cat_detector_cpp/detections` | plays `~/cats/voices/{jente,arik}-*.mp3` via `ffmpeg | paplay`; on start plays `ready-chime.mp3` and latches **`/cat_patrol/audio_ready`** |
| `connect_usb_speaker.sh` | `use_voice` (early one-shot) | — | sets PulseAudio default sink to USB voice card **0c76:1203** JMTek “USB PnP Audio Device” (chime probe deferred to `cat_voice`). Legacy BT: `connect_bt_speaker.sh` / `start_bt:=true` |

`detections[i].results[0]` = generic "cat" (YOLO); `results[1]` = per-cat
identity `white`/`brown` + confidence (present only when the classifier fired).
`white → jente-*`, `brown → arik-*`.

---

## 3. System diagram (data flow)

Read each block as "PRODUCER -- topic --> CONSUMER". `patrol_manager` is **one
node** (in the `patrol_manager` package) with four jobs: snapshots, stall guard,
email, and navigation client.

```
CAMERA
  astra_camera --/camera/color/image_raw--> cat_detector_cpp
  astra_camera --/camera/color/image_raw--> patrol_manager

PERCEPTION
  cat_detector_cpp --/..._cpp/detections--> cat_voice
  cat_detector_cpp --/..._cpp/detections--> patrol_manager
  cat_detector_cpp --/Buzzer (beep)-------> driver

BRAIN  (patrol_manager = snapshots+stall+email+nav)
  patrol_manager --action navigate_to_pose--> Nav2
  patrol_manager --/cat_patrol/mail_request-> mail_node -> SMTP

NAVIGATION
  Nav2 --/cmd_vel--> driver --> wheels
  Nav2 consumes: /scan_filtered, /odom, TF map->base

ODOMETRY + LOCALIZATION  (builds the TF chain Nav2 needs)
  driver     --/vel_raw-------> base_node_X3
  driver     --/imu/data_raw--> EKF
  base_node  --/odom_raw------> EKF
  EKF        --/odom + TF odom->base_footprint-->
  sllidar    --/scan---------> scan_front_filter
  scan_filt  --/scan_filtered-> amcl, Nav2
  map_server --/map----------> amcl
  amcl       --TF map->odom-->
  driver     --/vel_raw,/cmd_vel--> patrol_manager (stall)

  driver = Mcnamu_driver_X3
```

### TF tree

```
map
 +- odom               (amcl:  map->odom)
    +- base_footprint  (EKF:   odom->base_footprint)
       +- base_link    (robot_state_publisher / URDF)
          +- laser_link    (URDF; yaw=pi vs mount)
          +- camera_link   (URDF)
```

---

## 4. Patrol FSM (`patrol_manager`)

```
  SLEEPING
     | wake timer fires
     v
  WAKING          (wait Nav2 server; if use_voice also wait /cat_patrol/audio_ready
                   after ready-chime — fail-closed timeout → SLEEPING)
     | loop-START email + send first waypoint goal
     v
  PATROLLING      (Nav2 drives to waypoint)
     | arrived
     v
  CAPTURE         (snapshot + waypoint email)
     |
     +- more waypoints --> PATROLLING
     +- last waypoint  --> RETURNING
                             | arrived home (loop-END email)
                             v
                           SLEEPING
                             (re-arm after patrol_pause_sec)

  STALL guard (in PATROLLING / RETURNING):
     wheels idle while commanded
     --> stall-alert email, go SLEEPING
```

- **WAKING** waits (non-blocking) for the `navigate_to_pose` server, then sends the first goal.
- **CAPTURE** saves one snapshot per waypoint and emails it immediately.
- **Stall guard** (active in PATROLLING/RETURNING): if `/cmd_vel` commands motion
  but `/vel_raw` shows the wheels aren't turning for `stall_timeout_sec` (6 s) →
  cancel goal, email a STALL alert, go SLEEPING.

---

## 5. Cat recognition + email events

All emails go out as JSON on `/cat_patrol/mail_request` → `mail_node` → SMTP.

| Event | Trigger | Attachment |
|---|---|---|
| Waypoint capture | arrived at a waypoint (CAPTURE) | that waypoint's photo |
| **Cat recognized** | `white`/`brown` in `results[1]` ≥ `min_conf`, **during a cycle**, per-cat cooldown | one snapshot |
| **Loop start** | entering WAKING (new cycle) | one snapshot |
| **Loop end** | returned home successfully | one snapshot |
| Stall alert | stall guard fired | none |

Tunables in `patrol_manager_params.yaml`: `cat_recognized_mail_enabled`,
`cat_recognized_min_conf` (0.6), `cat_recognized_cooldown_sec` (30),
`loop_boundary_mail_enabled`. SMTP creds come from `CAT_PATROL_SMTP_*` env
(missing → `mail_node` logs and skips; patrol still runs).

The cat identity stream (`/cat_detector_cpp/detections`) has **three**
independent consumers: `cat_voice` (audio), `patrol_manager` (email), and the
detector's own `/Buzzer` beep.

---

## 6. Startup order & timing (`patrol_system.launch.py`)

| t (s) | Started | Depends on |
|---|---|---|
| 0 | camera, bringup (driver/base/EKF/RSP), lidar + `scan_front_filter`, `mail_node` | hardware |
| 1 | `connect_usb_speaker` (only if `use_voice`) | PulseAudio; overlaps bringup |
| 6 | `localization` (AMCL@home), `navigation` (Nav2) | odom/TF + `/scan_filtered` must exist first |
| 8 | RViz (only if `use_rviz:=true`) | — |
| 12 | `patrol_manager` (gates on audio if voice), `cat_detector_cpp`, `cat_voice` (ready-chime → `/cat_patrol/audio_ready`) | camera; Nav2 action server; USB sink for audio gate |

Delays are conservative safety margins; `patrol_manager` additionally polls for
the Nav2 action server, so exact timing is soft.

---

## 7. Launch toggles

```bash
patrol start                              # full stack (detector + voice + USB speakers)
patrol start use_voice:=false             # silent: detector ON, no audio/USB setup
patrol start use_detector:=false          # no detection → no cat-recognized email
patrol start use_rviz:=true               # also open RViz (pose override / monitor)
patrol start start_lidar:=false           # skip lidar (also skips scan_front_filter)
```

`use_detector` and `use_voice` are independent because the detector feeds BOTH
the voice line and the cat-recognized email; you can keep recognition/email
while silencing audio.

**Audio hardware (production):** USB voice card **JMTek / Solid State System
“USB PnP Audio Device”**, USB ID **`0c76:1203`**, Pulse sink name contains
`USB_PnP_Audio_Device`. ALSA card id often `Device` (`plughw:Device,0` /
`/dev/snd/by-id/usb-Solid_State_System_Co._Ltd._USB_PnP_Audio_Device_...`).
Bluetooth (`connect_bt_speaker.sh`) remains installed but is **off by default**.

---

## 8. Mapping to the legacy manual sequence

`patrol start` reproduces, in one process group, what was previously ten
scripts run by hand in `~/myscripts2/`:

| Old script | Now part of |
|---|---|
| `t1.sh` bringup (`/dev/myserial`, `use_joystick:=false`) | Layer 1 bringup |
| `t2.sh` sllidar | Layer 1 lidar |
| `t2.5.sh` scan_front_filter (`trusted_center=π`, `halfwidth=1.92`) | Layer 1 `scan_front_filter` |
| `t3_amcl.sh` | Layer 2 localization |
| `t4_nav.sh` (+ `stop ollama`, `xhost`) | Layer 3 navigation (script does ollama/xhost best-effort) |
| `t5_patrol.sh` | Layer 4 patrol_manager |
| `t6_camera.sh` (sources `library_ws`) | Layer 1 camera (script sources `library_ws`) |
| `t7_mail.sh` | Layer 1 mail_node |
| `t9_detector_cpp.sh` / `t10_cat_voice.sh` | Layer 5 detector + voice |
| `t8_detector.sh` (Python `cat_detector`) | **not used** — superseded by `cat_detector_cpp` |

---

## 9. Gotchas / invariants

- **`/scan_filtered`, not `/scan`** feeds AMCL + Nav2. If lidar is off, that
  publisher is off too and localization/obstacle-avoidance get no laser.
- **`/dev/myserial`** for the MCU (stable udev alias). The raw by-id is
  ambiguous between two CH340s and can yield a zero-IMU/free-fall reading.
- **Domain 28 + CycloneDDS**: run any manual `ros2` command against this stack
  with the same env, or it won't see the graph. `patrol status` sets it for you.
- **One brain only**: `patrol_manager` drives `/cmd_vel`; the odom `patrol_node`
  is kept out of this stack.
- **Home vs map**: `initial_pose` in `amcl_params.yaml` must match
  `home_pose` in `patrol_manager_params.yaml` (both `0.331, -0.086, 0.043` on
  `living_room_v1`). Re-map the room → update both.
- **Audio gate (use_voice)**: robot must not leave home until `cat_voice` plays
  `~/cats/voices/ready-chime.mp3` through the USB voice card (Pulse default sink
  matching `USB_PnP_Audio_Device`) and latches `/cat_patrol/audio_ready`.
  Timeout 90s → fail-closed (SLEEPING). Silent runs: `use_voice:=false`.
  Manual check: `bash …/connect_usb_speaker.sh` then listen for ready-chime.
