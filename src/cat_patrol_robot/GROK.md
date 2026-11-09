# GROK.md — Working memo for Grok (Savelij / cat patrol)

> Created 2026-07-24 when ownership moved Claude → Grok.
> Keep this short and accurate. Update the **Session log** and **Current
> focus** when something material changes.

**Main architecture (authoritative runtime view):**
[`loop-patrol-architecture.md`](loop-patrol-architecture.md)

**Historical hardware→EKF diagram (still useful, less operational):**
[`architecture.md`](architecture.md)

**Plan / status trail:**
[`plan.md`](plan.md) · [`phase0-status.md`](phase0-status.md) …
[`phase5-status.md`](phase5-status.md)

Workspace-level agent rules: `yahboomcar_ws/AGENTS.md`.

---

## 1. What this robot is

Indoor autonomous patrol on **Yahboom ROSMASTER X3** + **Jetson Orin NX 8GB**,
ROS 2 Humble, robot name **Savelij**.

Working product today:

1. One command: `patrol start`
2. Localizes on map `living_room_v1` (AMCL seeded at home)
3. Drives Nav2 waypoint loop (`wp1`…`wp10` → home)
4. Photos + email at waypoints / loop boundaries / recognized cats
5. Sees cats via `cat_detector_cpp` (YOLO + white/brown classifier)
6. Plays per-cat mp3 lines via **USB voice card** speakers (`cat_voice`; BT legacy off)

**Not implemented yet (Phase 6):** cancel active `NavigateToPose` on cat,
enter `INVESTIGATING`, nav-toward-cat, then resume interrupted waypoint.
Detection currently runs **in parallel** with patrol (voice + mail only).

---

## 2. Where you are in the plan

| Phase | Status | Notes |
|---|---|---|
| 0 Foundation / odom | **Done** | Trim, square/out-back tests, EKF TF clean |
| 1 Map | **Done** | `maps/living_room_v1.{pgm,yaml}` |
| 2 AMCL | **Done** | Seeds at home; `/scan_filtered` only |
| 3 Nav2 | **Done** | Omni/MPPI path; action client patterns |
| 4 patrol_manager | **Done** | Looping, stall guard, mail, captures |
| 5 Cat detection | **End of phase / production path up** | C++ detector + ID + voice. 5b dataset small (test acc ~82%); live Astra ID still worth hardening |
| 6 See cat → interrupt patrol | **Not started** | User is **not** starting this yet |
| 7 Polish | Later | Battery, systemd, dashboard, logs |

**User intent (2026-07-24):** stay on post–Phase-5 hardening — improve design
and fix bugs. Do not start Phase 6 unless asked.

---

## 3. One-command ops

```bash
patrol preflight
patrol start                 # full stack: detector + voice + BT
patrol start use_voice:=false
patrol start use_detector:=false
patrol start use_rviz:=true
patrol status
patrol logs
patrol stop                  # kills whole process group
```

- Binary: `src/cat_patrol_robot/scripts/patrol` (also `~/myscripts2/patrol`)
- Launch root: `patrol_system.launch.py` (staggered layers — see loop-patrol-architecture §6)
- Logs: `~/cat_patrol_data/logs/`
- PID: `$XDG_RUNTIME_DIR/cat_patrol/patrol.pid`

Legacy manual scripts `~/myscripts2/t1.sh`…`t10_*.sh` still exist; `patrol`
replaces the ten-step sequence. Prefer `patrol`.

---

## 4. Runtime env (must match always)

| Variable | Value |
|---|---|
| `ROS_DOMAIN_ID` | **28** |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` |
| `CYCLONEDDS_URI` | `file:///home/jetson/cyclonedds.xml` |

Source order:

1. `/opt/ros/humble/setup.bash`
2. `~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash`  ← **astra_camera**
3. `~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash`

Missing `library_ws` ⇒ camera package not found.

---

## 5. Live stack layers (short)

```
Layer 1  cat_patrol.launch   camera, Mcnamu_driver, base, madgwick, EKF,
                             RSP, sllidar, scan_front_filter, mail_node
Layer 2  localization        map_server + amcl (map→odom)
Layer 3  navigation          Nav2 (cmd_vel out)
Layer 4  patrol_manager      brain: FSM + NavigateToPose client
Layer 5  cat_voice.launch    cat_detector_cpp + cat_voice + USB voice card
```

**TF:** `map → odom (amcl) → base_footprint (EKF) → base_link → laser/camera`

**Critical filters / devices:**

- `/scan_filtered` only for AMCL + costmaps (lidar mount ~180° vs URDF;
  `laser_joint` yaw=π; trusted arc center π, half-width 1.92)
- MCU: `/dev/myserial` (not raw ambiguous CH340 by-id)
- Lidar: Silicon Labs CP2102 by-id in preflight

---

## 6. Key packages & paths

```
yahboomcar_ws/src/
  cat_patrol_robot/     launches, maps, mail, voice, scan filter, legacy patrol_node
  patrol_manager/       brain (C++)
  cat_detector_cpp/     live vision (C++, ONNX + OpenCV CUDA)
  cat_detector/         Python vision (reference)
  odom_drift_checker/
  yahboomcar_*          vendor chassis / nav / etc.
```

Data on disk:

| Path | Content |
|---|---|
| `~/cat_patrol_data/captures/` | Waypoint photos |
| `~/cat_patrol_data/detections/` | Python detector saves |
| `~/cat_patrol_data/detections_cpp/` | C++ detector saves |
| `~/cat_patrol_data/dataset/` | Labeled train data |
| `~/cat_patrol_data/logs/` | patrol run logs |
| `~/cats/white`, `~/cats/brown` | Manual photo sets |
| `~/cats/voices/` | `jente-*.mp3` (white), `arik-*.mp3` (brown) |

Params worth knowing:

- `patrol_manager/config/patrol_manager_params.yaml` — home, waypoints, mail, stall
- `cat_patrol_robot/config/amcl_params.yaml` — initial pose
- `cat_patrol_robot/config/nav2_params.yaml` — planners / costmaps
- `cat_detector_cpp/config/cat_detector_cpp_params.yaml`
- `cat_patrol_robot/config/cat_voice_params.yaml` (if present)

---

## 7. Detection + voice contract

`/cat_detector_cpp/detections` (`vision_msgs/Detection2DArray`):

- `results[0]` — generic cat (YOLO / class 15)
- `results[1]` — optional identity: `class_id` ∈ {`white`,`brown`}, score

Consumers (independent):

1. `cat_voice` — mp3 via `ffmpeg | paplay` (Pulse default sink = USB PnP Audio Device)
2. `patrol_manager` — cat-recognized email (cooldown 30s, min_conf 0.6)
3. Detector itself — short `/Buzzer` beep

Cat names in code stay **white/brown** (not personal names). Audio branding:
white→Jente lines, brown→Arik lines.

Classifier: MobileNetV2 head, small dataset (~75 images), ~81.8% on 11-image
held-out set; all errors brown→white. Treat ID conf as soft; improve with more
flywheel data rather than over-trusting labels.

---

## 8. patrol_manager FSM (current)

```
SLEEPING → WAKING → PATROLLING(i) → CAPTURE(i) → …
                              ↘ last wp → RETURNING → SLEEPING
```

- Stall guard in PATROLLING/RETURNING: cmd_vel high + vel_raw near zero ≥ 6s
  → cancel, stall email, SLEEPING
- `loop_patrol: true`, `patrol_pause_sec: 10`
- No `INVESTIGATING` state yet (Phase 6)

---

## 9. Hardware realities (don’t re-learn the hard way)

- **Chassis curves right** without software trim (`trim_vy_per_vx ≈ 0.012` at
  0.15 m/s in Mcnamu driver). Odom is honest; robot is biased.
- **Mecanum + mixed floors:** in-place spin on smooth floor slips; `/odom`
  under-reports translation. Prefer arcing turns on laminate; AMCL corrects pose.
- **Orin NX 8GB is tight.** Full stack + YOLO is viable (measured ~+12pp CPU,
  GPU ~60% with detector) but watch RAM; `t4_nav` historically stops `ollama`.
- **Camera orphans:** killing only the launch parent leaves `astra_camera_node`
  holding USB → next start “Resource busy”. Use `patrol stop` / kill process group.
- **`/tmp` is ephemeral** — never store patrol photos there.

More detail: [`hardware.md`](hardware.md), Phase 0 status.

---

## 10. Build notes

```bash
export ROS_DOMAIN_ID=28
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds.xml
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
colcon build --symlink-install --packages-select patrol_manager
# or: cat_detector_cpp, cat_patrol_robot, ...
source install/setup.bash
```

Many launch files load **src-tree YAML by absolute path** (symlink-install
friendly). Changing params often needs no rebuild — restart the node/stack.

---

## 11. Current focus (improve & fix — not Phase 6)

Default work queue when user says “improve / fix bugs”:

1. Reproduce from `patrol logs` / `ros2 topic` with domain 28 env
2. Prefer minimal diffs; one brain, one detector, don’t dual-start stacks
3. Touch vendor Yahboom code only when necessary; prefer overlays/params here
4. After a fix: smoke on bench if motion, note outcome in Session log below
5. If a bug is really “Phase 6 territory,” say so and wait for go-ahead

Known soft spots to watch (not necessarily open bugs):

- Small 5b dataset → ID mistakes (esp. brown→white)
- USB voice card present after reboot / Pulse default sink (was BT pairing)
- SMTP only if `CAT_PATROL_SMTP_*` set; missing → mail_node skips, patrol continues
- Continuous YOLO cost vs long unattended multi-cycle runs
- Home/AMCL pose must stay in sync if map is remade

---

## 12. Session log

### 2026-07-24 — Grok onboarding

- Read `loop-patrol-architecture.md` (primary), `architecture.md`, `plan.md`,
  phase5/4 status, `patrol` script, `patrol_manager_params.yaml`, `voice_node.py`.
- User: at end of Phase 5; robot waypoint-patrols and plays mp3 on cats;
  **Phase 6 not started**; work is design polish + bugs.
- Created `AGENTS.md` (workspace root) + this `GROK.md`.

### 2026-07-24 — White-background bias on identity

- Root cause: tiny MobileNet ID head is color-biased; YOLO box + old +15% pad
  pulled in white walls/floor so brown cats (and white pillows) became "white".
  Example: `detections_cpp/cat_20260724_114400_324_white.jpg` — brown tabby
  ear-only at left edge, rest of frame white wall.
- Live mitigations (after white-cat regression fix):
  - mild crop: `id_box_shrink=0.90`, `id_pad_frac=0.08` (not 0.75/0.0)
  - softmax margin `id_min_margin=0.12`
  - veto "white" only if brown_fur_frac ≥ 0.22 (not 0.08)
  - **near-white BG veto DISABLED** (`id_white_veto_bg_frac: 1.01`) — it
    blocked real white cats (white fur ≈ white wall pixels)
- Longer-term: retrain with brown-on-white hard examples + hard negatives
  (pillows/walls); optional third class `none`.

### 2026-07-24 — Identity flip-flop (white+brown same second)

- Symptom: one brown cat → voice/mail for white@0.81 then brown@0.67 ~250ms later.
- Cause: MobileNet ID head flip-flops frame-to-frame; detector published every
  raw label; voice/mail cooldowns were **per-label** so both fired.
- Fix:
  1. `cat_detector_cpp`: `id_min_conf` + `id_stable_frames` lock + switch margin;
     only publish `results[1]` once locked.
  2. `cat_voice` + `patrol_manager`: global any-cat cooldown (not only per label).

### 2026-07-24 — BT audio ready gate (superseded by USB, 2026-07-28)

- Problem: first 20–30s of patrol, cat voice “plays” but BT/A2DP not ready.
- Fix (fail-closed, only when `use_voice`):
  1. Early sink setup at t≈1s (sink only; skip chime).
  2. `cat_voice` waits for matching default sink, plays `~/cats/voices/ready-chime.mp3`,
     latches `/cat_patrol/audio_ready`.
  3. Suppresses cat mp3 until ready.
  4. `patrol_manager` `require_audio_ready` (set true by `patrol_system` when voice on):
     WAKING waits for topic; 90s timeout → SLEEPING, no drive.

### 2026-07-28 — USB voice card replaces Bluetooth

- Hardware: JMTek / Solid State **USB PnP Audio Device** `0c76:1203` on
  USB path `1-2.4.4` (hub port). Pulse sink:
  `alsa_output.usb-Solid_State_System_Co._Ltd._USB_PnP_Audio_Device_…analog-stereo`.
- Production path now:
  1. Early `connect_usb_speaker.sh` (default sink + volume; skip chime).
  2. `cat_voice` `ready_sink_match: USB_PnP_Audio_Device` (not bluez).
  3. BT scripts kept (`connect_bt_speaker.sh`, `start_bt:=true`) but off by default.
- Verified: `paplay` / `ffmpeg|paplay` of ready-chime + jente line exit 0 on USB sink.
- Files: `connect_usb_speaker.sh`, `voice_node.py`, `cat_voice.launch.py`,
  `patrol_system.launch.py`, `cat_voice_params.yaml`, architecture/GROK.

### 2026-07-24 — Empty-room white FP + brown (arik) miss

- Evidence: `detections_cpp` saved **shoes/slippers with no cat** as sightings
  (e.g. `cat_20260724_190202_821.jpg`, `…_190520_799.jpg`). YOLO n@0.45 is
  loose indoors; classifier then often locks **white** (wall bias) → jente voice.
  Day stats ~149 white-labeled saves vs ~39 brown. False jente also burns the
  **global** voice cooldown and can silence a real arik line right after.
- Brown *does* sometimes lock (`…_194815_896_brown.jpg`); “never arik” is often
  ID bias + cooldown, not total YOLO failure.
- Live fix (built `cat_detector_cpp`):
  - YOLO `conf_threshold` **0.45 → 0.58**
  - Asymmetric ID floors: `id_min_conf_white=0.80`, `id_min_conf_brown=0.55`,
    default `id_min_conf=0.70`, `id_stable_frames=3`, `id_min_margin=0.15`,
    `id_min_box_area_px=2800`
  - Sighting log includes `yolo=` score for tuning
  - Voice: `min_identity_conf=0.55`, `cooldown_sec=12`
- Restart stack to pick up binary + YAML. Longer-term: retrain ID with hard
  negatives (shoes/empty) and more brown-on-white crops; optional `none` class.

*(Append new entries below with date + short bullets.)*
