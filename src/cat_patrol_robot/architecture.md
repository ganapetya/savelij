# Cat Patrol Robot — System Architecture

End-to-end view of how the cat-patrol stack fits together: hardware on
the bottom-left, the EKF and TF tree in the middle, SLAM / Nav2 at the
top-right, and command flow looping back down to the wheels.

---

## Diagram

```mermaid
flowchart TB

    %% ----------------------------------------------------------------
    %% LAYER 1 — Physical hardware
    %% ----------------------------------------------------------------
    subgraph HW["🔧 Hardware (physical)"]
        direction LR
        WHEELS["4× Motors<br/>mecanum drive"]
        ENCS["4× Encoders<br/>in motors"]
        IMUC["MPU-9250 IMU chip<br/>on Rosmaster board"]
        LIDARHW["SLAMTEC RPLiDAR A1<br/>spinning turret"]
        CAMHW["Orbbec Astra+<br/>RGB + IR + depth"]
        BAT["3S LiPo battery"]
    end

    %% ----------------------------------------------------------------
    %% LAYER 2 — MCU firmware
    %% ----------------------------------------------------------------
    FWPID[["📦 MCU Firmware (STM32 on Rosmaster board)<br/>━━━━━━━━━━━━━━━━━━━━━━━━━━━━<br/>• Per-wheel PID @ ~1 kHz<br/>• Mecanum kinematics<br/>• Reads MPU-9250 over I²C<br/>• Reads battery voltage<br/>• Binary protocol over USB-serial"]]

    %% ----------------------------------------------------------------
    %% LAYER 3 — ROS 2 driver nodes (on Jetson)
    %% ----------------------------------------------------------------
    subgraph DRV["⚙️ ROS 2 driver nodes (Jetson)"]
        MC["Mcnamu_driver_X3.py<br/>(Python, owns /dev/ttyUSB1)"]
        TRIM[/"cmd_vel_callback<br/>applies trim_vy_per_vx<br/>before set_car_motion"/]
        SLLIDAR["sllidar_node<br/>(C++, owns /dev/ttyUSB0)"]
        ASTRA["astra_camera_node<br/>(C++, owns USB UVC)"]
    end

    %% ----------------------------------------------------------------
    %% LAYER 4 — Filtering / integration
    %% ----------------------------------------------------------------
    subgraph PROC["🧮 Filters & integrators"]
        MADG["imu_filter_madgwick<br/>gyro+accel → orientation"]
        BASE["base_node_X3<br/>(C++) Euler-integrates<br/>/vel_raw → /odom_raw"]
    end

    %% ----------------------------------------------------------------
    %% LAYER 5 — EKF fusion (robot_localization)
    %% ----------------------------------------------------------------
    EKF[["🟢 ekf_filter_node<br/>(robot_localization)<br/>━━━━━━━━━━━━━━━━━━━━<br/>fuses /odom_raw + /imu/data<br/>→ publishes /odom + TF"]]

    %% ----------------------------------------------------------------
    %% LAYER 6 — TF infrastructure
    %% ----------------------------------------------------------------
    subgraph TFP["📐 TF publishers"]
        RSP["robot_state_publisher<br/>static URDF TFs<br/>(/tf_static)"]
        JSP["joint_state_publisher<br/>wheel-joint TFs"]
    end

    TFTREE[("📡 /tf + /tf_static<br/>━━━━━━━━━━━━━━━━<br/>map → odom →<br/>base_footprint →<br/>base_link →<br/>{laser_link, imu_link,<br/>camera_link, wheels}")]

    %% ----------------------------------------------------------------
    %% LAYER 7 — Mapping / localization / navigation (Phase 1+)
    %% ----------------------------------------------------------------
    subgraph P1PLUS["🗺️ Phase 1+ (not running yet in Phase 0)"]
        SLAM["slam_toolbox<br/>(Phase 1: build a map)"]
        AMCL["nav2_amcl<br/>(Phase 2: localize on saved map)"]
        NAV2["Nav2 stack<br/>━━━━━━━━━━━━<br/>• planner_server<br/>• controller_server<br/>• costmap_2d (×2)<br/>• behavior_server<br/>• lifecycle_manager<br/>(Phase 3)"]
    end

    %% ----------------------------------------------------------------
    %% LAYER 8 — Command sources
    %% ----------------------------------------------------------------
    subgraph CMDS["🎮 Command sources"]
        TELEOP["teleop_twist_keyboard<br/>(human in the loop)"]
        PATROL["patrol_node<br/>(existing FSM, Phase 0)<br/>OR<br/>patrol_manager<br/>(Phase 4 rewrite)"]
    end

    %% ----------------------------------------------------------------
    %% LAYER 9 — Phase 0 diagnostic (passive observer)
    %% ----------------------------------------------------------------
    ODC["🔍 odom_drift_checker<br/>(Phase 0 diagnostic, observes /odom + TF)"]

    %% ================================================================
    %% SIGNAL FLOWS
    %% ================================================================

    %% --- physical / electrical ---
    ENCS -- "encoder pulses<br/>(electrical)" --> FWPID
    IMUC -- "I²C bus<br/>(internal)" --> FWPID
    FWPID -- "PWM duty per motor" --> WHEELS
    WHEELS -. "shaft rotation" .-> ENCS
    BAT -. "12 V power" .-> FWPID

    %% --- MCU ↔ chassis driver over USB-serial ---
    FWPID -- "binary protocol<br/>(USB-serial /dev/ttyUSB1)" --> MC
    TRIM -- "set_car_motion(vx,<br/>vy_corrected, w_corrected)" --> FWPID
    MC --- TRIM

    %% --- Mcnamu_driver outputs (10 Hz) ---
    MC -- "/vel_raw<br/>(Twist)" --> BASE
    MC -- "/imu/data_raw" --> MADG
    MC -. "/imu/mag" .-> MADG
    MC -- "/joint_states" --> JSP
    MC -. "/voltage" .-> Future["(future)<br/>battery monitor<br/>Phase 7"]

    %% --- LiDAR ---
    LIDARHW -- "USB-serial<br/>(/dev/ttyUSB0)" --> SLLIDAR
    SLLIDAR -- "/scan<br/>(LaserScan)" --> SLAM
    SLLIDAR -- "/scan" --> AMCL
    SLLIDAR -- "/scan" --> NAV2

    %% --- camera ---
    CAMHW -- "USB UVC" --> ASTRA
    ASTRA -- "/camera/depth/*<br/>/camera/color/*" --> NAV2

    %% --- pre-EKF filtering ---
    MADG -- "/imu/data<br/>(filtered)" --> EKF
    BASE -- "/odom_raw" --> EKF

    %% --- EKF outputs ---
    EKF -- "/odom<br/>(fused)" --> SLAM
    EKF -- "/odom" --> AMCL
    EKF -- "/odom" --> NAV2
    EKF -- "/odom" --> ODC

    %% --- TF tree contributions ---
    EKF -- "odom → base_footprint" --> TFTREE
    RSP -- "base_footprint → base_link<br/>→ {sensors, wheels}" --> TFTREE
    JSP -- "wheel rotation TFs" --> TFTREE
    SLAM -. "map → odom<br/>(once active)" .-> TFTREE
    AMCL -. "map → odom<br/>(once active)" .-> TFTREE

    %% --- TF consumers ---
    TFTREE -. "TF lookups" .-> SLAM
    TFTREE -. "TF lookups" .-> AMCL
    TFTREE -. "TF lookups" .-> NAV2
    TFTREE -. "TF lookups" .-> ODC

    %% --- map flow ---
    SLAM -- "/map" --> NAV2
    AMCL -- "/map" --> NAV2

    %% --- command flow back to wheels ---
    TELEOP -- "/cmd_vel" --> TRIM
    PATROL -- "/cmd_vel" --> TRIM
    NAV2 -- "/cmd_vel" --> TRIM

    %% ================================================================
    %% Styling
    %% ================================================================
    classDef hardware fill:#fff3e0,stroke:#e65100,stroke-width:2px,color:#000
    classDef firmware fill:#fce4ec,stroke:#c2185b,stroke-width:2px,color:#000
    classDef driver   fill:#e3f2fd,stroke:#1565c0,stroke-width:2px,color:#000
    classDef proc     fill:#f3e5f5,stroke:#6a1b9a,stroke-width:2px,color:#000
    classDef ekf      fill:#c8e6c9,stroke:#1b5e20,stroke-width:3px,color:#000
    classDef tf       fill:#fffde7,stroke:#f57f17,stroke-width:2px,color:#000
    classDef phase1   fill:#ede7f6,stroke:#4527a0,stroke-width:2px,color:#000,stroke-dasharray:5 3
    classDef cmd      fill:#fff8e1,stroke:#ff6f00,stroke-width:2px,color:#000
    classDef diag     fill:#efebe9,stroke:#3e2723,stroke-width:1px,color:#000

    class WHEELS,ENCS,IMUC,LIDARHW,CAMHW,BAT hardware
    class FWPID firmware
    class MC,TRIM,SLLIDAR,ASTRA driver
    class MADG,BASE proc
    class EKF ekf
    class RSP,JSP,TFTREE tf
    class SLAM,AMCL,NAV2 phase1
    class TELEOP,PATROL cmd
    class ODC,Future diag
```

---

## Reading the diagram

### Layers, top → bottom

| Color | Layer | What lives there |
|---|---|---|
| 🟧 Orange | **Hardware** | Real silicon and metal: motors, encoders, IMU chip, lidar turret, camera, battery |
| 🟪 Pink | **MCU firmware** | Closed-source STM32 firmware on the Rosmaster board. Per-wheel PID, mecanum kinematics, sensor reading. |
| 🟦 Blue | **ROS 2 driver nodes** | Talk to hardware over USB-serial / UVC. Translate to/from ROS messages. |
| 🟪 Purple | **Filters & integrators** | `imu_filter_madgwick` (orientation from gyro+accel) and `base_node_X3` (Euler-integrate `/vel_raw` → `/odom_raw`). |
| 🟢 Green | **EKF fusion** | The hub: `robot_localization`'s `ekf_filter_node` fuses `/odom_raw` + `/imu/data` → `/odom` and the `odom → base_footprint` TF. |
| 🟨 Yellow | **TF infrastructure** | `robot_state_publisher` (static URDF TFs), `joint_state_publisher` (wheel rotations), and the live `/tf` + `/tf_static` data. |
| 🟪 Indigo (dashed border) | **Phase 1+ stack** | `slam_toolbox`, `nav2_amcl`, Nav2. **Not running in Phase 0** — that's why the border is dashed. |
| 🟨 Amber | **Command sources** | Where `/cmd_vel` originates: keyboard teleop, the legacy `patrol_node`, and (eventually) Nav2. |
| ⬜ Gray | **Phase 0 diagnostic** | `odom_drift_checker` — passive observer of `/odom` + TF, no commands published. |

### Arrow types

- **Solid arrow with topic name** — a normal ROS 2 publish/subscribe edge (e.g. `MC --"/vel_raw"--> BASE`).
- **Dotted arrow** — either a TF lookup (read by a consumer) or a future / optional connection that's wired but not exercised in Phase 0.
- **Solid no-label edges between hardware nodes** — physical / electrical / I²C connections that aren't ROS topics (encoder pulses, motor PWM, internal I²C bus).

### Two flow directions to notice

1. **Sensor data flows up the diagram**: hardware → MCU → driver → filter → EKF → SLAM/Nav2/diagnostic. Every layer adds processing or fusion; the top layers consume the result.
2. **Commands flow down**: teleop / patrol / Nav2 publish `/cmd_vel` → `cmd_vel_callback` (where the trim is applied) → `set_car_motion` → MCU firmware → motor PWM → wheels.

The system is a **ring**: sensor data goes up, navigation decisions come back down, the wheels move, and the cycle repeats at 10 Hz (the rate of the slowest topics, `/odom`, `/imu/data`, `/vel_raw`).

### A few subtleties worth knowing

- **`/vel_raw` is NOT raw encoders.** It's the firmware's own chassis-frame velocity report (after mecanum kinematics + IMU integration inside the MCU). Per-wheel encoder counts are accessed separately via `Rosmaster_Lib.get_motor_encoder()` (used by `wheel_balance_diagnostic.py`, not by the running stack).
- **`/odom_raw` is wheel-only**, `/odom` is wheel + IMU fused. Always subscribe to `/odom`. The drift checker, SLAM, AMCL, and Nav2 all do.
- **The TF tree is collaboratively built.** The EKF publishes only one link (`odom → base_footprint`); `robot_state_publisher` adds the URDF static frames; `joint_state_publisher` adds wheel rotations; `slam_toolbox` / `amcl` (when running) add the `map → odom` correction. Four publishers, one shared `/tf` channel.
- **The trim (`cmd_vel_callback`) sits at the only chokepoint between ROS and hardware.** Any source of `/cmd_vel` — teleop, patrol, Nav2 — gets the trim applied for free. That's why the trim works in Phase 1+ even though we never had to wire it explicitly into SLAM or Nav2.

---

## How to render this diagram

The Mermaid source above renders in:

- **GitHub / GitLab / Bitbucket** Markdown viewers (commit and push, click the file)
- **VS Code** with the Markdown Preview Mermaid Support extension
- **mermaid.live** — paste the source, get instant SVG/PNG, export
- **Cursor** — depends on UI version; usually works in Markdown preview pane

If you want a static image to embed in a presentation, the easiest path is to paste into mermaid.live, click Export → SVG/PNG.
