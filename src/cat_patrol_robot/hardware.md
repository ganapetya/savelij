# 🤖 Savelij — Hardware Reference

> **Platform:** Yahboom ROSMASTER X3 (Superior Edition)
> **Compute:** NVIDIA Jetson Orin NX 8 GB — **Ubuntu 22.04.5 LTS, JetPack 6.1 (Jetson Linux 36.4.3)**
> **Purpose:** Autonomous cat patrol robot — SLAM, Nav2, YOLOv8 visual detection, voice interaction
> **Operating environment:** Indoor, mixed flooring (carpet area + smooth flooring), 1–5 m range, domestic rooms
> **ROS version:** ROS 2 Humble

> **What's verified vs. inferred.** Sections 1–5 and 9–13 reflect what was directly observed during Phase 0 testing — USB IDs, topic rates, frame names, port assignments, encoder mappings, calibration values. Section 6 (Astra+) and Section 8 (Voice module) describe hardware that's mounted on the robot but not all of it is actively in use — verify resolutions / connectivity before relying on those numbers. See [`phase0-status.md`](phase0-status.md) for the full verified-vs-conjectured record.

---

## Table of Contents

1. [System Architecture Overview](#1-system-architecture-overview)
2. [Power Flow Diagram](#2-power-flow-diagram)
3. [USB Hub Board — YB-ERV02-V2.0](#3-usb-hub-board--yb-erv02-v20)
4. [Expansion Board — YB-ERF01-V3.0](#4-expansion-board--yb-erf01-v30)
5. [Compute — Jetson Orin NX 8GB](#5-compute--jetson-orin-nx-8gb)
6. [Depth Camera — Orbbec Astra+](#6-depth-camera--orbbec-astra)
7. [LiDAR — SLAMTEC RPLiDAR A1](#7-lidar--slamtec-rplidar-a1)
8. [Voice Module — YB-MASR-V1.0](#8-voice-module--yb-masr-v10)
9. [Port Assignment Master Table](#9-port-assignment-master-table)
10. [ROS2 Topic Map](#10-ros2-topic-map)
11. [Feature-to-Hardware Matrix](#11-feature-to-hardware-matrix)
12. [Known Limitations & Gotchas](#12-known-limitations--gotchas)
13. [Links & Resources](#13-links--resources)

---

## 1. System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        SAVELIJ BOT SYSTEM                           │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │               JETSON ORIN NX 8GB  (192.168.0.120)           │   │
│  │   ROS2 Humble · SLAM · Nav2 · YOLOv8 · All high-level logic │   │
│  └───────┬──────────────────────────────────────────────────────┘   │
│          │ USB-A ×5                                                  │
│          ├── [L] 7-inch display                                      │
│          ├── [C] Orbbec Astra+    (depth camera, USB ID 2bc5:050f)  │
│          ├── [C] Orbbec Astra+    (depth-sensor port, 2bc5:060f)    │
│          ├── [R] SLAMTEC RPLiDAR A1 → /dev/ttyUSB0 (CP2102)         │
│          └── [R] Yahboom Rosmaster MCU → /dev/ttyUSB1 (CH340)       │
│                                                                     │
│  ┌──────────────────┐       ┌──────────────────────────────────┐    │
│  │  USB Hub Board   │       │       Expansion Board            │    │
│  │  YB-ERV02-V2.0   │──12V──│       YB-ERF01-V3.0              │    │
│  │                  │       │  Motors · Servos · IMU · CAN     │    │
│  │  IN:  12V (×2)  │       │  I2C · RGB · SBUS · SWD          │    │
│  │       5V micro   │       └──────────────────────────────────┘    │
│  │  OUT: USB ×4    │                                                │
│  │       VBUS 5V   │       ┌──────────────────────────────────┐    │
│  └──────────────────┘       │     Voice Module YB-MASR-V1.0   │    │
│          ▲                  │  CI1302 ASR · STC8H · Speaker   │    │
│          │                  │  UART or I2C to Jetson/Exp.Bd   │    │
│   3S2P 18650 Battery        └──────────────────────────────────┘    │
│   (Panasonic NCR18650B)                                             │
│   ~12.6V fully charged                                              │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 2. Power Flow Diagram

```
  ┌─────────────────────────────────┐
  │  3S2P 18650 Battery Pack        │
  │  Panasonic NCR18650B cells      │
  │  JYS-6042-3S-TK BMS             │
  │  ~12.6V full · ~11.1V nominal   │
  │  ~6800mAh effective capacity    │
  └────────────┬────────────────────┘
               │ 12V DC
               ▼
  ┌─────────────────────────────────┐
  │  USB Hub Board  YB-ERV02-V2.0   │
  │                                 │
  │  ┌──────────┐  ┌─────────────┐  │
  │  │ ON/OFF   │  │ DC 12V ×2   │  │◄── Battery
  │  │ Switch   │  │ DCIN 6V-24V │  │
  │  └──────────┘  └─────────────┘  │
  │                                 │
  │  VBUS 5V ──────────────────────►│──► Jetson (5V rail if needed)
  │  USB1-USB4 ────────────────────►│──► Peripheral devices
  └────────────┬────────────────────┘
               │ 12V DC
               ▼
  ┌─────────────────────────────────┐
  │  Expansion Board  YB-ERF01-V3.0 │
  │                                 │
  │  12V IN → Motor H-bridge        │──► Motors M1–M4
  │  Type-C 5V OUT ────────────────►│──► Voice module / sensors
  │  DC 5V OUT ────────────────────►│──► Logic peripherals
  └─────────────────────────────────┘

  Jetson Orin NX powers itself via:
  └── USB-C 5V or dedicated power connector from hub VBUS
```

---

## 3. USB Hub Board — YB-ERV02-V2.0

**Role:** Power distribution spine and USB data routing hub.

**Product:** Yahboom YB-ERV02-V2.0  
**Location in bot:** Center of chassis, below the Jetson mount  

### Interfaces

| Direction | Label | Connector | Notes |
|-----------|-------|-----------|-------|
| **IN** | USB data input | White USB-A | Data path from Jetson |
| **IN** | DC 12V input (×2) | Screw terminal `DCIN 6V-24V` | Both are parallel — wire battery here |
| **IN** | DC 5V input | Micro USB `DCIN 5V-2A` | Auxiliary 5V if needed |
| **OUT** | USB data output (×2) | White USB-A (top) | Data out to peripherals |
| **OUT** | USB1–USB4 | Right-side USB-A ports | Powered USB for peripherals |
| **OUT** | VBUS 5V | Header `VBUS` | 5V bus power rail |

### Controls

| Component | Function |
|-----------|----------|
| ON/OFF slide switch | Cuts all board outputs. Use for safe power cycling |

### Key ICs (visible on PCB)

| Component | Purpose |
|-----------|---------|
| Large MCU (center) | USB hub controller IC |
| Blue electrolytic cap | Bulk power filtering (absorbs inrush) |
| Red indicator LEDs | Power and data status |

### Notes

- The two DC 12V screw terminals are **electrically parallel** — wire battery to either, or both for redundancy
- Data path is strictly: **Jetson → USB IN → USB data outputs → lidar / camera**. The USB ports are NOT an independent hub; they pass through from the Jetson
- When flipping the ON/OFF switch, allow 2–3 seconds for capacitors to discharge before reconnecting
- If the Jetson draws too much current through VBUS (e.g. when external display is attached), consider a direct barrel jack supply to the Jetson

---

## 4. Expansion Board — YB-ERF01-V3.0

**Role:** Low-level actuator control, sensor buses, secondary power regulation. The STM32-based bridge between Jetson ROS2 and physical hardware.

**Product:** Yahboom YB-ERF01-V3.0
**Communicates with Jetson via:** Micro USB serial — appears as **`/dev/ttyUSB1`** on this Jetson (USB device `1a86:7523` QinHeng CH340). The CH340 variant on this board does not expose a serial number, so there is no stable `by-id` symlink for it; the path stays as `/dev/ttyUSB1` provided enumeration order is preserved.
**Firmware library on Jetson:** `Rosmaster_Lib` (Python, version 3.3.9 installed).

### Power Interfaces

| Label | Connector | Direction | Notes |
|-------|-----------|-----------|-------|
| DC 12V input | Large terminal block (top-right) | IN | From USB hub board |
| DC 12V output | Same block | OUT | Passthrough / daisy-chain |
| Type-C DC | Type-C port | OUT | 5V regulated output |
| DC 5V output | 2-pin header | OUT | Logic-level 5V rail |

### Communication Interfaces

| Interface | Connector | Notes |
|-----------|-----------|-------|
| Micro USB data | Micro USB port | Serial to Jetson — main comms channel |
| I2C | Header: `GND SDA SCL 3.3V` | For sensors, OLED displays, etc. |
| CAN bus | Header: `CAN` | Future peripherals, arm actuators |
| SWD debug | Header: `SWD` (top-left) | STM32 firmware update via ST-Link |

### Actuator Interfaces

| Interface | Pins | Notes |
|-----------|------|-------|
| Motor M1–M4 | `3U3 · GND · H_A · H_B` per motor | H-bridge outputs. Match to wheel position carefully |
| PWM servo | Standard 3-pin servo header | RC-style servos, pan/tilt |
| Serial servo (robotic arm) | Bus header | Smart digital servos (e.g. Dynamixel-compatible) |
| SBUS receiver | `SBUS` header | RC model aircraft remote input |

### Motor Wiring Convention

The H-bridge wires four motors. Note that **encoder index** (m1..m4) and physical wheel position were established empirically on this unit using `scripts/wheel_balance_diagnostic.py discover` (Phase 0):

```
Empirically determined encoder → wheel mapping (this unit):
                           ┌──────────────┐
m1 ──► Front-Left          │  m1       m3 │
m2 ──► Back-Left           │              │
m3 ──► Front-Right         │  m2       m4 │
m4 ──► Back-Right          └──────────────┘

i.e. left side = {m1, m2}, right side = {m3, m4};
     front = {m1, m3},     back     = {m2, m4}.
```

Note: the H-bridge channel labels printed on the PCB (M1..M4) may not match the encoder numbering above. If you ever need to know which physical motor is M1 etc., spin each motor individually via `Rosmaster_Lib.set_motor` — that takes 4 PWM duty values, indexed by H-bridge channel. The diagnostic script's `discover` mode finds the encoder index per wheel, which is what matters for kinematics and odometry.

H_A / H_B pin polarity determines forward/reverse per motor. Verify by running each motor independently before SLAM.

### Onboard Sensors

| Sensor | Type | Interface | Notes |
|--------|------|-----------|-------|
| IMU (onboard) | **9-axis** inertial — gyro + accelerometer + magnetometer | Internal MCU bus (I²C to STM32) | Almost certainly **MPU-9250** or close variant. Magnetometer is published but not used by `imu_filter_madgwick` (`use_mag: false`). Feeds odometry data; the EKF (`robot_localization`) fuses gyro yaw with wheel-derived odometry. |
| RGB light bar | Addressable LEDs | `RGB` header | Status indicator — configurable per ROS2 state |
| Buzzer | Piezo | MCU GPIO | Startup tones, low-battery warning |

### Firmware

The expansion board runs a Yahboom-provided STM32 firmware that:
- Runs per-wheel PID at ~1 kHz (encoder-feedback closed loop on each motor's commanded speed)
- Decomposes a chassis-level Twist (`set_car_motion(vx, vy, w)`) into four wheel speed setpoints via mecanum kinematics
- Reads MPU-9250 IMU via I²C and forwards via the binary protocol
- Reports back chassis-frame velocity (NOT raw encoder counts) on each tick
- Controls RGB, servos, buzzer via serial commands

The firmware is closed-source and burned to MCU flash. From the ROS layer we can call its public API (via `Rosmaster_Lib`), but we cannot read or modify the firmware itself. Custom low-level behavior would require flashing replacement firmware via SWD with an ST-Link.

### Empirical calibration for THIS unit (Phase 0 results)

These are unit-specific numbers — re-measure if you replace any wheel, motor, or chassis component:

| Parameter | Value | Notes |
|-----------|-------|-------|
| `trim_vy_per_vx` | **0.012** | Software trim added in `Mcnamu_driver_X3.py`. When commanding forward, the chassis driver injects `vy_corrected = vy + 0.012 × vx`. Compensates for the chassis's tendency to curve right at ~0.15 m/s. |
| `trim_w_per_vx` | 0.0 | No yaw-rate correction needed at the calibrated speed. |
| `angular_scale` (in `base_node_X3`) | 0.5 | Yahboom firmware over-reports yaw rate by ~2×. The `base_node_X3` integrator scales it down. |
| Per-wheel encoder spread (commanded forward, on blocks) | ±3.2 % from 4-wheel mean | m1 (FL) +1.6 %, m2 (BL) −2.5 %, m3 (FR) −2.2 %, m4 (BR) +3.2 %. Pattern: diagonal {FL,BR} is faster than {BL,FR} by ~5 %. |
| Closed-loop drift (out-and-back, 4.69 m total path, with trim active) | **14.4 cm position, 4.7° yaw** | Phase 0 acceptance gate. Matches plan's healthy band (10–20 cm position, ≤ 5–10° yaw). Forward leg ≈ 0; most drift accumulates during the uncalibrated backward leg. |

For full Phase 0 calibration record see [`phase0-status.md`](phase0-status.md), section 5.

Flash new firmware via SWD (ST-Link) if you need custom low-level behavior.

---

## 5. Compute — Jetson Orin NX 8GB

**Role:** All ROS2 high-level computation — SLAM, Nav2 path planning, YOLOv8 cat detection, voice processing, WebSocket comms.

**Specs:** NVIDIA Jetson Orin NX 8GB, **JetPack 6.1 (Jetson Linux 36.4.3)**, **Ubuntu 22.04.5 LTS (Jammy)**
**IP on local network:** `192.168.0.120`
**SSH:** `ssh jetson@192.168.0.120` (the on-bot user is `jetson`; on the host machine the same workspace is mirrored under user `bots`)

### Physical Interface Layout

```
  TOP EDGE
  ┌──────────────────────────────────────────────────────┐
  │  [ANT1]                                    [ANT2]    │  ← WiFi antenna U.FL connectors
  │                    ┌─────────┐                       │
  │                    │ COOLING │    [FAN JST]           │  ← cooling fan connector
  │   [CAM1 MIPI]      │  FAN   │    [OLED I2C]  ████   │  ← color-coded header
  │                    └─────────┘                       │
  │   [SSD M.2 — on BACK of board]                       │
  └──────────────────────────────────────────────────────┘
  BOTTOM EDGE — USB-A ports (left to right):
  ┌────────────────────────────────────────────────────┐
  │  [USB-A] ──────── 7-inch touch display             │
  │  [USB-A] [USB-A] ─ Orbbec Astra FHD (depth cam)   │
  │  [USB-A] [USB-A] ─ Expansion board + YDLIDAR X3   │
  └────────────────────────────────────────────────────┘
```

### USB Port Assignments — verified empirically on this unit

The actual `lsusb` listing on this Jetson:

```
1a86:7522  QinHeng Electronics  USB Serial   ← unidentified peripheral (silent on idle)
1a86:7523  QinHeng Electronics  CH340         ← Yahboom Rosmaster MCU (chassis)
10c4:ea60  Silicon Labs          CP210x UART   ← SLAMTEC RPLiDAR A1
2bc5:050f  Orbbec 3D Technology  USB 2.0 Camera (UVC color)  ← Astra+
2bc5:060f  Orbbec 3D Technology  ORBBEC Depth Sensor          ← Astra+
```

Mapped to Linux device paths (verified with byte-stream sniff and udev introspection):

| Linux device | USB ID | Hardware | Cable type |
|---|---|---|---|
| `/dev/ttyUSB0` | `10c4:ea60` (Silicon Labs CP2102) | **SLAMTEC RPLiDAR A1** | USB-A to micro-USB-B (LiDAR-side adapter board) |
| `/dev/ttyUSB1` | `1a86:7523` (CH340) | **Yahboom Rosmaster MCU** (chassis) | USB-A to micro-USB |
| `/dev/ttyUSB2` | `1a86:7522` (CH340) | Unidentified (silent on idle); possibly Astra Pro UART control endpoint or unused. Not used by `cat_patrol_robot`. | — |
| (UVC video) | `2bc5:050f` + `2bc5:060f` | Orbbec Astra+ (color + depth) | USB-A, single cable |
| (HID + USB) | — | 7-inch HDMI/USB touch display | USB-A to USB-A |

**Stable `by-id` symlinks** (immune to USB enumeration order):

- LiDAR: `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`
- Chassis MCU: **no `by-id` symlink**, because this CH340 variant doesn't expose a serial number. Stays at `/dev/ttyUSB1` if enumeration order is preserved.

> ⚠️ All 5 Jetson USB-A ports are occupied in full configuration. Voice module must use expansion board UART or I²C — not a direct USB connection to the Jetson.

### Other Connectors

| Connector | Location | Assignment |
|-----------|----------|------------|
| WiFi antenna ×2 | Top edge | U.FL → external antenna wires |
| OLED I2C | Right edge, color-coded header | Status OLED display (0x3C typical) |
| Cooling fan | Right edge, JST | PWM-controlled fan |
| CAM1 MIPI CSI | Left edge | Currently unused — available for CSI camera |
| SSD M.2 NVMe | Back of board | System SSD installed |

### Performance Notes

| Workload | Expected behavior |
|----------|------------------|
| ROS2 + SLAM | Moderate CPU load, GPU idle |
| YOLOv8 inference | GPU active, ~30fps on 640px input |
| SLAM + Nav2 + YOLO simultaneously | Monitor thermal throttle — fan must be functional |
| WebSocket streaming | CPU load depends on resolution; use JPEG compression |

---

## 6. Depth Camera — Orbbec Astra+

**Role:** 3D spatial perception — obstacle avoidance costmap, RTAB-Map depth input, RGB feed for YOLOv8 cat detection, object distance estimation.

**Identification on this unit:** USB IDs `2bc5:050f` (UVC color stream) and `2bc5:060f` (depth sensor). The system banner labels the camera as `astraplus` (the Yahboom robot ships with the Astra+ variant).
**Technology:** Structured light (IR projector + IR camera + custom Orbbec ASIC)
**Product page:** https://www.orbbec.com/products/structured-light-camera/astra-series/
**ROS2 driver used in `cat_patrol.launch.py`:** the legacy `astra_camera` package (XML launch `astra_pro.launch.xml`, `uvc_product_id=0x050f`) — **not** the newer `OrbbecSDK_ROS2`. The Yahboom workspace has both available; we use the one that matches the existing launch.

### Specifications (Astra+ / Astra Pro Plus)

> The Astra+ family overlaps in naming with "Astra Pro" and "Astra FHD" — Orbbec's product line was renamed across generations. The values below match the most-likely shipping variant for this Yahboom kit. **Verify by running the camera and checking actual stream resolutions.**

| Parameter | Value |
|-----------|-------|
| Depth technology | Structured light |
| Depth range | 0.6 m – 6 m (some specs list 8 m max) |
| Depth resolution | 640 × 480 @ 30 fps |
| Depth FOV | ≈ 60° H × 49° V |
| Depth accuracy | ±3 mm @ 1 m (typical) |
| RGB resolution | up to 1920 × 1080 @ 30 fps on Astra+; **1280 × 720 on plain Astra Pro** — verify on this unit |
| RGB FOV | ≈ 66° H × 40° V |
| Interface | USB 2.0 Type-A — single cable (power + data) |
| Power draw | Average < 2.4 W |
| IMU | ❌ Not present (so the Madgwick filter / EKF must use the chassis-board IMU) |
| Operating temp | 10 °C – 40 °C, 10–85 % RH |
| D2C alignment | ✅ Supported |
| Data outputs | Depth map · RGB · IR · Point Cloud |

### How Structured Light Works

```
  IR Projector         IR Camera
       │                  │
       │  Known pattern   │  Captured deformed pattern
       │ ─────────────►   │
       │                  ▼
       │          ┌──────────────┐
       │          │ Orbbec ASIC  │
       │          │ depth solver │
       │          └──────┬───────┘
       │                 │
       ▼                 ▼
   Scene             Depth map (per pixel distance)
```

The IR projector casts a structured dot pattern. Deformation of that pattern captured by the IR camera reveals depth at every pixel. On-chip ASIC handles all computation — no CPU load for depth processing.

**Implication:** The camera produces depth data independently; the Jetson only handles the streams.

### ROS2 Integration

**Driver setup (run once on Jetson):**
```bash
cd ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Build:**
```bash
cd ~/ros2_ws
colcon build --packages-select orbbec_camera
source install/setup.bash
```

**Launch:**
```bash
ros2 launch orbbec_camera astra_pro.launch.py
```

**Key topics published:**

| Topic | Type | Rate | Notes |
|-------|------|------|-------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 30 Hz | Full 1080p RGB |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 30 Hz | 640×480 depth map |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 30 Hz | 3D point cloud |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | Intrinsics |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | Intrinsics |

> ⚠️ **QoS:** Default is `sensor_data` (Best Effort). Set RViz2 reliability to Best Effort to avoid topic subscription failures.

**Enable D2C alignment** (for cat detection — maps depth onto color frame):
```bash
# In launch file or via parameter:
ros2 param set /camera/orbbec_camera_node depth_registration true
```

### Relevant Use Cases for Savelij

| Feature | Camera contribution |
|---------|-------------------|
| SLAM (RTAB-Map) | Depth stream → 3D voxel map reconstruction |
| Nav2 obstacle avoidance | Depth → `depthimage_to_laserscan` → costmap |
| YOLOv8 cat detection | RGB 1080p → detection input |
| Per-cat identification | RGB crop → classification model |
| Cat distance estimation | Depth map lookup at bounding box center |
| Visual odometry | RGB + depth → RTAB-Map VO fallback |

### Blind Zone Warning

```
   0m    0.6m                  8m
   │──────│──────────────────────│
   ████   ·  valid depth range   ·
   blind
   zone
```

Objects closer than **0.6m** produce no depth data. Cats sitting right under the camera are invisible to it. Combine with YDLIDAR X3 Pro for full close-range coverage.

---

## 7. LiDAR — SLAMTEC RPLiDAR A1

**Role:** 360° 2D plane scanning for SLAM map building, Nav2 costmap obstacle layer, wall-following patrol, room boundary detection.

**Identification on this unit (verified at runtime by the driver):**
- Brand / model: SLAMTEC RPLiDAR A1 (the version we're running reports itself as `SLLidar` — same hardware family, different driver naming convention)
- Serial number: `83C2FA89C7E19EC8BCE499F008155670` (printed by the driver on startup)
- Firmware version: 1.29
- Hardware revision: 7
- Reported scan mode: "Sensitivity", sample rate **8 kHz**, max range **12 m**, scan frequency **10 Hz**
- USB-serial bridge: Silicon Labs CP2102 (`10c4:ea60`) → `/dev/ttyUSB0`

**Manufacturer:** SLAMTEC (Shanghai SLAMTEC Co., Ltd.)
**Product page:** https://www.slamtec.com/en/Lidar/A1
**ROS2 driver used in `cat_patrol.launch.py`:** [`sllidar_ros2`](https://github.com/Slamtec/sllidar_ros2) (the SLAMTEC official driver). Note: this is **not** YDLIDAR — different vendor, different driver, different protocol despite both being USB-serial 2D triangulation scanners.

### Technology: Laser Triangulation

```
  Rotating laser turret (the part that physically spins)
       │
  Laser ──► scene ──► reflected back ──► CMOS line sensor
                                              │
                          angle of spot on sensor → distance
```

A laser beam (Class 1, ~785 nm or 905 nm depending on revision; eye-safe) is emitted, hits the scene, and reflects back. The reflected spot lands on a small CMOS line sensor at a position determined by the target's distance. Depth is computed from the **angle** of the reflected ray — no time-of-flight measurement. This makes the A1 cheap and accurate at short-medium range, but noisier beyond ~6 m.

### Specifications (A1 / A1M8)

| Parameter | Value |
|-----------|-------|
| Ranging technology | Laser triangulation |
| Scan FOV | 360° |
| Max range | up to **12 m** (white indoor wall, this unit) |
| Min range (blind zone) | ~0.15 m |
| Sample rate | **8000 samples/s** (this unit's scan-mode report) |
| Scan rate | **5.5 Hz typical, up to 10 Hz** (~7.4 Hz observed in our Phase 0 testing) |
| Serial baud rate | **115200** (A1; do not confuse with the 256 000 / 1 000 000 baud of higher-end A2/A3/S1 models) |
| Interface | USB-serial via CP2102 |
| Laser class | Class 1 (eye-safe) |
| Angular resolution at 8 kHz / 7 Hz | ≈ 0.32° per sample (≈ 1100 points per scan) |
| Angular resolution at 8 kHz / 10 Hz | ≈ 0.45° per sample (≈ 800 points per scan) |

### Adapter

The A1 connects to the Jetson via the SLAMTEC USB-serial adapter (containing the CP2102 chip). Both data and motor power flow through a single USB cable. There is no separate "motor power" port on the A1 (that's a feature of the A2/A3 family). On boot, the driver explicitly issues a "start motor" command; on clean shutdown it issues "stop motor" — which is also exposed as a runtime service (see below).

### ROS2 Integration

**Driver source** (already installed in `software/library_ws/install/sllidar_ros2`):

```
https://github.com/Slamtec/sllidar_ros2
```

**Launch as used by Phase 0:**

```bash
ros2 launch sllidar_ros2 sllidar_launch.py \
    serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
    serial_baudrate:=115200 \
    frame_id:=laser_link
```

**Important:** `frame_id:=laser_link` must match the URDF's static TF chain (`base_link → laser_link`). An earlier default of `frame_id:=laser` produced a frame that didn't exist in `/tf_static` — caught and fixed during Phase 0 (see `cat_patrol.launch.py` change history).

**Verify it's running:**

```bash
ros2 topic hz /scan                                          # expect ~7 Hz
ros2 topic echo /scan --once --field header                  # frame_id: laser_link
ros2 service call /stop_motor std_srvs/srv/Empty             # pause the spinning turret without killing the node
ros2 service call /start_motor std_srvs/srv/Empty            # resume
```

**Key topics & services:**

| Topic / service | Type | Rate / role |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | ~7 Hz, 360° range array (frame_id `laser_link`) |
| `/start_motor` | `std_srvs/srv/Empty` | Service — start the LiDAR's spinning turret |
| `/stop_motor` | `std_srvs/srv/Empty` | Service — stop the turret (without taking down the node) |

### TF configuration

The static transform `base_link → laser_link` is published by `robot_state_publisher` from the Yahboom URDF (`yahboomcar_X3.urdf`):

```
base_link → laser_link  =  translation (0.043, 0.0, 0.110) m, no rotation
```

So the LiDAR sits ~4.3 cm forward of the chassis center and ~11 cm above the `base_link` plane. Combined with the `base_footprint → base_link` static (~8 cm), that puts the laser plane around **19 cm above the floor**. Worth knowing: things lying flat on the floor (cats sleeping, low cables) may be below the scan plane and invisible to SLAM.

### Relevant use cases for this project

| Feature | LiDAR contribution |
|---------|-------------------|
| `slam_toolbox` map building (Phase 1) | `/scan` → 2D occupancy grid |
| `nav2_amcl` localization (Phase 2) | `/scan` matched against the saved map → `map → odom` correction |
| Nav2 local costmap obstacle avoidance (Phase 3) | `/scan` → obstacle inflation layer for the local planner |
| Room boundary detection | 360° sweep identifies walls, doors, furniture perimeter |
| Close-range obstacle avoidance | Down to ~15 cm — covers the depth camera's 60 cm blind zone |
| Wall-following / pseudo-docking | Consistent offset from scan returns in front arc; known scan signature near dock pose |

---

## 8. Voice Module — YB-MASR-V1.0

**Role:** Offline voice command recognition and audio feedback. Wake-word triggered, custom command words, no cloud dependency.

**SKU:** 6000400536  
**Price:** $19  
**Product page:** https://category.yahboom.net/products/voice_module_asr-tts  
**Tutorials:** https://www.yahboom.net/study/Voice-interaction

### Physical Layout

```
  FRONT                          BACK (PCB side)
  ┌────────────────┐             ┌────────────────────────────┐
  │                │             │  [Type-C]  ← power / flash │
  │   ●●●●●●       │             │                            │
  │  ●●●●●●●       │             │  [Amplifier chip]          │
  │   ●●●●●●       │  Speaker    │  [CI1302 ASR chip]         │
  │  ●●●●●●●       │             │  [STC8H MCU chip]          │
  │   ●●●●●●       │             │  [Anti-reverse protection] │
  │                │             │                            │
  │  [Slide SW]    │             │  [IIC header]              │
  │  [Reset BTN]   │             │  [Serial header]           │
  │  [MCU LED]     │             │                            │
  │  [PWR LED]     │             └────────────────────────────┘
  │                │
  │     (MIC)      │  ← high-performance microphone
  └────────────────┘

  BOTTOM — two PH2.0 connectors:
  ┌──────────────────────────────────────┐
  │  5V · GND · TX · RX                 │  ← UART to host
  │  RX1 · TX1 · GND · 5V              │  ← secondary UART
  │  GND · SDA · SCL · 3.3V            │  ← I2C to host
  └──────────────────────────────────────┘
```

### Key ICs

| Chip | Role |
|------|------|
| **CI1302** | Main ASR (Automatic Speech Recognition) engine. Contains a neural network processor with deep learning noise reduction and echo cancellation |
| **STC8H** | MCU bridge — converts CI1302 recognition events into serial UART frames or I2C messages the host reads |
| **Amplifier** | Drives the onboard speaker for TTS / audio feedback |

### Capabilities

| Feature | Detail |
|---------|--------|
| Preset commands | 110+ factory-loaded voice commands |
| Custom commands | Via Yahboom web tool → generate firmware → flash via PC over Type-C |
| Storage | 2MB internal, ~120 command words max |
| Wake word | Customizable |
| Languages | Chinese + English |
| Recognition rate | 99% at ≤5m range |
| Communication mode | UART **or** I2C — choose one at integration time |

### Cables Included

| Cable | Length | Purpose |
|-------|--------|---------|
| PH2.0-4Pin to DuPont | 20cm | Connect to Jetson GPIO UART or expansion board |
| PH2.0-4Pin to PH2.0-4Pin | 20cm | Board-to-board |
| Type-C | 100cm | Power + firmware flashing |
| M3×5mm screws ×2 | — | Mounting |
| M3×6+4mm copper standoffs ×2 | — | Mounting |

### Integration with Savelij

**Connection path (recommended):** Voice module UART TX/RX → Expansion board serial header → Jetson reads via `/dev/ttyUSBx` serial

**Alternative:** PH2.0 DuPont cable → Jetson GPIO UART pins (if expansion board serial port is occupied)

**ROS2 node concept:**
```
/dev/ttyUSB1  →  voice_serial_node  →  /voice_cmd (String)
                                              │
                                      voice_dispatcher_node
                                              │
                          ┌───────────────────┼──────────────┐
                          ▼                   ▼              ▼
                    /cmd_vel           /patrol/start    /camera/snapshot
                  (move robot)       (begin patrol)    (take photo)
```

**Custom command word examples for Savelij:**

| Command word | Action |
|-------------|--------|
| "Savelij, go forward" | Publish to `/cmd_vel` |
| "Savelij, stop" | Zero velocity |
| "Savelij, start patrol" | Trigger patrol scheduler node |
| "Savelij, find cat" | Enable YOLOv8 detection mode |
| "Savelij, go home" | Trigger Nav2 return-to-dock |
| "Savelij, take picture" | Trigger camera snapshot |

**I2C address note:** If using I2C mode, verify voice module I2C address does not conflict with OLED (typically 0x3C). Scan bus: `i2cdetect -y 1`

### Firmware Customization Workflow

```
1. Go to Yahboom web customization tool
2. Enter custom command words (Chinese or English)
3. Download generated .bin firmware file
4. Connect module via Type-C to PC
5. Flash using Yahboom PC tool (Windows)
6. Module reboots with new vocabulary
```

---

## 9. Port Assignment Master Table

Complete wiring reference — every physical connection in the system.

| From | To | Cable | Notes |
|------|----|-------|-------|
| Battery (+/−) | USB hub board DCIN 6V-24V | 14AWG silicone wire | Both screw terminals in parallel |
| USB hub board 12V OUT | Expansion board 12V IN | DC cable | Powers motors |
| USB hub board USB_DATA | Jetson USB-A (center pair, via hub) | USB-A to USB-A | Data routing |
| **Expansion board Micro USB** | **Jetson USB-A** | Micro USB to USB-A | Serial comms via CH340 (`1a86:7523`) → **`/dev/ttyUSB1`**. No `by-id` symlink (CH340 has no serial number). |
| **SLAMTEC RPLiDAR A1** | **Jetson USB-A** | USB-A direct to LiDAR adapter | Serial comms via CP2102 (`10c4:ea60`) → **`/dev/ttyUSB0`**. Stable `by-id` path: `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` |
| Orbbec Astra+ | Jetson USB-A center pair | USB-A (single cable) | Power + depth + RGB data. USB IDs `2bc5:050f` (color) + `2bc5:060f` (depth) |
| 7-inch display | Jetson USB-A leftmost | USB-A to USB-A | Touch + USB data |
| Voice module UART | Expansion board serial header | PH2.0-4Pin to DuPont | `5V GND TX RX`. Voice module is **not currently integrated** into the running stack. |
| OLED display | Jetson OLED-I2C header | JST/I2C cable | Address 0x3C |
| WiFi antennas ×2 | Jetson top U.FL | U.FL cables | Both required for 2×2 MIMO |
| Cooling fan | Jetson fan JST connector | JST cable | PWM-controlled |
| SSD (NVMe) | Jetson M.2 back slot | — | Already installed |

---

## 10. ROS2 Topic Map

All topics Savelij produces and consumes in normal operation.

### Sensor inputs (produced by hardware/driver nodes)

Rates are as **measured** on this unit during Phase 0 testing.

| Topic | Type | Source node | Rate (measured) | Notes |
|-------|------|-------------|--------|-------|
| `/scan` | `LaserScan` | `sllidar_node` (`sllidar_ros2`) | **~7.4 Hz** | `frame_id: laser_link`. 360° range array. |
| `/camera/depth/image_raw` | `Image` | `astra_camera_node` | 30 Hz | 640×480 depth |
| `/camera/color/image_raw` | `Image` | `astra_camera_node` | 30 Hz | up to 1080p on Astra+ |
| `/camera/depth/points` | `PointCloud2` | `astra_camera_node` | 30 Hz | 3D point cloud |
| `/imu/data_raw` | `Imu` | `Mcnamu_driver_X3.py` | **10 Hz** | Raw gyro + accel. **No** orientation field populated. |
| `/imu/data` | `Imu` | `imu_filter_madgwick_node` | **10 Hz** | Madgwick-filtered. Adds orientation quaternion. |
| `/imu/mag` | `MagneticField` | `Mcnamu_driver_X3.py` | 10 Hz | Magnetometer. **Not** used by Madgwick (`use_mag: false`). |
| `/vel_raw` | `Twist` | `Mcnamu_driver_X3.py` | 10 Hz | Chassis-frame velocity reported by the MCU firmware. **Not** raw encoders. |
| `/odom_raw` | `Odometry` | `base_node_X3` (C++) | 10 Hz | Wheel-only integration of `/vel_raw`. |
| `/odom` | `Odometry` | `ekf_filter_node` (`robot_localization`) | **10.0 Hz** | EKF fusion of `/odom_raw` + `/imu/data`. **This is the trustworthy odom topic — subscribe to this, not `/odom_raw`.** |
| `/joint_states` | `JointState` | `Mcnamu_driver_X3.py` | 10 Hz | Names only; `position`/`velocity` are not currently populated. |
| `/voltage` | `Float32` | `Mcnamu_driver_X3.py` | 10 Hz | Battery voltage in volts. |
| `/voice_cmd` | `String` | (voice serial node, **not running yet**) | event | Reserved for future voice integration. |

### Command Outputs (consumed by hardware nodes)

| Topic | Type | Consumer | Effect |
|-------|------|----------|--------|
| `/cmd_vel` | `Twist` | expansion board node | Motor velocity |
| `/rgb_control` | `String` | expansion board node | RGB light bar color |
| `/servo_cmd` | `Int32` | expansion board node | Servo position |

### Computed / Navigation Topics

| Topic | Type | Producer | Consumer |
|-------|------|----------|---------|
| `/map` | `OccupancyGrid` | SLAM node | Nav2, RViz2 |
| `/tf` | `TFMessage` | multiple | everyone |
| `/goal_pose` | `PoseStamped` | operator / patrol node | Nav2 |
| `/detection/cats` | `Detection2DArray` | YOLOv8 node | logger, alerter |
| `/costmap/local` | `OccupancyGrid` | Nav2 | path planner |

---

## 11. Feature-to-Hardware Matrix

How each planned Savelij feature maps to specific hardware.

| Feature | LiDAR | Depth Cam | IMU | Motors | Voice | Jetson GPU |
|---------|:-----:|:---------:|:---:|:------:|:-----:|:----------:|
| 2D SLAM mapping | ✅ primary | optional | ✅ assist | — | — | — |
| 3D SLAM (RTAB-Map) | ✅ assist | ✅ primary | ✅ assist | — | — | — |
| Nav2 navigation | ✅ obstacle | ✅ obstacle | — | ✅ drive | — | — |
| Wall-following patrol | ✅ primary | — | — | ✅ drive | — | — |
| Cat detection (YOLO) | — | ✅ RGB | — | — | — | ✅ inference |
| Cat distance estimate | — | ✅ depth | — | — | — | — |
| Per-cat identity | — | ✅ RGB | — | — | — | ✅ classify |
| Image capture | — | ✅ RGB | — | — | — | — |
| Voice command control | — | — | — | ✅ execute | ✅ primary | — |
| Audio feedback (TTS) | — | — | — | — | ✅ speaker | — |
| Return-to-dock | ✅ landmark | — | — | ✅ drive | — | — |
| Odometry fallback | — | — | ✅ primary | ✅ encoders | — | — |
| Close obstacle avoidance | ✅ primary | ✅ depth | — | ✅ react | — | — |

---

## 12. Known Limitations & Gotchas

### Power

- **USB current budget:** Jetson USB-A ports supply 5V/0.9A each. With 5 devices (display, camera ×2, expansion, lidar), total draw is close to limit. Monitor for USB resets under load (`dmesg | grep usb`)
- **Battery voltage sag:** Under full motor load, battery voltage can drop 0.5–1V. This can trigger BMS low-voltage cutoff — ensure motors don't stall under load for extended periods

### LiDAR (SLAMTEC RPLiDAR A1)

- **Baud rate is 115200** for the A1 (and A2-2.x). The 256 000 baud setting is for the A2-3.x and A3 models, and 1 000 000 is the S1. Using the wrong baud produces garbage data with no error message.
- **Glass and mirrors** produce phantom returns or missing data — the patrol map near fish tanks or mirrors will be inaccurate.
- **Scan plane is fixed at ~19 cm above the floor** on this unit (URDF `base_footprint → laser_link` chain). Cats lying flat on the floor may be below or in the scan plane.
- **No external motor power input** — the A1 motor is powered through the USB cable; budget ~300 mA inrush at startup. If motor stalls or stutters under load, try a powered USB hub between Jetson and lidar.
- **`/start_motor` and `/stop_motor` services** can pause/resume the spinning turret without restarting the node.

### Mecanum chassis on mixed-floor environments (Phase 0 finding)

- **Pure in-place rotation behaves drastically differently per surface.** On carpet, the wheels grip and the chassis pivots cleanly (~360° rotation, ~5 cm unwanted translation). On smooth flooring (laminate / tile), the rollers slip; the chassis arcs in a large circle (we measured ~190 cm of unwanted translation for ~90° of physical rotation). The IMU gyro catches the rotation correctly in both cases; the wheel-derived `/vel_raw` is **blind** to lateral roller-slip on smooth floors.
- **Implication for path planning**: prefer arcing turns (forward + small angular.z) over in-place spins on smooth floors. Keep the wheels rolling forward, where mecanum is honest.
- **Implication for `/odom`**: position estimates from `/odom` are unreliable on smooth floors during rotation; the EKF cannot detect the lateral drift. SLAM and AMCL (Phases 1–2) compensate via lidar-against-map matching; Nav2 (Phase 3) closes the loop via the controller.

### Forward-direction chassis bias and trim

- This unit physically curves to the right when commanded straight forward. Per-wheel encoder rates differ by ~3–5 % across the four wheels (FL fastest, FR slowest, with a ~5 % {FL,BR} > {BL,FR} diagonal pattern).
- A chassis-level trim was added in `Mcnamu_driver_X3.py` (parameters `trim_vy_per_vx`, `trim_w_per_vx`) and calibrated empirically:
  - `trim_vy_per_vx = 0.012` at vx ≈ 0.15 m/s — adds a small leftward strafe to commanded forward motion to cancel the rightward curve.
  - **Speed-dependent**. Calibrated at ~0.15 m/s; expect somewhat worse straight-line behavior at very different speeds. Nav2's controller will compensate for any residual bias once Phase 3 is online.
  - **Forward-only**. Backward motion is uncompensated and will drift; needs a separate `trim_vy_per_vx_reverse` if it ever becomes problematic.
- Diagnostic: `cat_patrol_robot/scripts/wheel_balance_diagnostic.py` (with `discover` and `measure --on-blocks` modes) measures per-wheel encoder rates directly via `Rosmaster_Lib.get_motor_encoder()`. Re-run after any wheel/motor service.

### Depth Camera (Orbbec Astra FHD)

- **0.6m blind zone** — nothing closer than 60cm produces depth. Cats directly under the camera are invisible to depth
- **Sunlight degrades structured light** — IR pattern washes out near bright windows. Schedule patrol away from high-sun periods if needed
- **USB 2.0 bandwidth ceiling** — running 640×480 depth + 1080p RGB simultaneously is near the practical USB 2.0 limit. If frame drops occur, reduce RGB resolution or disable one stream
- **No IMU** — unlike Astra 2, this model has no built-in IMU. Odometry comes from expansion board IMU + encoders only

### Voice Module

- **Windows-only firmware flash tool** — custom vocabulary requires a Windows machine for the official flash utility. Keep a Windows VM or partition available
- **I2C address conflict risk** — if using I2C mode, scan for conflicts with OLED (`i2cdetect -y 1`) before connecting
- **5m recognition limit** — at larger room distances, false triggers or missed words increase. Design patrol waypoints to bring robot closer to operator for voice interaction

### Expansion Board

- **Motor polarity** — H_A / H_B wiring determines forward/reverse per motor. Test each motor individually with a simple `/cmd_vel` command before SLAM. Miswired motors will spin the wrong way and destroy map quality
- **IMU calibration** — the onboard IMU needs calibration (static + dynamic) before use in odometry. Run the Yahboom calibration routine after assembly
- **SWD flashing requires ST-Link** — if custom firmware needed, you'll need an ST-Link V2 (~$5) connected to the SWD header

### Jetson

- **All USB-A ports occupied** — no spare Jetson USB ports in full configuration. Any additional USB devices must go through the hub board's USB1–USB4 ports
- **OLED I2C vs. other I2C devices** — OLED typically at address 0x3C. If adding more I2C sensors, verify addresses don't conflict: `i2cdetect -y 0` and `i2cdetect -y 1`
- **Thermal throttling** — running SLAM + Nav2 + YOLOv8 simultaneously under heavy GPU load can trigger thermal throttle. Ensure fan is spinning and check: `tegrastats`

---

## 13. Links & Resources

### Official Hardware Docs

| Component | Link |
|-----------|------|
| Yahboom ROSMASTER X3 | https://category.yahboom.net/products/rosmaster-x3 |
| USB Hub board tutorials | https://www.yahboom.com/study/ROSMASTER-X3 |
| Voice module product | https://category.yahboom.net/products/voice_module_asr-tts |
| Voice module tutorials | https://www.yahboom.net/study/Voice-interaction |
| **SLAMTEC RPLiDAR A1** product page | https://www.slamtec.com/en/Lidar/A1 |
| SLAMTEC product family overview | https://www.slamtec.com/en/Lidar |
| Orbbec Astra series | https://www.orbbec.com/products/structured-light-camera/astra-series/ |
| Orbbec Astra+ datasheet | https://www.orbbec.com/products/structured-light-camera/astra-series/ |

### ROS2 Drivers & SDKs

| Component | Driver | Branch / notes |
|-----------|--------|--------|
| **SLAMTEC RPLiDAR A1** (used here) | https://github.com/Slamtec/sllidar_ros2 | `humble` — published as `sllidar_ros2`. Built and installed under `software/library_ws/install/sllidar_ros2`. |
| Orbbec Astra+ (used here) | `astra_camera` package | Built under `software/library_ws/install/astra_camera`. Launch: `astra_pro.launch.xml`. Older Orbbec stack — predates `OrbbecSDK_ROS2`. |
| Orbbec next-gen (alternative) | https://github.com/orbbec/OrbbecSDK_ROS2 | `v2-main`. Newer, but our current launch uses the legacy `astra_camera`. Migrating later is an option. |
| `robot_localization` (EKF) | https://github.com/cra-ros-pkg/robot_localization | `humble`. Provides `ekf_filter_node`. Config: `software/library_ws/.../params/ekf_x1_x3.yaml`. |
| `imu_filter_madgwick` | ROS 2 binary install | Madgwick AHRS filter. |
| Voice module ROS example | https://www.yahboom.net/study/Voice-interaction | Not currently integrated. |

### Development & Tooling

| Tool | Purpose |
|------|---------|
| `tegrastats` | Jetson GPU/CPU/thermal monitor |
| `ros2 topic hz /scan` | Verify LiDAR publish rate |
| `ros2 topic hz /camera/depth/image_raw` | Verify camera publish rate |
| `i2cdetect -y 1` | Scan I2C bus for address conflicts |
| `dmesg \| grep usb` | Diagnose USB enumeration / resets |
| `ros2 run rviz2 rviz2` | Visual debugging of all sensor data |
| `ros2 run rqt_graph rqt_graph` | Visualize node/topic graph |

### Key ROS2 Packages (install on Jetson)

```bash
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-rtabmap-ros \
  ros-humble-depthimage-to-laserscan \
  ros-humble-tf2-ros \
  ros-humble-robot-localization \
  ros-humble-cartographer-ros
```

---
