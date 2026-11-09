# Phase 5 — Cat Detection Status & Reference

Companion to [`plan.md`](plan.md), [`phase4-status.md`](phase4-status.md),
[`phase3-status.md`](phase3-status.md), [`phase2-status.md`](phase2-status.md),
[`architecture.md`](architecture.md), and [`hardware.md`](hardware.md).

Phase 5 = build the **vision pipeline**. Two sub-stages:
- **5a — generic cat detection.** A Python ROS 2 node that runs YOLOv8 on the
  Astra color stream, publishes `vision_msgs/Detection2DArray` + an annotated
  image, and is exported to TensorRT for real-time inference.
- **5b — per-cat identification.** Use the Phase 4 patrol photos as a dataset
  *flywheel*, fine-tune a classifier that distinguishes our two cats, and (the
  C++ learning exercise) front the detector with a producer-consumer image
  pipeline that drops stale frames under load.

> **STATUS: 5a VERIFIED WORKING LIVE (2026-07-18). 5b VERIFIED WORKING LIVE
> (2026-07-21): dataset built, MobileNetV2 classifier trained (81.8% test
> acc on a small 11-image held-out set), wired into `detector_node.py`, and
> confirmed live -- publishing a known white-cat photo and a known
> brown-cat photo through the running node correctly logged "Cat sighting
> started (white)" and "(brown)" respectively. Not yet tested against the
> live Astra camera feed (only static test photos so far) or across a full
> patrol cycle.**
> Both `cat_detector` (Python) and `cat_detector_cpp` (C++) are built and
> confirmed live against the real Astra camera and a real cat: correct
> generic-cat detection, sighting-throttled saves, retention cap, audible
> beep via the real buzzer — and, critically, **confirmed compatible with a
> live Nav2 navigation goal** (goal succeeded, zero recoveries, zero Nav2
> rate warnings, detector held steady FPS throughout). See §9 Step E and
> §11 Session 1 for the full live-test record, including a real measured
> CPU/GPU cost of running the detector alongside patrol.
>
> **What's NOT done yet:** 5b (per-cat identification — still just generic
> "a cat" detection, not "which cat"); TensorRT export specifically was
> superseded by OpenCV's `cv::dnn` CUDA backend for the C++ node (§14b),
> which already clears the ≥10 FPS bar without it. Phase 6 integration
> (detector → patrol FSM reacting to a sighting) has not started — right
> now the detector runs *alongside* patrol, it does not interrupt it.

---

## 1. Purpose and scope

**Goal (from [`plan.md`](plan.md) Phase 5).** Stand up an on-board, no-cloud
vision pipeline that detects cats in the Astra color stream at usable frame
rate, and eventually tells *our two cats apart*. Detection output is a ROS 2
topic that Phase 6 will subscribe to in order to interrupt patrol, capture a
close-up, mail it with the cat's name, and bark.

**Why this phase matters (from `plan.md`).**
- **5a** teaches `cv_bridge`, the `vision_msgs` family, and TensorRT export /
  FPS measurement on a CPU-constrained Jetson.
- **5b** is the textbook **C++ concurrency** exercise of the whole project:
  a bounded, latest-wins queue between a camera-callback *producer* and a
  worker *consumer*, dropping stale frames under inference load — the exact
  `std::mutex` / `std::condition_variable` / bounded-queue pattern from
  *C++ Concurrency in Action*.

**What this phase is NOT.**
- It does **not** touch the patrol FSM or Nav2. Phase 5 only *produces*
  detections; Phase 6 *consumes* them (cancel goal → INVESTIGATING → bark).
  Keep the detector a standalone package with a clean topic contract so
  Phase 6 is a pure integration step.
- It does **not** require the robot to be driving. Most of 5a/5b can be
  developed on the bench with only the camera up (`t6_camera.sh`), which
  saves battery and keeps iteration fast.

**Scope.**
- One color camera (`/camera/color/image_raw`, Astra Pro via `t6_camera.sh`).
- YOLOv8n as the base detector ("cat" is COCO class 15 — works zero-shot).
- Two target cats for 5b (fill in real names in §11 Session 1).
- On-board inference only — GPU/TensorRT on the Orin NX, no off-board compute.

---

## 2. Data collection strategy — two capture streams feed the Phase 5b flywheel

There are now **two independent, always-on sources** writing into the Phase 5b
dataset, and neither should depend on the other:

1. **Waypoint captures (Phase 4, already running).** `patrol_manager` saves
   one JPEG per waypoint per cycle to `/tmp/cat_patrol_images`, running since
   2026-07-09, looping unattended since 2026-07-17. These are useful for "is
   there a cat in frame at all" but shot from patrol height on a fixed
   schedule — most of them won't contain a cat at all, and the ones that do
   are a narrow, top-down distribution (weak for fine-grained per-cat ID).
2. **Detection-triggered captures (Phase 5a, new in this revision).** The
   `cat_detector` node itself now saves a full frame **every time it actually
   sees a cat**, independent of patrol timing — see §3/§6c. This is the richer
   source for 5b: every saved frame is guaranteed to contain a cat, at
   whatever distance/angle/lighting it was actually sighted, not just at the
   10 fixed waypoint poses.

**Both must be persistent, not `/tmp`.** `/tmp` is wiped on reboot — every
unattended photo captured so far may already be gone across reboots. **First
action of Phase 5, before writing any other Phase 5 code:**

- Create `~/cat_patrol_data/` on the Jetson with two subdirectories:
  - `~/cat_patrol_data/captures/` — waypoint photos (Phase 4 source).
  - `~/cat_patrol_data/detections/` — sighting-triggered photos (Phase 5a source).
- Change `image_save_dir` in `patrol_manager/config/patrol_manager_params.yaml`
  from `/tmp/cat_patrol_images` to `~/cat_patrol_data/captures` (no rebuild —
  the launch loads the src YAML by absolute path, per Phase 4 Session 1).
- Point `cat_detector`'s new `detection_image_dir` param (§6b) at
  `~/cat_patrol_data/detections` — the node creates it itself on startup
  (`os.makedirs(..., exist_ok=True)`), so it survives every reboot from the
  first run onward.
- Both directories use timestamped filenames (`cat_YYYYMMDD_HHMMSS_mmm.jpg`),
  so nothing is ever overwritten and cross-referencing a capture to "what time
  did this happen" is free.
- `~/cat_patrol_data/detections/` is **capped at `max_detection_images`
  (default 500)** — the oldest files are deleted automatically once the cap
  is exceeded (§6c `_enforce_retention`), so unattended multi-week running
  can't silently fill the Jetson's storage. Curate/export a copy elsewhere
  (external drive, host machine) before a labeling pass if you want to keep
  data past the 500-image window — once evicted, it's gone.

**Note when curating for 5b (§12 has more):** because the detections/
directory only ever contains frames with a confirmed cat, and a cat sitting in
frame for a while produces many near-duplicate frames a few seconds apart
(see `sighting_gap_sec` / `detection_save_interval_sec` throttling in §6c),
dedupe visually similar frames before splitting train/val/test — otherwise
near-identical frames can leak across the split and inflate test accuracy.

---

## 3. Acceptance criteria

Phase 5 is done when the required items pass. Mirrors `plan.md`'s "Done when."

### Required — 5a (generic cat detection)
- [x] `cat_detector` package builds/installs and launches cleanly
      (`ament_python`). Confirmed 2026-07-18 — also built a second,
      independent full C++ port (`cat_detector_cpp`, §14).
- [x] Node subscribes to `/camera/color/image_raw` via `cv_bridge` and runs
      YOLOv8 inference on the GPU. Confirmed live on real CUDA hardware,
      both Python (torch) and C++ (`cv::dnn` CUDA backend, §14b).
- [x] Publishes `vision_msgs/Detection2DArray` on `/cat_detector/detections`
      and an annotated `sensor_msgs/Image` on `/cat_detector/image_annotated`.
- [x] Detections are filtered to the "cat" class with a configurable
      confidence threshold. Confirmed on a real cat photo (0.82 score) and
      live against a real cat (§9 Step A, §11).
- [x] **On every new cat sighting, the robot beeps** via the existing onboard
      buzzer (`/Buzzer` `std_msgs/Bool`, already wired to hardware in
      `Mcnamu_driver_X3.py` — no new hardware plumbing needed). One short beep
      per new sighting, not a continuous alarm while the cat stays in frame.
      **Confirmed audibly live 2026-07-18** (both detectors).
- [x] **Every cat sighting saves a timestamped frame** to a persistent
      directory (`~/cat_patrol_data/detections/`, survives reboots), with
      light throttling so a single long sighting doesn't flood the disk with
      near-duplicates (§6c: `sighting_gap_sec`, `detection_save_interval_sec`).
      This is the primary 5b dataset source (§2), independent of waypoint
      timing. Confirmed live — real sighting produced correctly-throttled
      saves in both `~/cat_patrol_data/detections/` and `detections_cpp/`.
- [x] **Retention cap on `~/cat_patrol_data/detections/`**: keep at most
      `max_detection_images` (default 500) — after each save, delete the
      oldest file(s) until back at the cap (§6c `_enforce_retention`). Bounds
      disk usage for good during unattended multi-week runs; no manual
      archiving required. Confirmed via a synthetic-cap=3 test (both
      languages) — oldest-first eviction, count settles exactly at the cap.
- [ ] ~~Model exported to TensorRT (`.engine`); FPS measured~~ — **superseded**:
      not done for the Python node; the C++ node used OpenCV's `cv::dnn` CUDA
      backend on a plain ONNX export instead (§14b) and already clears the
      FPS bar below without TensorRT. Revisit only if FPS ever becomes the
      bottleneck.
- [x] Sustained **≥ 10 FPS** cat detection while the camera is streaming
      (plan's bar). **Measured live 2026-07-18**: Python ~24–30 FPS (matches
      the camera's native 30Hz), C++ ~23–25 FPS — both comfortably clear the
      bar, including while Nav2 was actively driving a waypoint (§9 Step E).

### Required — 5b (per-cat identification)
- [ ] A curated, labeled dataset of both cats (from the flywheel + a
      deliberate close-up collection session), split train/val/test.
- [ ] A fine-tuned model (classifier head over the YOLO crop, **or** YOLO with
      two custom classes `cat_A`/`cat_B`) that distinguishes the two cats with
      **reasonable accuracy on a held-out test set** (record the confusion
      matrix in §11).
- [ ] Per-cat identity is published on the detection topic (class label /
      name + score), consumable by Phase 6 without further parsing.
- [ ] The pipeline **drops frames gracefully** under inference load — no
      unbounded memory/latency growth when the detector can't keep up.

### Nice-to-have (stretch)
- [ ] C++ producer-consumer front-end (`std::mutex` + `std::condition_variable`
      bounded queue, capacity 1, latest-wins) — the plan's headline C++ 5b
      exercise. Can be deferred; the Python node can drop-stale on its own
      first (queue-of-1 in the subscriber) and the C++ version added as the
      concurrency lesson.
- [ ] Depth fusion: attach approximate range to each detection from the Astra
      depth stream at the detection bearing (sets up Phase 6's nav-toward-cat
      stretch goal).
- [ ] Detection telemetry: FPS, queue depth, drop count published for
      operator observability (mirrors Phase 4's `/patrol_manager/state`).
- [ ] Small labeling helper (crop-and-sort script) to keep the flywheel cheap
      to curate.

---

## 4. What we need to create

| File | Purpose |
|---|---|
| `~/cat_patrol_data/captures/` | Persistent home for Phase 4 waypoint photos, moved off `/tmp` (§2) |
| `~/cat_patrol_data/detections/` | Persistent, reboot-safe dir of timestamped cat-sighting frames — created by `detector_node.py` itself on startup, not checked into git |
| `../cat_detector/` (package) | `ament_python` package for the detector node(s) |
| `../cat_detector/cat_detector/detector_node.py` | 5a: subscribe image → YOLOv8 → publish detections + annotated image |
| `../cat_detector/config/cat_detector_params.yaml` | Model path, class filter, thresholds, topics, resolution |
| `../cat_detector/launch/cat_detector.launch.py` | Bring up detector with params |
| `../cat_detector/models/` | `yolov8n.pt`, exported `.engine`, later the fine-tuned weights |
| `../cat_patrol_msgs/` (package) | Custom msgs once `vision_msgs` isn't enough (per-cat detection, later patrol status) |
| `../cat_detector/scripts/collect_dataset.py` | 5b: archive + crop + sort flywheel photos into a labeled dataset |
| `../cat_detector/scripts/export_tensorrt.py` | 5a: `.pt` → `.engine` + a before/after FPS benchmark |
| `../cat_detector/src/frame_pump_node.cpp` (stretch) | 5b C++ producer-consumer bounded queue (its own `ament_cmake` pkg if C++) |
| `~/myscripts2/t8_detector.sh` | Convenience launch wrapper (mirrors t1–t5) |

Notes:
- Package location is workspace-level: `yahboomcar_ws/src/cat_detector`.
- **5a is Python on purpose** (`plan.md`): `ultralytics` is dramatically easier
  than C++ here and it costs no C++ practice — the *interesting* concurrency is
  the 5b producer/consumer, which is where the optional C++ package goes.
- Reuse references from existing code:
  - Manual `sensor_msgs::Image` → `cv::Mat` conversion pattern (if the C++
    front-end is built): `patrol_manager/src/patrol_manager_node.cpp`
    `save_current_image()` (avoids a `cv_bridge` ABI dependency in C++).
  - Existing detection scaffolding for reference only:
    `yahboomcar_visual/detection/target_detection.py` (OpenCV-DNN SSD, COCO
    labels in `object_detection_coco.txt`) — *superseded* by YOLOv8, but a
    useful map of the class list and annotation style.
  - Camera bring-up: `~/myscripts2/t6_camera.sh` (Astra color+depth; sources
    `software/library_ws`, `uvc_product_id:=0x050f`). The staged t1–t4 do
    **not** start the camera — Phase 5 always needs t6 up.

---

## 5. Environment already confirmed present (2026-07-18)

Checked on the Jetson before drafting — no install needed to start 5a:

- `ultralytics 8.3.65`
- `torch 2.5.0a0 (nv24.08)`, **CUDA available: True**
- `opencv 4.10.0`, `cv_bridge` importable in Python
- Astra color topic name: `/camera/color/image_raw` (confirm live rate with
  `ros2 topic hz` once `t6_camera.sh` is up).
- **Buzzer already wired end-to-end** — `Mcnamu_driver_X3.py` subscribes
  `std_msgs/Bool` on topic `Buzzer` (`Mcnamu_driver_X3.py:90`) and toggles the
  real onboard buzzer; `patrol_node.cpp`/`till_obstacle_back_pattern.cpp`
  already use this exact mechanism for capture beeps. `cat_detector` reuses
  the same topic — no new hardware plumbing, no new driver code.

Still to verify in Session 1:
- `vision_msgs` installed: `ros2 interface show vision_msgs/msg/Detection2DArray`
  → if missing, `sudo apt install ros-humble-vision-msgs`.
- TensorRT export path works on this JetPack 6.1 / L4T 36.4.3 stack
  (`ultralytics` uses `torch2trt`/`onnx`→`trtexec`; the exact route can be
  finicky on aarch64 — budget time, this is the classic Jetson friction point
  the plan warns about, "4–6 weeks" honest timeline).

---

## 6. Files to create (before first session)

### 6a. Create the package scaffold (5a, Python)

```bash
cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src
ros2 pkg create cat_detector \
  --build-type ament_python \
  --dependencies rclpy sensor_msgs vision_msgs std_msgs cv_bridge

mkdir -p cat_detector/config cat_detector/launch cat_detector/models cat_detector/scripts
```

### 6b. `../cat_detector/config/cat_detector_params.yaml`

```yaml
cat_detector:
  ros__parameters:
    # --- Model ---
    model_path: "/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_detector/models/yolov8n.pt"
    # After TensorRT export, point this at the .engine instead:
    # model_path: ".../models/yolov8n.engine"
    device: "cuda:0"
    imgsz: 640

    # --- Class filtering ---
    # COCO 'cat' == class 15. For the 5b two-cat model, switch to the
    # custom class ids and names of the fine-tuned weights.
    target_classes: [15]
    class_names: {15: "cat"}
    conf_threshold: 0.45
    iou_threshold: 0.50

    # --- Topics ---
    image_topic: "/camera/color/image_raw"
    detections_topic: "/cat_detector/detections"
    annotated_topic: "/cat_detector/image_annotated"

    # --- Throughput / drop-stale ---
    publish_annotated: true
    max_input_hz: 15.0        # skip frames above this to protect the GPU
    log_fps_every_sec: 5.0

    # --- Persistent dataset capture on sighting (Phase 5b flywheel) ---
    save_on_detection: true
    detection_image_dir: "/home/jetson/cat_patrol_data/detections"
    sighting_gap_sec: 3.0             # no-detection gap after which the next
                                       # detection counts as a NEW sighting
    detection_save_interval_sec: 5.0  # while one sighting continues, save
                                       # another frame at most this often
    max_detection_images: 500         # retention cap; oldest deleted past this

    # --- Beep on sighting ---
    beep_on_detection: true
    beep_topic: "Buzzer"               # same topic Mcnamu_driver_X3.py already
                                        # subscribes to for the real buzzer
    beep_duration_sec: 0.15
```

### 6c. `../cat_detector/cat_detector/detector_node.py` (5a starter)

Intentionally compact — get one detection flowing, then harden. Uses a
**latest-wins queue of 1** in the subscriber so the detector never backs up
behind the camera (the Python analogue of the 5b C++ bounded queue).

```python
#!/usr/bin/env python3
import datetime
import glob
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import cv2
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray, Detection2D, \
    ObjectHypothesisWithPose, BoundingBox2D


class CatDetector(Node):
    def __init__(self):
        super().__init__("cat_detector")

        self.model_path = self.declare_parameter("model_path", "yolov8n.pt").value
        self.device = self.declare_parameter("device", "cuda:0").value
        self.imgsz = int(self.declare_parameter("imgsz", 640).value)
        self.target_classes = list(self.declare_parameter("target_classes", [15]).value)
        self.conf = float(self.declare_parameter("conf_threshold", 0.45).value)
        self.iou = float(self.declare_parameter("iou_threshold", 0.50).value)
        image_topic = self.declare_parameter("image_topic", "/camera/color/image_raw").value
        det_topic = self.declare_parameter("detections_topic", "/cat_detector/detections").value
        ann_topic = self.declare_parameter("annotated_topic", "/cat_detector/image_annotated").value
        self.publish_annotated = bool(self.declare_parameter("publish_annotated", True).value)
        self.log_fps_every = float(self.declare_parameter("log_fps_every_sec", 5.0).value)

        # --- Persistent dataset capture on sighting (Phase 5b flywheel) ---
        self.save_on_detection = bool(self.declare_parameter("save_on_detection", True).value)
        self.detection_image_dir = self.declare_parameter(
            "detection_image_dir", "/home/jetson/cat_patrol_data/detections").value
        self.sighting_gap_sec = float(self.declare_parameter("sighting_gap_sec", 3.0).value)
        self.detection_save_interval_sec = float(
            self.declare_parameter("detection_save_interval_sec", 5.0).value)
        self.max_detection_images = int(
            self.declare_parameter("max_detection_images", 500).value)
        os.makedirs(self.detection_image_dir, exist_ok=True)
        self._last_seen_time = None    # monotonic time of the last frame with a cat
        self._last_saved_time = None   # monotonic time of the last saved dataset frame

        # --- Beep on sighting ---
        self.beep_on_detection = bool(self.declare_parameter("beep_on_detection", True).value)
        beep_topic = self.declare_parameter("beep_topic", "Buzzer").value
        self.beep_duration_sec = float(self.declare_parameter("beep_duration_sec", 0.15).value)
        self.beep_pub = (
            self.create_publisher(Bool, beep_topic, 10) if self.beep_on_detection else None
        )

        self.get_logger().info(f"Loading model {self.model_path} on {self.device}")
        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()

        # Latest-wins queue of 1: the callback only STORES the newest frame;
        # a timer does the (slow) inference. If inference is slower than the
        # camera, intermediate frames are simply overwritten -> dropped stale,
        # bounded memory. This is the drop-stale requirement (§3) in Python.
        self._latest = None

        self.det_pub = self.create_publisher(Detection2DArray, det_topic, 10)
        self.ann_pub = self.create_publisher(Image, ann_topic, 10) if self.publish_annotated else None
        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)

        # Inference loop runs as fast as it can; overwrite semantics do the throttling.
        self.create_timer(0.001, self._infer)
        self._frames = 0
        self._t0 = time.monotonic()

    def _on_image(self, msg):
        self._latest = msg  # overwrite -> latest-wins

    def _infer(self):
        msg = self._latest
        if msg is None:
            return
        self._latest = None  # consume

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model.predict(
            frame, imgsz=self.imgsz, conf=self.conf, iou=self.iou,
            classes=self.target_classes, device=self.device, verbose=False)
        r = results[0]

        out = Detection2DArray()
        out.header = msg.header
        for b in r.boxes:
            cls_id = int(b.cls[0])
            score = float(b.conf[0])
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            d = Detection2D()
            d.header = msg.header
            bb = BoundingBox2D()
            bb.center.position.x = (x1 + x2) / 2.0
            bb.center.position.y = (y1 + y2) / 2.0
            bb.size_x = float(x2 - x1)
            bb.size_y = float(y2 - y1)
            d.bbox = bb
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(cls_id)
            hyp.hypothesis.score = score
            d.results.append(hyp)
            out.detections.append(d)
        self.det_pub.publish(out)

        if out.detections:
            self._handle_sighting(frame)

        if self.ann_pub is not None:
            annotated = r.plot()  # BGR np.ndarray with boxes drawn
            ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            ann_msg.header = msg.header
            self.ann_pub.publish(ann_msg)

        self._frames += 1
        now = time.monotonic()
        if now - self._t0 >= self.log_fps_every:
            fps = self._frames / (now - self._t0)
            self.get_logger().info(f"inference {fps:.1f} FPS, {len(out.detections)} det")
            self._frames = 0
            self._t0 = now

    def _handle_sighting(self, frame):
        """Called once per frame that contains >=1 cat detection.

        Beeps once per NEW sighting (a gap of sighting_gap_sec with no cat
        counts as "new"), and saves a timestamped frame for the Phase 5b
        dataset -- immediately on a new sighting, then throttled to at most
        one more every detection_save_interval_sec while it continues, so a
        cat napping in frame for ten minutes doesn't fill the disk with
        near-identical images.
        """
        now = time.monotonic()
        is_new_sighting = (
            self._last_seen_time is None
            or (now - self._last_seen_time) > self.sighting_gap_sec
        )
        self._last_seen_time = now

        if is_new_sighting:
            self.get_logger().info("Cat sighting started")
            if self.beep_on_detection:
                self._beep()
            if self.save_on_detection:
                self._save_frame(frame)
                self._last_saved_time = now
        elif self.save_on_detection and (
            self._last_saved_time is None
            or (now - self._last_saved_time) >= self.detection_save_interval_sec
        ):
            self._save_frame(frame)
            self._last_saved_time = now

    def _beep(self):
        on = Bool()
        on.data = True
        self.beep_pub.publish(on)
        # Buzzer driver just toggles a GPIO on/off (Mcnamu_driver_X3.py); turn
        # it back off after one short pulse via a plain Python timer so a
        # sighting doesn't leave it latched on. threading.Timer fires off the
        # rclpy executor thread, but a simple publish() call from it is fine.
        threading.Timer(self.beep_duration_sec, self._beep_off).start()

    def _beep_off(self):
        off = Bool()
        off.data = False
        self.beep_pub.publish(off)

    def _save_frame(self, frame):
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        fp = os.path.join(self.detection_image_dir, f"cat_{ts}.jpg")
        if cv2.imwrite(fp, frame):
            self.get_logger().info(f"Saved detection frame: {fp}")
            self._enforce_retention()

    def _enforce_retention(self):
        # Filenames are cat_<timestamp>.jpg, so a plain name sort is also a
        # chronological sort -- oldest files come first, no stat() calls needed.
        files = sorted(glob.glob(os.path.join(self.detection_image_dir, "cat_*.jpg")))
        excess = len(files) - self.max_detection_images
        for fp in files[:max(excess, 0)]:
            try:
                os.remove(fp)
            except OSError as e:
                self.get_logger().warning(f"Failed to evict {fp}: {e}")


def main():
    rclpy.init()
    node = CatDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

> **Note on the timer loop:** the `0.001s` timer + overwrite pattern is a
> deliberately simple first cut. The `vision_msgs` field names above
> (`hypothesis.class_id`, `center.position.x`) are the **Humble** layout —
> verify against `ros2 interface show` in Session 1, they changed between
> distros. This is exactly the kind of thing to smoke-test before trusting it.

### 6d. `../cat_detector/launch/cat_detector.launch.py`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = DeclareLaunchArgument(
        "params_file",
        default_value="/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_detector/config/cat_detector_params.yaml",
        description="Full path to cat_detector params YAML",
    )
    node = Node(
        package="cat_detector",
        executable="detector_node",
        name="cat_detector",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )
    return LaunchDescription([params, node])
```

Wire `detector_node` as a console_script in `setup.py`
(`entry_points={'console_scripts': ['detector_node = cat_detector.detector_node:main']}`)
and add `config`/`launch`/`models` to `data_files`.

### 6e. `../cat_detector/scripts/export_tensorrt.py` (5a)

```python
#!/usr/bin/env python3
"""Export yolov8n.pt -> TensorRT .engine and benchmark FPS before/after."""
import time
import numpy as np
from ultralytics import YOLO

PT = "/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_detector/models/yolov8n.pt"
IMGSZ = 640
dummy = (np.random.rand(IMGSZ, IMGSZ, 3) * 255).astype("uint8")


def bench(model, n=100):
    for _ in range(10):
        model.predict(dummy, imgsz=IMGSZ, device="cuda:0", verbose=False)
    t0 = time.monotonic()
    for _ in range(n):
        model.predict(dummy, imgsz=IMGSZ, device="cuda:0", verbose=False)
    return n / (time.monotonic() - t0)


pt = YOLO(PT)
print(f"PyTorch .pt : {bench(pt):.1f} FPS")

# half=True (FP16) is the usual Orin win; int8 needs a calib set.
pt.export(format="engine", imgsz=IMGSZ, half=True, device=0)
engine = YOLO(PT.replace(".pt", ".engine"))
print(f"TensorRT .engine (fp16): {bench(engine):.1f} FPS")
```

### 6f. `../cat_detector/scripts/collect_dataset.py` (5b flywheel)

Skeleton — Session where 5b starts:
- Archive `~/cat_patrol_data/captures/*.jpg` (persistent, per §2) into
  `~/cat_patrol_data/raw/<date>/`.
- Run the 5a detector over each, crop the "cat" bbox, drop crops into
  `dataset/unsorted/`.
- Manually sort `unsorted/` into `dataset/cat_A/`, `dataset/cat_B/`,
  `dataset/none/` (or use a tiny Tk/CLI sorter).
- Emit a train/val/test split manifest for the fine-tune step.

### 6g. `~/myscripts2/t8_detector.sh`

```bash
cat > ~/myscripts2/t8_detector.sh << 'EOF'
#!/usr/bin/env bash
export ROS_DOMAIN_ID=28
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds.xml
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch cat_detector cat_detector.launch.py
EOF
chmod +x ~/myscripts2/t8_detector.sh
```

### 6h. Build

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
colcon build --symlink-install --packages-select cat_detector
source install/setup.bash
ros2 pkg executables cat_detector | grep detector_node
```

---

## 7. Architecture

```
Jetson
──────────────────────────────────────────────────────────────────────────
t6_camera.sh  Astra color+depth  (/camera/color/image_raw, /camera/depth/...)

NEW (Phase 5)
t8_detector.sh cat_detector
  ├─ sub  /camera/color/image_raw   (cv_bridge -> cv::Mat / np.ndarray)
  ├─ [5b stretch] C++ frame_pump: bounded queue(1), latest-wins, drop-stale
  ├─ YOLOv8 inference on GPU (.pt -> .engine TensorRT)
  ├─ pub  /cat_detector/detections        (vision_msgs/Detection2DArray)
  ├─ pub  /cat_detector/image_annotated    (sensor_msgs/Image, boxes drawn)
  ├─ on new sighting: pub Buzzer=true, then =false after beep_duration_sec
  │        (same /Buzzer topic Mcnamu_driver_X3.py already wires to hardware)
  └─ on sighting (throttled): save frame -> ~/cat_patrol_data/detections/
           (persistent, timestamped, capped at max_detection_images=500 with
           oldest-first eviction -- Phase 5b flywheel, independent of patrol
           waypoint timing)

Phase 6 (later) consumes /cat_detector/detections:
  patrol_manager --sub detections--> on confident cat -> cancel nav goal
                                     -> INVESTIGATING -> capture/mail/bark
```

The detector is **decoupled from patrol** by design: it publishes a topic and
knows nothing about the FSM. That is what makes Phase 6 a pure integration.

---

## 8. Pre-flight checklist

### 8a. One-time before first session
- [ ] Move Phase 4 capture dir off `/tmp` (§2) so the flywheel survives reboots.
- [ ] Confirm `vision_msgs` installed (§5).
- [ ] Download `yolov8n.pt` into `cat_detector/models/`.
- [ ] Build `cat_detector`, verify `detector_node` executable appears.

### 8b. Every session
- [ ] `t6_camera.sh` up; `ros2 topic hz /camera/color/image_raw` shows a
      steady rate before starting the detector.
- [ ] GPU not already saturated (`tegrastat`s / `jtop`) — the Astra node alone
      eats ~75–85% of a CPU core (per Phase 4 notes); watch thermals/CPU too.
- [ ] For a live patrol+detect run: t1→t5 + t6 + t8 all up (heavy — check
      battery, this is the most loaded the robot has ever run).
- [ ] `t1.sh` (which starts `Mcnamu_driver_X3.py`) is up before
      `t8_detector.sh` — the beep is a silent no-op if nothing is subscribed
      to `/Buzzer`.
- [ ] Spot-check `~/cat_patrol_data/detections/` occasionally — retention
      caps it at `max_detection_images` (default 500) automatically, but
      confirm the count isn't unexpectedly stuck below that (would mean
      `_enforce_retention` is over-deleting, or saves are failing silently).

---

## 9. Session procedure

### Step A — camera + detector only (bench, no driving)
1. `bash ~/myscripts2/t6_camera.sh`
2. `bash ~/myscripts2/t8_detector.sh` (bring up `t1.sh` first too, so the
   buzzer hardware subscriber is alive)
3. Watch: `ros2 topic hz /cat_detector/detections` and the node's FPS log line.
4. Point the camera at a cat (or a cat photo/phone) — confirm a detection
   fires and the annotated topic shows a box (view in RViz Image display on
   the host, or `ros2 run rqt_image_view rqt_image_view`).
5. Confirm the beep fires once per new sighting (not continuously), and that
   a new file appears in `~/cat_patrol_data/detections/` within a second or
   two. Walk the cat/photo out of frame and back in after >`sighting_gap_sec`
   — confirm it beeps again (new sighting), and while held continuously in
   frame, confirm it does NOT beep again but does save another frame roughly
   every `detection_save_interval_sec`.

### Step B — TensorRT export + benchmark (5a done-when)
1. `python3 src/cat_detector/scripts/export_tensorrt.py`
2. Record `.pt` vs `.engine` FPS in §11. Switch `model_path` to the `.engine`
   and re-run Step A to confirm ≥10 FPS live.

### Step C — dataset collection (5b flywheel)
1. Run patrol cycles (t1→t5+t6) and/or a deliberate close-up session; let
   photos accumulate in the persistent capture dir.
2. `collect_dataset.py` → crop → sort into `cat_A`/`cat_B`/`none`.
3. Repeat across days until a few hundred usable crops per cat.

### Step D — fine-tune + evaluate (5b done-when)
1. Train (classifier head or 2-class YOLO). 2. Evaluate on held-out test set;
   record accuracy + confusion matrix in §11. 3. Point `model_path` /
   `target_classes` / `class_names` at the new weights, re-run Step A.

### Step E — full patrol WITH cat recognition running (confirmed working 2026-07-18)

Recognition runs **alongside** patrol right now, not integrated into its FSM
(that's Phase 6). This just means: bring up the usual Phase 4 stack, plus the
camera, plus one detector.

```bash
# 1. Base bringup (chassis, IMU, EKF) -- also owns the buzzer hardware
bash ~/myscripts2/t1.sh

# 2. LiDAR
bash ~/myscripts2/t2.sh

# 3. Front-arc scan filter
bash ~/myscripts2/t2.5.sh

# 4. AMCL localization
bash ~/myscripts2/t3_amcl.sh

# 5. Nav2 (this also stops the ollama service automatically -- see its own
#    "Free RAM before the heaviest stack" comment; not a detector-specific step)
bash ~/myscripts2/t4_nav.sh

# 6. Camera -- needed by BOTH patrol_manager's own waypoint capture AND the detector
bash ~/myscripts2/t6_camera.sh

# 7. Cat detector -- pick ONE, not both (see caution below)
bash ~/myscripts2/t8_detector.sh        # Python
# or
bash ~/myscripts2/t9_detector_cpp.sh    # C++

# 8. Start the actual patrol cycle
bash ~/myscripts2/t5_patrol.sh
```

**Before step 8**, AMCL needs an initial pose or every patrol nav goal will
just fail:
- Click "2D Pose Estimate" in RViz on the host at the robot's real position, or
- `ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: <x>, y: <y>, z: 0.0}, orientation: {z: <sin(yaw/2)>, w: <cos(yaw/2)>}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}"`

**Only run one detector, not both** — both publish beeps to the same
physical `/Buzzer` (double-beep on a shared sighting) and both compete for
the same GPU/CPU for no benefit during a real patrol.

**Empirically confirmed safe to combine** (§11 Session 1): running one
detector alongside the full nav stack raised average CPU from 37% to 49%
(GPU 0%→62%) with Nav2 idle, and a live `NavigateToPose` goal sent with the
detector running **succeeded with zero recoveries and zero Nav2 rate
warnings**, detector holding steady FPS throughout. Not yet tested across a
long, multi-waypoint unattended cycle — worth keeping an eye on `ros2 topic
hz /cmd_vel` and the `t4_nav` terminal for rate warnings the first few times.

---

## 10. Pitfalls and recovery

- **Camera not up / wrong topic**
  - Symptom: detector idle, no FPS log.
  - Fix: `t6_camera.sh` must run first (t1–t4 don't start it); confirm the
    topic name matches `image_topic`.
- **TensorRT export fails on aarch64 / JetPack**
  - Symptom: `export(format="engine")` errors mid-ONNX or `trtexec`.
  - Fix: this is the expected Jetson friction (plan's honest 4–6wk note). Try
    exporting ONNX first, then `trtexec` manually; match TensorRT version to
    the JetPack. Keep running the `.pt` model in the meantime — it still works.
- **`vision_msgs` field mismatch**
  - Symptom: `AttributeError` on `hypothesis.class_id` / `center.position`.
  - Fix: `ros2 interface show vision_msgs/msg/Detection2D` — the layout in
    §6c is Humble; adjust to what's actually installed.
- **GPU/CPU saturation during live patrol**
  - Symptom: FPS collapses, Nav2 loop-rate warnings, laggy state.
  - Fix: lower `imgsz` / `max_input_hz`, use the `.engine` not `.pt`, or run
    the detector only during CAPTURE/INVESTIGATING rather than continuously.
    Remember the Orin NX is CPU-bound and Nav2 already competes for it.
- **Unbounded memory / latency (no drop-stale)**
  - Symptom: RAM climbs, detections lag further behind reality over time.
  - Fix: the latest-wins queue-of-1 (§6c) must actually overwrite, not buffer.
    Verify `_latest` is a single slot, not a list; QoS depth on the sub is
    small. This is the exact failure the 5b bounded-queue exercise prevents.
- **Two cats look alike / dataset too small**
  - Symptom: 5b test accuracy near chance.
  - Fix: more close-up data (patrol-height shots aren't enough — §2/§12),
    augment, or fall back to "generic cat" for Phase 6 and treat per-cat ID
    as an ongoing flywheel improvement rather than a hard gate.
- **Beep never fires**
  - Symptom: detections publish fine, no sound.
  - Fix: confirm `Mcnamu_driver_X3.py` is actually running (`t1.sh`) and
    subscribed to `Buzzer` — `ros2 topic info /Buzzer -v` should show one
    subscriber; confirm `beep_topic` param matches exactly (`"Buzzer"`, no
    leading slash, case-sensitive) and `beep_on_detection: true`.
- **Beeps constantly / never stops while a cat is around**
  - Symptom: buzzer chatters continuously instead of one beep per sighting.
  - Fix: this means new "sightings" are being declared too often — check
    `_last_seen_time` is actually being updated every detected frame
    (§6c `_handle_sighting`), and that detection itself isn't flickering
    on/off every other frame (lower `conf_threshold` slightly, or check the
    cat isn't right at the edge of frame causing the box to disappear/reappear).
- **`~/cat_patrol_data/detections/` not staying near 500 files**
  - Symptom: count keeps climbing well past `max_detection_images`.
  - Fix: `_enforce_retention()` (§6c) only runs after a *successful* save —
    if `cv2.imwrite` silently fails (bad path, permissions, disk full) it's
    never called at all that frame; check node logs for `Failed to evict`
    warnings and confirm the directory is actually writable.
  - Symptom: count stuck noticeably *below* 500 despite frequent sightings.
    Fix: likely legitimate — the cap is a ceiling, not a target; 500 real
    sightings takes time to accumulate. Not a bug unless sightings are
    frequent and the count still doesn't grow.
  - Remember: the cap discards the *oldest* frames first, so a labeling
    session should pull data out (§2) before assuming it'll still be there
    weeks later.

---

## 11. Phase 5 session log

> Fill this in as sessions happen — same running-log style as
> [`phase4-status.md`](phase4-status.md) §11. Record what was *actually*
> observed (FPS numbers, export route that worked, confusion matrix), not just
> what was attempted.

### Session 1 — 2026-07-18

- [x] Cat names / which is `cat_A` vs `cat_B`: not decided yet — this session
      only exercised generic (5a) detection, no per-cat identity involved.
- [x] `~/cat_patrol_data/{captures,detections,detections_cpp,dataset}/`
      created; `patrol_manager`'s `image_save_dir` repointed from
      `/tmp/cat_patrol_images` to `~/cat_patrol_data/captures`.
- [x] `vision_msgs` present: **no, had to install** `ros-humble-vision-msgs`
      (wasn't on the system). Field layout confirmed live against
      `ros2 interface show` for Humble 4.1.1 — matches what's coded (§6c/§14a):
      `BoundingBox2D.center.position.{x,y}`, `ObjectHypothesisWithPose
      .hypothesis.{class_id,score}`.
- [x] First live detection on `.pt`: FPS 24–30 (Python) / 23–25 (C++), worked:
      yes — first tested on a real cat photo from an existing local dataset
      (`class_id=15`, score 0.82), then live against a real cat via the Astra
      camera.
- [x] Beep confirmed audible on new sighting (heard live with `t1.sh`'s
      `Mcnamu_driver_X3` up as the real `/Buzzer` subscriber), silent during a
      held sighting, beeps again after a fresh gap: **yes**, both detectors.
- [x] First detection-triggered frames appeared in
      `~/cat_patrol_data/detections/`: **13 frames** (Python) and
      **12 frames** (C++ `detections_cpp/`) from one real cat's visit.
- [x] Retention verified: dropped `max_detection_images` to 3 in an isolated
      logic test (Python: direct method calls; C++: live ROS test against a
      real cat photo) — both settled at exactly the cap with oldest-first
      eviction.
- [x] **Nav2 compatibility measured, not assumed** (this wasn't originally
      planned for Session 1, but came up when asked "why would GPU inference
      compete with Nav2's CPU?"): brought up the full t1–t4 stack + camera,
      measured `tegrastats` baseline (nav idle, no detector) vs. with one
      detector running. CPU 36.8%→48.8% avg across 6 cores (+12pp), GPU
      0%→61.6% (GR3D_FREQ) — confirms the YOLO forward pass really is GPU-only,
      but the surrounding ROS/OpenCV pipeline (deserialize, `blobFromImage`,
      NMS/box decode, publish, JPEG encode on save) is real, measurable CPU
      cost, not free just because inference is on CUDA. Then sent a real
      `NavigateToPose` goal to wp1 with the detector running: **goal
      SUCCEEDED, 0 recoveries, 0 Nav2 rate-degradation warnings**, detector
      held 30 FPS the whole drive.
- Notes: hit and fixed two real operational bugs along the way — (1) `/tmp`
  is wiped on reboot so had to persist capture dirs before anything else
  (§2); (2) a orphaned `astra_camera_node` process survived an earlier
  teardown (only the `ros2 launch` parent had been killed, not the child),
  blocking the depth stream on the next camera bring-up with "Resource busy"
  — fixed by killing by exact PID and being more careful about full
  process-tree teardown afterward. `t4_nav.sh` also stops the `ollama`
  systemd service as an existing (not new) OOM-safety measure; restored/
  re-stopped it per operator preference after testing (kept **stopped**, per
  final instruction).

### Session 2 — 2026-07-21

- Cat names: still generic "white"/"brown" in code and data dirs -- user
  chose not to introduce real names, keep it simple.
- Photo transfer: user copied 46 white-cat + 28 brown-cat photos from host
  to `~/cats/white/` and `~/cats/brown/` via rsync/scp (mixed `.jpg`/`.jpeg`/
  `.JPG` extensions, all handled).
- `collect_dataset.py` rewritten to handle two kinds of sources: pre-labeled
  (`~/cats/white`, `~/cats/brown` -> crop straight into `dataset/<label>/`,
  passthrough uncropped if YOLO finds no box rather than dropping the photo)
  and unlabeled flywheel dirs (`captures/`, `detections/`, `detections_cpp/`
  -> `dataset/unsorted/` for manual review). Idempotent re-runs via a
  `__crop<n>`/`passthrough__` filename marker.
- First run produced `dataset/white` (46), `dataset/brown` (28),
  `dataset/unsorted` (45, later found 24 were exact-duplicate crops of
  photos already in `dataset/white` -- see below -- removed, leaving 21).
- **Finding: most "white cat" host photos were already-known robot data.**
  24 of the 46 white-cat photos had the exact filename pattern
  `cat_YYYYMMDD_HHMMSS_mmm.jpg` that `detector_node.py`/`detector_node.cpp`
  generate, and were confirmed byte-identical to files already in
  `~/cat_patrol_data/detections*`. So the user's "many white cat photos"
  were substantially the Session 1 flywheel captures, not new deliberate
  shots -- true novel white-cat data is closer to ~12-22 images. Brown
  barely overlapped (3 of 28).
- User decided **not** to manually sort the remaining 21 `dataset/unsorted/`
  crops -- 74 pre-labeled images was judged enough to start, and sorting 21
  more images by eye (via rsync round-trip to the host, since the Jetson has
  no display) wasn't worth it right now. `dataset/unsorted/` deleted.
- `train_classifier.py` written: MobileNetV2 (ImageNet-pretrained, frozen
  backbone, fine-tune only the linear head -- appropriate given ~75 total
  images), group-aware train/val/test split (60s sighting buckets so
  near-duplicate frames from one sighting can't leak across the split, per
  the dedup warning in §2), mild augmentation (flip/rotation/brightness) with
  **no hue/saturation jitter** since color is the actual class signal.
  Trained 25 epochs, best val checkpoint kept. **Test acc 81.8% (9/11)**,
  all errors brown->white (see §11 5b final results above for full
  breakdown). Saved to
  `cat_detector/models/cat_id_classifier.pt` + `.json` metadata.
- **Classifier wired into `detector_node.py`**: after YOLO finds a "cat" box,
  the box (padded by `ID_CROP_PAD_FRAC=0.15`, matching `collect_dataset.py`'s
  `PAD_FRAC` so live crops match the training distribution) is cropped and
  run through the MobileNetV2 classifier. Result is appended as a *second*
  `ObjectHypothesisWithPose` on the same `Detection2D` (first stays
  `class_id="15"` for back-compat with any generic-cat consumer), so Phase 6
  can read either the generic detection or the per-cat identity without a
  message-contract change. New params: `enable_identification` (default
  true), `classifier_model_path`. Falls back to identification-disabled with
  a warning if the `.pt` file is missing, rather than crashing the node.
  Annotated image now also gets the label+confidence drawn on it
  (`cv2.putText`), and saved detection filenames get a `_white`/`_brown`
  suffix (after the timestamp, so `_enforce_retention`'s chronological sort
  still works) for easier dataset review later.
- **Verified live** (not just built): built `cat_detector` cleanly, launched
  the real node, and used a new helper script (`publish_test_image.py` --
  publishes one static JPEG on `/camera/color/image_raw` repeatedly, no
  camera hardware needed) to replay one known white-cat photo and one known
  brown-cat photo through the running node. Log output confirmed: `Cat
  sighting started (white)` for the white photo, `Cat sighting started
  (brown)` for the brown photo -- correct both directions, end to end
  (YOLO detect -> crop -> classify -> beep/save/log). One test photo choice
  mattered: the first photo tried had been a `passthrough__` file during
  `collect_dataset.py` (no cat found even at conf=0.25), so naturally
  produced 0 detections live too at the stricter conf=0.45 -- picked a
  different, actually-detected photo and it worked immediately. Test
  artifacts (saved frames, logs) cleaned up after; `publish_test_image.py`
  kept as a permanent script (bench-testing any image subscriber without
  live camera hardware).
- **Ported identification to `cat_detector_cpp` too** (user asked whether
  the C++ node also had it -- it didn't, this was a separate follow-up).
  `cat_detector_cpp` has no libtorch dependency (only `cv::dnn`, per §14b),
  so `cat_id_classifier.pt` can't be loaded directly in C++ -- new
  `export_classifier_onnx.py` exports it to ONNX the same way `yolov8n.onnx`
  was exported. Verified numerically before touching C++: compared
  PyTorch-vs-cv::dnn logits on 6 real crops, using `cv2.dnn.blobFromImage`
  for the NHWC->NCHW/batch step but manual per-channel mean/std math (a
  single `blobFromImage` scalefactor can't do ImageNet's per-channel std
  division). Absolute logit values differ a bit between backends (PIL vs
  OpenCV resize aren't bit-exact), but predicted class matched on every one
  of the 6 test crops. `INTER_AREA` was picked for the downscale specifically
  because a bench comparison showed it tracks PIL's antialiased
  `Resize(256)` noticeably closer than `INTER_LINEAR`.
  `detector_node.cpp` changes: `crop_for_identification` (pads by
  `kIdCropPadFrac=0.15`, matching Python/`collect_dataset.py`),
  `preprocess_for_classifier` (resize-256-shortest-side + center-crop-224 +
  manual ImageNet normalize), `classify_crop` (forward pass + manual
  softmax since the exported graph has no softmax layer). New params:
  `enable_identification`, `classifier_model_path`, `classifier_classes`
  (order must match `export_classifier_onnx.py`'s printed class order --
  currently `["brown", "white"]`). Same fallback behavior as Python: missing
  classifier file -> warn and disable, not crash.
  **Verified live**: built cleanly (`colcon build --packages-select
  cat_detector_cpp`), launched the real node, replayed the same known
  white/brown test photos via `publish_test_image.py` (works for either
  detector, it just publishes on the shared `/camera/color/image_raw`
  topic) -- logged `Cat sighting started (white)` and `(brown)` correctly,
  same as the Python node. Saved-frame filenames also get the
  `_white`/`_brown` suffix now, matching the Python side.
- Next: run this live against the real Astra camera (not just a static
  photo) and, ideally, get a real-world read on accuracy beyond the 11-image
  test set -- watch the annotated image / logs during normal patrol and see
  how often the identification looks right by eye. This applies to both
  detectors now.

### 5a final results
- Model / imgsz: YOLOv8n / 640×640 (`yolov8n.pt` for Python, `yolov8n.onnx`
  export for C++)
- `.pt` FPS: 24–30 live (Python, full ROS pipeline). `.engine`: not built —
  superseded by the C++ `cv::dnn` CUDA path instead (see below).
- TensorRT export route: **not exercised** — the C++ node used a plain ONNX
  export (`ultralytics`, `opset=12, simplify=True`) run through OpenCV's
  `cv::dnn::readNetFromONNX` + `DNN_BACKEND_CUDA`/`DNN_TARGET_CUDA` instead.
  Raw benchmark (no ROS overhead): **52 FPS CUDA vs. 8.9 FPS CPU** for
  yolov8n @ 640×640 on this Jetson.
- Sustained live FPS with camera: **24–30 FPS (Python), 23–25 FPS (C++)**
  — both well over the ≥10 FPS target, including while Nav2 was actively
  navigating.
- 5a acceptance: **all required items met** (see checkboxes in §3). TensorRT
  item specifically superseded, not failed — the underlying goal (real-time
  GPU inference) was met a different way.

### 5b final results
- Dataset size (per cat, train/val/test): brown 28 photos, white 46 photos
  (raw counts before crop; note ~34 of the "white" photos turned out to be
  exact-duplicate re-copies of frames the robot itself had already saved to
  `~/cat_patrol_data/detections*` during Session 1 -- so genuinely novel
  white-cat data is closer to ~12 deliberate photos, not 46. Brown barely
  overlapped, ~25 of 28 were new). Group-aware split (60s sighting buckets
  so near-duplicate frames never cross train/test): train 56, val 7, test 11.
- Model type: crop-classifier (MobileNetV2, ImageNet-pretrained, backbone
  frozen, only the final linear head fine-tuned -- chosen over 2-class YOLO
  because the dataset is far too small, ~75 images total, to retrain a
  detection head reliably). `train_classifier.py` (§6f note: this superseded
  the collect_dataset.py described there being unsorted-only -- it also
  handles pre-labeled sources, see script docstring for both source kinds).
- Test accuracy + confusion matrix: **81.8%** (9/11). Confusion (rows=true,
  cols=pred): brown -> [5 brown, 2 white], white -> [0 brown, 4 white]. All
  errors were brown misclassified as white, none the other way -- worth
  watching as more data comes in (could be sample noise given the tiny
  11-image test set, or a real lighting/background confusion). Not a
  precise number given how few test images there are; treat as a rough
  first-pass signal, re-evaluate once more real sightings accumulate.
- Drop-stale verified (no unbounded growth under load): not yet re-verified
  with the classifier wired in -- carries over from 5a's queue-of-1 design
  (§6c), same mechanism, classifier adds one more (small) synchronous step
  per detected frame.
- C++ producer-consumer built (stretch): no.
- 5b acceptance: **dataset + model done, integration into detector_node.py
  in progress** -- see below for wiring status once complete.

---

## 12. Open questions / forward links

- **Patrol-height vs sighting-triggered distribution (§2).** The new
  detections/ stream is richer than waypoint photos (any distance/angle the
  cat was actually seen at, not just 10 fixed poses), but still worth checking
  once real data accumulates: is it *close enough* for fine-grained per-cat ID,
  or does 5b still need a deliberate close-up collection session? Decide after
  looking at a week or two of real `~/cat_patrol_data/detections/` output.
- **Is 500 images the right cap?** Chosen as a simple, easy-to-reason-about
  ceiling rather than time/size-based cleanup. At `detection_save_interval_sec:
  5.0`, a cat lounging in frame continuously could evict same-day data within
  ~40min of steady sightings — worth revisiting the number (or moving to a
  smarter per-cat/per-day quota) once real sighting frequency is observed.
  Until then, **pull a copy out to the host before each labeling pass** (§2) —
  the cap deletes oldest-first with no warning.
- **Beep now, bark later.** Phase 5a's beep is a generic "I see *a* cat"
  signal; `plan.md`'s Phase 6 is where a real bark plays through the
  Bluetooth speaker, gated on per-cat identification. Keep the buzzer beep
  even after Phase 6 lands — it's a cheap, immediate, always-available signal
  distinct from the bark, useful for debugging detection latency by ear.
- **Continuous vs event-driven inference.** Running YOLO 24/7 during patrol
  may starve Nav2 on this CPU-bound Orin NX. Options for Phase 6: run the
  detector only in CAPTURE/INVESTIGATING, or throttle hard (`max_input_hz`).
  Decide before Phase 6 integration.
- **Detection → map-frame position (Phase 6 stretch).** The depth-fusion
  nice-to-have (§3) sets up `plan.md` Phase 6's "nav-toward-cat": combine
  detection bearing + depth/lidar range, TF camera→map at the detection
  stamp, send a Nav2 goal in front of the cat. Keep the detection message
  carrying enough (bbox + stamp + frame) to make that possible later.
- **Message contract for Phase 6.** `vision_msgs/Detection2DArray` is enough
  for 5a. When per-cat name + a "confident sighting" flag are needed, decide:
  extend via `class_id`=cat name string, or introduce `cat_patrol_msgs`
  (`CatDetection.msg`) — the plan reserves `cat_patrol_msgs` for exactly this.
- **Where the C++ 5b exercise lands.** ~~If built, `frame_pump_node.cpp` is
  its own `ament_cmake` package~~ — superseded 2026-07-18: rather than the
  narrow producer-consumer frame-pump exercise, a **full C++ port of 5a**
  was built instead (`cat_detector_cpp`, §14). The bounded-queue/drop-stale
  concurrency exercise specifically (§13b) is still open if wanted later —
  `cat_detector_cpp`'s latest-wins slot is a single-threaded overwrite, not a
  `std::mutex`/`std::condition_variable` producer/consumer across threads.

---

## 13. C++ / ROS 2 learning checklist (Phase 5)

### 13a. 5a (Python-side, ROS 2 vision plumbing)
- [ ] `cv_bridge` both directions (`imgmsg_to_cv2` / `cv2_to_imgmsg`).
- [ ] `vision_msgs` layout on Humble (`Detection2DArray` /
      `ObjectHypothesisWithPose`).
- [ ] `SensorDataQoS` for camera streams; small depth to avoid buffering lag.
- [ ] TensorRT export + honest FPS benchmarking on the actual hardware.

### 13b. 5b (the C++ concurrency exercise)
- [ ] Bounded queue with `std::mutex` + `std::condition_variable`.
- [ ] Capacity-1 latest-wins semantics (producer overwrites, consumer drains).
- [ ] Drop-stale under load — prove no unbounded growth (the acceptance bar).
- [ ] Producer (camera callback) / consumer (worker thread) separation, clean
      shutdown (join the worker, no dangling threads).

### 13c. Data discipline (not code, but the phase lives or dies on it)
- [ ] Persistent, dated, backed-up capture dir (never `/tmp`).
- [ ] Reproducible train/val/test split (manifest, not ad-hoc folders).
- [ ] Confusion matrix recorded, not just a single accuracy number.

---

## 14. C++ full port — `cat_detector_cpp` (built 2026-07-18)

Implemented and verified live the same day as the Python `cat_detector`
(§4–§10). User asked for "a similar C++ node"; given the choice between the
plan's narrow producer-consumer frame-pump exercise (§13b) and a full C++
port of the whole 5a pipeline, **full port** was chosen. This is a second,
independent node — it does not replace `cat_detector` (Python); both can run
side by side.

### 14a. What it is
New package `cat_detector_cpp` (`ament_cmake`), `src/detector_node.cpp`:
same feature set as `detector_node.py` — subscribe `/camera/color/image_raw`,
detect cats, publish `vision_msgs/Detection2DArray` + annotated image, beep
on new sighting via the shared `/Buzzer` topic, save timestamped frames with
the same throttling + 500-image retention cap — but inference runs **in C++
via OpenCV's `cv::dnn` module**, not ultralytics/Python.

### 14b. Key technical findings (worth knowing before touching this again)
- **Two OpenCV installs coexist on this Jetson**: the ROS/apt package
  (`libopencv-*4.5d`, 4.5.4, no CUDA) and a custom build at `/usr/local`
  (4.10.0, **CUDA 12.6 + cuDNN 9.6**). Plain `find_package(OpenCV REQUIRED
  COMPONENTS core dnn imgproc imgcodecs)` correctly resolves to the
  `/usr/local` 4.10.0 build (confirmed via a throwaway CMake project before
  writing any node code) — same one `patrol_manager` already links against.
  Verified the built executable actually links `libopencv_dnn.so.410` from
  `/usr/local`, not the apt one.
- **YOLOv8n exported to ONNX** (`ultralytics`, `format="onnx", opset=12,
  simplify=True`) loads fine in `cv::dnn::readNetFromONNX` and runs on
  `DNN_BACKEND_CUDA`/`DNN_TARGET_CUDA`. Benchmarked on this hardware at
  640×640: **~52 FPS on CUDA vs. ~9 FPS on CPU** (`DNN_BACKEND_OPENCV`/
  `DNN_TARGET_CPU`). A one-line `WARN` about cuDNN 9.6 vs. the 9.3 OpenCV
  was built against prints at startup — harmless, inference still runs.
- **Decode**: YOLOv8's ONNX output is `(1, 4+num_classes, num_anchors)` — no
  objectness column (unlike v5). The node reinterprets the forward() output
  buffer directly as a `(dims x num_anchors)` `cv::Mat` via a raw-pointer
  constructor (`cv::Mat(dims, num_anchors, CV_32F, output.ptr<float>())`,
  matching Ultralytics' own official C++ example) rather than relying on
  `Mat::reshape()` semantics on a 3-D Mat, then transposes so each row is one
  candidate `[cx, cy, w, h, class0..classN]`; `cv::dnn::NMSBoxes` does the
  final de-dup. **Known simplification**: input is squashed to `imgsz x
  imgsz` with a plain resize, no letterbox padding — boxes are rescaled back
  correctly, but non-square source frames get a mildly distorted receptive
  field. Add letterboxing later if accuracy on the real Astra feed suffers.
- **Threading**: unlike `patrol_manager` (which needed `MultiThreadedExecutor`
  + callback groups because of concurrent Nav2 goals), this node uses the
  default single-threaded `rclcpp::spin()` — the image-callback "latest
  frame" slot and the inference timer never run concurrently, so no mutex is
  needed for it. Buzzer-off is handled the same way `till_obstacle_back_pattern.cpp`
  already does it in this repo: track an off-time, check it every tick,
  rather than a raw `std::thread`/timer-per-beep.

### 14c. Verified live (not just built)
- `colcon build --symlink-install --packages-select cat_detector_cpp` —
  clean, no warnings even with `-Wall -Wextra -Wpedantic`.
- Ran the real node against a **real cat photo** (from an existing local
  Kaggle cats-vs-dogs dataset on this machine, no camera hardware needed for
  this test) published on `/camera/color/image_raw`: correctly detected
  `class_id=15` ("cat") at **0.82 confidence** with a sane bounding box.
- Confirmed exactly one beep pulse (`True`→`False`) per new sighting across
  4 separated sightings (8 messages total, not one per frame).
- Confirmed retention holds at the configured cap (tested with cap=3) with
  oldest-first eviction, same as the Python version.

### 14d. Files / how to run
| File | Purpose |
|---|---|
| `cat_detector_cpp/src/detector_node.cpp` | The node |
| `cat_detector_cpp/config/cat_detector_cpp_params.yaml` | Params — distinct topics (`/cat_detector_cpp/...`) and a distinct capture dir (`~/cat_patrol_data/detections_cpp/`) from the Python node, so both can run side-by-side for an FPS comparison without colliding |
| `cat_detector_cpp/launch/cat_detector_cpp.launch.py` | Launch |
| `cat_detector_cpp/models/yolov8n.onnx` | Exported ONNX weights |
| `~/myscripts2/t9_detector_cpp.sh` | Convenience launch wrapper (mirrors t1–t8) |

**Caveat if running both detectors at once against the real robot**: they
share the same physical `/Buzzer` topic, so a simultaneous sighting from
both would double-beep. Fine for a side-by-side FPS/accuracy comparison
session; pick one for real unattended patrol use.

Not yet done: live test against the actual Astra camera stream (this
session's test used a published static image, not `t6_camera.sh`); TensorRT
(rather than plain CUDA `cv::dnn`) has not been tried for this path — cv::dnn
CUDA already comfortably clears the ≥10 FPS bar (§3) so it wasn't needed, but
is an option if this ever competes with Nav2 for the GPU.
