"""Side steps while Savelij speaks. He stays inside a few centimeters of home.

The stroke alternates. One time is 2 cm left, then 4 cm right. The next time
is 2 cm right, then 2 cm left. A step that would leave the fence is turned
around, so the pair does not walk him off the table.

Positive linear.y is left when left_sign is 1. The chassis driver turns that
into wheel speed. This module only publishes cmd_vel.
"""

import json
import os
import sys
import threading
import time
from pathlib import Path

HARDWARE = Path("/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/lk_body/config/hardware.json")


class Sway:
    def __init__(self):
        self._which = 0
        self._spin_thread = None
        self._speed = 0.15
        self._left_sign = 1.0
        self._distance_cm = 8.0
        self._hold_s = 1.25
        self._enabled = True
        self._domain = "28"
        self._ready = False
        self._reported = False
        self._node = None
        self._pub = None
        self._twist = None
        self._vy = 0.0
        self._spin_stop = threading.Event()
        self._lock = threading.Lock()
        self._load()

    def open(self) -> bool:
        """Create the publisher before the first sentence so the driver is already listening."""
        return self._ensure()

    def during(self, still_playing, cancelled) -> None:
        """One out-and-back for this sentence, then more while the audio lasts.

        The first trip finishes even when the sentence is short. A new sentence
        does not cut that trip. Each trip goes out and comes back, so he stays on the patch.
        """
        if not self._enabled or not self._ensure():
            while still_playing():
                time.sleep(0.05)
            return
        try:
            self._trip(self._which, cancelled, force=True)
            self._which += 1
            while still_playing() and not cancelled():
                self._trip(self._which, cancelled, force=False)
                self._which += 1
        finally:
            self.stop()

    def stop(self) -> None:
        self._vy = 0.0
        self._send(0.0)

    def _trip(self, index: int, cancelled, force: bool) -> None:
        # Even sentences: left, then right. Odd sentences: right, then left.
        directions = (-1.0, 1.0) if index % 2 == 0 else (1.0, -1.0)
        for direction in directions:
            if not force and cancelled():
                return
            vy = direction * self._left_sign * self._speed
            side = "left" if direction < 0 else "right"
            hold_s = self._hold_s if self._hold_s > 0 else (self._distance_cm / 100.0) / self._speed
            print(f"sway {side} cm={self._distance_cm:.0f} vy={vy:.2f} s={hold_s:.2f}", file=sys.stderr, flush=True)
            end = time.monotonic() + hold_s
            while time.monotonic() < end:
                if not force and cancelled():
                    print(f"sway {side} cut", file=sys.stderr, flush=True)
                    return
                self._send(vy)
                time.sleep(0.05)
            print(f"sway {side} done", file=sys.stderr, flush=True)

    def _send(self, vy: float) -> None:
        """Send the wheel command from here. A gap lets the chassis stop the move."""
        self._vy = vy
        twist = self._twist
        pub = self._pub
        if twist is None or pub is None:
            return
        twist.linear.x = 0.0
        twist.linear.y = vy
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    def _ensure(self) -> bool:
        with self._lock:
            if self._ready and self._spin_thread is not None and self._spin_thread.is_alive():
                return True
            self._ready = False
            os.environ["ROS_DOMAIN_ID"] = self._domain
            try:
                import rclpy
                from geometry_msgs.msg import Twist
            except ImportError as exc:
                if not self._reported:
                    print(f"sway unavailable: {exc}", file=sys.stderr, flush=True)
                    self._reported = True
                return False
            if not rclpy.ok():
                rclpy.init()
            self._node = rclpy.create_node("lk_body_sway")
            self._pub = self._node.create_publisher(Twist, "cmd_vel", 10)
            self._twist = Twist()
            self._spin_stop.clear()
            self._spin_thread = threading.Thread(target=self._spin, args=(rclpy,), daemon=True)
            self._spin_thread.start()
            self._ready = True
            deadline = time.monotonic() + 2.0
            while self._pub.get_subscription_count() < 1 and time.monotonic() < deadline:
                time.sleep(0.05)
            print(
                f"sway ready listeners={self._pub.get_subscription_count()}",
                file=sys.stderr,
                flush=True,
            )
            return True

    def _spin(self, rclpy) -> None:
        """Keep the publisher discovered. The move itself is sent from the play thread."""
        while not self._spin_stop.is_set():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.05)
            except Exception as exc:
                print(f"sway spin stopped: {exc}", file=sys.stderr, flush=True)
                return

    def _load(self) -> None:
        try:
            raw = json.loads(HARDWARE.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return
        sway = raw.get("sway") if isinstance(raw, dict) else None
        if not isinstance(sway, dict):
            return
        self._enabled = sway.get("enabled", True) is not False
        speed = sway.get("speed_m_s")
        if isinstance(speed, (int, float)) and speed > 0:
            self._speed = float(speed)
        sign = sway.get("left_sign")
        if isinstance(sign, (int, float)) and sign != 0:
            self._left_sign = float(sign)
        distance = sway.get("distance_cm")
        if isinstance(distance, (int, float)) and distance > 0:
            self._distance_cm = float(distance)
        hold = sway.get("hold_s")
        if isinstance(hold, (int, float)) and hold > 0:
            self._hold_s = float(hold)
        domain = sway.get("ros_domain_id")
        if domain is not None:
            self._domain = str(domain)
