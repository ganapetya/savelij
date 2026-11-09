"""Play 22050 Hz mono PCM on the USB speaker. pacat is one process group."""

import json
import os
import signal
import subprocess
import threading
from pathlib import Path

HARDWARE = Path("/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/lk_body/config/hardware.json")


def speaker_sink() -> str:
    """The USB PnP port named Speakers. Empty when the file has no sink."""
    try:
        raw = json.loads(HARDWARE.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return ""
    sink = raw.get("speaker_sink") if isinstance(raw, dict) else ""
    return sink.strip() if isinstance(sink, str) else ""


class PcmPlayer:
    def __init__(self):
        self._proc = None
        self._lock = threading.Lock()

    def playing(self) -> bool:
        with self._lock:
            proc = self._proc
        return proc is not None and proc.poll() is None

    def play(self, pcm: bytes, volume: int = 65536) -> None:
        if not pcm:
            return
        self.stop()
        command = [
            "pacat",
            "--playback",
            "--rate",
            "22050",
            "--format",
            "s16le",
            "--channels",
            "1",
            "--volume",
            str(int(volume)),
            "--latency-msec",
            "50",
            "--client-name",
            "lk_body",
        ]
        sink = speaker_sink()
        if sink:
            command.extend(["--device", sink])
        proc = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        with self._lock:
            self._proc = proc

        def feed() -> None:
            try:
                if proc.stdin is None:
                    return
                view = memoryview(pcm)
                step = 4096
                for start in range(0, len(view), step):
                    proc.stdin.write(view[start : start + step])
                    proc.stdin.flush()
                proc.stdin.close()
            except (BrokenPipeError, ValueError, OSError):
                pass

        threading.Thread(target=feed, daemon=True).start()

    def stop(self) -> None:
        with self._lock:
            proc = self._proc
            self._proc = None
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except OSError:
            proc.terminate()
        try:
            proc.wait(timeout=1)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except OSError:
                proc.kill()
