"""Talk node. Playback only. No ROS publish and no microphone."""

import asyncio
import base64
import json
import logging
import queue
import shutil
import subprocess
import threading
import time
from pathlib import Path

from lk_body.envfile import load
from lk_body.listen import HoldMic
from lk_body.playback import PcmPlayer, speaker_sink
from lk_body.sway import Sway

MIMICS = Path("/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/lk_body/config/mimics.json")
log = logging.getLogger("lk_body")


class LkBody:
    def __init__(self):
        self._mic = HoldMic()
        self._player = PcmPlayer()
        self._sway = Sway()
        self._sway.open()
        self._queue: queue.Queue = queue.Queue()
        self._closed = threading.Event()
        self._stop_line = threading.Event()
        self._on_played = None
        self._mimics = json.loads(MIMICS.read_text(encoding="utf-8"))
        self._speaking = self._mimics["speaking"]["action"]
        self._worker = threading.Thread(target=self._play_loop, daemon=True)
        self._worker.start()

    def startup_status(self) -> tuple:
        if shutil.which("pacat") is None or not _pulse():
            return False, "speaker_missing"
        sink = speaker_sink()
        if sink and sink not in _sinks():
            return False, "speaker_missing"
        return True, ""

    def on_down(self) -> str:
        return "mic_not_used"

    def on_up(self) -> bytes:
        return b""

    def enqueue(self, pcm: bytes, actions: list | None = None, language: str = "") -> None:
        del actions
        self._stop_line.set()
        self._player.stop()
        self._queue.put((pcm, language or ""))

    def _play_loop(self) -> None:
        while True:
            item = self._queue.get()
            if item is None:
                return
            pcm, language = item
            self._stop_line.clear()
            self._player.play(pcm, _speech_volume(language))

            def still() -> bool:
                return self._player.playing() and not self._stop_line.is_set()

            self._sway.during(still, self._stop_line.is_set)
            if self._stop_line.is_set():
                self._player.stop()
            elif pcm:
                self._note_played()

    def _note_played(self) -> None:
        notify = self._on_played
        if notify is not None:
            notify()

    def shutdown(self) -> None:
        self._closed.set()
        self._stop_line.set()
        self._player.stop()
        self._sway.stop()
        self._mic.stop()
        self._queue.put(None)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="lk_body %(message)s")
    env = load()
    url = env.get("SAVELIJ_HUB_WS", "")
    token = env.get("SAVELIJ_ATTACH_TOKEN", "")
    if not url or len(token) < 32:
        log.error("dial url or attach token is missing")
        raise SystemExit(1)
    node = LkBody()
    try:
        asyncio.run(_dial(node, url, token))
    finally:
        node.shutdown()


async def _dial(node: LkBody, url: str, token: str) -> None:
    import websockets

    while not node._closed.is_set():
        try:
            async with websockets.connect(url, open_timeout=10) as socket:
                await _session(node, socket, token)
        except Exception as exc:
            log.warning("dial failed: %s", type(exc).__name__)
            await asyncio.sleep(2)


async def _session(node: LkBody, socket, token: str) -> None:
    await socket.send(json.dumps({"type": "attach", "body_id": "savelij", "token": token}))
    attached = json.loads(await socket.recv())
    if attached.get("type") != "attached":
        log.warning("attach refused: %s", attached.get("reason", ""))
        return
    await socket.send(json.dumps(_register(node)))
    registered = json.loads(await socket.recv())
    if registered.get("type") != "registered":
        log.warning("register failed")
        return
    config = json.loads(await socket.recv())
    if config.get("type") != "config":
        log.warning("missing config")
        return
    ready, reason = node.startup_status()
    await socket.send(json.dumps({"type": "status", "ready": ready, "reason": reason}))
    log.info("ready %s", ready)
    notes: asyncio.Queue = asyncio.Queue()
    loop = asyncio.get_running_loop()

    def notify() -> None:
        loop.call_soon_threadsafe(notes.put_nowait, None)

    node._on_played = notify

    async def announce() -> None:
        while True:
            await notes.get()
            await socket.send(json.dumps({"type": "played"}))

    announcer = asyncio.create_task(announce())
    try:
        await _turns(node, socket)
    finally:
        node._on_played = None
        announcer.cancel()


async def _turns(node: LkBody, socket) -> None:
    while True:
        raw = await socket.recv()
        message = json.loads(raw)
        kind = message.get("type")
        if kind == "speak":
            log.info("microphone is not used")
        elif kind == "delivery":
            pcm = base64.b64decode(message.get("pcm") or "")
            node.enqueue(pcm, message.get("actions"), str(message.get("language") or ""))
        elif kind == "config":
            ready, reason = node.startup_status()
            await socket.send(json.dumps({"type": "status", "ready": ready, "reason": reason}))


def _register(node: LkBody) -> dict:
    speaking = node._mimics["speaking"]
    return {
        "type": "register",
        "speaking": {"action": speaking["action"], "intent": speaking["intent"]},
        "stances": node._mimics.get("stances") or [],
    }


def _speech_volume(language: str) -> int:
    """Full stream volume so the USB speaker is audible. The sink volume stays as set."""
    del language
    return 65536


def _pulse() -> bool:
    try:
        result = subprocess.run(["pactl", "info"], capture_output=True, timeout=2)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0 and b"Default Sink:" in result.stdout


def _sinks() -> str:
    try:
        result = subprocess.run(["pactl", "list", "short", "sinks"], capture_output=True, timeout=2)
    except (OSError, subprocess.TimeoutExpired):
        return ""
    return result.stdout.decode("utf-8", "replace")


if __name__ == "__main__":
    main()
