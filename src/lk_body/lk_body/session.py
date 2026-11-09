"""Socket loop shared by every body. Hardware stays in the package on the robot.

on_delivery runs in the receive loop, before any other await, so playback
starts on the delivery message.
"""

import asyncio
import base64
import json
import logging
from collections.abc import Awaitable, Callable

log = logging.getLogger("lk_body")


async def run(
    url: str,
    token: str,
    body_id: str,
    register: dict,
    startup_status: Callable[[], tuple],
    on_delivery: Callable[[bytes, list, str], None],
    on_speak: Callable[[bool, object], Awaitable[None]],
    set_played: Callable[[Callable[[], None] | None], None],
    running: Callable[[], bool],
    on_registered: Callable[[], None] | None = None,
) -> None:
    import websockets

    while running():
        try:
            async with websockets.connect(url, open_timeout=10) as socket:
                await _session(
                    socket,
                    token,
                    body_id,
                    register,
                    startup_status,
                    on_delivery,
                    on_speak,
                    set_played,
                    on_registered,
                )
        except Exception as exc:
            log.warning("dial failed: %s", type(exc).__name__)
            await asyncio.sleep(2)


async def _session(
    socket,
    token: str,
    body_id: str,
    register: dict,
    startup_status: Callable[[], tuple],
    on_delivery: Callable[[bytes, list, str], None],
    on_speak: Callable[[bool, object], Awaitable[None]],
    set_played: Callable[[Callable[[], None] | None], None],
    on_registered: Callable[[], None] | None,
) -> None:
    await socket.send(json.dumps({"type": "attach", "body_id": body_id, "token": token}))
    attached = json.loads(await socket.recv())
    if attached.get("type") != "attached":
        log.warning("attach refused: %s", attached.get("reason", ""))
        return
    await socket.send(json.dumps(register))
    registered = json.loads(await socket.recv())
    if registered.get("type") != "registered":
        log.warning("register failed")
        return
    if on_registered is not None:
        on_registered()
    config = json.loads(await socket.recv())
    if config.get("type") != "config":
        log.warning("missing config")
        return
    ready, reason = startup_status()
    await socket.send(json.dumps({"type": "status", "ready": ready, "reason": reason}))
    log.info("ready %s", ready)
    notes: asyncio.Queue = asyncio.Queue()
    loop = asyncio.get_running_loop()

    def notify() -> None:
        loop.call_soon_threadsafe(notes.put_nowait, None)

    set_played(notify)

    async def announce() -> None:
        while True:
            await notes.get()
            await socket.send(json.dumps({"type": "played"}))

    announcer = asyncio.create_task(announce())
    try:
        await _turns(socket, startup_status, on_delivery, on_speak)
    finally:
        set_played(None)
        announcer.cancel()


async def _turns(socket, startup_status, on_delivery, on_speak) -> None:
    while True:
        raw = await socket.recv()
        message = json.loads(raw)
        kind = message.get("type")
        if kind == "speak":
            await on_speak(bool(message.get("down")), socket)
        elif kind == "delivery":
            pcm = base64.b64decode(message.get("pcm") or "")
            actions = message.get("actions") if isinstance(message.get("actions"), list) else []
            on_delivery(pcm, actions, str(message.get("language") or ""))
        elif kind == "config":
            ready, reason = startup_status()
            await socket.send(json.dumps({"type": "status", "ready": ready, "reason": reason}))
