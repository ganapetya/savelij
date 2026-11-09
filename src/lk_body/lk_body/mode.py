"""Enter and leave his talk mode. The car's bringup stays up."""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

BODY_MARK = "lk_" + "body.node"
PID_PATH = Path("/tmp/lk_body_savelij.pid")
DRIVER_MARK = "Mcnamu_driver_X3"
DRIVER_PID = Path("/tmp/lk_mcnamu.pid")
ROOT = "/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/lk_body"
WS = "/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws"


def main(argv=None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if args == ["enter"]:
        return enter()
    if args == ["leave"]:
        return leave()
    print("usage: lk_bot_mode enter|leave", file=sys.stderr)
    return 2


def enter() -> int:
    start_driver()
    if body_running():
        return 0
    env = os.environ.copy()
    previous = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = ROOT + (":" + previous if previous else "")
    env["ROS_DOMAIN_ID"] = "28"
    log_file = Path("/tmp/lk_body_savelij.log").open("a", encoding="utf-8")
    proc = subprocess.Popen(
        [
            "bash",
            "-lc",
            "set +u; source /opt/ros/humble/setup.bash; "
            f"source {WS}/install/setup.bash; set -u; "
            "export ROS_DOMAIN_ID=28; "
            f"export PYTHONPATH={ROOT}:${{PYTHONPATH}}; "
            "exec python3 -m lk_body.node",
        ],
        cwd=ROOT,
        env=env,
        stdin=subprocess.DEVNULL,
        stdout=log_file,
        stderr=log_file,
        start_new_session=True,
    )
    log_file.close()
    PID_PATH.write_text(str(proc.pid), encoding="utf-8")
    time.sleep(0.8)
    if proc.poll() is not None:
        print("lk_body did not stay up", file=sys.stderr)
        return 1
    return 0


def leave() -> int:
    stop_body()
    stop_driver()
    return 0


def start_driver() -> None:
    """The side step needs the mecanum driver. A driver already up is left alone."""
    for _ in range(20):
        if not any(DRIVER_MARK in cmd for _pid, cmd in _commands()):
            break
        time.sleep(0.2)
    else:
        return
    log_file = Path("/tmp/lk_mcnamu.log").open("a", encoding="utf-8")
    log_file.write("starting chassis driver\n")
    log_file.flush()
    proc = subprocess.Popen(
        [
            "bash",
            "-lc",
            "set +u; source /opt/ros/humble/setup.bash; "
            f"source {WS}/install/setup.bash; set -u; "
            "export ROS_DOMAIN_ID=28; "
            "exec ros2 run yahboomcar_bringup Mcnamu_driver_X3 --ros-args -p cmd_timeout_sec:=2.0",
        ],
        stdin=subprocess.DEVNULL,
        stdout=log_file,
        stderr=log_file,
        start_new_session=True,
    )
    log_file.close()
    DRIVER_PID.write_text(str(proc.pid), encoding="utf-8")
    time.sleep(1.5)
    if proc.poll() is not None:
        print("chassis driver did not stay up", file=sys.stderr)


def stop_driver() -> None:
    if not DRIVER_PID.is_file():
        return
    try:
        pid = int(DRIVER_PID.read_text(encoding="utf-8").strip())
        os.killpg(pid, signal.SIGTERM)
    except (OSError, ValueError):
        pass
    DRIVER_PID.unlink(missing_ok=True)


def body_running() -> bool:
    return any(BODY_MARK in cmd for _pid, cmd in _commands())


def stop_body() -> None:
    for pid, cmd in _commands():
        if BODY_MARK not in cmd:
            continue
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    if PID_PATH.is_file():
        try:
            pid = int(PID_PATH.read_text(encoding="utf-8").strip())
            os.killpg(pid, signal.SIGTERM)
        except (OSError, ValueError):
            pass
        PID_PATH.unlink(missing_ok=True)


def _commands():
    proc = Path("/proc")
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / "cmdline").read_bytes().replace(b"\x00", b" ").decode("utf-8", "replace")
        except OSError:
            continue
        yield int(entry.name), raw


if __name__ == "__main__":
    raise SystemExit(main())
