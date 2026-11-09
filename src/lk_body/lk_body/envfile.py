"""Dial URL and attach token.

A variable that is already set, including set to an empty string, is kept.
An empty value does not fall through to the file.
"""

import os
from pathlib import Path

ENV_PATH = Path("/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/lk_body/.env")
KEYS = ("SAVELIJ_HUB_WS", "SAVELIJ_ATTACH_TOKEN")


def load(path: Path = ENV_PATH) -> dict:
    file_values = _parse(path)
    found = {}
    for key in KEYS:
        if key in os.environ:
            found[key] = os.environ[key]
        else:
            found[key] = file_values.get(key, "")
    return found


def _parse(path: Path) -> dict:
    if not path.is_file():
        return {}
    found = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        raw = line.strip()
        if not raw or raw.startswith("#") or "=" not in raw:
            continue
        key, value = raw.split("=", 1)
        found[key.strip()] = value.strip().strip('"').strip("'")
    return found
