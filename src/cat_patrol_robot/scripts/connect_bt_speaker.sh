#!/bin/bash
# Pair and connect RX-V485, route audio, set PulseAudio default sink to bluez.
# Usage: connect_bt_speaker.sh [MAC] [name_substring] [scan_seconds]
set -uo pipefail

MAC="${1:-0C:8E:29:3B:F2:5F}"
NAME_FILTER="${2:-RX-V485}"
SCAN_SEC="${3:-40}"
BT_PATH="/org/bluez/hci0/dev_${MAC//:/_}"
CARD_NAME="bluez_card.${MAC//:/_}"

export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

log() { echo "[connect_bt_speaker] $*" >&2; }

check_a2dp_enabled() {
  local exec_line
  exec_line="$(systemctl show bluetooth -p ExecStart --value 2>/dev/null || true)"
  if echo "$exec_line" | grep -q "noplugin=.*a2dp"; then
    log "ERROR: bluetoothd has A2DP disabled (NVIDIA --noplugin=a2dp)."
    log "       Run once: sudo bash .../enable_bt_a2dp.sh && sudo reboot"
    return 1
  fi
  if ! dpkg -l pulseaudio-module-bluetooth 2>/dev/null | grep -q "^ii"; then
    log "ERROR: sudo apt-get install -y pulseaudio-module-bluetooth"
    return 1
  fi
  return 0
}

ensure_pulseaudio() {
  sudo killall bluealsa 2>/dev/null || true

  if ! pactl info >/dev/null 2>&1; then
    log "PulseAudio not reachable — restarting ..."
    pulseaudio -k 2>/dev/null || true
    sleep 2
    if ! pulseaudio --check 2>/dev/null; then
      rm -f "${XDG_RUNTIME_DIR}/pulse/native" "${XDG_RUNTIME_DIR}/pulse/pid" 2>/dev/null || true
    fi
    pulseaudio --start 2>/dev/null || true
    sleep 2
  fi

  if ! pactl info >/dev/null 2>&1; then
    log "ERROR: PulseAudio still not running."
    log "       Run: pulseaudio --start"
    log "       Or log in to the desktop session as $(whoami) first."
    return 1
  fi

  # Bluetooth modules must be loaded before the device connects.
  if ! pactl list modules short 2>/dev/null | grep -q "module-bluetooth-discover"; then
    log "Loading PulseAudio Bluetooth modules ..."
    pactl load-module module-bluetooth-policy 2>/dev/null || true
    pactl load-module module-bluetooth-discover 2>/dev/null || true
    pactl load-module module-bluez5-discover 2>/dev/null || true
    sleep 1
  fi
  return 0
}

device_visible() {
  bluetoothctl devices 2>/dev/null | grep -qiF "$MAC" && return 0
  bluetoothctl devices 2>/dev/null | grep -qi "$NAME_FILTER" && return 0
  return 1
}

resolve_mac() {
  if bluetoothctl devices 2>/dev/null | grep -qiF "$MAC"; then
    echo "$MAC"
    return 0
  fi
  local found
  found="$(bluetoothctl devices 2>/dev/null | grep -i "$NAME_FILTER" | head -1 | awk '{print $2}')"
  if [[ -n "$found" ]]; then
    log "Resolved $NAME_FILTER -> $found"
    echo "$found"
    return 0
  fi
  return 1
}

is_connected() {
  bluetoothctl info "$MAC" 2>/dev/null | grep -q "Connected: yes"
}

is_paired() {
  bluetoothctl info "$MAC" 2>/dev/null | grep -q "Paired: yes"
}

bt_reconnect_for_pulse() {
  log "Reconnecting BT so PulseAudio picks up the device ..."
  bluetoothctl disconnect "$MAC" >/dev/null 2>&1 || true
  sleep 2
  bluetoothctl connect "$MAC" >/dev/null 2>&1 || true
  sleep 10
}

wait_for_sink() {
  local i sink idx load_out

  if ! pactl list cards short 2>/dev/null | grep -q "bluez_card"; then
    bt_reconnect_for_pulse
  fi

  if ! pactl list cards short 2>/dev/null | grep -q "bluez_card"; then
    load_out="$(pactl load-module module-bluez5-device "path=${BT_PATH}" 2>&1)" || true
    if [[ -n "$load_out" && "$load_out" != *"exists"* ]]; then
      log "load-module: $load_out"
    fi
    sleep 3
  fi

  idx="$(pactl list cards short 2>/dev/null | awk "/bluez_card.${MAC//:/_}/ {print \$1; exit}")"
  if [[ -z "$idx" ]]; then
    log "No bluez_card in PulseAudio."
    log "  Check: pactl info   (must work, not 'Connection refused')"
    log "  Disconnect RX-V485 from your phone, power cycle speaker, retry."
    return 1
  fi

  for i in $(seq 1 20); do
    if pactl set-card-profile "$idx" a2dp_sink 2>/dev/null; then
      sleep 1
      sink="$(pactl list short sinks 2>/dev/null | awk '/bluez_sink/ {print $2; exit}')"
      if [[ -n "$sink" ]]; then
        pactl set-default-sink "$sink" 2>/dev/null || true
        pactl set-sink-mute "$sink" 0 2>/dev/null || true
        pactl set-sink-volume "$sink" 100% 2>/dev/null || true
        log "Default sink: $sink"
        return 0
      fi
    fi
    sleep 1
  done

  log "bluez_card present but a2dp_sink did not activate."
  pactl list cards 2>/dev/null | sed -n "/bluez_card.${MAC//:/_}/,/^$/p" | head -18 >&2 || true
  return 1
}

# End-to-end A2DP probe: play a short file through the same path cat_voice uses
# (ffmpeg mp3→wav | paplay → Pulse default sink). "Connected: yes" alone is not
# enough — this is the deterministic gate that audio actually works.
# Override file with CAT_PATROL_READY_SOUND. Set CAT_PATROL_SKIP_READY_PROBE=1
# to only set up the sink (voice_node will still probe if enabled).
play_ready_probe() {
  local sound default_sink
  sound="${CAT_PATROL_READY_SOUND:-$HOME/cats/voices/ready-chime.mp3}"

  if [[ "${CAT_PATROL_SKIP_READY_PROBE:-0}" == "1" ]]; then
    log "Skipping ready probe (CAT_PATROL_SKIP_READY_PROBE=1)"
    return 0
  fi

  default_sink="$(pactl get-default-sink 2>/dev/null || true)"
  if [[ "$default_sink" != *bluez* ]]; then
    log "ERROR: default sink is not bluez ('$default_sink') — refusing ready probe"
    return 1
  fi

  if [[ ! -f "$sound" ]]; then
    log "ERROR: ready sound missing: $sound"
    log "       Copy ready-chime.mp3 to ~/cats/voices/ or set CAT_PATROL_READY_SOUND"
    return 1
  fi

  log "Ready probe: playing $sound on $default_sink ..."
  if [[ "$sound" == *.wav || "$sound" == *.WAV ]]; then
    if ! paplay "$sound" >/dev/null 2>&1; then
      log "ERROR: paplay failed for $sound"
      return 1
    fi
  else
    # Same pipeline as voice_node: decode mp3 → wav on stdout → paplay.
    if ! ffmpeg -loglevel error -i "$sound" -f wav - 2>/dev/null | paplay >/dev/null 2>&1; then
      log "ERROR: ffmpeg|paplay ready probe failed for $sound"
      return 1
    fi
  fi

  # Re-check sink did not flap back to HDMI/local during playback.
  default_sink="$(pactl get-default-sink 2>/dev/null || true)"
  if [[ "$default_sink" != *bluez* ]]; then
    log "ERROR: default sink left bluez during probe ('$default_sink')"
    return 1
  fi

  log "Ready probe OK (heard path through $default_sink)"
  return 0
}

finish_audio_setup() {
  wait_for_sink || return 1
  play_ready_probe || return 1
  return 0
}

check_a2dp_enabled || exit 1
ensure_pulseaudio || exit 1

# PulseAudio must see the connect event — drop any stale ACL link first.
bluetoothctl disconnect "$MAC" >/dev/null 2>&1 || true
sleep 2

# --- Fast path: already paired, just connect ---
if is_paired && ! is_connected; then
  log "Paired — connecting to $MAC ..."
  bluetoothctl connect "$MAC" >/dev/null 2>&1 || true
  sleep 3
fi

if is_connected; then
  log "Connected to $MAC — setting up A2DP ..."
  finish_audio_setup && exit 0
  exit 1
fi

log "Scanning up to ${SCAN_SEC}s — RX-V485 ON, not connected to phone ..."
{
  echo "power on"
  echo "agent NoInputNoOutput"
  echo "default-agent"
  echo "scan on"
  sleep "$SCAN_SEC"
  echo "scan off"
} | bluetoothctl >/dev/null 2>&1 || true

if device_visible; then
  resolved="$(resolve_mac || true)"
  [[ -n "$resolved" ]] && MAC="$resolved"
  BT_PATH="/org/bluez/hci0/dev_${MAC//:/_}"
  CARD_NAME="bluez_card.${MAC//:/_}"
  log "Found device: $MAC"
else
  log "Not seen in scan — trying configured MAC $MAC"
fi

log "Pairing $MAC ..."
pair_out="$(bluetoothctl pair "$MAC" 2>&1)" || true
if ! echo "$pair_out" | grep -qiE "successful|Already Exists|already paired"; then
  is_paired || log "Pair: $pair_out"
fi
bluetoothctl trust "$MAC" >/dev/null 2>&1 || true

log "Connecting $MAC ..."
for attempt in $(seq 1 10); do
  bluetoothctl connect "$MAC" >/dev/null 2>&1 || true
  is_connected && break
  sleep 3
done

if ! is_connected; then
  log "Connect failed"
  exit 1
fi

log "Connected; setting up PulseAudio A2DP ..."
finish_audio_setup
