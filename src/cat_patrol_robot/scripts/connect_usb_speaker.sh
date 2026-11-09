#!/bin/bash
# Route PulseAudio to the USB voice card (JMTek / Solid State System "USB PnP
# Audio Device", USB ID 0c76:1203) and make it the default sink for paplay.
#
# Replaces connect_bt_speaker.sh for the wired USB + amp path.
#
# Usage: connect_usb_speaker.sh [sink_match_substring]
# Env:
#   CAT_PATROL_USB_SINK_MATCH  substring to find the sink (default below)
#   CAT_PATROL_USB_VOLUME      Pulse volume, e.g. 80% (default 80%)
#   CAT_PATROL_READY_SOUND     ready probe file (default ~/cats/voices/ready-chime.mp3)
#   CAT_PATROL_SKIP_READY_PROBE=1  only set default sink (voice_node probes)
set -uo pipefail

SINK_MATCH="${1:-${CAT_PATROL_USB_SINK_MATCH:-USB_PnP_Audio_Device}}"
VOLUME="${CAT_PATROL_USB_VOLUME:-80%}"

export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

log() { echo "[connect_usb_speaker] $*" >&2; }

ensure_pulseaudio() {
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
  return 0
}

find_usb_sink() {
  # Prefer an exact substring match on the Pulse sink name.
  local sink
  sink="$(pactl list short sinks 2>/dev/null | awk -v m="$SINK_MATCH" 'tolower($2) ~ tolower(m) {print $2; exit}')"
  if [[ -n "$sink" ]]; then
    echo "$sink"
    return 0
  fi
  # Fallbacks for common names on this card family.
  for m in 'USB_PnP_Audio_Device' 'Solid_State_System' 'usb-0c76'; do
    sink="$(pactl list short sinks 2>/dev/null | awk -v m="$m" 'tolower($2) ~ tolower(m) {print $2; exit}')"
    if [[ -n "$sink" ]]; then
      echo "$sink"
      return 0
    fi
  done
  return 1
}

wait_for_sink() {
  local i sink
  for i in $(seq 1 30); do
    sink="$(find_usb_sink || true)"
    if [[ -n "$sink" ]]; then
      if pactl set-default-sink "$sink" 2>/dev/null; then
        pactl set-sink-mute "$sink" 0 2>/dev/null || true
        pactl set-sink-volume "$sink" "$VOLUME" 2>/dev/null || true
        # Best-effort ALSA mixer on card "Device" (USB PnP).
        amixer -c Device set Speaker "$VOLUME" unmute >/dev/null 2>&1 || true
        amixer -c Device set PCM "$VOLUME" unmute >/dev/null 2>&1 || true
        log "Default sink: $sink (volume $VOLUME, unmuted)"
        return 0
      fi
    fi
    if [[ $i -eq 1 || $((i % 5)) -eq 0 ]]; then
      log "Waiting for USB audio sink matching '$SINK_MATCH' (${i}/30) ..."
      if [[ $i -eq 1 ]]; then
        log "  Available sinks:"
        pactl list short sinks 2>/dev/null | sed 's/^/    /' >&2 || true
        log "  USB devices (expect 0c76:1203 JMTek USB PnP Audio):"
        lsusb 2>/dev/null | grep -iE '0c76|audio|jmtek|pnp' | sed 's/^/    /' >&2 || true
      fi
    fi
    sleep 1
  done

  log "ERROR: USB audio sink matching '$SINK_MATCH' not found."
  log "       Check: lsusb | grep -i 0c76   and   pactl list short sinks"
  log "       Cable/power the voice card, then re-run."
  return 1
}

# End-to-end probe: same path cat_voice uses (ffmpeg mp3→wav | paplay).
play_ready_probe() {
  local sound default_sink
  sound="${CAT_PATROL_READY_SOUND:-$HOME/cats/voices/ready-chime.mp3}"

  if [[ "${CAT_PATROL_SKIP_READY_PROBE:-0}" == "1" ]]; then
    log "Skipping ready probe (CAT_PATROL_SKIP_READY_PROBE=1)"
    return 0
  fi

  default_sink="$(pactl get-default-sink 2>/dev/null || true)"
  if [[ "${default_sink,,}" != *"${SINK_MATCH,,}"* ]]; then
    # Accept any of our known USB markers if custom match was used at set time.
    case "${default_sink,,}" in
      *usb_pnp_audio_device*|*solid_state_system*) ;;
      *)
        log "ERROR: default sink is not USB voice card ('$default_sink') — refusing ready probe"
        return 1
        ;;
    esac
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
    if ! ffmpeg -loglevel error -i "$sound" -f wav - 2>/dev/null | paplay >/dev/null 2>&1; then
      log "ERROR: ffmpeg|paplay ready probe failed for $sound"
      return 1
    fi
  fi

  default_sink="$(pactl get-default-sink 2>/dev/null || true)"
  case "${default_sink,,}" in
    *usb_pnp_audio_device*|*solid_state_system*|*"${SINK_MATCH,,}"*) ;;
    *)
      log "ERROR: default sink left USB voice card during probe ('$default_sink')"
      return 1
      ;;
  esac

  log "Ready probe OK (heard path through $default_sink)"
  return 0
}

ensure_pulseaudio || exit 1
wait_for_sink || exit 1
play_ready_probe || exit 1
log "USB voice card ready."
exit 0
