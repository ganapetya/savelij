#!/bin/bash
# Optional real bark sound (USB speaker). Set CAT_PATROL_BARK_WAV to a .wav file.
set -euo pipefail
WAV="${CAT_PATROL_BARK_WAV:-}"
if [[ -z "$WAV" || ! -f "$WAV" ]]; then
  echo "cat_patrol: set CAT_PATROL_BARK_WAV to a wave file path" >&2
  exit 1
fi
if command -v paplay >/dev/null; then paplay "$WAV"
elif command -v aplay >/dev/null; then aplay "$WAV"
else
  echo "Install pulseaudio-utils or alsa-utils for paplay/aplay" >&2
  exit 1
fi
