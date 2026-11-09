#!/bin/bash
# Re-enable Bluetooth A2DP audio on Yahboom Jetson images.
#
# NVIDIA's bluetooth.service drop-in disables audio/a2dp/avrcp plugins:
#   --noplugin=audio,a2dp,avrcp
# Without A2DP, PulseAudio never creates a bluez sink and bark.wav is silent.
#
# Run once with sudo:
#   sudo bash enable_bt_a2dp.sh
set -euo pipefail

if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
  echo "Run as root: sudo bash $0" >&2
  exit 1
fi

DROPIN_DIR="/etc/systemd/system/bluetooth.service.d"
# zz- prefix ensures this loads AFTER nv-bluetooth-service.conf (which disables A2DP).
DROPIN_FILE="${DROPIN_DIR}/zz-cat-patrol-a2dp.conf"

mkdir -p "$DROPIN_DIR"
cat > "$DROPIN_FILE" <<'EOF'
# cat_patrol_robot: restore A2DP so BT speakers (RX-V485) work with PulseAudio.
# Overrides NVIDIA nv-bluetooth-service.conf which uses --noplugin=audio,a2dp,avrcp
[Service]
ExecStart=
ExecStart=/usr/lib/bluetooth/bluetoothd
EOF

echo "Wrote $DROPIN_FILE"

if ! dpkg -l pulseaudio-module-bluetooth 2>/dev/null | grep -q "^ii"; then
  echo "Installing pulseaudio-module-bluetooth ..."
  apt-get update -qq
  apt-get install -y pulseaudio-module-bluetooth
fi

systemctl daemon-reload
systemctl restart bluetooth
sleep 2

systemctl disable --now bluealsa 2>/dev/null || true
killall bluealsa 2>/dev/null || true

# Restart PulseAudio for the desktop user (usually jetson uid 1000)
if id jetson &>/dev/null; then
  sudo -u jetson pulseaudio -k 2>/dev/null || true
  sleep 1
  sudo -u jetson pulseaudio --start 2>/dev/null || true
fi

echo ""
echo "Done. REBOOT once, then test:"
echo "  sudo reboot"
echo "  bash $(dirname "$0")/connect_bt_speaker.sh"
echo "  paplay /home/jetson/bark.wav"
echo ""
systemctl show bluetooth -p ExecStart --value
