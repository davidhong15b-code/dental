#!/usr/bin/env bash
# Make a stable /dev/dental_overhead symlink from a chosen *overhead* V4L2 device.
# Example: bash scripts/cameras/setup_overhead_udev.sh /dev/video2
# Never changes permissions to 0666 or pretends a RGB-D depth stream is a webcam.
set -euo pipefail
if [[ $# -ne 1 || ! "$1" =~ ^/dev/video[0-9]+$ || ! -c "$1" ]]; then
  echo 'Usage: bash scripts/cameras/setup_overhead_udev.sh /dev/videoN' >&2
  exit 2
fi
device="$1"
name="${device##*/}"
index="$(cat "/sys/class/video4linux/$name/index")"
if [[ "$index" != 0 ]]; then
  echo "Selected device $device has video index $index, not primary video index 0." >&2
  echo 'Check v4l2-ctl --list-devices; use the primary capture endpoint.' >&2
  exit 2
fi
props="$(udevadm info --query=property --name="$device")"
get_prop() { sed -n "s/^$1=//p" <<< "$props" | head -1; }
vid="$(get_prop ID_VENDOR_ID)"
pid="$(get_prop ID_MODEL_ID)"
serial="$(get_prop ID_SERIAL_SHORT)"
path="$(get_prop ID_PATH)"
if [[ ! "$vid" =~ ^[0-9a-fA-F]{4}$ || ! "$pid" =~ ^[0-9a-fA-F]{4}$ ]]; then
  echo 'Cannot find unique USB vendor/product IDs; no rules installed.' >&2; exit 2
fi
if [[ -n "$serial" && "$serial" =~ ^[a-zA-Z0-9._-]+$ ]]; then
  identity="ENV{ID_SERIAL_SHORT}==\"$serial\""
  mode='camera USB serial; independent of physical USB port'
elif [[ -n "$path" && "$path" =~ ^[a-zA-Z0-9._:-]+$ ]]; then
  identity="ENV{ID_PATH}==\"$path\""
  mode='USB topology; reconnect overhead camera to SAME port'
else
  echo 'No usable serial or physical USB topology. Refusing an ambiguous udev rule.' >&2; exit 2
fi
rule="SUBSYSTEM==\"video4linux\", KERNEL==\"video*\", ATTR{index}==\"0\", ENV{ID_VENDOR_ID}==\"$vid\", ENV{ID_MODEL_ID}==\"$pid\", $identity, SYMLINK+=\"dental_overhead\""
echo "Chosen: $device / $mode"
echo "$rule"
echo
read -r -p 'Install as /etc/udev/rules.d/99-dental-overhead.rules? [y/N] ' answer
if [[ "$answer" != y && "$answer" != Y ]]; then
  echo 'No changes made.'; exit 0
fi
printf '%s\n' "$rule" | sudo tee /etc/udev/rules.d/99-dental-overhead.rules >/dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=video4linux
sleep 1
ls -l /dev/dental_overhead || {
 echo 'Rule installed but link absent. Unplug/reconnect and check identity with udevadm.' >&2
 exit 1
}
if ! id -nG | grep -qw video; then
  echo "Access may require membership of group video: sudo usermod -aG video \"$USER\" (log out/in)."
fi
