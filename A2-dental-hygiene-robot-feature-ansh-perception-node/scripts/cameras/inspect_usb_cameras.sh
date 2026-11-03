#!/usr/bin/env bash
# Inspect direct-USB camera endpoints for C++ camera acquisition.
set -euo pipefail
printf '\nAvailable V4L2 devices:\n'
if command -v v4l2-ctl >/dev/null 2>&1; then v4l2-ctl --list-devices || true; fi
printf '\nStable V4L2 names:\n'
ls -l /dev/v4l/by-id/* /dev/v4l/by-path/* 2>/dev/null || true
printf '\nVideo endpoint identities:\n'
shopt -s nullglob
for d in /dev/video*; do
  printf '\n%s (%s)\n' "$d" "$(cat "/sys/class/video4linux/${d##*/}/name" 2>/dev/null || echo unknown)"
  udevadm info --query=property --name="$d" 2>/dev/null |
    grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_PATH|ID_V4L_CAPABILITIES|ID_V4L_PRODUCT)=' || true
  printf 'video index: '; cat "/sys/class/video4linux/${d##*/}/index" 2>/dev/null || echo unknown
done
if command -v rs-enumerate-devices >/dev/null 2>&1; then
  printf '\nRealSense SDK devices:\n'
  rs-enumerate-devices 2>&1 | head -100 || true
else
  printf '\nInstall RealSense SDK utilities and run rs-enumerate-devices if the depth camera is RealSense.\n'
fi
