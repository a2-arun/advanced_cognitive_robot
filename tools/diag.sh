#!/bin/bash
# One-screen diagnostics for the Jetson. Run:  bash tools/diag.sh
# Prints a short summary (photograph the screen) and saves it to results/diag.txt (copy it to a USB stick if you like).
# Read-only: changes nothing.
cd "$(dirname "$0")/.." || exit 1
mkdir -p results
{
echo "=== DIAG $(date '+%F %T') ==="
echo "--- WiFi dongle (usb) ---"
lsusb | grep -i -E "d-link|realtek|ralink|mediatek|wireless|wlan|802" || echo "none matched in lsusb"
echo "--- network devices ---"
nmcli -t -f DEVICE,TYPE,STATE device 2>&1 | head -8
echo "--- wireless interfaces ---"
iwconfig 2>&1 | grep -E "IEEE|ESSID" | head -4 || true
echo "--- wifi driver messages ---"
dmesg 2>/dev/null | grep -i -E "rtl|rt2800|rt28|wlan|firmware load|8192|8812" | tail -5
echo "--- ip ---"
hostname -I
echo "--- arduino serial ---"
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "no serial device"
echo "--- python / camera libs ---"
python3 tools/probe_environment.py 2>&1 | grep -a -E "^(python |jetpack|RAM total|disk free)|^(numpy|cv2|serial|pyrealsense2) |cv2.aruco|devices found|  name|  usb type" | cut -c1-110
echo "=== END ==="
} 2>&1 | tee results/diag.txt
