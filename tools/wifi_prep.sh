#!/bin/bash
# Read-only readiness check before building a Wi-Fi driver for the D-Link (2001:3319, RTL8192EU) dongle.
# Run:  bash tools/wifi_prep.sh      then photograph the screen.
echo "=== WIFI PREP $(date '+%T') ==="
echo "--- internet (needs phone USB tethering on) ---"
if ping -c 2 -W 3 8.8.8.8 >/dev/null 2>&1; then echo "internet: YES"; else echo "internet: NO  (turn on USB tethering on the phone)"; fi
nmcli -t -f DEVICE,TYPE,STATE device 2>&1 | grep -v -E "docker|l4tbr|dummy|lo:" | head -6
echo "--- kernel and headers (needed to build the driver) ---"
uname -r
ls -d /usr/src/linux-headers* 2>/dev/null | head -4 || true
[ -e /lib/modules/$(uname -r)/build ] && echo "build link: present" || echo "build link: MISSING"
echo "--- build tools ---"
for t in gcc make git dkms; do printf "%-6s" $t; command -v $t >/dev/null && echo yes || echo NO; done
echo "--- driver already loaded? ---"
lsmod | grep -i -E "8192|rtl" || echo "no realtek wifi module loaded"
echo "--- disk free ---"
df -h / | tail -1 | awk '{print $4 " free"}'
echo "=== END ==="
