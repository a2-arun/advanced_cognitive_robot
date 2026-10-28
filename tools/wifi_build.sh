#!/bin/bash
# Build (NOT install) the Realtek RTL8192EU driver for the D-Link DWA-131 (USB id 2001:3319) on the Jetson Nano.
# Needs internet (phone USB tethering). Run:  bash tools/wifi_build.sh   (takes a few minutes), then photograph the end.
# It only writes inside ~/rtl8192eu-linux-driver and results/wifi_build.log. It does not use sudo.
# Community driver: https://github.com/windrunner414/rtl8192eu-linux-driver  (not from NVIDIA; untested on JetPack 4.6.1)
cd "$(dirname "$0")/.." && mkdir -p results && LOG="$PWD/results/wifi_build.log"
D="$HOME/rtl8192eu-linux-driver"
if [ ! -d "$D" ]; then git clone --depth 1 https://github.com/windrunner414/rtl8192eu-linux-driver "$D" || { echo "clone failed (internet?)"; exit 1; }; fi
cd "$D" || exit 1
echo "--- platform options found in Makefile ---"
grep -n -E "^CONFIG_PLATFORM_[A-Z0-9_]+ *=" Makefile | head -30
cp -n Makefile Makefile.orig
sed -i 's/^CONFIG_PLATFORM_I386_PC *= *y/CONFIG_PLATFORM_I386_PC = n/' Makefile
CHOSEN=none
for P in JETSON ARM_AARCH64 ARM64; do
  if grep -q "^CONFIG_PLATFORM_${P} *=" Makefile; then
    sed -i "s/^CONFIG_PLATFORM_${P} *= *n/CONFIG_PLATFORM_${P} = y/" Makefile; CHOSEN=$P; break
  fi
done
echo "--- ARM64 platform flag enabled: $CHOSEN ---"
make clean >/dev/null 2>&1
echo "--- building (several minutes) ---"
make ARCH=arm64 -j4 >"$LOG" 2>&1
if ls "$D"/*.ko >/dev/null 2>&1; then
  echo "=== BUILD OK: $(ls "$D"/*.ko) ==="
  echo "Tell Claude 'build ok' - the install step comes next."
else
  echo "=== BUILD FAILED. Last lines of $LOG: ==="
  tail -12 "$LOG" | cut -c1-120
fi
