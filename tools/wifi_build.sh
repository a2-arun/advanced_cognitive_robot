#!/bin/bash
# Build (NOT install) the Realtek RTL8192EU driver for the D-Link DWA-131 (USB id 2001:3319) on the Jetson Nano.
# Needs internet (phone USB tethering). Run:  bash tools/wifi_build.sh   (a few minutes), then photograph the end.
# Only writes inside ~/rtl8192eu-linux-driver and results/wifi_build.log. No sudo.
# Community driver: https://github.com/windrunner414/rtl8192eu-linux-driver  (not from NVIDIA; untested on JetPack 4.6.1)
cd "$(dirname "$0")/.." && mkdir -p results && LOG="$PWD/results/wifi_build.log"
D="$HOME/rtl8192eu-linux-driver"
if [ ! -d "$D" ]; then git clone --depth 1 https://github.com/windrunner414/rtl8192eu-linux-driver "$D" || { echo "clone failed (internet?)"; exit 1; }; fi
cd "$D" || exit 1
[ -f Makefile.orig ] || cp Makefile Makefile.orig       # pristine copy
KV="$(uname -r)"

build() {   # KSRC/KVER must be passed explicitly: the Makefile only sets them inside platform branches
  make clean >/dev/null 2>&1
  make ARCH=arm64 KVER="$KV" KSRC="/lib/modules/$KV/build" -j2 >"$LOG" 2>&1   # -j2: Nano power limits
  ls "$D"/*.ko >/dev/null 2>&1
}

echo "--- attempt A: stock Makefile (its x86 settings only add little-endian + defaults; the Jetson CPU is little-endian) ---"
cp Makefile.orig Makefile
if build; then OK=A; else
  echo "attempt A failed; first errors:"; grep -n -m 4 -E "error:|fatal error:|undefined reference" "$LOG" | cut -c1-140
  echo "--- attempt B: x86 option off + explicit little-endian flag ---"
  cp Makefile.orig Makefile
  sed -i 's/^CONFIG_PLATFORM_I386_PC *= *y/CONFIG_PLATFORM_I386_PC = n/' Makefile
  sed -i '1i EXTRA_CFLAGS += -DCONFIG_LITTLE_ENDIAN' Makefile
  if build; then OK=B; else OK=""; fi
fi

if [ -n "$OK" ]; then
  echo "=== BUILD OK (attempt $OK): $(ls "$D"/*.ko) ==="
  echo "Tell Claude 'build ok' - the install step comes next."
else
  echo "=== BUILD FAILED (both attempts). FIRST compiler errors: ==="
  grep -n -m 8 -E "error:|fatal error:|undefined reference" "$LOG" | cut -c1-150
  echo "--- full log: $LOG ---"
fi
