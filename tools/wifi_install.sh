#!/bin/bash
# Load the freshly built RTL8192EU driver, check that a Wi-Fi interface appears, then make it load at every boot.
# Run:  bash tools/wifi_install.sh      (asks for your Jetson password once, for sudo)
KV="$(uname -r)"; D="$HOME/rtl8192eu-linux-driver"
KO="$(ls "$D"/*.ko 2>/dev/null | head -1)"
[ -n "$KO" ] || { echo "no built driver found: run  bash tools/wifi_build.sh  first"; exit 1; }
echo "driver file: $KO"
sudo modprobe cfg80211 2>/dev/null
if lsmod | grep -q "^8192eu"; then echo "driver already loaded"; else
  sudo insmod "$KO" || { echo "=== insmod FAILED. dmesg tail: ==="; dmesg | tail -12 | cut -c1-140; exit 1; }
fi
echo "waiting for the Wi-Fi interface..."; sleep 5
nmcli -t -f DEVICE,TYPE,STATE device | grep -E "wifi|wlan" || { echo "=== NO wifi interface appeared. dmesg tail: ==="; dmesg | tail -12 | cut -c1-140; exit 1; }
echo "--- Wi-Fi interface is up: making the driver permanent ---"
sudo mkdir -p "/lib/modules/$KV/kernel/drivers/net/wireless"
sudo cp "$KO" "/lib/modules/$KV/kernel/drivers/net/wireless/"
sudo depmod -a
echo "8192eu" | sudo tee /etc/modules-load.d/8192eu.conf >/dev/null
echo "=== INSTALL OK. Next:  bash tools/wifi_connect.sh ==="
