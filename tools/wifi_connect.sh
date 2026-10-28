#!/bin/bash
# Connect the Jetson to your phone/laptop hotspot. Run:  bash tools/wifi_connect.sh
# Tip: put the hotspot on 2.4 GHz (iPhone: "Maximize Compatibility"; Android: AP band 2.4 GHz); the dongle is 2.4 GHz only.
echo "scanning..."; sudo nmcli device wifi rescan 2>/dev/null; sleep 4
nmcli -f SSID,SIGNAL,SECURITY device wifi list | head -12
read -r -p "Hotspot name (exactly as shown): " SSID
read -r -s -p "Hotspot password: " PASS; echo
sudo nmcli device wifi connect "$SSID" password "$PASS" && {
  echo "=== CONNECTED. Jetson IP address(es): ==="; hostname -I
  echo "Your laptop must be on the SAME hotspot. Then on the laptop:  ssh a2-nano@<the first IP above>"
}
