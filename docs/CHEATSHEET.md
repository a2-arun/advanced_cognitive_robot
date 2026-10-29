# Cheat sheet: commands, features, what to use

Machines: **[J]** Jetson (in an SSH window from your laptop) · **[L]** laptop (PowerShell, in the project folder) · **[A]** Arduino Serial Monitor (115200 baud, "Newline").

## 1. The dashboard (the main thing)
Start it, then open the URL it prints (the one with `?token=` when you use `--host 0.0.0.0`).

| What you want | Command | Where |
|---|---|---|
| Simulation only | `python -m warehouse_bot` | L |
| Real robot + camera | `python3 -m warehouse_bot --mode hardware --serial-port /dev/ttyACM0 --camera realsense --host 0.0.0.0` | J |
| Real robot, no camera | `... --serial-port /dev/ttyACM0 --camera none --host 0.0.0.0` | J |
| Camera only (perception, no robot) | `... --mode hardware --camera realsense --host 0.0.0.0` | J |
| Replay a recorded camera session | `... --camera session:results/sessions/NAME` | J or L |
| Faster on the Jetson | add `--resolution 424x240` | J |
| Your ArUco marker | add `--aruco-dict DICT_6X6_50 --marker-id 2 --marker-size-cm 9.5` | J |
| Separate output folder per clip | add `--results results/clipN_name` | any |
| Other settings from a file | add `--config my.json` (only the keys you change) | any |
| Choose the port | add `--port 8765` (it also skips ports Windows blocks) | any |

Stop it with **Ctrl+C**. That also stops and disarms the motors.

All options: `--config --host --port --mode {sim,hardware} --serial-port --camera --results --resolution --aruco-dict --marker-id --marker-size-cm`.

### In the dashboard
| Control | What it does |
|---|---|
| **SIMULATION / HARDWARE** | Switches mode. Stops everything and disarms. The simulator can't move the robot and vice versa. |
| **STOP** (button, **Space**, **Esc**) | Stops now and turns autonomy off. Always works. |
| **W A S D** or arrow keys / the arrow buttons | One short full-speed pulse (forward, left, back, right). Pulse length slider, 50-800 ms. |
| **Connect / ARM / Disarm / Disconnect** | Hardware only. Nothing moves until ARM. |
| **Manual / Avoid / Dock / Mission** | Autonomy mode. Mission is simulation-only. |
| **Execute actions** | Off = perception only (shows the decision, robot doesn't move). On = acts. Asks for confirmation on hardware. |
| **advance when path is clear** | In Avoid mode, drives forward when nothing is ahead. |
| **Go to dock / Pick waypoint** | Simulation mission: navigate, avoid, face the dock, dock. |
| **Pause / Step / Reset** | Simulation controls. Click the map to add or remove a box. |
| **Snapshot** | Saves the three annotated views and the state as JSON to `results/screenshots/`. |
| **rec obstacle / rec dock** | Saves an MP4 of that view to `results/videos/`. |
| **Capture cloud / live / Save .ply** | Point cloud. The `.ply` goes to `results/pointclouds/`. |
| **Camera: Open** | Re-opens the camera (auto, RealSense, webcam, none). |

Chips at the top always say where the data comes from: **SIMULATED DATA / LIVE CAMERA DATA / RECORDED FOOTAGE / NO CAMERA**.

## 2. Hardware tools (Jetson)
| Tool | Command | Use it for |
|---|---|---|
| Environment probe | `python3 tools/probe_environment.py` | Versions, OpenCV/ArUco, serial port, RealSense. Report saved to `results/environment_report.txt`. Read-only. |
| Hardware check | `python3 tools/check_hardware.py --port /dev/ttyACM0` | **Wheels lifted.** Four pulses, PASS/FAIL per wheel and encoder. Report saved as JSON. Good review evidence. `--ms 250` sets pulse length. |
| One-screen diagnostics | `bash tools/diag.sh` | Wi-Fi, serial, library summary. |
| Record a camera session | `python3 tools/record_session.py --name NAME --seconds 20` | Lets you replay real depth data later without the camera. |
| Calibrate odometry | `python3 tools/calibrate_odometry.py --port /dev/ttyACM0` | Needs the robot on the floor and a tape measure. Optional. Only matters for dead-reckoned paths. |
| Print an ArUco marker | `python tools/make_aruco_marker.py --size-cm 10 --id 0` | Prints a PNG to `results/markers/`. `--dict DICT_6X6_50` for other families. |

## 3. Wi-Fi setup scripts (Jetson, only needed once; already done for you)
`tools/wifi_prep.sh` (check) → `wifi_build.sh` (build driver) → `wifi_install.sh` (load it, make it permanent) → `wifi_connect.sh` (join hotspot).

## 4. Everyday commands
| Command | Where |
|---|---|
| `ssh a2-nano@<jetson-ip>` | L (PowerShell) |
| `cd ~/advanced_cognitive_robot && git pull` | J (updates the code) |
| `hostname -I` | J (shows its IP) |
| `groups` | J (must include `dialout`) |
| `uptime` | J (shows whether it rebooted) |
| `python -m unittest discover -s tests` | L or J (runs the 69 tests, about 45 s) |
| `tar czf evidence.tgz results/` | J (bundle the evidence) |
| `scp a2-nano@<ip>:~/advanced_cognitive_robot/evidence.tgz .` | L (download it) |

## 5. Arduino Serial Monitor commands [A]
Close the monitor before using the Jetson tools. Type them with no seq or checksum.

| Command | Reply / effect |
|---|---|
| `PING` | `OK 0 PONG wb-0.2` |
| `PINS` | Pin map to compare with your wiring |
| `ARM` / `DISARM` | Arm or disarm. Disarms itself after 10 s of silence. |
| `M F 200` (also `B`, `L`, `R`) | One pulse. 20-1200 ms. Without a heartbeat, over 700 ms triggers the watchdog. |
| `S` or `STOP` | Stop now (works even when disarmed) |
| `ER` | Zero the four encoder counters |
| Telemetry lines | `T <ms> <IDLE/MOVING/BRAKE> <armed> TL TR BR BL` ten times a second |
| Errors | `NOTARMED BUSY GAP RANGE DIR ARGS CHK UNKNOWN LONG` |

## 6. What each feature is, and whether to use it for the review
| Feature | State | Verdict |
|---|---|---|
| Teleoperation (real robot) | Tested on your robot | **Use.** Clip 1. |
| Obstacle perception (live camera) | Tested on your robot | **Use.** Clip 2a. Perception only. |
| ArUco docking perception | Tested with your marker | **Use.** Clip 3a. Perception only. |
| Point cloud and `.ply` | Tested | **Use.** Clip 6. Single frame, **not a map**. |
| 2D simulation incl. avoid + dock mission | Tested in code | **Use.** Clip 4. Label it as simulated. |
| Dashboard tour | Tested | **Use.** Clip 5. |
| Physical obstacle avoidance (`Avoid` + Execute on the floor) | Not tested | **Optional.** Only claim it if recorded. |
| Physical docking (`Dock` + Execute) | Not tested | **Optional.** Same rule. |
| Odometry calibration / dead reckoning | Not tested | **Skip** unless you have spare time. |
| Mission on the real robot | Doesn't exist (no localisation) | **Don't claim.** Simulation only. |
| **SLAM** | **Not implemented** | **Future work.** Say so. |
| Wi-Fi driver scripts | Done | Already used. |

## 7. Files and folders
```
README.md                     setup and quick start
TEST_CHECKLIST.md             tick-off list; only claim what you ticked
docs/CHEATSHEET.md            this file
docs/RECORDING_PLAN.md        the six clips, step by step
docs/WIRING_AND_SAFETY.md     pin map, current limits, emergency procedures
docs/ARCHITECTURE.md          modules, protocol, concept mapping, limitations
firmware/warehouse_bot/       Arduino sketch
warehouse_bot/                the Python package (dashboard, perception, docking, simulator)
tools/                        the scripts above
tests/                        69 automated tests
results/                      everything you record lands here
```
Settings you may want to change are in `warehouse_bot/config.py`. The ones people usually touch: obstacle `stop_distance_m` (0.8 m), `roi_front` (detection zone), docking `dock_distance_m` (0.4 m) and `default_pulse_ms` (250 ms).

## 8. Limits to say out loud
- No speed control: every move is a full-speed pulse. Distance per pulse isn't calibrated.
- The camera is a D435i, not a D455. Say that in the report.
- Obstacle detection is a forward depth test. It is blind to the sides and rear, and to glass and black surfaces.
- The simulation is a kinematic model, not physics, and not calibrated to the robot.
- No SLAM. Don't call a point cloud or a dead-reckoned path a map.
