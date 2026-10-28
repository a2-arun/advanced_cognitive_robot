# Test checklist

Tick a row only when you saw the expected result yourself. The "Software" column was already verified by automated
tests (`python -m unittest discover -s tests` → `OK`); the "On robot" column is **yours to confirm** — until then do
not claim it.

Legend: **L** laptop · **J** Jetson · **A** Arduino IDE · **R** robot.

## 0. Environment
- [ ] **J** `python3 tools/probe_environment.py` runs; Python 3.6.x; note OpenCV version and `cv2.aruco` PRESENT/MISSING → `results/environment_report.txt`
- [ ] **J** `pyrealsense2` imports; D455 listed with colour + depth streams (needs USB 3)
- [ ] **J** Arduino appears as `/dev/ttyACM*` and is readable (dialout group)
- [ ] **L** `python -m unittest discover -s tests` → `OK` (68 tests)

## 1. Firmware and link (A, J, R)
- [ ] **A** `warehouse_bot.ino` compiles for Arduino Uno with no errors (not yet compiled by me)
- [ ] **A** Uploaded. Serial Monitor 115200: `PING` → `OK 0 PONG wb-0.2`; `PINS` matches the wiring table
- [ ] **A** Battery disconnected: rotating each wheel by hand changes its own count (TL, TR, BR, BL) in the `T` line
- [ ] **A** `M F 200` before `ARM` → `ERR 0 NOTARMED`; `ARM` → `OK 0 ARMED`
- [ ] **J** `python3 tools/check_hardware.py --port /dev/ttyACM0`, wheels lifted → **PASS** (all 4 wheels, 4 directions)
- [ ] **R** all four wheels turn forward on F, and pivot left on L (visual)
- [ ] **R** Current/heat measured during pulses; L298N not hot after 10 pulses (docs/WIRING_AND_SAFETY.md)

## 2. Safety behaviour (R, wheels lifted)
- [ ] `M F 1200` → motors stop by themselves after ~1.2 s
- [ ] `M F 5000` → `ERR 0 RANGE`; no movement
- [ ] Dashboard STOP during a long pulse stops it within ~0.2 s
- [ ] Unplug USB while a pulse runs → motors stop (≤ 0.7 s) and dashboard shows SERIAL LINK DOWN
- [ ] After replug: robot is **disarmed** and ignores motion until ARM
- [ ] Hardware buttons disabled while disarmed / link down
- [ ] Teleop is refused while Avoid/Dock is active

## 3. Demonstration A — Teleoperation
- [ ] Forward/back/left/right pulses from the dashboard; encoder counts change; history lines appear
- [ ] Floor test (clear area, 150–250 ms pulses): robot moves briefly and stops each time

## 4. Demonstration B — Obstacle perception
- [ ] Source label reads LIVE CAMERA DATA (or RECORDED / SIMULATED, as appropriate)
- [ ] Empty scene → PATH CLEAR; box at ~0.6 m → OBSTACLE AHEAD with distance
- [ ] Decision picks the freer side; both sides blocked → STOP BLOCKED; lens covered → DEPTH UNRELIABLE (no invented distance)
- [ ] RGB-only source (`--camera webcam`) shows NO DEPTH and no distance
- [ ] (optional, separate recording) Execute actions: robot turns, re-checks, stops when blocked

## 5. Demonstration C — Docking
- [ ] Marker printed at 100 %, black square measured, `marker_size_m` set
- [ ] Boundary + centre drawn; NO MARKER ↔ MARKER DETECTED toggles when hidden/shown
- [ ] LEFT / RIGHT / CENTERED follow the marker's side; distance shown with source (depth)
- [ ] TOO_FAR → CLOSE_ENOUGH → DOCKED (perception only, robot not moving)
- [ ] Marker lost → state returns to SEARCH, no motion
- [ ] (optional) physical docking with Execute actions; record number of pulses and result honestly

## 6. Demonstration D / simulation
- [ ] **L** `python -m warehouse_bot` → amber SIMULATION banner; all panels labelled SIMULATED
- [ ] W/A/S/D move the simulated robot; trail and estimated path draw; click map adds/removes boxes
- [ ] Avoid + Execute actions + advance: robot turns away from the box (no collision)
- [ ] Go to dock: mission reaches DONE with state DOCKED
- [ ] Pause / Step / Reset work
- [ ] Switching to HARDWARE mode stops the sim and hides simulation controls; teleop in SIM mode never produces serial traffic
- [ ] **J→L** `--host 0.0.0.0`: laptop opens the token URL; URL without token is rejected (401)

## 7. Optional
- [ ] Capture cloud + rotate + Save .ply (opens in MeshLab/CloudCompare); label says single-frame, not SLAM
- [ ] `tools/record_session.py` then replay with `--camera session:...` labelled RECORDED
- [ ] `tools/calibrate_odometry.py` run; config snippet saved (dead reckoning only)
