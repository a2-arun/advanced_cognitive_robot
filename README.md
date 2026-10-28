# Autonomous Warehouse Robot — SLAM-Based Navigation and Vision-Guided Docking

Cognitive Robotics project. 4WD skid-steer robot (Arduino Uno + L298N + 4 encoders) driven by a Jetson Nano with an
Intel RealSense D455. This repository holds the working foundation for the first review:

| # | Demonstration | Runs on | Physical hardware needed |
|---|---|---|---|
| A | Teleoperation (forward/back/left/right/stop, encoders, watchdog) | Jetson + Arduino | yes (wheels lifted first) |
| B | Depth-based obstacle detection + left/right/stop decision | Jetson (live/recorded) or any PC (simulation) | camera optional |
| C | ArUco vision-guided docking (detect, align, approach, stop) | same | camera optional |
| D | Interactive web dashboard | Jetson, or laptop | no |
| – | 2D warehouse simulation (same detector + state machines) | laptop or Jetson | no |
| – | Single-frame point cloud (+ `.ply` export) | needs depth frames | camera or simulation |

**SLAM is NOT implemented.** The title says SLAM-based navigation; what exists is perception, reactive avoidance,
visual docking, a simulator, and optional encoder dead reckoning. SLAM is documented as future work. See
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md#limitations-read-before-presenting).

> The older BeetleBot Lyra files in this repo (`lyra_control/`, `older_references/`, `RUN_COMMANDS.md`) belong to a
> different robot (ROS 2 + LiDAR). Nothing in this project uses them.

---

## 0. What is verified and what is not (be honest in the review)

**Verified by automated tests (68 tests, on a Windows laptop, Python 3.11) — software only:**
serial protocol + link logic against a *software model* of the firmware (arming, pulse limits, checksum, watchdog
stop, stale-telemetry refusal), obstacle detector/state machine, ArUco detection + docking state machine, simulator
kinematics/raycast, closed-loop avoidance and docking in the simulator (using the same detector and state machines),
server API, hardware/simulation separation, token protection, point-cloud math, recording/replay, tools.

**NOT yet verified — you must test these on the real robot and only then claim them:**
- `firmware/warehouse_bot/warehouse_bot.ino` has **not been compiled or run** (no Arduino toolchain was available here).
  Compile it in the Arduino IDE first.
- Motor directions, encoder signs, current draw, L298N heating with four motors.
- RealSense streams, `pyrealsense2`, and `cv2.aruco` on your Jetson (`tools/probe_environment.py` tells you).
- Frame rate on the Nano; whether the depth ROI sees the floor from your camera mount.
- Any physical obstacle avoidance or docking. Perception-only runs are the default and are real camera data;
  robot actions are separate, optional, and must be recorded separately.

---

## 1. Safety first (read once)

* The L298N enable pins are jumpered, so **there is no speed control**: every movement is a full-speed pulse.
  The firmware only accepts pulses of 20–1200 ms (the dashboard clamps to 50–800 ms, default 250 ms) and disarms
  itself if the Jetson goes silent for 0.7 s.
* **Wheels lifted for every first test.** Then short floor tests in a clear area.
* **Keep a hand on the battery switch / connector.** To cut motor power: switch off or pull the 3S battery
  connector. The Arduino is powered over USB, so it keeps reporting while the motors are dead.
* Check driver current before long runs: [docs/WIRING_AND_SAFETY.md](docs/WIRING_AND_SAFETY.md).
* The dashboard has a big **STOP** button (also **Space** / **Esc**). It also works if autonomy is active.
* The server only listens on `127.0.0.1` unless you pass `--host 0.0.0.0`, in which case a random token is required.
  Never port-forward it to the internet.

Pin map (taken from your own test sketch): IN1–IN4 = D2–D5 (channel 1 = right motors, channel 2 = left motors, as found on the robot),
encoders (B,A): top-left D6/D7, top-right D8/D9, bottom-right D10/D11, bottom-left D12/D13.

---

## 2. Quick start without any hardware (laptop) — simulation

Laptop (Windows PowerShell or any shell), from the repo root:

```bash
pip install numpy opencv-contrib-python pyserial     # opencv-contrib is needed for cv2.aruco
python -m warehouse_bot
```

Expected: `Dashboard running (SIM mode). Open:  http://127.0.0.1:8080/`. If Windows has reserved 8080 (`WinError 10013`),
the server prints a NOTE and uses 8081, 8082, … instead — always open the URL it prints, or pick one with `--port 8765`. Open that URL. You should see an amber
**SIMULATION MODE** banner, a robot on the map, and three camera panels labelled SIMULATED.

Try: press **W/A/S/D** (teleop) → click **Avoid**, tick **Execute actions**, tick **advance when path is clear** →
click **Go to dock** (mission: navigate, avoid the box, face the dock, dock with the ArUco marker).
Click the map to add/remove obstacles. A full run takes about 75 s (verified headless: avoids the box, 62 pulses, docks, no collision).

Run the whole test suite: `python -m unittest discover -s tests` (about 45 s; expect `OK`).

---

## 3. Jetson setup (Ubuntu 18.04, Python 3.6.9, JetPack 4.6.1)

### 3.1 Get the code onto the Jetson — **Jetson terminal** (or SSH)
```bash
git clone https://github.com/a2-arun/cognitive_obstacle_detection advanced_cognitive_bot   # or copy the folder with scp
cd advanced_cognitive_bot
```
(Everything below happens in this folder. Commit/push from the laptop first if the code only exists there.)

### 3.2 Inspect before installing anything — **Jetson**
```bash
python3 tools/probe_environment.py
```
It only reads. Expected: Python 3.6.x, OpenCV version, whether `cv2.aruco` is PRESENT, serial devices, RealSense
library version and camera streams. Report saved in `results/environment_report.txt`.

### 3.3 Minimal install — **Jetson**
Keep JetPack's own numpy/OpenCV; add only pyserial, in a venv that can see system packages:
```bash
python3 -m venv --system-site-packages .venv        # if "ensurepip is not available": sudo apt install python3-venv
source .venv/bin/activate
pip install pyserial==3.5                           # no venv? use:  pip3 install --user pyserial==3.5
python3 tools/probe_environment.py | grep -E "serial|cv2|aruco|pyrealsense2"
```
Each new terminal: `cd advanced_cognitive_bot && source .venv/bin/activate`.
Serial permission (once): `sudo usermod -aG dialout $USER` then log out/in.

If `cv2.aruco` is MISSING or `pyrealsense2` fails to import, see **Troubleshooting** — do not rebuild OpenCV
or librealsense tonight.

### 3.4 Upload the firmware — **laptop or Jetson with Arduino IDE**
1. Open `firmware/warehouse_bot/warehouse_bot.ino` in the Arduino IDE. Board: **Arduino Uno**. Select the port.
2. **Motor battery disconnected, wheels lifted.** Click Verify (compile) then Upload. Fix any compile error and
   tell me the message.
3. Serial Monitor at **115200 baud**, line ending **Newline**. Type each line and press Enter:
   `PING` → `OK 0 PONG wb-0.2`; `PINS` → pin map to compare with your wiring; telemetry lines `T ... IDLE 0 0 0 0 0` stream at 10 Hz.
   Rotate each wheel by hand: its count in the `T` line must change (this checks each encoder before any motor power).
4. Close the Serial Monitor (it holds the port).

### 3.5 Hardware bring-up — **Jetson**, battery connected, **wheels off the ground**
```bash
python3 tools/check_hardware.py --port /dev/ttyACM0
```
Type `LIFTED` when asked. It sends four 0.25 s pulses and prints per-wheel encoder counts and PASS/FAIL. If a wheel's
sign is wrong, flip `ENC_SIGN[i]` (or `LEFT_/RIGHT_DRIVE_SIGN`) in the sketch and re-upload.

### 3.6 Run the dashboard

**Directly on the Jetson with the HDMI monitor** — Jetson terminal:
```bash
python3 -m warehouse_bot --mode hardware --serial-port /dev/ttyACM0 --camera auto
```
then open `http://127.0.0.1:8080/` in the Jetson's browser.

**From your laptop over Wi-Fi** — Jetson terminal:
```bash
python3 -m warehouse_bot --mode hardware --serial-port /dev/ttyACM0 --camera auto --host 0.0.0.0
```
It prints a URL with `?token=...` (the Jetson's Wi-Fi IP). Open exactly that URL in the laptop browser (same network).

Options: `--mode sim` (default), `--camera auto|realsense|webcam|video:<file>|session:<folder>|none`,
`--serial-port mock` (software test firmware: UI check only, shown as MOCK), `--results <folder>`, `--config my.json`.

---

## 4. Using the dashboard

* **Mode switch (top)** — SIMULATION or HARDWARE. Switching stops everything and disarms the robot. The simulator can
  never send serial commands; hardware mode never moves the simulator. A coloured chip always says
  SIMULATED DATA / LIVE CAMERA DATA / RECORDED FOOTAGE / NO CAMERA.
* **Hardware link** — Connect, then **ARM**. Until armed (and while telemetry is fresh) all motion buttons are disabled.
  If the cable drops, the page turns red and hardware commands are blocked; the Arduino also stops by itself.
* **Teleoperation** — each press = one pulse of the chosen length. No two controllers at once: manual buttons are
  refused while an autonomy mode is active.
* **Autonomy** — *Avoid* (obstacle FSM), *Dock* (ArUco FSM), *Mission* (simulation only).
  **“Execute actions” is OFF by default = perception only**: the decision is shown, the robot does not move. In
  hardware it asks for confirmation and requires an armed, healthy link; it switches itself off on STOP, link loss,
  mode change.
* **Evidence capture** — *Snapshot* saves the three annotated views + state JSON; *rec obstacle / rec dock* save MP4;
  *Save .ply* saves the point cloud. Everything goes under `results/` (see [results/README.md](results/README.md));
  a JSONL log of commands and state changes is written to `results/logs/`.

---

## 5. Printing the ArUco marker — **laptop or Jetson**
```bash
python3 tools/make_aruco_marker.py --size-cm 10        # -> results/markers/aruco_4x4_50_id0_10cm.png
```
Print at 100 % scale, measure the black square with a ruler, set `docking.marker_size_m` if it differs (config below).
Distance comes from depth (preferred) or from PnP using that size; with neither it shows `n/a` and will not approach.

## 6. Recording and testing

* [docs/RECORDING_PLAN.md](docs/RECORDING_PLAN.md) — six independent clips with exact commands.
* [TEST_CHECKLIST.md](TEST_CHECKLIST.md) — tick-off list; a feature counts as done only when its row is ticked.
* Camera borrowed and returned? Record a short RGB-D clip once: `python3 tools/record_session.py --name obstacle_demo --seconds 20`,
  then replay it any time with `--camera session:results/sessions/obstacle_demo` (labelled RECORDED).

## 7. Configuration
`warehouse_bot/config.py` lists every default. Override with a JSON file containing only what you change:
```json
{"obstacle": {"stop_distance_m": 0.6, "roi_front": [0.3, 0.7, 0.3, 0.65]},
 "docking": {"marker_size_m": 0.12, "dock_distance_m": 0.45},
 "motion": {"default_pulse_ms": 200}}
```
`python3 -m warehouse_bot --config my.json ...`. After `tools/calibrate_odometry.py`, add its `odometry` block too.

## 8. Troubleshooting

| Symptom | Fix |
|---|---|
| `no telemetry (wrong port, wrong firmware...)` | Upload the sketch; close the Arduino Serial Monitor; try the other `/dev/ttyACM*`; baud must be 115200. |
| `cannot open /dev/ttyACM0: Permission denied` | `sudo usermod -aG dialout $USER`, log out and in. |
| Motors never move, `ERR NOTARMED` | Press ARM. The Uno resets (disarms) every time the port is opened. |
| `ERR BUSY` / `ERR GAP` | Previous pulse still running; wait ~0.3 s. Intentional. |
| Robot turns the wrong way / count signs wrong | `tools/check_hardware.py`, then flip `*_DRIVE_SIGN` / `ENC_SIGN` in the sketch. |
| L298N or motors hot, battery sags, Arduino resets on start | Stop. See [docs/WIRING_AND_SAFETY.md](docs/WIRING_AND_SAFETY.md) (stall current, separate grounds, shorter pulses). |
| `cv2.aruco` MISSING | Docking view shows "ARUCO UNAVAILABLE". Record a session on the Jetson (`record_session.py` needs no aruco) and run docking on the laptop with `--camera session:...`; or tell me the probe output. Do not rebuild OpenCV tonight. |
| `pyrealsense2` not importable | Check the venv was created with `--system-site-packages`; `find / -name "pyrealsense2*" 2>/dev/null`; add its folder to `PYTHONPATH`. Otherwise use `--camera webcam` (colour only, no depth) or simulation. |
| RealSense found but no frames | Use a USB 3 port and the original USB 3 cable; `--camera realsense` after unplug/replug; lower fps in config. |
| Obstacle always detected | The depth ROI sees the floor. Tilt the camera up or reduce `roi_front` height (`[x0,x1,y0,y1]` fractions). |
| `UNRELIABLE` / `NO DEPTH` | Not enough valid depth in the ROI (too close <0.4 m, glass, black surfaces) → the system stops/holds by design. |
| `WinError 10013` on start | Handled: the server skips unavailable ports automatically. Open the URL it prints. |
| Dashboard unreachable from laptop | Use the exact URL with the token; same Wi-Fi; Jetson firewall; bind with `--host 0.0.0.0`. |
| Browser shows frozen video | Reload the page; video streams reconnect automatically after 1.5 s. |

## 9. Repository layout
```
firmware/warehouse_bot/warehouse_bot.ino   Arduino firmware (pulses, 4 encoders, watchdog)
warehouse_bot/                             Python package (python -m warehouse_bot)
  protocol.py serial_robot.py mock_arduino.py   serial protocol, link, software firmware model
  camera.py perception.py docking.py pointcloud.py odometry.py   sensing + decision logic
  sim.py engine.py server.py web/               simulator, orchestration + safety gate, dashboard
tools/   probe_environment, check_hardware, make_aruco_marker, record_session, calibrate_odometry
tests/   68 unit/integration tests      docs/   architecture, wiring/safety, recording plan
```
