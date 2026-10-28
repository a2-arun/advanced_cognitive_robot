# Recording plan — six independent clips

Each clip uses its own results folder so one failure never affects another. **Easiest way to record:** on your
laptop, open the dashboard in a browser (Jetson served over Wi-Fi, or local simulation) and screen-record the browser
with OBS, Windows Game Bar (Win+Alt+R) or macOS QuickTime. Narrate what is real and what is simulated. The dashboard
also saves annotated MP4s itself (`rec obstacle`, `rec dock`) and snapshots.

Rules for honest evidence: the dashboard always shows SIMULATED / LIVE / RECORDED. Say it out loud. Do not call a
simulation result a hardware result. Do not say the robot avoided or docked unless Clip 2b / 3b was actually done.

| Clip | Folder | Needs |
|---|---|---|
| 1 Teleoperation | `results/clip1_teleop` | robot + Arduino (wheels lifted) |
| 2 Obstacle perception (2a perception, 2b robot action) | `results/clip2_obstacle` | D455, or a recorded session, or simulation |
| 3 Docking (3a perception, 3b optional physical) | `results/clip3_docking` | D455 + printed marker (or simulation) |
| 4 Simulation | `results/clip4_sim` | laptop only |
| 5 Dashboard tour | `results/clip5_dashboard` | any |
| 6 Point cloud (optional) | `results/clip6_pointcloud` | D455 or simulation |

Start every session with `python3 tools/probe_environment.py` once and keep `results/environment_report.txt`.

---
## Clip 1 — Teleoperation (hardware, wheels lifted) — ~60 s
1. **Jetson:** `python3 tools/check_hardware.py --port /dev/ttyACM0` (keep the PASS output; the JSON lands in `results/`).
2. **Jetson:** `python3 -m warehouse_bot --mode hardware --serial-port /dev/ttyACM0 --camera none --host 0.0.0.0 --results results/clip1_teleop`
3. **Laptop:** open the printed URL. Start screen recording.
4. Show the top chips: *ARDUINO /dev/ttyACM0 · disarmed*. Press **ARM** (confirm). Chip turns green *ARMED*.
5. Press W, S, A, D one at a time (pulse 250 ms). After each, point at the **encoders row** (TL TR BR BL) and the
   **Command history**. Say: "full-speed pulses, no PWM".
6. Press **STOP**. Then press **Disarm**: buttons go grey.
7. Optional watchdog proof: ARM, press W and **unplug the USB cable** while the wheels spin → the motors stop and the
   page turns red (*SERIAL LINK DOWN*). Plug back in; `Connect` again.
8. Click **Snapshot**. Stop recording.
No hardware at all? Do **not** substitute `--serial-port mock` as proof of the robot; it is for UI checks only.

## Clip 2 — Obstacle perception
**2a (perception only, no robot motion) — ~60 s**
1. **Jetson:** `python3 -m warehouse_bot --mode hardware --camera realsense --host 0.0.0.0 --results results/clip2_obstacle`
   (no serial connection needed; autonomy defaults to perception only). Chip must read **LIVE CAMERA DATA**.
   Recorded alternative: `--camera session:results/sessions/obstacle_demo` → chip reads **RECORDED FOOTAGE**.
   No camera: run `python -m warehouse_bot` on the laptop → chip reads **SIMULATED DATA**.
2. Click **Avoid** with *Execute actions* **unticked** (pill: *PERCEPTION ONLY*).
3. Empty scene → `PATH CLEAR`. Place a box ~0.6 m ahead → `OBSTACLE AHEAD`, red front box, distance in metres,
   FSM chips walk CLEAR → OBSTACLE DETECTED → SELECT DIRECTION → TURN, and the overlay shows `DECISION: TURN -> LEFT/RIGHT`
   (toward the side with more clearance).
4. Block both sides too → `STOP BLOCKED`. Cover the lens → `DEPTH UNRELIABLE - STOP`.
5. Click **rec obstacle** at the start and again to stop; click **Snapshot**.
If the floor triggers a false obstacle: tilt the camera up or shrink `roi_front` (README troubleshooting).

**2b (robot action, only after the wheels-lifted and floor teleop tests passed)** — record separately
1. Connect, **ARM**, click **Avoid**, tick **Execute actions** (confirm). Keep the robot on the floor, area clear, hand on the switch.
2. Place a box ahead. Expect: obstacle → turn pulse → re-check. Press STOP when done.
Only claim "physically avoided an obstacle" if this was recorded and worked.

## Clip 3 — Docking
**3a (perception, ~60 s)**
1. Print the marker (README §5). Stick it flat on a wall/box at the camera's height. Measure it; set `marker_size_m`.
2. **Jetson:** `python3 -m warehouse_bot --mode hardware --camera realsense --host 0.0.0.0 --results results/clip3_docking`
3. Click **Dock** (perception only). Hold the robot (or move the marker) so it is: off to the left (`LEFT`), off to the right
   (`RIGHT`), centred (`CENTERED`), far (`TOO_FAR`, distance shown with source *depth*), then close (`CLOSE_ENOUGH` →
   state `DOCKED`, progress bar full). Hide the marker → `NO MARKER`, state returns to SEARCH.
4. **rec dock** + **Snapshot**. Say: "perception-based; the robot did not move".

**3b (optional physical docking)** — only after 2b: Connect, ARM, **Dock**, tick **Execute actions**. Pulses are short; the
robot stops between corrections and never moves when the marker is lost. Press STOP at any time. Record the first
successful run, and say how many attempts it took.

## Clip 4 — Simulation — ~90 s (laptop only)
1. `python -m warehouse_bot --results results/clip4_sim` → open `http://127.0.0.1:8080/`. Banner: SIMULATION MODE.
2. Teleop with W/A/S/D; show the **blue true path** and the **orange estimated path** (drifting odometry) on the map.
3. Click the map to add a box; click it again to remove it.
4. **Avoid** + *Execute actions* + *advance when path is clear*: it drives, detects the box from the *simulated depth*, turns, continues.
5. **Reset**, then **Go to dock**: navigates, avoids, faces the dock, ALIGN → APPROACH → DOCKED. Show **Pause / Step / Reset**.
6. Say: "2D kinematic simulation, speeds not calibrated, not SLAM." Snapshot + `rec obstacle`.

## Clip 5 — Dashboard tour — ~60 s
Run in either mode (ideally hardware with the camera + Arduino connected). Walk through: mode switch, data-label chip, link chip,
teleop + pulse slider, STOP, hardware card with encoders, camera / depth / docking panels, obstacle + docking status cards,
mission card (simulation), map, evidence capture. Switch to SIMULATION and show the banner change and that hardware controls disappear.

## Clip 6 (optional) — Point cloud — ~30 s
Camera pointing at a scene with a box: click **Capture cloud**, drag to rotate, **Save .ply** (saved in `results/clip6_pointcloud/pointclouds/`).
Say: "single-frame camera point cloud — not a map, not SLAM." (In simulation the cloud is labelled SIMULATED.)

## After recording — collect evidence
```bash
# Jetson
tar czf results_$(date +%Y%m%d).tgz results/
# laptop
scp <user>@<jetson-ip>:~/advanced_cognitive_bot/results_*.tgz .
```
Keep: environment_report.txt, hardware_check_*.json, logs/*.jsonl, screenshots/, videos/, pointclouds/, the screen recordings.
