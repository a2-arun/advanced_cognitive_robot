# Architecture, modules and Cognitive Robotics mapping

```
 laptop browser ──HTTP/MJPEG──► Jetson: warehouse_bot.server  ─►  Engine (safety gate, 10 Hz loop)
                                                                   │ camera.py ── RealSense / webcam / video / session / simulated
                                                                   │ perception.py  ObstacleDetector → AvoidanceFSM
                                                                   │ docking.py     ArucoDetector    → DockingFSM
                                                                   │ sim.py         2D kinematic world + ray-cast RGB-D
                                                                   └ serial_robot.py ──USB serial──► Arduino Uno ──► L298N ──► 4 motors
                                                                                                          └── 4 quadrature encoders
```
The same `ObstacleDetector`, `AvoidanceFSM`, `ArucoDetector` and `DockingFSM` objects run on live camera frames, recorded
sessions and simulated frames; only the actuator differs (`RobotLink` vs `Sim`, selected in one place in `engine.py`).

## Modules
| Module | What it does | Test |
|---|---|---|
| `firmware/warehouse_bot/warehouse_bot.ino` | Executes timed pulses on the L298N, decodes 4 quadrature encoders (pin-change ISRs), 10 Hz telemetry, checksum/validation, heartbeat watchdog, disarmed-on-boot | `tools/check_hardware.py`, checklist §1–2 |
| `protocol.py` | Line protocol `seq:CMD args*HH` (XOR checksum), telemetry parsing | `tests/test_serial_link.py` |
| `serial_robot.py` | Threads for reading/heartbeat, acknowledgements, link-health gating, history | same (against `mock_arduino.py`) |
| `mock_arduino.py` | Software model of the firmware for tests/UI checks only (labelled MOCK) | — |
| `camera.py` | Frame sources + honest labels (live/recorded/simulated); depth in metres, 0 = invalid | `test_tools.py` |
| `perception.py` | Depth ROI statistics → median-smoothed, hysteretic CLEAR/OBSTACLE/UNRELIABLE; FSM `CLEAR → OBSTACLE_DETECTED → SELECT_DIRECTION → TURN → RECHECK` (or STOP) | `test_perception_docking.py`, `test_sim.py` |
| `docking.py` | ArUco detection (old/new OpenCV API), distance from depth or PnP, FSM `SEARCH → MARKER_DETECTED → ALIGN → APPROACH → DOCKED` with deadband/hysteresis, self-tuning alignment pulse, action cap | same |
| `pointcloud.py` | Depth + intrinsics → points; `.ply` export | `test_perception_docking.py` |
| `odometry.py` | Encoder dead reckoning; refuses to produce a pose until calibrated | — |
| `sim.py` | 2D warehouse, pulse kinematics, collision, ray-cast depth + warped real ArUco marker, noisy odometry estimate | `test_sim.py` |
| `engine.py` | Mode switch, behaviours, mission (simulation only), STOP, logging, recording | `test_server_safety.py` |
| `server.py`, `web/` | Standard-library HTTP server (no Flask needed on Python 3.6), MJPEG, JSON API, dashboard | same |

## Serial protocol (115200, ASCII, one line per message)
Host → Arduino: `[seq:]CMD [args][*HH]` — `PING`, `ARM`, `DISARM`, `M <F|B|L|R> <ms>`, `S`/`STOP`, `ER`, `PINS`, `HB`.
Arduino → host: `OK <seq> …`, `ERR <seq> <NOTARMED|BUSY|GAP|RANGE|DIR|ARGS|CHK|UNKNOWN|LONG>`,
`EVT <BOOT|DONE|TIMEOUT|DISARM_IDLE>`, and `T <ms> <IDLE|MOVING|BRAKE> <armed> <TL> <TR> <BR> <BL>` at 10 Hz.
Directions: `L`/`R` pivot in place (skid steer). Timeouts: 0.7 s moving / 10 s armed-idle (firmware), 1 s telemetry (host).

## Mapping to Cognitive Robotics concepts
| Concept | Where it appears |
|---|---|
| Perception / sensing (RGB-D, fiducials) | depth ROI analysis; ArUco detection; point cloud |
| Uncertainty and sensor reliability | valid-pixel fraction, median smoothing, hysteresis, `UNRELIABLE`/`NO_DEPTH` → stop (never invented values); depth vs PnP cross-check |
| Reactive, behaviour-based control | `AvoidanceFSM` (stop-and-turn); avoidance has priority over goal seeking in the mission (arbitration) |
| Visual servoing (discrete, image-based) | `DockingFSM`: image offset → align pulse → re-observe; deadband to prevent oscillation |
| Action under actuator limits | no PWM → pulse control, settle-and-re-observe loop, self-tuning pulse length |
| State machines / deliberation | obstacle, docking and mission FSMs shown live on the dashboard |
| State estimation | encoder dead reckoning (`odometry.py`); simulator shows its drift against ground truth |
| Simulation-based validation | `sim.py` runs the production perception/decision code in closed loop |
| Safety and autonomy levels | disarmed boot, watchdog, perception-only default, explicit "Execute actions", single STOP path |
| SLAM / mapping | **not implemented** — future work |

## Limitations (read before presenting)
* **No SLAM.** There is no map built from sensing and no localisation against it. The "map" in simulation is the
  simulator's own known world; in hardware mode the dashboard draws no map unless odometry is calibrated, and then
  only a drifting dead-reckoned path. The point cloud is one camera frame, not a registered map.
* **Simulation is a kinematic model**, speeds are not calibrated to your robot, obstacles are full-height boxes,
  the camera is ray-cast (no floor, no lighting effects). Mission navigation uses the simulator's true pose
  (stated in the UI); the real robot has no localisation yet, so missions are simulation-only.
* **No speed control.** Distance per pulse is unknown until calibrated and varies with floor and battery. Pulse
  distances in the real system must be measured (`tools/calibrate_odometry.py`).
* **Obstacle detection** is a forward depth-ROI test: it cannot see obstacles outside the camera cone, below the
  camera's near limit (~0.4 m), on glass/black/shiny surfaces, and may see the floor if the camera is tilted down.
  There is no rear sensing, so reversing is blind (the avoidance FSM never reverses).
* **Docking** needs the marker in view; distance uses depth or the measured marker size. It does not claim a docking
  accuracy; what "docked" means (distance threshold from the camera to the marker) is configurable and unvalidated.
  While docking, obstacle detection is intentionally not acting (the dock wall is "in the way" by design).
* **Firmware** has not been compiled or run by me; the Python side was tested against a software model of it.
* **Performance** on the Jetson Nano (CPU-only OpenCV, JPEG encoding of three streams) is untested; lower
  `camera.fps` or close unused browser tabs if it lags.
* The dashboard has no user accounts; the token only keeps casual visitors out of a trusted LAN.
