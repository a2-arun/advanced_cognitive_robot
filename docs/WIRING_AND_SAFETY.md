# Wiring, current limits and safe operation

## Pin map used by the firmware
Taken from your working test sketch (not changed): 

| Function | Arduino pin |
|---|---|
| L298N IN1 / IN2 (channel 1, **right** motors) | D2 / D3 |
| L298N IN3 / IN4 (channel 2, **left** motors) | D4 / D5 |
| Encoder top-left (B, A) | D6, D7 |
| Encoder top-right (B, A) | D8, D9 |
| Encoder bottom-right (B, A) | D10, D11 |
| Encoder bottom-left (B, A) | D12, D13 |
| L298N ENA / ENB | not connected to the Arduino (jumpers fitted, always enabled) |

D0/D1 are the USB serial port, which is why every other pin is already used. No wiring change is needed.

Found on the robot: channel 1 drives the **right** side and channel 2 the **left** (the original sketch comment assumed the
opposite; a pivot test showed `M L` turning right, so the firmware swaps them). The two motors of each side are assumed paralleled on one channel. If instead you use two L298N boards, tell me and update this section.

## The L298N and four motors — check before prolonged use
I do not know your exact motor numbers; **measure, do not assume**.
* An L298N channel is rated for roughly 2 A continuous (about 3 A peak for short, non-repetitive loads — check your
  board's datasheet). It also drops roughly 1.5–2 V, so the motors see about 9–10 V from a 11.1–12.6 V pack.
* Two geared motors in parallel on one channel can exceed that during **start-up and stall**, especially when a
  4WD skid-steer chassis pivots on a high-friction floor. GB37 12 V gear motors commonly list stall currents of
  several amps *each* (see the listing/datasheet for your Rhino GB37 180 RPM).
* Do this once: wheels lifted, run `tools/check_hardware.py` while watching a bench supply / clamp meter / multimeter
  in series with the battery. Note running current and the spike at start. Then repeat with the robot on the floor
  doing a pivot (the worst case) using **100–150 ms** pulses.
* After a few pulses touch the L298N heatsink (briefly, with a fingertip). Hot = stop.
* Signs of overload: battery voltage sags and the Arduino resets, motors stutter, the L298N smells, pulses get weaker.
* If it is marginal: shorter pulses; add a heatsink/fan; fuse the battery lead with a fuse sized from your measurements;
  or move to one driver per side / a stronger driver (BTS7960, etc.).
* Common ground between the L298N and the Arduino is required; the Arduino should be powered from the Jetson USB
  (as now) or a separate 5 V supply — **not** from a motor-noisy rail.

## What the software does for you
| Protection | Where |
|---|---|
| Boots disarmed; all motor pins LOW at start | firmware `setup()` |
| Each move is a firmware-timed pulse, max 1200 ms (host clamps to 800 ms) | firmware + `config.motion` |
| Heartbeat every 0.2 s; no valid line for 0.7 s while moving → brake, coast, disarm, `EVT TIMEOUT` | firmware + `serial_robot.py` |
| Idle armed with silence for 10 s → disarm | firmware |
| Minimum 150 ms between pulses; rejects new move while moving (`BUSY`/`GAP`) | firmware |
| Short brake (IN pins both HIGH, 80 ms) then coast | firmware |
| Host refuses any command unless telemetry is < 1 s old | `RobotLink` |
| Autonomous actions need ARM + healthy link + "Execute actions"; cleared by STOP/link loss/mode change | `engine.py` |
| Simulation can never reach the serial port | `engine.py` (single actuator selector) |
| Closing the dashboard (Ctrl+C) stops and disarms | `server.py` |

These are tested against a software model of the firmware. They are **not proven on your Arduino until
`tools/check_hardware.py` and the watchdog test in TEST_CHECKLIST.md pass.**

## Emergency procedures
1. Dashboard **STOP** (or Space/Esc).
2. If the page is unresponsive: unplug the Jetson↔Arduino USB cable — the firmware stops within 0.7 s.
3. Always works: **switch off or unplug the motor battery** (keep the switch/connector reachable; consider adding an
   inline kill switch or XT60 plug that you can pull with one hand).
4. If it runs away: pick it up by the chassis, then power off. Never put fingers near the wheels while armed.

## Test order for a new robot
1. Battery **disconnected**: upload firmware, `PING`, `PINS`, rotate wheels by hand, confirm each of the four counts changes.
2. Battery connected, **wheels in the air**: `tools/check_hardware.py`. Fix signs. Measure current.
3. Dashboard teleop with wheels in the air; test STOP; unplug USB to watch the watchdog stop the motors.
4. Floor, clear 2 m × 2 m, 150–250 ms pulses.
5. Only then perception-driven actions (obstacle turn, docking approach), one at a time, with a hand on the switch.
