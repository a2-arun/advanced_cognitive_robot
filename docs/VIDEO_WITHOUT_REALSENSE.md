# Autonomous obstacle avoidance + ArUco docking video WITHOUT the Jetson / RealSense

Setup: **laptop** + **Arduino on the robot (USB to the laptop)** + **a USB webcam** + printed ArUco markers.
The Python code, the state machines and the dashboard are the same ones used with the RealSense. Only the sensing differs.

| Part | With RealSense (earlier) | In this setup |
|---|---|---|
| Docking marker position | image offset from ArUco | same (real) |
| Docking distance | depth at the marker | **from the marker's apparent size** (needs the focal length, measured below) |
| Obstacle distance | depth ROI of any object | **a box tagged with its own ArUco marker is the obstacle** (proxy). Untagged objects are NOT detected. |
| Robot motion | real pulses | real pulses (same firmware, same safety rules) |

**Say this in the video / report:** "No depth camera was available for this recording. Docking uses the ArUco marker and its measured
size; obstacles are boxes tagged with a second marker. The RealSense depth code path exists in the submitted code but was not used here."
The dashboard also shows it: the chip reads *LIVE WEBCAM + MARKER-OBSTACLE PROXY (no depth sensor)*.

## 0. One-time laptop setup
```
pip install pyserial numpy opencv-contrib-python
```
Close the Arduino IDE Serial Monitor (it holds the port). Plug the Arduino into the laptop. Find its port: Device Manager → Ports (COM & LPT), e.g. **COM5**,
or run `python -m serial.tools.list_ports`. The firmware is already on the Arduino.

## 1. Physical setup
- **Webcam** on the front of the robot, facing forward, level, about 15-25 cm high, fixed firmly (a shaking camera ruins distance).
  The laptop's built-in webcam cannot ride on the robot: use an external USB webcam (then `--webcam-index 1`).
- **Cables** long enough that the robot can move about 1.5 m and not get dragged or wound into the wheels.
- **Clear floor** at least 3 m x 1.5 m. Wheels lifted for the first test.
- **Dock marker:** your existing 6x6 **ID 2** page, flat on a wall/box at camera height.
- **Obstacle marker:** print a second one with the same size and family:
  ```
  python tools/make_aruco_marker.py --dict DICT_6X6_50 --id 3 --size-cm 9.5
  ```
  Print at 100 %, measure the black square, stick it flat in the middle of the front face of a box about 30 cm wide, at camera height.

## 2. Measure the webcam's focal length (one minute, once)
Put marker ID 2 flat, facing the camera squarely, **exactly 50 cm from the lens** (tape measure), keep still:
```
python tools/calibrate_webcam_focal.py --distance-cm 50 --marker-size-cm 9.5 --dict DICT_6X6_50 --id 2 --index 1
```
It prints `Use:  --focal-px 5xx`. Write that number down. (If it says the marker was seen in too few frames: more light, marker flatter, 30-80 cm.)
Use your REAL measured black-square size instead of 9.5 if it differs. If you change the camera position/zoom, redo this.

## 3. Start the dashboard (laptop, PowerShell, in the project folder)
Replace `COM5`, the webcam index and the focal number:
```
python -m warehouse_bot --mode hardware --serial-port COM5 --camera webcam --webcam-index 1 --focal-px 550 --aruco-dict DICT_6X6_50 --marker-id 2 --marker-size-cm 9.5 --obstacle-marker-id 3 --dock-distance-cm 50 --config configs/webcam_demo.json --results results/clip_no_realsense
```
Open the printed URL. You should see the amber chip, a live webcam view, and **ARM** enabled once the serial link is up.
(`--config configs/webcam_demo.json` makes the pulses gentler: 150 ms teleop, 80-200 ms docking.)

## 4. Safety check before any autonomous run
1. Wheels lifted. **ARM**, press **W**, **A**, **D**: all wheels respond. **STOP** works. Disarm.
2. Floor, clear area, hand on the battery switch. Teleop a couple of 150 ms pulses.
3. Hold marker 3 in front of the camera: the obstacle view shows an orange **TAGGED BOX x.xx m** and PATH CLEAR turns into OBSTACLE AHEAD below 0.8 m.
   Check the distance against a tape measure. If it's off by more than ~10 %, redo step 2 (focal) or check the marker size you passed.

## 5. Record: obstacle avoidance (autonomous)
Box with marker 3, about 1.3 m ahead, roughly centered; open floor around it.
1. Start screen recording. **ARM**.
2. Click **Avoid**, tick **Execute actions** (confirm) and **advance when path is clear**.
3. The robot creeps forward in short pulses. When the tagged box is nearer than 0.8 m the state goes OBSTACLE DETECTED → SELECT DIRECTION → TURN, it turns away (left if both sides are clear), re-checks, and goes on.
4. **STOP** when done. Click **Snapshot**.
Limits: it only sees tagged boxes, only inside the camera's view, and never reverses. Leave space; keep a hand ready.

## 6. Record: ArUco docking (autonomous)
Marker 2 about 1.2 m ahead, offset 20-30 cm to one side so you can see the alignment.
1. Start screen recording. **ARM**. Remove the obstacle box.
2. Click **Dock**, tick **Execute actions** (confirm).
3. It aligns (LEFT/RIGHT pulses, shrinking as it converges), approaches in short pulses and stops at 50 cm: state **DOCKED**, progress bar full.
4. If you hide the marker it stops and goes back to SEARCH (it never drives blind). **STOP** ends the run.
Optionally keep recording the docking panel with **rec dock**, then **Snapshot**.

## 7. If something goes wrong
| Problem | Fix |
|---|---|
| `serial: FAILED ... Access denied` | Close the Arduino IDE / Serial Monitor; unplug-replug; check COM number. |
| Dock distance shows `n/a` / "will NOT approach blindly" | No `--focal-px`. Redo step 2 and pass it. |
| Distance wrong by a lot | Marker size or focal wrong. Re-measure the black square; recalibrate at a measured 50 cm. |
| Robot overshoots / oscillates | Lower pulses in `configs/webcam_demo.json`, or move the camera to the very front. |
| Webcam opens the laptop's own camera | Add `--webcam-index 1` (try 2). |
| Marker not detected | More even light, no glare, flat paper, within about 0.3-1.5 m, correct `--aruco-dict`/`--marker-id`. |
| Port 8080 busy | Open the URL it prints. |

## 8. The zip you submit
Everything is in the repository, including the RealSense path (`warehouse_bot/camera.py` RealSenseSource, depth ROI in `perception.py`,
point cloud in `pointcloud.py`) and the Jetson tools. In the report, state which parts each video used (simulation / webcam+marker / RealSense).
