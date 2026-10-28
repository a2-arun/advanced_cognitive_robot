# results/

Everything the tools and the dashboard save lands here (use `--results results/clipN_name` to keep clips apart).

| Path | Written by |
|---|---|
| `environment_report.txt` | `tools/probe_environment.py` |
| `hardware_check_*.json` | `tools/check_hardware.py` |
| `odometry_calibration.json` | `tools/calibrate_odometry.py` |
| `markers/*.png` | `tools/make_aruco_marker.py` |
| `sessions/<name>/` | `tools/record_session.py` (RGB-D frames for offline replay) |
| `logs/session_*.jsonl` | dashboard: every command + state change with timestamps |
| `screenshots/*` | dashboard Snapshot (three annotated views + state JSON) |
| `videos/*.mp4` | dashboard rec buttons |
| `pointclouds/*.ply` | dashboard Save .ply |
