#!/usr/bin/env python3
"""Measure counts-per-metre and the effective track width, so encoder dead reckoning becomes meaningful.

    python3 tools/calibrate_odometry.py --port /dev/ttyACM0

Robot ON THE FLOOR in a clear area with tape measure + masking tape. Short pulses only (default 4 x 250 ms).
Results are written to results/odometry_calibration.json and printed as a config snippet:
    {"odometry": {"counts_per_m": ..., "effective_track_m": ...}}   -> save as my_config.json and run with --config.

Skid-steer turning slips, so the effective track is larger than the physical one and varies with the floor.
This is a rough calibration for dead reckoning, NOT a substitute for SLAM.
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from warehouse_bot import protocol
from warehouse_bot.config import load_config
from warehouse_bot.serial_robot import RobotLink, RobotLinkError


def pulses(link, d, n, ms):
    for _ in range(n):
        for _try in range(20):
            r = link.move(d, ms)
            if r["ok"]:
                break
            time.sleep(0.1)
        else:
            raise RuntimeError("move refused: %s" % r["error"])
        time.sleep(ms / 1000.0 + 0.7)


def main(argv=None, ask=input):
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="auto")
    ap.add_argument("--pulses", type=int, default=4)
    ap.add_argument("--ms", type=int, default=250)
    a = ap.parse_args(argv)
    link = RobotLink(load_config())
    try:
        link.connect(a.port)
    except RobotLinkError as e:
        print("FAIL:", e)
        return 1
    result = {}
    try:
        if ask("Robot on the floor, area clear, hand near the power switch? Type FLOOR: ").strip().upper() != "FLOOR":
            print("aborted")
            return 1
        link.arm()
        time.sleep(0.3)
        # ---- straight line
        link.reset_encoders()
        time.sleep(0.3)
        l0, r0 = protocol.side_counts(link.enc)
        pulses(link, "F", a.pulses, a.ms)
        l1, r1 = protocol.side_counts(link.enc)
        dist = float(ask("Measure how far the robot moved forward (metres): "))
        counts = ((l1 - l0) + (r1 - r0)) / 2.0
        result["counts_per_m"] = round(counts / dist, 1)
        result["forward_m_per_pulse"] = round(dist / a.pulses, 3)
        # ---- pivot
        l0, r0 = protocol.side_counts(link.enc)
        pulses(link, "L", a.pulses, a.ms)
        l1, r1 = protocol.side_counts(link.enc)
        ang = float(ask("Measure how far it rotated LEFT (degrees, counter-clockwise): "))
        dl, dr = (l1 - l0) / result["counts_per_m"], (r1 - r0) / result["counts_per_m"]
        result["effective_track_m"] = round(abs(dr - dl) / math.radians(ang), 3)
        result["deg_per_turn_pulse"] = round(ang / a.pulses, 1)
    finally:
        link.stop()
        link.disarm()
        link.disconnect()
    os.makedirs("results", exist_ok=True)
    with open(os.path.join("results", "odometry_calibration.json"), "w") as f:
        json.dump(result, f, indent=1)
    print(json.dumps(result, indent=1))
    print('Config snippet: {"odometry": {"counts_per_m": %s, "effective_track_m": %s}}' %
          (result["counts_per_m"], result["effective_track_m"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
