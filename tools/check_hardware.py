#!/usr/bin/env python3
"""Hardware bring-up test: serial link, firmware pins, drive directions and all four encoders.

    python3 tools/check_hardware.py --port /dev/ttyACM0

PUT THE ROBOT ON A BLOCK SO THE WHEELS ARE OFF THE GROUND FIRST. It sends four short pulses (forward, backward,
left, right) and reports, per wheel, whether its encoder counted in the expected direction. Nothing here is
automatic beyond those four 0.25 s pulses, and the script refuses to run until you type LIFTED.

Expected (convention used by the whole project): on FORWARD every encoder counts UP; on LEFT (pivot) the left-side
encoders (TL, BL) count DOWN and the right-side encoders (TR, BR) count UP.
If a wheel disagrees, flip that wheel's value in ENC_SIGN[] (or the side's DRIVE_SIGN) in the .ino and re-upload.

    --port mock   runs the same script against the software test firmware (no hardware, no motion).
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from warehouse_bot.config import load_config
from warehouse_bot.serial_robot import RobotLink, RobotLinkError

NAMES = ("TL", "TR", "BR", "BL")
EXPECT = {"F": (1, 1, 1, 1), "B": (-1, -1, -1, -1), "L": (-1, 1, 1, -1), "R": (1, -1, -1, 1)}


def run(port, ms, confirm=None):
    cfg = load_config()
    link = RobotLink(cfg)
    report = {"port": port, "pulse_ms": ms, "time": time.strftime("%Y-%m-%d %H:%M:%S"), "pulses": {}, "ok": False}
    try:
        print("connecting to %s (the Uno resets when the port opens, ~3 s)..." % port)
        link.connect(port)
    except RobotLinkError as e:
        print("FAIL: %s" % e)
        return report
    try:
        if not link.link_ok:
            print("FAIL: no telemetry. Wrong port/baud, or the warehouse_bot.ino sketch is not uploaded.")
            return report
        print("link OK, firmware: %s" % link.fw)
        r = link.send("PINS")
        print("firmware pin map: %s" % r.get("text"))
        print("Compare with your wiring: IN1-4 = D2-D5; encoders (B,A): TL D6/D7, TR D8/D9, BR D10/D11, BL D12/D13.")
        print("Encoders now: %s   (turn a wheel by hand: its count should change)" % dict(zip(NAMES, link.enc)))
        ans = confirm() if confirm else input("\nAre the wheels OFF THE GROUND and your hand near the battery switch? Type LIFTED to continue: ")
        if ans.strip().upper() != "LIFTED":
            print("aborted - nothing moved.")
            return report
        if not link.arm()["ok"]:
            print("FAIL: could not arm")
            return report
        time.sleep(0.3)
        all_ok = True
        for d, label in (("F", "FORWARD"), ("B", "BACKWARD"), ("L", "LEFT (pivot)"), ("R", "RIGHT (pivot)")):
            link.reset_encoders()
            time.sleep(0.25)
            before = list(link.enc)
            res = link.move(d, ms)
            if not res["ok"]:
                print("FAIL: %s refused: %s" % (label, res["error"]))
                all_ok = False
                continue
            time.sleep(ms / 1000.0 + 0.7)
            delta = [a - b for a, b in zip(link.enc, before)]
            verdicts = []
            for i, n in enumerate(NAMES):
                if abs(delta[i]) < 3:
                    verdicts.append("%s: NO COUNTS (encoder wire? motor didn't spin?)" % n)
                    all_ok = False
                elif (delta[i] > 0) != (EXPECT[d][i] > 0):
                    verdicts.append("%s: %+d  WRONG SIGN -> flip ENC_SIGN[%d] (or DRIVE sign if the wheel spun the wrong way)" % (n, delta[i], i))
                    all_ok = False
                else:
                    verdicts.append("%s: %+d ok" % (n, delta[i]))
            print("%-14s %s" % (label, " | ".join(verdicts)))
            report["pulses"][d] = delta
            time.sleep(0.6)
        link.disarm()
        report["ok"] = all_ok
        print("\nRESULT: %s" % ("PASS - all four wheels and encoders respond with the expected signs" if all_ok else
                                "CHECK THE LINES ABOVE (fix wiring/signs, re-upload, rerun)"))
        print("Also verify by eye: during FORWARD all four wheels must turn the robot-forward direction.")
        return report
    finally:
        link.stop()
        link.disconnect()
        os.makedirs("results", exist_ok=True)
        path = os.path.join("results", "hardware_check_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
        with open(path, "w") as f:
            json.dump(report, f, indent=1)
        print("report saved: %s" % path)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="auto")
    ap.add_argument("--ms", type=int, default=250)
    a = ap.parse_args(argv)
    rep = run(a.port, a.ms)
    return 0 if rep.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
