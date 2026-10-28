#!/usr/bin/env python3
"""Record a short RGB-D session from the RealSense for later offline playback (the camera can then go back).

    python3 tools/record_session.py --name obstacle_demo --seconds 20
    python3 -m warehouse_bot --camera session:results/sessions/obstacle_demo     # replay, labelled RECORDED

Writes color_00000.jpg + depth_00000.png (16-bit, millimetre-scale units) + meta.json (intrinsics, depth scale).
Playback is clearly labelled RECORDED in the dashboard; it is real measured depth from the earlier run.
"""
import argparse
import json
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from warehouse_bot import camera


def record(source, out_dir, seconds, fps=10.0, depth_unit_m=0.001):
    os.makedirs(out_dir, exist_ok=True)
    t0, last, n, intr = time.time(), 0.0, 0, None
    while time.time() - t0 < seconds:
        f = source.read()
        if f is None or f.depth is None:
            continue
        if time.time() - last < 1.0 / fps:
            continue
        last = time.time()
        intr = f.intrinsics
        cv2.imwrite(os.path.join(out_dir, "color_%05d.jpg" % n), f.color, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        depth_u16 = (f.depth / depth_unit_m).clip(0, 65535).astype("uint16")
        cv2.imwrite(os.path.join(out_dir, "depth_%05d.png" % n), depth_u16)
        n += 1
    meta = {"frames": n, "fps": fps, "depth_scale_m_per_unit": depth_unit_m, "intrinsics": intr.to_dict() if intr else None,
            "source": getattr(source, "label", "?"), "recorded": time.strftime("%Y-%m-%d %H:%M:%S")}
    with open(os.path.join(out_dir, "meta.json"), "w") as fh:
        json.dump(meta, fh, indent=1)
    return n


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default=time.strftime("session_%Y%m%d_%H%M%S"))
    ap.add_argument("--seconds", type=float, default=15)
    ap.add_argument("--fps", type=float, default=10)
    a = ap.parse_args(argv)
    cfg = load_cfg()
    src, msg = camera.open_source("realsense", cfg)
    print(msg)
    if src is None:
        print("cannot record: no RealSense. (Nothing is invented.)")
        return 1
    out = os.path.join("results", "sessions", a.name)
    print("recording %.0f s to %s ..." % (a.seconds, out))
    n = record(src, out, a.seconds, a.fps)
    src.close()
    print("saved %d frames" % n)
    return 0 if n else 1


def load_cfg():
    from warehouse_bot.config import load_config
    return load_config()


if __name__ == "__main__":
    sys.exit(main())
