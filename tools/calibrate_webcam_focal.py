#!/usr/bin/env python3
"""One-point focal-length calibration for a plain webcam, so ArUco marker size can give distance.

    python tools/calibrate_webcam_focal.py --distance-cm 50 --marker-size-cm 9.5 --dict DICT_6X6_50 --id 2

How: put the marker FLAT on a wall/box, facing the camera squarely. Measure the distance from the camera lens to the
marker with a tape measure (e.g. 50 cm) and the width of the BLACK square with a ruler. Keep both still while this runs.
focal_px = (measured marker edge in pixels) * distance / marker edge.

Accuracy is roughly +-5 %, limited by how well you measure and how square-on the marker is. It assumes no lens
distortion. Use the printed --focal-px value on the dashboard:  python -m warehouse_bot ... --focal-px <value>
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from warehouse_bot import camera, docking


def focal_from_sizes(sizes_px, distance_m, marker_size_m):
    return float(np.median(sizes_px)) * distance_m / marker_size_m


def edge_px(quad):
    return float(np.mean([np.linalg.norm(quad[i] - quad[(i + 1) % 4]) for i in range(4)]))


def measure(source, detector, marker_id, seconds=5.0, max_frames=400):
    sizes, t0, n = [], time.time(), 0
    while time.time() - t0 < seconds and n < max_frames:
        f = source.read()
        n += 1
        if f is None:
            continue
        for mid, quad in detector.detect_all(f.color):
            if mid == marker_id:
                sizes.append(edge_px(quad))
    return sizes


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--distance-cm", type=float, required=True, help="lens-to-marker distance, measured")
    ap.add_argument("--marker-size-cm", type=float, default=9.5, help="black square edge, measured")
    ap.add_argument("--dict", default="DICT_6X6_50")
    ap.add_argument("--id", type=int, default=2)
    ap.add_argument("--index", type=int, default=0, help="webcam index (0 = laptop's own)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--seconds", type=float, default=5.0)
    a = ap.parse_args(argv)
    try:
        src = camera.WebcamSource(a.index, a.width, a.height)
    except camera.SourceError as e:
        print("FAIL:", e)
        return 1
    det = docking.ArucoDetector({"aruco_dict": a.dict, "marker_id": a.id, "marker_size_m": a.marker_size_cm / 100.0})
    print("measuring for %.0f s... keep the marker still and square-on" % a.seconds)
    sizes = measure(src, det, a.id, a.seconds)
    src.close()
    if len(sizes) < 10:
        print("FAIL: marker id %d seen in only %d frames. Check the id/dictionary, light, distance (0.3-1 m)." % (a.id, len(sizes)))
        return 1
    f = focal_from_sizes(sizes, a.distance_cm / 100.0, a.marker_size_cm / 100.0)
    spread = 100.0 * (np.percentile(sizes, 90) - np.percentile(sizes, 10)) / np.median(sizes)
    os.makedirs("results", exist_ok=True)
    with open(os.path.join("results", "webcam_focal.json"), "w") as fh:
        json.dump({"focal_px": round(f, 1), "width": a.width, "distance_cm": a.distance_cm,
                   "marker_size_cm": a.marker_size_cm, "frames": len(sizes)}, fh, indent=1)
    print("marker edge: %.1f px (%d frames, spread %.1f %%)" % (np.median(sizes), len(sizes), spread))
    print("FOCAL LENGTH: %.1f px at %d px wide" % (f, a.width))
    if spread > 8:
        print("WARNING: the size jittered a lot; hold everything still and repeat.")
    print("Use:  --focal-px %.0f" % f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
