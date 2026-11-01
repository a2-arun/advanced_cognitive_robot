#!/usr/bin/env python3
"""One-point focal-length calibration for a plain webcam, so ArUco marker size can give distance.

    python tools/calibrate_webcam_focal.py --distance-cm 50 --marker-size-cm 9.5 --dict DICT_6X6_50 --id 2 --index 0

How: put the marker FLAT on a wall/box, facing the camera squarely. Measure the distance from the camera lens to the
marker with a tape measure (e.g. 50 cm) and the width of the BLACK square with a ruler. Keep both still while this runs.
focal_px = (measured marker edge in pixels) * distance / marker edge.

Accuracy is roughly +-5 %, limited by how well you measure and how square-on the marker is. It assumes no lens
distortion. Use the printed --focal-px value on the dashboard:  python -m warehouse_bot ... --focal-px <value>

A window opens showing the live camera with the detected marker outlined; move the marker until it says FOUND, then press SPACE.
(--no-preview skips the window.)

If the marker is not found, the tool tells you what the camera DID see (brightness, markers of other families/ids) and saves
a picture to results/cameras/focal_debug.jpg so you can look at what the camera sees.
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from warehouse_bot import camera, docking


def focal_from_sizes(sizes_px, distance_m, marker_size_m):
    return float(np.median(sizes_px)) * distance_m / marker_size_m


def edge_px(quad):
    return float(np.mean([np.linalg.norm(quad[i] - quad[(i + 1) % 4]) for i in range(4)]))


def warm_up(source, seconds=4.0, min_reads=15, bright=25.0):
    """Read and discard frames until the picture is not black (webcam auto-exposure takes 1-3 s). Returns the last frame."""
    t0, n, last = time.time(), 0, None
    while time.time() - t0 < seconds:
        f = source.read()
        if f is None:
            continue
        n += 1
        last = f
        if n >= min_reads and float(f.color.mean()) > bright:
            break
    return last


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


def annotate_preview(color, detector, marker_id, note=""):
    """Draw every detected marker (green = the wanted one, orange = another id) and a status line. Returns (image, wanted_edge_px or None)."""
    vis = color.copy()
    h, w = vis.shape[:2]
    wanted = None
    for mid, quad in detector.detect_all(color):
        ok = mid == marker_id
        col = (0, 220, 0) if ok else (0, 140, 255)
        cv2.polylines(vis, [quad.astype(np.int32).reshape(-1, 1, 2)], True, col, 3)
        cv2.putText(vis, "id %d" % mid, (int(quad[0][0]), max(int(quad[0][1]) - 8, 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
        if ok:
            wanted = edge_px(quad)
    if wanted is not None:
        msg, col = "FOUND id %d  edge %.0f px  -> press SPACE to measure" % (marker_id, wanted), (0, 220, 0)
    else:
        msg, col = "id %d NOT found: show the WHOLE black square + white border, flat, square-on" % marker_id, (0, 0, 255)
    cv2.rectangle(vis, (0, 0), (w, 30), (0, 0, 0), -1)
    cv2.putText(vis, msg, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
    if note:
        cv2.putText(vis, note, (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    return vis, wanted


def live_preview(source, detector, marker_id, window="focal calibration (SPACE = measure, Q = quit)"):
    """Show the camera live until SPACE is pressed. Returns True to go on, False to quit, None if no window can be shown."""
    try:
        cv2.namedWindow(window)
    except cv2.error:
        return None
    try:
        while True:
            f = source.read()
            if f is None:
                continue
            vis, _ = annotate_preview(f.color, detector, marker_id)
            cv2.imshow(window, vis)
            k = cv2.waitKey(1) & 0xFF
            if k == 32:
                return True
            if k in (ord("q"), ord("Q"), 27):
                return False
    except cv2.error:
        return None
    finally:
        try:
            cv2.destroyWindow(window)
            cv2.waitKey(1)
        except cv2.error:
            pass


def diagnose_frame(color):
    """What markers of ANY ArUco family are visible in this frame? Returns {dictionary_name: [ids]}."""
    found = {}
    for name in sorted(n for n in dir(cv2.aruco) if n.startswith("DICT_") and "MIP" not in n):
        try:
            det = docking.ArucoDetector({"aruco_dict": name, "marker_id": 0, "marker_size_m": 0.1})
            ids = sorted(set(i for i, _ in det.detect_all(color)))
        except Exception:
            continue
        if ids:
            found[name] = ids
    return found


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--distance-cm", type=float, required=True, help="lens-to-marker distance, measured")
    ap.add_argument("--marker-size-cm", type=float, default=9.5, help="black square edge, measured")
    ap.add_argument("--dict", default="DICT_6X6_50")
    ap.add_argument("--id", type=int, default=2)
    ap.add_argument("--index", type=int, default=None,
                    help="webcam index. REQUIRED if several cameras exist (python tools/list_cameras.py shows which is which)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--no-preview", action="store_true", help="skip the live window (default: show it so you can line the marker up)")
    a = ap.parse_args(argv)
    try:
        src = camera.WebcamSource(camera.resolve_webcam_index(a.index), a.width, a.height)
    except camera.SourceError as e:
        print("FAIL:", e)
        return 1
    det = docking.ArucoDetector({"aruco_dict": a.dict, "marker_id": a.id, "marker_size_m": a.marker_size_cm / 100.0})
    print("warming up the camera (auto-exposure)...")
    last = warm_up(src)
    if not a.no_preview:
        print("A window shows what the camera sees. Move the marker until it says FOUND (green outline), then press SPACE. Q quits.")
        go = live_preview(src, det, a.id)
        if go is False:
            src.close()
            print("Quit.")
            return 1
        if go is None:
            print("(no window available here; measuring without a preview)")
    print("measuring for %.0f s... keep the marker still and square-on" % a.seconds)
    sizes = measure(src, det, a.id, a.seconds)
    if last is not None:
        last = src.read() or last
    src.close()
    if len(sizes) < 10:
        print("\nFAIL: marker id %d (%s) seen in only %d frames." % (a.id, a.dict, len(sizes)))
        if last is None:
            print("The camera delivered no frames at all. Close other apps using it, or check --index.")
            return 1
        bright = float(last.color.mean())
        seen = diagnose_frame(last.color)
        print("What the camera saw: brightness %.0f/255 (%s), picture %dx%d" % (
            bright, "TOO DARK/black: more light, wait, or wrong camera" if bright < 25 else "ok", last.color.shape[1], last.color.shape[0]))
        if seen:
            for name, ids in seen.items():
                print("  it DOES see markers: %s ids %s" % (name, ids))
            print("-> Use the matching --dict and --id (and re-measure)." if a.dict not in seen else
                  "-> It sees the family but not id %d: wrong marker, or a different id." % a.id)
        else:
            print("  no ArUco marker of any family is visible. Check: marker fully in view with its white border, flat (not curled),"
                  " no glare, 30-80 cm away, camera is the Logitech (--index).")
        vis = last.color.copy()
        cv2.putText(vis, "focal calibration: nothing found for %s id %d" % (a.dict, a.id), (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 255), 2)
        os.makedirs(os.path.join("results", "cameras"), exist_ok=True)
        path = os.path.join("results", "cameras", "focal_debug.jpg")
        cv2.imwrite(path, vis)
        print("Saved what the camera sees: %s  (open it)" % os.path.abspath(path))
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
