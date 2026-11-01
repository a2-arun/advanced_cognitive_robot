#!/usr/bin/env python3
"""See what the YOLO bottle detector sees, BEFORE driving the robot.

    python tools/check_bottle_detection.py --index 0 --focal-px 981 --bottle-height-cm 20

A window shows the webcam with a box around every bottle, its confidence, and the size-based distance estimate. Stand the bottle
upright at a tape-measured distance (e.g. 50 cm and 80 cm from the lens) and compare. Q or Esc quits.

  --all       show EVERY object YOLO recognises (to see what it calls your bottle, e.g. 'cup' or 'vase')
  --classes   comma list of class names to treat as the obstacle (default: bottle)
  --conf      confidence threshold (lower it, e.g. 0.15, if a shiny metal bottle is missed)
  --image F   run on one picture instead of the camera (saves results/cameras/bottle_check.jpg)

The first run downloads the small pretrained weights (yolov8n.pt, ~6 MB).
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from warehouse_bot import camera
from warehouse_bot.yolo_proxy import BottleObstacleProxy, UltralyticsDetector


def draw(color, proxy, intr):
    """Annotate one frame. Returns (image, list of the proxy's 'seen' dicts)."""
    _, seen = proxy.depth(color, intr)
    vis = color.copy()
    for s in seen:
        b = s["box"]
        cv2.rectangle(vis, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)
        if s["distance_m"] is None:
            txt = "%s %.2f  (lying down: no distance)" % (s["class"], s["conf"])
        else:
            txt = "%s %.2f  %s%.2f m" % (s["class"], s["conf"], "<= " if s["cropped"] else "~", s["distance_m"])
        cv2.putText(vis, txt, (b[0], max(b[1] - 8, 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.rectangle(vis, (0, vis.shape[0] - 26), (vis.shape[1], vis.shape[0]), (0, 0, 0), -1)
    cv2.putText(vis, "%d object(s) | distance = ESTIMATE from size, not measured | Q quits" % len(seen), (6, vis.shape[0] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return vis, seen


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=None, help="webcam index (python tools/list_cameras.py)")
    ap.add_argument("--focal-px", type=float, required=True, help="from tools/calibrate_webcam_focal.py")
    ap.add_argument("--bottle-height-cm", type=float, required=True, help="MEASURED height of the bottle, upright")
    ap.add_argument("--classes", default="bottle")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--conf", type=float, default=0.30)
    ap.add_argument("--model", default="yolov8n.pt")
    ap.add_argument("--imgsz", type=int, default=480)
    ap.add_argument("--image", help="a picture file instead of the camera")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    a = ap.parse_args(argv)

    print("loading YOLO (%s)... first run downloads the weights" % a.model)
    det = UltralyticsDetector(a.model, None if a.all else [c.strip() for c in a.classes.split(",")], a.conf, a.imgsz)
    proxy = BottleObstacleProxy(det, a.bottle_height_cm / 100.0, 4.0)

    if a.image:
        img = cv2.imread(a.image)
        if img is None:
            print("FAIL: cannot read", a.image)
            return 1
        h, w = img.shape[:2]
        vis, seen = draw(img, proxy, camera.Intrinsics(w, h, a.focal_px, a.focal_px, w / 2.0, h / 2.0))
        os.makedirs(os.path.join("results", "cameras"), exist_ok=True)
        path = os.path.join("results", "cameras", "bottle_check.jpg")
        cv2.imwrite(path, vis)
        print("%d object(s): %s" % (len(seen), [(s["class"], s["conf"], s["distance_m"]) for s in seen]))
        print("saved", os.path.abspath(path))
        return 0

    try:
        src = camera.WebcamSource(camera.resolve_webcam_index(a.index), a.width, a.height, a.focal_px)
    except camera.SourceError as e:
        print("FAIL:", e)
        return 1
    win = "bottle check (Q quits)"
    last_print = None
    try:
        while True:
            f = src.read()
            if f is None:
                continue
            vis, seen = draw(f.color, proxy, f.intrinsics)
            try:
                cv2.imshow(win, vis)
                k = cv2.waitKey(1) & 0xFF
            except cv2.error:
                print("no window available here; use --image instead")
                return 1
            if k in (ord("q"), ord("Q"), 27):
                break
            line = ", ".join("%s %.2f ~%s m" % (s["class"], s["conf"], s["distance_m"]) for s in seen) or "nothing"
            if line != last_print:
                print(line)
                last_print = line
    finally:
        src.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
