#!/usr/bin/env python3
"""Show which camera index is which, so you pick the USB webcam and not the laptop's own camera.

    python tools/list_cameras.py

It opens cameras 0..5 briefly (the laptop camera light may flash), prints each one that works, and saves a labelled
picture of what each sees to results/cameras/cameras.jpg and results/cameras/cam_N.jpg. Open the picture: the tile that shows
the view from your robot's webcam is the index to use (--webcam-index N).

Certain way to tell: unplug the USB webcam, run this, plug it back in, run again. The index that appears is the USB webcam.
"""
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from warehouse_bot import camera


def verbose_probe(max_index=4):
    """Print, per camera index and per backend, whether it opens and delivers frames. Saves NO pictures."""
    import time
    for i in range(max_index):
        for name, code in camera._backends():
            if name != "ANY" and code is None:
                continue
            cap = camera.open_with_backend(i, code)
            opened = cap.isOpened()
            got = None
            if opened:
                for _ in range(12):
                    ok, img = cap.read()
                    if ok and img is not None:
                        got = "%dx%d" % (img.shape[1], img.shape[0])
                        break
                    time.sleep(0.08)
            cap.release()
            print("index %d  backend %-5s  opened=%-5s  frames=%s" % (i, name, opened, got or "NONE"))


def main():
    if "--verbose" in sys.argv:
        verbose_probe()
        print("\nIf an index says opened=True but frames=NONE on every backend: another app is using the camera (close the Camera app,")
        print("Teams, Zoom, browser tabs), or Windows blocks desktop apps: Settings > Privacy & security > Camera >")
        print("'Let desktop apps access your camera' = On.")
        return 0
    out_dir = os.path.join("results", "cameras")
    os.makedirs(out_dir, exist_ok=True)
    cams = camera.list_webcams()
    if not cams:
        print("No camera delivered a frame. Is the USB webcam plugged in? Is another app (Teams, browser) using it?")
        return 1
    tiles = []
    for idx, w, h in cams:
        cap = camera._open_capture(idx)
        img, bright = None, 0.0
        for k in range(75):                     # auto-exposure can take 1-3 s (webcams start BLACK): wait until bright
            ok, frame = cap.read()
            if ok and frame is not None:
                img, bright = frame, float(frame.mean())
                if k >= 20 and bright > 25:
                    break
            time.sleep(0.04)
        cap.release()
        if img is None:
            continue
        cv2.imwrite(os.path.join(out_dir, "cam_%d.jpg" % idx), img)
        tile = cv2.resize(img, (426, 320))
        cv2.rectangle(tile, (0, 0), (426, 34), (0, 0, 0), -1)
        cv2.putText(tile, "--webcam-index %d  (%dx%d)" % (idx, w, h), (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        tiles.append(tile)
        print("camera index %d: %dx%d  brightness %.0f%s" % (idx, w, h, bright,
                                                          "   <-- still black: wait/more light, or this is an IR camera" if bright < 15 else ""))
    cols = min(3, len(tiles))
    rows = [np.hstack(tiles[i:i + cols] + [np.zeros_like(tiles[0])] * (cols - len(tiles[i:i + cols])))
            for i in range(0, len(tiles), cols)]
    sheet = np.vstack(rows)
    path = os.path.join(out_dir, "cameras.jpg")
    cv2.imwrite(path, sheet)
    print("\nPicture saved: %s" % os.path.abspath(path))
    print("Open it. The tile showing your robot's webcam view tells you the index to pass as --webcam-index N.")
    print("If unsure: unplug the USB webcam, run this again; the index that disappears is the USB webcam.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
