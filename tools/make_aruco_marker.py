#!/usr/bin/env python3
"""Generate a printable ArUco marker at an exact physical size.

    python3 tools/make_aruco_marker.py                    # id 0, 4x4_50, 10 cm  -> results/markers/
    python3 tools/make_aruco_marker.py --size-cm 15 --id 3

PRINT AT 100 % SCALE (no "fit to page"), then MEASURE the black square with a ruler and, if it is not exactly the
size you asked for, set  docking.marker_size_m  in your config. Distance by PnP depends on this number.
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from warehouse_bot import docking


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--dict", default="DICT_4X4_50")
    ap.add_argument("--size-cm", type=float, default=10.0, help="edge length of the BLACK square when printed")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--out", default=os.path.join("results", "markers"))
    a = ap.parse_args()

    px = int(round(a.size_cm / 2.54 * a.dpi))
    marker = docking.generate_marker(a.dict, a.id, px)
    margin = px // 4                                       # white quiet zone is required for detection
    page = np.full((px + 2 * margin + 90, px + 2 * margin), 255, np.uint8)
    page[margin:margin + px, margin:margin + px] = marker
    label = "%s id=%d  black square = %.1f cm  (print at 100%%)" % (a.dict, a.id, a.size_cm)
    cv2.putText(page, label, (margin, px + 2 * margin + 45), cv2.FONT_HERSHEY_SIMPLEX, max(0.5, px / 700.0), 0, 2, cv2.LINE_AA)
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, "aruco_%s_id%d_%gcm.png" % (a.dict.replace("DICT_", "").lower(), a.id, a.size_cm))
    cv2.imwrite(path, page)
    print("wrote %s  (%dx%d px at %d dpi)" % (path, page.shape[1], page.shape[0], a.dpi))
    print("Print it at 100%% scale. Measure the black square: it should be %.1f cm." % a.size_cm)
    print("Then set docking.marker_size_m = %.3f in your config if your measurement differs." % (a.size_cm / 100.0))


if __name__ == "__main__":
    main()
