"""OBSTACLE PROXY for setups WITHOUT a depth sensor (e.g. a plain USB webcam).

A depth camera measures distance to ANY obstacle. A webcam cannot. As an honest stand-in, a box can be tagged with
its own ArUco marker (a different id from the docking marker); the distance to that marker comes from its apparent
size (PnP, needs the camera focal length and the printed marker size). We then build a *proxy depth image* that
is "far" everywhere except in the columns the tagged box occupies, and feed it to the SAME ObstacleDetector and
AvoidanceFSM used with a real depth camera.

LIMITS (say them out loud in any video): only tagged boxes are "seen"; an untagged obstacle is invisible; this is
a stand-in for the RealSense path, not a measurement of the room.
"""
import numpy as np

from .docking import marker_pnp_distance


class MarkerObstacleProxy(object):
    kind_label = "MARKER-OBSTACLE PROXY"
    depth_note = "PROXY DEPTH: tagged boxes only, NOT measured"
    item_label = "TAGGED BOX"

    def __init__(self, detector, obstacle_id, size_m, far_m, width_factor=3.0):
        self.detector = detector
        self.obstacle_id = obstacle_id
        self.size_m = size_m
        self.far_m = far_m
        self.width_factor = width_factor          # tagged box assumed this many marker-widths wide

    def depth(self, color, intr):
        """Returns (proxy_depth float32 HxW in metres, list of dicts describing the tagged obstacles seen)."""
        h, w = color.shape[:2]
        d = np.full((h, w), self.far_m, np.float32)
        seen = []
        for mid, quad in self.detector.detect_all(color):
            if mid != self.obstacle_id:
                continue
            dist = marker_pnp_distance(quad, self.size_m, intr, w, h)
            if dist is None:
                continue
            x0, x1 = float(quad[:, 0].min()), float(quad[:, 0].max())
            grow = (x1 - x0) * (self.width_factor - 1.0) / 2.0
            c0, c1 = max(0, int(x0 - grow)), min(w, int(x1 + grow) + 1)
            d[:, c0:c1] = np.minimum(d[:, c0:c1], dist)
            seen.append({"id": mid, "distance_m": round(dist, 3), "cols": [c0, c1]})
        return d, seen
