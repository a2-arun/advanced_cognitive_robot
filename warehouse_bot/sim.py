"""Lightweight 2D warehouse SIMULATION (kinematic model, not physics, not SLAM).

* Robot pose is integrated from the same pulse commands the real robot receives (move(dir, ms) / stop()).
* A simulated RGB-D camera is produced by ray casting: depth per image column (obstacles are full-height) and a
  first-person colour view with a REAL ArUco marker warped onto the dock wall, so the production ArUco detector and
  the production obstacle/docking state machines run on simulated input.
* Speeds are NOT calibrated to the real robot.
* Two paths are kept: ground truth, and an "estimated" path from noisy simulated wheel odometry (it drifts) —
  illustrating dead reckoning, which is NOT SLAM.
"""
import math
import random
import threading
from collections import deque

import cv2
import numpy as np

from .camera import Frame, Intrinsics, KIND_SIM
from . import docking

CAM_W, CAM_H, HFOV_DEG, CAM_HEIGHT_M = 640, 480, 87.0, 0.25
WALL_H_M = 1.5
MARKER_SIM_SIZE_M = 0.40        # printed-marker size inside the simulation (sim only; the real print size is in config)
SIM_COUNTS_PER_M = 1000.0       # purely illustrative simulated encoder scale


class Box(object):
    def __init__(self, x0, y0, x1, y1, kind="wall", oid=None):
        self.x0, self.y0, self.x1, self.y1 = float(x0), float(y0), float(x1), float(y1)
        self.kind, self.id = kind, oid

    def to_dict(self):
        return {"x0": self.x0, "y0": self.y0, "x1": self.x1, "y1": self.y1, "kind": self.kind, "id": self.id}


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Sim(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.s = cfg["sim"]
        self.W, self.H = self.s["world_w"], self.s["world_h"]
        f = (CAM_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))
        self.intr = Intrinsics(CAM_W, CAM_H, f, f, CAM_W / 2.0, CAM_H / 2.0)
        self.lock = threading.RLock()
        self.running = True
        self._marker_img = None
        self.reset()

    # ------------------------------------------------------------------ world
    def reset(self):
        with self.lock:
            self.rng = random.Random(self.s["seed"])
            self.np_rng = np.random.RandomState(self.s["seed"])
            t = 0.2
            self.walls = [Box(-t, -t, self.W + t, 0, oid="w_s"), Box(-t, self.H, self.W + t, self.H + t, oid="w_n"),
                          Box(-t, 0, 0, self.H, oid="w_w"), Box(self.W, 0, self.W + t, self.H, oid="w_e")]
            self.shelves = [Box(2.6, 0.8, 5.6, 1.2, "shelf", "sh1"), Box(2.6, 4.8, 5.6, 5.2, "shelf", "sh2")]
            self._next_id = 1
            self.obstacles = []
            self.x, self.y, self.th = 1.0, self.H / 2.0, 0.0
            self.add_obstacle(4.0, 3.0, 0.6)
            # dock station on the east wall, marker faces west (into the arena)
            self.dock = {"x": self.W, "y": self.H / 2.0, "normal": math.pi, "size": MARKER_SIM_SIZE_M}
            self.ex, self.ey, self.eth = self.x, self.y, self.th        # odometry estimate
            self.trail = deque(maxlen=3000)
            self.est_trail = deque(maxlen=3000)
            self.trail.append((self.x, self.y))
            self.est_trail.append((self.ex, self.ey))
            self.motion = None          # {"dir","left"}
            self.collided = False
            self.last_event = "reset"
            self.counts = [0.0, 0.0, 0.0, 0.0]
            self.pulses = 0
            self.sim_time = 0.0

    def add_obstacle(self, x, y, size=0.5):
        with self.lock:
            h = size / 2.0
            b = Box(x - h, y - h, x + h, y + h, "obstacle", "o%d" % self._next_id)
            if self._collides_box(b, self.x, self.y, self.s["robot_radius_m"] + 0.1):
                return None
            self._next_id += 1
            self.obstacles.append(b)
            return b.id

    def remove_obstacle_at(self, x, y):
        with self.lock:
            for b in list(self.obstacles):
                if b.x0 <= x <= b.x1 and b.y0 <= y <= b.y1:
                    self.obstacles.remove(b)
                    return True
            return False

    def _all_boxes(self):
        return self.walls + self.shelves + self.obstacles

    @staticmethod
    def _collides_box(b, x, y, r):
        cx = min(max(x, b.x0), b.x1)
        cy = min(max(y, b.y0), b.y1)
        return (x - cx) ** 2 + (y - cy) ** 2 < r * r

    def _collides(self, x, y):
        r = self.s["robot_radius_m"]
        return any(self._collides_box(b, x, y, r) for b in self._all_boxes())

    # ------------------------------------------------------------------ actuator interface (mirrors RobotLink)
    def move(self, direction, ms=None):
        with self.lock:
            if self.motion is not None:
                return {"ok": False, "error": "BUSY"}
            ms = int(ms if ms is not None else self.cfg["motion"]["default_pulse_ms"])
            ms = max(self.cfg["motion"]["min_pulse_ms"], min(self.cfg["motion"]["max_pulse_ms"], ms))
            if direction not in ("F", "B", "L", "R"):
                return {"ok": False, "error": "DIR"}
            self.motion = {"dir": direction, "left": ms / 1000.0}
            self.collided = False
            self.pulses += 1
            self.last_event = "pulse %s %d ms" % (direction, ms)
            return {"ok": True, "ms": ms}

    def stop(self):
        with self.lock:
            self.motion = None
            self.last_event = "stopped"
            return {"ok": True}

    def is_busy(self):
        return self.motion is not None

    # ------------------------------------------------------------------ dynamics
    def tick(self, dt):
        """Advance by dt seconds of wall-clock time if running. Returns True if a pulse just finished."""
        with self.lock:
            if not self.running:
                return False
            return self._advance(dt)

    def step_once(self, dt=0.1):
        with self.lock:
            return self._advance(dt)

    def _advance(self, dt):
        self.sim_time += dt
        m = self.motion
        if m is None:
            return False
        d = min(dt, m["left"])
        spd, turn = self.s["speed_mps"], math.radians(self.s["turn_dps"])
        if m["dir"] in ("F", "B"):
            sgn = 1.0 if m["dir"] == "F" else -1.0
            nx, ny = self.x + sgn * spd * d * math.cos(self.th), self.y + sgn * spd * d * math.sin(self.th)
            if self._collides(nx, ny):
                self.collided = True
                self.motion = None
                self.last_event = "COLLISION - stopped"
                return True
            dist = sgn * spd * d
            self.x, self.y = nx, ny
            # noisy wheel odometry (scale error + noise) -> estimate drifts from truth
            dist_est = dist * 1.04 + self.np_rng.normal(0, 0.002)
            self.ex += dist_est * math.cos(self.eth)
            self.ey += dist_est * math.sin(self.eth)
            self._count(dist, dist)
        else:
            sgn = 1.0 if m["dir"] == "L" else -1.0
            dth = sgn * turn * d
            self.th = _wrap(self.th + dth)
            self.eth = _wrap(self.eth + dth * 0.93 + self.np_rng.normal(0, 0.002))
            arc = 0.12 * dth
            self._count(-arc, arc)
        m["left"] -= d
        if self.trail[-1] is None or math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) > 0.03:
            self.trail.append((self.x, self.y))
            self.est_trail.append((self.ex, self.ey))
        if m["left"] <= 1e-6:
            self.motion = None
            return True
        return False

    def _count(self, dl, dr):
        c = SIM_COUNTS_PER_M
        self.counts[0] += dl * c
        self.counts[3] += dl * c
        self.counts[1] += dr * c
        self.counts[2] += dr * c

    # ------------------------------------------------------------------ simulated camera
    def _rect_array(self):
        return np.array([[b.x0, b.y0, b.x1, b.y1] for b in self._all_boxes()], dtype=np.float64)

    def _cast(self):
        """Return (z per column, hit-box index per column)."""
        u = np.arange(CAM_W, dtype=np.float64)
        phi = np.arctan((u - self.intr.ppx) / self.intr.fx)
        ang = self.th - phi
        dx, dy = np.cos(ang), np.sin(ang)
        dx = np.where(np.abs(dx) < 1e-9, 1e-9, dx)
        dy = np.where(np.abs(dy) < 1e-9, 1e-9, dy)
        R = self._rect_array()                                  # M x 4
        with np.errstate(divide="ignore", invalid="ignore"):
            tx0 = (R[:, 0][:, None] - self.x) / dx[None, :]
            tx1 = (R[:, 2][:, None] - self.x) / dx[None, :]
            ty0 = (R[:, 1][:, None] - self.y) / dy[None, :]
            ty1 = (R[:, 3][:, None] - self.y) / dy[None, :]
        tmin = np.maximum(np.minimum(tx0, tx1), np.minimum(ty0, ty1))
        tmax = np.minimum(np.maximum(tx0, tx1), np.maximum(ty0, ty1))
        hit = (tmax >= np.maximum(tmin, 0.0)) & (tmin > 0)
        t = np.where(hit, tmin, np.inf)
        idx = np.argmin(t, axis=0)
        tt = t[idx, np.arange(CAM_W)]
        z = tt * np.cos(phi)
        return z, idx, phi, dx, dy

    def _marker_bitmap(self):
        if self._marker_img is None:
            n = 120
            m = docking.generate_marker(self.cfg["docking"]["aruco_dict"], self.cfg["docking"]["marker_id"], n)
            pad = n // 6
            plate = np.full((n + 2 * pad, n + 2 * pad), 255, np.uint8)
            plate[pad:pad + n, pad:pad + n] = m
            self._marker_img = cv2.cvtColor(plate, cv2.COLOR_GRAY2BGR)
        return self._marker_img

    def render(self):
        """Return a Frame with simulated colour + depth. Depth is metres, 0 = invalid."""
        with self.lock:
            z, idx, phi, dx, dy = self._cast()
            boxes = self._all_boxes()
            color = np.zeros((CAM_H, CAM_W, 3), np.uint8)
            cy = int(self.intr.ppy)
            color[:cy] = (70, 60, 55)          # ceiling
            for r in range(cy, CAM_H):
                g = 40 + int(50 * (r - cy) / (CAM_H - cy))
                color[r] = (g, g + 5, g)       # floor gradient
            base = {"wall": (150, 140, 130), "shelf": (60, 130, 200), "obstacle": (40, 60, 220)}
            for u in range(CAM_W):
                zi = z[u]
                if not np.isfinite(zi):
                    continue
                top = int(cy - self.intr.fy * (WALL_H_M - CAM_HEIGHT_M) / zi)
                bot = int(cy + self.intr.fy * CAM_HEIGHT_M / zi)
                shade = max(0.25, min(1.0, 1.6 / (zi + 0.6)))
                col = np.array(base[boxes[idx[u]].kind], np.float64) * shade
                color[max(0, top):min(CAM_H, bot)+1, u] = col.astype(np.uint8)
            self._draw_marker(color, z, phi, dx, dy)
            depth = np.empty((CAM_H, CAM_W), np.float32)
            depth[:] = np.where(np.isfinite(z), z, 0.0).astype(np.float32)[None, :]
            if self.s["depth_noise_m"] > 0:
                depth += self.np_rng.normal(0, self.s["depth_noise_m"], depth.shape).astype(np.float32)
            depth[(depth < 0.3) | (depth > 8.0)] = 0.0
        return Frame(color, depth, self.intr, KIND_SIM, "SIMULATED camera (ray-cast, not real)")

    def _draw_marker(self, color, z, phi, dx, dy):
        d = self.dock
        n = np.array([math.cos(d["normal"]), math.sin(d["normal"])])
        h = -n                                           # direction a viewer in front of it looks
        right = np.array([h[1], -h[0]])
        c = np.array([d["x"], d["y"]])
        if np.dot(np.array([self.x, self.y]) - c, n) <= 0.05:
            return
        half = d["size"] / 2.0 * (4.0 / 3.0 * 1.0)       # plate (marker + white margin) = 1.333 x marker
        up_h = CAM_HEIGHT_M                              # marker centre at camera height
        corners3 = [(c - right * half, up_h + half), (c + right * half, up_h + half),
                    (c + right * half, up_h - half), (c - right * half, up_h - half)]
        f = np.array([math.cos(self.th), math.sin(self.th)])
        r = np.array([math.sin(self.th), -math.cos(self.th)])
        dst = []
        for p, zh in corners3:
            rel = p - np.array([self.x, self.y])
            Zc, Xc = float(np.dot(rel, f)), float(np.dot(rel, r))
            if Zc < 0.15:
                return
            dst.append([self.intr.fx * Xc / Zc + self.intr.ppx, self.intr.fy * (CAM_HEIGHT_M - zh) / Zc + self.intr.ppy])
        bm = self._marker_bitmap()
        hh, ww = bm.shape[:2]
        src = np.float32([[0, 0], [ww - 1, 0], [ww - 1, hh - 1], [0, hh - 1]])
        Hm = cv2.getPerspectiveTransform(src, np.float32(dst))
        warped = cv2.warpPerspective(bm, Hm, (CAM_W, CAM_H), flags=cv2.INTER_LINEAR, borderValue=(0, 0, 0))
        mask = cv2.warpPerspective(np.full((hh, ww), 255, np.uint8), Hm, (CAM_W, CAM_H), flags=cv2.INTER_NEAREST)
        # occlusion: marker plane depth along each column vs. what the ray actually hits
        with np.errstate(divide="ignore", invalid="ignore"):
            tm = np.dot(c - np.array([self.x, self.y]), n) / (dx * n[0] + dy * n[1])
        zm = tm * np.cos(phi)
        visible = np.isfinite(z) & (z >= zm - 0.06) & (tm > 0)
        mask[:, ~visible] = 0
        color[mask > 0] = warped[mask > 0]

    # ------------------------------------------------------------------ UI state
    def state(self):
        with self.lock:
            return {
                "is_sim": True,
                "world": {"w": self.W, "h": self.H, "shelves": [b.to_dict() for b in self.shelves],
                          "obstacles": [b.to_dict() for b in self.obstacles]},
                "dock": self.dock,
                "robot": {"x": round(self.x, 3), "y": round(self.y, 3), "th": round(self.th, 4),
                          "radius": self.s["robot_radius_m"], "moving": self.motion is not None,
                          "dir": None if self.motion is None else self.motion["dir"]},
                "est": {"x": round(self.ex, 3), "y": round(self.ey, 3), "th": round(self.eth, 4)},
                "trail": [(round(a, 2), round(b, 2)) for a, b in list(self.trail)[-600:]],
                "est_trail": [(round(a, 2), round(b, 2)) for a, b in list(self.est_trail)[-600:]],
                "collided": self.collided,
                "running": self.running,
                "event": self.last_event,
                "pulses": self.pulses,
                "enc": [int(c) for c in self.counts],
                "sim_time": round(self.sim_time, 1),
                "hfov_deg": HFOV_DEG,
            }
