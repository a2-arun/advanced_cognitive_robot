"""ArUco marker detection, distance estimation and the docking state machine.

SEARCH -> MARKER_DETECTED -> ALIGN -> APPROACH -> DOCKED

Distance comes from, in order of preference: (1) aligned depth at the marker centre, (2) solvePnP using the
printed marker size and camera intrinsics. If neither is available the distance is None and the FSM will NOT
approach (it never drives blindly). Nothing here talks to hardware.
"""
import cv2
import numpy as np

SEARCH = "SEARCH"
DETECTED = "MARKER_DETECTED"
ALIGN = "ALIGN"
APPROACH = "APPROACH"
DOCKED = "DOCKED"
ABORT = "ABORT"


class ArucoUnavailable(Exception):
    pass


def aruco_available():
    return hasattr(cv2, "aruco")


def _dictionary(name):
    if not aruco_available():
        raise ArucoUnavailable("this OpenCV build has no cv2.aruco (need opencv-contrib). See README troubleshooting.")
    code = getattr(cv2.aruco, name)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(code)
    return cv2.aruco.Dictionary_get(code)


def generate_marker(dict_name, marker_id, pixels=600):
    d = _dictionary(dict_name)
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(d, marker_id, pixels)
    return cv2.aruco.drawMarker(d, marker_id, pixels)


class MarkerObservation(object):
    def __init__(self):
        self.detected = False
        self.id = None
        self.corners = None          # 4x2 float
        self.center = None           # (x, y) px
        self.size_px = None
        self.offset = None           # -1 (left edge) .. +1 (right edge); 0 = image centre
        self.distance_m = None
        self.distance_src = None     # 'depth' | 'pnp' | None
        self.pnp_distance_m = None
        self.depth_distance_m = None
        self.other_ids = []

    def to_dict(self):
        r = lambda v, n=3: None if v is None else round(float(v), n)
        return {"detected": self.detected, "id": self.id, "offset": r(self.offset), "distance_m": r(self.distance_m),
                "distance_src": self.distance_src, "size_px": r(self.size_px, 1),
                "pnp_distance_m": r(self.pnp_distance_m), "depth_distance_m": r(self.depth_distance_m),
                "center": None if self.center is None else [round(float(self.center[0]), 1),
                                                            round(float(self.center[1]), 1)]}


class ArucoDetector(object):
    def __init__(self, dcfg, ocfg=None):
        self.c = dcfg
        self.ocfg = ocfg or {"min_range_m": 0.3, "max_range_m": 6.0}
        self.dict = _dictionary(dcfg["aruco_dict"])
        if hasattr(cv2.aruco, "ArucoDetector"):
            self._det = cv2.aruco.ArucoDetector(self.dict, cv2.aruco.DetectorParameters())
            self._detect = lambda g: self._det.detectMarkers(g)[:2]
        else:
            params = cv2.aruco.DetectorParameters_create()
            self._detect = lambda g: cv2.aruco.detectMarkers(g, self.dict, parameters=params)[:2]

    def observe(self, color, depth=None, intr=None):
        obs = MarkerObservation()
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY) if color.ndim == 3 else color
        corners, ids = self._detect(gray)
        if ids is None or len(ids) == 0:
            return obs
        ids = ids.flatten().tolist()
        obs.other_ids = [i for i in ids if i != self.c["marker_id"]]
        if self.c["marker_id"] not in ids:
            return obs
        quad = corners[ids.index(self.c["marker_id"])].reshape(4, 2).astype(np.float64)
        h, w = gray.shape[:2]
        obs.detected, obs.id, obs.corners = True, self.c["marker_id"], quad
        obs.center = quad.mean(axis=0)
        obs.size_px = float(np.mean([np.linalg.norm(quad[i] - quad[(i + 1) % 4]) for i in range(4)]))
        obs.offset = float((obs.center[0] - w / 2.0) / (w / 2.0))
        # (1) depth at the marker centre (median of valid pixels in a small patch)
        if depth is not None:
            dh, dw = depth.shape[:2]
            cx, cy = obs.center[0] * dw / float(w), obs.center[1] * dh / float(h)
            r = max(2, int(0.2 * obs.size_px * dw / float(w)))
            patch = depth[max(0, int(cy) - r):int(cy) + r + 1, max(0, int(cx) - r):int(cx) + r + 1]
            ok = patch[np.isfinite(patch) & (patch >= self.ocfg["min_range_m"]) & (patch <= 8.0)]
            if ok.size >= 6:
                obs.depth_distance_m = float(np.median(ok))
        # (2) PnP from known marker size + intrinsics
        if intr is not None and self.c.get("marker_size_m"):
            s = self.c["marker_size_m"] / 2.0
            obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)
            K = intr.scaled(w, h).matrix() if (intr.width != w or intr.height != h) else intr.matrix()
            try:
                flag = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
                ok, rvec, tvec = cv2.solvePnP(obj, quad, K, intr.dist(), flags=flag)
                if ok and tvec[2][0] > 0:
                    obs.pnp_distance_m = float(tvec[2][0])
            except cv2.error:
                pass
        if obs.depth_distance_m is not None:
            obs.distance_m, obs.distance_src = obs.depth_distance_m, "depth"
        elif obs.pnp_distance_m is not None:
            obs.distance_m, obs.distance_src = obs.pnp_distance_m, "pnp"
        return obs


class DockingFSM(object):
    def __init__(self, dcfg, mcfg):
        self.c, self.m = dcfg, mcfg
        self.reset()

    def reset(self):
        self.state = SEARCH
        self.t_state = 0.0
        self.seen = 0
        self.lost = 0
        self.centered = False
        self.docked_frames = 0
        self.actions_used = 0
        self.awaiting = False
        self.frames_after = 99
        self.settle_until = 0.0
        self.start_dist = None
        self.dist = None
        self.decision = "NO_MARKER"
        self.text = "searching for marker"
        self.search_ticks = 0
        self.align_ms = float(self.c["align_pulse_ms"])
        self._last_align = None      # (sign of offset, |offset|) before the last alignment pulse

    def _adapt_align(self, off):
        """Self-tuning pulse length (no PWM, no calibration): shorten after an overshoot, lengthen if the last
        pulse barely moved the marker."""
        if self._last_align is None:
            return
        sign0, mag0 = self._last_align
        self._last_align = None
        lo = float(self.m["min_pulse_ms"])
        hi = float(self.c.get("align_max_ms", 400))
        if off * sign0 < 0:                       # crossed the centre line: overshoot
            self.align_ms = max(lo, self.align_ms * 0.6)
        elif abs(off) > 0.7 * mag0:               # not much progress
            self.align_ms = min(hi, self.align_ms * 1.4)

    def _go(self, s, now):
        self.state, self.t_state = s, now

    def action_done(self, now):
        self.awaiting = False
        self.frames_after = 0
        self.settle_until = now + self.m["settle_s"]
        self.seen = 0       # re-confirm the marker with fresh frames after motion

    def progress(self):
        if self.state == DOCKED:
            return 1.0
        if self.dist is None or self.start_dist is None or self.start_dist <= self.c["dock_distance_m"]:
            return 0.0
        p = 1.0 - (self.dist - self.c["dock_distance_m"]) / (self.start_dist - self.c["dock_distance_m"])
        return float(max(0.0, min(1.0, p)))

    def update(self, obs, now, can_act):
        """obs: MarkerObservation (or any object with .detected/.offset/.distance_m). Returns a status dict."""
        c = self.c
        self.frames_after += 1
        if obs.detected:
            self.seen += 1
            self.lost = 0
            self.dist = obs.distance_m
            if obs.distance_m is not None and (self.start_dist is None or obs.distance_m > self.start_dist):
                self.start_dist = obs.distance_m
        else:
            self.lost += 1
            self.seen = 0
        action = None
        confirmed = obs.detected and self.seen >= c["confirm_frames"]
        ready = (not self.awaiting) and now >= self.settle_until and self.frames_after >= c["confirm_frames"]

        if self.state == DOCKED:
            self.decision, self.text = "CLOSE_ENOUGH", "docked (distance <= %.2f m)" % c["dock_distance_m"]
            return self._out(None)
        if self.state == ABORT:
            self.decision, self.text = "ABORT", "action limit reached - reset to retry"
            return self._out(None)

        if self.lost >= c["lost_frames"] and self.state != SEARCH:
            self._go(SEARCH, now)
            self.centered = False
        if not obs.detected:
            self.decision = "NO_MARKER"
            self.text = "marker not visible - holding still" if self.state != SEARCH else "searching for marker"
            if (self.state == SEARCH and can_act and c["search_pulse_ms"] > 0 and ready and self.lost >= 3):
                action = self._act("R", c["search_pulse_ms"], "search rotate")
            return self._out(action)

        # marker visible
        if self.state == SEARCH and confirmed:
            self._go(DETECTED, now)
        elif self.state == DETECTED and (now - self.t_state) >= 0.35 and confirmed:
            self._go(ALIGN, now)

        off = obs.offset
        if self.centered:
            self.centered = abs(off) < c["enter_align"]
        else:
            self.centered = abs(off) < c["deadband"]
        align_dir = "L" if off < 0 else "R"

        if self.state in (SEARCH, DETECTED):
            self.decision = "CENTERED" if self.centered else ("LEFT" if off < 0 else "RIGHT")
            self.text = "marker found, confirming (%d/%d)" % (min(self.seen, c["confirm_frames"]), c["confirm_frames"])
        elif self.state == ALIGN:
            if self.centered:
                self.decision, self.text = "CENTERED", "centered (offset %+.2f)" % off
                if confirmed and (now - self.t_state) >= 0.35:
                    self._go(APPROACH, now)
            else:
                self.decision = "LEFT" if off < 0 else "RIGHT"
                self.text = "marker %s of centre (offset %+.2f): turn %s" % (
                    "left" if off < 0 else "right", off, "LEFT" if off < 0 else "RIGHT")
                if can_act and ready and confirmed:
                    self._adapt_align(off)
                    action = self._act(align_dir, int(self.align_ms), "align")
                    if action:
                        self._last_align = (1 if off > 0 else -1, abs(off))
        elif self.state == APPROACH:
            if not self.centered:
                self._go(ALIGN, now)
                self.decision = "LEFT" if off < 0 else "RIGHT"
                self.text = "drifted off-centre, re-aligning"
            elif obs.distance_m is None:
                self.decision = "NO_DISTANCE"
                self.text = "no reliable distance (no depth, no intrinsics) - will NOT approach blindly"
            elif obs.distance_m <= c["dock_distance_m"]:
                self.docked_frames += 1
                self.decision, self.text = "CLOSE_ENOUGH", "at %.2f m <= %.2f m, confirming" % (
                    obs.distance_m, c["dock_distance_m"])
                if self.docked_frames >= c["confirm_frames"]:
                    self._go(DOCKED, now)
            else:
                self.docked_frames = 0
                self.decision = "TOO_FAR"
                self.text = "%.2f m to marker, target %.2f m: approach" % (obs.distance_m, c["dock_distance_m"])
                if can_act and ready and confirmed:
                    near = obs.distance_m - c["dock_distance_m"] < c.get("near_margin_m", 0.30)
                    ms = c.get("near_pulse_ms", 100) if near else c["approach_pulse_ms"]
                    action = self._act("F", ms, "approach")
        return self._out(action)

    def _act(self, d, ms, reason):
        if self.actions_used >= self.c["max_actions"]:
            self.state = ABORT
            return None
        self.actions_used += 1
        self.awaiting = True
        if reason != "align":
            self._last_align = None
        return {"kind": "move", "dir": d, "ms": ms, "reason": reason}

    def _out(self, action):
        return {"state": self.state, "decision": self.decision, "text": self.text, "action": action,
                "progress": round(self.progress(), 3), "actions_used": self.actions_used}


def annotate_docking(color, obs, st, dcfg, mode_text, kind_text):
    out = color.copy()
    h, w = out.shape[:2]
    cv2.line(out, (w // 2, 0), (w // 2, h), (200, 200, 200), 1)
    for k in (-dcfg["deadband"], dcfg["deadband"]):
        x = int(w / 2 + k * w / 2)
        cv2.line(out, (x, 0), (x, h), (0, 200, 0), 1)
    if obs.detected:
        pts = obs.corners.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(out, [pts], True, (0, 255, 0), 3)
        cx, cy = int(obs.center[0]), int(obs.center[1])
        cv2.circle(out, (cx, cy), 6, (0, 0, 255), -1)
        cv2.line(out, (w // 2, h // 2), (cx, cy), (0, 255, 255), 2)
        dist = "n/a" if obs.distance_m is None else "%.2f m (%s)" % (obs.distance_m, obs.distance_src)
        cv2.putText(out, "ID %s  offset %+.2f  dist %s" % (obs.id, obs.offset, dist), (8, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
    banner = (40, 120, 40) if obs.detected else (60, 60, 160)
    cv2.rectangle(out, (0, 0), (w, 30), banner, -1)
    cv2.putText(out, "MARKER DETECTED" if obs.detected else "NO MARKER", (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                (255, 255, 255), 2, cv2.LINE_AA)
    cv2.rectangle(out, (0, h - 52), (w, h), (30, 30, 30), -1)
    cv2.putText(out, "DOCK: %s | %s" % (st["state"], st["decision"]), (8, h - 32), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(out, "%s | %s" % (mode_text, kind_text), (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 255, 255), 1, cv2.LINE_AA)
    return out
