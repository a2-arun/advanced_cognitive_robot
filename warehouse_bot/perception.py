"""Depth-based obstacle detection + the avoidance decision state machine.

CLEAR -> OBSTACLE_DETECTED -> SELECT_DIRECTION -> TURN -> RECHECK -> CLEAR   (or STOP_BLOCKED / STOP_UNRELIABLE)

Nothing here moves a robot. The FSM *returns* an action; whoever owns the actuator (hardware link or simulator)
decides whether to execute it. With can_act=False it only reports what it WOULD do (perception mode).
"""
from collections import deque

import cv2
import numpy as np

# detector states
CLEAR = "CLEAR"
OBSTACLE = "OBSTACLE"
UNRELIABLE = "UNRELIABLE"
NO_DEPTH = "NO_DEPTH"
WARMUP = "WARMUP"       # window refilling after a flush; never treated as clear, never triggers a move


def roi_box(shape, roi):
    h, w = shape[:2]
    x0, x1, y0, y1 = roi
    return int(x0 * w), int(x1 * w), int(y0 * h), int(y1 * h)


def region_distance(depth, roi, ocfg):
    """Return (near_distance_m or None, valid_fraction). Beyond max_range counts as 'far', 0/NaN as invalid."""
    x0, x1, y0, y1 = roi_box(depth.shape, roi)
    patch = depth[y0:y1, x0:x1]
    if patch.size == 0:
        return None, 0.0
    ok = np.isfinite(patch) & (patch >= ocfg["min_range_m"])
    frac = float(ok.sum()) / patch.size
    if frac == 0.0:
        return None, 0.0
    vals = np.minimum(patch[ok], ocfg["max_range_m"])
    return float(np.percentile(vals, ocfg["percentile"])), frac


class ObstacleDetector(object):
    def __init__(self, ocfg):
        self.c = ocfg
        self.flush()

    def flush(self):
        n = self.c["smooth_frames"]
        self._front = deque(maxlen=n)
        self._left = deque(maxlen=n)
        self._right = deque(maxlen=n)
        self.state = CLEAR
        self._cand = None
        self._cand_n = 0
        self._seen = 0                 # frames processed since flush (window warm-up)

    def _smooth(self, q):
        vals = [v for v in q if v is not None]
        if len(vals) < max(1, (q.maxlen + 1) // 2):
            return None
        return float(np.median(vals))

    def process(self, depth):
        c = self.c
        if depth is None:
            self.state = NO_DEPTH
            return {"state": NO_DEPTH, "front_m": None, "left_m": None, "right_m": None,
                    "valid_front": 0.0, "reliable": False, "reason": "no depth stream - distance NOT estimated"}
        f, vf = region_distance(depth, c["roi_front"], c)
        l, vl = region_distance(depth, c["roi_left"], c)
        r, vr = region_distance(depth, c["roi_right"], c)
        self._front.append(f if vf >= c["min_valid_fraction"] else None)
        self._left.append(l if vl >= c["min_valid_fraction"] else None)
        self._right.append(r if vr >= c["min_valid_fraction"] else None)
        sf, sl, sr = self._smooth(self._front), self._smooth(self._left), self._smooth(self._right)
        self._seen += 1
        out = {"front_m": sf, "left_m": sl, "right_m": sr, "valid_front": round(vf, 2),
               "raw_front_m": f, "reliable": sf is not None}
        if sf is None and self._seen < c["smooth_frames"]:
            out.update(state=WARMUP, reason="filling smoothing window (%d/%d)" % (self._seen, c["smooth_frames"]))
            return out
        if sf is None:
            self.state = UNRELIABLE
            self._cand, self._cand_n = None, 0
            out.update(state=UNRELIABLE, reason="front depth too sparse (valid %.0f%% < %.0f%%)" %
                       (vf * 100, c["min_valid_fraction"] * 100))
            return out
        thr = c["stop_distance_m"] + (c["clear_hysteresis_m"] if self.state == OBSTACLE else 0.0)
        cand = OBSTACLE if sf < thr else CLEAR
        if self.state in (UNRELIABLE, NO_DEPTH):
            self.state = CLEAR          # depth is back; an obstacle must still be confirmed over several frames
        if cand == self.state:
            self._cand, self._cand_n = None, 0
        else:
            if cand == self._cand:
                self._cand_n += 1
            else:
                self._cand, self._cand_n = cand, 1
            if self._cand_n >= c["confirm_frames"]:
                self.state, self._cand, self._cand_n = cand, None, 0
        out.update(state=self.state, reason="front %.2f m (thr %.2f)" % (sf, thr))
        return out


class AvoidanceFSM(object):
    S_CLEAR = "CLEAR"
    S_DETECTED = "OBSTACLE_DETECTED"
    S_SELECT = "SELECT_DIRECTION"
    S_TURN = "TURN"
    S_RECHECK = "RECHECK"
    S_BLOCKED = "STOP_BLOCKED"
    S_UNREL = "STOP_UNRELIABLE"

    def __init__(self, ocfg, mcfg, detector, min_dwell_s=0.35):
        self.c, self.m, self.det = ocfg, mcfg, detector
        self.min_dwell = min_dwell_s
        self.reset()

    def reset(self):
        self.state = self.S_CLEAR
        self.t_state = 0.0
        self.turns = 0
        self.intended = None          # 'L' | 'R' | None
        self.awaiting = False         # an action was issued and not yet reported done
        self.frames_after = 0
        self.settle_until = 0.0
        self.last_text = "path clear"

    def _go(self, s, now, text=None):
        self.state, self.t_state = s, now
        if text:
            self.last_text = text

    def action_done(self, now):
        """Executor calls this when the issued pulse has finished."""
        self.awaiting = False
        self.det.flush()
        self.frames_after = 0
        self.settle_until = now + self.m["settle_s"]
        self._go(self.S_RECHECK, now, "re-checking after turn")

    def update(self, d, now, can_act):
        """d = ObstacleDetector.process() result. Returns {"state", "action", "intended", "text"}."""
        action = None
        ds = d["state"]
        if self.awaiting:
            return self._out(None)
        if ds == WARMUP:                       # hold state, issue nothing, do not count toward RECHECK
            return self._out(None)
        if ds in (UNRELIABLE, NO_DEPTH):
            if self.state != self.S_UNREL:
                self._go(self.S_UNREL, now)
            self.last_text = "STOP: " + d.get("reason", "depth unreliable")
            self.intended = None
            self.turns = 0
            return self._out({"kind": "stop"} if can_act else None)
        if self.state == self.S_UNREL:
            self._go(self.S_CLEAR, now, "depth recovered")

        dwell = (now - self.t_state) >= self.min_dwell
        if self.state == self.S_CLEAR:
            self.intended = None
            self.last_text = "path clear (front %.2f m)" % d["front_m"]
            if ds == OBSTACLE:
                self._go(self.S_DETECTED, now, "obstacle ahead at %.2f m" % d["front_m"])
            elif can_act and self.c["advance_when_clear"] and now >= self.settle_until:
                action = {"kind": "move", "dir": "F", "ms": self.c["advance_pulse_ms"], "reason": "path clear"}
                self.awaiting = True
        elif self.state == self.S_DETECTED:
            if ds == CLEAR:
                self._go(self.S_CLEAR, now)
            elif dwell:
                self._go(self.S_SELECT, now, "choosing side...")
        elif self.state == self.S_SELECT:
            if ds == CLEAR:
                self._go(self.S_CLEAR, now)
            elif dwell:
                choice = self._choose(d)
                if choice is None:
                    self._go(self.S_BLOCKED, now, "STOP: no side has %.1f m clearance (L=%s R=%s)" %
                             (self.c["side_clear_m"], _fmt(d["left_m"]), _fmt(d["right_m"])))
                else:
                    self.intended = choice
                    self._go(self.S_TURN, now, "turn %s (L=%s R=%s)" %
                             ("LEFT" if choice == "L" else "RIGHT", _fmt(d["left_m"]), _fmt(d["right_m"])))
        elif self.state == self.S_TURN:
            if ds == CLEAR:
                self._go(self.S_CLEAR, now)
            elif can_act and dwell:
                action = {"kind": "move", "dir": self.intended, "ms": self.c["turn_pulse_ms"],
                          "reason": "avoid obstacle"}
                self.awaiting = True
                self.turns += 1
        elif self.state == self.S_RECHECK:
            self.frames_after += 1
            if now >= self.settle_until and self.frames_after >= self.c["recheck_frames"]:
                if ds == CLEAR:
                    self.turns = 0
                    self._go(self.S_CLEAR, now, "clear after turn")
                elif self.turns >= self.c["max_consecutive_turns"]:
                    self._go(self.S_BLOCKED, now, "STOP: still blocked after %d turns" % self.turns)
                else:
                    self._go(self.S_SELECT, now, "still blocked, choosing again")
        elif self.state == self.S_BLOCKED:
            if ds == CLEAR:
                self.turns = 0
                self._go(self.S_CLEAR, now, "obstacle gone")
        return self._out(action)

    def _choose(self, d):
        need = self.c["side_clear_m"]
        l, r = d["left_m"], d["right_m"]
        lok = l is not None and l >= need
        rok = r is not None and r >= need
        if lok and rok:
            return "L" if l >= r else "R"
        if lok:
            return "L"
        if rok:
            return "R"
        return None

    def _out(self, action):
        return {"state": self.state, "action": action, "intended": self.intended, "text": self.last_text,
                "turns": self.turns}


def _fmt(v):
    return "n/a" if v is None else "%.2f m" % v


def colorize_depth(depth, max_range=4.0):
    """Depth (metres) -> BGR visual. Invalid (0) pixels are black, never filled in."""
    img = np.zeros(depth.shape[:2] + (3,), np.uint8)
    ok = np.isfinite(depth) & (depth > 0)
    norm = np.clip(depth / max_range, 0, 1)
    cm = cv2.applyColorMap((255 - norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    img[ok] = cm[ok]
    return img


def annotate_obstacle(color, det, fsm, ocfg, mode_text, kind_text):
    """Draw ROIs and decision on a copy of the colour image."""
    out = color.copy()
    h, w = out.shape[:2]
    cols = {"front": (0, 200, 255), "left": (255, 200, 0), "right": (255, 200, 0)}
    if det.get("state") == OBSTACLE:
        cols["front"] = (0, 0, 255)
    for name in ("front", "left", "right"):
        x0, x1, y0, y1 = roi_box(out.shape, ocfg["roi_" + name])
        cv2.rectangle(out, (x0, y0), (x1, y1), cols[name], 2)
        v = det.get(name + "_m")
        cv2.putText(out, "%s %s" % (name.upper(), _fmt(v)), (x0 + 3, y0 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    cols[name], 1, cv2.LINE_AA)
    banner = {"WARMUP": ((90, 90, 90), "SETTLING..."), "CLEAR": ((60, 160, 60), "PATH CLEAR"), "OBSTACLE": ((0, 0, 200), "OBSTACLE AHEAD"),
              "UNRELIABLE": ((0, 140, 255), "DEPTH UNRELIABLE - STOP"), "NO_DEPTH": ((0, 140, 255), "NO DEPTH DATA")}
    col, txt = banner.get(det.get("state"), ((90, 90, 90), "?"))
    cv2.rectangle(out, (0, 0), (w, 30), col, -1)
    cv2.putText(out, txt, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
    intent = fsm["intended"]
    dtxt = "DECISION: %s" % fsm["state"]
    if intent:
        dtxt += " -> %s" % ("LEFT" if intent == "L" else "RIGHT")
    cv2.rectangle(out, (0, h - 52), (w, h), (30, 30, 30), -1)
    cv2.putText(out, dtxt, (8, h - 32), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(out, "%s | %s" % (mode_text, kind_text), (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 255, 255), 1, cv2.LINE_AA)
    return out
