"""Engine: owns the camera source, perception, state machines, the simulator and the hardware link, and enforces
the safety rules in ONE place:

  * mode is 'sim' or 'hardware'. Motion commands go ONLY to the actuator of the current mode. The simulator can
    never reach the serial port, and teleop/autonomy never reach the simulator in hardware mode.
  * autonomous actions are executed only when `act` is True. In hardware that additionally needs a healthy, ARMED
    link. `act` is cleared on STOP, on mode change and whenever the link goes down.
  * teleop is refused while a behaviour is active (no two controllers at once).
"""
import json
import math
import os
import threading
import time
import traceback
from collections import deque

import cv2
import numpy as np

from . import camera as cam
from . import docking as dk
from . import perception as pc
from . import pointcloud as pcl
from .odometry import Odometry
from .serial_robot import RobotLink
from .sim import Sim, MARKER_SIM_SIZE_M

BEHAVIORS = ("off", "avoid", "dock", "mission")


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def placeholder(text, sub="", w=640, h=480):
    img = np.full((h, w, 3), 35, np.uint8)
    cv2.putText(img, text, (20, h // 2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2, cv2.LINE_AA)
    if sub:
        cv2.putText(img, sub, (20, h // 2 + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
    return img


def jpeg(img, q=72):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), q])
    return buf.tobytes() if ok else b""


class Engine(object):
    def __init__(self, cfg, results_dir=None):
        self.cfg = cfg
        self.results_dir = results_dir or cfg["server"]["results_dir"]
        self.lock = threading.RLock()
        self.mode = "sim"
        self.sim = Sim(cfg)
        self.link = RobotLink(cfg)
        self.odo = Odometry(cfg["odometry"])
        self.source = None
        self.source_msg = "no source opened"
        self._source_busy = False

        self.ocfg = dict(cfg["obstacle"])
        self.det = pc.ObstacleDetector(self.ocfg)
        self.avoid = pc.AvoidanceFSM(self.ocfg, cfg["motion"], self.det)
        self.mcfg = cfg["motion"]
        self.aruco = None
        self.aruco_err = None
        self._apply_dock_cfg()

        self.behavior = "off"
        self.act = False
        self.inflight = None
        self.settle_until = 0.0
        self.mission = {"stage": "IDLE", "waypoint": None, "text": "no mission", "hold": 0}
        self.history = deque(maxlen=60)
        self.frame = None
        self.percep = {}
        self.jpgs = {"obstacle": b"", "depth": b"", "dock": b""}
        self.jpg_seq = 0
        self.jpg_cv = threading.Condition()
        self.recorders = {}
        self.fps = 0.0
        self._fcount, self._ftime = 0, time.time()
        self._run = False
        self._acc = 0.0
        self._last_states = {}
        os.makedirs(os.path.join(self.results_dir, "logs"), exist_ok=True)
        self.logpath = os.path.join(self.results_dir, "logs", "session_%s.jsonl" % time.strftime("%Y%m%d_%H%M%S"))
        self._log("start", mode=self.mode)

    # ------------------------------------------------------------------ helpers
    def _apply_dock_cfg(self):
        """Docking parameters per mode. The simulator has its own marker size and (slower) kinematics."""
        d = dict(self.cfg["docking"])
        if self.mode == "sim":
            d.update(marker_size_m=MARKER_SIM_SIZE_M, dock_distance_m=0.6, approach_pulse_ms=600, max_actions=80)
        self.dcfg = d
        self._make_aruco(d)
        self.dock = dk.DockingFSM(d, self.mcfg)

    def _make_aruco(self, dcfg):
        try:
            self.aruco = dk.ArucoDetector(dcfg, self.cfg["obstacle"])
            self.aruco_err = None
        except Exception as e:
            self.aruco, self.aruco_err = None, str(e)

    def _log(self, kind, **kw):
        rec = dict(t=round(time.time(), 3), kind=kind, mode=getattr(self, "mode", None))
        rec.update(kw)
        try:
            with open(self.logpath, "a") as f:
                f.write(json.dumps(rec, default=str) + "\n")
        except Exception:
            pass

    def _hist(self, src, text, ok=True):
        self.history.append({"t": time.strftime("%H:%M:%S"), "src": src, "text": text, "ok": ok})
        self._log("command", src=src, text=text, ok=ok)

    def _actuator(self):
        return self.sim if self.mode == "sim" else self.link

    def _hw_ready(self):
        return self.link.link_ok and self.link.armed

    def _motion_allowed(self):
        if self.mode == "sim":
            return self.sim.running
        return self._hw_ready()

    # ------------------------------------------------------------------ lifecycle
    def start(self):
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()

    def shutdown(self):
        self._run = False
        try:
            self.sim.stop()
            self.link.stop()
            self.link.disconnect()
        except Exception:
            pass
        if self.source:
            self.source.close()
        for w in self.recorders.values():
            w.release()

    # ------------------------------------------------------------------ user-level commands
    def stop_all(self, reason="STOP button"):
        """Always works. Stops whichever actuator is live (and, defensively, both)."""
        self.sim.stop()
        if self.link.status()["connected"]:
            self.link.stop()
        self.behavior, self.act, self.inflight = "off", False, None
        self.avoid.reset()
        self.dock.reset()
        if self.mission["stage"] not in ("IDLE", "DONE", "FAILED"):
            self.mission.update(stage="IDLE", text="mission cancelled by STOP", hold=0)
        self._hist("UI", "STOP (%s)" % reason)
        return {"ok": True}

    def set_mode(self, mode):
        if mode not in ("sim", "hardware"):
            return {"ok": False, "error": "mode must be sim|hardware"}
        if mode == self.mode:
            return {"ok": True}
        self.stop_all("mode change")
        if self.link.status()["connected"]:
            self.link.disarm()
        self.mode = mode
        self._apply_dock_cfg()
        self.avoid.reset()
        self.det.flush()
        if mode == "hardware":
            self.sim.running = False
            self.open_source(self.cfg["camera"]["source"])
        else:
            self.sim.running = True
            self._close_source()
        self._hist("UI", "mode -> %s" % mode.upper())
        return {"ok": True}

    def teleop(self, direction, ms=None):
        if self.behavior != "off":
            self._hist("UI", "teleop %s REFUSED: autonomy '%s' active" % (direction, self.behavior), False)
            return {"ok": False, "error": "autonomous behaviour active - press STOP first"}
        if self.mode == "sim" and not self.sim.running:
            return {"ok": False, "error": "simulation paused"}
        r = self._actuator().move(direction, ms)
        self._hist("UI", "%s %s %s ms%s" % ("SIM" if self.mode == "sim" else "HW", direction, r.get("ms", ms),
                                           "" if r.get("ok") else " -> " + str(r.get("error"))), bool(r.get("ok")))
        return r

    def connect(self, port=None):
        if self.mode != "hardware":
            return {"ok": False, "error": "switch to HARDWARE mode first"}
        try:
            self.link.connect(port or self.cfg["serial"]["port"])
        except Exception as e:
            self._hist("HW", "connect failed: %s" % e, False)
            return {"ok": False, "error": str(e)}
        self.odo.reset()
        self._hist("HW", "connected %s" % self.link.port)
        return {"ok": True, "status": self.link.status()}

    def disconnect(self):
        self.stop_all("disconnect")
        self.link.disconnect()
        return {"ok": True}

    def arm(self):
        if self.mode != "hardware":
            return {"ok": False, "error": "hardware mode only"}
        r = self.link.arm()
        self._hist("HW", "ARM -> %s" % ("ok" if r["ok"] else r["error"]), r["ok"])
        return r

    def disarm(self):
        self.act = False
        r = self.link.disarm() if self.link.status()["connected"] else {"ok": True}
        self._hist("HW", "DISARM")
        return r

    def set_behavior(self, behavior, act=False, advance=None):
        if behavior not in BEHAVIORS:
            return {"ok": False, "error": "unknown behaviour"}
        if behavior == "mission" and self.mode != "sim":
            return {"ok": False, "error": "missions need the simulator's pose; hardware has no localisation yet"}
        if act and self.mode == "hardware" and not self._hw_ready():
            return {"ok": False, "error": "hardware not connected+armed; staying in perception-only"}
        if advance is not None:
            self.ocfg["advance_when_clear"] = bool(advance)
        self.sim.stop() if self.mode == "sim" else self.link.stop()
        self.inflight = None
        self.avoid.reset()
        self.dock.reset()
        self.behavior, self.act = behavior, bool(act) and behavior != "off"
        self._hist("UI", "behaviour=%s %s" % (behavior, "ACTING" if self.act else "perception-only"))
        return {"ok": True}

    def sim_control(self, action, x=None, y=None):
        if action == "pause":
            self.sim.running = False
            self.sim.stop()
        elif action == "resume":
            if self.mode == "sim":
                self.sim.running = True
        elif action == "step":
            if self.mode == "sim":
                self.sim.step_once(0.1)
        elif action == "reset":
            self.stop_all("sim reset")
            self.sim.reset()
            self.mission.update(stage="IDLE", waypoint=None, text="no mission", hold=0)
            if self.mode == "sim":
                self.sim.running = True
        elif action == "toggle_obstacle" and x is not None:
            if not self.sim.remove_obstacle_at(x, y):
                if self.sim.add_obstacle(x, y) is None:
                    return {"ok": False, "error": "too close to the robot"}
        else:
            return {"ok": False, "error": "bad sim action"}
        self._hist("SIM", "sim %s" % action)
        return {"ok": True}

    def start_mission(self, x=None, y=None):
        if self.mode != "sim":
            return {"ok": False, "error": "mission is simulation-only"}
        if x is None:
            x, y = self.sim.dock["x"] - 2.4, self.sim.dock["y"]
        self.set_behavior("mission", act=True)
        self.mission.update(stage="NAVIGATE", waypoint=[round(x, 2), round(y, 2)], hold=0, text="navigating to waypoint")
        self._hist("UI", "mission: waypoint (%.1f, %.1f) then dock" % (x, y))
        return {"ok": True}

    # ------------------------------------------------------------------ camera
    def open_source(self, spec):
        if self._source_busy:
            return {"ok": False, "error": "camera is being opened"}
        self._close_source()
        self._source_busy = True
        self.source_msg = "opening camera (%s)..." % spec

        def work():
            try:
                src, msg = cam.open_source(spec, self.cfg)
                self.source, self.source_msg = src, msg
                self._hist("CAM", msg, src is not None)
            finally:
                self._source_busy = False
        threading.Thread(target=work, daemon=True).start()
        return {"ok": True}

    def _close_source(self):
        s, self.source = self.source, None
        if s:
            try:
                s.close()
            except Exception:
                pass

    # ------------------------------------------------------------------ main loop
    def _loop(self):
        last = time.time()
        while self._run:
            now = time.time()
            dt = min(now - last, 0.2)
            last = now
            try:
                self._watch_link()
                if self.mode == "sim":
                    self.sim.tick(dt)
                    self._check_inflight(now)
                    self._acc += dt
                    if self._acc < 0.1:
                        time.sleep(0.01)
                        continue
                    self._acc = 0.0
                    frame = self.sim.render()
                else:
                    self._check_inflight(now)
                    frame = self._read_frame()
                self._process(frame, now)
            except Exception:
                self._log("error", trace=traceback.format_exc())
                traceback.print_exc()
                time.sleep(0.5)

    def _read_frame(self):
        s = self.source
        if s is None:
            time.sleep(0.1)
            return None
        try:
            f = s.read()
        except Exception as e:
            self.source_msg = "camera read failed: %s" % e
            time.sleep(0.2)
            return None
        if f is None:
            time.sleep(0.05)
        return f

    def _watch_link(self):
        if self.mode == "hardware" and self.link.status()["connected"]:
            if (self.act or self.behavior != "off") and not self.link.link_ok:
                self.behavior, self.act, self.inflight = "off", False, None
                self._hist("HW", "link lost: autonomy disabled", False)
            if self.act and not self.link.armed:
                self.act = False
                self._hist("HW", "disarmed: autonomy now perception-only", False)
            self.odo.update(self.link.enc)

    # ------------------------------------------------------------------ pulse bookkeeping
    def _exec(self, action, owner):
        now = time.time()
        act = self._actuator()
        if action["kind"] == "stop":
            act.stop()
            return
        r = act.move(action["dir"], action["ms"])
        self._hist("AUTO", "%s %s %sms (%s)%s" % ("SIM" if self.mode == "sim" else "HW", action["dir"], action["ms"],
                                                action.get("reason", ""), "" if r.get("ok") else " REFUSED " +
                                                str(r.get("error"))), bool(r.get("ok")))
        if r.get("ok"):
            self.inflight = {"owner": owner, "start": now, "ms": r.get("ms", action["ms"]), "dir": action["dir"]}
        elif owner is not None:
            owner.action_done(now)

    def _check_inflight(self, now):
        f = self.inflight
        if not f:
            return
        act = self._actuator()
        margin = 0.0 if self.mode == "sim" else 0.25
        timed_out = now > f["start"] + f["ms"] / 1000.0 + 2.0
        if (now >= f["start"] + f["ms"] / 1000.0 + margin and not act.is_busy()) or timed_out:
            self.inflight = None
            self.settle_until = now + self.mcfg["settle_s"]
            if f["dir"] in ("L", "R") and f["owner"] is self.avoid and self.behavior == "mission":
                self.mission["hold"] = 5
            if f["owner"] is not None:
                f["owner"].action_done(now)

    # ------------------------------------------------------------------ perception cycle
    def _process(self, frame, now):
        self.frame = frame
        depth = frame.depth if frame else None
        color = frame.color if frame else None
        kind_text = frame.kind.upper() if frame else "NO CAMERA"
        label = frame.label if frame else self.source_msg
        acting = self.act and self._motion_allowed()
        mode_text = ("ACTING" if acting else "PERCEPTION ONLY") + (" | SIM" if self.mode == "sim" else " | HARDWARE")

        det = self.det.process(depth)
        mstage = self.mission["stage"]
        avoid_can = acting and (self.behavior == "avoid" or (self.behavior == "mission" and mstage == "NAVIGATE"))
        dock_can = acting and (self.behavior == "dock" or (self.behavior == "mission" and mstage == "DOCK"))

        if frame is not None and self.aruco is not None:
            obs = self.aruco.observe(color, depth, frame.intrinsics)
        else:
            obs = dk.MarkerObservation()

        avoid_out = self.avoid.update(det, now, avoid_can)
        dock_out = self.dock.update(obs, now, dock_can)

        if avoid_can and avoid_out["action"] and not self.inflight:
            self._exec(avoid_out["action"], self.avoid)
        elif avoid_can and avoid_out["action"]:
            self.avoid.action_done(now)
        if dock_can and dock_out["action"] and not self.inflight:
            self._exec(dock_out["action"], self.dock)
        elif dock_can and dock_out["action"]:
            self.dock.action_done(now)
        if self.behavior == "mission" and acting:
            self._mission_step(now)

        # ---- annotated views
        if color is not None:
            v_obs = pc.annotate_obstacle(color, det, avoid_out, self.ocfg, mode_text, "%s | %s" % (kind_text, label[:46]))
            v_dock = dk.annotate_docking(color, obs, dock_out, self.dcfg, mode_text, kind_text) \
                if self.aruco is not None else placeholder("ARUCO UNAVAILABLE", self.aruco_err or "")
            v_depth = pc.colorize_depth(depth, self.ocfg["max_range_m"]) if depth is not None else \
                placeholder("NO DEPTH STREAM", "distance is NOT estimated without depth")
            if depth is not None:
                cv2.putText(v_depth, kind_text + " DEPTH", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2,
                            cv2.LINE_AA)
        else:
            v_obs = placeholder("NO CAMERA", self.source_msg)
            v_dock = placeholder("NO CAMERA", self.source_msg)
            v_depth = placeholder("NO DEPTH STREAM", self.source_msg)
        with self.jpg_cv:
            self.jpgs = {"obstacle": jpeg(v_obs), "depth": jpeg(v_depth), "dock": jpeg(v_dock)}
            self.jpg_seq += 1
            self.jpg_cv.notify_all()
        for name, img in (("obstacle", v_obs), ("dock", v_dock), ("depth", v_depth)):
            w = self.recorders.get(name)
            if w is not None:
                w.write(cv2.resize(img, (640, 480)))

        self.percep = {"det": det, "avoid": avoid_out, "obs": obs.to_dict(), "dock": dock_out, "kind": kind_text,
                       "label": label, "acting": acting, "has_depth": depth is not None}
        for key, val in (("avoid", avoid_out["state"]), ("dock", dock_out["state"]), ("obstacle_det", det["state"])):
            if self._last_states.get(key) != val:
                self._last_states[key] = val
                self._log("state", fsm=key, state=val, avoid_text=avoid_out["text"], dock_text=dock_out["text"],
                          front_m=det.get("front_m"), obs=obs.to_dict())
        self._fcount += 1
        if now - self._ftime >= 1.0:
            self.fps = self._fcount / (now - self._ftime)
            self._fcount, self._ftime = 0, now

    # ------------------------------------------------------------------ mission (simulation only: uses sim pose)
    def _mission_step(self, now):
        m, s = self.mission, self.sim
        stage = m["stage"]
        if stage == "NAVIGATE":
            if self.inflight or now < self.settle_until or self.avoid.state != "CLEAR" or self.avoid.awaiting:
                m["text"] = "obstacle avoidance in control" if self.avoid.state != "CLEAR" else m["text"]
                return
            wx, wy = m["waypoint"]
            dx, dy = wx - s.x, wy - s.y
            dist = math.hypot(dx, dy)
            if dist < 0.30:
                m.update(stage="FACE_DOCK", text="arrived; facing dock")
                return
            if m["hold"] > 0:
                m["hold"] -= 1
                m["text"] = "clearing obstacle (forward hold %d)" % m["hold"]
                self._exec({"kind": "move", "dir": "F", "ms": 400, "reason": "clear obstacle"}, None)
                return
            err = _wrap(math.atan2(dy, dx) - s.th)
            if abs(err) > math.radians(10):
                ms = int(max(100, min(500, abs(math.degrees(err)) / self.cfg["sim"]["turn_dps"] * 1000 * 0.8)))
                m["text"] = "turning toward waypoint (%.0f deg off)" % math.degrees(err)
                self._exec({"kind": "move", "dir": "L" if err > 0 else "R", "ms": ms, "reason": "goto turn"}, None)
            else:
                ms = int(max(150, min(600, dist / self.cfg["sim"]["speed_mps"] * 1000 * 0.8)))
                m["text"] = "driving to waypoint (%.2f m)" % dist
                self._exec({"kind": "move", "dir": "F", "ms": ms, "reason": "goto"}, None)
        elif stage == "FACE_DOCK":
            if self.inflight or now < self.settle_until:
                return
            err = _wrap(math.atan2(s.dock["y"] - s.y, s.dock["x"] - s.x) - s.th)
            if abs(err) > math.radians(8):
                ms = int(max(100, min(400, abs(math.degrees(err)) / self.cfg["sim"]["turn_dps"] * 1000 * 0.8)))
                self._exec({"kind": "move", "dir": "L" if err > 0 else "R", "ms": ms, "reason": "face dock"}, None)
            else:
                self.dock.reset()
                m.update(stage="DOCK", text="docking with ArUco marker")
        elif stage == "DOCK":
            m["text"] = "docking: %s" % self.dock.state
            if self.dock.state == dk.DOCKED:
                m.update(stage="DONE", text="mission complete: docked")
                self._hist("AUTO", "mission complete: docked")
            elif self.dock.state == dk.ABORT:
                m.update(stage="FAILED", text="docking aborted (action limit)")
            elif self.dock.state == dk.SEARCH and not self.inflight and now >= self.settle_until and \
                    self.dock.lost >= 3 and not self.dock.awaiting:
                self._exec({"kind": "move", "dir": "L", "ms": 200, "reason": "search for marker"}, None)

    # ------------------------------------------------------------------ outputs
    def get_state(self):
        with self.lock:
            hw = self.link.status()
            p = self.percep
            return {
                "mode": self.mode,
                "data_label": ("SIMULATED" if self.mode == "sim" else (p.get("kind") or "NO CAMERA")),
                "behavior": self.behavior,
                "act": self.act and self._motion_allowed(),
                "advance_when_clear": self.ocfg["advance_when_clear"],
                "hw": hw,
                "hw_can_move": self.mode == "hardware" and self._hw_ready(),
                "odometry": self.odo.state(),
                "sim": self.sim.state() if self.mode == "sim" else {"is_sim": True, "running": False,
                                                                     "inactive": True},
                "camera": {"label": p.get("label", self.source_msg), "kind": p.get("kind"),
                           "message": self.source_msg, "has_depth": p.get("has_depth", False),
                           "fps": round(self.fps, 1)},
                "obstacle": {"det": p.get("det"), "fsm": p.get("avoid")},
                "docking": {"obs": p.get("obs"), "fsm": p.get("dock"), "marker_size_m": self.dcfg["marker_size_m"],
                            "dock_distance_m": self.dcfg["dock_distance_m"], "aruco_error": self.aruco_err},
                "mission": self.mission,
                "history": list(self.history)[-30:],
                "recording": sorted(self.recorders.keys()),
                "cfg": {"pulse_ms": self.mcfg["default_pulse_ms"], "min_pulse": self.mcfg["min_pulse_ms"],
                        "max_pulse": self.mcfg["max_pulse_ms"], "stop_distance_m": self.ocfg["stop_distance_m"]},
                "time": time.strftime("%H:%M:%S"),
            }

    def pointcloud(self, save=False):
        f = self.frame
        if f is None or f.depth is None or f.intrinsics is None:
            return {"ok": False, "error": "no depth frame available - nothing to project (no invented points)"}
        pts, cols = pcl.depth_to_points(f.depth, f.intrinsics, f.color, stride=6, zmin=self.ocfg["min_range_m"],
                                        zmax=self.ocfg["max_range_m"])
        out = {"ok": True, "n": int(len(pts)), "kind": f.kind.upper(), "label": f.label,
               "note": "single-frame camera point cloud (camera frame). Not a map, not SLAM.",
               "points": np.round(pts, 3).tolist(),
               "colors": [] if cols is None else cols.tolist()}
        if save:
            d = os.path.join(self.results_dir, "pointclouds")
            os.makedirs(d, exist_ok=True)
            path = os.path.join(d, "cloud_%s_%s.ply" % (f.kind, time.strftime("%Y%m%d_%H%M%S")))
            pcl.save_ply(path, pts, cols, "%s | single-frame camera point cloud, not SLAM" % f.label)
            out["saved"] = path
            self._hist("UI", "point cloud saved: %s" % path)
        return out

    def snapshot(self):
        d = os.path.join(self.results_dir, "screenshots")
        os.makedirs(d, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        saved = []
        for name, data in self.jpgs.items():
            if data:
                p = os.path.join(d, "%s_%s_%s.jpg" % (stamp, self.mode, name))
                with open(p, "wb") as f:
                    f.write(data)
                saved.append(p)
        p = os.path.join(d, "%s_%s_state.json" % (stamp, self.mode))
        with open(p, "w") as f:
            json.dump(self.get_state(), f, indent=1, default=str)
        saved.append(p)
        self._hist("UI", "snapshot saved (%d files) in %s" % (len(saved), d))
        return {"ok": True, "files": saved}

    def recording(self, action, name):
        if name not in ("obstacle", "dock", "depth"):
            return {"ok": False, "error": "bad stream"}
        if action == "start" and name not in self.recorders:
            d = os.path.join(self.results_dir, "videos")
            os.makedirs(d, exist_ok=True)
            path = os.path.join(d, "%s_%s_%s.mp4" % (time.strftime("%Y%m%d_%H%M%S"), self.mode, name))
            w = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), 10.0, (640, 480))
            if not w.isOpened():
                return {"ok": False, "error": "cannot open video writer"}
            self.recorders[name] = w
            self._hist("UI", "recording %s -> %s" % (name, path))
            return {"ok": True, "path": path}
        if action == "stop" and name in self.recorders:
            self.recorders.pop(name).release()
            self._hist("UI", "recording %s stopped" % name)
        return {"ok": True}
