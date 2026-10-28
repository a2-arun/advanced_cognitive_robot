"""Frame sources. Every Frame says honestly where it came from (live / recorded / simulated).

Depth is always float32 METRES with 0 = "no valid measurement". Sources without depth return depth=None and the
rest of the system refuses to invent distances.
"""
import glob
import json
import os
import time

import cv2
import numpy as np

KIND_LIVE = "live"
KIND_RECORDED = "recorded"
KIND_SIM = "simulated"


class Intrinsics(object):
    def __init__(self, width, height, fx, fy, ppx, ppy, coeffs=None):
        self.width, self.height = width, height
        self.fx, self.fy, self.ppx, self.ppy = fx, fy, ppx, ppy
        self.coeffs = list(coeffs) if coeffs is not None else [0.0] * 5

    def matrix(self):
        return np.array([[self.fx, 0, self.ppx], [0, self.fy, self.ppy], [0, 0, 1]], dtype=np.float64)

    def dist(self):
        return np.array(self.coeffs, dtype=np.float64)

    def scaled(self, w, h):
        sx, sy = float(w) / self.width, float(h) / self.height
        return Intrinsics(w, h, self.fx * sx, self.fy * sy, self.ppx * sx, self.ppy * sy, self.coeffs)

    def to_dict(self):
        return dict(width=self.width, height=self.height, fx=self.fx, fy=self.fy, ppx=self.ppx, ppy=self.ppy,
                    coeffs=self.coeffs)

    @staticmethod
    def from_dict(d):
        return Intrinsics(d["width"], d["height"], d["fx"], d["fy"], d["ppx"], d["ppy"], d.get("coeffs"))


class Frame(object):
    def __init__(self, color, depth, intrinsics, kind, label, ts=None):
        self.color = color              # BGR uint8
        self.depth = depth              # float32 metres or None
        self.intrinsics = intrinsics    # Intrinsics for color (depth is aligned to color) or None
        self.kind = kind
        self.label = label
        self.ts = ts if ts is not None else time.time()


class SourceError(Exception):
    pass


class RealSenseSource(object):
    kind = KIND_LIVE

    def __init__(self, width=640, height=480, fps=15):
        try:
            import pyrealsense2 as rs
        except ImportError:
            raise SourceError("pyrealsense2 is not importable in this Python (see README: RealSense on Jetson)")
        self.rs = rs
        ctx = rs.context()
        devs = ctx.query_devices()
        if len(devs) == 0:
            raise SourceError("no RealSense device detected (USB3 cable? borrowed camera returned?)")
        self.name = devs[0].get_info(rs.camera_info.name)
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        try:
            profile = self.pipe.start(cfg)
        except Exception as e:
            raise SourceError("RealSense pipeline failed to start at %dx%d@%d: %s" % (width, height, fps, e))
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)
        i = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intr = Intrinsics(i.width, i.height, i.fx, i.fy, i.ppx, i.ppy, list(i.coeffs))
        self.label = "LIVE %s" % self.name

    def read(self):
        frames = self.pipe.wait_for_frames(3000)
        frames = self.align.process(frames)
        d, c = frames.get_depth_frame(), frames.get_color_frame()
        if not d or not c:
            return None
        depth = np.asanyarray(d.get_data()).astype(np.float32) * self.depth_scale
        return Frame(np.asanyarray(c.get_data()).copy(), depth, self.intr, KIND_LIVE, self.label)

    def close(self):
        try:
            self.pipe.stop()
        except Exception:
            pass


class WebcamSource(object):
    """Plain RGB camera (laptop webcam, or the D455 colour stream exposed as /dev/video*). NO depth."""
    kind = KIND_LIVE

    def __init__(self, index=0, width=640, height=480):
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise SourceError("cannot open webcam index %s" % index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.label = "LIVE webcam #%s (RGB only, no depth)" % index

    def read(self):
        ok, img = self.cap.read()
        if not ok:
            return None
        return Frame(img, None, None, KIND_LIVE, self.label)

    def close(self):
        self.cap.release()


class VideoFileSource(object):
    """Recorded RGB footage. NO depth, so obstacle distances are unavailable; ArUco alignment still works."""
    kind = KIND_RECORDED

    def __init__(self, path, loop=True):
        if not os.path.exists(path):
            raise SourceError("video file not found: %s" % path)
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise SourceError("cannot decode video: %s" % path)
        self.loop = loop
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 15.0
        self._last = 0.0
        self.label = "RECORDED video %s (RGB only, no depth)" % os.path.basename(path)

    def read(self):
        wait = 1.0 / self.fps - (time.time() - self._last)
        if wait > 0:
            time.sleep(wait)
        self._last = time.time()
        ok, img = self.cap.read()
        if not ok and self.loop:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, img = self.cap.read()
        if not ok:
            return None
        return Frame(img, None, None, KIND_RECORDED, self.label)

    def close(self):
        self.cap.release()


class SessionSource(object):
    """Recorded RGB-D session written by tools/record_session.py: color_00000.jpg, depth_00000.png (uint16 mm),
    meta.json (intrinsics + depth scale). Depth is REAL measured depth from an earlier camera run."""
    kind = KIND_RECORDED

    def __init__(self, folder, loop=True, fps=10.0):
        meta = os.path.join(folder, "meta.json")
        if not os.path.exists(meta):
            raise SourceError("not a session folder (no meta.json): %s" % folder)
        with open(meta) as f:
            self.meta = json.load(f)
        self.colors = sorted(glob.glob(os.path.join(folder, "color_*.jpg")))
        if not self.colors:
            raise SourceError("session has no frames: %s" % folder)
        self.intr = Intrinsics.from_dict(self.meta["intrinsics"])
        self.i = 0
        self.loop = loop
        self.fps = fps
        self._last = 0.0
        self.label = "RECORDED RGB-D session %s" % os.path.basename(os.path.normpath(folder))

    def read(self):
        wait = 1.0 / self.fps - (time.time() - self._last)
        if wait > 0:
            time.sleep(wait)
        self._last = time.time()
        if self.i >= len(self.colors):
            if not self.loop:
                return None
            self.i = 0
        cpath = self.colors[self.i]
        dpath = cpath.replace("color_", "depth_").replace(".jpg", ".png")
        self.i += 1
        color = cv2.imread(cpath)
        d = cv2.imread(dpath, cv2.IMREAD_UNCHANGED)
        depth = None if d is None else d.astype(np.float32) * float(self.meta.get("depth_scale_m_per_unit", 0.001))
        return Frame(color, depth, self.intr, KIND_RECORDED, self.label)

    def close(self):
        pass


def open_source(spec, cfg):
    """spec: auto | realsense | webcam | video:<path> | session:<dir> | none. Returns (source|None, message)."""
    c = cfg["camera"]
    spec = spec or c["source"]
    try:
        if spec == "none":
            return None, "no camera source selected"
        if spec == "realsense":
            return RealSenseSource(c["width"], c["height"], c["fps"]), "RealSense opened"
        if spec == "webcam":
            return WebcamSource(c["webcam_index"], c["width"], c["height"]), "webcam opened"
        if spec.startswith("video:"):
            return VideoFileSource(spec[6:]), "video opened"
        if spec.startswith("session:"):
            return SessionSource(spec[8:]), "session opened"
        if spec == "auto":
            try:
                return RealSenseSource(c["width"], c["height"], c["fps"]), "RealSense opened"
            except SourceError as e1:
                try:
                    return WebcamSource(c["webcam_index"], c["width"], c["height"]), \
                        "RealSense unavailable (%s); using webcam, RGB only" % e1
                except SourceError as e2:
                    return None, "no camera: RealSense: %s | webcam: %s" % (e1, e2)
        return None, "unknown camera source %r" % spec
    except SourceError as e:
        return None, str(e)
