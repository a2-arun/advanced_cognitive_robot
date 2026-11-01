"""YOLO BOTTLE OBSTACLE PROXY for setups WITHOUT a depth sensor (a plain USB webcam).

A pretrained YOLO model (COCO classes, nothing trained by us) finds a water bottle in the picture. A webcam cannot measure
distance, so the distance is ESTIMATED from the bottle's apparent height with the pinhole model:

    distance = focal_px * real_bottle_height / bottle_height_in_pixels

It needs the camera focal length (tools/calibrate_webcam_focal.py) and the bottle's MEASURED height. As with the ArUco
proxy, we then build a proxy depth image that is "far" everywhere except in the columns the bottle occupies, and feed it to
the SAME ObstacleDetector and AvoidanceFSM used with a real depth camera.

LIMITS (say them out loud in any video): only bottles are "seen" (any other obstacle is invisible); only inside the camera's
view; the distance is a size-based estimate (about +-10-15 %, upright bottle only), not a measurement of the room; a bottle
cut off by the top/bottom of the picture gives an upper bound. This is a stand-in for the RealSense path.
"""
import numpy as np


class UltralyticsDetector(object):
    """Pretrained YOLO (ultralytics). Loads lazily; downloads the weights file on first use if it is not on disk."""

    def __init__(self, model="yolov8n.pt", classes=("bottle",), conf=0.30, imgsz=480):
        from ultralytics import YOLO                     # imported here: only needed when this proxy is used
        self.model = YOLO(model)
        self.classes = None if classes is None else set(c.lower() for c in classes)      # None = report every class
        self.conf, self.imgsz = conf, imgsz
        names = self.model.names
        self.names = names if isinstance(names, dict) else dict(enumerate(names))
        if self.classes is not None and not any(n.lower() in self.classes for n in self.names.values()):
            raise ValueError("none of %s is a class of model %s" % (sorted(self.classes), model))

    def __call__(self, color):
        """-> list of (class_name, confidence, (x0, y0, x1, y1)) for the wanted classes."""
        res = self.model.predict(color, imgsz=self.imgsz, conf=self.conf, verbose=False)[0]
        out = []
        if res.boxes is None:
            return out
        for xyxy, c, k in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.conf.cpu().numpy(), res.boxes.cls.cpu().numpy()):
            name = str(self.names[int(k)])
            if self.classes is None or name.lower() in self.classes:
                out.append((name, float(c), tuple(float(v) for v in xyxy)))
        return out


class BottleObstacleProxy(object):
    kind_label = "YOLO-BOTTLE PROXY"
    depth_note = "PROXY DEPTH: YOLO bottle, size-based estimate, NOT measured"
    item_label = "BOTTLE"

    def __init__(self, detector, object_height_m, far_m, width_factor=3.0):
        if not object_height_m or object_height_m <= 0:
            raise ValueError("the bottle's real height is required (measure it): distance is not guessed")
        self.detector = detector
        self.height_m = float(object_height_m)
        self.far_m = far_m
        self.width_factor = width_factor          # the avoidance zone is this many bottle-widths wide (safety margin)

    def depth(self, color, intr):
        """Returns (proxy_depth float32 HxW in metres, list of dicts for the bottles seen)."""
        h, w = color.shape[:2]
        d = np.full((h, w), self.far_m, np.float32)
        seen = []
        fy = intr.scaled(w, h).fy if (intr.width != w or intr.height != h) else intr.fy
        for name, conf, (x0, y0, x1, y1) in self.detector(color):
            bh, bw = max(y1 - y0, 1.0), max(x1 - x0, 1.0)
            item = {"label": self.item_label, "class": name, "conf": round(conf, 2),
                    "box": [int(x0), int(y0), int(x1), int(y1)], "distance_m": None, "cropped": False}
            item["cropped"] = bool(y0 <= 2 or y1 >= h - 3)
            grow = bw * (self.width_factor - 1.0) / 2.0
            c0, c1 = max(0, int(x0 - grow)), min(w, int(x1 + grow) + 1)
            item["cols"] = [c0, c1]
            if bw <= bh:                                   # upright: its height is the known real height
                dist = fy * self.height_m / bh
                item["distance_m"] = round(float(dist), 3)
                d[:, c0:c1] = np.minimum(d[:, c0:c1], dist)
            seen.append(item)                              # lying down: seen, but NO distance is invented for it
        return d, seen
