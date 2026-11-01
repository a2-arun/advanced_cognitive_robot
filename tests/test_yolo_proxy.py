import contextlib
import io
import os
import sys
import tempfile
import time
import unittest
from unittest import mock

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)

from warehouse_bot import server
from warehouse_bot.camera import Frame, Intrinsics
from warehouse_bot.config import load_config
from warehouse_bot.engine import Engine
from warehouse_bot.yolo_proxy import BottleObstacleProxy

F_PX, W, H = 600.0, 640, 480
BOTTLE_M = 0.20
INTR = Intrinsics(W, H, F_PX, F_PX, W / 2.0, H / 2.0)


def bottle_box(dist_m, cx=320, aspect=0.3, bottom=340):
    """Pixel box an upright BOTTLE_M-tall bottle would have at dist_m under the pinhole model."""
    hp = F_PX * BOTTLE_M / dist_m
    return (cx - hp * aspect / 2.0, bottom - hp, cx + hp * aspect / 2.0, bottom)


def fake_detector(*boxes):
    """Stands in for the YOLO model (no weights, no network): always 'sees' these boxes as bottles."""
    return lambda color: [("bottle", 0.9, b) for b in boxes]


def grey():
    return np.full((H, W, 3), 190, np.uint8)


class BottleProxyTests(unittest.TestCase):
    def test_distance_from_apparent_height_and_columns(self):
        p = BottleObstacleProxy(fake_detector(bottle_box(0.6)), BOTTLE_M, 4.0, 3.0)
        d, seen = p.depth(grey(), INTR)
        self.assertEqual(len(seen), 1)
        self.assertAlmostEqual(seen[0]["distance_m"], 0.6, delta=0.02)
        self.assertAlmostEqual(float(d[240, 320]), 0.6, delta=0.02)
        self.assertEqual(float(d[240, 5]), 4.0)                          # away from the bottle: far
        self.assertAlmostEqual(float(d[5, 320]), 0.6, delta=0.02)        # full height of the image, like a box
        self.assertLess(seen[0]["cols"][0], int(bottle_box(0.6)[0]))     # the avoidance zone is wider than the bottle (margin)

    def test_nearer_bottle_reads_nearer(self):
        near = BottleObstacleProxy(fake_detector(bottle_box(0.5)), BOTTLE_M, 4.0).depth(grey(), INTR)[1][0]["distance_m"]
        far = BottleObstacleProxy(fake_detector(bottle_box(1.5)), BOTTLE_M, 4.0).depth(grey(), INTR)[1][0]["distance_m"]
        self.assertLess(near, far)

    def test_no_bottle_means_everything_is_far(self):
        d, seen = BottleObstacleProxy(fake_detector(), BOTTLE_M, 4.0).depth(grey(), INTR)
        self.assertEqual(seen, [])
        self.assertTrue(np.all(d == 4.0))

    def test_bottle_lying_down_gets_no_invented_distance(self):
        lying = (200.0, 300.0, 330.0, 340.0)                              # wider than tall
        d, seen = BottleObstacleProxy(fake_detector(lying), BOTTLE_M, 4.0).depth(grey(), INTR)
        self.assertEqual(len(seen), 1)
        self.assertIsNone(seen[0]["distance_m"])
        self.assertTrue(np.all(d == 4.0))

    def test_bottle_cut_off_at_the_bottom_is_flagged(self):
        p = BottleObstacleProxy(fake_detector((300.0, 200.0, 340.0, 479.0)), BOTTLE_M, 4.0)
        self.assertTrue(p.depth(grey(), INTR)[1][0]["cropped"])

    def test_real_height_is_required(self):
        for bad in (None, 0, -1):
            with self.assertRaises(ValueError):
                BottleObstacleProxy(fake_detector(), bad, 4.0)

    def test_intrinsics_are_rescaled_to_the_picture_size(self):
        half = np.full((H // 2, W // 2, 3), 190, np.uint8)
        b = tuple(v / 2.0 for v in bottle_box(0.6))                       # same scene at half resolution
        seen = BottleObstacleProxy(fake_detector(b), BOTTLE_M, 4.0).depth(half, INTR)[1]
        self.assertAlmostEqual(seen[0]["distance_m"], 0.6, delta=0.03)


class FakeBottleCam(object):
    """RGB only with intrinsics; the bottle itself comes from the patched detector."""
    kind = "live"
    label = "LIVE webcam #0 (RGB only, no depth, focal calibrated)"

    def read(self):
        time.sleep(0.05)
        return Frame(grey(), None, INTR, "live", self.label)

    def close(self):
        pass


class RobotPathWithBottleTests(unittest.TestCase):
    def make_engine(self, boxes, fail=None):
        cfg = load_config()
        cfg["server"]["results_dir"] = tempfile.mkdtemp()
        cfg["camera"]["source"] = "none"
        cfg["proxy"].update(yolo_bottle=True, bottle_height_cm=BOTTLE_M * 100)

        def factory(*a, **k):
            if fail:
                raise ImportError(fail)
            return fake_detector(*boxes)
        patcher = mock.patch("warehouse_bot.engine.UltralyticsDetector", factory)    # stays on for the whole test: never download
        patcher.start()
        self.addCleanup(patcher.stop)
        eng = Engine(cfg)
        eng.start()
        eng.set_mode("hardware")                       # (mode switch rebuilds the proxy: the detector is cached)
        time.sleep(0.5)
        eng.source = FakeBottleCam()
        self.assertTrue(eng.connect("mock")["ok"])
        self.assertTrue(eng.arm()["ok"])
        time.sleep(0.4)
        return eng

    def wait_for(self, eng, pred, timeout=12.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if pred(eng.link._ser.log):
                return True
            time.sleep(0.1)
        return False

    def test_bottle_ahead_makes_the_robot_turn_and_is_labelled(self):
        eng = self.make_engine([bottle_box(0.5)])
        try:
            self.assertTrue(eng.set_behavior("avoid", act=True)["ok"])
            self.assertTrue(self.wait_for(eng, lambda log: any(c.startswith("M L") or c.startswith("M R") for c in log)),
                            "no turn command reached the Arduino")
            st = eng.get_state()
            self.assertIn("YOLO", st["data_label"])
            self.assertTrue(st["camera"]["proxy"])
            self.assertFalse(st["camera"]["has_depth"])
            self.assertFalse(any(c.startswith("M F") for c in eng.link._ser.log))
        finally:
            eng.shutdown()

    def test_far_bottle_is_not_an_obstacle(self):
        eng = self.make_engine([bottle_box(2.5)])
        try:
            eng.set_behavior("avoid", act=False)
            time.sleep(3.0)
            self.assertEqual(eng.get_state()["obstacle"]["det"]["state"], "CLEAR")
        finally:
            eng.shutdown()

    def test_perception_only_never_moves_the_robot(self):
        eng = self.make_engine([bottle_box(0.5)])
        try:
            eng.set_behavior("avoid", act=False)
            time.sleep(3.0)
            self.assertFalse(any(c.startswith("M ") for c in eng.link._ser.log))
            self.assertEqual(eng.get_state()["obstacle"]["det"]["state"], "OBSTACLE")
        finally:
            eng.shutdown()

    def test_model_unavailable_is_reported_and_never_faked(self):
        eng = self.make_engine([], fail="No module named ultralytics")
        try:
            self.assertIsNone(eng.proxy)
            self.assertIn("ultralytics", eng.proxy_err)
            eng.set_behavior("avoid", act=True)
            time.sleep(3.0)
            st = eng.get_state()
            self.assertIn("YOLO UNAVAILABLE", st["data_label"])
            self.assertFalse(any(c.startswith("M ") for c in eng.link._ser.log))      # no depth -> no motion
        finally:
            eng.shutdown()


class CliTests(unittest.TestCase):
    def test_yolo_bottle_without_height_or_focal_is_refused(self):
        with contextlib.redirect_stderr(io.StringIO()):                  # argparse prints usage; keep the test log clean
            with self.assertRaises(SystemExit):
                server.main(["--yolo-bottle", "--focal-px", "900"])
            with self.assertRaises(SystemExit):
                server.main(["--yolo-bottle", "--bottle-height-cm", "20"])


if __name__ == "__main__":
    unittest.main()
