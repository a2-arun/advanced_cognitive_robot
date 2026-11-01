import importlib.util
import os
import sys
import tempfile
import time
import unittest

import cv2
import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)

from warehouse_bot import docking
from warehouse_bot.camera import Frame, Intrinsics
from warehouse_bot.config import load_config
from warehouse_bot.engine import Engine
from warehouse_bot.proxy import MarkerObstacleProxy

F_PX, SIZE_M, W, H = 600.0, 0.095, 640, 480
DICT = "DICT_6X6_50"


def load_tool(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(ROOT, "tools", name + ".py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def scene(markers):
    """markers: list of (id, center_x, distance_m). Renders each as a head-on marker whose pixel size matches the
    pinhole model with focal F_PX, so PnP must recover the distance."""
    img = np.full((H, W, 3), 190, np.uint8)
    for mid, cx, dist in markers:
        px = int(round(F_PX * SIZE_M / dist))
        m = cv2.cvtColor(docking.generate_marker(DICT, mid, px), cv2.COLOR_GRAY2BGR)
        x0, y0 = int(cx - px / 2), int(H / 2 - px / 2)
        img[y0:y0 + px, x0:x0 + px] = m
    return img


INTR = Intrinsics(W, H, F_PX, F_PX, W / 2.0, H / 2.0)


class FakeWebcam(object):
    """RGB only (depth=None), with intrinsics, like a focal-calibrated webcam."""
    kind = "live"
    label = "LIVE webcam #0 (RGB only, no depth, focal calibrated)"

    def __init__(self, markers):
        self.img = scene(markers)

    def read(self):
        time.sleep(0.05)
        return Frame(self.img.copy(), None, INTR, "live", self.label)

    def close(self):
        pass


class FocalTests(unittest.TestCase):
    def test_focal_recovered_from_marker_size_at_known_distance(self):
        t = load_tool("calibrate_webcam_focal")
        det = docking.ArucoDetector({"aruco_dict": DICT, "marker_id": 2, "marker_size_m": SIZE_M})
        src = FakeWebcam([(2, 320, 0.5)])
        sizes = t.measure(src, det, 2, seconds=1.0, max_frames=15)
        self.assertGreaterEqual(len(sizes), 10)
        f = t.focal_from_sizes(sizes, 0.5, SIZE_M)
        self.assertAlmostEqual(f, F_PX, delta=0.04 * F_PX)

    def test_wrong_marker_id_is_not_counted(self):
        t = load_tool("calibrate_webcam_focal")
        det = docking.ArucoDetector({"aruco_dict": DICT, "marker_id": 2, "marker_size_m": SIZE_M})
        self.assertEqual(t.measure(FakeWebcam([(5, 320, 0.5)]), det, 2, seconds=0.5, max_frames=8), [])


class FocalDiagnosisTests(unittest.TestCase):
    def test_diagnose_reports_the_family_and_id_that_is_actually_visible(self):
        t = load_tool("calibrate_webcam_focal")
        img = np.full((H, W, 3), 190, np.uint8)
        m = cv2.cvtColor(docking.generate_marker("DICT_4X4_50", 7, 120), cv2.COLOR_GRAY2BGR)
        img[100:220, 200:320] = m
        seen = t.diagnose_frame(img)
        self.assertIn("DICT_4X4_50", seen)
        self.assertIn(7, seen["DICT_4X4_50"])
        self.assertNotIn("DICT_6X6_50", seen)               # so the tool can say: wrong family

    def test_preview_outlines_the_wanted_marker_and_reports_its_size(self):
        t = load_tool("calibrate_webcam_focal")
        det = docking.ArucoDetector({"aruco_dict": DICT, "marker_id": 2, "marker_size_m": SIZE_M})
        frame = FakeWebcam([(2, 320, 0.5)]).read().color
        vis, edge = t.annotate_preview(frame, det, 2)
        self.assertIsNotNone(edge)
        self.assertGreater(edge, 20)
        self.assertEqual(vis.shape, frame.shape)
        self.assertFalse(np.array_equal(vis, frame))         # something was drawn

    def test_preview_says_not_found_when_marker_absent(self):
        t = load_tool("calibrate_webcam_focal")
        det = docking.ArucoDetector({"aruco_dict": DICT, "marker_id": 2, "marker_size_m": SIZE_M})
        vis, edge = t.annotate_preview(np.full((H, W, 3), 190, np.uint8), det, 2)
        self.assertIsNone(edge)

    def test_diagnose_on_empty_scene_finds_nothing(self):
        t = load_tool("calibrate_webcam_focal")
        self.assertEqual(t.diagnose_frame(np.full((H, W, 3), 190, np.uint8)), {})

    def test_warm_up_skips_black_frames(self):
        t = load_tool("calibrate_webcam_focal")

        class Slow(object):
            n = 0

            def read(self):
                Slow.n += 1
                img = np.zeros((H, W, 3), np.uint8) if Slow.n < 25 else np.full((H, W, 3), 120, np.uint8)
                return Frame(img, None, None, "live", "x")
        f = t.warm_up(Slow(), seconds=3.0)
        self.assertGreater(float(f.color.mean()), 25)


class ProxyTests(unittest.TestCase):
    def setUp(self):
        cfg = load_config()
        self.det = docking.ArucoDetector(dict(cfg["docking"], aruco_dict=DICT, marker_id=2, marker_size_m=SIZE_M))
        self.proxy = MarkerObstacleProxy(self.det, 3, SIZE_M, 4.0, 3.0)

    def test_tagged_box_distance_and_columns(self):
        d, seen = self.proxy.depth(scene([(3, 320, 0.6)]), INTR)
        self.assertEqual(len(seen), 1)
        self.assertAlmostEqual(seen[0]["distance_m"], 0.6, delta=0.05)
        self.assertAlmostEqual(float(d[240, 320]), 0.6, delta=0.05)
        self.assertEqual(float(d[240, 5]), 4.0)                # outside the box: "far"
        self.assertAlmostEqual(float(d[10, 320]), seen[0]["distance_m"], places=2)  # full height, like a box

    def test_other_ids_are_ignored_and_no_marker_means_clear(self):
        d, seen = self.proxy.depth(scene([(2, 320, 0.6)]), INTR)    # the DOCK marker is not an obstacle
        self.assertEqual(seen, [])
        self.assertTrue(np.all(d == 4.0))

    def test_no_intrinsics_means_no_invented_distance(self):
        d, seen = self.proxy.depth(scene([(3, 320, 0.6)]), None)
        self.assertEqual(seen, [])


class RobotPathWithWebcamTests(unittest.TestCase):
    """The REAL engine + serial link (against the mock Arduino) driven by a fake focal-calibrated webcam."""

    def make_engine(self, markers):
        cfg = load_config()
        cfg["server"]["results_dir"] = tempfile.mkdtemp()
        cfg["camera"]["source"] = "none"
        cfg["docking"].update(aruco_dict=DICT, marker_id=2, marker_size_m=SIZE_M)
        cfg["proxy"]["obstacle_marker_id"] = 3
        eng = Engine(cfg)
        eng.start()
        eng.set_mode("hardware")
        time.sleep(0.5)
        eng.source = FakeWebcam(markers)
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

    def test_tagged_obstacle_makes_the_robot_turn_and_is_labelled_as_proxy(self):
        eng = self.make_engine([(3, 320, 0.6)])
        try:
            self.assertTrue(eng.set_behavior("avoid", act=True)["ok"])
            self.assertTrue(self.wait_for(eng, lambda log: any(c.startswith("M L") or c.startswith("M R") for c in log)),
                            "no turn command reached the Arduino")
            st = eng.get_state()
            self.assertIn("PROXY", st["data_label"])
            self.assertTrue(st["camera"]["proxy"])
            self.assertFalse(st["camera"]["has_depth"])         # honest: there is no real depth
            self.assertFalse(any(c.startswith("M F") for c in eng.link._ser.log))   # never drives into it
        finally:
            eng.shutdown()

    def test_perception_only_never_moves_the_robot(self):
        eng = self.make_engine([(3, 320, 0.6)])
        try:
            eng.set_behavior("avoid", act=False)
            time.sleep(3.0)
            self.assertFalse(any(c.startswith("M ") for c in eng.link._ser.log))
            self.assertEqual(eng.get_state()["obstacle"]["det"]["state"], "OBSTACLE")
        finally:
            eng.shutdown()

    def test_docking_aligns_toward_a_marker_off_to_the_right(self):
        eng = self.make_engine([(2, 520, 0.9)])
        try:
            eng.set_behavior("dock", act=True)
            self.assertTrue(self.wait_for(eng, lambda log: any(c.startswith("M R") for c in log)),
                            "no right-align pulse reached the Arduino")
            obs = eng.get_state()["docking"]["obs"]
            self.assertEqual(obs["distance_src"], "pnp")           # distance from marker size, not invented
            self.assertAlmostEqual(obs["distance_m"], 0.9, delta=0.08)
        finally:
            eng.shutdown()

    def test_no_focal_means_docking_will_not_approach(self):
        eng = self.make_engine([(2, 320, 0.9)])
        eng.source = type("NoFocal", (FakeWebcam,), {"read": lambda s: (time.sleep(0.05),
                          Frame(s.img.copy(), None, None, "live", "LIVE webcam (no focal)"))[1]})([(2, 320, 0.9)])
        try:
            eng.set_behavior("dock", act=True)
            time.sleep(4.0)
            self.assertFalse(any(c.startswith("M F") for c in eng.link._ser.log))   # no distance -> no blind approach
            self.assertEqual(eng.get_state()["docking"]["fsm"]["decision"], "NO_DISTANCE")
        finally:
            eng.shutdown()


if __name__ == "__main__":
    unittest.main()
