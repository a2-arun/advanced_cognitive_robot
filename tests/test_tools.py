import importlib.util
import os
import subprocess
import sys
import tempfile
import unittest

import cv2

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)

from warehouse_bot import docking, camera
from warehouse_bot.config import load_config
from warehouse_bot.sim import Sim


def load_tool(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(ROOT, "tools", name + ".py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


class ToolTests(unittest.TestCase):
    def setUp(self):
        self.old = os.getcwd()
        self.tmp = tempfile.mkdtemp()
        os.chdir(self.tmp)

    def tearDown(self):
        os.chdir(self.old)

    def test_check_hardware_passes_against_mock_and_refuses_without_confirmation(self):
        t = load_tool("check_hardware")
        rep = t.run("mock", 200, confirm=lambda: "no")
        self.assertFalse(rep["ok"])
        self.assertEqual(rep["pulses"], {})                      # nothing moved
        rep = t.run("mock", 200, confirm=lambda: "LIFTED")
        self.assertTrue(rep["ok"], rep)
        f, b, l, r = (rep["pulses"][k] for k in "FBLR")
        self.assertTrue(all(x > 0 for x in f))
        self.assertTrue(all(x < 0 for x in b))
        self.assertTrue(l[0] < 0 and l[3] < 0 and l[1] > 0 and l[2] > 0)

    def test_marker_generator_output_is_detectable(self):
        t = load_tool("make_aruco_marker")
        t_argv = sys.argv
        sys.argv = ["x", "--id", "0", "--size-cm", "6", "--dpi", "200"]
        try:
            t.main()
        finally:
            sys.argv = t_argv
        path = os.path.join("results", "markers", "aruco_4x4_50_id0_6cm.png")
        img = cv2.imread(path)
        self.assertIsNotNone(img)
        obs = docking.ArucoDetector(load_config()["docking"]).observe(img)
        self.assertTrue(obs.detected)

    def test_record_and_replay_session_roundtrip(self):
        rec = load_tool("record_session")
        cfg = load_config()
        cfg["sim"]["depth_noise_m"] = 0.0
        sim = Sim(cfg)

        class Adapter(object):
            label = "SIMULATED camera"

            def read(self):
                return sim.render()
        n = rec.record(Adapter(), "sess", seconds=1.0, fps=5)
        self.assertGreaterEqual(n, 3)
        src = camera.SessionSource("sess", fps=50)
        f = src.read()
        self.assertEqual(f.kind, "recorded")
        self.assertIn("RECORDED", f.label)
        live = sim.render()
        self.assertAlmostEqual(float(f.depth[240, 320]), float(live.depth[240, 320]), delta=0.01)

    def test_calibrate_odometry_against_mock(self):
        t = load_tool("calibrate_odometry")
        answers = iter(["FLOOR", "0.50", "90"])
        rc = t.main(["--port", "mock", "--pulses", "2", "--ms", "200"], ask=lambda prompt: next(answers))
        self.assertEqual(rc, 0)
        import json
        res = json.load(open(os.path.join("results", "odometry_calibration.json")))
        self.assertGreater(res["counts_per_m"], 100)
        self.assertGreater(res["effective_track_m"], 0)

    def test_probe_environment_runs_and_writes_report(self):
        r = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "probe_environment.py")],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60, universal_newlines=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("OpenCV / ArUco", r.stdout)
        self.assertTrue(os.path.exists(os.path.join("results", "environment_report.txt")))


if __name__ == "__main__":
    unittest.main()
