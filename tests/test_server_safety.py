import json
import os
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from warehouse_bot.config import load_config
from warehouse_bot.server import build_server


class ServerBase(unittest.TestCase):
    token = None

    @classmethod
    def setUpClass(cls):
        cfg = load_config()
        cfg["server"]["results_dir"] = tempfile.mkdtemp()
        cfg["camera"]["source"] = "none"          # tests must never grab a real camera
        cls.eng, cls.httpd, cls.tok = build_server(cfg, "127.0.0.1", 0, token=cls.token)
        cls.eng.start()
        threading.Thread(target=cls.httpd.serve_forever, daemon=True).start()
        cls.base = "http://127.0.0.1:%d" % cls.httpd.server_address[1]
        time.sleep(0.6)

    @classmethod
    def tearDownClass(cls):
        cls.httpd.shutdown()
        cls.eng.shutdown()
        cls.httpd.server_close()

    def q(self, path):
        return self.base + path + (("?token=" + self.tok) if self.tok else "")

    def post(self, path, body=None):
        req = urllib.request.Request(self.q(path), data=json.dumps(body or {}).encode(),
                                     headers={"Content-Type": "application/json"})
        return json.loads(urllib.request.urlopen(req, timeout=10).read().decode())

    def get(self, path):
        return json.loads(urllib.request.urlopen(self.q(path), timeout=10).read().decode())


class SafetyTests(ServerBase):
    def setUp(self):
        self.post("/api/stop")
        self.post("/api/mode", {"mode": "sim"})
        self.post("/api/sim", {"action": "reset"})
        time.sleep(0.2)

    def test_sim_state_and_labels(self):
        s = self.get("/api/state")
        self.assertEqual(s["mode"], "sim")
        self.assertEqual(s["data_label"], "SIMULATED")
        self.assertTrue(s["sim"]["is_sim"])

    def test_static_page_served(self):
        html = urllib.request.urlopen(self.base + "/").read().decode()
        self.assertIn("Warehouse Robot", html)
        js = urllib.request.urlopen(self.base + "/app.js").read().decode()
        self.assertIn("api/stop", js)

    def test_sim_teleop_moves_sim_and_never_touches_serial(self):
        self.post("/api/mode", {"mode": "hardware"})
        self.assertTrue(self.post("/api/connect", {"port": "mock"})["ok"])
        mock = self.eng.link._ser
        self.assertTrue(self.post("/api/arm")["ok"])
        self.post("/api/mode", {"mode": "sim"})              # also disarms the hardware
        n_before = len([c for c in mock.log if c.startswith("M ")])
        x0 = self.get("/api/state")["sim"]["robot"]["x"]
        r = self.post("/api/move", {"dir": "F", "ms": 400})
        self.assertTrue(r["ok"], r)
        time.sleep(0.8)
        self.assertGreater(self.get("/api/state")["sim"]["robot"]["x"], x0)
        self.assertEqual(len([c for c in mock.log if c.startswith("M ")]), n_before)   # no motor command reached serial
        self.assertEqual(mock.motor_levels, (0, 0))
        self.post("/api/mode", {"mode": "hardware"})
        self.post("/api/disconnect")

    def test_hardware_teleop_reaches_mock_only_when_armed(self):
        self.post("/api/mode", {"mode": "hardware"})
        self.post("/api/connect", {"port": "mock"})
        mock = self.eng.link._ser
        r = self.post("/api/move", {"dir": "F", "ms": 200})
        self.assertFalse(r["ok"])                              # not armed
        self.assertEqual(self.get("/api/state")["sim"].get("running"), False)   # simulator is inactive in hw mode
        self.post("/api/arm")
        time.sleep(0.3)
        r = self.post("/api/move", {"dir": "L", "ms": 200})
        self.assertTrue(r["ok"], r)
        time.sleep(0.6)
        self.assertTrue(any(c.startswith("M L") for c in mock.log))
        st = self.get("/api/state")
        self.assertEqual(st["data_label"] in ("NO CAMERA", "LIVE", "RECORDED"), True)
        self.assertEqual(len(st["hw"]["enc"]), 4)
        self.post("/api/disconnect")

    def test_hardware_commands_blocked_when_link_dies(self):
        self.post("/api/mode", {"mode": "hardware"})
        self.post("/api/connect", {"port": "mock"})
        self.post("/api/arm")
        time.sleep(0.3)
        self.assertTrue(self.get("/api/state")["hw_can_move"])
        self.eng.link._ser._alive = False                      # cable pulled: telemetry stops
        time.sleep(1.4)
        st = self.get("/api/state")
        self.assertFalse(st["hw"]["link_ok"])
        self.assertFalse(st["hw_can_move"])
        r = self.post("/api/move", {"dir": "F", "ms": 100})
        self.assertFalse(r["ok"])
        self.post("/api/disconnect")

    def test_autonomy_cannot_act_on_hardware_unless_armed(self):
        self.post("/api/mode", {"mode": "hardware"})
        r = self.post("/api/behavior", {"behavior": "avoid", "act": True})
        self.assertFalse(r["ok"])
        r = self.post("/api/behavior", {"behavior": "avoid", "act": False})   # perception-only is fine
        self.assertTrue(r["ok"])
        self.assertFalse(self.get("/api/state")["act"])
        r = self.post("/api/behavior", {"behavior": "mission", "act": False})
        self.assertFalse(r["ok"])                                             # missions are sim-only

    def test_teleop_refused_while_autonomy_active(self):
        self.post("/api/behavior", {"behavior": "avoid", "act": True})
        r = self.post("/api/move", {"dir": "F", "ms": 100})
        self.assertFalse(r["ok"])
        self.assertIn("autonomous", r["error"])
        self.post("/api/stop")
        self.assertEqual(self.get("/api/state")["behavior"], "off")
        self.assertTrue(self.post("/api/move", {"dir": "F", "ms": 100})["ok"])

    def test_stop_halts_sim_motion_and_clears_autonomy(self):
        self.post("/api/behavior", {"behavior": "avoid", "act": True, "advance": True})
        time.sleep(1.0)
        self.post("/api/stop")
        st = self.get("/api/state")
        self.assertEqual(st["behavior"], "off")
        self.assertFalse(st["act"])
        self.assertFalse(st["sim"]["robot"]["moving"])

    def test_sim_perception_only_does_not_move_robot(self):
        self.post("/api/behavior", {"behavior": "avoid", "act": False, "advance": True})
        x0 = self.get("/api/state")["sim"]["robot"]["x"]
        time.sleep(1.5)
        self.assertEqual(self.get("/api/state")["sim"]["robot"]["x"], x0)

    def test_mjpeg_streams_deliver_jpegs(self):
        for name in ("obstacle", "depth", "dock"):
            r = urllib.request.urlopen(self.q("/stream/%s.mjpg" % name), timeout=10)
            chunk = r.read(20000)
            r.close()
            self.assertIn(b"image/jpeg", chunk)
            self.assertIn(b"\xff\xd8", chunk)

    def test_pointcloud_from_sim_is_labelled_simulated(self):
        time.sleep(0.4)
        pc = self.get("/api/pointcloud")
        self.assertTrue(pc["ok"])
        self.assertEqual(pc["kind"], "SIMULATED")
        self.assertGreater(pc["n"], 500)
        saved = self.post("/api/pointcloud/save")
        self.assertTrue(os.path.exists(saved["saved"]))
        with open(saved["saved"]) as fh:
            head = fh.read(400)
        self.assertIn("not SLAM", head)

    def test_snapshot_writes_files(self):
        time.sleep(0.4)
        r = self.post("/api/snapshot")
        self.assertTrue(r["ok"])
        self.assertTrue(all(os.path.exists(f) for f in r["files"]))

    def test_obstacle_toggle_via_api(self):
        n = len(self.get("/api/state")["sim"]["world"]["obstacles"])
        self.assertTrue(self.post("/api/sim", {"action": "toggle_obstacle", "x": 6.0, "y": 4.0})["ok"])
        self.assertEqual(len(self.get("/api/state")["sim"]["world"]["obstacles"]), n + 1)
        self.post("/api/sim", {"action": "toggle_obstacle", "x": 6.0, "y": 4.0})
        self.assertEqual(len(self.get("/api/state")["sim"]["world"]["obstacles"]), n)

    def test_bad_requests(self):
        self.assertFalse(self.post("/api/move", {"dir": "X"})["ok"])
        self.assertFalse(self.post("/api/mode", {"mode": "warp"})["ok"])
        self.assertFalse(self.post("/api/nonexistent")["ok"])


class TokenTests(ServerBase):
    token = "s3cret"

    def test_requires_token(self):
        for path in ("/api/state", "/stream/obstacle.mjpg"):
            with self.assertRaises(urllib.error.HTTPError) as c:
                urllib.request.urlopen(self.base + path, timeout=5)
            self.assertEqual(c.exception.code, 401)
        req = urllib.request.Request(self.base + "/api/stop", data=b"{}", headers={"Content-Type": "application/json"})
        with self.assertRaises(urllib.error.HTTPError) as c:
            urllib.request.urlopen(req, timeout=5)
        self.assertEqual(c.exception.code, 401)
        self.assertEqual(self.get("/api/state")["mode"], "sim")      # with the token it works
        wrong = urllib.request.Request(self.base + "/api/state?token=nope")
        with self.assertRaises(urllib.error.HTTPError):
            urllib.request.urlopen(wrong, timeout=5)

    def test_non_local_bind_generates_token(self):
        cfg = load_config()
        cfg["server"]["results_dir"] = tempfile.mkdtemp()
        eng, httpd, tok = build_server(cfg, "0.0.0.0", 0)
        try:
            self.assertTrue(tok and len(tok) >= 8)
        finally:
            httpd.server_close()
            eng.shutdown()


if __name__ == "__main__":
    unittest.main()
