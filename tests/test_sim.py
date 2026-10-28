import math
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from warehouse_bot.config import load_config
from warehouse_bot.sim import Sim
from warehouse_bot import perception as P
from warehouse_bot import docking as D


def make():
    cfg = load_config()
    cfg["sim"]["depth_noise_m"] = 0.0
    return cfg, Sim(cfg)


def run_pulse(sim, d, ms):
    r = sim.move(d, ms)
    assert r["ok"], r
    for _ in range(int(ms / 20) + 5):
        sim.step_once(0.02)


class SimTests(unittest.TestCase):
    def test_pulse_moves_robot_by_commanded_amount(self):
        cfg, s = make()
        x0 = s.x
        run_pulse(s, "F", 500)
        self.assertAlmostEqual(s.x - x0, cfg["sim"]["speed_mps"] * 0.5, places=2)
        self.assertFalse(s.is_busy())

    def test_turn_changes_heading_left_positive(self):
        cfg, s = make()
        run_pulse(s, "L", 400)
        self.assertAlmostEqual(s.th, math.radians(cfg["sim"]["turn_dps"] * 0.4), places=2)
        run_pulse(s, "R", 400)
        self.assertAlmostEqual(s.th, 0.0, places=2)

    def test_busy_rejects_second_command_and_stop_works(self):
        cfg, s = make()
        self.assertTrue(s.move("F", 500)["ok"])
        self.assertEqual(s.move("B", 100)["error"], "BUSY")
        s.stop()
        self.assertFalse(s.is_busy())

    def test_collision_with_obstacle_stops(self):
        cfg, s = make()
        for _ in range(600):
            if s.collided:
                break
            if not s.is_busy():
                s.move("F", 800)
            s.step_once(0.05)
        self.assertTrue(s.collided)
        self.assertLess(s.x, 4.0)                      # stopped before passing the box at x=3.7..4.3

    def test_depth_matches_geometry(self):
        cfg, s = make()
        s.obstacles = []
        f = s.render()
        mid = f.depth[:, 320]
        self.assertAlmostEqual(float(mid[240]), s.W - s.x, delta=0.02)   # east wall dead ahead
        s.add_obstacle(3.0, 3.0, 0.5)                                    # box face at x=2.75
        f = s.render()
        self.assertAlmostEqual(float(f.depth[240, 320]), 2.75 - 1.0, delta=0.03)

    def test_reset_restores_start(self):
        cfg, s = make()
        run_pulse(s, "F", 300)
        s.reset()
        self.assertEqual((s.x, s.th), (1.0, 0.0))
        self.assertEqual(s.pulses, 0)

    def test_odometry_estimate_drifts_from_truth(self):
        cfg, s = make()
        s.obstacles = []
        for _ in range(4):
            run_pulse(s, "F", 700)
        self.assertGreater(abs(s.ex - s.x), 0.01)       # the estimate is NOT ground truth

    def test_real_detector_sees_simulated_marker(self):
        cfg, s = make()
        s.obstacles = []
        s.x, s.y, s.th = 5.0, 3.0, 0.0
        f = s.render()
        det = D.ArucoDetector(dict(cfg["docking"], marker_size_m=0.40))
        obs = det.observe(f.color, f.depth, f.intrinsics)
        self.assertTrue(obs.detected)
        self.assertAlmostEqual(obs.offset, 0.0, delta=0.05)
        self.assertAlmostEqual(obs.depth_distance_m, 3.0, delta=0.1)
        self.assertAlmostEqual(obs.pnp_distance_m, 3.0, delta=0.3)

    def test_marker_occluded_by_obstacle_and_not_visible_from_behind(self):
        cfg, s = make()
        det = D.ArucoDetector(dict(cfg["docking"], marker_size_m=0.40))
        s.x, s.y, s.th = 5.0, 3.0, 0.0
        s.obstacles = []
        s.add_obstacle(6.5, 3.0, 0.9)                  # directly between robot and marker
        f = s.render()
        self.assertFalse(det.observe(f.color, f.depth, f.intrinsics).detected)
        s.obstacles = []
        s.x, s.y, s.th = 5.0, 3.0, math.pi            # facing away
        f = s.render()
        self.assertFalse(det.observe(f.color, f.depth, f.intrinsics).detected)

    def test_marker_offset_sign(self):
        cfg, s = make()
        s.obstacles = []
        det = D.ArucoDetector(dict(cfg["docking"], marker_size_m=0.40))
        s.x, s.y, s.th = 5.0, 3.0, math.radians(8)    # turned left (CCW) -> marker appears to the RIGHT
        f = s.render()
        obs = det.observe(f.color, f.depth, f.intrinsics)
        self.assertTrue(obs.detected)
        self.assertGreater(obs.offset, 0.1)


class ClosedLoopTests(unittest.TestCase):
    """Production perception + FSMs driving the simulator through the actuator interface."""

    def test_avoidance_turns_away_from_box_instead_of_hitting_it(self):
        cfg, s = make()
        ocfg = dict(cfg["obstacle"], advance_when_clear=True)
        det = P.ObstacleDetector(ocfg)
        fsm = P.AvoidanceFSM(ocfg, cfg["motion"], det, min_dwell_s=0.1)
        t = 0.0
        turned = []
        for i in range(2000):
            if i % 5 == 0:
                fr = s.render()
                out = fsm.update(det.process(fr.depth), t, True)
                a = out["action"]
                if a and a["kind"] == "move" and not s.is_busy():
                    if a["dir"] in ("L", "R"):
                        turned.append(a["dir"])
                    s.move(a["dir"], a["ms"])
            was = s.is_busy()
            s.step_once(0.02)
            if was and not s.is_busy():
                fsm.action_done(t)
            t += 0.02
        self.assertFalse(s.collided, "robot hit something")
        self.assertTrue(turned, "never turned")
        self.assertGreater(s.x, 2.5, "robot never approached the obstacle")

    def test_docking_closed_loop_in_simulator(self):
        cfg, s = make()
        s.obstacles = []
        s.x, s.y, s.th = 4.0, 3.8, math.radians(-8)
        dcfg = dict(cfg["docking"], marker_size_m=0.40, dock_distance_m=0.8, approach_pulse_ms=600, max_actions=80)
        det = D.ArucoDetector(dcfg)
        fsm = D.DockingFSM(dcfg, cfg["motion"])
        t = 0.0
        for i in range(6000):
            if i % 5 == 0:
                fr = s.render()
                obs = det.observe(fr.color, fr.depth, fr.intrinsics)
                out = fsm.update(obs, t, True)
                a = out["action"]
                if a and not s.is_busy():
                    s.move(a["dir"], a["ms"])
            was = s.is_busy()
            s.step_once(0.02)
            if was and not s.is_busy():
                fsm.action_done(t)
            t += 0.02
            if fsm.state in ("DOCKED", "ABORT"):
                break
        self.assertEqual(fsm.state, "DOCKED", (fsm.state, fsm.text, s.x, s.y))
        self.assertFalse(s.collided)
        self.assertLess(abs(s.y - 3.0), 0.5)
        self.assertLess(s.W - s.x, 1.2)


if __name__ == "__main__":
    unittest.main()
