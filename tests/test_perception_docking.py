import os
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from warehouse_bot.config import load_config
from warehouse_bot import perception as P
from warehouse_bot import docking as D
from warehouse_bot import pointcloud as PC
from warehouse_bot.camera import Intrinsics

CFG = load_config()


def depth_frame(front=3.0, left=3.0, right=3.0, h=120, w=160, hole=False):
    d = np.full((h, w), 3.5, np.float32)
    d[:, int(0.35 * w):int(0.65 * w)] = front
    d[:, :int(0.30 * w)] = left
    d[:, int(0.70 * w):] = right
    if hole:
        d[:] = 0
    return d


def run_fsm(frames, can_act=False, start=0.0, dt=0.1, executor=None):
    ocfg = dict(CFG["obstacle"])
    det = P.ObstacleDetector(ocfg)
    fsm = P.AvoidanceFSM(ocfg, CFG["motion"], det, min_dwell_s=0.2)
    t = start
    trace = []
    for fr in frames:
        d = det.process(fr)
        out = fsm.update(d, t, can_act)
        if out["action"] and out["action"]["kind"] == "move" and executor:
            executor(out["action"], fsm, t)
        trace.append((out["state"], out["intended"], out["action"]))
        t += dt
    return trace, det, fsm


class ObstacleTests(unittest.TestCase):
    def test_no_depth_is_not_invented(self):
        det = P.ObstacleDetector(CFG["obstacle"])
        r = det.process(None)
        self.assertEqual(r["state"], P.NO_DEPTH)
        self.assertIsNone(r["front_m"])

    def test_all_invalid_depth_is_unreliable(self):
        det = P.ObstacleDetector(CFG["obstacle"])
        for _ in range(6):
            r = det.process(depth_frame(hole=True))
        self.assertEqual(r["state"], P.UNRELIABLE)

    def test_clear_then_obstacle_needs_confirmation(self):
        det = P.ObstacleDetector(CFG["obstacle"])
        for _ in range(5):
            r = det.process(depth_frame(front=3.0))
        self.assertEqual(r["state"], P.CLEAR)
        seen = []
        for _ in range(8):
            seen.append(det.process(depth_frame(front=0.5))["state"])
        self.assertEqual(seen[0], P.CLEAR)                 # single frame doesn't flip it
        self.assertEqual(seen[-1], P.OBSTACLE)
        self.assertAlmostEqual(det.process(depth_frame(front=0.5))["front_m"], 0.5, places=2)

    def test_hysteresis_prevents_flicker(self):
        det = P.ObstacleDetector(CFG["obstacle"])
        for _ in range(8):
            det.process(depth_frame(front=0.5))
        self.assertEqual(det.state, P.OBSTACLE)
        for _ in range(8):                                  # 0.85 m is above 0.80 but inside hysteresis band
            det.process(depth_frame(front=0.85))
        self.assertEqual(det.state, P.OBSTACLE)
        for _ in range(8):
            det.process(depth_frame(front=1.2))
        self.assertEqual(det.state, P.CLEAR)

    def test_perception_mode_selects_open_side_without_acting(self):
        frames = [depth_frame(3, 3, 3)] * 6 + [depth_frame(0.5, 0.5, 2.5)] * 25
        trace, det, fsm = run_fsm(frames, can_act=False)
        self.assertEqual(trace[-1][0], "TURN")
        self.assertEqual(trace[-1][1], "R")                 # right side is open
        self.assertTrue(all(a is None for _, _, a in trace))  # perception mode never emits an action

    def test_selects_left_when_left_open(self):
        frames = [depth_frame(3, 3, 3)] * 6 + [depth_frame(0.5, 2.5, 0.6)] * 25
        trace, _, _ = run_fsm(frames)
        self.assertEqual(trace[-1][1], "L")

    def test_both_blocked_stops(self):
        frames = [depth_frame(3, 3, 3)] * 6 + [depth_frame(0.5, 0.6, 0.6)] * 30
        trace, _, _ = run_fsm(frames, can_act=True)
        self.assertEqual(trace[-1][0], "STOP_BLOCKED")
        self.assertTrue(all(a is None or a["kind"] != "move" for _, _, a in trace))

    def test_unreliable_depth_emits_stop_when_acting(self):
        frames = [depth_frame(3, 3, 3)] * 6 + [depth_frame(hole=True)] * 10
        trace, _, _ = run_fsm(frames, can_act=True)
        self.assertEqual(trace[-1][0], "STOP_UNRELIABLE")
        self.assertEqual(trace[-1][2]["kind"], "stop")

    def test_acting_turns_once_then_rechecks_and_clears(self):
        moves = []
        world = {"front": 0.5}

        def executor(action, fsm, t):
            moves.append(action["dir"])
            world["front"] = 3.0                            # the turn cleared the obstacle
            fsm.action_done(t + 0.3)

        frames = []

        class Lazy(list):
            pass
        ocfg = dict(CFG["obstacle"])
        det = P.ObstacleDetector(ocfg)
        fsm = P.AvoidanceFSM(ocfg, CFG["motion"], det, min_dwell_s=0.2)
        t = 0.0
        states = []
        for i in range(80):
            d = det.process(depth_frame(world["front"], 0.6, 2.5) if i >= 6 else depth_frame(3, 3, 3))
            if i < 6:
                pass
            out = fsm.update(d, t, True)
            if out["action"] and out["action"]["kind"] == "move":
                moves.append(out["action"]["dir"])
                world["front"] = 3.0
                fsm.action_done(t)
            states.append(out["state"])
            t += 0.1
        self.assertEqual(moves, ["R"])
        self.assertEqual(states[-1], "CLEAR")
        for s in ("OBSTACLE_DETECTED", "SELECT_DIRECTION", "TURN", "RECHECK"):
            self.assertIn(s, states)

    def test_gives_up_after_max_turns(self):
        ocfg = dict(CFG["obstacle"])
        det = P.ObstacleDetector(ocfg)
        fsm = P.AvoidanceFSM(ocfg, CFG["motion"], det, min_dwell_s=0.1)
        t, moves = 0.0, 0
        for i in range(400):
            out = fsm.update(det.process(depth_frame(0.5, 0.6, 2.5) if i > 5 else depth_frame()), t, True)
            if out["action"] and out["action"]["kind"] == "move":
                moves += 1
                fsm.action_done(t)                          # turning never helps
            t += 0.1
        self.assertEqual(moves, ocfg["max_consecutive_turns"])
        self.assertEqual(fsm.state, "STOP_BLOCKED")


class DockingTests(unittest.TestCase):
    def marker_scene(self, cx=320, size=120, w=640, h=480, marker_id=0):
        img = np.full((h, w, 3), 200, np.uint8)
        m = D.generate_marker("DICT_4X4_50", marker_id, size)
        m = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
        x0, y0 = int(cx - size / 2), int(h / 2 - size / 2)
        img[y0:y0 + size, x0:x0 + size] = m
        return img

    def test_aruco_detects_generated_marker(self):
        det = D.ArucoDetector(CFG["docking"])
        obs = det.observe(self.marker_scene(cx=320))
        self.assertTrue(obs.detected)
        self.assertAlmostEqual(obs.offset, 0.0, delta=0.03)
        self.assertIsNone(obs.distance_m)                   # no depth, no intrinsics -> no invented distance

    def test_offset_sign_and_wrong_id(self):
        det = D.ArucoDetector(CFG["docking"])
        self.assertLess(det.observe(self.marker_scene(cx=150)).offset, -0.4)
        self.assertGreater(det.observe(self.marker_scene(cx=500)).offset, 0.4)
        self.assertFalse(det.observe(self.marker_scene(marker_id=7)).detected)

    def test_depth_and_pnp_distance(self):
        cfg = dict(CFG["docking"])
        det = D.ArucoDetector(cfg)
        img = self.marker_scene(size=100)
        depth = np.full((480, 640), 1.25, np.float32)
        intr = Intrinsics(640, 480, 600.0, 600.0, 320.0, 240.0)
        obs = det.observe(img, depth, intr)
        self.assertEqual(obs.distance_src, "depth")
        self.assertAlmostEqual(obs.distance_m, 1.25, places=2)
        # marker 0.10 m printed, 100 px at fx=600 -> z = 0.1*600/100 = 0.6 m
        self.assertAlmostEqual(obs.pnp_distance_m, 0.6, delta=0.05)
        obs2 = det.observe(img, None, intr)
        self.assertEqual(obs2.distance_src, "pnp")

    def fsm_run(self, seq, can_act=True, executor=None):
        fsm = D.DockingFSM(dict(CFG["docking"]), CFG["motion"])
        t, outs = 0.0, []
        for o in seq:
            out = fsm.update(o, t, can_act)
            if out["action"] and executor:
                executor(out["action"], fsm, t)
            outs.append(out)
            t += 0.1
        return fsm, outs

    @staticmethod
    def obs(offset=0.0, dist=1.5, detected=True):
        o = D.MarkerObservation()
        o.detected = detected
        if detected:
            o.offset, o.distance_m, o.distance_src = offset, dist, "depth"
        return o

    def test_state_progression_perception_only(self):
        fsm, outs = self.fsm_run([self.obs(0.0, 1.5)] * 30, can_act=False)
        states = [o["state"] for o in outs]
        self.assertEqual(states[0], "SEARCH")
        self.assertIn("MARKER_DETECTED", states)
        self.assertIn("ALIGN", states)
        self.assertEqual(states[-1], "APPROACH")
        self.assertEqual(outs[-1]["decision"], "TOO_FAR")
        self.assertTrue(all(o["action"] is None for o in outs))

    def test_alignment_decisions_and_deadband(self):
        fsm, outs = self.fsm_run([self.obs(-0.5, 1.5)] * 20, can_act=False)
        self.assertEqual(outs[-1]["decision"], "LEFT")
        fsm, outs = self.fsm_run([self.obs(0.5, 1.5)] * 20, can_act=False)
        self.assertEqual(outs[-1]["decision"], "RIGHT")
        fsm, outs = self.fsm_run([self.obs(0.05, 1.5)] * 20, can_act=False)
        self.assertEqual(outs[-1]["decision"], "TOO_FAR")  # inside deadband -> centered -> approaching

    def test_no_blind_approach_without_distance(self):
        o = self.obs(0.0, None)
        fsm, outs = self.fsm_run([o] * 30, can_act=True)
        self.assertEqual(outs[-1]["decision"], "NO_DISTANCE")
        self.assertTrue(all(x["action"] is None for x in outs))

    def test_marker_lost_stops_and_returns_to_search(self):
        seq = [self.obs(0.0, 1.5)] * 15 + [self.obs(detected=False)] * 12
        fsm, outs = self.fsm_run(seq, can_act=True, executor=lambda a, f, t: f.action_done(t))
        self.assertEqual(outs[-1]["state"], "SEARCH")
        self.assertEqual(outs[-1]["decision"], "NO_MARKER")
        self.assertIsNone(outs[-1]["action"])               # never drives when marker is lost

    def test_docks_when_close_enough_and_holds(self):
        seq = [self.obs(0.0, 1.5)] * 15 + [self.obs(0.0, 0.35)] * 10
        fsm, outs = self.fsm_run(seq, can_act=False)
        self.assertEqual(outs[-1]["state"], "DOCKED")
        self.assertEqual(outs[-1]["progress"], 1.0)

    def test_closed_loop_with_executor_reaches_dock(self):
        # fake kinematics: each approach pulse reduces distance by 0.2 m; each turn pulse shifts offset by 0.12
        state = {"off": -0.5, "dist": 1.6}
        cmds = []

        def executor(a, fsm, t):
            cmds.append(a["dir"])
            if a["dir"] == "L":
                state["off"] += 0.12
            elif a["dir"] == "R":
                state["off"] -= 0.12
            elif a["dir"] == "F":
                state["dist"] -= 0.2
            fsm.action_done(t)

        fsm = D.DockingFSM(dict(CFG["docking"]), CFG["motion"])
        t = 0.0
        for _ in range(600):
            out = fsm.update(self.obs(state["off"], state["dist"]), t, True)
            if out["action"]:
                executor(out["action"], fsm, t)
            t += 0.1
            if out["state"] == "DOCKED":
                break
        self.assertEqual(fsm.state, "DOCKED")
        self.assertIn("L", cmds)
        self.assertIn("F", cmds)
        self.assertLessEqual(state["dist"], CFG["docking"]["dock_distance_m"])
        self.assertLess(fsm.actions_used, CFG["docking"]["max_actions"])

    def test_action_limit_aborts(self):
        fsm = D.DockingFSM(dict(CFG["docking"], max_actions=3), CFG["motion"])
        t = 0.0
        for _ in range(300):
            out = fsm.update(self.obs(-0.6, 1.5), t, True)
            if out["action"]:
                fsm.action_done(t)                          # never converges
            t += 0.1
        self.assertEqual(fsm.state, "ABORT")


class PointCloudTests(unittest.TestCase):
    def test_flat_wall(self):
        intr = Intrinsics(160, 120, 100.0, 100.0, 80.0, 60.0)
        depth = np.full((120, 160), 2.0, np.float32)
        pts, cols = PC.depth_to_points(depth, intr, stride=8)
        self.assertTrue(np.allclose(pts[:, 2], 2.0))
        self.assertAlmostEqual(float(pts[:, 0].min()), (0 - 80) / 100.0 * 2.0, places=2)
        self.assertLessEqual(len(pts), 6000)

    def test_invalid_depth_excluded(self):
        intr = Intrinsics(160, 120, 100.0, 100.0, 80.0, 60.0)
        pts, _ = PC.depth_to_points(np.zeros((120, 160), np.float32), intr)
        self.assertEqual(len(pts), 0)


if __name__ == "__main__":
    unittest.main()
