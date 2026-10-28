import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from warehouse_bot import protocol
from warehouse_bot.config import load_config
from warehouse_bot.serial_robot import RobotLink


class ProtocolTests(unittest.TestCase):
    def test_encode_checksum(self):
        b = protocol.encode(5, "M F 250")
        self.assertTrue(b.endswith(b"\n"))
        body, _, hh = b.decode().strip().rpartition("*")
        self.assertEqual(int(hh, 16), protocol.checksum(body))

    def test_parse(self):
        self.assertEqual(protocol.parse_line("T 1200 IDLE 1 10 -4 5 6")["enc"], [10, -4, 5, 6])
        self.assertEqual(protocol.parse_line("ERR 7 BUSY")["code"], "BUSY")
        self.assertEqual(protocol.parse_line("EVT DONE 3 F 100 101 99 98")["enc"][1], 101)
        self.assertIsNone(protocol.parse_line("garbage ###"))
        self.assertIsNone(protocol.parse_line("T 1 2"))
        self.assertIsNone(protocol.parse_line("T 1200 IDLE 1 10 -4"))   # old 2-encoder format rejected


class LinkTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config()
        self.link = RobotLink(self.cfg)
        self.link.connect("mock")
        self.mock = self.link._ser

    def tearDown(self):
        self.link.disconnect()

    def wait_idle(self, t=3.0):
        t0 = time.time()
        while time.time() - t0 < t:
            if self.link.state == "IDLE":
                return True
            time.sleep(0.02)
        return False

    def test_link_up_and_ping(self):
        self.assertTrue(self.link.link_ok)
        self.assertIn("mock", self.link.fw)

    def test_move_refused_when_not_armed(self):
        r = self.link.move("F", 200)
        self.assertFalse(r["ok"])
        self.assertEqual(self.mock.motor_levels, (0, 0))

    def test_arm_move_done_and_encoders(self):
        self.assertTrue(self.link.arm()["ok"])
        time.sleep(0.15)
        self.assertTrue(self.link.armed)
        r = self.link.move("F", 300)
        self.assertTrue(r["ok"], r)
        time.sleep(0.1)
        self.assertEqual(self.mock.motor_levels, (1, 1))
        time.sleep(0.45)
        self.assertEqual(self.mock.motor_levels, (0, 0))   # firmware ended the pulse itself
        time.sleep(0.3)
        self.assertTrue(all(c > 100 for c in self.link.enc), self.link.enc)
        self.assertIsNotNone(self.link.last_done)

    def test_right_side_raw_counts_are_corrected_by_encoder_signs(self):
        self.link.arm()
        time.sleep(0.15)
        self.assertTrue(self.link.move("F", 300)["ok"])
        time.sleep(0.8)
        self.assertLess(self.mock.enc[1], 0)          # raw (mirrored) right encoder counts down
        self.assertGreater(self.link.enc[1], 0)       # corrected value counts up like the left side

    def test_turn_signs(self):
        self.link.arm()
        time.sleep(0.15)
        self.assertTrue(self.link.move("L", 200)["ok"])
        time.sleep(0.5)
        tl, tr, br, bl = self.link.enc
        self.assertTrue(tl < 0 and bl < 0 and tr > 0 and br > 0, self.link.enc)

    def test_pulse_clamped_by_host_and_rejected_by_firmware(self):
        self.link.arm()
        time.sleep(0.15)
        r = self.link.move("F", 99999)
        self.assertEqual(r["ms"], self.cfg["motion"]["max_pulse_ms"])
        self.link.stop()
        time.sleep(0.4)
        raw = self.link.send("M F 5000")        # bypass host clamp: firmware must refuse
        self.assertEqual(raw.get("code"), "RANGE")

    def test_bad_direction_and_unknown_command(self):
        self.assertFalse(self.link.move("X", 100)["ok"])
        self.assertEqual(self.link.send("FLY").get("code"), "UNKNOWN")
        self.assertEqual(self.link.send("M Q 100").get("code"), "DIR")

    def test_bad_checksum_rejected(self):
        before = len(self.mock.log)
        self.mock.write(b"1:M F 100*00\n")
        time.sleep(0.1)
        self.assertEqual(len(self.mock.log), before)
        self.assertEqual(self.mock.motor_levels, (0, 0))

    def test_stop_interrupts_motion(self):
        self.link.arm()
        time.sleep(0.15)
        self.assertTrue(self.link.move("F", 800)["ok"])
        time.sleep(0.15)
        self.assertEqual(self.mock.motor_levels, (1, 1))
        self.link.stop()
        time.sleep(0.1)
        self.assertEqual(self.mock.motor_levels, (0, 0))

    def test_watchdog_stops_motors_when_host_goes_silent(self):
        self.link.arm()
        time.sleep(0.15)
        self.link.hb_enabled = False            # simulate host process hanging
        self.assertTrue(self.link.move("F", 1200)["ok"])
        time.sleep(0.2)
        self.assertEqual(self.mock.motor_levels, (1, 1))
        time.sleep(1.0)                          # > 700 ms comm timeout, < 1200 ms pulse
        self.assertEqual(self.mock.motor_levels, (0, 0))
        self.assertFalse(self.mock.armed)        # firmware disarmed itself
        self.assertGreaterEqual(self.link.timeouts, 1)

    def test_commands_refused_when_telemetry_stale(self):
        self.link.arm()
        time.sleep(0.15)
        self.mock.ignore_input = True
        self.mock._alive = False                 # kill telemetry
        time.sleep(1.3)
        self.assertFalse(self.link.link_ok)
        r = self.link.move("F", 100)
        self.assertFalse(r["ok"])
        self.assertIn("link down", r["error"])

    def test_busy_while_moving(self):
        self.link.arm()
        time.sleep(0.15)
        self.assertTrue(self.link.move("F", 600)["ok"])
        r = self.link.move("B", 100)
        self.assertFalse(r["ok"])
        self.assertEqual(r["error"], "BUSY")


if __name__ == "__main__":
    unittest.main()
