import os
import sys
import unittest

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)

from warehouse_bot import camera
from warehouse_bot.config import load_config


class CameraSelectionTests(unittest.TestCase):
    def setUp(self):
        self._orig = camera.list_webcams
        self.calls = 0

    def tearDown(self):
        camera.list_webcams = self._orig

    def fake(self, cams):
        def f(max_index=6):
            self.calls += 1
            return cams
        camera.list_webcams = f

    def test_explicit_index_is_used_without_probing(self):
        self.fake([(0, 640, 480), (1, 640, 480)])
        self.assertEqual(camera.resolve_webcam_index(1), 1)
        self.assertEqual(self.calls, 0)

    def test_two_cameras_and_no_choice_refuses_instead_of_guessing(self):
        self.fake([(0, 640, 480), (1, 640, 480)])
        with self.assertRaises(camera.SourceError) as c:
            camera.resolve_webcam_index(None)
        self.assertIn("--webcam-index", str(c.exception))
        self.assertIn("list_cameras", str(c.exception))

    def test_single_camera_is_used(self):
        self.fake([(3, 640, 480)])
        self.assertEqual(camera.resolve_webcam_index(None), 3)

    def test_no_camera_is_an_error(self):
        self.fake([])
        with self.assertRaises(camera.SourceError):
            camera.resolve_webcam_index(None)

    def test_open_source_webcam_with_two_cameras_does_not_open_the_laptop_camera(self):
        self.fake([(0, 640, 480), (1, 640, 480)])
        cfg = load_config()
        src, msg = camera.open_source("webcam", cfg)
        self.assertIsNone(src)
        self.assertIn("--webcam-index", msg)

    def test_auto_does_not_silently_fall_back_to_the_laptop_camera(self):
        self.fake([(0, 640, 480), (1, 640, 480)])
        cfg = load_config()
        src, msg = camera.open_source("auto", cfg)       # no RealSense on this machine
        self.assertIsNone(src)

    def test_default_config_has_no_preselected_webcam(self):
        self.assertIsNone(load_config()["camera"]["webcam_index"])


if __name__ == "__main__":
    unittest.main()
