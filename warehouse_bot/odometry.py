"""Encoder dead reckoning. This is NOT SLAM and NOT a map: it integrates wheel counts into a pose estimate that
drifts (wheel slip on a skid-steer chassis is large when turning).

Needs calibration (config odometry.counts_per_m and odometry.effective_track_m). Without it we only report counts
and refuse to produce a pose.
"""
import math

from . import protocol


class Odometry(object):
    def __init__(self, ocfg):
        self.cpm = ocfg.get("counts_per_m")
        self.track = ocfg.get("effective_track_m")
        self.reset()

    @property
    def calibrated(self):
        return bool(self.cpm) and bool(self.track)

    def reset(self):
        self.x = self.y = self.th = 0.0
        self.trail = [(0.0, 0.0)]
        self._last = None

    def update(self, enc):
        """enc = [TL, TR, BR, BL] cumulative counts."""
        l, r = protocol.side_counts(enc)
        if self._last is None:
            self._last = (l, r)
            return
        dl, dr = l - self._last[0], r - self._last[1]
        self._last = (l, r)
        if not self.calibrated or (dl == 0 and dr == 0):
            return
        dl, dr = dl / self.cpm, dr / self.cpm
        ds, dth = (dl + dr) / 2.0, (dr - dl) / self.track
        self.x += ds * math.cos(self.th + dth / 2.0)
        self.y += ds * math.sin(self.th + dth / 2.0)
        self.th += dth
        self.trail.append((self.x, self.y))
        if len(self.trail) > 2000:
            del self.trail[:500]

    def state(self):
        return {"calibrated": self.calibrated, "x": round(self.x, 3), "y": round(self.y, 3),
                "th": round(self.th, 3), "trail": [(round(a, 3), round(b, 3)) for a, b in self.trail[-500:]],
                "label": "encoder dead reckoning (NOT SLAM, drifts)"}
