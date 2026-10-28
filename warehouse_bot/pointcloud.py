"""Single-frame camera point cloud from depth + intrinsics.

This is a SNAPSHOT of what the camera sees in the camera frame (x right, y down, z forward, metres).
It is not a map, not registered over time, and not SLAM.
"""
import numpy as np


def depth_to_points(depth, intr, color=None, stride=4, zmin=0.3, zmax=4.0, max_points=6000):
    h, w = depth.shape[:2]
    K = intr.scaled(w, h) if (intr.width != w or intr.height != h) else intr
    ys, xs = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[0:h:stride, 0:w:stride]
    ok = np.isfinite(z) & (z >= zmin) & (z <= zmax)
    z, xs, ys = z[ok], xs[ok].astype(np.float32), ys[ok].astype(np.float32)
    x = (xs - K.ppx) / K.fx * z
    y = (ys - K.ppy) / K.fy * z
    pts = np.stack([x, y, z], axis=1).astype(np.float32)
    cols = None
    if color is not None:
        ch, cw = color.shape[:2]
        sy = np.clip((ys * ch / float(h)).astype(int), 0, ch - 1)
        sx = np.clip((xs * cw / float(w)).astype(int), 0, cw - 1)
        cols = color[sy, sx][:, ::-1].copy()      # BGR -> RGB
    if len(pts) > max_points:
        idx = np.random.RandomState(1).choice(len(pts), max_points, replace=False)
        pts = pts[idx]
        cols = None if cols is None else cols[idx]
    return pts, cols


def save_ply(path, pts, cols=None, comment=""):
    n = len(pts)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\ncomment %s\nelement vertex %d\n" % (comment or "camera point cloud", n))
        f.write("property float x\nproperty float y\nproperty float z\n")
        if cols is not None:
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for i in range(n):
            if cols is not None:
                f.write("%.4f %.4f %.4f %d %d %d\n" % (pts[i][0], pts[i][1], pts[i][2], cols[i][0], cols[i][1], cols[i][2]))
            else:
                f.write("%.4f %.4f %.4f\n" % tuple(pts[i]))
