#!/usr/bin/env python3
"""Priority-1 inspection. Run this ON THE JETSON before installing anything:

    python3 tools/probe_environment.py

It only READS: versions, memory, disk, serial ports, OpenCV/ArUco, pyserial, RealSense library + camera streams.
It installs nothing and changes nothing. The report is saved to results/environment_report.txt.
Standard library only; works on Python 3.6.
"""
import glob
import os
import platform
import shutil
import sys
import time

lines = []


def out(s=""):
    print(s)
    lines.append(s)


def read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except Exception:
        return None


def section(t):
    out("")
    out("== %s ==" % t)


out("Warehouse robot environment probe  %s" % time.strftime("%Y-%m-%d %H:%M:%S"))

section("System")
out("python        : %s  (%s)" % (sys.version.split()[0], sys.executable))
out("platform      : %s %s" % (platform.system(), platform.machine()))
out("os-release    : %s" % ((read("/etc/os-release") or "n/a").split("\n")[0]))
out("jetpack/L4T   : %s" % (read("/etc/nv_tegra_release") or "n/a (not a Jetson, or file missing)"))
out("model         : %s" % (read("/proc/device-tree/model") or "n/a"))
mem = read("/proc/meminfo")
if mem:
    d = dict(l.split(":", 1) for l in mem.split("\n") if ":" in l)
    out("RAM total/avail: %s / %s" % (d.get("MemTotal", "?").strip(), d.get("MemAvailable", "?").strip()))
    out("swap total    : %s" % d.get("SwapTotal", "?").strip())
try:
    u = shutil.disk_usage(os.getcwd())
    out("disk free     : %.1f GB of %.1f GB" % (u.free / 1e9, u.total / 1e9))
except Exception as e:
    out("disk          : %s" % e)

section("Python packages")
for name in ("numpy", "cv2", "serial", "pyrealsense2", "flask"):
    try:
        m = __import__(name)
        ver = getattr(m, "__version__", None) or getattr(m, "VERSION", None) or "imported (no version attr)"
        out("%-13s: %s   [%s]" % (name, ver, getattr(m, "__file__", "")))
    except Exception as e:
        out("%-13s: MISSING (%s)" % (name, str(e)[:80]))

section("OpenCV / ArUco")
try:
    import cv2
    out("cv2 version   : %s" % cv2.__version__)
    has = hasattr(cv2, "aruco")
    out("cv2.aruco     : %s" % ("PRESENT" if has else "MISSING -> docking needs opencv-contrib (see README troubleshooting)"))
    if has:
        out("new API (ArucoDetector): %s" % hasattr(cv2.aruco, "ArucoDetector"))
        out("old API (detectMarkers): %s" % hasattr(cv2.aruco, "detectMarkers"))
    info = cv2.getBuildInformation()
    for key in ("CUDA", "GStreamer", "Video I/O"):
        for l in info.split("\n"):
            if l.strip().startswith(key + ":"):
                out("build %-10s: %s" % (key, l.split(":", 1)[1].strip()))
except Exception as e:
    out("OpenCV not usable: %s" % e)

section("Serial ports (Arduino)")
ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
out("device nodes  : %s" % (ports or "none found (cable? Arduino powered over USB?)"))
for p in ports:
    out("  %s  readable/writable by you: %s" % (p, os.access(p, os.R_OK | os.W_OK)))
    if not os.access(p, os.R_OK | os.W_OK):
        out("  -> fix once with:  sudo usermod -aG dialout $USER   then log out/in")
try:
    import serial.tools.list_ports as lp
    for p in lp.comports():
        out("  pyserial sees: %s | %s | %s" % (p.device, p.description, p.hwid))
except Exception:
    pass

section("RealSense")
try:
    import pyrealsense2 as rs
    out("pyrealsense2  : %s" % getattr(rs, "__version__", "?"))
    ctx = rs.context()
    devs = ctx.query_devices()
    out("devices found : %d" % len(devs))
    for dev in devs:
        out("  name     : %s" % dev.get_info(rs.camera_info.name))
        out("  serial   : %s" % dev.get_info(rs.camera_info.serial_number))
        out("  firmware : %s" % dev.get_info(rs.camera_info.firmware_version))
        try:
            out("  usb type : %s  (needs 3.x for depth+colour streams)" % dev.get_info(rs.camera_info.usb_type_descriptor))
        except Exception:
            pass
        for s in dev.query_sensors():
            prof = set()
            for p in s.get_stream_profiles():
                v = p.as_video_stream_profile() if p.is_video_stream_profile() else None
                if v and v.width() in (424, 480, 640, 848):
                    prof.add("%s %dx%d@%d" % (p.stream_name(), v.width(), v.height(), v.fps()))
            out("  sensor %-16s: %s" % (s.get_info(rs.camera_info.name), ", ".join(sorted(prof)[:8]) + (" ..." if len(prof) > 8 else "")))
    if len(devs) == 0:
        out("  -> no camera attached (borrowed camera not plugged in?). The dashboard will use SIMULATION or recorded data.")
except ImportError:
    out("pyrealsense2 not importable from this Python.")
    out("  -> The librealsense install may exist only for another Python, or without python bindings.")
    out("  -> See README 'RealSense on Jetson' (do not rebuild everything; try the quick checks first).")
except Exception as e:
    out("RealSense query failed: %s" % e)

section("Verdict")
out("Send / keep this file: results/environment_report.txt")
os.makedirs("results", exist_ok=True)
with open(os.path.join("results", "environment_report.txt"), "w") as f:
    f.write("\n".join(lines) + "\n")
