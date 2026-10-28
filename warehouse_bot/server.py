"""Dashboard HTTP server (standard library only: runs on Python 3.6 with no web-framework install).

Binds to 127.0.0.1 by default. If you bind to a LAN address a random access token is generated and required for
every API call and video stream. Do NOT port-forward this to the internet: it can move a robot.
"""
import argparse
import json
import mimetypes
import os
import secrets
import signal
import socket
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from urllib.parse import urlparse, parse_qs

from .config import load_config
from .engine import Engine

WEB_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")


class ThreadedServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    # On Windows SO_REUSEADDR lets processes share/steal ports and can make bind() fail with WinError 10013.
    allow_reuse_address = (os.name != "nt")


class Handler(BaseHTTPRequestHandler):
    engine = None
    token = None
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        pass

    # ------------------------------------------------------------------ plumbing
    def _qs(self):
        return parse_qs(urlparse(self.path).query)

    def _authorized(self):
        if not self.token:
            return True
        given = self.headers.get("X-Token") or (self._qs().get("token") or [""])[0]
        return secrets.compare_digest(str(given), self.token)

    def _json(self, obj, code=200):
        body = json.dumps(obj, default=str).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _deny(self):
        self._json({"ok": False, "error": "unauthorized: open the URL printed by the server (it contains ?token=...)"},
                   401)

    # ------------------------------------------------------------------ GET
    def do_GET(self):
        path = urlparse(self.path).path
        try:
            if path.startswith("/api/") or path.startswith("/stream/"):
                if not self._authorized():
                    return self._deny()
                if path == "/api/state":
                    return self._json(self.engine.get_state())
                if path == "/api/pointcloud":
                    return self._json(self.engine.pointcloud(False))
                if path == "/api/log":
                    return self._json({"path": self.engine.logpath})
                if path.startswith("/stream/") and path.endswith(".mjpg"):
                    return self._stream(path[len("/stream/"):-len(".mjpg")])
                return self._json({"ok": False, "error": "not found"}, 404)
            self._static(path)
        except OSError:
            pass

    def _static(self, path):
        if path in ("", "/"):
            path = "/index.html"
        full = os.path.normpath(os.path.join(WEB_ROOT, path.lstrip("/")))
        if not full.startswith(WEB_ROOT) or not os.path.isfile(full):
            self.send_response(404)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        with open(full, "rb") as f:
            data = f.read()
        self.send_response(200)
        self.send_header("Content-Type", mimetypes.guess_type(full)[0] or "application/octet-stream")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(data)

    def _stream(self, name):
        eng = self.engine
        if name not in eng.jpgs:
            return self._json({"ok": False, "error": "unknown stream"}, 404)
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        seq = -1
        while True:
            with eng.jpg_cv:
                eng.jpg_cv.wait_for(lambda: eng.jpg_seq != seq, timeout=3.0)
                seq = eng.jpg_seq
                data = eng.jpgs.get(name)
            if not data:
                continue
            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(data)).encode() +
                             b"\r\n\r\n" + data + b"\r\n")

    # ------------------------------------------------------------------ POST
    def do_POST(self):
        path = urlparse(self.path).path
        try:
            if not self._authorized():
                return self._deny()
            n = int(self.headers.get("Content-Length") or 0)
            if n > 4096:
                return self._json({"ok": False, "error": "body too large"}, 413)
            try:
                body = json.loads(self.rfile.read(n).decode("utf-8")) if n else {}
            except ValueError:
                return self._json({"ok": False, "error": "bad json"}, 400)
            return self._json(self._route(path, body))
        except OSError:
            pass
        except Exception as e:
            self._json({"ok": False, "error": "server error: %s" % e}, 500)

    def _route(self, path, b):
        e = self.engine
        if path == "/api/stop":
            return e.stop_all()
        if path == "/api/mode":
            return e.set_mode(b.get("mode"))
        if path == "/api/move":
            return e.teleop(str(b.get("dir", "")).upper(), b.get("ms"))
        if path == "/api/connect":
            return e.connect(b.get("port") or None)
        if path == "/api/disconnect":
            return e.disconnect()
        if path == "/api/arm":
            return e.arm()
        if path == "/api/disarm":
            return e.disarm()
        if path == "/api/behavior":
            return e.set_behavior(b.get("behavior", "off"), bool(b.get("act")), b.get("advance"))
        if path == "/api/sim":
            return e.sim_control(b.get("action"), b.get("x"), b.get("y"))
        if path == "/api/mission":
            if b.get("action") == "clear":
                return e.stop_all("mission cleared")
            return e.start_mission(b.get("x"), b.get("y"))
        if path == "/api/source":
            return e.open_source(b.get("source", "auto"))
        if path == "/api/snapshot":
            return e.snapshot()
        if path == "/api/record":
            return e.recording(b.get("action"), b.get("stream"))
        if path == "/api/pointcloud/save":
            r = e.pointcloud(True)
            r.pop("points", None)
            r.pop("colors", None)
            return r
        return {"ok": False, "error": "unknown endpoint"}


def lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "<jetson-ip>"


def lan_addresses():
    """All usable IPv4 addresses of this machine (Linux: `hostname -I`), Docker's bridge excluded."""
    try:
        import subprocess
        out = subprocess.check_output(["hostname", "-I"], universal_newlines=True, timeout=3).split()
        ips = [i for i in out if "." in i and not i.startswith("172.17.")]
        if ips:
            return ips
    except Exception:
        pass
    return [lan_ip()]


def build_server(cfg, host=None, port=None, token=None):
    host = host or cfg["server"]["host"]
    port = port if port is not None else cfg["server"]["port"]
    local = host in ("127.0.0.1", "localhost", "::1")
    if not local and token is None:
        token = secrets.token_urlsafe(9)
    handler = type("BoundHandler", (Handler,), {"engine": None, "token": token})
    httpd, last = None, None
    for p in ([0] if port == 0 else range(port, port + 20)):     # port taken/reserved? try the next ones
        try:
            httpd = ThreadedServer((host, p), handler)
            break
        except OSError as e:
            last = e
    if httpd is None:
        raise last
    if port and httpd.server_address[1] != port:
        print("NOTE: port %d is not available (%s); using %d instead." % (port, last, httpd.server_address[1]))
    eng = Engine(cfg)
    handler.engine = eng
    return eng, httpd, token


def main(argv=None):
    ap = argparse.ArgumentParser(description="Warehouse robot dashboard (simulation + hardware)")
    ap.add_argument("--config", help="JSON file overriding defaults")
    ap.add_argument("--host", help="default 127.0.0.1. Use 0.0.0.0 to reach it from your laptop over Wi-Fi "
                                   "(a token is then required)")
    ap.add_argument("--port", type=int)
    ap.add_argument("--mode", choices=["sim", "hardware"], default="sim")
    ap.add_argument("--serial-port", help="e.g. /dev/ttyACM0, or 'mock' for the software test firmware")
    ap.add_argument("--camera", help="auto|realsense|webcam|video:<file>|session:<dir>|none")
    ap.add_argument("--results", help="results folder (default ./results)")
    a = ap.parse_args(argv)

    cfg = load_config(a.config)
    if a.results:
        cfg["server"]["results_dir"] = a.results
    if a.camera:
        cfg["camera"]["source"] = a.camera
    eng, httpd, token = build_server(cfg, a.host, a.port)
    eng.start()
    if a.mode == "hardware":
        eng.set_mode("hardware")
        if a.serial_port:
            r = eng.connect(a.serial_port)
            print("serial:", "connected" if r["ok"] else "FAILED - %s" % r["error"])
    host, port = httpd.server_address[0], httpd.server_address[1]
    shown = lan_addresses() if host == "0.0.0.0" else [host]
    urls = ["http://%s:%d/" % (h, port) + ("?token=%s" % token if token else "") for h in shown]
    print("=" * 70)
    print("Dashboard running (%s mode). Open ONE of:" % a.mode.upper())
    for u in urls:
        print("    %s" % u)
    print("Results are saved under: %s" % os.path.abspath(cfg["server"]["results_dir"]))
    if token:
        print("LAN access is ON: only people with this token URL can control the robot.")
        print("Never expose this port to the internet.")
    print("Ctrl+C to quit (motors are stopped on exit).")
    print("=" * 70)

    def bye(*_):
        print("\nstopping...")
        threading.Thread(target=httpd.shutdown, daemon=True).start()
    signal.signal(signal.SIGINT, bye)
    try:
        httpd.serve_forever()
    finally:
        eng.shutdown()
        httpd.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
