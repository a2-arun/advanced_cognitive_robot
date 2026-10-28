"""Host-side link to the Arduino: framing, checksums, acknowledgements, heartbeat, telemetry, history.

Safety properties implemented here (the firmware enforces its own independently):
  * commands are refused unless telemetry is fresh (link_ok)
  * a heartbeat is sent every serial.heartbeat_s so the Arduino watchdog stays satisfied; if this process
    dies or the cable is pulled, the Arduino stops the motors by itself (COMM_TIMEOUT_MS)
  * stop() never waits and never takes the command lock
"""
import threading
import time
from collections import deque

from . import protocol

try:
    import serial  # pyserial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - reported at connect time
    serial = None
    list_ports = None


class RobotLinkError(Exception):
    pass


def find_port():
    """Best-effort Arduino port discovery. Returns a device string or None."""
    if list_ports is None:
        return None
    ports = list(list_ports.comports())
    for p in ports:
        desc = ((p.description or "") + " " + (p.manufacturer or "")).lower()
        if "arduino" in desc or "ch340" in desc or "usb serial" in desc or "ftdi" in desc:
            return p.device
    for p in ports:
        if "ACM" in p.device or "ttyUSB" in p.device:
            return p.device
    return None


class RobotLink(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.scfg = cfg["serial"]
        self.mcfg = cfg["motion"]
        self._ser = None
        self.port = None
        self.is_mock = False
        self._wlock = threading.Lock()       # serialises writes
        self._cmd_lock = threading.Lock()    # serialises request/ack exchanges
        self._seq = 0
        self._pending = {}
        self._alive = False
        self._threads = []
        self.history = deque(maxlen=60)
        self.fw = None
        self.last_error = None
        self.armed = False
        self.state = "UNKNOWN"
        self.enc = [0, 0, 0, 0]              # TL, TR, BR, BL
        self.last_telem = 0.0
        self.booted_at = None
        self.timeouts = 0                    # EVT TIMEOUT count from firmware
        self.last_done = None
        self.hb_enabled = True               # test hook
        self.signs = list(self.scfg.get("encoder_signs", [1, 1, 1, 1]))

    # ------------------------------------------------------------------ connection
    def connect(self, port=None):
        self.disconnect()
        port = port or self.scfg.get("port", "auto")
        if port == "mock":
            from .mock_arduino import MockArduino
            self._ser = MockArduino()
            self.is_mock = True
            self.port = "mock"
        else:
            if serial is None:
                raise RobotLinkError("pyserial is not installed (pip3 install pyserial)")
            if port == "auto":
                port = find_port()
                if not port:
                    raise RobotLinkError("no serial port found; plug in the Arduino or pass --port")
            try:
                self._ser = serial.Serial(port, self.scfg["baud"], timeout=0.05, write_timeout=0.5)
            except Exception as e:
                raise RobotLinkError("cannot open %s: %s" % (port, e))
            self.is_mock = False
            self.port = port
            time.sleep(self.scfg["connect_wait_s"])  # Uno auto-reset on open
            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass
        self._alive = True
        self.last_error = None
        self.last_telem = 0.0
        self._note("connected to %s%s" % (self.port, " (MOCK FIRMWARE - no hardware)" if self.is_mock else ""))
        for target in (self._reader, self._heartbeat):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)
        # wait for the first telemetry frame
        t0 = time.time()
        while time.time() - t0 < 2.0 and self.last_telem == 0.0:
            time.sleep(0.02)
        if self.last_telem == 0.0:
            self.last_error = "connected but no telemetry (wrong port, wrong firmware, or wrong baud?)"
            self._note(self.last_error, "error")
        else:
            r = self.send("PING")
            if r.get("kind") == "OK":
                self.fw = r["text"].replace("PONG", "").strip()

    def disconnect(self):
        was = self._ser is not None
        if was:
            try:
                self.stop()
                self._write_raw(protocol.encode(self._next_seq(), "DISARM"))
            except Exception:
                pass
        self._alive = False
        for t in self._threads:
            t.join(timeout=0.5)
        self._threads = []
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self.armed = False
        self.state = "UNKNOWN"
        self.last_telem = 0.0
        if was:
            self._note("disconnected")

    # ------------------------------------------------------------------ status
    @property
    def link_ok(self):
        return (self._ser is not None and self._alive and self.last_telem > 0.0 and
                (time.time() - self.last_telem) < self.scfg["telemetry_timeout_s"])

    def status(self):
        age = (time.time() - self.last_telem) if self.last_telem else None
        return {
            "connected": self._ser is not None,
            "link_ok": self.link_ok,
            "port": self.port,
            "mock": self.is_mock,
            "fw": self.fw,
            "armed": bool(self.armed and self.link_ok),
            "state": self.state if self.link_ok else "UNKNOWN",
            "enc": list(self.enc),
            "enc_names": list(protocol.ENC_NAMES),
            "enc_l": protocol.side_counts(self.enc)[0],
            "enc_r": protocol.side_counts(self.enc)[1],
            "telemetry_age_s": None if age is None else round(age, 2),
            "firmware_timeouts": self.timeouts,
            "last_error": self.last_error,
            "history": list(self.history)[-25:],
        }

    def is_busy(self):
        return self.state != "IDLE"

    # ------------------------------------------------------------------ commands
    def arm(self):
        return self._cmd("ARM", need_link=True)

    def disarm(self):
        return self._cmd("DISARM", need_link=False)

    def reset_encoders(self):
        return self._cmd("ER", need_link=True)

    def move(self, direction, ms=None):
        direction = str(direction).upper()
        if direction not in protocol.MOVE_DIRS:
            return {"ok": False, "error": "bad direction %r" % direction}
        ms = int(ms if ms is not None else self.mcfg["default_pulse_ms"])
        ms = max(self.mcfg["min_pulse_ms"], min(self.mcfg["max_pulse_ms"], ms))
        if not self.link_ok:
            self._note("REFUSED move %s %d: link down" % (direction, ms), "error")
            return {"ok": False, "error": "link down (no fresh telemetry)"}
        if not self.armed:
            return {"ok": False, "error": "not armed (press ARM first)"}
        r = self._cmd("M %s %d" % (direction, ms), need_link=True)
        r["ms"] = ms
        return r

    def stop(self):
        """Fire-and-forget emergency stop. Safe to call from any thread, any time."""
        ok = False
        for _ in range(2):
            try:
                self._write_raw(protocol.encode(self._next_seq(), "S"))
                ok = True
            except Exception as e:
                self.last_error = "stop write failed: %s" % e
        self._note("STOP sent" if ok else "STOP FAILED (no link)", "info" if ok else "error")
        return {"ok": ok}

    # ------------------------------------------------------------------ internals
    def _next_seq(self):
        self._seq = self._seq % 9999 + 1
        return self._seq

    def _note(self, text, level="info"):
        self.history.append({"t": time.strftime("%H:%M:%S"), "text": text, "level": level})

    def _write_raw(self, data):
        if self._ser is None:
            raise RobotLinkError("not connected")
        with self._wlock:
            self._ser.write(data)

    def send(self, cmd, timeout=None):
        """Send a command and wait for OK/ERR. Returns the reply dict or {"kind": "TIMEOUT"}."""
        timeout = self.scfg["ack_timeout_s"] if timeout is None else timeout
        with self._cmd_lock:
            seq = self._next_seq()
            ev = threading.Event()
            self._pending[seq] = [ev, None]
            try:
                self._write_raw(protocol.encode(seq, cmd))
            except Exception as e:
                self._pending.pop(seq, None)
                self.last_error = str(e)
                return {"kind": "TIMEOUT", "error": str(e)}
            ev.wait(timeout)
            slot = self._pending.pop(seq, None)
            return slot[1] if slot and slot[1] else {"kind": "TIMEOUT"}

    def _cmd(self, cmd, need_link):
        if need_link and not self.link_ok:
            return {"ok": False, "error": "link down (no fresh telemetry)"}
        r = self.send(cmd)
        ok = r.get("kind") == "OK"
        err = None if ok else (r.get("code") or r.get("error") or "no acknowledgement")
        self._note("%s -> %s" % (cmd, "OK" if ok else "ERR " + str(err)), "info" if ok else "error")
        return {"ok": ok, "error": err, "reply": r}

    def _reader(self):
        buf = bytearray()
        while self._alive:
            try:
                n = getattr(self._ser, "in_waiting", 0) or 1
                chunk = self._ser.read(n)
            except Exception as e:
                self.last_error = "serial read failed: %s" % e
                self._note(self.last_error, "error")
                self.last_telem = 0.0
                self._alive = False
                break
            if not chunk:
                continue
            buf.extend(chunk)
            while b"\n" in buf:
                i = buf.index(b"\n")
                raw = bytes(buf[:i])
                del buf[:i + 1]
                try:
                    msg = protocol.parse_line(raw.decode("ascii", "replace"))
                except Exception:
                    msg = None
                if msg:
                    self._dispatch(msg)
            if len(buf) > 512:
                del buf[:]

    def _dispatch(self, m):
        k = m["kind"]
        if k == "T":
            self.state = m["state"]
            self.armed = m["armed"]
            self.enc = [c * s for c, s in zip(m["enc"], self.signs)]
            self.last_telem = time.time()
        elif k in ("OK", "ERR"):
            slot = self._pending.get(m["seq"])
            if slot:
                slot[1] = m
                slot[0].set()
        elif k == "EVT":
            name = m["name"]
            if name == "TIMEOUT":
                self.timeouts += 1
                self._note("FIRMWARE WATCHDOG TIMEOUT: motors stopped, disarmed", "error")
            elif name == "DONE":
                self.last_done = m
                self._note("pulse done %s enc(TL,TR,BR,BL)=%s" % (m.get("dir"), [c * s for c, s in zip(m.get("enc", [0, 0, 0, 0]), self.signs)]))
            elif name == "BOOT":
                self._note("firmware boot: %s" % m["text"])
            else:
                self._note("EVT %s %s" % (name, m["text"]), "warn")

    def _heartbeat(self):
        while self._alive:
            time.sleep(self.scfg["heartbeat_s"])
            if self.hb_enabled and self._ser is not None:
                try:
                    self._write_raw(protocol.encode(self._next_seq(), "HB"))
                except Exception as e:
                    self.last_error = "heartbeat write failed: %s" % e
                    self.last_telem = 0.0
