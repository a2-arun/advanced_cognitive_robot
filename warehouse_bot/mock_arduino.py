"""Software model of firmware/warehouse_bot/warehouse_bot.ino for testing WITHOUT hardware.

It exposes the same minimal interface as pyserial.Serial (write / read / in_waiting / close) so the real
RobotLink code path (framing, checksums, watchdog, telemetry) is exercised.

IMPORTANT: this is a *model*. Passing tests against it proves the Python side and the protocol logic, NOT
that the .ino compiles or that the wiring is right. Encoder counts here are synthetic.
"""
import threading
import time
from collections import deque

MAX_PULSE_MS = 1200
MIN_PULSE_MS = 20
COMM_TIMEOUT_MS = 700
ARM_IDLE_TIMEOUT_MS = 10000
MIN_GAP_MS = 150
BRAKE_MS = 80
TELEMETRY_PERIOD_MS = 100
FAKE_COUNTS_PER_S = 900.0


class MockArduino(object):
    def __init__(self, boot_delay_s=0.0):
        self._rx = bytearray()
        self._tx = deque()
        self._tx_buf = bytearray()
        self._lock = threading.RLock()
        self._alive = True
        self._t0 = time.time()
        self.armed = False
        self.motion = "IDLE"
        self.move_dir = "F"
        self.move_seq = 0
        self.move_end = 0.0
        self.brake_end = 0.0
        self.last_stop = 0.0
        self.last_rx = time.time()
        self.last_telem = 0.0
        self.enc = [0.0, 0.0, 0.0, 0.0]   # TL, TR, BR, BL
        self.motor_levels = (0, 0)        # (left, right) in {-1,0,1}: what would be on the L298N inputs
        self.ignore_input = False         # test hook: drop everything the host sends
        self.log = []                     # every command line accepted
        self._last_tick = time.time()
        self._emit("EVT BOOT wb-0.2-mock")
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    # ---- pyserial-like interface -------------------------------------------------
    @property
    def in_waiting(self):
        with self._lock:
            return len(self._tx_buf)

    def write(self, data):
        with self._lock:
            if self.ignore_input:
                return len(data)
            self._rx.extend(data)
            while b"\n" in self._rx:
                i = self._rx.index(b"\n")
                line = bytes(self._rx[:i]).decode("ascii", "replace").replace("\r", "")
                del self._rx[:i + 1]
                if line:
                    self._handle(line)
        return len(data)

    def read(self, n=1):
        with self._lock:
            out = bytes(self._tx_buf[:n])
            del self._tx_buf[:n]
        if not out:
            time.sleep(0.005)
        return out

    def readline(self):
        with self._lock:
            if b"\n" in self._tx_buf:
                i = self._tx_buf.index(b"\n")
                out = bytes(self._tx_buf[:i + 1])
                del self._tx_buf[:i + 1]
                return out
        time.sleep(0.01)
        return b""

    def reset_input_buffer(self):
        with self._lock:
            self._tx_buf.clear()

    def flush(self):
        pass

    def close(self):
        self._alive = False

    # ---- firmware model ----------------------------------------------------------
    def _emit(self, text):
        with self._lock:
            self._tx_buf.extend((text + "\n").encode("ascii"))

    def _set_motors(self, l, r):
        self.motor_levels = (l, r)

    def _stop_motors(self, now):
        was = self.motion == "MOVING"
        self._set_motors(0, 0)
        self.brake_end = now + BRAKE_MS / 1000.0
        self.last_stop = now
        self.motion = "BRAKE"
        if was:
            self._emit("EVT DONE %d %s %d %d %d %d" % ((self.move_seq, self.move_dir) + tuple(int(x) for x in self.enc)))

    def _handle(self, line):
        now = time.time()
        if "*" in line:
            body, _, hh = line.rpartition("*")
            calc = 0
            for ch in body:
                calc ^= ord(ch)
            try:
                ok = len(hh) == 2 and int(hh, 16) == calc
            except ValueError:
                ok = False
            if not ok:
                self._emit("ERR 0 CHK")
                return
            line = body
        seq = 0
        cmd = line
        if ":" in line:
            s, _, cmd = line.partition(":")
            try:
                seq = int(s)
            except ValueError:
                seq = 0
        toks = cmd.split()
        if not toks:
            self._emit("ERR %d EMPTY" % seq)
            return
        self.last_rx = now
        self.log.append(cmd)
        t = toks[0]
        if t == "HB":
            return
        if t in ("S", "STOP"):
            if self.motion == "MOVING":
                self._stop_motors(now)
            else:
                self._set_motors(0, 0)
            self._emit("OK %d STOPPED" % seq)
        elif t == "PING":
            self._emit("OK %d PONG wb-0.2-mock" % seq)
        elif t == "ARM":
            self.armed = True
            self._emit("OK %d ARMED" % seq)
        elif t == "DISARM":
            if self.motion == "MOVING":
                self._stop_motors(now)
            self.armed = False
            self._emit("OK %d DISARMED" % seq)
        elif t == "ER":
            self.enc = [0.0, 0.0, 0.0, 0.0]
            self._emit("OK %d ER" % seq)
        elif t == "PINS":
            self._emit("OK %d PINS L_IN=4,5 R_IN=2,3 ENC(B,A)=TL:6,7 TR:8,9 BR:10,11 BL:12,13" % seq)
        elif t == "M":
            if len(toks) != 3 or len(toks[1]) != 1:
                self._emit("ERR %d ARGS" % seq)
                return
            try:
                ms = int(toks[2])
            except ValueError:
                self._emit("ERR %d ARGS" % seq)
                return
            d = toks[1]
            if d not in "FBLR":
                self._emit("ERR %d DIR" % seq)
            elif ms < MIN_PULSE_MS or ms > MAX_PULSE_MS:
                self._emit("ERR %d RANGE" % seq)
            elif not self.armed:
                self._emit("ERR %d NOTARMED" % seq)
            elif self.motion != "IDLE":
                self._emit("ERR %d BUSY" % seq)
            elif (now - self.last_stop) * 1000.0 < MIN_GAP_MS:
                self._emit("ERR %d GAP" % seq)
            else:
                self._set_motors(*{"F": (1, 1), "B": (-1, -1), "L": (-1, 1), "R": (1, -1)}[d])
                self.motion = "MOVING"
                self.move_dir = d
                self.move_seq = seq
                self.move_end = now + ms / 1000.0
                self._emit("OK %d M %s %d" % (seq, d, ms))
        else:
            self._emit("ERR %d UNKNOWN" % seq)

    def _run(self):
        while self._alive:
            time.sleep(0.004)
            with self._lock:
                now = time.time()
                dt = now - self._last_tick
                self._last_tick = now
                l, r = self.motor_levels
                for i, lvl in enumerate((l, r, r, l)):      # TL, TR, BR, BL
                    self.enc[i] += lvl * FAKE_COUNTS_PER_S * dt
                if self.motion == "MOVING":
                    if now >= self.move_end:
                        self._stop_motors(now)
                    elif (now - self.last_rx) * 1000.0 > COMM_TIMEOUT_MS:
                        self._stop_motors(now)
                        self.armed = False
                        self._emit("EVT TIMEOUT")
                elif self.motion == "BRAKE":
                    if now >= self.brake_end:
                        self.motion = "IDLE"
                elif self.armed and (now - self.last_rx) * 1000.0 > ARM_IDLE_TIMEOUT_MS:
                    self.armed = False
                    self._emit("EVT DISARM_IDLE")
                if (now - self.last_telem) * 1000.0 >= TELEMETRY_PERIOD_MS:
                    self.last_telem = now
                    self._emit("T %d %s %d %d %d %d %d" % ((int((now - self._t0) * 1000), self.motion,
                                                         1 if self.armed else 0) + tuple(int(x) for x in self.enc)))
