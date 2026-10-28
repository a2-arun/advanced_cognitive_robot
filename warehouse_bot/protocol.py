"""Serial protocol shared by the Python host, the Arduino firmware and the mock firmware.

Host -> Arduino:   "<seq>:<CMD> [args]*HH\n"      HH = XOR of every character before '*', 2 hex digits
Arduino -> host:   "OK <seq> ..." | "ERR <seq> <CODE>" | "EVT ..." | "T <ms> <state> <armed> <TL> <TR> <BR> <BL>"
"""

MOVE_DIRS = ("F", "B", "L", "R")
DIR_NAMES = {"F": "forward", "B": "backward", "L": "left", "R": "right"}
MAX_LINE = 47
ENC_NAMES = ("TL", "TR", "BR", "BL")   # top-left, top-right, bottom-right, bottom-left


def side_counts(enc):
    """Mean encoder count of the left pair (TL, BL) and right pair (TR, BR)."""
    return (enc[0] + enc[3]) / 2.0, (enc[1] + enc[2]) / 2.0


def checksum(payload):
    c = 0
    for ch in payload:
        c ^= ord(ch)
    return c


def encode(seq, cmd, with_checksum=True):
    body = "%d:%s" % (seq, cmd)
    if with_checksum:
        body += "*%02X" % checksum(body)
    if len(body) > MAX_LINE:
        raise ValueError("command too long: %r" % body)
    return (body + "\n").encode("ascii")


def parse_line(line):
    """Parse one line from the Arduino into a dict, or None if unrecognised.

    Returns {"kind": "T"|"OK"|"ERR"|"EVT", ...}
    """
    line = line.strip()
    if not line:
        return None
    parts = line.split()
    kind = parts[0]
    try:
        if kind == "T" and len(parts) == 8:
            return {"kind": "T", "uptime_ms": int(parts[1]), "state": parts[2],
                    "armed": parts[3] == "1", "enc": [int(x) for x in parts[4:8]]}
        if kind == "OK" and len(parts) >= 2:
            return {"kind": "OK", "seq": int(parts[1]), "text": " ".join(parts[2:])}
        if kind == "ERR" and len(parts) >= 3:
            return {"kind": "ERR", "seq": int(parts[1]), "code": parts[2]}
        if kind == "EVT" and len(parts) >= 2:
            ev = {"kind": "EVT", "name": parts[1], "text": " ".join(parts[2:])}
            if parts[1] == "DONE" and len(parts) == 8:
                ev.update(seq=int(parts[2]), dir=parts[3], enc=[int(x) for x in parts[4:8]])
            return ev
    except ValueError:
        return None
    return None
