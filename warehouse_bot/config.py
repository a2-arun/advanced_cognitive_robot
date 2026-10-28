"""Central configuration. Python 3.6 compatible (no dataclasses).

Every tunable number lives here so it can be changed without editing code:
    python -m warehouse_bot.dashboard --config my_config.json
where my_config.json only needs the keys you want to override, e.g.
    {"obstacle": {"stop_distance_m": 0.6}}
"""
import copy
import json

DEFAULTS = {
    "serial": {
        "port": "auto",               # "auto", "/dev/ttyACM0", "COM5", or "mock" (software-only test firmware)
        "baud": 115200,
        "connect_wait_s": 2.2,        # Uno resets when the port opens; wait for the bootloader
        "heartbeat_s": 0.2,           # host -> Arduino watchdog refresh
        "telemetry_timeout_s": 1.0,   # no telemetry for this long => link considered DOWN
        "ack_timeout_s": 0.6,
    },
    "motion": {
        # Motors have NO PWM: every movement is a full-speed pulse. Distances are NOT known until calibrated.
        "default_pulse_ms": 250,
        "min_pulse_ms": 50,
        "max_pulse_ms": 800,          # host-side clamp (firmware hard cap is 1200)
        "settle_s": 0.6,              # pause after each pulse before the camera is trusted again
    },
    "camera": {
        "source": "auto",             # auto | realsense | webcam | video:<path> | session:<dir> | none
        "width": 640,
        "height": 480,
        "fps": 15,                    # Jetson Nano USB3 + depth: 15 fps is plenty for stop-and-go motion
        "webcam_index": 0,
    },
    "obstacle": {
        # Regions of interest as fractions of the image (x0, x1, y0, y1).
        "roi_front": [0.35, 0.65, 0.30, 0.65],
        "roi_left":  [0.02, 0.17, 0.30, 0.65],
        "roi_right": [0.83, 0.98, 0.30, 0.65],
        "stop_distance_m": 0.80,      # obstacle if the front ROI is closer than this
        "clear_hysteresis_m": 0.15,   # must be this much farther to return to CLEAR
        "side_clear_m": 1.00,         # a side must be at least this open to be chosen
        "min_range_m": 0.30,          # ignore depth below this (D455 cannot see closer than ~0.4 m)
        "max_range_m": 4.00,          # ignore depth beyond this
        "min_valid_fraction": 0.25,   # fraction of ROI pixels with valid depth, else UNRELIABLE
        "percentile": 10,             # near-distance estimate = this percentile of valid ROI depths
        "smooth_frames": 5,           # median over this many frames
        "confirm_frames": 3,          # consecutive frames needed to change state
        "turn_pulse_ms": 300,
        "recheck_frames": 4,          # fresh frames required after a turn before re-deciding
        "max_consecutive_turns": 4,   # then give up and STOP
        "advance_when_clear": False,  # if True (and acting), pulse forward when the path is clear
        "advance_pulse_ms": 250,
    },
    "docking": {
        "aruco_dict": "DICT_4X4_50",
        "marker_id": 0,
        "marker_size_m": 0.10,        # PRINTED black-square edge length. MEASURE YOUR PRINT and edit this.
        "deadband": 0.10,             # |offset| below this => CENTERED (offset = (cx - W/2) / (W/2))
        "enter_align": 0.16,          # once centered, only re-align if |offset| exceeds this (hysteresis)
        "dock_distance_m": 0.40,      # stop when camera-to-marker distance <= this
        "confirm_frames": 3,
        "lost_frames": 8,
        "align_pulse_ms": 150,
        "approach_pulse_ms": 250,
        "near_margin_m": 0.30,        # within this of the dock distance, use the shorter pulse
        "near_pulse_ms": 100,
        "search_pulse_ms": 0,         # >0 enables rotate-to-search pulses (default: no blind motion)
        "settle_s": 0.6,
        "max_actions": 40,            # safety cap on pulses per docking attempt
    },
    "odometry": {
        "counts_per_m": None,         # MUST be calibrated: tools/calibrate_odometry.py
        "effective_track_m": None,    # MUST be calibrated for turning
    },
    "sim": {
        "world_w": 8.0, "world_h": 6.0,
        "speed_mps": 0.30,            # NOT calibrated to the real robot
        "turn_dps": 100.0,            # NOT calibrated to the real robot
        "robot_radius_m": 0.17,
        "depth_noise_m": 0.01,
        "seed": 7,
    },
    "server": {
        "host": "127.0.0.1",          # localhost only by default; use --host 0.0.0.0 for LAN (token required)
        "port": 8080,
        "results_dir": "results",
    },
}


def _merge(base, over):
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _merge(base[k], v)
        else:
            base[k] = v
    return base


def load_config(path=None):
    cfg = copy.deepcopy(DEFAULTS)
    if path:
        with open(path, "r") as f:
            _merge(cfg, json.load(f))
    return cfg
