"""
Global configuration for the stereo camera system.
Edit values here; all other scripts read from this file.
"""

import json
import os

_ROOT = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------
# Scan
# ---------------------------------------------------------------------------
SCAN_START_INDEX = 1       # skip index 0 (built-in webcam)
SCAN_MAX_INDEX   = 10

# ---------------------------------------------------------------------------
# Capture resolution  (HBVCAm-4m2214HD-2 V11 / OV4689 dual sensor)
# Full stitched side-by-side output.  Supported modes:
#   3840x1080 @ 30 fps   (MJPEG/YUY2)
#   2560x720  @ 30 fps
#   1920x540  @ 30 fps
# CAPTURE_DOWNSCALE is applied to NATIVE dims to pick the actual mode.
# ---------------------------------------------------------------------------
NATIVE_WIDTH      = 3840
NATIVE_HEIGHT     = 1080
CAPTURE_DOWNSCALE = 0.5    # 0.5 → 1920x540 (better frame rate over USB 2.0)

# Computed capture resolution (aspect ratio preserved)
CAPTURE_WIDTH  = int(NATIVE_WIDTH  * CAPTURE_DOWNSCALE)
CAPTURE_HEIGHT = int(NATIVE_HEIGHT * CAPTURE_DOWNSCALE)

# ---------------------------------------------------------------------------
# Display — independent of capture; scales the preview window only
# ---------------------------------------------------------------------------
DISPLAY_SCALE = 0.8

# ---------------------------------------------------------------------------
# Calibration board — calib.io plain checkerboard, 10x9 squares, 20mm
# BOARD_DICT = None means plain checkerboard (no ArUco markers needed)
# OpenCV uses inner corners: (COLS-1) x (ROWS-1) = 9x8
# ---------------------------------------------------------------------------
BOARD_DICT        = None    # None = plain checkerboard
BOARD_COLS        = 10      # squares wide
BOARD_ROWS        = 9       # squares tall
BOARD_SQ_MM       = 20.0   # square size in millimetres
BOARD_MARKER_MM   = None    # CharUco only — leave None

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
CAMERA_IDS_PATH      = os.path.join(_ROOT, "camera_ids.json")
CAMERA_SETTINGS_PATH = os.path.join(_ROOT, "camera_settings.json")
CALIB_DATA_DIR       = os.path.join(_ROOT, "calibration", "data")
CALIB_RESULTS_PATH   = os.path.join(_ROOT, "calibration", "calib_results.json")

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def load_camera_id():
    if not os.path.exists(CAMERA_IDS_PATH):
        return None
    with open(CAMERA_IDS_PATH) as f:
        return json.load(f)


def save_camera_id(index):
    with open(CAMERA_IDS_PATH, "w") as f:
        json.dump({"camera": index}, f, indent=2)
    print(f"Camera ID saved to {CAMERA_IDS_PATH}")


def load_camera_settings():
    if not os.path.exists(CAMERA_SETTINGS_PATH):
        return {}
    with open(CAMERA_SETTINGS_PATH) as f:
        return json.load(f)


def save_camera_settings(settings: dict):
    with open(CAMERA_SETTINGS_PATH, "w") as f:
        json.dump(settings, f, indent=2)
    print(f"Camera settings saved to {CAMERA_SETTINGS_PATH}")


def save_calib_results(results: dict):
    with open(CALIB_RESULTS_PATH, "w") as f:
        json.dump(results, f, indent=2)
    print(f"Calibration results saved to {CALIB_RESULTS_PATH}")
