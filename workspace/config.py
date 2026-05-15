"""
Project-wide settings for the HBVCAM stereo triangulation workflow.

The HBVCAM-4M2214HD-2 V11 uses one USB connection. In OpenCV that normally
appears as one camera index that outputs one wide side-by-side stereo frame.
The left and right lens images are the left and right halves of that frame.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

# Root of the full project (data + hardware + code workspace).
# This file lives in <project_root>/workspace/config.py.
PROJECT_ROOT = Path(__file__).resolve().parent.parent

# Run camera_scan.py if this index is wrong. camera_ids.json overrides this.
CAMERA_INDEX = 1

# Windows usually behaves best with DirectShow for UVC camera controls.
CAMERA_BACKEND = "DSHOW"
CAMERA_FOURCC = "MJPG"
CAMERA_FPS = 10

0

# Modes worth probing with camera_scan.py. Keep these explicit because some
# drivers do not honor scaled requests and silently fall back to mono modes.
STEREO_STREAM_MODES = (
    
  (3840,1080),  # 2x 1920x1080 (16:9) - ratio 3.56
  (2688,1520),  # 2x 1344x760 (4:3) - ratio 2.67
  (2560,720),   # 2x 1280x720 (16:9) - ratio 3.56
  
)
MODE = 1  # index into the above tuple for the requested stereo stream mode. If the driver does not support it, capture_pairs.py will tell you what it returned.


MODE_INDEX = STEREO_STREAM_MODES[MODE-1]


# Requested stitched-stereo stream size. If the driver falls back to a mono mode,
# `capture_pairs.py` will refuse to split and will tell you what it returned.
STREAM_WIDTH, STREAM_HEIGHT =  MODE_INDEX[0], MODE_INDEX[1]


# Preview-only scale for the display window. Saved images are not resized.
DISPLAY_SCALE = 1

WARMUP_FRAMES = 20
# Side-by-side stereo aspect ratio is roughly 2.67 for two 4:3 VGA views
# and roughly 3.56 for two 16:9 views. Mono camera modes are usually ~1.78.
STEREO_MIN_ASPECT_RATIO = 2.4

# Camera settings captured from:
# camera_test_properties_20260423_121205_4961.json
# timestamp: 2026-04-23T12:12:05.566253
# Set values to None to leave the camera default unchanged.
CAMERA_SETTINGS: dict[str, float | int | None] = {
    "Brightness": 0,
    "Contrast": 0,
    "Saturation": 64,
    "Hue": 0,
    "Sharpness": 2,
    "Gamma": 100,
    "Gain": -1,
    "Exposure": -5,
    "Auto Exposure": -1,
}

# Convenience exposure controls (optional).
#
# Notes (Windows + DirectShow quirks):
# - Many UVC cameras use negative exposure values (often -13 .. -1).
# - For some drivers, CAP_PROP_AUTO_EXPOSURE uses 3=auto, 1=manual.
# - If you see no effect, the driver may ignore these properties at this mode.
AUTO_EXPOSURE_MODE: str | None = None  # None, "auto", or "manual"
EXPOSURE_VALUE: int | float | None = None  # e.g. -6 for manual exposure
AUTO_EXPOSURE_VALUE_AUTO: int | float = 3
AUTO_EXPOSURE_VALUE_MANUAL: int | float = 1

# Image preprocessing (optional). This runs in software and can be applied to:
# - "preview": only affects the displayed windows
# - "saved": affects the images written to disk
# - "both": preview and saved
# - "none": disabled
PREPROCESS: dict[str, Any] = {
    "apply_to": "preview",  # "none" | "preview" | "saved" | "both"
    "gamma": 1.0,           # >1 darkens, <1 brightens
    "clahe": False,         # contrast normalization on luminance channel
    "clahe_clip": 2.0,
    "clahe_grid": (8, 8),
    "blur_ksize": 0,        # 0 disables; otherwise 3,5,7...
    "sharpen": 0.0,         # 0 disables; typical 0.2..1.0
}


# ---------------------------------------------------------------------------
# Live AI demo
# ---------------------------------------------------------------------------

AI_TOPFACE_RECORD_VIDEO = True
AI_TOPFACE_RECORD_FPS = 30.0
AI_TOPFACE_RECORD_SCALE = 1.0
AI_TOPFACE_PLOT_FRAME = "box"  # "box" or "camera"
# ---------------------------------------------------------------------------
# AI top-face segmentation / test recording
# ---------------------------------------------------------------------------

AI_INPUT_SIZE = 640
AI_BOUNDARY_SAMPLE_COUNT = 32
CALIBRATION_ROOT = PROJECT_ROOT / "calibration"

RECORD_TEST_VIDEO = False
TEST_RECORDINGS_DIR = CALIBRATION_ROOT / "test_recordings"
TEST_RECORD_VIDEO_FPS = 15.0
TEST_RECORD_VIDEO_SCALE = 1.0

# ---------------------------------------------------------------------------
# Calib.io checkerboard
# ---------------------------------------------------------------------------

# Calib.io checkerboard square counts, not inner-corner counts.
BOARD_SQUARES_X = 10
BOARD_SQUARES_Y = 9
BOARD_SQUARE_SIZE_MM = 20.0

BOARD_INNER_CORNERS = (BOARD_SQUARES_X - 1, BOARD_SQUARES_Y - 1)
MIN_CALIBRATION_PAIRS = 8
LIVE_BOARD_DETECTION = False


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

CAMERA_IDS_PATH = PROJECT_ROOT / "camera_ids.json"
CAMERA_SETTINGS_PATH = PROJECT_ROOT / "camera_settings.json"

# Root directory that holds all stereo pair session folders (e.g. pair_0001, pair_0002 …).
# calibrate.py scans every pair_* subfolder here automatically.
STEREO_PAIR_DIR = CALIBRATION_ROOT / "pairs"
MATRIX_ROOT = CALIBRATION_ROOT / "matrices"

# Name of the session subfolder inside STEREO_PAIR_DIR where capture_pairs.py saves images.
# Change this to start a fresh capture session without overwriting old data.
# calibrate.py will discover the new folder automatically on next run.
CAPTURE_SESSION_NAME = "pair_0017"
CAPTURE_SESSION_DIR = STEREO_PAIR_DIR / CAPTURE_SESSION_NAME

# Calibration matrix output — downstream scripts (rectify, disparity, etc.) read from here.
ACTIVE_MATRIX_DIR = MATRIX_ROOT / "latest"
ACTIVE_CALIBRATION_NPZ = ACTIVE_MATRIX_DIR / "stereo_calibration.npz"
ACTIVE_CALIBRATION_JSON = ACTIVE_MATRIX_DIR / "calibration_summary.json"


def ensure_directories() -> None:
    """Create project output directories used by the capture/calibration scripts."""
    STEREO_PAIR_DIR.mkdir(parents=True, exist_ok=True)
    ACTIVE_MATRIX_DIR.mkdir(parents=True, exist_ok=True)
    CAPTURE_SESSION_DIR.mkdir(parents=True, exist_ok=True)


def _load_json(path: Path, default: Any) -> Any:
    if not path.exists():
        return default
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _save_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def load_camera_index() -> int:
    """Return the configured OpenCV camera index for the single USB stereo stream."""
    data = _load_json(CAMERA_IDS_PATH, {})
    return int(data.get("camera", CAMERA_INDEX))


def save_camera_index(index: int) -> None:
    _save_json(CAMERA_IDS_PATH, {"camera": int(index)})

STREAM_EYE_WIDTH = STREAM_WIDTH // 2
STREAM_EYE_HEIGHT = STREAM_HEIGHT
EYE_WIDTH = STREAM_EYE_WIDTH
EYE_HEIGHT = STREAM_EYE_HEIGHT


def load_camera_settings() -> dict[str, float | int]:
    """Merge config defaults with optional camera_settings.json overrides."""
    settings = {k: v for k, v in CAMERA_SETTINGS.items() if v is not None}
    settings.update(_load_json(CAMERA_SETTINGS_PATH, {}))

    # Convenience exposure controls override the above if set.
    if AUTO_EXPOSURE_MODE is not None:
        mode = AUTO_EXPOSURE_MODE.strip().lower()
        if mode == "auto":
            settings["Auto Exposure"] = AUTO_EXPOSURE_VALUE_AUTO
        elif mode == "manual":
            settings["Auto Exposure"] = AUTO_EXPOSURE_VALUE_MANUAL
            if EXPOSURE_VALUE is not None:
                settings["Exposure"] = EXPOSURE_VALUE

    return settings


def save_camera_settings(settings: dict[str, float | int]) -> None:
    _save_json(CAMERA_SETTINGS_PATH, settings)


def load_stereo_calibration(path: Path = ACTIVE_CALIBRATION_NPZ) -> dict[str, np.ndarray] | None:
    """
    Load the latest saved stereo calibration matrices.

    Import config.STEREO_CALIBRATION in downstream triangulation code when you
    want a ready-to-use dictionary of matrices, or call this function directly
    if you need to load a specific calibration file.
    """
    if not path.exists():
        return None
    with np.load(path, allow_pickle=False) as data:
        return {name: data[name] for name in data.files}


STEREO_CALIBRATION = load_stereo_calibration()
