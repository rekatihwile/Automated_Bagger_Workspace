"""
Live camera tuner for HBVCAm-4m2214HD-2 V11 (OV4689 dual sensor).
Trackbars adjust UVC properties in real time. Press S to save, Q to quit.

Supported resolutions (stitched side-by-side):
  3840x1080 @ 30fps  (max, slow over USB 2.0)
  1920x540  @ 30fps  (CAPTURE_DOWNSCALE = 0.5, default)
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np
from config import (
    CAPTURE_WIDTH, CAPTURE_HEIGHT, DISPLAY_SCALE,
    load_camera_id, load_camera_settings, save_camera_settings,
)

# (label, cv2_prop, min, max, default)
# Trackbar values are always non-negative integers.
# Exposure is stored as an offset: trackbar_val - 13 = actual UVC value (-13..-1)
CONTROLS = [
    ("Brightness",    cv2.CAP_PROP_BRIGHTNESS,   0,   255, 128),
    ("Contrast",      cv2.CAP_PROP_CONTRAST,      0,   255, 128),
    ("Saturation",    cv2.CAP_PROP_SATURATION,    0,   255, 128),
    ("Hue",           cv2.CAP_PROP_HUE,           0,   180,  90),
    ("Sharpness",     cv2.CAP_PROP_SHARPNESS,     0,   255,   0),
    ("Gamma",         cv2.CAP_PROP_GAMMA,        100,  500, 220),
    ("Gain",          cv2.CAP_PROP_GAIN,           0,   255,   0),
    ("Exposure",      cv2.CAP_PROP_EXPOSURE,       0,    12,   6),
    ("Auto Exposure", cv2.CAP_PROP_AUTO_EXPOSURE,  0,     3,   3),
]

CTRL_WINDOW  = "Camera Tuner Controls"
LEFT_WINDOW  = "Left"
RIGHT_WINDOW = "Right"


def apply_settings(cap, settings):
    for label, prop, lo, hi, _ in CONTROLS:
        if label in settings:
            val = settings[label]
            cap.set(prop, val - 13 if label == "Exposure" else val)


def _read_current(cap):
    current = {}
    for label, prop, lo, hi, default in CONTROLS:
        raw = cap.get(prop)
        if label == "Exposure":
            current[label] = int(np.clip(raw + 13, lo, hi))
        else:
            current[label] = int(np.clip(raw if raw >= 0 else default, lo, hi))
    return current


def _make_cb(cap, label, prop):
    def cb(val):
        cap.set(prop, val - 13 if label == "Exposure" else val)
    return cb


def main():
    ids = load_camera_id()
    if ids is None:
        print("No camera_ids.json found. Run camera_scan.py first.")
        return

    index = ids["camera"]
    print(f"Opening stereo camera at index {index}  ({CAPTURE_WIDTH}x{CAPTURE_HEIGHT})")

    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"Failed to open camera at index {index}.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Actual capture resolution: {actual_w}x{actual_h}")

    current = _read_current(cap)
    saved   = load_camera_settings()
    current.update({k: v for k, v in saved.items() if k in current})
    apply_settings(cap, current)

    cv2.namedWindow(CTRL_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CTRL_WINDOW, 500, 40 + len(CONTROLS) * 45)
    for label, prop, lo, hi, _ in CONTROLS:
        cv2.createTrackbar(label, CTRL_WINDOW, current[label], hi - lo,
                           _make_cb(cap, label, prop))

    half_w = int((actual_w // 2) * DISPLAY_SCALE)
    half_h = int(actual_h * DISPLAY_SCALE)
    for win in (LEFT_WINDOW, RIGHT_WINDOW):
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, half_w, half_h)

    print("Trackbars: adjust live  |  S = save settings  |  Q = quit")

    while True:
        ret, frame = cap.read()
        if ret:
            mid   = actual_w // 2
            left  = cv2.resize(frame[:, :mid],  (half_w, half_h))
            right = cv2.resize(frame[:, mid:],  (half_w, half_h))
            cv2.imshow(LEFT_WINDOW,  left)
            cv2.imshow(RIGHT_WINDOW, right)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            snapshot = {label: cv2.getTrackbarPos(label, CTRL_WINDOW) + lo
                        for label, _, lo, _, _ in CONTROLS}
            save_camera_settings(snapshot)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
