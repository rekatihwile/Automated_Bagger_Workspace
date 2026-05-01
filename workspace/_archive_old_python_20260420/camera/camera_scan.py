"""
Scans USB video devices (skipping the built-in webcam at index 0) and saves
the stereo camera index to camera_ids.json via config.
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
from config import SCAN_START_INDEX, SCAN_MAX_INDEX, save_camera_id


def scan_cameras():
    found = []
    print(f"Scanning indices {SCAN_START_INDEX} to {SCAN_MAX_INDEX - 1} (skipping webcam at 0)...\n")

    for i in range(SCAN_START_INDEX, SCAN_MAX_INDEX):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap.isOpened():
            ret, _ = cap.read()
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"  Index {i}: OK  —  {w}x{h}  (frame read: {ret})")
            found.append(i)
            cap.release()
        else:
            print(f"  Index {i}: not available")

    print()
    if found:
        index = found[0]
        print(f"Stereo camera found at index {index}.")
        print("The device outputs a single side-by-side frame; stereo_view.py will split it.")
        save_camera_id(index)
    else:
        print("No cameras found. Check USB connection.")

    return found


if __name__ == "__main__":
    scan_cameras()
