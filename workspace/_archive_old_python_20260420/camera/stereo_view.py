"""
Opens the stereo camera (single USB device, side-by-side frame) and displays
left and right views in separate windows.
Run camera_scan.py first to generate camera_ids.json.
Press Q to quit.
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
from camera import Camera
from config import load_camera_id, load_camera_settings


def main():
    ids = load_camera_id()
    if ids is None:
        print("No camera_ids.json found. Run camera_scan.py first.")
        return

    index = ids["camera"]
    print(f"Opening stereo camera at index {index}")

    with Camera(index, name="Stereo") as cam:
        saved = load_camera_settings()
        if saved:
            cam.apply_settings(saved)
            print("Loaded saved camera settings.")

        half_w = int((cam.capture_width // 2) * cam.display_scale)
        half_h = int(cam.capture_height      * cam.display_scale)

        cv2.namedWindow("Left",  cv2.WINDOW_NORMAL)
        cv2.namedWindow("Right", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Left",  half_w, half_h)
        cv2.resizeWindow("Right", half_w, half_h)

        print(f"Capture: {cam.capture_width}x{cam.capture_height}  "
              f"Display per eye: {half_w}x{half_h}  |  Q to quit")

        while True:
            ret, left, right = cam.read_stereo()
            if ret:
                cv2.imshow("Left",  left)
                cv2.imshow("Right", right)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
