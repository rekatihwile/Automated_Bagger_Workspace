"""
Opens the stereo camera (single USB device, side-by-side frame) and displays
left and right views in separate windows.
Run camera_scan.py first to generate camera_ids.json.
Press Q to quit.
"""

import cv2
from camera import Camera
from config import CAPTURE_WIDTH, CAPTURE_HEIGHT, DISPLAY_SCALE, load_camera_id, load_camera_settings
from camera_tuner import CONTROLS, apply_settings


def main():
    ids = load_camera_id()
    if ids is None:
        print("No camera_ids.json found. Run camera_scan.py first.")
        return

    index = ids["camera"]
    print(f"Opening stereo camera at index {index}")

    with Camera(index, name="Stereo", width=CAPTURE_WIDTH, height=CAPTURE_HEIGHT) as cam:
        cam.set_scale(DISPLAY_SCALE)

        saved = load_camera_settings()
        if saved:
            apply_settings(cam._cap, saved)
            print("Loaded saved camera settings.")

        cv2.namedWindow("Left",  cv2.WINDOW_NORMAL)
        cv2.namedWindow("Right", cv2.WINDOW_NORMAL)

        half_w = int((CAPTURE_WIDTH // 2) * DISPLAY_SCALE)
        half_h = int(CAPTURE_HEIGHT * DISPLAY_SCALE)
        cv2.resizeWindow("Left",  half_w, half_h)
        cv2.resizeWindow("Right", half_w, half_h)

        print("Press Q to quit.")

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
