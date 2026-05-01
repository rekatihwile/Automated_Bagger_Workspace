"""
Script 1 — Stereo pair capture.
Resolution is controlled by CAPTURE_DOWNSCALE in config.py.
Lower it (e.g. 0.33) for better frame rate; raise it for more detail.
Calibration data will be valid for whichever resolution you capture at.

Controls:
  SPACE  — capture and save left_NNNN.png / right_NNNN.png
  D      — delete the last saved pair
  Q      — quit

Green border = board detected in both eyes. Red = not detected.
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "camera"))

import cv2
import numpy as np
from pathlib import Path
from config import (
    DISPLAY_SCALE, BOARD_COLS, BOARD_ROWS,
    load_camera_id, load_camera_settings, CALIB_DATA_DIR,
)
from camera import Camera

INNER         = (BOARD_COLS - 1, BOARD_ROWS - 1)
SAVE_DIR      = Path(CALIB_DATA_DIR)
WARMUP_FRAMES = 20
SUBPIX        = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def overlay_text(img, lines, origin=(20, 40), line_height=32, color=(0, 255, 0)):
    x, y = origin
    for i, line in enumerate(lines):
        pos = (x, y + i * line_height)
        cv2.putText(img, line, (pos[0]+2, pos[1]+2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, line, pos,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)


def detect_fast(gray):
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
             | cv2.CALIB_CB_NORMALIZE_IMAGE
             | cv2.CALIB_CB_FAST_CHECK)
    ok, _ = cv2.findChessboardCorners(gray, INNER, flags)
    return ok


def get_start_index(save_dir: Path) -> int:
    existing = list(save_dir.glob("left_*.png"))
    if not existing:
        return 0
    indices = []
    for f in existing:
        try:
            indices.append(int(f.stem.split("_")[1]))
        except Exception:
            pass
    return max(indices) + 1 if indices else 0


def main():
    SAVE_DIR.mkdir(parents=True, exist_ok=True)

    ids = load_camera_id()
    if ids is None:
        print("No camera_ids.json found. Run camera/camera_scan.py first.")
        return

    with Camera(ids["camera"], name="Stereo") as cam:
        saved_settings = load_camera_settings()
        if saved_settings:
            cam.apply_settings(saved_settings)

        print(f"Warming up camera...")
        for _ in range(WARMUP_FRAMES):
            cam.read()

        eye_w = cam.capture_width // 2
        eye_h = cam.capture_height
        print(f"Capture: {cam.capture_width}x{cam.capture_height}  |  Each eye: {eye_w}x{eye_h}")
        print(f"Saving to: {SAVE_DIR}")
        print(f"Board inner corners: {INNER}")
        print("SPACE=capture  D=delete last  Q=quit\n")

        idx       = get_start_index(SAVE_DIR)
        last_pair = None
        status    = [f"Pairs saved: {idx}", "SPACE=capture  D=delete last  Q=quit"]
        color     = (200, 200, 200)

        while True:
            ret, full = cam.read()
            if not ret:
                continue

            left_raw  = full[:, :eye_w]
            right_raw = full[:, eye_w:]

            # Live detection for border color
            gl    = cv2.cvtColor(left_raw,  cv2.COLOR_BGR2GRAY)
            gr    = cv2.cvtColor(right_raw, cv2.COLOR_BGR2GRAY)
            ok_l  = detect_fast(gl)
            ok_r  = detect_fast(gr)
            both  = ok_l and ok_r

            # Scale for display
            dl = cv2.resize(left_raw,  None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE,
                            interpolation=cv2.INTER_AREA)
            dr = cv2.resize(right_raw, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE,
                            interpolation=cv2.INTER_AREA)

            cv2.rectangle(dl, (0, 0), (dl.shape[1]-1, dl.shape[0]-1),
                          (0, 200, 0) if ok_l else (0, 0, 200), 4)
            cv2.rectangle(dr, (0, 0), (dr.shape[1]-1, dr.shape[0]-1),
                          (0, 200, 0) if ok_r else (0, 0, 200), 4)
            cv2.putText(dl, "LEFT",  (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)
            cv2.putText(dr, "RIGHT", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)

            divider = np.full((dl.shape[0], 3, 3), 200, dtype=np.uint8)
            display = cv2.hconcat([dl, divider, dr])

            board_label = "BOARD OK" if both else f"NO BOARD  L={'ok' if ok_l else 'X'} R={'ok' if ok_r else 'X'}"
            overlay_text(display, [board_label] + status,
                         color=(0, 255, 0) if both else (0, 80, 255))

            cv2.imshow("Stereo Capture", display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break

            elif key == ord(' '):
                lp = SAVE_DIR / f"left_{idx:04d}.png"
                rp = SAVE_DIR / f"right_{idx:04d}.png"
                cv2.imwrite(str(lp), left_raw)
                cv2.imwrite(str(rp), right_raw)
                last_pair = (lp, rp)
                print(f"  Saved pair {idx:04d}  {'(board ok)' if both else '(WARNING: no board detected)'}")
                idx += 1
                status = [f"Pairs saved: {idx}", "SPACE=capture  D=delete last  Q=quit"]
                color  = (200, 200, 200)

            elif key == ord('d') and last_pair:
                for p in last_pair:
                    if p.exists():
                        p.unlink()
                        print(f"  Deleted {p.name}")
                idx -= 1
                last_pair = None
                status = [f"Pairs saved: {idx}  (last deleted)", "SPACE=capture  Q=quit"]
                color  = (0, 165, 255)

    cv2.destroyAllWindows()
    print(f"\nDone. {idx} pairs in {SAVE_DIR}")


if __name__ == "__main__":
    main()
