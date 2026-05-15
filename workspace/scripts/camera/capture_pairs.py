"""
Capture stereo calibration pairs from the HBVCAM side-by-side stream.

Controls:
  Enter or Space - save the current stereo pair
  D              - delete the last pair saved in this run
  Q or Esc       - quit
"""

from __future__ import annotations

from pathlib import Path
import sys

import cv2

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import config
from camera import StereoCamera


WINDOW_NAME = "Stereo Pair Capture"


def next_pair_index() -> int:
    indices: list[int] = []
    for path in config.CAPTURE_SESSION_DIR.glob("left_*.png"):
        try:
            indices.append(int(path.stem.split("_", 1)[1]))
        except ValueError:
            pass
    return max(indices) + 1 if indices else 0


def delete_pair(pair_index: int) -> None:
    for side in ("left", "right"):
        f = config.CAPTURE_SESSION_DIR / f"{side}_{pair_index:05d}.png"
        if f.exists():
            f.unlink()


def detect_checkerboard(frame) -> bool:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    ok, _ = cv2.findChessboardCorners(gray, config.BOARD_INNER_CORNERS, flags)
    return bool(ok)


def save_pair(pair_index: int, left, right, stereo: StereoCamera) -> tuple[Path, Path]:
    left_path = config.CAPTURE_SESSION_DIR / f"left_{pair_index:05d}.png"
    right_path = config.CAPTURE_SESSION_DIR / f"right_{pair_index:05d}.png"
    cv2.imwrite(str(left_path), left)
    cv2.imwrite(str(right_path), right)
    return left_path, right_path


def main() -> None:
    config.ensure_directories()
    pair_index = next_pair_index()
    last_saved_index: int | None = None

    with StereoCamera() as stereo:
        stereo.warmup()
        stereo.create_preview_window(WINDOW_NAME)

        actual = stereo.actual_size
        eye = stereo.eye_size
        print(f"Camera index: {config.load_camera_index()}")
        print(f"Requested stream: {config.STREAM_WIDTH}x{config.STREAM_HEIGHT} {config.CAMERA_FOURCC}")
        print(f"Actual stream:    {actual.width}x{actual.height}")

        try:
            stereo.require_stereo_shape()
        except RuntimeError as exc:
            print()
            print(exc)
            return

        print(f"Saved eye size:   {eye.width}x{eye.height}")
        print(f"Saving pairs to:  {config.CAPTURE_SESSION_DIR}")
        print("Enter/Space=capture  D=delete last  Q/Esc=quit")

        while True:
            ok, left, right = stereo.read_pair()
            if not ok or left is None or right is None:
                continue

            left_ok = right_ok = None
            board_line = "Board check disabled"
            if config.LIVE_BOARD_DETECTION:
                left_ok = detect_checkerboard(left)
                right_ok = detect_checkerboard(right)
                board_line = f"Board: L={'ok' if left_ok else 'no'} R={'ok' if right_ok else 'no'}"

            lines = [
                f"Next pair: pair_{pair_index:04d}",
                "Enter/Space=capture  D=delete last  Q/Esc=quit",
                board_line,
            ]
            cv2.imshow(WINDOW_NAME, stereo.render_preview(left, right, lines, left_ok, right_ok))
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break
            if key in (13, 10, ord(" ")):
                left_path, right_path = save_pair(pair_index, left, right, stereo)
                print(f"Saved {left_path.relative_to(config.PROJECT_ROOT)}, {right_path.name}")
                last_saved_index = pair_index
                pair_index += 1
            elif key == ord("d") and last_saved_index is not None:
                delete_pair(last_saved_index)
                print(f"Deleted pair {last_saved_index:05d}")
                pair_index = last_saved_index
                last_saved_index = None

    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()
