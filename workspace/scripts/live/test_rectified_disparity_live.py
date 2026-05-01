"""
Live rectification + disparity debug viewer for the HBVCam stereo camera.

What this does:
1. Opens the side-by-side HBVCam stereo stream.
2. Splits the stitched frame into left/right images.
3. Loads saved stereo calibration from config.ACTIVE_CALIBRATION_NPZ.
4. Rectifies both images.
5. Draws horizontal guide lines so you can visually check rectification.
6. Computes a basic StereoSGBM disparity map.
7. Shows:
   - raw stereo pair
   - rectified stereo pair
   - disparity map

Controls:
  q / Esc : quit
  s       : save current raw/rectified/disparity images
  [ / ]   : decrease/increase numDisparities
  - / =   : decrease/increase blockSize
"""

from __future__ import annotations

import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


# -------------------------------------------------------------------------
# Import your workspace modules
# -------------------------------------------------------------------------

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import config
from camera import StereoCamera


WINDOW_RAW = "Raw left/right"
WINDOW_RECTIFIED = "Rectified left/right"
WINDOW_DISPARITY = "StereoSGBM disparity"


# -------------------------------------------------------------------------
# Helpers
# -------------------------------------------------------------------------

def require_calibration() -> dict[str, np.ndarray]:
    calib = config.load_stereo_calibration()

    if calib is None:
        raise FileNotFoundError(
            f"\nNo calibration file found at:\n"
            f"  {config.ACTIVE_CALIBRATION_NPZ}\n\n"
            f"Run your calibration script first:\n"
            f"  python workspace/scripts/camera/calibrate.py\n"
        )

    required = [
        "image_size",
        "left_camera_matrix",
        "left_distortion_coefficients",
        "right_camera_matrix",
        "right_distortion_coefficients",
        "rectification_left",
        "rectification_right",
        "projection_left_rectified",
        "projection_right_rectified",
        "disparity_to_depth_Q",
    ]

    missing = [k for k in required if k not in calib]
    if missing:
        raise KeyError(
            "Calibration file exists, but is missing these keys:\n"
            + "\n".join(f"  - {k}" for k in missing)
            + "\n\nYour calibration .npz may have different variable names."
        )

    return calib


def make_rectification_maps(
    calib: dict[str, np.ndarray],
    live_size: tuple[int, int],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Create remap matrices for left and right rectification.

    live_size is (width, height) for each eye.
    """
    w, h = live_size

    k_l = calib["left_camera_matrix"]
    d_l = calib["left_distortion_coefficients"]
    k_r = calib["right_camera_matrix"]
    d_r = calib["right_distortion_coefficients"]

    r_l = calib["rectification_left"]
    r_r = calib["rectification_right"]
    p_l = calib["projection_left_rectified"]
    p_r = calib["projection_right_rectified"]

    map_lx, map_ly = cv2.initUndistortRectifyMap(
        k_l, d_l, r_l, p_l, (w, h), cv2.CV_32FC1
    )

    map_rx, map_ry = cv2.initUndistortRectifyMap(
        k_r, d_r, r_r, p_r, (w, h), cv2.CV_32FC1
    )

    return map_lx, map_ly, map_rx, map_ry


def draw_horizontal_guides(
    img: np.ndarray,
    spacing: int = 40,
    color: tuple[int, int, int] = (0, 255, 255),
) -> np.ndarray:
    out = img.copy()
    h, w = out.shape[:2]

    for y in range(spacing, h, spacing):
        cv2.line(out, (0, y), (w - 1, y), color, 1, cv2.LINE_AA)

    return out


def side_by_side(left: np.ndarray, right: np.ndarray, label: str = "") -> np.ndarray:
    left_view = left.copy()
    right_view = right.copy()

    cv2.putText(
        left_view,
        "LEFT",
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    cv2.putText(
        right_view,
        "RIGHT",
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    divider = np.full((left_view.shape[0], 8, 3), 30, dtype=np.uint8)
    out = cv2.hconcat([left_view, divider, right_view])

    if label:
        cv2.putText(
            out,
            label,
            (15, out.shape[0] - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    return out


def make_sgbm(num_disparities: int, block_size: int) -> cv2.StereoSGBM:
    """
    Create a StereoSGBM matcher.

    num_disparities must be divisible by 16.
    block_size must be odd and usually between 3 and 11.
    """
    num_disparities = max(16, int(round(num_disparities / 16)) * 16)
    block_size = int(block_size)
    if block_size % 2 == 0:
        block_size += 1
    block_size = max(3, block_size)

    return cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disparities,
        blockSize=block_size,
        P1=8 * 3 * block_size**2,
        P2=32 * 3 * block_size**2,
        disp12MaxDiff=1,
        uniquenessRatio=8,
        speckleWindowSize=80,
        speckleRange=2,
        preFilterCap=31,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )


def compute_disparity_display(
    matcher: cv2.StereoSGBM,
    left_rect: np.ndarray,
    right_rect: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Returns:
      disparity_float : raw disparity in pixels
      disparity_color : colored visualization
    """
    gray_l = cv2.cvtColor(left_rect, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right_rect, cv2.COLOR_BGR2GRAY)

    disp_raw = matcher.compute(gray_l, gray_r)

    # OpenCV stores SGBM disparity scaled by 16.
    disp = disp_raw.astype(np.float32) / 16.0

    valid = disp > 0

    if np.any(valid):
        lo = np.percentile(disp[valid], 2)
        hi = np.percentile(disp[valid], 98)
        disp_norm = np.clip((disp - lo) / max(hi - lo, 1e-6), 0, 1)
    else:
        disp_norm = np.zeros_like(disp, dtype=np.float32)

    disp_u8 = (disp_norm * 255).astype(np.uint8)
    disp_color = cv2.applyColorMap(disp_u8, cv2.COLORMAP_TURBO)

    # Invalid disparity = black
    disp_color[~valid] = (0, 0, 0)

    return disp, disp_color


def save_debug_images(
    out_dir: Path,
    raw_left: np.ndarray,
    raw_right: np.ndarray,
    rect_left: np.ndarray,
    rect_right: np.ndarray,
    disp_color: np.ndarray,
    disp_float: np.ndarray,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    cv2.imwrite(str(out_dir / f"{stamp}_raw_left.png"), raw_left)
    cv2.imwrite(str(out_dir / f"{stamp}_raw_right.png"), raw_right)
    cv2.imwrite(str(out_dir / f"{stamp}_rect_left.png"), rect_left)
    cv2.imwrite(str(out_dir / f"{stamp}_rect_right.png"), rect_right)
    cv2.imwrite(str(out_dir / f"{stamp}_disparity_color.png"), disp_color)

    np.save(out_dir / f"{stamp}_disparity_float.npy", disp_float)

    print(f"[SAVE] Saved debug images to: {out_dir}")


# -------------------------------------------------------------------------
# Main
# -------------------------------------------------------------------------

def main() -> None:
    print("[INFO] Loading stereo calibration...")
    calib = require_calibration()

    print(f"[INFO] Calibration file: {config.ACTIVE_CALIBRATION_NPZ}")
    print(f"[INFO] Calibration keys: {sorted(calib.keys())}")

    num_disparities = 96
    block_size = 5
    matcher = make_sgbm(num_disparities, block_size)

    save_dir = config.CALIBRATION_ROOT / "rectified_disparity_debug"

    with StereoCamera() as cam:
        print("[INFO] Opening HBVCam stereo stream...")
        cam.require_stereo_shape()
        cam.warmup()

        eye = cam.eye_size
        live_size = (eye.width, eye.height)

        print(f"[INFO] Full stream size: {cam.actual_size.width} x {cam.actual_size.height}")
        print(f"[INFO] Eye size:         {eye.width} x {eye.height}")

        calib_size = tuple(int(v) for v in np.asarray(calib["image_size"]).reshape(-1))
        print(f"[INFO] Calibration eye size: {calib_size}")

        if calib_size != live_size:
            print(
                "\n[WARNING] Your live eye size does not match the calibration image_size.\n"
                f"          Live eye size:        {live_size}\n"
                f"          Calibration eye size: {calib_size}\n"
                "          Rectification may be wrong unless calibration was done at this same size.\n"
            )

        print("[INFO] Building rectification maps...")
        map_lx, map_ly, map_rx, map_ry = make_rectification_maps(calib, live_size)

        cv2.namedWindow(WINDOW_RAW, cv2.WINDOW_NORMAL)
        cv2.namedWindow(WINDOW_RECTIFIED, cv2.WINDOW_NORMAL)
        cv2.namedWindow(WINDOW_DISPARITY, cv2.WINDOW_NORMAL)

        print("\n[READY]")
        print("  q / Esc : quit")
        print("  s       : save current debug images")
        print("  [ / ]   : decrease/increase numDisparities")
        print("  - / =   : decrease/increase blockSize")
        print()

        last_raw_left = None
        last_raw_right = None
        last_rect_left = None
        last_rect_right = None
        last_disp = None
        last_disp_color = None

        while True:
            ok, left, right = cam.read_pair()
            if not ok or left is None or right is None:
                print("[WARN] Failed to read stereo pair.")
                continue

            # Rectify / undistort
            left_rect = cv2.remap(left, map_lx, map_ly, cv2.INTER_LINEAR)
            right_rect = cv2.remap(right, map_rx, map_ry, cv2.INTER_LINEAR)

            # Disparity
            disp, disp_color = compute_disparity_display(matcher, left_rect, right_rect)

            # Guide lines
            left_raw_guided = draw_horizontal_guides(left)
            right_raw_guided = draw_horizontal_guides(right)
            left_rect_guided = draw_horizontal_guides(left_rect)
            right_rect_guided = draw_horizontal_guides(right_rect)

            raw_view = side_by_side(
                left_raw_guided,
                right_raw_guided,
                label="RAW stereo pair. Lines probably will NOT align perfectly.",
            )

            rect_view = side_by_side(
                left_rect_guided,
                right_rect_guided,
                label="RECTIFIED stereo pair. Matching features should be on same horizontal lines.",
            )

            cv2.putText(
                disp_color,
                f"SGBM: numDisp={num_disparities}, blockSize={block_size}",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow(WINDOW_RAW, raw_view)
            cv2.imshow(WINDOW_RECTIFIED, rect_view)
            cv2.imshow(WINDOW_DISPARITY, disp_color)

            last_raw_left = left
            last_raw_right = right
            last_rect_left = left_rect
            last_rect_right = right_rect
            last_disp = disp
            last_disp_color = disp_color

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            elif key == ord("s"):
                save_debug_images(
                    save_dir,
                    last_raw_left,
                    last_raw_right,
                    last_rect_left,
                    last_rect_right,
                    last_disp_color,
                    last_disp,
                )

            elif key == ord("["):
                num_disparities = max(16, num_disparities - 16)
                matcher = make_sgbm(num_disparities, block_size)
                print(f"[PARAM] numDisparities = {num_disparities}")

            elif key == ord("]"):
                num_disparities = min(256, num_disparities + 16)
                matcher = make_sgbm(num_disparities, block_size)
                print(f"[PARAM] numDisparities = {num_disparities}")

            elif key == ord("-"):
                block_size = max(3, block_size - 2)
                matcher = make_sgbm(num_disparities, block_size)
                print(f"[PARAM] blockSize = {block_size}")

            elif key in (ord("="), ord("+")):
                block_size = min(15, block_size + 2)
                matcher = make_sgbm(num_disparities, block_size)
                print(f"[PARAM] blockSize = {block_size}")

    cv2.destroyAllWindows()
    print("[DONE] Closed rectification/disparity viewer.")


if __name__ == "__main__":
    main()