"""
Calibrate the HBVCAM stereo pair and save triangulation-ready matrices.
"""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
import sys

import cv2
import numpy as np

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import config


CRITERIA_SUBPIX = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 1e-4)
CRITERIA_CALIB = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-7)


def _print_matrix(name: str, mat: np.ndarray) -> None:
    print(f"\n{name}:")
    print(np.asarray(mat))


def print_results(results: dict[str, np.ndarray]) -> None:
    """
    Print intrinsics/extrinsics for each camera and the core stereo matrices.
    """
    np.set_printoptions(precision=6, suppress=True)

    img_w, img_h = (int(results["image_size"][0]), int(results["image_size"][1]))
    print("\n" + "=" * 70)
    print(f"Stereo Calibration Results  (image size: {img_w}x{img_h})")
    print("=" * 70)

    print(f"RMS left:   {float(results['rms_left'][0]):.6f} px")
    print(f"RMS right:  {float(results['rms_right'][0]):.6f} px")
    print(f"RMS stereo: {float(results['rms_stereo'][0]):.6f} px")
    print(f"Baseline:   {float(results['baseline_mm'][0]):.3f} mm")

    # Intrinsics
    _print_matrix("Left K (camera_matrix)", results["left_camera_matrix"])
    _print_matrix("Left D (distortion_coefficients)", results["left_distortion_coefficients"].ravel())

    _print_matrix("Right K (camera_matrix)", results["right_camera_matrix"])
    _print_matrix("Right D (distortion_coefficients)", results["right_distortion_coefficients"].ravel())

    # Extrinsics (Left -> Right)
    _print_matrix("R (left_to_right)", results["stereo_rotation_left_to_right"])
    _print_matrix("T_mm (left_to_right)", results["stereo_translation_left_to_right_mm"].ravel())

    # Core stereo matrices
    _print_matrix("Essential E", results["essential_matrix"])
    _print_matrix("Fundamental F", results["fundamental_matrix"])

    # Rectification / projection
    _print_matrix("Rectification R1 (left)", results["rectification_left"])
    _print_matrix("Rectification R2 (right)", results["rectification_right"])
    _print_matrix("Projection P1 (left rectified)", results["projection_left_rectified"])
    _print_matrix("Projection P2 (right rectified)", results["projection_right_rectified"])
    _print_matrix("Q (disparity_to_depth)", results["disparity_to_depth_Q"])

    # Useful raw projection matrices too
    _print_matrix("Projection P_left_raw = K_left [I|0]", results["projection_left_raw"])
    _print_matrix("Projection P_right_raw = K_right [R|T]", results["projection_right_raw"])


def load_pairs(pair_dir: Path) -> list[tuple[str, Path, Path]]:
    folder_pairs: list[tuple[str, Path, Path]] = []
    for pair_path in sorted(pair_dir.glob("pair_*")):
        left = pair_path / "left.png"
        right = pair_path / "right.png"
        if left.exists() and right.exists():
            folder_pairs.append((pair_path.name, left, right))

    if folder_pairs:
        return folder_pairs

    # Backward-compatible loader for old left_0000.png/right_0000.png datasets.
    flat_pairs: list[tuple[str, Path, Path]] = []
    lefts = sorted(pair_dir.glob("left_*.png"))
    for left in lefts:
        index = left.stem.split("_", 1)[1]
        right = pair_dir / f"right_{index}.png"
        if right.exists():
            flat_pairs.append((f"pair_{index}", left, right))
    return flat_pairs


def object_points() -> np.ndarray:
    cols, rows = config.BOARD_INNER_CORNERS
    points = np.zeros((cols * rows, 3), np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    points *= float(config.BOARD_SQUARE_SIZE_MM)
    return points


def detect_corners(gray: np.ndarray) -> tuple[bool, np.ndarray | None]:
    if hasattr(cv2, "findChessboardCornersSB"):
        flags_sb = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
        ok, corners = cv2.findChessboardCornersSB(gray, config.BOARD_INNER_CORNERS, flags_sb)
        if ok and corners is not None:
            return True, corners.astype(np.float32)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, config.BOARD_INNER_CORNERS, flags)
    if not ok or corners is None:
        return False, None
    corners = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), CRITERIA_SUBPIX)
    return True, corners.astype(np.float32)


def collect_detections(pairs: list[tuple[str, Path, Path]]):
    template = object_points()
    objpoints: list[np.ndarray] = []
    left_points: list[np.ndarray] = []
    right_points: list[np.ndarray] = []
    used_pairs: list[str] = []
    image_size: tuple[int, int] | None = None

    print(f"Checking {len(pairs)} saved pair(s)...")
    for name, left_path, right_path in pairs:
        left = cv2.imread(str(left_path), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(right_path), cv2.IMREAD_GRAYSCALE)
        if left is None or right is None:
            print(f"  {name}: could not read images")
            continue
        if left.shape != right.shape:
            print(f"  {name}: left/right sizes differ")
            continue

        current_size = (left.shape[1], left.shape[0])
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            print(f"  {name}: skipped, size {current_size} does not match {image_size}")
            continue

        ok_left, corners_left = detect_corners(left)
        ok_right, corners_right = detect_corners(right)
        if ok_left and ok_right and corners_left is not None and corners_right is not None:
            objpoints.append(template.copy())
            left_points.append(corners_left)
            right_points.append(corners_right)
            used_pairs.append(name)
            print(f"  {name}: OK")
        else:
            print(f"  {name}: FAIL L={'ok' if ok_left else 'no'} R={'ok' if ok_right else 'no'}")

    return objpoints, left_points, right_points, used_pairs, image_size


def calibrate(objpoints, left_points, right_points, image_size: tuple[int, int]):
    rms_left, k_left, d_left, _, _ = cv2.calibrateCamera(
        objpoints,
        left_points,
        image_size,
        None,
        None,
        criteria=CRITERIA_CALIB,
    )
    rms_right, k_right, d_right, _, _ = cv2.calibrateCamera(
        objpoints,
        right_points,
        image_size,
        None,
        None,
        criteria=CRITERIA_CALIB,
    )

    rms_stereo, k_left, d_left, k_right, d_right, rot, trans, essential, fundamental = cv2.stereoCalibrate(
        objpoints,
        left_points,
        right_points,
        k_left,
        d_left,
        k_right,
        d_right,
        image_size,
        criteria=CRITERIA_CALIB,
        flags=cv2.CALIB_FIX_INTRINSIC,
    )

    r1, r2, p1, p2, q, roi_left, roi_right = cv2.stereoRectify(
        k_left,
        d_left,
        k_right,
        d_right,
        image_size,
        rot,
        trans,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0,
    )

    projection_left_raw = k_left @ np.hstack([np.eye(3), np.zeros((3, 1))])
    projection_right_raw = k_right @ np.hstack([rot, trans])

    return {
        "image_size": np.array(image_size, dtype=np.int32),
        "board_inner_corners": np.array(config.BOARD_INNER_CORNERS, dtype=np.int32),
        "board_square_size_mm": np.array([config.BOARD_SQUARE_SIZE_MM], dtype=np.float64),
        "rms_left": np.array([rms_left], dtype=np.float64),
        "rms_right": np.array([rms_right], dtype=np.float64),
        "rms_stereo": np.array([rms_stereo], dtype=np.float64),
        "left_camera_matrix": k_left,
        "left_distortion_coefficients": d_left,
        "right_camera_matrix": k_right,
        "right_distortion_coefficients": d_right,
        "stereo_rotation_left_to_right": rot,
        "stereo_translation_left_to_right_mm": trans,
        "essential_matrix": essential,
        "fundamental_matrix": fundamental,
        "rectification_left": r1,
        "rectification_right": r2,
        "projection_left_rectified": p1,
        "projection_right_rectified": p2,
        "disparity_to_depth_Q": q,
        "roi_left": np.array(roi_left, dtype=np.int32),
        "roi_right": np.array(roi_right, dtype=np.int32),
        "projection_left_raw": projection_left_raw,
        "projection_right_raw": projection_right_raw,
        "baseline_mm": np.array([float(np.linalg.norm(trans))], dtype=np.float64),
    }


def save_matrix_set(results: dict[str, np.ndarray], used_pairs: list[str]) -> None:
    out_dir = config.ACTIVE_MATRIX_DIR
    out_dir.mkdir(parents=True, exist_ok=True)

    np.savez(out_dir / "stereo_calibration.npz", **results)

    matrix_names = [
        "left_camera_matrix",
        "left_distortion_coefficients",
        "right_camera_matrix",
        "right_distortion_coefficients",
        "stereo_rotation_left_to_right",
        "stereo_translation_left_to_right_mm",
        "essential_matrix",
        "fundamental_matrix",
        "rectification_left",
        "rectification_right",
        "projection_left_rectified",
        "projection_right_rectified",
        "projection_left_raw",
        "projection_right_raw",
        "disparity_to_depth_Q",
    ]
    for name in matrix_names:
        np.savetxt(out_dir / f"{name}.csv", np.asarray(results[name]), delimiter=",", fmt="%.12g")

    summary = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "units": {
            "board_square_size": "millimeters",
            "translation": "millimeters",
            "baseline": "millimeters",
        },
        "image_size": results["image_size"].tolist(),
        "board_inner_corners": results["board_inner_corners"].tolist(),
        "board_square_size_mm": float(results["board_square_size_mm"][0]),
        "pairs_used": used_pairs,
        "rms_left": float(results["rms_left"][0]),
        "rms_right": float(results["rms_right"][0]),
        "rms_stereo": float(results["rms_stereo"][0]),
        "baseline_mm": float(results["baseline_mm"][0]),
        "matrix_files": [f"{name}.csv" for name in matrix_names],
        "npz_file": "stereo_calibration.npz",
    }
    with config.ACTIVE_CALIBRATION_JSON.open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
        f.write("\n")


def main() -> None:
    config.ensure_directories()
    pairs = load_pairs(config.STEREO_PAIR_DIR)
    if not pairs:
        print(f"No stereo pairs found in {config.STEREO_PAIR_DIR}")
        print("Run python capture_pairs.py first.")
        return

    objpoints, left_points, right_points, used_pairs, image_size = collect_detections(pairs)
    if image_size is None:
        print("No readable image pairs found.")
        return
    if len(objpoints) < config.MIN_CALIBRATION_PAIRS:
        print(f"Only {len(objpoints)} usable pair(s). Need at least {config.MIN_CALIBRATION_PAIRS}.")
        return

    print(f"\nCalibrating with {len(objpoints)} pair(s) at {image_size[0]}x{image_size[1]}...")
    results = calibrate(objpoints, left_points, right_points, image_size)
    print_results(results)
    save_matrix_set(results, used_pairs)

    print("\nCalibration saved.")
    print(f"  Folder:       {config.ACTIVE_MATRIX_DIR}")
    print(f"  RMS stereo:   {float(results['rms_stereo'][0]):.6f} px")
    print(f"  Baseline:     {float(results['baseline_mm'][0]):.3f} mm")
    print("  Config var:   config.STEREO_CALIBRATION")


if __name__ == "__main__":
    main()
