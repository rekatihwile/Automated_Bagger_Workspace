import cv2
import numpy as np
import glob
import json
import sys
from pathlib import Path

# Add workspace root to Python path
ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT))

from hardware.cameras import StereoCameras


# ============================================================
# CONFIG
# ============================================================
PATTERN_SIZE    = (6, 7)      # inner corners (cols, rows)
SQUARE_SIZE_MM  = 25.0
MAX_VIEWS       = 30
MIN_VIEWS       = 8           # need this many before calibration is attempted
RMS_THRESHOLD   = 1.0         # px — pairs producing RMS above this are rejected
DISPLAY_SCALE   = 0.6
WARMUP_FRAMES   = 10
SAVE_DEBUG      = False

BASE_DIR    = Path(__file__).resolve().parent
SAVE_DIR    = BASE_DIR / "calib_pairs"
DEBUG_DIR   = BASE_DIR / "checkerboard_debug"
CALIB_FILE  = BASE_DIR / "stereo_checkerboard_fisheye_calib.npz"
RECTIFY_FILE= BASE_DIR / "stereo_checkerboard_fisheye_rectify_maps.npz"
JSON_FILE   = BASE_DIR / "stereo_checkerboard_fisheye_calib.json"

SAVE_DIR.mkdir(parents=True, exist_ok=True)

SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
CALIB_CRITERIA  = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-7)


# ============================================================
# HELPERS — IMAGE INDEX
# ============================================================
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


def load_all_pairs(save_dir: Path):
    lefts  = sorted(save_dir.glob("left_*.png"))
    rights = sorted(save_dir.glob("right_*.png"))
    n = min(len(lefts), len(rights))
    return list(zip(lefts[:n], rights[:n]))


# ============================================================
# CHECKERBOARD DETECTION
# ============================================================
def make_object_points(pattern_size, square_size_mm):
    cols, rows = pattern_size
    objp = np.zeros((cols * rows, 1, 3), dtype=np.float64)
    objp[:, 0, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * float(square_size_mm)
    return objp


def detect_checkerboard(gray, pattern_size):
    found, corners = False, None

    if hasattr(cv2, "findChessboardCornersSB"):
        try:
            flags_sb = (
                cv2.CALIB_CB_EXHAUSTIVE
                | cv2.CALIB_CB_ACCURACY
                | cv2.CALIB_CB_NORMALIZE_IMAGE
            )
            found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags_sb)
        except Exception:
            found, corners = False, None

    if not found or corners is None:
        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH
            | cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_FAST_CHECK
        )
        found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
        if found and corners is not None:
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), SUBPIX_CRITERIA)

    if not found or corners is None:
        return False, None

    corners = np.asarray(corners, dtype=np.float64).reshape(-1, 1, 2)
    expected = pattern_size[0] * pattern_size[1]
    if corners.shape != (expected, 1, 2):
        return False, None

    return True, corners


# ============================================================
# DATASET BUILD
# ============================================================
def gather_points(pairs, pattern_size):
    obj_template = make_object_points(pattern_size, SQUARE_SIZE_MM)
    objpoints, imgpointsL, imgpointsR = [], [], []

    for l_path, r_path in pairs:
        imgL = cv2.imread(str(l_path))
        imgR = cv2.imread(str(r_path))
        if imgL is None or imgR is None:
            continue

        grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

        foundL, cornersL = detect_checkerboard(grayL, pattern_size)
        foundR, cornersR = detect_checkerboard(grayR, pattern_size)

        if foundL and foundR:
            expected = pattern_size[0] * pattern_size[1]
            cL = np.asarray(cornersL, dtype=np.float64).reshape(-1, 1, 2)
            cR = np.asarray(cornersR, dtype=np.float64).reshape(-1, 1, 2)
            if cL.shape == (expected, 1, 2) and cR.shape == (expected, 1, 2):
                objpoints.append(obj_template.copy())
                imgpointsL.append(cL)
                imgpointsR.append(cR)

    return objpoints, imgpointsL, imgpointsR


def thin_dataset(objpoints, imgpointsL, imgpointsR, max_views):
    if len(objpoints) <= max_views:
        return objpoints, imgpointsL, imgpointsR
    idx = sorted(set(np.linspace(0, len(objpoints) - 1, max_views).astype(int).tolist()))
    return [objpoints[i] for i in idx], [imgpointsL[i] for i in idx], [imgpointsR[i] for i in idx]


# ============================================================
# CALIBRATION
# ============================================================
def calibrate_single_fisheye(objpoints, imgpoints, img_size):
    K = np.eye(3, dtype=np.float64)
    D = np.zeros((4, 1), dtype=np.float64)
    flags = (
        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
        | cv2.fisheye.CALIB_CHECK_COND
        | cv2.fisheye.CALIB_FIX_SKEW
    )
    rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        objectPoints=objpoints,
        imagePoints=imgpoints,
        image_size=img_size,
        K=K, D=D, rvecs=None, tvecs=None,
        flags=flags,
        criteria=CALIB_CRITERIA,
    )
    return rms, K, D, rvecs, tvecs


def sanitize_stereo_inputs(objpoints, imgpointsL, imgpointsR, pattern_size):
    expected = pattern_size[0] * pattern_size[1]
    obj_c, left_c, right_c = [], [], []
    for obj, lpt, rpt in zip(objpoints, imgpointsL, imgpointsR):
        obj = np.asarray(obj, dtype=np.float64).reshape(-1, 1, 3)
        lpt = np.asarray(lpt, dtype=np.float64).reshape(-1, 1, 2)
        rpt = np.asarray(rpt, dtype=np.float64).reshape(-1, 1, 2)
        if obj.shape == (expected, 1, 3) and lpt.shape == (expected, 1, 2) and rpt.shape == (expected, 1, 2):
            obj_c.append(obj); left_c.append(lpt); right_c.append(rpt)
    return obj_c, left_c, right_c


def calibrate_stereo_fisheye(objpoints, imgpointsL, imgpointsR, K1, D1, K2, D2, img_size, pattern_size):
    objpoints, imgpointsL, imgpointsR = sanitize_stereo_inputs(
        objpoints, imgpointsL, imgpointsR, pattern_size
    )
    if len(objpoints) < MIN_VIEWS:
        raise RuntimeError(f"Only {len(objpoints)} clean stereo pairs after sanitization.")

    R_init = np.eye(3, dtype=np.float64)
    T_init = np.zeros((3, 1), dtype=np.float64)

    out = cv2.fisheye.stereoCalibrate(
        objectPoints=objpoints,
        imagePoints1=imgpointsL,
        imagePoints2=imgpointsR,
        K1=K1, D1=D1, K2=K2, D2=D2,
        imageSize=img_size,
        R=R_init, T=T_init,
        flags=cv2.fisheye.CALIB_FIX_INTRINSIC,
        criteria=CALIB_CRITERIA,
    )
    rms, K1o, D1o, K2o, D2o, R, T = out[:7]
    return rms, K1o, D1o, K2o, D2o, R, T


def save_results(K1o, D1o, K2o, D2o, R, T, rmsL, rmsR, rmsStereo, img_size, objpoints, per_pair_used):
    np.savez(
        CALIB_FILE,
        model="fisheye_checkerboard",
        imageSize=np.array(img_size, dtype=np.int32),
        patternSize=np.array(PATTERN_SIZE, dtype=np.int32),
        squareSizeMM=float(SQUARE_SIZE_MM),
        K1=K1o, D1=D1o, K2=K2o, D2=D2o,
        R=R, T=T,
        rmsLeft=float(rmsL), rmsRight=float(rmsR), rmsStereo=float(rmsStereo),
    )

    R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
        K1o, D1o, K2o, D2o, img_size, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY,
        newImageSize=img_size, balance=0.0, fov_scale=1.0,
    )
    map1L, map2L = cv2.fisheye.initUndistortRectifyMap(K1o, D1o, R1, P1, img_size, cv2.CV_16SC2)
    map1R, map2R = cv2.fisheye.initUndistortRectifyMap(K2o, D2o, R2, P2, img_size, cv2.CV_16SC2)
    np.savez(RECTIFY_FILE, R1=R1, R2=R2, P1=P1, P2=P2, Q=Q,
             map1L=map1L, map2L=map2L, map1R=map1R, map2R=map2R)

    report = {
        "model": "fisheye_checkerboard",
        "image_size": list(img_size),
        "pattern_size_inner_corners": list(PATTERN_SIZE),
        "square_size_mm": SQUARE_SIZE_MM,
        "usable_stereo_pairs": len(objpoints),
        "rms_left": float(rmsL),
        "rms_right": float(rmsR),
        "rms_stereo": float(rmsStereo),
        "baseline_mm": float(np.linalg.norm(T)),
        "K1": K1o.tolist(), "D1": D1o.tolist(),
        "K2": K2o.tolist(), "D2": D2o.tolist(),
        "R": R.tolist(), "T": T.tolist(),
        "used_pairs": per_pair_used,
    }
    with open(JSON_FILE, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)


# ============================================================
# LIVE CALIBRATION ATTEMPT
# ============================================================
def overlay_text(img, lines, origin=(20, 40), line_height=32,
                 color=(0, 255, 0), bg_color=(0, 0, 0)):
    """Render multi-line text with a dark drop-shadow for readability."""
    x, y = origin
    for i, line in enumerate(lines):
        pos = (x, y + i * line_height)
        cv2.putText(img, line, (pos[0]+2, pos[1]+2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, bg_color, 3, cv2.LINE_AA)
        cv2.putText(img, line, pos,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)


def try_calibrate_with_all_pairs(img_size):
    """
    Load every saved pair, run full stereo fisheye calibration.
    Returns (success, rms_stereo, status_message).
    """
    pairs = load_all_pairs(SAVE_DIR)
    if len(pairs) < MIN_VIEWS:
        return None, None, f"Accumulating ({len(pairs)}/{MIN_VIEWS} min pairs)"

    objpoints, imgpointsL, imgpointsR = gather_points(pairs, PATTERN_SIZE)

    if len(objpoints) < MIN_VIEWS:
        return False, None, f"Only {len(objpoints)} detected — need {MIN_VIEWS}"

    objpoints, imgpointsL, imgpointsR = thin_dataset(
        objpoints, imgpointsL, imgpointsR, MAX_VIEWS
    )

    try:
        rmsL, K1, D1, _, _ = calibrate_single_fisheye(objpoints, imgpointsL, img_size)
        rmsR, K2, D2, _, _ = calibrate_single_fisheye(objpoints, imgpointsR, img_size)
        rmsStereo, K1o, D1o, K2o, D2o, R, T = calibrate_stereo_fisheye(
            objpoints, imgpointsL, imgpointsR,
            K1, D1, K2, D2, img_size, PATTERN_SIZE
        )
    except Exception as e:
        return False, None, f"Calib error: {e}"

    if rmsStereo > RMS_THRESHOLD:
        return False, rmsStereo, f"RMS {rmsStereo:.3f} > threshold {RMS_THRESHOLD}"

    # Passed — persist results
    used_names = [{"left": Path(l).name, "right": Path(r).name} for l, r in pairs]
    save_results(K1o, D1o, K2o, D2o, R, T, rmsL, rmsR, rmsStereo, img_size, objpoints, used_names)
    return True, rmsStereo, f"OK  RMS={rmsStereo:.4f}  base={np.linalg.norm(T):.1f}mm"


# ============================================================
# MAIN CAPTURE + CALIBRATE LOOP
# ============================================================
def main():
    cams = StereoCameras()
    cams.open()

    print("Warming up cameras…")
    for _ in range(WARMUP_FRAMES):
        left, right = cams.read_pair()

    img_size = (left.shape[1], left.shape[0])   # (width, height)
    idx = get_start_index(SAVE_DIR)
    pair_count = idx                             # how many pairs already on disk
    status_lines = [
        f"Pairs on disk: {pair_count}",
        "SPACE=capture  C=calibrate  Q=quit",
    ]
    status_color = (200, 200, 200)

    print(f"\nSaving pairs to: {SAVE_DIR}")
    print(f"SPACE = capture  |  C = calibrate  |  Q = quit\n")

    while True:
        left, right = cams.read_pair()

        # ---- build display ----
        display = cv2.hconcat([left, right])
        display_small = cv2.resize(display, None,
                                   fx=DISPLAY_SCALE, fy=DISPLAY_SCALE,
                                   interpolation=cv2.INTER_AREA)
        overlay_text(display_small, status_lines, color=status_color)
        cv2.imshow("Stereo Capture + Calibrate", display_small)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        if key == ord(' '):
            left_path  = SAVE_DIR / f"left_{idx:04d}.png"
            right_path = SAVE_DIR / f"right_{idx:04d}.png"
            cv2.imwrite(str(left_path),  left)
            cv2.imwrite(str(right_path), right)
            idx += 1
            pair_count += 1
            print(f"[{idx-1:04d}] Saved pair  (total on disk: {pair_count})")
            status_lines = [
                f"Pairs on disk: {pair_count}",
                "SPACE=capture  C=calibrate  Q=quit",
            ]
            status_color = (200, 200, 200)

        elif key == ord('c'):
            if pair_count < MIN_VIEWS:
                msg = f"Need {MIN_VIEWS} pairs to calibrate — only {pair_count} on disk"
                print(f"       {msg}")
                status_lines = [
                    f"Pairs: {pair_count}  (need {MIN_VIEWS})",
                    msg,
                    "SPACE=capture  C=calibrate  Q=quit",
                ]
                status_color = (0, 165, 255)
            else:
                print(f"Calibrating with {pair_count} pairs…")
                status_lines = ["Calibrating — please wait…"]
                status_color = (200, 200, 0)
                # Briefly show the "calibrating" overlay before blocking call
                display_small_copy = display_small.copy()
                overlay_text(display_small_copy, status_lines, color=status_color)
                cv2.imshow("Stereo Capture + Calibrate", display_small_copy)
                cv2.waitKey(1)

                success, rms, msg = try_calibrate_with_all_pairs(img_size)

                if success is None:
                    print(f"       {msg}")
                    status_lines = [f"Pairs: {pair_count}  ({msg})",
                                    "SPACE=capture  C=calibrate  Q=quit"]
                    status_color = (200, 200, 200)
                elif success:
                    print(f"       ✓ Valid — {msg}")
                    status_lines = [f"Pairs: {pair_count}  ✓ {msg}",
                                    "Calibration saved!  SPACE=add more  C=recalibrate  Q=quit"]
                    status_color = (0, 255, 80)
                else:
                    print(f"       ✗ Failed — {msg}")
                    status_lines = [f"Pairs: {pair_count}  ✗ FAILED",
                                    f"Reason: {msg}",
                                    "SPACE=capture  C=retry  Q=quit"]
                    status_color = (0, 80, 255)

    cv2.destroyAllWindows()
    print("\nDone.")


if __name__ == "__main__":
    main()