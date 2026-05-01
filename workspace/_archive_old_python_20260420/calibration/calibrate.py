"""
Script 3 — Stereo calibration.
Works in two modes:

  --live    Open camera, capture pairs (SPACE), calibrate on C, quit on Q.
  --static  Read saved pairs from data/ and calibrate immediately. (default)

Detection uses findChessboardCornersSB (more robust) with a fallback to
the standard findChessboardCorners + subpixel refinement.

Prints per-pair reprojection errors and overall RMS. Saves to calib_results.json.

Usage:
  python calibrate.py --live
  python calibrate.py --static
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "camera"))

import cv2
import numpy as np
from pathlib import Path
from config import (
    BOARD_COLS, BOARD_ROWS, BOARD_SQ_MM,
    CALIB_DATA_DIR, load_camera_id, load_camera_settings, save_calib_results,
    DISPLAY_SCALE,
)
from camera import Camera

INNER        = (BOARD_COLS - 1, BOARD_ROWS - 1)
SAVE_DIR     = Path(CALIB_DATA_DIR)
WARMUP_FRAMES = 10
MIN_PAIRS    = 8
RMS_THRESHOLD = 1.0

SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
CALIB_CRITERIA  = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-7)


# ---------------------------------------------------------------------------
# Detection
# ---------------------------------------------------------------------------

def detect_corners(gray):
    """Try findChessboardCornersSB first, fall back to standard + subpix."""
    if hasattr(cv2, "findChessboardCornersSB"):
        try:
            flags = (cv2.CALIB_CB_EXHAUSTIVE
                     | cv2.CALIB_CB_ACCURACY
                     | cv2.CALIB_CB_NORMALIZE_IMAGE)
            ok, corners = cv2.findChessboardCornersSB(gray, INNER, flags)
            if ok and corners is not None:
                return True, corners.astype(np.float64).reshape(-1, 1, 2)
        except Exception:
            pass

    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
             | cv2.CALIB_CB_NORMALIZE_IMAGE
             | cv2.CALIB_CB_FAST_CHECK)
    ok, corners = cv2.findChessboardCorners(gray, INNER, flags)
    if ok and corners is not None:
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), SUBPIX_CRITERIA)
        return True, corners.astype(np.float64).reshape(-1, 1, 2)

    return False, None


def make_obj_points():
    cols, rows = INNER
    objp = np.zeros((cols * rows, 1, 3), dtype=np.float64)
    objp[:, 0, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * BOARD_SQ_MM
    return objp


# ---------------------------------------------------------------------------
# Dataset helpers
# ---------------------------------------------------------------------------

def load_pairs(save_dir: Path):
    lefts  = sorted(save_dir.glob("left_*.png"))
    rights = sorted(save_dir.glob("right_*.png"))
    n = min(len(lefts), len(rights))
    return list(zip(lefts[:n], rights[:n]))


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


def gather_detections(pairs):
    obj_template = make_obj_points()
    obj_pts, img_l, img_r = [], [], []
    expected = INNER[0] * INNER[1]

    for lp, rp in pairs:
        left  = cv2.imread(str(lp), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(rp), cv2.IMREAD_GRAYSCALE)
        if left is None or right is None:
            print(f"  {Path(lp).stem}: could not read image")
            continue
        ok_l, cl = detect_corners(left)
        ok_r, cr = detect_corners(right)
        tag = Path(lp).stem.replace("left_", "pair_")
        size_str = f"{left.shape[1]}x{left.shape[0]}"
        if ok_l and ok_r and cl.shape == (expected, 1, 2) and cr.shape == (expected, 1, 2):
            obj_pts.append(obj_template.copy())
            img_l.append(cl)
            img_r.append(cr)
            print(f"  {tag}  OK   ({size_str})")
        else:
            print(f"  {tag}  FAIL ({size_str})  L={'ok' if ok_l else 'no'} R={'ok' if ok_r else 'no'}")

    return obj_pts, img_l, img_r


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------

def run_calibration(obj_pts, img_l, img_r, img_size):
    rms_l, K_l, D_l, _, _ = cv2.calibrateCamera(
        obj_pts, img_l, img_size, None, None, criteria=CALIB_CRITERIA)
    rms_r, K_r, D_r, _, _ = cv2.calibrateCamera(
        obj_pts, img_r, img_size, None, None, criteria=CALIB_CRITERIA)

    rms, K_l, D_l, K_r, D_r, R, T, E, F = cv2.stereoCalibrate(
        obj_pts, img_l, img_r,
        K_l, D_l, K_r, D_r,
        img_size,
        criteria=CALIB_CRITERIA,
        flags=cv2.CALIB_FIX_INTRINSIC,
    )

    per_pair = []
    for i, (obj, il, ir) in enumerate(zip(obj_pts, img_l, img_r)):
        proj_l, _ = cv2.projectPoints(obj, np.zeros(3), np.zeros(3), K_l, D_l)
        proj_r, _ = cv2.projectPoints(obj, R, T, K_r, D_r)
        el = float(np.sqrt(np.mean((il - proj_l.reshape(-1, 1, 2))**2)))
        er = float(np.sqrt(np.mean((ir - proj_r.reshape(-1, 1, 2))**2)))
        per_pair.append({"pair": i, "L": round(el, 4), "R": round(er, 4)})

    return rms, K_l, D_l, K_r, D_r, R, T, per_pair


def print_results(rms, K_l, D_l, K_r, D_r, R, T, per_pair):
    print("\n" + "="*60)
    print(f"  STEREO RMS: {rms:.6f} px  ({'OK' if rms < RMS_THRESHOLD else 'ABOVE THRESHOLD'})")
    print("="*60)
    print("\nPer-pair reprojection errors:")
    for p in per_pair:
        flag = "  <--" if max(p["L"], p["R"]) > RMS_THRESHOLD else ""
        print(f"  pair {p['pair']:04d}  L={p['L']} px  R={p['R']} px{flag}")
    print(f"\nLeft  K:\n{K_l}")
    print(f"Left  D: {D_l.ravel()}")
    print(f"\nRight K:\n{K_r}")
    print(f"Right D: {D_r.ravel()}")
    print(f"\nR:\n{R}")
    print(f"T (m): {T.ravel()}")
    print(f"Baseline: {np.linalg.norm(T)*1000:.2f} mm\n")


# ---------------------------------------------------------------------------
# Overlay helper
# ---------------------------------------------------------------------------

def overlay_text(img, lines, origin=(20, 40), line_height=32, color=(0, 255, 0)):
    x, y = origin
    for i, line in enumerate(lines):
        pos = (x, y + i * line_height)
        cv2.putText(img, line, (pos[0]+2, pos[1]+2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, line, pos,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)


# ---------------------------------------------------------------------------
# Static mode
# ---------------------------------------------------------------------------

def run_static():
    pairs = load_pairs(SAVE_DIR)
    if not pairs:
        print(f"No pairs in {SAVE_DIR}. Run capture_pairs.py first.")
        return

    print(f"Processing {len(pairs)} pairs...")
    obj_pts, img_l, img_r = gather_detections(pairs)

    if len(obj_pts) < MIN_PAIRS:
        print(f"Only {len(obj_pts)} valid pairs (need {MIN_PAIRS}). Capture more.")
        return

    img = cv2.imread(str(pairs[0][0]), cv2.IMREAD_GRAYSCALE)
    img_size = (img.shape[1], img.shape[0])

    print(f"Calibrating with {len(obj_pts)} valid pairs...")
    rms, K_l, D_l, K_r, D_r, R, T, per_pair = run_calibration(obj_pts, img_l, img_r, img_size)
    print_results(rms, K_l, D_l, K_r, D_r, R, T, per_pair)

    save_calib_results({
        "rms": rms, "pairs_used": len(obj_pts),
        "image_size": list(img_size),
        "K_left": K_l.tolist(), "D_left": D_l.tolist(),
        "K_right": K_r.tolist(), "D_right": D_r.tolist(),
        "R": R.tolist(), "T": T.tolist(),
        "baseline_mm": float(np.linalg.norm(T) * 1000),
        "per_pair_errors": per_pair,
    })


# ---------------------------------------------------------------------------
# Live mode
# ---------------------------------------------------------------------------

def run_live():
    SAVE_DIR.mkdir(parents=True, exist_ok=True)

    ids = load_camera_id()
    if ids is None:
        print("No camera_ids.json found. Run camera/camera_scan.py first.")
        return

    with Camera(ids["camera"], name="Stereo") as cam:
        saved_settings = load_camera_settings()
        if saved_settings:
            cam.apply_settings(saved_settings)

        print("Warming up camera...")
        for _ in range(WARMUP_FRAMES):
            cam.read()

        eye_w    = cam.capture_width // 2
        eye_h    = cam.capture_height
        img_size = (eye_w, eye_h)

        idx       = get_start_index(SAVE_DIR)
        pair_count = idx
        status    = [f"Pairs: {pair_count}", "SPACE=capture  C=calibrate  Q=quit"]
        color     = (200, 200, 200)

        print(f"Saving to: {SAVE_DIR}")
        print("SPACE=capture  C=calibrate  Q=quit\n")

        while True:
            ret, full = cam.read()
            if not ret:
                continue

            left_raw  = full[:, :eye_w]
            right_raw = full[:, eye_w:]

            display = cv2.hconcat([left_raw, right_raw])
            display = cv2.resize(display, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE,
                                 interpolation=cv2.INTER_AREA)
            overlay_text(display, status, color=color)
            cv2.imshow("Stereo Capture + Calibrate", display)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break

            elif key == ord(' '):
                lp = SAVE_DIR / f"left_{idx:04d}.png"
                rp = SAVE_DIR / f"right_{idx:04d}.png"
                cv2.imwrite(str(lp), left_raw)
                cv2.imwrite(str(rp), right_raw)
                pair_count += 1
                idx += 1
                print(f"  Saved pair {idx-1:04d}  (total: {pair_count})")
                status = [f"Pairs: {pair_count}", "SPACE=capture  C=calibrate  Q=quit"]
                color  = (200, 200, 200)

            elif key == ord('c'):
                if pair_count < MIN_PAIRS:
                    msg = f"Need {MIN_PAIRS} pairs — only {pair_count} saved"
                    print(f"  {msg}")
                    status = [f"Pairs: {pair_count}  (need {MIN_PAIRS})", "SPACE=capture  Q=quit"]
                    color  = (0, 165, 255)
                else:
                    print(f"  Calibrating with {pair_count} pairs...")
                    status = ["Calibrating — please wait..."]
                    color  = (200, 200, 0)
                    overlay_text(display, status, color=color)
                    cv2.imshow("Stereo Capture + Calibrate", display)
                    cv2.waitKey(1)

                    try:
                        obj_pts, img_l, img_r = gather_detections(load_pairs(SAVE_DIR))
                        if len(obj_pts) < MIN_PAIRS:
                            raise RuntimeError(f"Only {len(obj_pts)} valid detections")
                        rms, K_l, D_l, K_r, D_r, R, T, per_pair = run_calibration(
                            obj_pts, img_l, img_r, img_size)
                        print_results(rms, K_l, D_l, K_r, D_r, R, T, per_pair)
                        save_calib_results({
                            "rms": rms, "pairs_used": len(obj_pts),
                            "image_size": list(img_size),
                            "K_left": K_l.tolist(), "D_left": D_l.tolist(),
                            "K_right": K_r.tolist(), "D_right": D_r.tolist(),
                            "R": R.tolist(), "T": T.tolist(),
                            "baseline_mm": float(np.linalg.norm(T) * 1000),
                            "per_pair_errors": per_pair,
                        })
                        if rms < RMS_THRESHOLD:
                            status = [f"Pairs: {pair_count}  RMS={rms:.4f} OK",
                                      "Saved!  SPACE=add more  C=recalibrate  Q=quit"]
                            color  = (0, 255, 80)
                        else:
                            status = [f"Pairs: {pair_count}  RMS={rms:.4f} HIGH",
                                      "Add more varied pairs and recalibrate.  Q=quit"]
                            color  = (0, 80, 255)
                    except Exception as e:
                        print(f"  Calibration failed: {e}")
                        status = [f"Calibration failed: {e}", "SPACE=capture  C=retry  Q=quit"]
                        color  = (0, 80, 255)

    cv2.destroyAllWindows()
    print("\nDone.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "--static"
    if mode == "--live":
        run_live()
    else:
        run_static()
