"""
Detection probe — capture a frame from the stereo camera (or load a saved image),
then interactively browse every combination of checkerboard detection flags and
image pre-processing to find what works best for your board.

Controls:
  SPACE          — capture a fresh frame from the live camera
  RIGHT / LEFT   — cycle through detection combinations
  S              — save the current (working) combo to config suggestion printout
  Q              — quit

Results are printed to the terminal so you can copy the best flags into
check_corners.py / calibrate.py.
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "camera"))

import cv2
import numpy as np
from itertools import combinations
from config import (
    BOARD_COLS, BOARD_ROWS, BOARD_SQ_MM,
    load_camera_id, load_camera_settings, CALIB_DATA_DIR,
)
from camera import Camera

INNER = (BOARD_COLS - 1, BOARD_ROWS - 1)
DISPLAY_MAX_W = 1400

# ---------------------------------------------------------------------------
# All flag combos to try
# ---------------------------------------------------------------------------
BASE_FLAGS = [
    ("ADAPTIVE",   cv2.CALIB_CB_ADAPTIVE_THRESH),
    ("NORMALIZE",  cv2.CALIB_CB_NORMALIZE_IMAGE),
    ("FILTER",     cv2.CALIB_CB_FILTER_QUADS),
    ("FAST_CHECK", cv2.CALIB_CB_FAST_CHECK),
]

def _all_flag_combos():
    combos = []
    for r in range(1, len(BASE_FLAGS) + 1):
        for subset in combinations(BASE_FLAGS, r):
            names = " | ".join(n for n, _ in subset)
            flags = 0
            for _, f in subset:
                flags |= f
            combos.append((names, flags))
    return combos

# ---------------------------------------------------------------------------
# Pre-processing variants
# ---------------------------------------------------------------------------
def _preprocess_variants(gray):
    variants = [("raw", gray)]

    # CLAHE
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    variants.append(("CLAHE", clahe.apply(gray)))

    # Gaussian blur
    variants.append(("blur3", cv2.GaussianBlur(gray, (3, 3), 0)))
    variants.append(("blur5", cv2.GaussianBlur(gray, (5, 5), 0)))

    # Sharpened
    kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
    variants.append(("sharp", cv2.filter2D(gray, -1, kernel)))

    # Otsu threshold → back to uint8 for display, but use binary for detection
    _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    variants.append(("otsu", otsu))

    return variants

# ---------------------------------------------------------------------------
# Run detection on all combos × preprocessings
# ---------------------------------------------------------------------------
def _run_all(gray):
    flag_combos = _all_flag_combos()
    preprocesings = _preprocess_variants(gray)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    results = []
    for prep_name, prep_img in preprocesings:
        for flag_name, flags in flag_combos:
            ok, corners = cv2.findChessboardCorners(prep_img, INNER, flags)
            if ok:
                corners = cv2.cornerSubPix(prep_img, corners, (11, 11), (-1, -1), criteria)
            results.append({
                "prep":    prep_name,
                "flags":   flag_name,
                "ok":      ok,
                "corners": corners,
                "img":     prep_img,
            })
    return results

# ---------------------------------------------------------------------------
# Display
# ---------------------------------------------------------------------------
def _render(result, idx, total, frame_bgr):
    gray     = result["img"]
    vis      = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    ok       = result["ok"]
    corners  = result["corners"]

    if ok:
        cv2.drawChessboardCorners(vis, INNER, corners, True)

    status = "FOUND" if ok else "NOT FOUND"
    color  = (0, 220, 0) if ok else (0, 0, 220)
    label1 = f"{idx+1}/{total}  [{status}]"
    label2 = f"prep={result['prep']}  flags={result['flags']}"

    cv2.putText(vis, label1, (10, 35),  cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
    cv2.putText(vis, label2, (10, 70),  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 0), 2)

    # Side by side: original colour | detection overlay
    orig = frame_bgr.copy()
    cv2.putText(orig, "original", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)

    h = max(orig.shape[0], vis.shape[0])
    def pad(img):
        ph = h - img.shape[0]
        return np.pad(img, ((0, ph), (0, 0), (0, 0))) if ph > 0 else img

    row = np.hstack([pad(orig), pad(vis)])
    if row.shape[1] > DISPLAY_MAX_W:
        scale = DISPLAY_MAX_W / row.shape[1]
        row = cv2.resize(row, (DISPLAY_MAX_W, int(row.shape[0] * scale)))

    return row

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    ids = load_camera_id()
    if ids is None:
        print("No camera_ids.json found. Run camera/camera_scan.py first.")
        return

    cv2.namedWindow("Probe", cv2.WINDOW_NORMAL)

    frame_bgr = None
    results   = []
    idx       = 0

    print(f"Board: {BOARD_COLS}x{BOARD_ROWS} squares, {BOARD_SQ_MM}mm, inner corners {INNER}")
    print("SPACE = capture frame from camera  |  LEFT/RIGHT = cycle combos  |  Q = quit\n")

    # Open camera ready for capture
    with Camera(ids["camera"], name="Stereo") as cam:
        saved = load_camera_settings()
        if saved:
            cam.apply_settings(saved)

        half_w = int((cam.capture_width // 2) * cam.display_scale)
        half_h = int(cam.capture_height      * cam.display_scale)

        cv2.namedWindow("Live Left", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Live Left", half_w, half_h)

        print("Showing live feed — press SPACE to capture and probe...")

        while True:
            ret, full = cam.read()
            if not ret:
                continue

            mid     = cam.capture_width // 2
            lf_raw  = full[:, :mid]

            # Show live preview
            live_disp = cv2.resize(lf_raw, (half_w, half_h))
            if frame_bgr is not None:
                cv2.putText(live_disp, "Captured! R/L to browse", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Live Left", live_disp)

            # Show probe result if we have one
            if results:
                cv2.imshow("Probe", _render(results[idx], idx, len(results), frame_bgr))

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            elif key == ord(" "):
                frame_bgr = lf_raw.copy()
                gray      = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
                print("Running detection probe on captured frame...")
                results   = _run_all(gray)
                found     = [r for r in results if r["ok"]]
                idx       = 0
                # Jump to first success if any
                for i, r in enumerate(results):
                    if r["ok"]:
                        idx = i
                        break
                print(f"  {len(found)}/{len(results)} combos detected corners.")
                if found:
                    print("  Working combos:")
                    for r in found:
                        print(f"    prep={r['prep']}  flags={r['flags']}")
                else:
                    print("  None worked. Try better lighting or a flatter board hold.")

            elif key == 83 and results:   # right arrow
                idx = (idx + 1) % len(results)
            elif key == 81 and results:   # left arrow
                idx = (idx - 1) % len(results)

            elif key == ord("s") and results and results[idx]["ok"]:
                r = results[idx]
                print(f"\n--- Best combo to use ---")
                print(f"  prep:  {r['prep']}")
                print(f"  flags: cv2.{' | cv2.'.join(r['flags'].split(' | '))}")
                print(f"  (add these to check_corners.py / calibrate.py)\n")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
