"""
Script 2 — Corner detection check on saved stereo pairs.

Reads all left_NNNN.png / right_NNNN.png from data/, runs corner detection,
and shows which pairs have valid detections in both eyes.

Controls:
  RIGHT / LEFT arrow  — next / previous pair
  Q                   — quit
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np
from config import (
    CALIB_DATA_DIR,
    BOARD_DICT, BOARD_COLS, BOARD_ROWS, BOARD_SQ_MM, BOARD_MARKER_MM,
)

DISPLAY_MAX_H = 600   # max height for the check window


def _load_pairs(data_dir):
    from pathlib import Path
    save_dir = Path(data_dir)
    lefts  = sorted(save_dir.glob("left_*.png"))
    rights = sorted(save_dir.glob("right_*.png"))
    n = min(len(lefts), len(rights))
    return [(str(l), str(r)) for l, r in zip(lefts[:n], rights[:n])]


def _build_detector():
    """Build the corner detector from config. Returns (detect_fn, draw_fn)."""
    if BOARD_DICT is None or BOARD_COLS is None or BOARD_ROWS is None:
        raise RuntimeError(
            "Board parameters not set in config.py.\n"
            "Fill in BOARD_DICT, BOARD_COLS, BOARD_ROWS, BOARD_SQ_MM "
            "(and BOARD_MARKER_MM for CharUco) then re-run."
        )

    dict_id = getattr(cv2.aruco, BOARD_DICT)
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

    if BOARD_MARKER_MM is not None:
        # CharUco board
        sq_m  = BOARD_SQ_MM   / 1000.0
        mrk_m = BOARD_MARKER_MM / 1000.0
        board = cv2.aruco.CharucoBoard((BOARD_COLS, BOARD_ROWS), sq_m, mrk_m, aruco_dict)
        detector = cv2.aruco.CharucoDetector(board)

        def detect(gray):
            corners, ids, _, _ = detector.detectBoard(gray)
            ok = corners is not None and len(corners) >= 4
            return ok, corners, ids

        def draw(img, corners, ids):
            if corners is not None and ids is not None:
                cv2.aruco.drawDetectedCornersCharuco(img, corners, ids)
            return img
    else:
        # Plain checkerboard fallback
        pattern = (BOARD_COLS - 1, BOARD_ROWS - 1)
        flags   = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE

        def detect(gray):
            ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
            if ok:
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners  = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            return ok, corners, None

        def draw(img, corners, ids):
            cv2.drawChessboardCorners(img, pattern, corners, True)
            return img

    return detect, draw


def _thumb(img, max_h):
    h, w = img.shape[:2]
    if h > max_h:
        img = cv2.resize(img, (int(w * max_h / h), max_h))
    return img


def main():
    pairs = _load_pairs(CALIB_DATA_DIR)
    if not pairs:
        print(f"No pairs found in {CALIB_DATA_DIR}. Run capture_pairs.py first.")
        return

    detect, draw = _build_detector()

    results = []
    print(f"Checking {len(pairs)} pairs...\n")
    for i, (lp, rp) in enumerate(pairs):
        left  = cv2.imread(lp)
        right = cv2.imread(rp)
        gl    = cv2.cvtColor(left,  cv2.COLOR_BGR2GRAY)
        gr    = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

        ok_l, cl, il = detect(gl)
        ok_r, cr, ir = detect(gr)
        both = ok_l and ok_r

        status = "OK  " if both else f"FAIL({'L' if not ok_l else ''} {'R' if not ok_r else ''})"
        print(f"  pair_{i:03d}  {status}")
        results.append((i, both, left, right, cl, il, cr, ir))

    good = sum(1 for r in results if r[1])
    print(f"\n{good}/{len(results)} pairs valid for calibration.")
    print("RIGHT/LEFT arrows to browse  |  Q to quit\n")

    cv2.namedWindow("Check", cv2.WINDOW_NORMAL)
    idx = 0

    while True:
        i, both, left, right, cl, il, cr, ir = results[idx]

        vis_l = left.copy()
        vis_r = right.copy()
        if cl is not None:
            draw(vis_l, cl, il)
        if cr is not None:
            draw(vis_r, cr, ir)

        row      = np.hstack([_thumb(vis_l, DISPLAY_MAX_H), _thumb(vis_r, DISPLAY_MAX_H)])
        color    = (0, 200, 0) if both else (0, 0, 220)
        label    = f"pair_{i:03d}  {'VALID' if both else 'INVALID'}  ({idx+1}/{len(results)})"
        cv2.putText(row, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        cv2.imshow("Check", row)

        key = cv2.waitKey(0) & 0xFF
        if key == ord("q"):
            break
        elif key == 83:   # right arrow
            idx = (idx + 1) % len(results)
        elif key == 81:   # left arrow
            idx = (idx - 1) % len(results)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
