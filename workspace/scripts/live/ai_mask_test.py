from __future__ import annotations

from pathlib import Path
import sys
import cv2
import numpy as np

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

from ai_topface import TopFaceSegmenter
from camera import StereoCamera

WINDOW_NAME = "AI Top-Face Mask Test (Hybrid)"

# --- tuning ---
YOLO_REFRESH_INTERVAL = 30   # run AI every N frames
LK_MAX_CORNERS = 40
LK_QUALITY = 0.01
LK_MIN_DIST = 5


def sample_points_from_mask(seg, max_pts=40):
    if seg is None:
        return None

    cnt = seg.contour
    if cnt is None or len(cnt) < 10:
        return None

    # uniform sampling
    idx = np.linspace(0, len(cnt) - 1, max_pts).astype(int)
    pts = cnt[idx]

    return pts.astype(np.float32)


def main() -> None:
    conf = 0.35
    segmenter = TopFaceSegmenter()

    prev_gray_l = None
    prev_gray_r = None

    pts_l = None
    pts_r = None

    frame_count = 0

    lk_params = dict(
        winSize=(15, 15),
        maxLevel=2,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
    )

    with StereoCamera() as stereo:
        stereo.warmup()
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

        while True:
            ok, left, right = stereo.read_pair()
            if not ok:
                continue

            gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

            use_yolo = (
                frame_count % YOLO_REFRESH_INTERVAL == 0
                or pts_l is None
                or pts_r is None
            )

            if use_yolo:
                seg_l = segmenter.detect(left, conf=conf)
                seg_r = segmenter.detect(right, conf=conf)

                pts_l = sample_points_from_mask(seg_l, LK_MAX_CORNERS)
                pts_r = sample_points_from_mask(seg_r, LK_MAX_CORNERS)

                prev_gray_l = gray_l.copy()
                prev_gray_r = gray_r.copy()

                status = "YOLO refresh"

            else:
                # --- optical flow ---
                pts_l_next, st_l, _ = cv2.calcOpticalFlowPyrLK(
                    prev_gray_l, gray_l, pts_l, None, **lk_params
                )
                pts_r_next, st_r, _ = cv2.calcOpticalFlowPyrLK(
                    prev_gray_r, gray_r, pts_r, None, **lk_params
                )

                if pts_l_next is not None and pts_r_next is not None:
                    good_l = st_l.squeeze() == 1
                    good_r = st_r.squeeze() == 1

                    mask = good_l & good_r

                    pts_l = pts_l_next[mask]
                    pts_r = pts_r_next[mask]

                    if len(pts_l) < 8:
                        pts_l = None
                        pts_r = None
                        status = "lost → refresh next"
                    else:
                        status = "tracking"

                    prev_gray_l = gray_l.copy()
                    prev_gray_r = gray_r.copy()
                else:
                    pts_l = None
                    pts_r = None
                    status = "flow failed"

            # --- visualization ---
            left_view = left.copy()
            right_view = right.copy()

            if pts_l is not None:
                for p in pts_l:
                    x, y = p.astype(int)
                    cv2.circle(left_view, (x, y), 3, (255, 0, 255), -1)

            if pts_r is not None:
                for p in pts_r:
                    x, y = p.astype(int)
                    cv2.circle(right_view, (x, y), 3, (255, 0, 255), -1)

            preview = stereo.render_preview(
                left_view,
                right_view,
                lines=[
                    "AI + Flow hybrid",
                    f"threshold: {conf:.2f}",
                    f"mode: {status}",
                    "[ / ] threshold | Q quit",
                ],
                left_ok=pts_l is not None,
                right_ok=pts_r is not None,
            )

            cv2.imshow(WINDOW_NAME, preview)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord("]"):
                conf = min(0.95, conf + 0.05)
            elif key == ord("["):
                conf = max(0.05, conf - 0.05)

            frame_count += 1

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
