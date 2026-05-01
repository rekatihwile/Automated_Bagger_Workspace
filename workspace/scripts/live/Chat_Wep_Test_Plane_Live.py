from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import sys
from typing import Optional

import cv2
import numpy as np

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import config
from ai_topface import SegmentationResult, TopFaceSegmenter
from camera import StereoCamera
from config import CALIBRATION_ROOT

WINDOW_NAME = "Stereo Plane Test (AI)"
DIVIDER_W = 8
INITIAL_CONFIDENCE = 0.35
FLOWLESS_STATUS = "Segmentation every frame"

PLOT_XY_LIMIT_MM = 500
PLOT_Z_MIN_MM = -100
PLOT_Z_MAX_MM = 1000


@dataclass
class BoundaryState:
    left_points: np.ndarray
    right_points: np.ndarray
    cyclic_shift: int


class CompositeFrameRecorder:
    def __init__(self, filepath: Path, fps: float, scale: float) -> None:
        self.filepath = str(filepath)
        self.fps = float(fps)
        self.scale = float(scale)
        self._frame_buffer: list[np.ndarray] = []
        print(f"[VIDEO] Recording plane test to {self.filepath}")

    def write(self, frame: np.ndarray) -> None:
        queued = frame.copy()
        if self.scale != 1.0:
            queued = cv2.resize(queued, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        h, w = queued.shape[:2]
        queued = queued[: h - (h % 2), : w - (w % 2)]
        self._frame_buffer.append(queued)

    def release(self) -> None:
        print("[VIDEO] Encoding queued plane-test frames...")
        if not self._frame_buffer:
            print("[VIDEO] No frames queued; nothing saved.")
            return

        h, w = self._frame_buffer[0].shape[:2]
        writer = cv2.VideoWriter(self.filepath, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, h))
        if not writer.isOpened():
            print(f"[VIDEO] Could not open VideoWriter for {self.filepath}")
            return

        for frame in self._frame_buffer:
            writer.write(frame)
        writer.release()
        print(f"[VIDEO] Saved {len(self._frame_buffer)} frame(s) to {self.filepath}")


def _undistort_pixel(pt_xy: np.ndarray, k: np.ndarray, d: np.ndarray) -> np.ndarray:
    pt = np.asarray(pt_xy, dtype=np.float32).reshape(1, 1, 2)
    und = cv2.undistortPoints(pt, k, d, P=k)
    return und.reshape(2).astype(np.float64)


def _triangulate_centered_mm(
    pt_l: np.ndarray,
    pt_r: np.ndarray,
    k_l: np.ndarray,
    d_l: np.ndarray,
    k_r: np.ndarray,
    d_r: np.ndarray,
    p_l: np.ndarray,
    p_r: np.ndarray,
    t_lr_mm: np.ndarray,
) -> np.ndarray:
    ul = _undistort_pixel(pt_l, k_l, d_l)
    ur = _undistort_pixel(pt_r, k_r, d_r)

    x_h = cv2.triangulatePoints(
        np.asarray(p_l, dtype=np.float64),
        np.asarray(p_r, dtype=np.float64),
        ul.reshape(2, 1),
        ur.reshape(2, 1),
    )
    x = (x_h[:3] / x_h[3]).reshape(3)
    return x - 0.5 * np.asarray(t_lr_mm, dtype=np.float64).reshape(3)


def _resample_to_reference(points_xy: np.ndarray, count: int) -> np.ndarray:
    pts = np.asarray(points_xy, dtype=np.float32).reshape(-1, 2)
    if len(pts) == count:
        return pts.copy()

    closed = np.vstack([pts, pts[0]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    total = float(np.sum(seg))
    if total <= 1e-6:
        return np.repeat(pts[:1], count, axis=0)

    targets = np.linspace(0.0, total, count, endpoint=False)
    out = []
    acc = 0.0
    j = 0
    for t in targets:
        while j < len(seg) - 1 and acc + seg[j] < t:
            acc += seg[j]
            j += 1
        if seg[j] <= 1e-6:
            out.append(closed[j].copy())
        else:
            a = (t - acc) / seg[j]
            out.append((1.0 - a) * closed[j] + a * closed[j + 1])
    return np.asarray(out, dtype=np.float32)


def _best_cyclic_shift_epipolar(left_pts: np.ndarray, right_pts: np.ndarray) -> int:
    n = len(left_pts)
    best_shift = 0
    best_score = float("inf")

    for s in range(n):
        r = np.roll(right_pts, shift=s, axis=0)
        y_term = np.mean(np.abs(r[:, 1] - left_pts[:, 1]))
        disp = r[:, 0] - left_pts[:, 0]
        disp_std = np.std(disp)
        score = float(y_term + 0.5 * disp_std)
        if score < best_score:
            best_score = score
            best_shift = s
    return best_shift


def _best_cyclic_shift_to_previous(current_pts: np.ndarray, prev_pts: np.ndarray) -> int:
    n = len(current_pts)
    best_shift = 0
    best_score = float("inf")
    for s in range(n):
        c = np.roll(current_pts, shift=s, axis=0)
        score = float(np.mean(np.linalg.norm(c - prev_pts, axis=1)))
        if score < best_score:
            best_score = score
            best_shift = s
    return best_shift


def _stabilize_boundary_pair(
    res_l: SegmentationResult,
    res_r: SegmentationResult,
    prev: Optional[BoundaryState],
) -> BoundaryState:
    left_pts = np.asarray(res_l.sampled, dtype=np.float32)
    right_pts = np.asarray(res_r.sampled, dtype=np.float32)

    if prev is not None:
        left_shift = _best_cyclic_shift_to_previous(left_pts, prev.left_points)
        left_pts = np.roll(left_pts, shift=left_shift, axis=0)

        right_shift = _best_cyclic_shift_to_previous(right_pts, prev.right_points)
        right_pts = np.roll(right_pts, shift=right_shift, axis=0)

        # Keep the previous stereo pairing convention if it still makes sense
        stereo_shift = _best_cyclic_shift_epipolar(left_pts, right_pts)
        right_pts = np.roll(right_pts, shift=stereo_shift, axis=0)
        return BoundaryState(left_points=left_pts, right_points=right_pts, cyclic_shift=stereo_shift)

    stereo_shift = _best_cyclic_shift_epipolar(left_pts, right_pts)
    right_pts = np.roll(right_pts, shift=stereo_shift, axis=0)
    return BoundaryState(left_points=left_pts, right_points=right_pts, cyclic_shift=stereo_shift)


def _draw_boundary_points(frame: np.ndarray, pts: Optional[np.ndarray], color: tuple[int, int, int]) -> np.ndarray:
    out = frame.copy()
    if pts is None or len(pts) == 0:
        return out

    poly = np.round(pts).astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(out, [poly], True, color, 2, cv2.LINE_AA)

    for p in pts:
        xy = tuple(np.round(p).astype(int))
        cv2.circle(out, xy, 2, color, -1, cv2.LINE_AA)

    return out


def _make_plotter(height_px: int, width_px: int):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(width_px / 100, height_px / 100), dpi=100)
    ax = fig.add_subplot(111, projection="3d")
    fig.tight_layout(pad=0.3)

    def render(boundary_xyz: np.ndarray | None, status: str) -> np.ndarray:
        ax.cla()
        ax.set_title("Triangulated Boundary (mm)")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        if boundary_xyz is not None and len(boundary_xyz) >= 3:
            pts = np.asarray(boundary_xyz, dtype=np.float64)
            closed = np.vstack([pts, pts[0]])

            ax.plot(closed[:, 0], closed[:, 1], closed[:, 2], color="magenta", linewidth=1.2)
            ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], color="magenta", s=8)

            poly = Poly3DCollection([pts], alpha=0.18, facecolor="orange", edgecolor="magenta", linewidth=1.0)
            ax.add_collection3d(poly)

            center = pts.mean(axis=0)
            span = max(200.0, float(np.max(np.ptp(pts, axis=0))) * 1.8)
            ax.set_xlim(center[0] - span, center[0] + span)
            ax.set_ylim(center[1] - span, center[1] + span)
            ax.set_zlim(center[2] - span, center[2] + span)
        else:
            ax.set_xlim(-PLOT_XY_LIMIT_MM, PLOT_XY_LIMIT_MM)
            ax.set_ylim(-PLOT_XY_LIMIT_MM, PLOT_XY_LIMIT_MM)
            ax.set_zlim(PLOT_Z_MIN_MM, PLOT_Z_MAX_MM)
            ax.text2D(0.02, 0.02, "Waiting for paired masks", transform=ax.transAxes)

        if status:
            ax.text2D(0.02, 0.08, status, transform=ax.transAxes)

        try:
            ax.set_box_aspect((1, 1, 1))
        except AttributeError:
            pass
        ax.view_init(elev=24, azim=-58)

        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        rgba = np.asarray(fig.canvas.buffer_rgba(), dtype=np.uint8).reshape(h, w, 4)
        return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)

    return render


def _should_record() -> bool:
    return bool(getattr(config, "RECORD_TEST_VIDEO", False))


def _record_dir() -> Path:
    return Path(getattr(config, "TEST_RECORDINGS_DIR", config.CALIBRATION_ROOT / "test_recordings"))


def _record_fps() -> float:
    return float(getattr(config, "TEST_RECORD_VIDEO_FPS", 15.0))


def _record_scale() -> float:
    return float(getattr(config, "TEST_RECORD_VIDEO_SCALE", 1.0))


def main() -> None:
    calib = config.load_stereo_calibration()
    if calib is None:
        print(f"No calibration found at {config.ACTIVE_CALIBRATION_NPZ}")
        print("Run python calibrate.py first.")
        return

    k_l = np.asarray(calib["left_camera_matrix"], dtype=np.float64)
    d_l = np.asarray(calib["left_distortion_coefficients"], dtype=np.float64)
    k_r = np.asarray(calib["right_camera_matrix"], dtype=np.float64)
    d_r = np.asarray(calib["right_distortion_coefficients"], dtype=np.float64)
    p_l = np.asarray(calib["projection_left_raw"], dtype=np.float64)
    p_r = np.asarray(calib["projection_right_raw"], dtype=np.float64)
    t_lr_mm = np.asarray(calib["stereo_translation_left_to_right_mm"], dtype=np.float64)

    sample_count = int(getattr(config, "AI_BOUNDARY_SAMPLE_COUNT", 32))
    input_size = int(getattr(config, "AI_INPUT_SIZE", 640))

    segmenter = TopFaceSegmenter(sample_count=sample_count, input_size=input_size)
    conf = INITIAL_CONFIDENCE
    status = FLOWLESS_STATUS
    state: Optional[BoundaryState] = None
    recorder: Optional[CompositeFrameRecorder] = None
    plot_render = None

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    with StereoCamera() as stereo:
        stereo.warmup()

        while True:
            ok, left, right = stereo.read_pair()
            if not ok or left is None or right is None:
                continue

            res_l = segmenter.detect(left, conf=conf)
            res_r = segmenter.detect(right, conf=conf)

            boundary_xyz = None
            sampled_l = None
            sampled_r = None

            if res_l is not None and res_r is not None:
                try:
                    state = _stabilize_boundary_pair(res_l, res_r, state)
                    sampled_l = state.left_points
                    sampled_r = state.right_points

                    pts_3d = []
                    for pl, pr in zip(sampled_l, sampled_r):
                        xyz = _triangulate_centered_mm(pl, pr, k_l, d_l, k_r, d_r, p_l, p_r, t_lr_mm)
                        pts_3d.append(xyz)
                    boundary_xyz = np.asarray(pts_3d, dtype=np.float64)

                    mean_y_err = float(np.mean(np.abs(sampled_r[:, 1] - sampled_l[:, 1])))
                    status = f"Seg OK | pts={len(sampled_l)} | mean |dy|={mean_y_err:.2f}px"
                except Exception as e:
                    state = None
                    status = f"Pairing failed: {e}"
            else:
                state = None
                status = "Missing left/right mask"

            left_view = segmenter.overlay(left, res_l, show_text=False)
            right_view = segmenter.overlay(right, res_r, show_text=False)

            left_view = _draw_boundary_points(left_view, sampled_l, (255, 0, 255))
            right_view = _draw_boundary_points(right_view, sampled_r, (255, 0, 255))

            if plot_render is None:
                plot_render = _make_plotter(left.shape[0], left.shape[1])
            plot_img = plot_render(boundary_xyz, status)

            preview_lr = stereo.render_preview(
                left_view,
                right_view,
                lines=[
                    "AI plane test",
                    f"threshold: {conf:.2f}",
                    status,
                    "[ / ] threshold | Q quit",
                ],
                left_ok=res_l is not None,
                right_ok=res_r is not None,
            )

            divider = np.full((preview_lr.shape[0], DIVIDER_W, 3), 24, dtype=np.uint8)
            plot_resized = cv2.resize(plot_img, (left.shape[1], left.shape[0]), interpolation=cv2.INTER_AREA)
            combined = cv2.hconcat([preview_lr, divider, plot_resized])

            if _should_record():
                if recorder is None:
                    _record_dir().mkdir(parents=True, exist_ok=True)
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    recorder = CompositeFrameRecorder(
                        _record_dir() / f"live_plane_test_ai_{timestamp}.mp4",
                        fps=_record_fps(),
                        scale=_record_scale(),
                    )
                recorder.write(combined)

            cv2.imshow(WINDOW_NAME, combined)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord("]"):
                conf = min(0.95, conf + 0.05)
                print(f"[AI] confidence threshold = {conf:.2f}")
            elif key == ord("["):
                conf = max(0.05, conf - 0.05)
                print(f"[AI] confidence threshold = {conf:.2f}")

    if recorder is not None:
        recorder.release()
    else:
        print("[VIDEO] Recording disabled or no frames recorded.")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
