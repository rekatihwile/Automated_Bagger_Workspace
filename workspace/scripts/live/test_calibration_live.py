"""
Live calibration test:

- Shows left/right camera views in one window.
- Click a point in LEFT, then click the corresponding point in RIGHT.
- Tracks both points with LK optical flow.
- Triangulates the 3D position using the saved stereo calibration and renders a
  live 3D plot on the right.

Controls:
  Left click: pick/replace point (left or right pane)
  R: reset point + path
  Q or Esc: quit
"""

from __future__ import annotations

import time
from collections import deque
from datetime import datetime
from pathlib import Path
import sys

import cv2
import numpy as np

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import config
from camera import StereoCamera


WINDOW_NAME = "Stereo + Live 3D"
DIVIDER_W = 8
POINT_COLOR = (0, 0, 255)
POINT_RADIUS = 2
RECORDINGS_DIR = config.CALIBRATION_ROOT / "test_recordings"
RECORD_VIDEO_FPS = 15.0
RECORD_VIDEO_SCALE = 1.0


class CompositeFrameRecorder:
    """
    Queues combined UI frames, then encodes on release.
    """

    def __init__(self, filepath: Path, fps: float = RECORD_VIDEO_FPS, scale: float = RECORD_VIDEO_SCALE) -> None:
        self.filepath = str(filepath)
        self.fps = fps
        self.scale = scale
        self._frame_buffer: list[np.ndarray] = []
        print(f"[VIDEO] Recording live test to {self.filepath}")

    def write(self, frame: np.ndarray) -> None:
        queued = frame.copy()
        if self.scale != 1.0:
            queued = cv2.resize(queued, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        h, w = queued.shape[:2]
        queued = queued[: h - (h % 2), : w - (w % 2)]
        self._frame_buffer.append(queued)

    def release(self) -> None:
        print("[VIDEO] Encoding queued live-test frames...")

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
    """Return undistorted pixel coordinate (x,y) in the same pixel space."""
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

    # Move origin to the midpoint between cameras (in the left camera frame).
    t = np.asarray(t_lr_mm, dtype=np.float64).reshape(3)
    return x - 0.5 * t


def _make_plotter(height_px: int, width_px: int):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(width_px / 100, height_px / 100), dpi=100)
    ax = fig.add_subplot(111, projection="3d")
    fig.tight_layout(pad=0.3)

    def render(points_xyz: list[np.ndarray], current: np.ndarray | None) -> np.ndarray:
        ax.cla()
        ax.set_title("Triangulated 3D (mm)")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        if points_xyz:
            pts = np.vstack(points_xyz)
            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color="red", linewidth=1.0)
            ax.scatter(pts[-1:, 0], pts[-1:, 1], pts[-1:, 2], color="red", s=10)

            # Dynamic bounds around the last point.
            p = pts[-1]
            span = 250.0
            ax.set_xlim(p[0] - span, p[0] + span)
            ax.set_ylim(p[1] - span, p[1] + span)
            ax.set_zlim(p[2] - span, p[2] + span)
        else:
            ax.set_xlim(-250, 250)
            ax.set_ylim(-250, 250)
            ax.set_zlim(0, 800)

        if current is not None:
            ax.text2D(
                0.02,
                0.02,
                f"X={current[0]:.1f}  Y={current[1]:.1f}  Z={current[2]:.1f}",
                transform=ax.transAxes,
            )

        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        # Matplotlib canvas APIs vary a bit by version; prefer buffer_rgba().
        if hasattr(fig.canvas, "buffer_rgba"):
            rgba = np.asarray(fig.canvas.buffer_rgba(), dtype=np.uint8).reshape(h, w, 4)
            return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)

        # Fallback for older/newer edge cases.
        if hasattr(fig.canvas, "tostring_argb"):
            argb = np.frombuffer(fig.canvas.tostring_argb(), dtype=np.uint8).reshape(h, w, 4)
            rgba = argb[:, :, [1, 2, 3, 0]]
            return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)

        raise RuntimeError("Unsupported Matplotlib canvas: cannot extract pixels.")

    return render


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

    # Tracking state (raw pixel coords in each eye).
    state = {
        "pt_l": None,  # np.ndarray([x,y])
        "pt_r": None,
        "prev_l": None,
        "prev_r": None,
        "path": deque(maxlen=400),
        "last_xyz": None,
        "disp_left_w": 0,
        "disp_right_w": 0,
        "disp_h": 0,
    }

    lk_params = dict(
        winSize=(21, 21),
        maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    def on_mouse(event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        left_w = state["disp_left_w"]
        right_w = state["disp_right_w"]
        h = state["disp_h"]
        if left_w <= 0 or right_w <= 0 or h <= 0:
            return

        # Ignore clicks outside the left/right panes.
        if y < 0 or y >= h:
            return

        if x < left_w:
            eye = "l"
            x_local = x
        elif left_w + DIVIDER_W <= x < left_w + DIVIDER_W + right_w:
            eye = "r"
            x_local = x - (left_w + DIVIDER_W)
        else:
            return

        # Map display pixel -> raw pixel.
        scale = float(config.DISPLAY_SCALE)
        raw_x = float(x_local) / scale
        raw_y = float(y) / scale
        pt = np.array([raw_x, raw_y], dtype=np.float32)

        state[f"pt_{eye}"] = pt
        state[f"prev_{eye}"] = None
        state["path"].clear()
        state["last_xyz"] = None

        print(f"Picked {'LEFT' if eye == 'l' else 'RIGHT'}: ({pt[0]:.1f}, {pt[1]:.1f})")

    cv2.setMouseCallback(WINDOW_NAME, on_mouse)

    # Plot renderer (created lazily once we know display height).
    plot_render = None
    last_plot_time = 0.0
    plot_img = None
    RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    recorder = CompositeFrameRecorder(RECORDINGS_DIR / f"live_calibration_test_{timestamp}.mp4")

    try:
        with StereoCamera() as stereo:
            stereo.warmup()

            while True:
                ok, left, right = stereo.read_pair()
                if not ok or left is None or right is None:
                    continue

                gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
                gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

                if state["pt_l"] is not None and state["prev_l"] is not None:
                    p0 = state["pt_l"].reshape(1, 1, 2).astype(np.float32)
                    p1, st, _err = cv2.calcOpticalFlowPyrLK(state["prev_l"], gray_l, p0, None, **lk_params)
                    if st is not None and st[0, 0] == 1:
                        state["pt_l"] = p1.reshape(2)
                    else:
                        state["pt_l"] = None
                        state["path"].clear()
                        state["last_xyz"] = None
                        print("Left tracking lost. Click to reselect.")

                if state["pt_r"] is not None and state["prev_r"] is not None:
                    p0 = state["pt_r"].reshape(1, 1, 2).astype(np.float32)
                    p1, st, _err = cv2.calcOpticalFlowPyrLK(state["prev_r"], gray_r, p0, None, **lk_params)
                    if st is not None and st[0, 0] == 1:
                        state["pt_r"] = p1.reshape(2)
                    else:
                        state["pt_r"] = None
                        state["path"].clear()
                        state["last_xyz"] = None
                        print("Right tracking lost. Click to reselect.")

                if state["pt_l"] is not None and state["pt_r"] is not None:
                    xyz = _triangulate_centered_mm(
                        state["pt_l"],
                        state["pt_r"],
                        k_l,
                        d_l,
                        k_r,
                        d_r,
                        p_l,
                        p_r,
                        t_lr_mm,
                    )
                    state["last_xyz"] = xyz
                    state["path"].append(xyz)

                state["prev_l"] = gray_l
                state["prev_r"] = gray_r

                # Render left/right preview using the existing helper (includes labels/overlay).
                lines = [
                    "Click LEFT and/or RIGHT to track",
                    "R=reset  Q/Esc=quit",
                ]
                if state["last_xyz"] is not None:
                    x, yv, z = state["last_xyz"]
                    lines.append(f"X={x:.1f}mm  Y={yv:.1f}mm  Z={z:.1f}mm")

                preview = stereo.render_preview(left, right, lines=lines)
                h, w = preview.shape[:2]
                # Recover display pane sizes from the rendered preview.
                # preview = left + divider(6) + right
                left_w = (w - 6) // 2
                right_w = w - 6 - left_w
                state["disp_left_w"] = left_w
                state["disp_right_w"] = right_w
                state["disp_h"] = h

                # Draw tracked points onto the preview.
                if state["pt_l"] is not None:
                    xd = int(state["pt_l"][0] * config.DISPLAY_SCALE)
                    yd = int(state["pt_l"][1] * config.DISPLAY_SCALE)
                    cv2.circle(preview, (xd, yd), POINT_RADIUS, POINT_COLOR, -1)
                if state["pt_r"] is not None:
                    xd = left_w + 6 + int(state["pt_r"][0] * config.DISPLAY_SCALE)
                    yd = int(state["pt_r"][1] * config.DISPLAY_SCALE)
                    cv2.circle(preview, (xd, yd), POINT_RADIUS, POINT_COLOR, -1)

                # Create/update plot renderer and plot image.
                if plot_render is None:
                    plot_w = max(360, int(h * 0.9))
                    plot_render = _make_plotter(height_px=h, width_px=plot_w)
                    plot_img = plot_render(list(state["path"]), state["last_xyz"])

                now = time.time()
                if plot_render is not None and (now - last_plot_time) >= 0.10:
                    plot_img = plot_render(list(state["path"]), state["last_xyz"])
                    last_plot_time = now

                if plot_img is None:
                    plot_img = np.zeros((h, max(360, int(h * 0.9)), 3), dtype=np.uint8)

                # Make plot match the preview height.
                if plot_img.shape[0] != h:
                    plot_img = cv2.resize(plot_img, (plot_img.shape[1], h), interpolation=cv2.INTER_AREA)

                divider = np.full((h, DIVIDER_W, 3), 32, dtype=np.uint8)
                combined = cv2.hconcat([preview, divider, plot_img])
                recorder.write(combined)

                cv2.imshow(WINDOW_NAME, combined)
                key = cv2.waitKey(1) & 0xFF

                if key in (ord("q"), 27):
                    break
                if key == ord("r"):
                    state["pt_l"] = None
                    state["pt_r"] = None
                    state["prev_l"] = None
                    state["prev_r"] = None
                    state["path"].clear()
                    state["last_xyz"] = None
    finally:
        recorder.release()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
