from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import time
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


WINDOW_NAME = "Stereo Plane Test (AI)"
MAX_CORNERS = 4
PLOT_XY_LIMIT_MM = 500
PLOT_Z_MIN_MM = -100
PLOT_Z_MAX_MM = 1000
AI_REFRESH_EVERY_N_FRAMES = 5
FLOW_MAX_REPROJ_PX = 18.0
FLOW_FB_MAX_ERR_PX = 2.5
INITIAL_CONFIDENCE = 0.35
GEOMETRY_MAX_ANGLE_ERROR_DEG = 18.0
GEOMETRY_MAX_OPPOSITE_EDGE_ERROR_RATIO = 0.32
GEOMETRY_MAX_DIAGONAL_ERROR_RATIO = 0.22
GEOMETRY_MAX_DIMENSION_DRIFT_RATIO = 0.35
GEOMETRY_MAX_PLANE_ERROR_MM = 25.0
GEOMETRY_MAX_BAD_FRAMES = 3
CAMERA_DEBUG_PRINT_PERIOD_S = 0.10
CAMERA_DEBUG_JUMP_WARN_MM = 60.0
PREVIEW_DIVIDER_W = 6
PLOT_DIVIDER_W = 8
MASK_ALPHA = 0.25
POINT_COLOR = (255, 0, 255)
MASK_COLOR = (0, 180, 255)
CONTOUR_COLOR = (0, 255, 255)
RECORDINGS_DIR = config.CALIBRATION_ROOT / "test_recordings"


class CompositeFrameRecorder:
    """Queues combined UI frames, then encodes on release."""

    def __init__(
        self,
        filepath: Path,
        fps: float = config.AI_TOPFACE_RECORD_FPS,
        scale: float = config.AI_TOPFACE_RECORD_SCALE,
    ) -> None:
        self.filepath = str(filepath)
        self.fps = float(fps)
        self.scale = float(scale)
        self._frame_buffer: list[np.ndarray] = []
        self._timestamps: list[float] = []
        print(f"[VIDEO] Recording AI plane test to {self.filepath}")

    def write(self, frame: np.ndarray) -> None:
        queued = frame.copy()
        if self.scale != 1.0:
            queued = cv2.resize(queued, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        h, w = queued.shape[:2]
        queued = queued[: h - (h % 2), : w - (w % 2)]
        self._frame_buffer.append(queued)
        self._timestamps.append(time.perf_counter())

    def release(self) -> None:
        if not self._frame_buffer:
            print("[VIDEO] No AI plane-test frames queued; nothing saved.")
            return

        elapsed_s = 0.0
        if len(self._timestamps) >= 2:
            elapsed_s = self._timestamps[-1] - self._timestamps[0]

        target_fps = max(1.0, self.fps)
        rel_times = np.asarray(self._timestamps, dtype=np.float64)
        if len(rel_times) >= 1:
            rel_times = rel_times - rel_times[0]

        output_indices: list[int]
        captured_fps = 0.0
        if len(rel_times) >= 2 and elapsed_s > 1e-6:
            captured_fps = (len(rel_times) - 1) / elapsed_s
            output_count = max(len(self._frame_buffer), int(round(elapsed_s * target_fps)) + 1)
            output_times = np.arange(output_count, dtype=np.float64) / target_fps
            output_times = np.clip(output_times, 0.0, rel_times[-1])
            output_indices = []
            for t_out in output_times:
                idx = int(np.searchsorted(rel_times, t_out, side="left"))
                if idx <= 0:
                    output_indices.append(0)
                elif idx >= len(rel_times):
                    output_indices.append(len(rel_times) - 1)
                else:
                    prev_idx = idx - 1
                    choose_prev = abs(t_out - rel_times[prev_idx]) <= abs(rel_times[idx] - t_out)
                    output_indices.append(prev_idx if choose_prev else idx)
        else:
            output_indices = list(range(len(self._frame_buffer)))

        print(f"[VIDEO] Encoding queued AI plane-test frames to {target_fps:.2f} fps...")
        h, w = self._frame_buffer[0].shape[:2]
        writer = cv2.VideoWriter(self.filepath, cv2.VideoWriter_fourcc(*"mp4v"), target_fps, (w, h))
        if not writer.isOpened():
            print(f"[VIDEO] Could not open VideoWriter for {self.filepath}")
            return

        for idx in output_indices:
            writer.write(self._frame_buffer[idx])
        writer.release()
        if elapsed_s > 0.0:
            print(
                f"[VIDEO] Saved {len(output_indices)} frame(s) over {elapsed_s:.2f}s"
                f" to {self.filepath} (captured ~{captured_fps:.2f} fps)"
            )
        else:
            print(f"[VIDEO] Saved {len(output_indices)} frame(s) to {self.filepath}")


@dataclass
class PlaneFrame:
    origin: np.ndarray
    x_axis: np.ndarray
    y_axis: np.ndarray
    z_axis: np.ndarray
    box_length_mm: float
    box_width_mm: float

    def transform_points(self, points: np.ndarray) -> np.ndarray:
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        basis = np.column_stack([self.x_axis, self.y_axis, self.z_axis])
        return (pts - self.origin) @ basis

    def rotate_points(self, points: np.ndarray) -> np.ndarray:
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        basis = np.column_stack([self.x_axis, self.y_axis, self.z_axis])
        return pts @ basis


@dataclass
class GeometryMetrics:
    max_angle_error_deg: float
    opposite_edge_error_ratio: float
    diagonal_error_ratio: float
    dimension_drift_ratio: float
    plane_error_mm: float
    long_edge_mm: float
    short_edge_mm: float
    diagonal_mm: float
    score: float
    is_valid: bool


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


def _topology_preserving_permutations(n: int) -> list[tuple[int, ...]]:
    base = tuple(range(n))
    reverse = tuple([0, *range(n - 1, 0, -1)])
    perms: list[tuple[int, ...]] = []
    seen: set[tuple[int, ...]] = set()
    for seq in (base, reverse):
        for shift in range(n):
            perm = tuple(seq[(i + shift) % n] for i in range(n))
            if perm not in seen:
                seen.add(perm)
                perms.append(perm)
    return perms


def _quad_from_segmentation(seg: Optional[SegmentationResult]) -> Optional[np.ndarray]:
    if seg is None:
        return None
    contour = np.asarray(seg.contour, dtype=np.float32).reshape(-1, 1, 2)
    if len(contour) < 4:
        return None
    rect = cv2.minAreaRect(contour)
    quad = cv2.boxPoints(rect).astype(np.float32)
    if quad.shape != (MAX_CORNERS, 2):
        return None
    return quad


def match_corner_constellation(left_points: np.ndarray, right_points: np.ndarray) -> np.ndarray | None:
    if left_points.shape != (MAX_CORNERS, 2) or right_points.shape != (MAX_CORNERS, 2):
        return None

    best_perm = None
    best_score = float("inf")
    for perm in _topology_preserving_permutations(MAX_CORNERS):
        ordered_right = right_points[list(perm)]
        deltas = ordered_right - left_points
        disparities = deltas[:, 0]
        y_offsets = deltas[:, 1]
        median_disp = float(np.median(disparities))
        if abs(median_disp) < 1.0:
            continue
        left_edges = np.linalg.norm(np.roll(left_points, -1, axis=0) - left_points, axis=1)
        right_edges = np.linalg.norm(np.roll(ordered_right, -1, axis=0) - ordered_right, axis=1)
        left_edges = left_edges / max(float(np.mean(left_edges)), 1e-6)
        right_edges = right_edges / max(float(np.mean(right_edges)), 1e-6)
        shape_error = float(np.mean(np.abs(left_edges - right_edges)))
        score = float(np.std(disparities) + 2.0 * np.mean(np.abs(y_offsets)) + np.std(y_offsets) + 4.0 * shape_error)
        if score < best_score:
            best_score = score
            best_perm = np.asarray(perm, dtype=np.int32)
    return best_perm


def reorder_points_like_previous(previous_points: Optional[np.ndarray], current_points: np.ndarray) -> np.ndarray:
    if previous_points is None or previous_points.shape != current_points.shape:
        return current_points.astype(np.float32)

    best_perm = None
    best_cost = float("inf")
    for perm in _topology_preserving_permutations(len(current_points)):
        candidate = current_points[list(perm)]
        cost = float(np.sum(np.linalg.norm(candidate - previous_points, axis=1)))
        if cost < best_cost:
            best_cost = cost
            best_perm = perm
    if best_perm is None:
        return current_points.astype(np.float32)
    return current_points[list(best_perm)].astype(np.float32)


def order_plane_corners(points_3d: np.ndarray) -> np.ndarray:
    centroid = points_3d.mean(axis=0)
    centered = points_3d - centroid
    _, _, vh = np.linalg.svd(centered)
    u = vh[0]
    v = vh[1]
    coords = np.column_stack([centered @ u, centered @ v])
    angles = np.arctan2(coords[:, 1], coords[:, 0])
    return np.argsort(angles)


def corner_angles_deg(points_3d: np.ndarray) -> list[float]:
    angles: list[float] = []
    for i in range(len(points_3d)):
        prev_pt = points_3d[(i - 1) % len(points_3d)]
        pt = points_3d[i]
        next_pt = points_3d[(i + 1) % len(points_3d)]
        a = prev_pt - pt
        b = next_pt - pt
        denom = np.linalg.norm(a) * np.linalg.norm(b)
        if denom <= 1e-9:
            angles.append(float("nan"))
            continue
        cosang = float(np.clip(np.dot(a, b) / denom, -1.0, 1.0))
        angles.append(float(np.degrees(np.arccos(cosang))))
    return angles


def _rectangle_dimensions(points_3d: np.ndarray) -> tuple[float, float, float]:
    pts = np.asarray(points_3d, dtype=np.float64).reshape(-1, 3)
    edges = np.linalg.norm(np.roll(pts, -1, axis=0) - pts, axis=1)
    pair_a = 0.5 * (edges[0] + edges[2])
    pair_b = 0.5 * (edges[1] + edges[3])
    long_edge_mm = float(max(pair_a, pair_b))
    short_edge_mm = float(min(pair_a, pair_b))
    diag_0 = float(np.linalg.norm(pts[2] - pts[0]))
    diag_1 = float(np.linalg.norm(pts[3] - pts[1]))
    diagonal_mm = 0.5 * (diag_0 + diag_1)
    return long_edge_mm, short_edge_mm, float(diagonal_mm)


def _planarity_error_mm(points_3d: np.ndarray) -> float:
    pts = np.asarray(points_3d, dtype=np.float64).reshape(-1, 3)
    centroid = pts.mean(axis=0)
    centered = pts - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    distances = np.abs(centered @ normal)
    return float(np.max(distances))


def measure_rectangle_geometry(
    points_3d: np.ndarray,
    reference_dims_mm: tuple[float, float, float] | None = None,
) -> GeometryMetrics:
    pts = np.asarray(points_3d, dtype=np.float64).reshape(-1, 3)
    angles = corner_angles_deg(pts)
    finite_angles = [a for a in angles if np.isfinite(a)]
    max_angle_error_deg = max((abs(a - 90.0) for a in finite_angles), default=float("inf"))

    edges = np.linalg.norm(np.roll(pts, -1, axis=0) - pts, axis=1)
    opposite_edge_error_ratio = max(
        abs(edges[0] - edges[2]) / max(0.5 * (edges[0] + edges[2]), 1e-6),
        abs(edges[1] - edges[3]) / max(0.5 * (edges[1] + edges[3]), 1e-6),
    )

    diag_0 = float(np.linalg.norm(pts[2] - pts[0]))
    diag_1 = float(np.linalg.norm(pts[3] - pts[1]))
    diagonal_mm = 0.5 * (diag_0 + diag_1)
    diagonal_error_ratio = abs(diag_0 - diag_1) / max(diagonal_mm, 1e-6)

    long_edge_mm, short_edge_mm, diagonal_mm = _rectangle_dimensions(pts)
    dimension_drift_ratio = 0.0
    if reference_dims_mm is not None:
        ref_long, ref_short, ref_diag = reference_dims_mm
        dimension_drift_ratio = max(
            abs(long_edge_mm - ref_long) / max(ref_long, 1e-6),
            abs(short_edge_mm - ref_short) / max(ref_short, 1e-6),
            abs(diagonal_mm - ref_diag) / max(ref_diag, 1e-6),
        )

    plane_error_mm = _planarity_error_mm(pts)
    score = (
        max_angle_error_deg
        + 35.0 * opposite_edge_error_ratio
        + 30.0 * diagonal_error_ratio
        + 25.0 * dimension_drift_ratio
        + 0.7 * plane_error_mm
    )
    is_valid = (
        max_angle_error_deg <= GEOMETRY_MAX_ANGLE_ERROR_DEG
        and opposite_edge_error_ratio <= GEOMETRY_MAX_OPPOSITE_EDGE_ERROR_RATIO
        and diagonal_error_ratio <= GEOMETRY_MAX_DIAGONAL_ERROR_RATIO
        and plane_error_mm <= GEOMETRY_MAX_PLANE_ERROR_MM
        and (reference_dims_mm is None or dimension_drift_ratio <= GEOMETRY_MAX_DIMENSION_DRIFT_RATIO)
    )
    return GeometryMetrics(
        max_angle_error_deg=float(max_angle_error_deg),
        opposite_edge_error_ratio=float(opposite_edge_error_ratio),
        diagonal_error_ratio=float(diagonal_error_ratio),
        dimension_drift_ratio=float(dimension_drift_ratio),
        plane_error_mm=float(plane_error_mm),
        long_edge_mm=float(long_edge_mm),
        short_edge_mm=float(short_edge_mm),
        diagonal_mm=float(diagonal_mm),
        score=float(score),
        is_valid=bool(is_valid),
    )


def _normalize(vec: np.ndarray) -> np.ndarray | None:
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-9:
        return None
    return np.asarray(vec, dtype=np.float64) / norm


def make_plane_frame(points_3d: np.ndarray) -> PlaneFrame | None:
    if len(points_3d) != MAX_CORNERS:
        return None

    pts = np.asarray(points_3d, dtype=np.float64)
    origin = pts.mean(axis=0)
    edge_pair_a = ((pts[1] - pts[0]) + (pts[2] - pts[3])) * 0.5
    edge_pair_b = ((pts[2] - pts[1]) + (pts[3] - pts[0])) * 0.5
    length_a = 0.5 * (np.linalg.norm(pts[1] - pts[0]) + np.linalg.norm(pts[2] - pts[3]))
    length_b = 0.5 * (np.linalg.norm(pts[2] - pts[1]) + np.linalg.norm(pts[3] - pts[0]))

    if length_a >= length_b:
        x_vec, y_vec = edge_pair_a, edge_pair_b
        box_length_mm, box_width_mm = float(length_a), float(length_b)
    else:
        x_vec, y_vec = edge_pair_b, edge_pair_a
        box_length_mm, box_width_mm = float(length_b), float(length_a)

    x_axis = _normalize(x_vec)
    rough_y = _normalize(y_vec)
    if x_axis is None or rough_y is None:
        return None

    z_axis = _normalize(np.cross(x_axis, rough_y))
    if z_axis is None:
        return None
    if float(np.dot(-origin, z_axis)) < 0:
        z_axis = -z_axis

    y_axis = _normalize(np.cross(z_axis, x_axis))
    if y_axis is None:
        return None

    return PlaneFrame(origin, x_axis, y_axis, z_axis, box_length_mm, box_width_mm)


def _track_points_with_lk(prev_gray: np.ndarray, gray: np.ndarray, points_xy: np.ndarray) -> tuple[Optional[np.ndarray], str]:
    if points_xy.shape != (MAX_CORNERS, 2):
        return None, "No points"

    lk_params = dict(
        winSize=(21, 21),
        maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )
    p0 = points_xy.astype(np.float32).reshape(-1, 1, 2)
    p1, st, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, p0, None, **lk_params)
    if p1 is None or st is None or np.any(st.reshape(-1) == 0):
        return None, "LK lost points"

    p0_back, st_back, _ = cv2.calcOpticalFlowPyrLK(gray, prev_gray, p1, None, **lk_params)
    if p0_back is None or st_back is None or np.any(st_back.reshape(-1) == 0):
        return None, "LK backward check failed"

    tracked = p1.reshape(-1, 2)
    back = p0_back.reshape(-1, 2)
    fb = np.linalg.norm(back - points_xy, axis=1)
    if float(np.max(fb)) > FLOW_FB_MAX_ERR_PX:
        return None, f"LK FB err {float(np.max(fb)):.1f}px"

    return tracked.astype(np.float32), "LK ok"


def _overlay_segmentation(
    frame_bgr: np.ndarray,
    seg: Optional[SegmentationResult],
    paired_points: Optional[np.ndarray],
) -> np.ndarray:
    out = frame_bgr.copy()
    if seg is None:
        return out

    overlay = out.copy()
    overlay[seg.mask > 0] = MASK_COLOR
    out = cv2.addWeighted(overlay, MASK_ALPHA, out, 1.0 - MASK_ALPHA, 0)

    contour_i = np.round(seg.contour).astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(out, [contour_i], True, CONTOUR_COLOR, 2, cv2.LINE_AA)

    if paired_points is not None:
        poly = np.round(paired_points).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(out, [poly], True, POINT_COLOR, 2, cv2.LINE_AA)
        for pt in paired_points:
            xy = tuple(np.round(pt).astype(int))
            cv2.circle(out, xy, 4, POINT_COLOR, -1, cv2.LINE_AA)

    return out


def _compose_preview(stereo: StereoCamera, left_view: np.ndarray, right_view: np.ndarray) -> np.ndarray:
    left_disp = stereo.camera.resize_for_display(left_view)
    right_disp = stereo.camera.resize_for_display(right_view)
    divider = np.full((left_disp.shape[0], PREVIEW_DIVIDER_W, 3), 24, dtype=np.uint8)
    return cv2.hconcat([left_disp, divider, right_disp])


def _make_plotter(height_px: int, width_px: int, t_lr_mm: np.ndarray):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(width_px / 100, height_px / 100), dpi=100)
    ax = fig.add_subplot(111, projection="3d")
    fig.tight_layout(pad=0.3)

    t_vec = np.asarray(t_lr_mm, dtype=np.float64).reshape(3)
    if np.linalg.norm(t_vec) < 1e-6:
        t_vec = np.array([80.0, 0.0, 0.0], dtype=np.float64)
    left_cam = -0.5 * t_vec
    right_cam = 0.5 * t_vec
    origin = np.zeros(3, dtype=np.float64)
    rig_half_depth = max(18.0, min(55.0, float(np.linalg.norm(t_vec)) * 0.18))
    rig_rect_raw = np.vstack(
        [
            left_cam + np.array([0.0, -rig_half_depth, 0.0]),
            right_cam + np.array([0.0, -rig_half_depth, 0.0]),
            right_cam + np.array([0.0, rig_half_depth, 0.0]),
            left_cam + np.array([0.0, rig_half_depth, 0.0]),
            left_cam + np.array([0.0, -rig_half_depth, 0.0]),
        ]
    )
    view_len = max(80.0, float(np.linalg.norm(t_vec)) * 0.75)
    view_tip = origin + np.array([0.0, 0.0, view_len])

    def transform_raw(points: np.ndarray, plane_frame: PlaneFrame) -> np.ndarray:
        return plane_frame.transform_points(np.asarray(points, dtype=np.float64))

    def render(
        points_3d: np.ndarray | None,
        angles: list[float] | None,
        plane_frame: PlaneFrame | None,
        camera_level_frame: PlaneFrame | None,
        frame_mode: str,
        debug_lines: Optional[list[str]] = None,
        status: str = "",
    ) -> np.ndarray:
        ax.cla()
        mode = frame_mode.strip().lower()
        limit = float(PLOT_XY_LIMIT_MM)

        if mode == "camera":
            ax.set_title("Camera Frame (mm)")
            ax.set_xlabel("Camera X")
            ax.set_ylabel("Camera Y")
            ax.set_zlabel("Below Camera")
        else:
            ax.set_title("Box Frame (mm)")
            ax.set_xlabel("Box X")
            ax.set_ylabel("Box Y")
            ax.set_zlabel("Height")
            plane = np.array(
                [[-limit, -limit, 0.0], [limit, -limit, 0.0], [limit, limit, 0.0], [-limit, limit, 0.0]],
                dtype=np.float64,
            )
            ax.add_collection3d(Poly3DCollection([plane], alpha=0.08, facecolor="0.5", edgecolor="0.75", linewidth=0.8))

        if points_3d is not None and (mode == "camera" or plane_frame is not None):
            if mode == "camera":
                if camera_level_frame is not None:
                    points_plot = camera_level_frame.rotate_points(points_3d)
                    rig_rect = camera_level_frame.rotate_points(rig_rect_raw)
                    left_plot, right_plot, origin_plot, view_tip_plot = camera_level_frame.rotate_points(
                        np.vstack([left_cam, right_cam, origin, view_tip])
                    )
                else:
                    points_plot = np.asarray(points_3d, dtype=np.float64).copy()
                    rig_rect = np.asarray(rig_rect_raw, dtype=np.float64).copy()
                    left_plot, right_plot, origin_plot, view_tip_plot = np.asarray(
                        np.vstack([left_cam, right_cam, origin, view_tip]),
                        dtype=np.float64,
                    )
                    points_plot[:, 2] *= -1.0
                    rig_rect[:, 2] *= -1.0
                    left_plot[2] *= -1.0
                    right_plot[2] *= -1.0
                    origin_plot[2] *= -1.0
                    view_tip_plot[2] *= -1.0
            else:
                points_plot = plane_frame.transform_points(points_3d)
                rig_rect = transform_raw(rig_rect_raw, plane_frame)
                left_plot, right_plot, origin_plot, view_tip_plot = transform_raw(
                    np.vstack([left_cam, right_cam, origin, view_tip]),
                    plane_frame,
                )

            ax.plot(rig_rect[:, 0], rig_rect[:, 1], rig_rect[:, 2], color="black", linewidth=1.8)
            ax.plot([left_plot[0], right_plot[0]], [left_plot[1], right_plot[1]], [left_plot[2], right_plot[2]], color="black", linewidth=2.0)
            ax.scatter([left_plot[0], right_plot[0]], [left_plot[1], right_plot[1]], [left_plot[2], right_plot[2]], color="royalblue", s=22)
            ax.scatter([origin_plot[0]], [origin_plot[1]], [origin_plot[2]], color="black", s=28)
            ax.plot([origin_plot[0], view_tip_plot[0]], [origin_plot[1], view_tip_plot[1]], [origin_plot[2], view_tip_plot[2]], color="black", linewidth=1.3)
            poly = Poly3DCollection([points_plot], alpha=0.35, facecolor="orange", edgecolor="red", linewidth=1.5)
            ax.add_collection3d(poly)
            closed = np.vstack([points_plot, points_plot[0]])
            ax.plot(closed[:, 0], closed[:, 1], closed[:, 2], color="red", linewidth=1.5)
            ax.scatter(points_plot[:, 0], points_plot[:, 1], points_plot[:, 2], color="red", s=12)
            for corner in points_plot:
                ax.plot(
                    [origin_plot[0], corner[0]],
                    [origin_plot[1], corner[1]],
                    [origin_plot[2], corner[2]],
                    color="0.7",
                    linewidth=0.8,
                    alpha=0.8,
                )
            if angles:
                ax.text2D(0.02, 0.02, "Angles: " + "  ".join(f"{a:.1f}" for a in angles), transform=ax.transAxes)
        else:
            wait_text = "Waiting for mask + stereo pairing"
            if mode == "box":
                wait_text += " + box frame"
            ax.text2D(0.02, 0.02, wait_text, transform=ax.transAxes)

        if status:
            ax.text2D(0.02, 0.08, status, transform=ax.transAxes)
        ax.text2D(0.02, 0.14, f"Mode: {'Camera Frame' if mode == 'camera' else 'Box Frame'}", transform=ax.transAxes)
        if mode == "camera" and camera_level_frame is not None:
            ax.text2D(0.02, 0.20, "Plane leveled: ON", transform=ax.transAxes)
        elif debug_lines:
            y0 = 0.20
            for line in debug_lines:
                ax.text2D(0.02, y0, line, transform=ax.transAxes)
                y0 += 0.06

        ax.set_xlim(-PLOT_XY_LIMIT_MM, PLOT_XY_LIMIT_MM)
        ax.set_ylim(-PLOT_XY_LIMIT_MM, PLOT_XY_LIMIT_MM)
        if mode == "camera":
            ax.set_zlim(-PLOT_Z_MAX_MM, -PLOT_Z_MIN_MM)
        else:
            ax.set_zlim(PLOT_Z_MIN_MM, PLOT_Z_MAX_MM)
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


def _evaluate_points(
    points_l: np.ndarray,
    points_r: np.ndarray,
    *,
    k_l: np.ndarray,
    d_l: np.ndarray,
    k_r: np.ndarray,
    d_r: np.ndarray,
    p_l: np.ndarray,
    p_r: np.ndarray,
    t_lr_mm: np.ndarray,
    reference_dims_mm: tuple[float, float, float] | None,
) -> tuple[np.ndarray, list[float], PlaneFrame | None, GeometryMetrics]:
    pts_3d = np.vstack(
        [
            _triangulate_centered_mm(points_l[i], points_r[i], k_l, d_l, k_r, d_r, p_l, p_r, t_lr_mm)
            for i in range(MAX_CORNERS)
        ]
    )
    # Keep the tracked 2D corner order. Re-sorting in 3D can create artificial
    # frame flips even when the underlying 2D tracks are stable.
    ordered_xyz = np.asarray(pts_3d, dtype=np.float64)
    angles = corner_angles_deg(ordered_xyz)
    plane_frame = make_plane_frame(ordered_xyz)
    metrics = measure_rectangle_geometry(ordered_xyz, reference_dims_mm)
    return ordered_xyz, angles, plane_frame, metrics


def _blend_reference_dims(
    current_ref: tuple[float, float, float] | None,
    new_dims: tuple[float, float, float],
    alpha: float = 0.15,
) -> tuple[float, float, float]:
    if current_ref is None:
        return new_dims
    return tuple(float((1.0 - alpha) * old + alpha * new) for old, new in zip(current_ref, new_dims))


def _camera_position_in_box_frame(plane_frame: PlaneFrame) -> np.ndarray:
    return plane_frame.transform_points(np.zeros((1, 3), dtype=np.float64))[0]


def _build_box_camera_debug(
    frame_idx: int,
    camera_pos_box: np.ndarray,
    previous_camera_pos_box: Optional[np.ndarray],
    points_l: Optional[np.ndarray],
    points_r: Optional[np.ndarray],
    geometry_metrics: Optional[GeometryMetrics],
) -> tuple[list[str], str, float]:
    jump_mm = 0.0
    if previous_camera_pos_box is not None:
        jump_mm = float(np.linalg.norm(camera_pos_box - previous_camera_pos_box))

    angle_error = float("nan")
    geometry_state = "?"
    if geometry_metrics is not None:
        angle_error = geometry_metrics.max_angle_error_deg
        geometry_state = "OK" if geometry_metrics.is_valid else "BAD"

    y_error_px = float("nan")
    disp_text = "n/a"
    worst_pair = "-"
    if points_l is not None and points_r is not None and len(points_l) == len(points_r):
        disparities = np.asarray(points_l[:, 0] - points_r[:, 0], dtype=np.float64)
        y_offsets = np.asarray(points_r[:, 1] - points_l[:, 1], dtype=np.float64)
        y_error_px = float(np.mean(np.abs(y_offsets)))
        disp_med = float(np.median(disparities))
        disp_dev = np.abs(disparities - disp_med)
        worst_pair = str(int(np.argmax(disp_dev)) + 1)
        disp_text = "[" + ", ".join(f"{d:.1f}" for d in disparities) + "]"

    overlay_lines = [
        f"Cam XYZ: {camera_pos_box[0]:.1f}, {camera_pos_box[1]:.1f}, {camera_pos_box[2]:.1f} mm",
        f"Jump={jump_mm:.1f}  ang={angle_error:.1f}  y={y_error_px:.1f}px  pair={worst_pair}  {geometry_state}",
    ]
    log_line = (
        f"frame={frame_idx} cam=({camera_pos_box[0]:.1f}, {camera_pos_box[1]:.1f}, {camera_pos_box[2]:.1f})"
        f" jump={jump_mm:.1f}mm ang={angle_error:.1f}deg yerr={y_error_px:.1f}px"
        f" disp={disp_text} worst_pair={worst_pair} geom={geometry_state}"
    )
    return overlay_lines, log_line, jump_mm


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
    plot_frame_mode = str(getattr(config, "AI_TOPFACE_PLOT_FRAME", "box")).strip().lower()
    if plot_frame_mode not in {"box", "camera"}:
        plot_frame_mode = "box"
    frame_idx = 0
    force_ai = True
    prev_gray_l: Optional[np.ndarray] = None
    prev_gray_r: Optional[np.ndarray] = None
    points_l: Optional[np.ndarray] = None
    points_r: Optional[np.ndarray] = None
    ai_l: Optional[SegmentationResult] = None
    ai_r: Optional[SegmentationResult] = None
    status = "Waiting for AI snap"
    recorder: CompositeFrameRecorder | None = None
    reference_dims_mm: tuple[float, float, float] | None = None
    last_good_points_l: Optional[np.ndarray] = None
    last_good_points_r: Optional[np.ndarray] = None
    geometry_bad_frames = 0
    camera_level_frame: Optional[PlaneFrame] = None
    previous_box_camera_pos: Optional[np.ndarray] = None
    last_camera_debug_print_s = 0.0

    print("Controls: A=force AI snap  C=clear pairing  M=toggle plot frame  P=level camera frame  [ / ]=confidence  Q/Esc=quit")
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    plot_render = None

    try:
        with StereoCamera() as stereo:
            stereo.warmup()

            while True:
                ok, left, right = stereo.read_pair()
                if not ok or left is None or right is None:
                    continue

                gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
                gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
                should_ai_snap = force_ai or points_l is None or points_r is None or frame_idx % AI_REFRESH_EVERY_N_FRAMES == 0
                ai_updated_this_frame = False
                if should_ai_snap:
                    ai_l = segmenter.detect(left, conf=conf)
                    ai_r = segmenter.detect(right, conf=conf)
                    ai_updated_this_frame = True
                snapped = False

                if should_ai_snap and ai_l is not None and ai_r is not None:
                    current_l = _quad_from_segmentation(ai_l)
                    current_r = _quad_from_segmentation(ai_r)
                    if current_l is None or current_r is None:
                        points_l = None
                        points_r = None
                        status = "AI masks found, quad extraction failed"
                        force_ai = True
                        snapped = False
                    else:
                        current_l = current_l.astype(np.float32)
                        current_r = current_r.astype(np.float32)

                        if points_l is None or points_r is None:
                            perm = match_corner_constellation(current_l, current_r)
                            if perm is None:
                                points_l = None
                                points_r = None
                                status = "AI masks found, stereo pairing failed"
                            else:
                                points_l = current_l
                                points_r = current_r[perm]
                                force_ai = False
                                snapped = True
                                status = "AI initial pair"
                        else:
                            snapped_l = reorder_points_like_previous(points_l, current_l)
                            snapped_r = reorder_points_like_previous(points_r, current_r)
                            y_mismatch = float(np.mean(np.abs(snapped_r[:, 1] - snapped_l[:, 1])))
                            if y_mismatch > FLOW_MAX_REPROJ_PX:
                                status = f"AI snap rejected: stereo y mismatch {y_mismatch:.1f}px"
                            else:
                                points_l = snapped_l
                                points_r = snapped_r
                                force_ai = False
                                snapped = True
                                status = f"AI snap @ frame {frame_idx}"

                if not snapped and prev_gray_l is not None and prev_gray_r is not None and points_l is not None and points_r is not None:
                    tracked_l, status_l = _track_points_with_lk(prev_gray_l, gray_l, points_l)
                    tracked_r, status_r = _track_points_with_lk(prev_gray_r, gray_r, points_r)
                    if tracked_l is None or tracked_r is None:
                        points_l = None
                        points_r = None
                        force_ai = True
                        status = f"Reacquire: {status_l} | {status_r}"
                    else:
                        y_mismatch = float(np.mean(np.abs(tracked_r[:, 1] - tracked_l[:, 1])))
                        if y_mismatch > FLOW_MAX_REPROJ_PX:
                            points_l = None
                            points_r = None
                            force_ai = True
                            status = f"Reacquire: stereo y mismatch {y_mismatch:.1f}px"
                        else:
                            points_l = tracked_l
                            points_r = tracked_r
                            steps_left = AI_REFRESH_EVERY_N_FRAMES - (frame_idx % AI_REFRESH_EVERY_N_FRAMES)
                            status = f"LK tracking  next AI snap in {steps_left}"
                elif not snapped and (points_l is None or points_r is None):
                    if ai_updated_this_frame and ai_l is None and ai_r is None:
                        status = "Waiting: YOLO missing on both views"
                    elif ai_updated_this_frame and ai_l is None:
                        status = "Waiting: YOLO missing on LEFT"
                    elif ai_updated_this_frame and ai_r is None:
                        status = "Waiting: YOLO missing on RIGHT"

                ordered_xyz = None
                angles = None
                plane_frame = None
                geometry_metrics: GeometryMetrics | None = None
                box_debug_lines: list[str] | None = None
                if points_l is not None and points_r is not None:
                    ordered_xyz, angles, plane_frame, geometry_metrics = _evaluate_points(
                        points_l,
                        points_r,
                        k_l=k_l,
                        d_l=d_l,
                        k_r=k_r,
                        d_r=d_r,
                        p_l=p_l,
                        p_r=p_r,
                        t_lr_mm=t_lr_mm,
                        reference_dims_mm=reference_dims_mm,
                    )

                    if geometry_metrics.is_valid and plane_frame is not None:
                        geometry_bad_frames = 0
                        dims_now = (
                            geometry_metrics.long_edge_mm,
                            geometry_metrics.short_edge_mm,
                            geometry_metrics.diagonal_mm,
                        )
                        if snapped or reference_dims_mm is None:
                            reference_dims_mm = _blend_reference_dims(reference_dims_mm, dims_now)
                        last_good_points_l = points_l.copy()
                        last_good_points_r = points_r.copy()
                        if "LK tracking" in status or "AI snap" in status or "AI initial pair" in status:
                            status = (
                                f"{status}  |  ang={geometry_metrics.max_angle_error_deg:.1f}deg"
                            )
                    else:
                        geometry_bad_frames += 1
                        correction_applied = False
                        if ai_updated_this_frame and ai_l is not None and ai_r is not None:
                            quad_l = _quad_from_segmentation(ai_l)
                            quad_r = _quad_from_segmentation(ai_r)
                            if quad_l is None or quad_r is None:
                                quad_l = None
                                quad_r = None
                            if quad_l is not None and quad_r is not None:
                                candidate_l = reorder_points_like_previous(points_l, quad_l.astype(np.float32))
                                candidate_r = reorder_points_like_previous(points_r, quad_r.astype(np.float32))
                            else:
                                candidate_l = None
                                candidate_r = None
                            if candidate_l is None or candidate_r is None:
                                candidate_y_mismatch = float("inf")
                            else:
                                candidate_y_mismatch = float(np.mean(np.abs(candidate_r[:, 1] - candidate_l[:, 1])))
                            if candidate_y_mismatch <= FLOW_MAX_REPROJ_PX:
                                cand_xyz, cand_angles, cand_plane, cand_metrics = _evaluate_points(
                                    candidate_l,
                                    candidate_r,
                                    k_l=k_l,
                                    d_l=d_l,
                                    k_r=k_r,
                                    d_r=d_r,
                                    p_l=p_l,
                                    p_r=p_r,
                                    t_lr_mm=t_lr_mm,
                                    reference_dims_mm=reference_dims_mm,
                                )
                                current_score = geometry_metrics.score if geometry_metrics is not None else float("inf")
                                if cand_plane is not None and cand_metrics.is_valid and cand_metrics.score < current_score:
                                    points_l = candidate_l
                                    points_r = candidate_r
                                    ordered_xyz = cand_xyz
                                    angles = cand_angles
                                    plane_frame = cand_plane
                                    geometry_metrics = cand_metrics
                                    reference_dims_mm = _blend_reference_dims(
                                        reference_dims_mm,
                                        (cand_metrics.long_edge_mm, cand_metrics.short_edge_mm, cand_metrics.diagonal_mm),
                                    )
                                    last_good_points_l = points_l.copy()
                                    last_good_points_r = points_r.copy()
                                    geometry_bad_frames = 0
                                    force_ai = False
                                    correction_applied = True
                                    status = f"Geometry snap fix  |  ang={cand_metrics.max_angle_error_deg:.1f}deg"

                        if not correction_applied:
                            force_ai = True
                            if last_good_points_l is not None and last_good_points_r is not None:
                                points_l = last_good_points_l.copy()
                                points_r = last_good_points_r.copy()
                                hold_xyz, hold_angles, hold_plane, hold_metrics = _evaluate_points(
                                    points_l,
                                    points_r,
                                    k_l=k_l,
                                    d_l=d_l,
                                    k_r=k_r,
                                    d_r=d_r,
                                    p_l=p_l,
                                    p_r=p_r,
                                    t_lr_mm=t_lr_mm,
                                    reference_dims_mm=reference_dims_mm,
                                )
                                ordered_xyz = hold_xyz
                                angles = hold_angles
                                plane_frame = hold_plane
                                geometry_metrics = hold_metrics
                                bad_angle = geometry_metrics.max_angle_error_deg if geometry_metrics is not None else float("nan")
                                if geometry_bad_frames >= GEOMETRY_MAX_BAD_FRAMES:
                                    status = f"Geometry hold (persistent) -> AI reacquire  |  ang={bad_angle:.1f}deg"
                                else:
                                    status = f"Geometry hold -> AI snap  |  ang={bad_angle:.1f}deg"
                            else:
                                # Keep the currently visible points alive so the overlay never vanishes.
                                # We still request a fresh AI snap, but we do not clear the user's view.
                                bad_angle = geometry_metrics.max_angle_error_deg if geometry_metrics is not None else float("nan")
                                status = f"Geometry bad -> keeping current points, waiting for AI  |  ang={bad_angle:.1f}deg"

                if plane_frame is not None:
                    box_camera_pos = _camera_position_in_box_frame(plane_frame)
                    box_debug_lines, box_log_line, box_jump_mm = _build_box_camera_debug(
                        frame_idx,
                        box_camera_pos,
                        previous_box_camera_pos,
                        points_l,
                        points_r,
                        geometry_metrics,
                    )
                    previous_box_camera_pos = box_camera_pos.copy()
                    if plot_frame_mode == "box":
                        now_debug = time.perf_counter()
                        if (
                            now_debug - last_camera_debug_print_s >= CAMERA_DEBUG_PRINT_PERIOD_S
                            or box_jump_mm >= CAMERA_DEBUG_JUMP_WARN_MM
                        ):
                            prefix = "[BOX JUMP]" if box_jump_mm >= CAMERA_DEBUG_JUMP_WARN_MM else "[BOX POSE]"
                            print(f"{prefix} {box_log_line}")
                            last_camera_debug_print_s = now_debug

                left_view = _overlay_segmentation(left, ai_l, points_l)
                right_view = _overlay_segmentation(right, ai_r, points_r)
                preview_lr = _compose_preview(stereo, left_view, right_view)

                if plot_render is None:
                    plot_w = max(500, int(preview_lr.shape[0] * 1.12))
                    plot_render = _make_plotter(preview_lr.shape[0], plot_w, t_lr_mm)
                plot_img = plot_render(
                    ordered_xyz,
                    angles,
                    plane_frame,
                    camera_level_frame,
                    plot_frame_mode,
                    box_debug_lines if plot_frame_mode == "box" else None,
                    status,
                )
                if plot_img.shape[0] != preview_lr.shape[0]:
                    plot_img = cv2.resize(plot_img, (plot_img.shape[1], preview_lr.shape[0]), interpolation=cv2.INTER_AREA)

                divider = np.full((preview_lr.shape[0], PLOT_DIVIDER_W, 3), 24, dtype=np.uint8)
                combined = cv2.hconcat([preview_lr, divider, plot_img])

                if config.AI_TOPFACE_RECORD_VIDEO:
                    if recorder is None:
                        RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        recorder = CompositeFrameRecorder(RECORDINGS_DIR / f"live_ai_plane_test_{timestamp}.mp4")
                    recorder.write(combined)

                cv2.imshow(WINDOW_NAME, combined)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("c"):
                    points_l = None
                    points_r = None
                    last_good_points_l = None
                    last_good_points_r = None
                    previous_box_camera_pos = None
                    last_camera_debug_print_s = 0.0
                    geometry_bad_frames = 0
                    force_ai = True
                    status = "Cleared pairing"
                elif key == ord("a"):
                    force_ai = True
                    status = "Manual AI snap requested"
                elif key == ord("m"):
                    plot_frame_mode = "camera" if plot_frame_mode == "box" else "box"
                    status = f"Plot mode -> {plot_frame_mode} frame"
                elif key == ord("p"):
                    if camera_level_frame is None:
                        if plane_frame is not None:
                            camera_level_frame = PlaneFrame(
                                origin=plane_frame.origin.copy(),
                                x_axis=plane_frame.x_axis.copy(),
                                y_axis=plane_frame.y_axis.copy(),
                                z_axis=plane_frame.z_axis.copy(),
                                box_length_mm=plane_frame.box_length_mm,
                                box_width_mm=plane_frame.box_width_mm,
                            )
                            status = "Camera frame leveled to current box plane"
                        else:
                            status = "Cannot level: need a valid box plane first"
                    else:
                        camera_level_frame = None
                        status = "Camera frame leveling cleared"
                elif key == ord("]"):
                    conf = min(0.95, conf + 0.05)
                    force_ai = True
                    print(f"[AI] confidence threshold = {conf:.2f}")
                elif key == ord("["):
                    conf = max(0.05, conf - 0.05)
                    force_ai = True
                    print(f"[AI] confidence threshold = {conf:.2f}")

                prev_gray_l = gray_l
                prev_gray_r = gray_r
                frame_idx += 1
    finally:
        if recorder is not None:
            recorder.release()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
