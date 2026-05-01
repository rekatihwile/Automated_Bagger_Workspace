from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
from pathlib import Path
import sys
from typing import Optional

import cv2
import numpy as np

import config


DIVIDER_W = 8
PLOT_XY_LIMIT_MM = 500.0
PLOT_Z_MIN_MM = -100.0
PLOT_Z_MAX_MM = 1000.0
MAX_CORNERS = 4

# ---------------------------------------------------------------------------
# USER SETTINGS (edit these)
# ---------------------------------------------------------------------------

# Paste your stitched input video path here.
# Example: r"C:\Users\elipp\OneDrive\Documents\Grocery Bagger\calibration\raw_stitched_recordings\raw_stitched_20260423_120259.mp4"
INPUT_VIDEO_PATH = r"C:\Users\elipp\OneDrive\Documents\Grocery Bagger\calibration\raw_stitched_recordings\raw_stitched_20260423_130427.mp4"

# Leave blank to auto-save under calibration/meshed_recordings.
# Can be a folder path, or a file path (parent folder will be used for PNG outputs).
OUTPUT_VIDEO_PATH = r"C:\Users\elipp\OneDrive\Documents\Grocery Bagger\calibration\meshed_recordings"

# Leave blank to use default model resolution logic in ai_topface.py.
YOLO_MODEL_PATH = r""

YOLO_CONFIDENCE = 0.35
YOLO_INPUT_SIZE = int(getattr(config, "AI_INPUT_SIZE", 640))
BOUNDARY_SAMPLES = int(getattr(config, "AI_BOUNDARY_SAMPLE_COUNT", 32))
MAX_STEREO_Y_ERROR_PX = 28.0

# Process only this many sampled frames from the input video.
SAMPLE_FRAME_COUNT = 4

# Show live preview while processing sampled frames.
# Keep this False for unattended runs from terminal.
SHOW_PREVIEW = False

# If preview is enabled, set True to require keypress per sample.
# False uses timed auto-advance so the script does not hang.
PREVIEW_BLOCKING_MODE = False
PREVIEW_WAIT_MS = 900

# Save each sampled frame visualization as PNG.
SAVE_PNG_RESULTS = True

# Preview/plot layout.
# Plot width is relative to stitched frame width (left+right).
PLOT_WIDTH_RATIO = 0.55
WINDOW_NAME = "YOLO Mesh Postprocess"

# Fit preview to monitor so combined view stays inside parent window.
FIT_PREVIEW_TO_SCREEN = True
SCREEN_MARGIN_PX = 80

# For stationary objects, keep mesh frame orientation fixed.
LOCK_MESH_FRAME_TO_FIRST_VALID = True
MAX_MESH_NORMAL_STEP_DEG = 35.0

# If INPUT_VIDEO_PATH is blank, auto-fallback to latest recording when available.
AUTO_USE_LATEST_VIDEO_IF_EMPTY = True


@dataclass
class BoundaryState:
    left_points: np.ndarray
    right_points: np.ndarray
    cyclic_shift: int


@dataclass
class PlaneFrame:
    origin_cam_mm: np.ndarray
    x_axis_cam: np.ndarray
    y_axis_cam: np.ndarray
    z_axis_cam: np.ndarray

    def rotation_rows(self) -> np.ndarray:
        return np.vstack([self.x_axis_cam, self.y_axis_cam, self.z_axis_cam]).astype(np.float64)


def _order_points_ccw_2d(points_xy: np.ndarray) -> np.ndarray:
    pts = np.asarray(points_xy, dtype=np.float32).reshape(-1, 2)
    c = np.mean(pts, axis=0)
    a = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
    return pts[np.argsort(a)]


def _topology_preserving_permutations(n: int) -> list[tuple[int, ...]]:
    base = tuple(range(n))
    reverse = tuple([0, *range(n - 1, 0, -1)])
    out: list[tuple[int, ...]] = []
    seen: set[tuple[int, ...]] = set()
    for seq in (base, reverse):
        for shift in range(n):
            perm = tuple(seq[(i + shift) % n] for i in range(n))
            if perm not in seen:
                seen.add(perm)
                out.append(perm)
    return out


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
    return _order_points_ccw_2d(quad)


def _match_corner_constellation(left_points: np.ndarray, right_points: np.ndarray) -> np.ndarray | None:
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
        left_edges /= max(float(np.mean(left_edges)), 1e-6)
        right_edges /= max(float(np.mean(right_edges)), 1e-6)
        shape_error = float(np.mean(np.abs(left_edges - right_edges)))

        score = float(np.std(disparities) + 2.0 * np.mean(np.abs(y_offsets)) + np.std(y_offsets) + 4.0 * shape_error)
        if score < best_score:
            best_score = score
            best_perm = np.asarray(perm, dtype=np.int32)
    return best_perm


def _order_plane_corners(points_3d: np.ndarray) -> np.ndarray:
    pts = np.asarray(points_3d, dtype=np.float64).reshape(-1, 3)
    c = np.mean(pts, axis=0)
    centered = pts - c
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    u = vh[0]
    v = vh[1]
    coords = np.column_stack([centered @ u, centered @ v])
    ang = np.arctan2(coords[:, 1], coords[:, 0])
    return np.argsort(ang)


def _make_plane_frame_from_corners(corners_cam_mm: np.ndarray) -> PlaneFrame | None:
    pts = np.asarray(corners_cam_mm, dtype=np.float64).reshape(-1, 3)
    if len(pts) != MAX_CORNERS:
        return None

    origin = np.mean(pts, axis=0)
    edge_a = ((pts[1] - pts[0]) + (pts[2] - pts[3])) * 0.5
    edge_b = ((pts[2] - pts[1]) + (pts[3] - pts[0])) * 0.5
    len_a = 0.5 * (np.linalg.norm(pts[1] - pts[0]) + np.linalg.norm(pts[2] - pts[3]))
    len_b = 0.5 * (np.linalg.norm(pts[2] - pts[1]) + np.linalg.norm(pts[3] - pts[0]))

    if len_a >= len_b:
        x_vec, y_vec = edge_a, edge_b
    else:
        x_vec, y_vec = edge_b, edge_a

    x_axis = _safe_normalize(x_vec)
    y_rough = _safe_normalize(y_vec)
    if x_axis is None or y_rough is None:
        return None

    z_axis = _safe_normalize(np.cross(x_axis, y_rough))
    if z_axis is None:
        return None

    # Keep +Z pointing toward camera, so the top face is close to z=0 and camera is +Z.
    if float(np.dot(-origin, z_axis)) < 0.0:
        z_axis = -z_axis

    y_axis = _safe_normalize(np.cross(z_axis, x_axis))
    if y_axis is None:
        return None
    x_axis = _safe_normalize(np.cross(y_axis, z_axis))
    if x_axis is None:
        return None

    return PlaneFrame(
        origin_cam_mm=origin,
        x_axis_cam=x_axis,
        y_axis_cam=y_axis,
        z_axis_cam=z_axis,
    )


def _transform_points_to_plane_frame(points_cam: np.ndarray, plane_frame: PlaneFrame) -> np.ndarray:
    pts = np.asarray(points_cam, dtype=np.float64).reshape(-1, 3)
    r = plane_frame.rotation_rows()
    return (pts - plane_frame.origin_cam_mm) @ r.T


def _vec3_list(v: np.ndarray | list[float] | tuple[float, float, float]) -> list[float]:
    a = np.asarray(v, dtype=np.float64).reshape(3)
    return [float(a[0]), float(a[1]), float(a[2])]


def _mat3_rows(m: np.ndarray) -> list[list[float]]:
    a = np.asarray(m, dtype=np.float64).reshape(3, 3)
    return [
        [float(a[0, 0]), float(a[0, 1]), float(a[0, 2])],
        [float(a[1, 0]), float(a[1, 1]), float(a[1, 2])],
        [float(a[2, 0]), float(a[2, 1]), float(a[2, 2])],
    ]


def _split_stitched(frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    mid = frame.shape[1] // 2
    return frame[:, :mid].copy(), frame[:, mid:].copy()


def _get_screen_size() -> tuple[int, int]:
    try:
        import ctypes

        user32 = ctypes.windll.user32
        return int(user32.GetSystemMetrics(0)), int(user32.GetSystemMetrics(1))
    except Exception:
        return 1600, 900


def _resize_to_fit(frame: np.ndarray, max_w: int, max_h: int) -> np.ndarray:
    h, w = frame.shape[:2]
    if w <= max_w and h <= max_h:
        return frame
    scale = min(max_w / max(w, 1), max_h / max(h, 1))
    out_w = max(1, int(round(w * scale)))
    out_h = max(1, int(round(h * scale)))
    return cv2.resize(frame, (out_w, out_h), interpolation=cv2.INTER_AREA)


def _draw_seg_debug(
    frame: np.ndarray,
    seg: Optional[SegmentationResult],
    points: Optional[np.ndarray],
    color: tuple[int, int, int],
) -> np.ndarray:
    out = frame.copy()
    if seg is None:
        return out

    overlay = out.copy()
    overlay[seg.mask > 0] = color
    out = cv2.addWeighted(overlay, 0.22, out, 0.78, 0)

    contour = np.round(seg.contour).astype(np.int32).reshape(-1, 1, 2)
    if len(contour) >= 3:
        cv2.polylines(out, [contour], True, color, 2, cv2.LINE_AA)

    if points is not None and len(points):
        for p in points:
            x, y = np.round(p).astype(np.int32)
            cv2.circle(out, (int(x), int(y)), 2, (255, 0, 255), -1, cv2.LINE_AA)

    return out


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


def _triangulate_points(
    pts_l: np.ndarray,
    pts_r: np.ndarray,
    *,
    k_l: np.ndarray,
    d_l: np.ndarray,
    k_r: np.ndarray,
    d_r: np.ndarray,
    p_l: np.ndarray,
    p_r: np.ndarray,
    t_lr_mm: np.ndarray,
) -> np.ndarray:
    if len(pts_l) == 0:
        return np.empty((0, 3), dtype=np.float64)

    out = np.empty((len(pts_l), 3), dtype=np.float64)
    for i, (pl, pr) in enumerate(zip(pts_l, pts_r)):
        out[i] = _triangulate_centered_mm(pl, pr, k_l, d_l, k_r, d_r, p_l, p_r, t_lr_mm)
    return out


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

    if prev is not None and len(prev.left_points) == len(left_pts) and len(prev.right_points) == len(right_pts):
        left_shift = _best_cyclic_shift_to_previous(left_pts, prev.left_points)
        left_pts = np.roll(left_pts, shift=left_shift, axis=0)

        right_shift = _best_cyclic_shift_to_previous(right_pts, prev.right_points)
        right_pts = np.roll(right_pts, shift=right_shift, axis=0)

        stereo_shift = _best_cyclic_shift_epipolar(left_pts, right_pts)
        right_pts = np.roll(right_pts, shift=stereo_shift, axis=0)
        return BoundaryState(left_points=left_pts, right_points=right_pts, cyclic_shift=stereo_shift)

    stereo_shift = _best_cyclic_shift_epipolar(left_pts, right_pts)
    right_pts = np.roll(right_pts, shift=stereo_shift, axis=0)
    return BoundaryState(left_points=left_pts, right_points=right_pts, cyclic_shift=stereo_shift)


def _sample_inside_mask(mask: np.ndarray, step_px: int, max_points: int) -> np.ndarray:
    step = max(2, int(step_px))
    h, w = mask.shape[:2]
    inner = cv2.erode(mask, np.ones((3, 3), dtype=np.uint8), iterations=1)

    xs = np.arange(step // 2, w, step, dtype=np.int32)
    ys = np.arange(step // 2, h, step, dtype=np.int32)
    if len(xs) == 0 or len(ys) == 0:
        return np.empty((0, 2), dtype=np.float32)

    xv, yv = np.meshgrid(xs, ys)
    cand = np.column_stack([xv.reshape(-1), yv.reshape(-1)]).astype(np.int32)
    inside = inner[cand[:, 1], cand[:, 0]] > 0
    pts = cand[inside].astype(np.float32)

    if len(pts) <= max_points:
        return pts

    idx = np.linspace(0, len(pts) - 1, int(max_points)).astype(np.int32)
    return pts[idx]


def _project_homography(points_xy: np.ndarray, h_mat: np.ndarray) -> np.ndarray:
    if len(points_xy) == 0:
        return np.empty((0, 2), dtype=np.float32)
    projected = cv2.perspectiveTransform(points_xy.reshape(-1, 1, 2).astype(np.float32), h_mat)
    return projected.reshape(-1, 2).astype(np.float32)


def _valid_pair_mask(
    pts_l: np.ndarray,
    pts_r: np.ndarray,
    mask_l: np.ndarray,
    mask_r: np.ndarray,
    max_y_error_px: float,
    boundary_count: int = 0,
) -> np.ndarray:
    h_l, w_l = mask_l.shape[:2]
    h_r, w_r = mask_r.shape[:2]

    xl = np.round(pts_l[:, 0]).astype(np.int32)
    yl = np.round(pts_l[:, 1]).astype(np.int32)
    xr = np.round(pts_r[:, 0]).astype(np.int32)
    yr = np.round(pts_r[:, 1]).astype(np.int32)

    in_l = (xl >= 0) & (xl < w_l) & (yl >= 0) & (yl < h_l)
    in_r = (xr >= 0) & (xr < w_r) & (yr >= 0) & (yr < h_r)

    good = in_l & in_r
    if np.any(good):
        good_idx = np.where(good)[0]
        good[good_idx] &= mask_l[yl[good_idx], xl[good_idx]] > 0
        good[good_idx] &= mask_r[yr[good_idx], xr[good_idx]] > 0

    good &= np.abs(pts_r[:, 1] - pts_l[:, 1]) <= float(max_y_error_px)
    if boundary_count > 0:
        keep_n = min(int(boundary_count), len(good))
        good[:keep_n] = in_l[:keep_n] & in_r[:keep_n]
    return good


def _build_triangles_2d(points_xy: np.ndarray, mask: np.ndarray) -> np.ndarray:
    if len(points_xy) < 3:
        return np.empty((0, 3), dtype=np.int32)

    import matplotlib.tri as mtri

    try:
        tri = mtri.Triangulation(points_xy[:, 0], points_xy[:, 1])
    except Exception:
        return np.empty((0, 3), dtype=np.int32)

    triangles = np.asarray(tri.triangles, dtype=np.int32)
    if len(triangles) == 0:
        return triangles

    h, w = mask.shape[:2]
    keep: list[np.ndarray] = []
    for t in triangles:
        pts = points_xy[t]
        area = 0.5 * abs(
            pts[0, 0] * (pts[1, 1] - pts[2, 1])
            + pts[1, 0] * (pts[2, 1] - pts[0, 1])
            + pts[2, 0] * (pts[0, 1] - pts[1, 1])
        )
        if area < 1.0:
            continue

        cx = int(round(float(np.mean(pts[:, 0]))))
        cy = int(round(float(np.mean(pts[:, 1]))))
        if cx < 0 or cx >= w or cy < 0 or cy >= h:
            continue
        if mask[cy, cx] == 0:
            continue
        keep.append(t)

    if not keep:
        return np.empty((0, 3), dtype=np.int32)
    return np.asarray(keep, dtype=np.int32)


def _filter_invalid_mesh(
    points_xy: np.ndarray,
    points_xyz: np.ndarray,
    triangles: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if len(points_xyz) == 0:
        return points_xy, points_xyz, np.empty((0, 3), dtype=np.int32)

    finite = np.all(np.isfinite(points_xyz), axis=1)
    finite &= np.abs(points_xyz[:, 2]) < 10000.0
    finite &= np.linalg.norm(points_xyz, axis=1) < 100000.0

    if not np.any(finite):
        return (
            np.empty((0, 2), dtype=np.float32),
            np.empty((0, 3), dtype=np.float64),
            np.empty((0, 3), dtype=np.int32),
        )

    idx_map = -np.ones(len(finite), dtype=np.int32)
    idx_map[np.where(finite)[0]] = np.arange(int(np.count_nonzero(finite)), dtype=np.int32)

    points_xy_out = points_xy[finite]
    points_xyz_out = points_xyz[finite]

    tri_out: list[np.ndarray] = []
    for tri in triangles:
        remap = idx_map[tri]
        if np.any(remap < 0):
            continue
        if len({int(remap[0]), int(remap[1]), int(remap[2])}) < 3:
            continue
        tri_out.append(remap)

    if not tri_out:
        return points_xy_out, points_xyz_out, np.empty((0, 3), dtype=np.int32)
    return points_xy_out, points_xyz_out, np.asarray(tri_out, dtype=np.int32)


def _safe_normalize(v: np.ndarray, eps: float = 1e-9) -> np.ndarray | None:
    n = float(np.linalg.norm(v))
    if n <= eps:
        return None
    return np.asarray(v, dtype=np.float64) / n


def _project_onto_plane(v: np.ndarray, plane_normal: np.ndarray) -> np.ndarray:
    vec = np.asarray(v, dtype=np.float64).reshape(3)
    n = _safe_normalize(np.asarray(plane_normal, dtype=np.float64).reshape(3))
    if n is None:
        return vec
    return vec - float(np.dot(vec, n)) * n


def _axis_in_plane(v: np.ndarray, plane_normal: np.ndarray) -> np.ndarray | None:
    return _safe_normalize(_project_onto_plane(v, plane_normal))


def _rotate_vector_about_axis(v: np.ndarray, axis: np.ndarray, angle_rad: float) -> np.ndarray:
    vec = np.asarray(v, dtype=np.float64).reshape(3)
    ax = _safe_normalize(np.asarray(axis, dtype=np.float64).reshape(3))
    if ax is None:
        return vec
    ct = float(np.cos(angle_rad))
    st = float(np.sin(angle_rad))
    return vec * ct + np.cross(ax, vec) * st + ax * float(np.dot(ax, vec)) * (1.0 - ct)


def _limit_vector_turn(prev_v: np.ndarray, target_v: np.ndarray, max_angle_deg: float) -> np.ndarray:
    prev = _safe_normalize(np.asarray(prev_v, dtype=np.float64).reshape(3))
    target = _safe_normalize(np.asarray(target_v, dtype=np.float64).reshape(3))
    if prev is None:
        return target if target is not None else np.array([0.0, 0.0, 1.0], dtype=np.float64)
    if target is None:
        return prev

    dot = float(np.clip(np.dot(prev, target), -1.0, 1.0))
    angle = float(np.arccos(dot))
    max_angle_rad = float(np.deg2rad(max(0.0, float(max_angle_deg))))
    if angle <= max_angle_rad:
        return target

    axis = _safe_normalize(np.cross(prev, target))
    if axis is None:
        trial = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(float(np.dot(trial, prev))) > 0.9:
            trial = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        axis = _safe_normalize(np.cross(prev, trial))
        if axis is None:
            return prev

    limited = _rotate_vector_about_axis(prev, axis, max_angle_rad)
    limited_n = _safe_normalize(limited)
    return limited_n if limited_n is not None else prev


def _average_mesh_normal(
    points_cam: np.ndarray,
    triangles: np.ndarray | None,
) -> np.ndarray | None:
    pts = np.asarray(points_cam, dtype=np.float64).reshape(-1, 3)
    if len(pts) < 3:
        return None

    centered = pts - np.mean(pts, axis=0)
    plane_n = None
    try:
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        if vh.shape[0] >= 3:
            plane_n = _safe_normalize(vh[-1])
    except Exception:
        plane_n = None

    tri_n = None
    if triangles is not None and len(triangles) > 0:
        tri = np.asarray(triangles, dtype=np.int32)
        tri_pts = pts[tri]
        e1 = tri_pts[:, 1] - tri_pts[:, 0]
        e2 = tri_pts[:, 2] - tri_pts[:, 0]
        normals = np.cross(e1, e2)
        tri_n = _safe_normalize(np.sum(normals, axis=0))

    if plane_n is not None and tri_n is not None:
        if float(np.dot(plane_n, tri_n)) < 0.0:
            tri_n = -tri_n
        blended = _safe_normalize((2.0 * plane_n) + tri_n)
        if blended is not None:
            return blended
    if plane_n is not None:
        return plane_n
    if tri_n is not None:
        return tri_n
    return None


def _build_mesh_frame(
    points_cam: np.ndarray,
    triangles: np.ndarray | None,
    left_cam: np.ndarray,
    right_cam: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Returns (center_cam, R_mesh_from_cam).
    center_cam is the mesh centroid in camera frame.
    R_mesh_from_cam maps centered camera-frame vectors into mesh frame:
      p_mesh = (p_cam - center_cam) @ R_mesh_from_cam.T
    Mesh frame:
      +Z = average mesh normal (oriented toward camera),
      +X = dominant in-plane mesh axis (fallback: projected stereo baseline),
      +Y = right-handed completion.
    """
    pts = np.asarray(points_cam, dtype=np.float64).reshape(-1, 3)
    center_cam = np.mean(pts, axis=0)

    z_axis = _average_mesh_normal(pts, triangles)
    if z_axis is None:
        z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)

    # Orient normal so it points toward the camera (camera at origin in cam frame).
    to_camera = -center_cam
    if float(np.dot(z_axis, to_camera)) < 0.0:
        z_axis = -z_axis

    # Prefer object geometry (principal axis in mesh plane) so frame is tied to the box,
    # not to the camera baseline direction.
    centered = pts - center_cam
    flat = centered - np.outer(centered @ z_axis, z_axis)
    x_axis = None
    if x_axis is None:
        try:
            _, _, vh = np.linalg.svd(flat, full_matrices=False)
            x_axis = _safe_normalize(vh[0])
        except Exception:
            x_axis = None
    if x_axis is None:
        baseline = np.asarray(right_cam, dtype=np.float64) - np.asarray(left_cam, dtype=np.float64)
        x_proj = baseline - float(np.dot(baseline, z_axis)) * z_axis
        x_axis = _safe_normalize(x_proj)
    if x_axis is None:
        # Final fallback: any vector orthogonal to z.
        trial = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(float(np.dot(trial, z_axis))) > 0.9:
            trial = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        x_axis = _safe_normalize(trial - float(np.dot(trial, z_axis)) * z_axis)
        if x_axis is None:
            x_axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)

    y_axis = _safe_normalize(np.cross(z_axis, x_axis))
    if y_axis is None:
        y_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    x_axis = _safe_normalize(np.cross(y_axis, z_axis))
    if x_axis is None:
        x_axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)

    r_mesh_from_cam = np.vstack([x_axis, y_axis, z_axis]).astype(np.float64)
    return center_cam, r_mesh_from_cam


def _orthonormalize_frame_rows(r: np.ndarray) -> np.ndarray:
    r = np.asarray(r, dtype=np.float64).reshape(3, 3)
    x = _safe_normalize(r[0])
    z = _safe_normalize(r[2])
    if x is None:
        x = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if z is None:
        z = np.array([0.0, 0.0, 1.0], dtype=np.float64)

    y = _safe_normalize(np.cross(z, x))
    if y is None:
        y = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    x = _safe_normalize(np.cross(y, z))
    if x is None:
        x = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    y = _safe_normalize(np.cross(z, x))
    if y is None:
        y = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    return np.vstack([x, y, z]).astype(np.float64)


def _stabilize_frame_to_history(
    r_now: np.ndarray,
    center_cam: np.ndarray,
    r_prev: np.ndarray | None,
    r_ref: np.ndarray | None,
) -> np.ndarray:
    """
    Keep the mesh frame continuous across frames.

    The mesh normal still follows the current 3D fit, but we cap unrealistic per-frame
    normal turns and transport the previous in-plane axis onto the new plane instead of
    re-solving that ambiguous axis from scratch every frame.
    """
    r = _orthonormalize_frame_rows(r_now)
    center = np.asarray(center_cam, dtype=np.float64).reshape(3)
    to_camera = _safe_normalize(-center)

    z_axis = r[2].copy()
    if to_camera is not None and float(np.dot(z_axis, to_camera)) < 0.0:
        z_axis = -z_axis

    prev = None if r_prev is None else np.asarray(r_prev, dtype=np.float64).reshape(3, 3)
    ref = None if r_ref is None else np.asarray(r_ref, dtype=np.float64).reshape(3, 3)

    if prev is not None:
        prev_z = prev[2].copy()
        if to_camera is not None and float(np.dot(prev_z, to_camera)) < 0.0:
            prev_z = -prev_z
        z_axis = _limit_vector_turn(prev_z, z_axis, MAX_MESH_NORMAL_STEP_DEG)
        if to_camera is not None and float(np.dot(z_axis, to_camera)) < 0.0:
            z_axis = -z_axis

    blended_x = np.zeros(3, dtype=np.float64)
    total_w = 0.0

    if prev is not None:
        prev_x = _axis_in_plane(prev[0], z_axis)
        if prev_x is not None:
            blended_x += 0.8 * prev_x
            total_w += 0.8

    if ref is not None:
        ref_x = _axis_in_plane(ref[0], z_axis)
        if ref_x is not None:
            blended_x += 0.2 * ref_x
            total_w += 0.2

    x_axis = _safe_normalize(blended_x) if total_w > 0.0 else None
    if x_axis is None:
        x_axis = _axis_in_plane(r[0], z_axis)
    if x_axis is None and prev is not None:
        prev_y = _axis_in_plane(prev[1], z_axis)
        if prev_y is not None:
            x_axis = _safe_normalize(np.cross(prev_y, z_axis))
    if x_axis is None:
        trial = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(float(np.dot(trial, z_axis))) > 0.9:
            trial = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        x_axis = _safe_normalize(_project_onto_plane(trial, z_axis))
    if x_axis is None:
        x_axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)

    if prev is not None and float(np.dot(x_axis, prev[0])) < 0.0:
        x_axis = -x_axis

    y_axis = _safe_normalize(np.cross(z_axis, x_axis))
    if y_axis is None:
        y_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    x_axis = _safe_normalize(np.cross(y_axis, z_axis))
    if x_axis is None:
        x_axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    y_axis = _safe_normalize(np.cross(z_axis, x_axis))
    if y_axis is None:
        y_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)

    return np.vstack([x_axis, y_axis, z_axis]).astype(np.float64)


def _make_plotter(height_px: int, width_px: int, t_lr_mm: np.ndarray):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(width_px / 100, height_px / 100), dpi=100)
    ax = fig.add_subplot(111, projection="3d")
    fig.subplots_adjust(left=0.03, right=0.98, bottom=0.05, top=0.93)

    t_vec = np.asarray(t_lr_mm, dtype=np.float64).reshape(3)
    if np.linalg.norm(t_vec) < 1e-6:
        t_vec = np.array([80.0, 0.0, 0.0], dtype=np.float64)

    left_cam = -0.5 * t_vec
    right_cam = 0.5 * t_vec
    origin = np.zeros(3, dtype=np.float64)
    view_tip = origin + np.array([0.0, 0.0, max(80.0, float(np.linalg.norm(t_vec)) * 0.9)])

    def render(
        corners_cam: np.ndarray | None,
        triangles: np.ndarray | None,
        status: str,
    ) -> tuple[np.ndarray, dict[str, object]]:
        ax.cla()
        ax.set_title("Box Frame (Top Face = XY Plane)", fontsize=11)
        ax.set_xlabel("Box X (mm)", labelpad=6, fontsize=9)
        ax.set_ylabel("Box Y (mm)", labelpad=6, fontsize=9)
        ax.set_zlabel("Box Z (mm)", labelpad=7, fontsize=9)
        ax.tick_params(axis="both", which="major", labelsize=8, pad=1)

        pose_debug: dict[str, object] = {
            "has_plane": False,
            "left_camera_position_cam_mm": _vec3_list(left_cam),
            "right_camera_position_cam_mm": _vec3_list(right_cam),
            "camera_origin_cam_mm": _vec3_list(origin),
            "camera_view_tip_cam_mm": _vec3_list(view_tip),
        }

        has_plane = corners_cam is not None and len(corners_cam) == MAX_CORNERS and triangles is not None and len(triangles) > 0
        if has_plane:
            corners_cam_np = np.asarray(corners_cam, dtype=np.float64).reshape(-1, 3)
            plane_frame = _make_plane_frame_from_corners(corners_cam_np)
            if plane_frame is not None:
                pts = _transform_points_to_plane_frame(corners_cam_np, plane_frame)
                left_plot = _transform_points_to_plane_frame(left_cam.reshape(1, 3), plane_frame)[0]
                right_plot = _transform_points_to_plane_frame(right_cam.reshape(1, 3), plane_frame)[0]
                origin_plot = _transform_points_to_plane_frame(origin.reshape(1, 3), plane_frame)[0]
                view_tip_plot = _transform_points_to_plane_frame(view_tip.reshape(1, 3), plane_frame)[0]

                pose_debug = {
                    "has_plane": True,
                    "plane_origin_cam_mm": _vec3_list(plane_frame.origin_cam_mm),
                    "rotation_box_from_camera_rows": _mat3_rows(plane_frame.rotation_rows()),
                    "rotation_camera_from_box_rows": _mat3_rows(plane_frame.rotation_rows().T),
                    "camera_origin_box_mm": _vec3_list(origin_plot),
                    "left_camera_position_box_mm": _vec3_list(left_plot),
                    "right_camera_position_box_mm": _vec3_list(right_plot),
                    "camera_view_tip_box_mm": _vec3_list(view_tip_plot),
                    "camera_to_box_center_mm": float(np.linalg.norm(plane_frame.origin_cam_mm)),
                }

                ax.plot(
                    [left_plot[0], right_plot[0]],
                    [left_plot[1], right_plot[1]],
                    [left_plot[2], right_plot[2]],
                    color="black",
                    linewidth=1.8,
                )
                ax.scatter(
                    [left_plot[0], right_plot[0]],
                    [left_plot[1], right_plot[1]],
                    [left_plot[2], right_plot[2]],
                    color="royalblue",
                    s=24,
                )
                ax.scatter([origin_plot[0]], [origin_plot[1]], [origin_plot[2]], color="black", s=28)
                ax.plot(
                    [origin_plot[0], view_tip_plot[0]],
                    [origin_plot[1], view_tip_plot[1]],
                    [origin_plot[2], view_tip_plot[2]],
                    color="black",
                    linewidth=1.2,
                )

                polys = pts[np.asarray(triangles, dtype=np.int32)]
                poly = Poly3DCollection(polys, alpha=0.35, facecolor="orange", edgecolor="firebrick", linewidth=1.0)
                ax.add_collection3d(poly)

                closed = np.vstack([pts, pts[0]])
                ax.plot(closed[:, 0], closed[:, 1], closed[:, 2], color="magenta", linewidth=1.3)
                ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], color="crimson", s=16)

                ax.quiver(0.0, 0.0, 0.0, 80.0, 0.0, 0.0, color="red", arrow_length_ratio=0.12)
                ax.quiver(0.0, 0.0, 0.0, 0.0, 80.0, 0.0, color="green", arrow_length_ratio=0.12)
                ax.quiver(0.0, 0.0, 0.0, 0.0, 0.0, 80.0, color="blue", arrow_length_ratio=0.12)

                span_xy = max(180.0, float(np.max(np.ptp(pts[:, :2], axis=0))) * 1.3)
                lim_xy = max(200.0, min(float(PLOT_XY_LIMIT_MM), span_xy))
                z_vals = np.concatenate([pts[:, 2], np.array([left_plot[2], right_plot[2], origin_plot[2]], dtype=np.float64)])
                z_pad = max(120.0, float(np.ptp(z_vals)) * 0.5)
                z_mid = float(np.mean(z_vals))
                ax.set_xlim(-lim_xy, lim_xy)
                ax.set_ylim(-lim_xy, lim_xy)
                ax.set_zlim(z_mid - z_pad, z_mid + z_pad)
            else:
                has_plane = False

        if not has_plane:
            ax.set_xlim(-PLOT_XY_LIMIT_MM, PLOT_XY_LIMIT_MM)
            ax.set_ylim(-PLOT_XY_LIMIT_MM, PLOT_XY_LIMIT_MM)
            ax.set_zlim(PLOT_Z_MIN_MM, PLOT_Z_MAX_MM)
            ax.text2D(0.03, 0.05, status, transform=ax.transAxes, fontsize=9)

        try:
            ax.set_box_aspect((1, 1, 1))
        except AttributeError:
            pass
        ax.view_init(elev=24, azim=-58)

        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        rgba = np.asarray(fig.canvas.buffer_rgba(), dtype=np.uint8).reshape(h, w, 4)
        return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR), pose_debug

    return render


def _default_output_path(input_video: Path) -> Path:
    out_dir = config.CALIBRATION_ROOT / "meshed_recordings"
    out_dir.mkdir(parents=True, exist_ok=True)
    return _next_mesh_video_path(out_dir, input_video)


def _next_mesh_video_path(out_dir: Path, input_video: Path) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = f"{input_video.stem}_mesh3d_{stamp}"
    out = out_dir / f"{base}.mp4"
    idx = 1
    while out.exists():
        out = out_dir / f"{base}_{idx:02d}.mp4"
        idx += 1
    return out


def _avoid_overwrite_file(path: Path) -> Path:
    if not path.exists():
        return path
    stem = path.stem
    suffix = path.suffix
    idx = 1
    while True:
        candidate = path.with_name(f"{stem}_{idx:02d}{suffix}")
        if not candidate.exists():
            return candidate
        idx += 1


def _move_blocking_file_and_make_dir(path: Path) -> None:
    if not path.exists() or not path.is_file():
        path.mkdir(parents=True, exist_ok=True)
        return

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_name(f"{path.name}_legacy_file_{stamp}")
    idx = 1
    while backup.exists():
        backup = path.with_name(f"{path.name}_legacy_file_{stamp}_{idx:02d}")
        idx += 1
    path.rename(backup)
    print(f"[POST] Moved blocking file to: {backup}")
    path.mkdir(parents=True, exist_ok=True)


def _estimate_video_fps_from_times(timestamps_s: list[float], fallback_fps: float) -> float:
    if len(timestamps_s) < 2:
        return float(np.clip(fallback_fps, 1.0, 240.0))

    ts = np.asarray(timestamps_s, dtype=np.float64)
    dt = np.diff(ts)
    dt = dt[np.isfinite(dt) & (dt > 1e-6)]
    if len(dt) == 0:
        return float(np.clip(fallback_fps, 1.0, 240.0))

    fps_med = 1.0 / float(np.median(dt))
    if not np.isfinite(fps_med) or fps_med <= 0.0:
        return float(np.clip(fallback_fps, 1.0, 240.0))
    return float(np.clip(fps_med, 1.0, 240.0))


def _find_latest_video() -> Path | None:
    exts = (".mp4", ".avi", ".mov", ".mkv", ".m4v")
    scan_dirs = [
        config.CALIBRATION_ROOT / "raw_stitched_recordings",
        Path(getattr(config, "TEST_RECORDINGS_DIR", config.CALIBRATION_ROOT / "test_recordings")),
        config.CALIBRATION_ROOT,
    ]

    best: Path | None = None
    best_time = -1.0
    for d in scan_dirs:
        if not d.exists():
            continue
        for p in d.glob("*"):
            if not p.is_file() or p.suffix.lower() not in exts:
                continue
            # Prefer source videos, skip already rendered mesh outputs.
            if p.stem.lower().endswith("_mesh3d"):
                continue
            try:
                t = p.stat().st_mtime
            except OSError:
                continue
            if t > best_time:
                best_time = t
                best = p
    return best


def _resolve_input_video(path_arg: Path | None) -> Path:
    if path_arg is not None:
        candidate = Path(path_arg).expanduser()
        if not candidate.is_absolute():
            candidate = (Path.cwd() / candidate).resolve()
        if candidate.exists() and candidate.is_file():
            return candidate
        raise FileNotFoundError(f"Input video not found: {candidate}")

    latest = _find_latest_video() if AUTO_USE_LATEST_VIDEO_IF_EMPTY else None
    if not sys.stdin.isatty():
        if latest is not None:
            print(f"[POST] No video path provided. Using latest video: {latest}")
            return latest
        raise RuntimeError("No video path provided and no default video found.")

    print("Paste full video path and press Enter.")
    if latest is not None:
        print(f"Press Enter to use latest detected video: {latest}")

    while True:
        raw = input("> ").strip()
        if not raw:
            if latest is not None:
                return latest
            print("No default video found. Paste a valid file path.")
            continue

        cleaned = raw.strip().strip('"').strip("'")
        candidate = Path(cleaned).expanduser()
        if not candidate.is_absolute():
            candidate = (Path.cwd() / candidate).resolve()

        if candidate.exists() and candidate.is_file():
            return candidate
        print(f"Video not found: {candidate}")


def _path_from_setting(value: str) -> Path | None:
    v = str(value).strip()
    if not v:
        return None
    return Path(v)


def _resolve_output_path(input_video: Path, output_setting: str) -> Path:
    out_override = _path_from_setting(output_setting)
    if out_override is None:
        return _default_output_path(input_video)

    p = out_override
    if not p.is_absolute():
        p = (Path.cwd() / p).resolve()

    video_exts = {".mp4", ".avi", ".mov", ".mkv", ".m4v"}
    if p.suffix.lower() in video_exts:
        p.parent.mkdir(parents=True, exist_ok=True)
        return _avoid_overwrite_file(p)

    if p.exists() and p.is_dir():
        return _next_mesh_video_path(p, input_video)

    # Extension-less path is treated as folder target.
    if p.suffix == "":
        _move_blocking_file_and_make_dir(p)
        return _next_mesh_video_path(p, input_video)

    # Unknown suffix: still honor it as a concrete file path but avoid overwrite.
    p.parent.mkdir(parents=True, exist_ok=True)
    return _avoid_overwrite_file(p)


def _manifest_path_for_output(out_path: Path) -> Path:
    return out_path.with_name(f"{out_path.stem}_manifest.json")


def _write_manifest(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _resolve_model_path(model_setting: str) -> Path:
    from ai_topface import DEFAULT_MODEL_NAME

    override = _path_from_setting(model_setting)
    if override is not None:
        p = override
        if not p.is_absolute():
            p = (Path.cwd() / p).resolve()
        return p

    base = Path(__file__).resolve().parent
    preferred = base / "models" / DEFAULT_MODEL_NAME
    legacy = base / DEFAULT_MODEL_NAME
    return preferred if preferred.exists() else legacy


def _resolve_output_dir(input_video: Path, output_setting: str) -> Path:
    out_override = _path_from_setting(output_setting)
    if out_override is None:
        out_dir = config.CALIBRATION_ROOT / "meshed_recordings"
        out_dir.mkdir(parents=True, exist_ok=True)
        return out_dir

    p = out_override
    if not p.is_absolute():
        p = (Path.cwd() / p).resolve()

    video_exts = {".mp4", ".avi", ".mov", ".mkv", ".m4v"}
    image_exts = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".webp"}
    if p.suffix.lower() in video_exts or p.suffix.lower() in image_exts:
        p = p.parent

    if p.suffix == "":
        _move_blocking_file_and_make_dir(p)
        return p

    # Unknown extension: write to parent folder.
    p.parent.mkdir(parents=True, exist_ok=True)
    return p.parent


def _sample_frame_indices(total_frames: int, sample_count: int) -> list[int]:
    total = max(1, int(total_frames))
    count = int(np.clip(int(sample_count), 1, total))
    if count == 1:
        return [0]

    idx = np.linspace(0, total - 1, count).round().astype(np.int32)
    out: list[int] = []
    seen: set[int] = set()
    for i in idx:
        v = int(i)
        if v in seen:
            continue
        seen.add(v)
        out.append(v)

    # Fill gaps if duplicates reduced count.
    if len(out) < count:
        for v in range(total):
            if v in seen:
                continue
            out.append(v)
            seen.add(v)
            if len(out) >= count:
                break
    return sorted(out)


def _draw_hud_block(frame: np.ndarray, lines: list[str]) -> None:
    if not lines:
        return
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.58
    thickness = 1
    pad = 8
    line_h = 24

    widths = []
    for line in lines:
        (w, _), _ = cv2.getTextSize(line, font, scale, thickness)
        widths.append(w)
    box_w = max(widths) + 2 * pad
    box_h = line_h * len(lines) + pad

    x0, y0 = 10, 10
    x1, y1 = x0 + box_w, y0 + box_h
    cv2.rectangle(frame, (x0, y0), (x1, y1), (8, 8, 8), -1)
    cv2.rectangle(frame, (x0, y0), (x1, y1), (210, 210, 210), 1)

    y = y0 + line_h - 7
    for line in lines:
        cv2.putText(frame, line, (x0 + pad, y), font, scale, (245, 245, 245), thickness, cv2.LINE_AA)
        y += line_h


def main() -> None:
    print("[POST] Starting mesh postprocess...")
    input_video = _resolve_input_video(_path_from_setting(INPUT_VIDEO_PATH))
    print(f"[POST] Input video: {input_video}")

    calib = config.load_stereo_calibration()
    if calib is None:
        raise RuntimeError(f"No calibration found at {config.ACTIVE_CALIBRATION_NPZ}. Run calibration first.")

    k_l = np.asarray(calib["left_camera_matrix"], dtype=np.float64)
    d_l = np.asarray(calib["left_distortion_coefficients"], dtype=np.float64)
    k_r = np.asarray(calib["right_camera_matrix"], dtype=np.float64)
    d_r = np.asarray(calib["right_distortion_coefficients"], dtype=np.float64)
    p_l = np.asarray(calib["projection_left_raw"], dtype=np.float64)
    p_r = np.asarray(calib["projection_right_raw"], dtype=np.float64)
    t_lr_mm = np.asarray(calib["stereo_translation_left_to_right_mm"], dtype=np.float64)

    print("[POST] Importing segmentation dependencies...")
    from ai_topface import TopFaceSegmenter

    model_path = _resolve_model_path(YOLO_MODEL_PATH)
    print(f"[POST] Model path: {model_path}")
    if not model_path.exists() or not model_path.is_file():
        raise FileNotFoundError(
            "YOLO model file not found. Set YOLO_MODEL_PATH to a valid .pt file. "
            f"Resolved path: {model_path}"
        )

    print("[POST] Loading segmentation model...")
    segmenter = TopFaceSegmenter(
        model_path=model_path,
        sample_count=max(8, int(BOUNDARY_SAMPLES)),
        input_size=max(64, int(YOLO_INPUT_SIZE)),
    )
    print("[POST] Model loaded.")

    out_dir = _resolve_output_dir(input_video, OUTPUT_VIDEO_PATH)
    run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = out_dir / f"{input_video.stem}_mesh3d_samples_{run_stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = run_dir / "manifest.json"
    manifest_frames: list[dict[str, object]] = []

    cap = cv2.VideoCapture(str(input_video))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open input video: {input_video}")

    src_fps_meta = float(cap.get(cv2.CAP_PROP_FPS))
    if not np.isfinite(src_fps_meta) or src_fps_meta <= 0.0:
        src_fps_meta = 15.0

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    sampled_indices = _sample_frame_indices(total_frames, int(SAMPLE_FRAME_COUNT)) if total_frames > 0 else list(range(int(max(1, SAMPLE_FRAME_COUNT))))
    print(f"[POST] Total frames={total_frames}; sampled indices={sampled_indices}")

    plotter = None
    plotter_size: tuple[int, int] = (0, 0)
    mesh_ok = 0
    processed = 0
    quit_requested = False
    saved_images: list[Path] = []

    try:
        if SHOW_PREVIEW:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

        screen_w, screen_h = _get_screen_size()
        max_disp_w = max(640, int(screen_w - SCREEN_MARGIN_PX))
        max_disp_h = max(360, int(screen_h - SCREEN_MARGIN_PX))

        for sample_id, frame_idx in enumerate(sampled_indices, start=1):
            if total_frames > 0:
                cap.set(cv2.CAP_PROP_POS_FRAMES, float(frame_idx))

            ok, frame = cap.read()
            if not ok or frame is None:
                continue

            pos_msec = float(cap.get(cv2.CAP_PROP_POS_MSEC))
            frame_time_s = (pos_msec / 1000.0) if (np.isfinite(pos_msec) and pos_msec > 0.0) else (float(frame_idx) / max(1.0, float(src_fps_meta)))

            left, right = _split_stitched(frame)
            seg_l = segmenter.detect(left, conf=float(YOLO_CONFIDENCE))
            seg_r = segmenter.detect(right, conf=float(YOLO_CONFIDENCE))

            mesh_tri = np.empty((0, 3), dtype=np.int32)
            corners_xyz = np.empty((0, 3), dtype=np.float64)
            debug_points_l: Optional[np.ndarray] = None
            debug_points_r: Optional[np.ndarray] = None
            pose_debug: dict[str, object] = {"has_plane": False}
            mesh_valid = False

            if seg_l is None and seg_r is None:
                status = "Missing masks on both views"
            elif seg_l is None:
                status = "Missing left mask"
            elif seg_r is None:
                status = "Missing right mask"
            else:
                quad_l = _quad_from_segmentation(seg_l)
                quad_r = _quad_from_segmentation(seg_r)
                if quad_l is None or quad_r is None:
                    status = "Mask found, 4-corner fit failed"
                else:
                    perm = _match_corner_constellation(quad_l, quad_r)
                    if perm is None:
                        status = "4-corner stereo pairing failed"
                    else:
                        corners_l = quad_l.astype(np.float32)
                        corners_r = quad_r[perm].astype(np.float32)

                        pair_ok = _valid_pair_mask(
                            corners_l,
                            corners_r,
                            seg_l.mask,
                            seg_r.mask,
                            max_y_error_px=float(MAX_STEREO_Y_ERROR_PX),
                            boundary_count=MAX_CORNERS,
                        )

                        debug_points_l = corners_l
                        debug_points_r = corners_r
                        if not np.all(pair_ok):
                            status = "4-corner pairing failed mask/y checks"
                        else:
                            tri_xyz = _triangulate_points(
                                corners_l,
                                corners_r,
                                k_l=k_l,
                                d_l=d_l,
                                k_r=k_r,
                                d_r=d_r,
                                p_l=p_l,
                                p_r=p_r,
                                t_lr_mm=t_lr_mm,
                            )
                            if len(tri_xyz) == MAX_CORNERS and np.all(np.isfinite(tri_xyz)):
                                order = _order_plane_corners(tri_xyz)
                                corners_xyz = tri_xyz[order]
                                debug_points_l = corners_l[order]
                                debug_points_r = corners_r[order]
                                mesh_tri = np.asarray([[0, 1, 2], [0, 2, 3]], dtype=np.int32)
                                mesh_valid = True
                                mean_dy = float(np.mean(np.abs(debug_points_r[:, 1] - debug_points_l[:, 1])))
                                status = f"4-corner mesh OK | mean|dy|={mean_dy:.2f}px"
                            else:
                                status = "4-corner triangulation invalid"

            left_vis = _draw_seg_debug(left, seg_l, debug_points_l, (0, 180, 255))
            right_vis = _draw_seg_debug(right, seg_r, debug_points_r, (0, 180, 255))
            stitched_vis = cv2.hconcat([left_vis, right_vis])

            plot_w = max(460, int(round(stitched_vis.shape[1] * float(PLOT_WIDTH_RATIO))))
            target_plot_size = (stitched_vis.shape[0], plot_w)
            if plotter is None or plotter_size != target_plot_size:
                plotter = _make_plotter(
                    height_px=stitched_vis.shape[0],
                    width_px=plot_w,
                    t_lr_mm=t_lr_mm,
                )
                plotter_size = target_plot_size

            plot_img, pose_debug = plotter(
                corners_xyz if mesh_valid else None,
                mesh_tri if mesh_valid else np.empty((0, 3), dtype=np.int32),
                status,
            )
            if plot_img.shape[0] != stitched_vis.shape[0] or plot_img.shape[1] != plot_w:
                plot_img = cv2.resize(plot_img, (plot_w, stitched_vis.shape[0]), interpolation=cv2.INTER_AREA)

            divider = np.full((stitched_vis.shape[0], DIVIDER_W, 3), 20, dtype=np.uint8)
            combined = cv2.hconcat([stitched_vis, divider, plot_img])

            hud_lines = [
                f"sample={sample_id}/{len(sampled_indices)} frame={int(frame_idx)} conf={float(YOLO_CONFIDENCE):.2f}",
                f"status: {status}",
            ]
            if bool(pose_debug.get("has_plane", False)):
                cam_box = np.asarray(pose_debug.get("camera_origin_box_mm", [np.nan, np.nan, np.nan]), dtype=np.float64)
                hud_lines.append(f"camera in box frame [mm]: x={cam_box[0]:.1f}, y={cam_box[1]:.1f}, z={cam_box[2]:.1f}")
            _draw_hud_block(combined, hud_lines)

            if SHOW_PREVIEW:
                display = combined
                if FIT_PREVIEW_TO_SCREEN:
                    display = _resize_to_fit(combined, max_disp_w, max_disp_h)
                cv2.imshow(WINDOW_NAME, display)
                cv2.resizeWindow(WINDOW_NAME, display.shape[1], display.shape[0])
                wait_ms = 0 if PREVIEW_BLOCKING_MODE else max(1, int(PREVIEW_WAIT_MS))
                key = cv2.waitKey(wait_ms) & 0xFF
                if key in (ord("q"), 27):
                    quit_requested = True

            png_path = run_dir / f"sample_{sample_id:02d}_frame_{int(frame_idx):06d}.png"
            if SAVE_PNG_RESULTS:
                cv2.imwrite(str(png_path), combined)
                saved_images.append(png_path)

            if mesh_valid:
                mesh_ok += 1
            processed += 1

            manifest_frames.append(
                {
                    "sample_id": int(sample_id),
                    "frame_index": int(frame_idx),
                    "time_s": float(frame_time_s),
                    "confidence": float(YOLO_CONFIDENCE),
                    "mesh_valid": bool(mesh_valid),
                    "status": str(status),
                    "mesh_point_count": int(len(corners_xyz)),
                    "mesh_triangle_count": int(len(mesh_tri)),
                    "pose": pose_debug,
                    "png_path": str(png_path) if SAVE_PNG_RESULTS else "",
                }
            )

            print(f"[POST] sample={sample_id}/{len(sampled_indices)} frame={frame_idx} | {status}")
            if quit_requested:
                break
    finally:
        cap.release()
        if SHOW_PREVIEW:
            cv2.destroyAllWindows()

    if SAVE_PNG_RESULTS and len(saved_images) > 1:
        thumbs: list[np.ndarray] = []
        for p in saved_images:
            im = cv2.imread(str(p), cv2.IMREAD_COLOR)
            if im is None:
                continue
            thumbs.append(im)
        if thumbs:
            cell_h = min(im.shape[0] for im in thumbs)
            cell_w = min(im.shape[1] for im in thumbs)
            resized = [cv2.resize(im, (cell_w, cell_h), interpolation=cv2.INTER_AREA) for im in thumbs]
            cols = 2
            rows = int(np.ceil(len(resized) / cols))
            blank = np.zeros((cell_h, cell_w, 3), dtype=np.uint8)
            tiles: list[np.ndarray] = []
            k = 0
            for _ in range(rows):
                row_imgs = []
                for _ in range(cols):
                    row_imgs.append(resized[k] if k < len(resized) else blank)
                    k += 1
                tiles.append(cv2.hconcat(row_imgs))
            contact_sheet = cv2.vconcat(tiles)
            contact_path = run_dir / "samples_contact_sheet.png"
            cv2.imwrite(str(contact_path), contact_sheet)

    manifest_payload: dict[str, object] = {
        "schema_version": 1,
        "input_video_path": str(input_video),
        "output_directory": str(run_dir),
        "generated_at_local": datetime.now().isoformat(timespec="seconds"),
        "source_fps_metadata": float(src_fps_meta),
        "sampled_frame_count_requested": int(SAMPLE_FRAME_COUNT),
        "sampled_frame_count_processed": int(processed),
        "valid_mesh_frame_count": int(mesh_ok),
        "frames": manifest_frames,
    }
    _write_manifest(manifest_path, manifest_payload)

    print(f"[POST] Done. Processed {processed} sampled frame(s). Valid mesh frames: {mesh_ok}")
    print(f"[POST] Output directory: {run_dir}")
    print(f"[POST] Manifest: {manifest_path}")


if __name__ == "__main__":
    main()
