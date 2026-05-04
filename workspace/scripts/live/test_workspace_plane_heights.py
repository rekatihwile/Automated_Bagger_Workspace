"""
Live workspace plane/object height experiment.

Workflow:
  1. Show a live rectified LEFT | RIGHT stereo preview with guide lines.
  2. Press SPACE to freeze one rectified stereo pair.
  3. Compute RAFT-Stereo disparity for the frozen pair.
  4. Click 4 matched tabletop/stool plane corners in LEFT and RIGHT.
  5. Add objects with SAM box/point prompts on the LEFT image.
  6. Save object clouds, a floor mesh, and a CSV height summary relative to the plane.

This script intentionally does not encode final robot geometry. It is a modular
workspace measurement experiment; clicked plane corners can later be replaced
with ArUco/QR marker detection.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, cast

import cv2
import numpy as np
import torch


# ---------------------------------------------------------------------------
# Workspace path setup
# ---------------------------------------------------------------------------

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = WORKSPACE_ROOT.parent
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import config
from camera import StereoCamera
from stereo_raft import compute_raft_disparity


# ================= USER SETTINGS =================
USE_RAFT = True
RAFT_ITERS = 32
RAFT_REPO_DIR = PROJECT_ROOT / "external" / "RAFT-Stereo"
RAFT_CHECKPOINT = WORKSPACE_ROOT / "models" / "raft_stereo" / "raftstereo-middlebury.pth"
RAFT_CORR_IMPLEMENTATION = "alt"
RAFT_MIXED_PRECISION = True

SAM_MODEL_PATH = WORKSPACE_ROOT / "models" / "sam" / "sam_vit_b_01ec64.pth"
SAM_MODEL_LEGACY_PATH = WORKSPACE_ROOT / "models" / "sam_vit_b_01ec64.pth"
SAM_MODEL_TYPE = "vit_b"

OUTPUT_ROOT = config.CALIBRATION_ROOT / "workspace_pose_tests"
DISPLAY_SCALE = 1.0
PANEL_HEIGHT = 142
GUIDE_SPACING_PX = 40
PLANE_SAMPLE_RADIUS = 3
MIN_DISPARITY = 1.0
SCALE_TO_METERS = 0.001
MAX_OBJECT_POINTS = 80_000
VOXEL_SIZE_M = 0.003
# =================================================


WINDOW_LIVE = "Workspace Height Live - SPACE freeze"
WINDOW_SESSION = "Workspace Height Session"


@dataclass
class PlaneModel:
    normal: np.ndarray
    offset: float
    points_m: np.ndarray

    def flipped(self) -> "PlaneModel":
        return PlaneModel(
            normal=-self.normal,
            offset=-self.offset,
            points_m=self.points_m.copy(),
        )

    def as_dict(self) -> dict[str, Any]:
        return {
            "normal": self.normal.tolist(),
            "offset": float(self.offset),
            "points_m": self.points_m.tolist(),
        }


@dataclass
class ObjectPrompt:
    box: tuple[int, int, int, int] | None = None
    pos_pts: list[tuple[int, int]] = field(default_factory=list)
    neg_pts: list[tuple[int, int]] = field(default_factory=list)
    masks: np.ndarray | None = None
    scores: np.ndarray | None = None
    mask_idx: int = 0

    @property
    def current_mask(self) -> np.ndarray | None:
        if self.masks is None:
            return None
        return self.masks[self.mask_idx]

    def clear(self) -> None:
        self.box = None
        self.pos_pts.clear()
        self.neg_pts.clear()
        self.masks = None
        self.scores = None
        self.mask_idx = 0


@dataclass
class ObjectResult:
    object_id: int
    mask: np.ndarray
    points_m: np.ndarray
    colors_bgr: np.ndarray
    metrics: dict[str, float | int | list[float] | str]


# ---------------------------------------------------------------------------
# Calibration and geometry
# ---------------------------------------------------------------------------

def load_calibration() -> dict[str, np.ndarray]:
    calib = config.load_stereo_calibration()
    if calib is None:
        raise FileNotFoundError(f"No calibration found at: {config.ACTIVE_CALIBRATION_NPZ}")

    required = [
        "left_camera_matrix",
        "left_distortion_coefficients",
        "right_camera_matrix",
        "right_distortion_coefficients",
        "rectification_left",
        "rectification_right",
        "projection_left_rectified",
        "projection_right_rectified",
        "disparity_to_depth_Q",
    ]
    missing = [name for name in required if name not in calib]
    if missing:
        raise KeyError("Calibration is missing keys:\n" + "\n".join(f"  {m}" for m in missing))
    return calib


def build_rectification_maps(
    calib: dict[str, np.ndarray],
    h: int,
    w: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    map_lx, map_ly = cv2.initUndistortRectifyMap(
        calib["left_camera_matrix"],
        calib["left_distortion_coefficients"],
        calib["rectification_left"],
        calib["projection_left_rectified"],
        (w, h),
        cv2.CV_32FC1,
    )
    map_rx, map_ry = cv2.initUndistortRectifyMap(
        calib["right_camera_matrix"],
        calib["right_distortion_coefficients"],
        calib["rectification_right"],
        calib["projection_right_rectified"],
        (w, h),
        cv2.CV_32FC1,
    )
    return map_lx, map_ly, map_rx, map_ry


def rectify_pair(
    left_bgr: np.ndarray,
    right_bgr: np.ndarray,
    maps: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    map_lx, map_ly, map_rx, map_ry = maps
    left_rect = cv2.remap(left_bgr, map_lx, map_ly, cv2.INTER_LINEAR)
    right_rect = cv2.remap(right_bgr, map_rx, map_ry, cv2.INTER_LINEAR)
    return left_rect, right_rect


def reproject_disparity_to_points(
    disparity: np.ndarray,
    q_matrix: np.ndarray,
    scale_to_meters: float,
) -> np.ndarray:
    points_raw = cv2.reprojectImageTo3D(disparity.astype(np.float32), q_matrix)
    return points_raw.astype(np.float64) * float(scale_to_meters)


def reproject_single_pixel(
    q_matrix: np.ndarray,
    x: float,
    y: float,
    disparity: float,
    scale_to_meters: float,
) -> np.ndarray:
    homog = q_matrix @ np.array([x, y, disparity, 1.0], dtype=np.float64)
    if abs(float(homog[3])) < 1e-12:
        raise ValueError("Cannot reproject point; homogeneous W is near zero.")
    return (homog[:3] / homog[3]) * float(scale_to_meters)


def triangulate_rectified_point(
    projection_left: np.ndarray,
    projection_right: np.ndarray,
    left_xy: tuple[int, int],
    right_xy: tuple[int, int],
    scale_to_meters: float,
) -> np.ndarray:
    left_pts = np.array([[left_xy[0]], [left_xy[1]]], dtype=np.float64)
    right_pts = np.array([[right_xy[0]], [right_xy[1]]], dtype=np.float64)
    homog = cv2.triangulatePoints(projection_left, projection_right, left_pts, right_pts)
    w = float(homog[3, 0])
    if abs(w) < 1e-12:
        raise ValueError("Cannot triangulate point; homogeneous W is near zero.")
    return (homog[:3, 0] / w) * float(scale_to_meters)


def sample_disparity_at(
    disparity: np.ndarray,
    x: int,
    y: int,
    radius: int,
    min_disparity: float,
) -> float | None:
    h, w = disparity.shape
    x0 = max(0, x - radius)
    x1 = min(w, x + radius + 1)
    y0 = max(0, y - radius)
    y1 = min(h, y + radius + 1)
    patch = disparity[y0:y1, x0:x1]
    valid = patch[np.isfinite(patch) & (patch > min_disparity) & (patch < float(w))]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def fit_plane_from_points(points_m: np.ndarray) -> PlaneModel:
    if points_m.shape[0] < 3:
        raise ValueError("Need at least 3 points to fit a plane.")
    center = np.mean(points_m, axis=0)
    _, _, vh = np.linalg.svd(points_m - center)
    normal = vh[-1]
    norm = np.linalg.norm(normal)
    if norm < 1e-12:
        raise ValueError("Plane fit failed; points are degenerate.")
    normal = normal / norm
    offset = -float(np.dot(normal, center))
    return PlaneModel(normal=normal, offset=offset, points_m=points_m.copy())


def distance_points_to_plane(points_m: np.ndarray, plane: PlaneModel) -> np.ndarray:
    return points_m @ plane.normal + plane.offset


def project_points_to_plane(points_m: np.ndarray, plane: PlaneModel) -> np.ndarray:
    distances = distance_points_to_plane(points_m, plane)[:, None]
    return points_m - distances * plane.normal[None, :]


# ---------------------------------------------------------------------------
# Point cloud processing
# ---------------------------------------------------------------------------

def filter_object_cloud(
    points_m: np.ndarray,
    colors_bgr: np.ndarray,
    voxel_size_m: float,
) -> tuple[np.ndarray, np.ndarray, str]:
    finite = np.all(np.isfinite(points_m), axis=1)
    points_m = points_m[finite]
    colors_bgr = colors_bgr[finite]
    if len(points_m) == 0:
        return points_m, colors_bgr, "empty"

    try:
        import open3d as o3d  # type: ignore

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_m)
        pcd.colors = o3d.utility.Vector3dVector(colors_bgr[:, ::-1].astype(np.float64) / 255.0)
        if len(points_m) >= 30:
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        if voxel_size_m > 0:
            pcd = pcd.voxel_down_sample(voxel_size=voxel_size_m)
        filtered_pts = np.asarray(pcd.points, dtype=np.float64)
        filtered_rgb = (np.asarray(pcd.colors) * 255.0).clip(0, 255).astype(np.uint8)
        return filtered_pts, filtered_rgb[:, ::-1].copy(), "open3d"
    except Exception:
        pass

    median = np.median(points_m, axis=0)
    radial = np.linalg.norm(points_m - median, axis=1)
    if len(points_m) >= 20:
        keep_radial = radial <= np.percentile(radial, 95)
        points_m = points_m[keep_radial]
        colors_bgr = colors_bgr[keep_radial]

    if len(points_m) >= 20:
        lo = np.percentile(points_m, 2, axis=0)
        hi = np.percentile(points_m, 98, axis=0)
        keep_box = np.all((points_m >= lo) & (points_m <= hi), axis=1)
        points_m = points_m[keep_box]
        colors_bgr = colors_bgr[keep_box]

    return points_m, colors_bgr, "percentile"


def downsample_limit(
    points_m: np.ndarray,
    colors_bgr: np.ndarray,
    max_points: int,
) -> tuple[np.ndarray, np.ndarray]:
    if max_points <= 0 or len(points_m) <= max_points:
        return points_m, colors_bgr
    chosen = np.random.choice(len(points_m), int(max_points), replace=False)
    return points_m[chosen], colors_bgr[chosen]


def save_ply(path: Path, xyz_m: np.ndarray, colors_bgr: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rgb = colors_bgr[:, ::-1]
    with path.open("w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(xyz_m)}\n")
        for prop in ("x", "y", "z"):
            f.write(f"property float {prop}\n")
        for prop in ("red", "green", "blue"):
            f.write(f"property uchar {prop}\n")
        f.write("end_header\n")
        for p, c in zip(xyz_m, rgb):
            f.write(
                f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                f"{int(c[0])} {int(c[1])} {int(c[2])}\n"
            )


def save_scene_ply(
    path: Path,
    floor_points_m: np.ndarray,
    objects: list[ObjectResult],
) -> None:
    """Save one PLY containing a quadrilateral floor mesh plus object points."""
    path.parent.mkdir(parents=True, exist_ok=True)

    floor = np.asarray(floor_points_m, dtype=np.float64)
    if floor.shape != (4, 3):
        raise ValueError(f"Expected four 3D floor corners, got shape {floor.shape}")

    floor_rgb = np.tile(np.array([[72, 92, 110]], dtype=np.uint8), (4, 1))
    object_points: list[np.ndarray] = []
    object_rgb: list[np.ndarray] = []
    for obj in objects:
        if len(obj.points_m) == 0:
            continue
        object_points.append(obj.points_m)
        object_rgb.append(obj.colors_bgr[:, ::-1])

    if object_points:
        pts = np.vstack([floor, *object_points])
        rgb = np.vstack([floor_rgb, *object_rgb])
    else:
        pts = floor
        rgb = floor_rgb

    faces = [(0, 1, 2), (0, 2, 3)]

    with path.open("w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        for prop in ("x", "y", "z"):
            f.write(f"property float {prop}\n")
        for prop in ("red", "green", "blue"):
            f.write(f"property uchar {prop}\n")
        f.write(f"element face {len(faces)}\n")
        f.write("property list uchar int vertex_indices\n")
        f.write("end_header\n")
        for p, c in zip(pts, rgb):
            f.write(
                f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                f"{int(c[0])} {int(c[1])} {int(c[2])}\n"
            )
        for a, b, c in faces:
            f.write(f"3 {a} {b} {c}\n")


def object_cloud_from_mask(
    mask: np.ndarray,
    disparity: np.ndarray,
    points_3d_m: np.ndarray,
    left_bgr: np.ndarray,
    min_disparity: float,
    max_points: int,
    voxel_size_m: float,
) -> tuple[np.ndarray, np.ndarray, str]:
    h, w = disparity.shape
    valid = (mask > 0) & np.isfinite(disparity) & (disparity > min_disparity) & (disparity < float(w))
    points = points_3d_m[valid].reshape(-1, 3)
    colors = left_bgr[valid].reshape(-1, 3)
    points, colors, method = filter_object_cloud(points, colors, voxel_size_m)
    points, colors = downsample_limit(points, colors, max_points)
    return points, colors, method


def compute_object_metrics(
    object_id: int,
    mask: np.ndarray,
    points_m: np.ndarray,
    plane: PlaneModel,
    filter_method: str,
) -> dict[str, float | int | list[float] | str]:
    if len(points_m) == 0:
        return {
            "object_id": object_id,
            "mask_area_px": int(np.count_nonzero(mask)),
            "point_count": 0,
            "centroid_x_m": float("nan"),
            "centroid_y_m": float("nan"),
            "centroid_z_m": float("nan"),
            "height_p05_m": float("nan"),
            "height_p50_m": float("nan"),
            "height_p95_m": float("nan"),
            "height_estimate_m": float("nan"),
            "median_xyz_m": [float("nan")] * 3,
            "footprint_centroid_m": [float("nan")] * 3,
            "filter_method": filter_method,
        }

    distances = distance_points_to_plane(points_m, plane)
    centroid = np.mean(points_m, axis=0)
    median_xyz = np.median(points_m, axis=0)
    p05, p50, p95 = np.percentile(distances, [5, 50, 95])
    footprint = np.mean(project_points_to_plane(points_m, plane), axis=0)
    return {
        "object_id": object_id,
        "mask_area_px": int(np.count_nonzero(mask)),
        "point_count": int(len(points_m)),
        "centroid_x_m": float(centroid[0]),
        "centroid_y_m": float(centroid[1]),
        "centroid_z_m": float(centroid[2]),
        "height_p05_m": float(p05),
        "height_p50_m": float(p50),
        "height_p95_m": float(p95),
        "height_estimate_m": float(p95 - p05),
        "median_xyz_m": median_xyz.tolist(),
        "footprint_centroid_m": footprint.tolist(),
        "filter_method": filter_method,
    }


# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------

def draw_horizontal_guides(image_bgr: np.ndarray, spacing: int) -> np.ndarray:
    out = image_bgr.copy()
    h, w = out.shape[:2]
    for y in range(spacing, h, spacing):
        cv2.line(out, (0, y), (w - 1, y), (60, 210, 210), 1, cv2.LINE_AA)
    return out


def disparity_colormap(disparity: np.ndarray, min_disparity: float) -> np.ndarray:
    valid = np.isfinite(disparity) & (disparity > min_disparity)
    if np.any(valid):
        lo = float(np.percentile(disparity[valid], 2))
        hi = float(np.percentile(disparity[valid], 98))
        norm = np.clip((disparity - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    else:
        norm = np.zeros_like(disparity, dtype=np.float32)
    u8 = (norm * 255).astype(np.uint8)
    color = cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)
    color[~valid] = (20, 20, 20)
    return color


def wrap_text(text: str, max_width_px: int, font_scale: float, thickness: int) -> list[str]:
    words = text.split()
    lines: list[str] = []
    current = ""
    for word in words:
        candidate = word if not current else f"{current} {word}"
        width = cv2.getTextSize(candidate, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0][0]
        if width <= max_width_px or not current:
            current = candidate
        else:
            lines.append(current)
            current = word
    if current:
        lines.append(current)
    return lines


def draw_ui_panel(
    image_bgr: np.ndarray,
    title: str,
    lines: list[str],
    panel_height: int = PANEL_HEIGHT,
) -> np.ndarray:
    h, w = image_bgr.shape[:2]
    panel = np.full((panel_height, w, 3), (24, 26, 30), dtype=np.uint8)
    cv2.rectangle(panel, (0, 0), (w - 1, panel_height - 1), (70, 75, 82), 1)

    x = 16
    y = 28
    cv2.putText(panel, title, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.64, (240, 240, 240), 2, cv2.LINE_AA)
    y += 28
    max_text_width = max(100, w - 2 * x)
    for line in lines:
        for wrapped in wrap_text(line, max_text_width, 0.52, 1):
            if y > panel_height - 14:
                break
            cv2.putText(panel, wrapped, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (214, 219, 224), 1, cv2.LINE_AA)
            y += 22
        if y > panel_height - 14:
            break

    return np.vstack([image_bgr, panel])


def stereo_preview_image(
    left_bgr: np.ndarray,
    right_bgr: np.ndarray,
    guide_spacing: int,
) -> np.ndarray:
    left = draw_horizontal_guides(left_bgr, guide_spacing)
    right = draw_horizontal_guides(right_bgr, guide_spacing)
    cv2.putText(left, "LEFT", (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(right, "RIGHT", (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
    divider = np.full((left.shape[0], 8, 3), 28, dtype=np.uint8)
    return np.hstack([left, divider, right])


def side_by_side_image(left_bgr: np.ndarray, right_bgr: np.ndarray) -> np.ndarray:
    left = left_bgr.copy()
    right = right_bgr.copy()
    cv2.putText(left, "LEFT", (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(right, "RIGHT", (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
    divider = np.full((left.shape[0], 8, 3), 28, dtype=np.uint8)
    return np.hstack([left, divider, right])


def draw_plane_overlay(left_bgr: np.ndarray, clicks: list[tuple[int, int]]) -> np.ndarray:
    out = left_bgr.copy()
    for i, (x, y) in enumerate(clicks):
        cv2.circle(out, (x, y), 7, (0, 220, 255), -1)
        cv2.putText(out, str(i + 1), (x + 9, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, str(i + 1), (x + 9, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)
    if len(clicks) >= 2:
        pts = np.array(clicks, dtype=np.int32)
        cv2.polylines(out, [pts], isClosed=len(clicks) >= 4, color=(0, 220, 255), thickness=2)
    return out


def draw_stereo_plane_overlay(
    left_bgr: np.ndarray,
    right_bgr: np.ndarray,
    left_clicks: list[tuple[int, int]],
    right_clicks: list[tuple[int, int]],
    pending_left: tuple[int, int] | None,
    pending_right: tuple[int, int] | None,
) -> np.ndarray:
    left = draw_plane_overlay(left_bgr, left_clicks)
    right = draw_plane_overlay(right_bgr, right_clicks)

    next_idx = len(left_clicks) + 1
    for image, pending in ((left, pending_left), (right, pending_right)):
        if pending is None:
            continue
        x, y = pending
        cv2.circle(image, (x, y), 7, (80, 255, 80), -1)
        cv2.putText(image, str(next_idx), (x + 9, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, str(next_idx), (x + 9, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)

    return side_by_side_image(left, right)


def draw_object_overlay(
    left_bgr: np.ndarray,
    objects: list[ObjectResult],
    current: ObjectPrompt | None = None,
) -> np.ndarray:
    out = left_bgr.copy()
    palette = [
        (0, 220, 120),
        (255, 170, 0),
        (80, 140, 255),
        (220, 90, 220),
        (80, 220, 255),
        (160, 220, 80),
    ]
    for obj in objects:
        color = palette[obj.object_id % len(palette)]
        mask_bool = obj.mask > 0
        layer = np.zeros_like(out)
        layer[mask_bool] = color
        out = cv2.addWeighted(out, 0.78, layer, 0.22, 0)
        contours, _ = cv2.findContours(mask_bool.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(out, contours, -1, color, 2)

    if current is not None:
        if current.current_mask is not None:
            mask_bool = current.current_mask > 0
            layer = np.zeros_like(out)
            layer[mask_bool] = (0, 255, 0)
            out = cv2.addWeighted(out, 0.72, layer, 0.28, 0)
        if current.box is not None:
            x1, y1, x2, y2 = current.box
            cv2.rectangle(out, (x1, y1), (x2, y2), (255, 190, 0), 2)
        for x, y in current.pos_pts:
            cv2.circle(out, (x, y), 5, (0, 255, 0), -1)
        for x, y in current.neg_pts:
            cv2.circle(out, (x, y), 5, (0, 0, 255), -1)
    return out


def save_ui_preview(output_root: Path) -> Path:
    output_root.mkdir(parents=True, exist_ok=True)
    h, w = 480, 640
    left = np.full((h, w, 3), (54, 58, 64), dtype=np.uint8)
    right = np.full((h, w, 3), (50, 54, 60), dtype=np.uint8)
    cv2.rectangle(left, (80, 110), (560, 365), (82, 90, 98), -1)
    cv2.rectangle(right, (64, 112), (540, 366), (82, 90, 98), -1)
    cv2.rectangle(left, (170, 175), (315, 320), (80, 150, 230), -1)
    cv2.rectangle(left, (360, 150), (490, 345), (120, 205, 110), -1)
    overlay = draw_stereo_plane_overlay(
        left,
        right,
        [(92, 368), (555, 362), (524, 116), (108, 112)],
        [(68, 368), (530, 362), (500, 116), (84, 112)],
        None,
        None,
    )
    prompt = ObjectPrompt(box=(170, 175, 315, 320), pos_pts=[(220, 235)], neg_pts=[(300, 190)])
    object_overlay = side_by_side_image(draw_object_overlay(left, [], prompt), right)
    ui = draw_ui_panel(
        object_overlay if prompt.box is not None else overlay,
        "Matched plane clicks | Object box ready",
        [
            "Click each floor corner in LEFT and RIGHT to triangulate the floor plane.",
            "Objects: b box on LEFT, left/right click refine, p preview mask, s save object.",
            "z writes workspace_floor_and_objects.ply with the floor mesh and object clouds.",
        ],
    )
    path = output_root / "ui_preview.png"
    cv2.imwrite(str(path), ui)
    return path


# ---------------------------------------------------------------------------
# SAM and session
# ---------------------------------------------------------------------------

def resolve_sam_checkpoint(path: Path, legacy_path: Path) -> Path:
    if path.exists():
        return path
    if legacy_path.exists():
        return legacy_path
    raise FileNotFoundError(
        f"SAM checkpoint not found:\n  {path}\n  {legacy_path}"
    )


def load_sam_predictor(checkpoint: Path, model_type: str):
    try:
        from segment_anything import SamPredictor, sam_model_registry
    except ImportError as exc:
        raise ImportError(
            "Could not import segment_anything. Install it before running the live SAM workflow."
        ) from exc

    device = "cuda" if torch.cuda.is_available() else "cpu"
    sam = sam_model_registry[model_type](checkpoint=str(checkpoint))
    sam.to(device=device)
    print(f"[INFO] SAM device: {device}")
    return SamPredictor(sam)


class WorkspacePlaneHeightSession:
    def __init__(
        self,
        left_rect: np.ndarray,
        right_rect: np.ndarray,
        disparity: np.ndarray,
        points_3d_m: np.ndarray,
        projection_left: np.ndarray,
        projection_right: np.ndarray,
        q_matrix: np.ndarray,
        predictor: Any,
        run_dir: Path,
        args: argparse.Namespace,
    ) -> None:
        self.left_rect = left_rect
        self.right_rect = right_rect
        self.disparity = disparity
        self.points_3d_m = points_3d_m
        self.projection_left = projection_left
        self.projection_right = projection_right
        self.q_matrix = q_matrix
        self.predictor = predictor
        self.run_dir = run_dir
        self.args = args

        self.stage = "plane"
        self.plane_left_clicks: list[tuple[int, int]] = []
        self.plane_right_clicks: list[tuple[int, int]] = []
        self.pending_plane_left: tuple[int, int] | None = None
        self.pending_plane_right: tuple[int, int] | None = None
        self.plane_points_m: list[np.ndarray] = []
        self.plane: PlaneModel | None = None
        self.current = ObjectPrompt()
        self.objects: list[ObjectResult] = []
        self.finished = False

        print("[INFO] Setting SAM image embedding for the frozen LEFT rectified image...")
        self.predictor.set_image(cv2.cvtColor(self.left_rect, cv2.COLOR_BGR2RGB))

    def mouse(self, event: int, x: int, y: int, flags: int, param: object) -> None:
        h, w = self.left_rect.shape[:2]
        if y < 0 or y >= h:
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            if self.stage == "plane":
                self.add_plane_click(x, y)
            elif 0 <= x < w:
                self.current.pos_pts.append((x, y))
        elif event == cv2.EVENT_RBUTTONDOWN and self.stage == "object":
            if 0 <= x < w:
                self.current.neg_pts.append((x, y))

    def add_plane_click(self, x: int, y: int) -> None:
        if len(self.plane_points_m) >= 4:
            print("[INFO] Plane already has 4 clicks. Press c to reset plane if needed.")
            return

        h, w = self.left_rect.shape[:2]
        divider = 8
        if 0 <= x < w:
            self.pending_plane_left = (x, y)
            print(f"[PLANE] pending corner {len(self.plane_points_m) + 1}/4 LEFT  at ({x}, {y})")
        elif w + divider <= x < w + divider + w:
            xr = x - w - divider
            self.pending_plane_right = (xr, y)
            print(f"[PLANE] pending corner {len(self.plane_points_m) + 1}/4 RIGHT at ({xr}, {y})")
        else:
            return

        if self.pending_plane_left is None or self.pending_plane_right is None:
            return

        left_xy = self.pending_plane_left
        right_xy = self.pending_plane_right
        try:
            point_m = triangulate_rectified_point(
                self.projection_left,
                self.projection_right,
                left_xy,
                right_xy,
                float(self.args.scale_to_meters),
            )
        except ValueError as exc:
            print(f"[WARN] Could not triangulate corner: {exc}")
            return

        self.plane_left_clicks.append(left_xy)
        self.plane_right_clicks.append(right_xy)
        self.plane_points_m.append(point_m)
        self.pending_plane_left = None
        self.pending_plane_right = None
        print(
            f"[PLANE] saved corner {len(self.plane_points_m)}/4 "
            f"L={left_xy} R={right_xy} XYZ={point_m.round(4).tolist()} m"
        )

        if len(self.plane_points_m) == 4:
            self.plane = fit_plane_from_points(np.vstack(self.plane_points_m))
            self.stage = "object"
            self.save_plane_overlay()
            self.save_workspace_scene_cloud()
            self.write_metadata()
            print("[PLANE] Plane fit complete. Add an object with b, refine clicks, then s.")

    def clear_current(self) -> None:
        if self.stage == "plane":
            self.plane_left_clicks.clear()
            self.plane_right_clicks.clear()
            self.pending_plane_left = None
            self.pending_plane_right = None
            self.plane_points_m.clear()
            self.plane = None
            print("[INFO] Cleared plane clicks.")
        else:
            self.current.clear()
            print("[INFO] Cleared current object prompt.")

    def draw_box(self) -> None:
        if self.stage != "object":
            print("[WARN] Define the plane first.")
            return
        roi = cv2.selectROI("Draw object box - Enter confirm, Esc cancel", self.left_rect, False, False)
        cv2.destroyWindow("Draw object box - Enter confirm, Esc cancel")
        x, y, w, h = roi
        if w > 0 and h > 0:
            self.current.box = (int(x), int(y), int(x + w), int(y + h))
            print(f"[PROMPT] object box: {self.current.box}")
        else:
            print("[INFO] Box canceled.")

    def predict_current(self) -> bool:
        if self.stage != "object":
            print("[WARN] Define the plane first.")
            return False
        coords = self.current.pos_pts + self.current.neg_pts
        point_coords = np.array(coords, dtype=np.float32) if coords else None
        point_labels = (
            np.array([1] * len(self.current.pos_pts) + [0] * len(self.current.neg_pts), dtype=np.int32)
            if coords
            else None
        )
        box_np = np.array(self.current.box, dtype=np.float32) if self.current.box is not None else None
        if point_coords is None and box_np is None:
            print("[WARN] Draw a box or add point prompts before segmenting.")
            return False

        masks_raw, scores_raw, _ = self.predictor.predict(
            point_coords=point_coords,
            point_labels=point_labels,
            box=box_np,
            multimask_output=True,
        )
        masks = cast(np.ndarray, masks_raw)
        scores = cast(np.ndarray, scores_raw)
        self.current.masks = masks
        self.current.scores = scores
        self.current.mask_idx = int(np.argmax(scores))
        print(f"[SAM] selected mask {self.current.mask_idx}, score={scores[self.current.mask_idx]:.4f}")
        return True

    def save_current_object(self) -> None:
        if self.plane is None:
            print("[WARN] Plane is not defined yet.")
            return
        if self.current.current_mask is None and not self.predict_current():
            return
        mask = self.current.current_mask
        if mask is None:
            return

        points_m, colors_bgr, method = object_cloud_from_mask(
            mask,
            self.disparity,
            self.points_3d_m,
            self.left_rect,
            float(self.args.min_disparity),
            int(self.args.max_points),
            float(self.args.voxel_size_m),
        )
        if len(points_m) == 0:
            print("[WARN] Object mask produced no valid 3D points.")
            return

        distances = distance_points_to_plane(points_m, self.plane)
        if float(np.median(distances)) < 0:
            self.plane = self.plane.flipped()
            print("[INFO] Plane normal flipped so saved object heights are positive.")

        object_id = len(self.objects)
        metrics = compute_object_metrics(object_id, mask, points_m, self.plane, method)
        result = ObjectResult(
            object_id=object_id,
            mask=(mask > 0).astype(np.uint8) * 255,
            points_m=points_m,
            colors_bgr=colors_bgr,
            metrics=metrics,
        )
        self.objects.append(result)
        save_ply(self.run_dir / f"object_{object_id:02d}_cloud.ply", points_m, colors_bgr)
        self.save_all_masks_overlay()
        self.save_workspace_scene_cloud()
        self.write_summary()
        self.write_metadata()
        self.print_object_result(metrics)
        self.current.clear()

    def print_object_result(self, metrics: dict[str, float | int | list[float] | str]) -> None:
        height_p50 = float(cast(float | int, metrics["height_p50_m"]))
        height_estimate = float(cast(float | int, metrics["height_estimate_m"]))
        print(
            "[OBJECT] "
            f"id={metrics['object_id']} points={metrics['point_count']} "
            f"h50={height_p50:.4f}m "
            f"h_est={height_estimate:.4f}m"
        )

    def finish(self) -> None:
        if self.current.box is not None or self.current.pos_pts or self.current.neg_pts or self.current.current_mask is not None:
            print("[INFO] Unsaved current prompt exists. Press s to save it, or z again after c to finish without it.")
            return
        self.save_all_masks_overlay()
        self.write_summary()
        self.write_metadata()
        self.save_combined_cloud()
        self.save_workspace_scene_cloud()
        self.finished = True
        print(f"[DONE] Wrote workspace height report to: {self.run_dir}")

    def save_plane_overlay(self) -> None:
        overlay = draw_stereo_plane_overlay(
            self.left_rect,
            self.right_rect,
            self.plane_left_clicks,
            self.plane_right_clicks,
            self.pending_plane_left,
            self.pending_plane_right,
        )
        cv2.imwrite(str(self.run_dir / "plane_clicks_overlay.png"), overlay)

    def save_all_masks_overlay(self) -> None:
        cv2.imwrite(str(self.run_dir / "all_object_masks_overlay.png"), draw_object_overlay(self.left_rect, self.objects))

    def save_combined_cloud(self) -> None:
        if not self.objects:
            return
        pts = np.vstack([obj.points_m for obj in self.objects if len(obj.points_m) > 0])
        cols = np.vstack([obj.colors_bgr for obj in self.objects if len(obj.colors_bgr) > 0])
        if len(pts) > 0:
            save_ply(self.run_dir / "combined_objects_cloud.ply", pts, cols)

    def save_workspace_scene_cloud(self) -> None:
        if self.plane is None or len(self.plane_points_m) != 4:
            return
        floor_points = project_points_to_plane(np.vstack(self.plane_points_m), self.plane)
        save_scene_ply(self.run_dir / "workspace_floor_and_objects.ply", floor_points, self.objects)

    def write_summary(self) -> None:
        summary_path = self.run_dir / "summary.csv"
        fields = [
            "object_id",
            "mask_area_px",
            "point_count",
            "centroid_x_m",
            "centroid_y_m",
            "centroid_z_m",
            "height_p05_m",
            "height_p50_m",
            "height_p95_m",
            "height_estimate_m",
        ]
        with summary_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for obj in self.objects:
                writer.writerow({key: obj.metrics[key] for key in fields})

    def write_metadata(self) -> None:
        metadata = {
            "created_at": datetime.now().isoformat(),
            "calibration": str(config.ACTIVE_CALIBRATION_NPZ),
            "raft_checkpoint": str(self.args.raft_checkpoint),
            "sam_model_path": str(self.args.sam_model_path),
            "scale_to_meters": float(self.args.scale_to_meters),
            "plane_clicks_left_px": [list(p) for p in self.plane_left_clicks],
            "plane_clicks_right_px": [list(p) for p in self.plane_right_clicks],
            "plane_corner_points_m": [p.tolist() for p in self.plane_points_m],
            "plane": self.plane.as_dict() if self.plane is not None else None,
            "objects": [
                {
                    "object_id": obj.object_id,
                    "metrics": obj.metrics,
                }
                for obj in self.objects
            ],
        }
        (self.run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    def render(self) -> np.ndarray:
        if self.stage == "plane":
            view = draw_stereo_plane_overlay(
                self.left_rect,
                self.right_rect,
                self.plane_left_clicks,
                self.plane_right_clicks,
                self.pending_plane_left,
                self.pending_plane_right,
            )
            title = f"Plane reference: {len(self.plane_points_m)}/4 matched corners"
            lines = [
                "For each workspace corner, click LEFT and the matching RIGHT point.",
                "Repeat 4 corners in order around the floor. These points triangulate the plane.",
                "c clears plane clicks. q/Esc quits.",
            ]
        else:
            left_view = draw_object_overlay(self.left_rect, self.objects, self.current)
            view = side_by_side_image(left_view, self.right_rect)
            score = ""
            if self.current.scores is not None:
                score = f" | mask {self.current.mask_idx}, score {self.current.scores[self.current.mask_idx]:.3f}"
            title = f"Objects saved: {len(self.objects)}{score}"
            lines = [
                "Object prompts are on the LEFT image only. Save object 1, then object 2.",
                "b draw box. Left/right click add positive/negative SAM refinement points.",
                "p previews/refines the current mask. s saves current object cloud and metrics.",
                "n starts a new object prompt. c clears current prompt. z writes floor+objects PLY.",
            ]
        return draw_ui_panel(view, title, lines, int(self.args.panel_height))

    def run(self) -> None:
        h, w = self.left_rect.shape[:2]
        cv2.namedWindow(WINDOW_SESSION, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_SESSION, int((w * 2 + 8) * float(self.args.display_scale)), int((h + self.args.panel_height) * float(self.args.display_scale)))
        cv2.setMouseCallback(WINDOW_SESSION, self.mouse)
        while True:
            cv2.imshow(WINDOW_SESSION, self.render())
            key = cv2.waitKey(20) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("b"):
                self.draw_box()
            elif key == ord("p"):
                self.predict_current()
            elif key == ord("n"):
                self.current.clear()
                print("[INFO] New object prompt ready.")
            elif key == ord("s"):
                self.save_current_object()
            elif key == ord("c"):
                self.clear_current()
            elif key == ord("z"):
                self.finish()
                break
        cv2.destroyWindow(WINDOW_SESSION)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def create_run_dir(output_root: Path) -> Path:
    run_dir = output_root / f"run_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def run_ui_preview(args: argparse.Namespace) -> None:
    path = save_ui_preview(args.output_root)
    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img is not None:
        cv2.namedWindow("Workspace UI Preview", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Workspace UI Preview", 900, 700)
        cv2.imshow("Workspace UI Preview", img)
        cv2.waitKey(1)
        print(f"[SAVE] UI preview: {path}")
        print("[INFO] Preview window opened. Press any key in the image window to close.")
        cv2.waitKey(0)
        cv2.destroyWindow("Workspace UI Preview")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Live RAFT/SAM workspace plane and object height experiment."
    )
    parser.add_argument("--ui-preview-only", action="store_true", help="Show and save a dummy UI preview without opening the camera.")
    parser.add_argument("--use-raft", action=argparse.BooleanOptionalAction, default=USE_RAFT)
    parser.add_argument("--raft-iters", type=int, default=RAFT_ITERS)
    parser.add_argument("--raft-repo-dir", type=Path, default=RAFT_REPO_DIR)
    parser.add_argument("--raft-checkpoint", type=Path, default=RAFT_CHECKPOINT)
    parser.add_argument("--raft-corr-implementation", choices=["reg", "alt", "reg_cuda", "alt_cuda"], default=RAFT_CORR_IMPLEMENTATION)
    parser.add_argument("--raft-mixed-precision", action=argparse.BooleanOptionalAction, default=RAFT_MIXED_PRECISION)
    parser.add_argument("--sam-model-path", type=Path, default=SAM_MODEL_PATH)
    parser.add_argument("--sam-legacy-path", type=Path, default=SAM_MODEL_LEGACY_PATH)
    parser.add_argument("--sam-model-type", choices=["vit_b", "vit_l", "vit_h"], default=SAM_MODEL_TYPE)
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    parser.add_argument("--display-scale", type=float, default=DISPLAY_SCALE)
    parser.add_argument("--panel-height", type=int, default=PANEL_HEIGHT)
    parser.add_argument("--guide-spacing", type=int, default=GUIDE_SPACING_PX)
    parser.add_argument("--plane-sample-radius", type=int, default=PLANE_SAMPLE_RADIUS)
    parser.add_argument("--min-disparity", type=float, default=MIN_DISPARITY)
    parser.add_argument("--scale-to-meters", type=float, default=SCALE_TO_METERS)
    parser.add_argument("--max-points", type=int, default=MAX_OBJECT_POINTS)
    parser.add_argument("--voxel-size-m", type=float, default=VOXEL_SIZE_M)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)

    if args.ui_preview_only:
        run_ui_preview(args)
        return

    if not args.use_raft:
        raise ValueError("This workspace-height experiment currently requires RAFT disparity.")

    sam_path = resolve_sam_checkpoint(args.sam_model_path, args.sam_legacy_path)
    args.sam_model_path = sam_path
    predictor = load_sam_predictor(sam_path, args.sam_model_type)

    calib = load_calibration()
    q_matrix = calib["disparity_to_depth_Q"]

    with StereoCamera() as stereo:
        stereo.require_stereo_shape()
        stereo.warmup()
        ok, left0, right0 = stereo.read_pair()
        if not ok or left0 is None or right0 is None:
            raise RuntimeError("Could not read an initial stereo pair from the camera.")

        h, w = left0.shape[:2]
        rect_maps = build_rectification_maps(calib, h, w)
        cv2.namedWindow(WINDOW_LIVE, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_LIVE, int((w * 2 + 8) * args.display_scale), int((h + args.panel_height) * args.display_scale))

        print("[INFO] Live rectified stereo preview active. Press SPACE to freeze, q/Esc to quit.")
        while True:
            ok, left, right = stereo.read_pair()
            if not ok or left is None or right is None:
                continue
            left_rect, right_rect = rectify_pair(left, right, rect_maps)
            preview = stereo_preview_image(left_rect, right_rect, int(args.guide_spacing))
            preview = draw_ui_panel(
                preview,
                "Live rectified stereo preview",
                ["SPACE freezes a pair and computes RAFT disparity.", "q/Esc quits without saving a run."],
                int(args.panel_height),
            )
            cv2.imshow(WINDOW_LIVE, preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                cv2.destroyWindow(WINDOW_LIVE)
                return
            if key == ord(" "):
                cv2.destroyWindow(WINDOW_LIVE)
                break

    run_dir = create_run_dir(args.output_root)
    cv2.imwrite(str(run_dir / "left_rect.png"), left_rect)
    cv2.imwrite(str(run_dir / "right_rect.png"), right_rect)

    print("[INFO] Computing RAFT disparity for frozen rectified pair...")
    disparity = compute_raft_disparity(
        left_rect,
        right_rect,
        raft_repo_dir=args.raft_repo_dir,
        checkpoint=args.raft_checkpoint,
        iters=int(args.raft_iters),
        corr_implementation=args.raft_corr_implementation,
        mixed_precision=bool(args.raft_mixed_precision),
    )
    np.save(run_dir / "raft_disparity.npy", disparity)
    cv2.imwrite(str(run_dir / "raft_disparity_color.png"), disparity_colormap(disparity, float(args.min_disparity)))
    points_3d_m = reproject_disparity_to_points(disparity, q_matrix, float(args.scale_to_meters))

    session = WorkspacePlaneHeightSession(
        left_rect=left_rect,
        right_rect=right_rect,
        disparity=disparity,
        points_3d_m=points_3d_m,
        projection_left=calib["projection_left_rectified"],
        projection_right=calib["projection_right_rectified"],
        q_matrix=q_matrix,
        predictor=predictor,
        run_dir=run_dir,
        args=args,
    )
    session.write_metadata()
    session.run()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
