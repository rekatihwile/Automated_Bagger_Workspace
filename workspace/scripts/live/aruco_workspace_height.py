"""
ArUco-guided workspace point-cloud capture.

This version fixes the multi-angle reference-frame problem.

Main idea:
  - ArUco markers define a fixed workspace frame.
  - Every object point cloud is transformed from stereo-camera coordinates
    into that fixed workspace frame before saving.
  - Use the *_workspace.ply files in CloudCompare.

Workspace frame convention:
  Origin = TL marker center
  +X     = TL -> TR
  +Y     = TL -> BL
  +Z     = cross(+X, +Y)
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


# ===================== USER SETTINGS =====================
# Marker IDs assigned to workspace corners in clockwise order:
# [TL_id, TR_id, BR_id, BL_id]
MARKER_IDS_CW: list[int] = [0, 1, 3, 2]

# Common choices:
#   cv2.aruco.DICT_ARUCO_ORIGINAL
#   cv2.aruco.DICT_4X4_50
#   cv2.aruco.DICT_6X6_250
ARUCO_DICT_TYPE: int = cv2.aruco.DICT_ARUCO_ORIGINAL

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
MIN_DISPARITY = 1.0
SCALE_TO_METERS = 0.001
MAX_OBJECT_POINTS = 80_000
VOXEL_SIZE_M = 0.003
# =========================================================


_ROLE_NAMES = ["TL", "TR", "BR", "BL"]
ROLE_BY_ID: dict[int, str] = {mid: name for mid, name in zip(MARKER_IDS_CW, _ROLE_NAMES)}

_BGR_BY_ROLE: dict[str, tuple[int, int, int]] = {
    "TL": (0, 255, 0),
    "TR": (255, 0, 0),
    "BR": (0, 0, 255),
    "BL": (0, 255, 255),
}

WINDOW_LIVE = "ArUco Workspace — SPACE to freeze"
WINDOW_SESSION = "ArUco Workspace Session"


# ---------------------------------------------------------------------------
# Dataclasses
# ---------------------------------------------------------------------------

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
class WorkspaceFrame:
    """
    Rigid transform from camera frame to fixed ArUco workspace frame.

    Convention:
        origin = TL marker center
        +X     = TL -> TR
        +Y     = TL -> BL
        +Z     = cross(+X, +Y)
    """

    origin_cam_m: np.ndarray
    R_cw: np.ndarray

    def camera_to_workspace_points(self, points_cam_m: np.ndarray) -> np.ndarray:
        points_cam_m = np.asarray(points_cam_m, dtype=np.float64)
        return (points_cam_m - self.origin_cam_m[None, :]) @ self.R_cw

    def camera_to_workspace_point(self, point_cam_m: np.ndarray) -> np.ndarray:
        point_cam_m = np.asarray(point_cam_m, dtype=np.float64)
        return (point_cam_m - self.origin_cam_m) @ self.R_cw

    def as_dict(self) -> dict[str, Any]:
        return {
            "origin_cam_m": self.origin_cam_m.tolist(),
            "R_cw": self.R_cw.tolist(),
            "convention": {
                "origin": "TL marker center",
                "+X": "TL -> TR",
                "+Y": "TL -> BL",
                "+Z": "cross(+X,+Y)",
            },
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
# Calibration helpers
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

    missing = [k for k in required if k not in calib]
    if missing:
        raise KeyError("Calibration missing keys:\n" + "\n".join(f"  {m}" for m in missing))

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


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

def triangulate_rectified_point(
    projection_left: np.ndarray,
    projection_right: np.ndarray,
    left_xy: tuple[float, float],
    right_xy: tuple[float, float],
    scale_to_meters: float,
) -> np.ndarray:
    left_pts = np.array([[left_xy[0]], [left_xy[1]]], dtype=np.float64)
    right_pts = np.array([[right_xy[0]], [right_xy[1]]], dtype=np.float64)

    homog = cv2.triangulatePoints(projection_left, projection_right, left_pts, right_pts)

    w = float(homog[3, 0])
    if abs(w) < 1e-12:
        raise ValueError("Cannot triangulate point; homogeneous W is near zero.")

    return (homog[:3, 0] / w) * float(scale_to_meters)


def reproject_disparity_to_points(
    disparity: np.ndarray,
    q_matrix: np.ndarray,
    scale_to_meters: float,
) -> np.ndarray:
    points_raw = cv2.reprojectImageTo3D(disparity.astype(np.float32), q_matrix)
    return points_raw.astype(np.float64) * float(scale_to_meters)


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


def build_workspace_frame_from_corners(corners_m: dict[str, np.ndarray]) -> WorkspaceFrame:
    """
    Build a fixed workspace frame from ArUco corners.

    Requires:
        TL, TR, BL

    Optional:
        BR is useful for sanity checking, but not required to define the basis.
    """
    required = ["TL", "TR", "BL"]
    missing = [r for r in required if r not in corners_m]

    if missing:
        raise ValueError(
            "Cannot build workspace frame. Need TL, TR, and BL visible in both cameras. "
            f"Missing: {missing}"
        )

    TL = np.asarray(corners_m["TL"], dtype=np.float64)
    TR = np.asarray(corners_m["TR"], dtype=np.float64)
    BL = np.asarray(corners_m["BL"], dtype=np.float64)

    x_axis = TR - TL
    x_norm = np.linalg.norm(x_axis)

    if x_norm < 1e-9:
        raise ValueError("Degenerate workspace frame: TL and TR are too close.")

    x_hat = x_axis / x_norm

    y_raw = BL - TL
    y_axis = y_raw - np.dot(y_raw, x_hat) * x_hat
    y_norm = np.linalg.norm(y_axis)

    if y_norm < 1e-9:
        raise ValueError("Degenerate workspace frame: TL, TR, and BL are nearly collinear.")

    y_hat = y_axis / y_norm

    z_hat = np.cross(x_hat, y_hat)
    z_norm = np.linalg.norm(z_hat)

    if z_norm < 1e-9:
        raise ValueError("Degenerate workspace frame: invalid plane normal.")

    z_hat = z_hat / z_norm

    # Recompute Y to make the basis exactly orthonormal.
    y_hat = np.cross(z_hat, x_hat)
    y_hat = y_hat / np.linalg.norm(y_hat)

    # Columns are workspace axes expressed in camera coordinates.
    # For row-vector points:
    #   p_workspace = (p_camera - origin_camera) @ R_cw
    R_cw = np.column_stack([x_hat, y_hat, z_hat])

    return WorkspaceFrame(origin_cam_m=TL, R_cw=R_cw)


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
        keep = radial <= np.percentile(radial, 95)
        points_m = points_m[keep]
        colors_bgr = colors_bgr[keep]

    if len(points_m) >= 20:
        lo = np.percentile(points_m, 2, axis=0)
        hi = np.percentile(points_m, 98, axis=0)
        keep = np.all((points_m >= lo) & (points_m <= hi), axis=1)
        points_m = points_m[keep]
        colors_bgr = colors_bgr[keep]

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

    valid = (
        (mask > 0)
        & np.isfinite(disparity)
        & (disparity > min_disparity)
        & (disparity < float(w))
    )

    points = points_3d_m[valid].reshape(-1, 3)
    colors = left_bgr[valid].reshape(-1, 3)

    points, colors, method = filter_object_cloud(points, colors, voxel_size_m)
    points, colors = downsample_limit(points, colors, max_points)

    return points, colors, method


def compute_object_metrics_workspace(
    object_id: int,
    mask: np.ndarray,
    points_workspace_m: np.ndarray,
    filter_method: str,
) -> dict[str, float | int | list[float] | str]:
    """
    Object metrics in fixed ArUco workspace coordinates.

    In this frame:
      x = TL -> TR
      y = TL -> BL
      z = signed height relative to the ArUco plane
    """
    if len(points_workspace_m) == 0:
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
            "frame": "aruco_workspace",
        }

    centroid = np.mean(points_workspace_m, axis=0)
    median_xyz = np.median(points_workspace_m, axis=0)

    z_vals = points_workspace_m[:, 2]
    p05, p50, p95 = np.percentile(z_vals, [5, 50, 95])

    footprint = np.array([centroid[0], centroid[1], 0.0], dtype=np.float64)

    return {
        "object_id": object_id,
        "mask_area_px": int(np.count_nonzero(mask)),
        "point_count": int(len(points_workspace_m)),
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
        "frame": "aruco_workspace",
    }


# ---------------------------------------------------------------------------
# PLY output
# ---------------------------------------------------------------------------

def save_ply(path: Path, xyz_m: np.ndarray, colors_bgr: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    xyz_m = np.asarray(xyz_m, dtype=np.float64)
    colors_bgr = np.asarray(colors_bgr, dtype=np.uint8)

    rgb = colors_bgr[:, ::-1]

    with path.open("w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(xyz_m)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
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
    """
    Save one PLY containing:
      - a 4-corner workspace floor mesh
      - all saved object clouds

    Everything passed into this function should already be in workspace coordinates.
    """
    path.parent.mkdir(parents=True, exist_ok=True)

    floor = np.asarray(floor_points_m, dtype=np.float64)

    if floor.shape != (4, 3):
        raise ValueError(f"Expected four 3D floor corners, got shape {floor.shape}")

    floor_rgb = np.tile(np.array([[72, 92, 110]], dtype=np.uint8), (4, 1))

    obj_pts: list[np.ndarray] = []
    obj_rgb: list[np.ndarray] = []

    for obj in objects:
        if len(obj.points_m) == 0:
            continue

        obj_pts.append(obj.points_m)
        obj_rgb.append(obj.colors_bgr[:, ::-1])

    if obj_pts:
        pts = np.vstack([floor, *obj_pts])
        rgb = np.vstack([floor_rgb, *obj_rgb])
    else:
        pts = floor
        rgb = floor_rgb

    faces = [(0, 1, 2), (0, 2, 3)]

    with path.open("w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
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


# ---------------------------------------------------------------------------
# ArUco detection
# ---------------------------------------------------------------------------

def _build_detector(dict_type: int) -> cv2.aruco.ArucoDetector:
    dictionary = cv2.aruco.getPredefinedDictionary(dict_type)
    params = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(dictionary, params)


def _detect_aruco(
    detector: cv2.aruco.ArucoDetector,
    image_bgr: np.ndarray,
) -> dict[int, np.ndarray]:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    result: dict[int, np.ndarray] = {}

    if ids is None:
        return result

    for marker_corners, marker_id in zip(corners, ids.flatten()):
        center = marker_corners.reshape(4, 2).mean(axis=0)
        result[int(marker_id)] = center.astype(np.float32)

    return result


def _draw_aruco_overlay(
    image_bgr: np.ndarray,
    detections: dict[int, np.ndarray],
    label_suffix: str = "",
) -> np.ndarray:
    out = image_bgr.copy()

    for mid, center in detections.items():
        role = ROLE_BY_ID.get(mid)
        color = _BGR_BY_ROLE.get(role, (180, 180, 180)) if role else (180, 180, 180)

        cx, cy = int(round(center[0])), int(round(center[1]))

        cv2.circle(out, (cx, cy), 8, color, 2, cv2.LINE_AA)

        label = (f"{role}" if role else f"ID{mid}") + label_suffix

        cv2.putText(
            out,
            label,
            (cx + 10, cy - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )

        cv2.putText(
            out,
            label,
            (cx + 10, cy - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            1,
            cv2.LINE_AA,
        )

    return out


def triangulate_aruco_corners(
    det_left: dict[int, np.ndarray],
    det_right: dict[int, np.ndarray],
    projection_left: np.ndarray,
    projection_right: np.ndarray,
    scale_to_meters: float,
) -> tuple[dict[str, np.ndarray], dict[str, tuple[int, int]], dict[str, tuple[int, int]]]:
    corners_m: dict[str, np.ndarray] = {}
    pixels_left: dict[str, tuple[int, int]] = {}
    pixels_right: dict[str, tuple[int, int]] = {}

    for mid in MARKER_IDS_CW:
        role = ROLE_BY_ID.get(mid)

        if role and mid in det_left and mid in det_right:
            cl = det_left[mid]
            cr = det_right[mid]

            left_xy = (float(cl[0]), float(cl[1]))
            right_xy = (float(cr[0]), float(cr[1]))

            try:
                pt_m = triangulate_rectified_point(
                    projection_left,
                    projection_right,
                    left_xy,
                    right_xy,
                    scale_to_meters,
                )

                corners_m[role] = pt_m
                pixels_left[role] = (int(round(cl[0])), int(round(cl[1])))
                pixels_right[role] = (int(round(cr[0])), int(round(cr[1])))

            except ValueError as exc:
                print(f"[WARN] Triangulation failed for marker {mid} ({role}): {exc}")

    return corners_m, pixels_left, pixels_right


# ---------------------------------------------------------------------------
# Visualization helpers
# ---------------------------------------------------------------------------

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

        width = cv2.getTextSize(
            candidate,
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            thickness,
        )[0][0]

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

    x, y = 16, 28

    cv2.putText(
        panel,
        title,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.64,
        (240, 240, 240),
        2,
        cv2.LINE_AA,
    )

    y += 28
    max_w = max(100, w - 2 * x)

    for line in lines:
        for wrapped in wrap_text(line, max_w, 0.52, 1):
            if y > panel_height - 14:
                break

            cv2.putText(
                panel,
                wrapped,
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (214, 219, 224),
                1,
                cv2.LINE_AA,
            )

            y += 22

        if y > panel_height - 14:
            break

    return np.vstack([image_bgr, panel])


def draw_horizontal_guides(image_bgr: np.ndarray, spacing: int) -> np.ndarray:
    out = image_bgr.copy()
    h, w = out.shape[:2]

    for y in range(spacing, h, spacing):
        cv2.line(out, (0, y), (w - 1, y), (60, 210, 210), 1, cv2.LINE_AA)

    return out


def stereo_preview(
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


def side_by_side(left_bgr: np.ndarray, right_bgr: np.ndarray) -> np.ndarray:
    left = left_bgr.copy()
    right = right_bgr.copy()

    cv2.putText(left, "LEFT", (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(right, "RIGHT", (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)

    divider = np.full((left.shape[0], 8, 3), 28, dtype=np.uint8)

    return np.hstack([left, divider, right])


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

        contours, _ = cv2.findContours(
            mask_bool.astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

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


def draw_aruco_corners_on_image(
    image_bgr: np.ndarray,
    pixels: dict[str, tuple[int, int]],
    corners_m: dict[str, np.ndarray],
) -> np.ndarray:
    out = image_bgr.copy()

    ordered = [r for r in _ROLE_NAMES if r in pixels]

    for role in ordered:
        x, y = pixels[role]
        color = _BGR_BY_ROLE[role]

        cv2.circle(out, (x, y), 9, color, 2, cv2.LINE_AA)
        cv2.circle(out, (x, y), 3, color, -1, cv2.LINE_AA)

        if role in corners_m:
            pt = corners_m[role]
            label = f"{role} ({pt[0]*1000:.0f},{pt[1]*1000:.0f},{pt[2]*1000:.0f})mm"
        else:
            label = role

        cv2.putText(
            out,
            label,
            (x + 11, y - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )

        cv2.putText(
            out,
            label,
            (x + 11, y - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            color,
            1,
            cv2.LINE_AA,
        )

    if all(r in pixels for r in _ROLE_NAMES):
        pts_poly = np.array([pixels[r] for r in _ROLE_NAMES], dtype=np.int32)
        cv2.polylines(out, [pts_poly], isClosed=True, color=(0, 200, 255), thickness=2)

    return out


# ---------------------------------------------------------------------------
# SAM helpers
# ---------------------------------------------------------------------------

def resolve_sam_checkpoint(path: Path, legacy_path: Path) -> Path:
    if path.exists():
        return path

    if legacy_path.exists():
        return legacy_path

    raise FileNotFoundError(f"SAM checkpoint not found:\n  {path}\n  {legacy_path}")


def load_sam_predictor(checkpoint: Path, model_type: str) -> Any:
    try:
        from segment_anything import SamPredictor, sam_model_registry
    except ImportError as exc:
        raise ImportError(
            "Could not import segment_anything. Install it before running this workflow."
        ) from exc

    device = "cuda" if torch.cuda.is_available() else "cpu"

    sam = sam_model_registry[model_type](checkpoint=str(checkpoint))
    sam.to(device=device)

    print(f"[INFO] SAM device: {device}")

    return SamPredictor(sam)


# ---------------------------------------------------------------------------
# Session
# ---------------------------------------------------------------------------

class ArUcoWorkspaceSession:
    """
    Interactive session for SAM object segmentation.

    Important:
      - self.points_3d_m is in camera frame.
      - saved object clouds are transformed into workspace frame.
    """

    def __init__(
        self,
        left_rect: np.ndarray,
        right_rect: np.ndarray,
        disparity: np.ndarray,
        points_3d_m: np.ndarray,
        plane: PlaneModel,
        corners_m: dict[str, np.ndarray],
        pixels_left: dict[str, tuple[int, int]],
        pixels_right: dict[str, tuple[int, int]],
        predictor: Any,
        run_dir: Path,
        args: argparse.Namespace,
        workspace_frame: WorkspaceFrame,
    ) -> None:
        self.left_rect = left_rect
        self.right_rect = right_rect
        self.disparity = disparity
        self.points_3d_m = points_3d_m

        # This plane is kept mainly for metadata/debugging.
        # Workspace-frame z is what matters for saved clouds.
        self.plane = plane

        self.corners_m = corners_m
        self.pixels_left = pixels_left
        self.pixels_right = pixels_right
        self.predictor = predictor
        self.run_dir = run_dir
        self.args = args
        self.workspace_frame = workspace_frame

        self.current = ObjectPrompt()
        self.objects: list[ObjectResult] = []
        self.finished = False

        print("[INFO] Setting SAM image embedding for the frozen LEFT rectified image...")
        self.predictor.set_image(cv2.cvtColor(self.left_rect, cv2.COLOR_BGR2RGB))

    # ------------------------------------------------------------------
    # Mouse
    # ------------------------------------------------------------------

    def mouse(self, event: int, x: int, y: int, flags: int, param: object) -> None:
        h, w = self.left_rect.shape[:2]

        if y < 0 or y >= h or x < 0 or x >= w:
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            self.current.pos_pts.append((x, y))

        elif event == cv2.EVENT_RBUTTONDOWN:
            self.current.neg_pts.append((x, y))

    # ------------------------------------------------------------------
    # SAM
    # ------------------------------------------------------------------

    def draw_box(self) -> None:
        roi = cv2.selectROI(
            "Draw object box — Enter confirm, Esc cancel",
            self.left_rect,
            False,
            False,
        )

        cv2.destroyWindow("Draw object box — Enter confirm, Esc cancel")

        x, y, w, h = roi

        if w > 0 and h > 0:
            self.current.box = (int(x), int(y), int(x + w), int(y + h))
            print(f"[PROMPT] box: {self.current.box}")
        else:
            print("[INFO] Box cancelled.")

    def predict_current(self) -> bool:
        coords = self.current.pos_pts + self.current.neg_pts

        point_coords = np.array(coords, dtype=np.float32) if coords else None

        point_labels = (
            np.array(
                [1] * len(self.current.pos_pts) + [0] * len(self.current.neg_pts),
                dtype=np.int32,
            )
            if coords
            else None
        )

        box_np = (
            np.array(self.current.box, dtype=np.float32)
            if self.current.box is not None
            else None
        )

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

        print(f"[SAM] mask {self.current.mask_idx}, score={scores[self.current.mask_idx]:.4f}")

        return True

    # ------------------------------------------------------------------
    # Object saving
    # ------------------------------------------------------------------

    def save_current_object(self) -> None:
        if self.current.current_mask is None and not self.predict_current():
            return

        mask = self.current.current_mask

        if mask is None:
            return

        points_cam_m, colors_bgr, method = object_cloud_from_mask(
            mask,
            self.disparity,
            self.points_3d_m,
            self.left_rect,
            float(self.args.min_disparity),
            int(self.args.max_points),
            float(self.args.voxel_size_m),
        )

        if len(points_cam_m) == 0:
            print("[WARN] Object mask produced no valid 3D points.")
            return

        # CRITICAL FIX:
        # Convert camera-frame object cloud into fixed ArUco workspace frame.
        points_workspace_m = self.workspace_frame.camera_to_workspace_points(points_cam_m)

        object_id = len(self.objects)

        metrics = compute_object_metrics_workspace(
            object_id,
            mask,
            points_workspace_m,
            method,
        )

        result = ObjectResult(
            object_id=object_id,
            mask=(mask > 0).astype(np.uint8) * 255,
            points_m=points_workspace_m,
            colors_bgr=colors_bgr,
            metrics=metrics,
        )

        self.objects.append(result)

        # Debug file: original camera-frame cloud.
        save_ply(
            self.run_dir / f"object_{object_id:02d}_cloud_camera_debug.ply",
            points_cam_m,
            colors_bgr,
        )

        # Main file: use this one in CloudCompare.
        save_ply(
            self.run_dir / f"object_{object_id:02d}_cloud_workspace.ply",
            points_workspace_m,
            colors_bgr,
        )

        self._save_all_masks_overlay()
        self._save_workspace_scene()
        self.write_summary()
        self.write_metadata()
        self._print_object_result(metrics)

        self.current.clear()

    def _print_object_result(self, metrics: dict[str, float | int | list[float] | str]) -> None:
        z50 = float(cast(float | int, metrics["height_p50_m"]))
        h_est = float(cast(float | int, metrics["height_estimate_m"]))

        print(
            f"[OBJECT] id={metrics['object_id']}  "
            f"pts={metrics['point_count']}  "
            f"z50={z50:.4f} m  "
            f"h_est={h_est:.4f} m"
        )

    # ------------------------------------------------------------------
    # Finish
    # ------------------------------------------------------------------

    def finish(self) -> None:
        if (
            self.current.box
            or self.current.pos_pts
            or self.current.neg_pts
            or self.current.current_mask is not None
        ):
            print("[INFO] Unsaved prompt exists. Press s to save it first, or c to clear.")
            return

        self._save_all_masks_overlay()
        self.write_summary()
        self.write_metadata()
        self._save_combined_cloud()
        self._save_workspace_scene()

        self.finished = True

        print(f"[DONE] Workspace report written to: {self.run_dir}")

    # ------------------------------------------------------------------
    # Save helpers
    # ------------------------------------------------------------------

    def _save_all_masks_overlay(self) -> None:
        cv2.imwrite(
            str(self.run_dir / "all_object_masks_overlay.png"),
            draw_object_overlay(self.left_rect, self.objects),
        )

    def _save_combined_cloud(self) -> None:
        pts_list = [obj.points_m for obj in self.objects if len(obj.points_m) > 0]
        cols_list = [obj.colors_bgr for obj in self.objects if len(obj.colors_bgr) > 0]

        if pts_list:
            save_ply(
                self.run_dir / "combined_objects_cloud.ply",
                np.vstack(pts_list),
                np.vstack(cols_list),
            )

    def _save_workspace_scene(self) -> None:
        corner_order = [r for r in _ROLE_NAMES if r in self.corners_m]

        if len(corner_order) < 4:
            return

        floor_pts_cam = np.vstack([self.corners_m[r] for r in corner_order[:4]])
        floor_pts_workspace = self.workspace_frame.camera_to_workspace_points(floor_pts_cam)

        save_scene_ply(
            self.run_dir / "workspace_floor_and_objects.ply",
            floor_pts_workspace,
            self.objects,
        )

    # ------------------------------------------------------------------
    # CSV / JSON output
    # ------------------------------------------------------------------

    def write_summary(self) -> None:
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

        with (self.run_dir / "summary.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()

            for obj in self.objects:
                writer.writerow({k: obj.metrics[k] for k in fields})

    def write_metadata(self) -> None:
        metadata = {
            "created_at": datetime.now().isoformat(),
            "calibration": str(config.ACTIVE_CALIBRATION_NPZ),
            "raft_checkpoint": str(self.args.raft_checkpoint),
            "sam_model_path": str(self.args.sam_model_path),
            "scale_to_meters": float(self.args.scale_to_meters),
            "aruco_marker_ids": {r: mid for mid, r in ROLE_BY_ID.items()},
            "corners_pixels_left": {r: list(px) for r, px in self.pixels_left.items()},
            "corners_pixels_right": {r: list(px) for r, px in self.pixels_right.items()},
            "corners_camera_m": {r: pt.tolist() for r, pt in self.corners_m.items()},
            "corners_workspace_m": {
                r: self.workspace_frame.camera_to_workspace_point(pt).tolist()
                for r, pt in self.corners_m.items()
            },
            "workspace_frame": self.workspace_frame.as_dict(),
            "plane_camera_frame": self.plane.as_dict(),
            "objects": [
                {
                    "object_id": obj.object_id,
                    "metrics": obj.metrics,
                }
                for obj in self.objects
            ],
            "important_output_files": {
                "use_in_cloudcompare": [
                    "object_XX_cloud_workspace.ply",
                    "combined_objects_cloud.ply",
                    "workspace_floor_and_objects.ply",
                ],
                "debug_only": [
                    "object_XX_cloud_camera_debug.ply",
                ],
            },
        }

        (self.run_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2) + "\n",
            encoding="utf-8",
        )

    # ------------------------------------------------------------------
    # Render
    # ------------------------------------------------------------------

    def render(self) -> np.ndarray:
        left_view = draw_object_overlay(self.left_rect, self.objects, self.current)
        left_view = draw_aruco_corners_on_image(left_view, self.pixels_left, self.corners_m)

        view = side_by_side(left_view, self.right_rect)

        score_str = ""

        if self.current.scores is not None:
            idx = self.current.mask_idx
            score_str = f"  |  mask {idx}, score {self.current.scores[idx]:.3f}"

        n_corners = len(self.corners_m)

        title = (
            f"Plane: {n_corners}/4 ArUco corners  |  "
            f"Objects saved: {len(self.objects)}{score_str}"
        )

        lines = [
            "Object prompts on LEFT image. Saved clouds are transformed into ArUco workspace frame.",
            "b draw box. left/right mouse = SAM positive/negative point.",
            "p preview mask. s save workspace cloud. n new prompt. c clear prompt.",
            "z finish and write combined workspace PLY. q/Esc quit.",
        ]

        return draw_ui_panel(view, title, lines, int(self.args.panel_height))

    # ------------------------------------------------------------------
    # Run loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        h, w = self.left_rect.shape[:2]

        cv2.namedWindow(WINDOW_SESSION, cv2.WINDOW_NORMAL)

        cv2.resizeWindow(
            WINDOW_SESSION,
            int((w * 2 + 8) * float(self.args.display_scale)),
            int((h + self.args.panel_height) * float(self.args.display_scale)),
        )

        cv2.setMouseCallback(WINDOW_SESSION, self.mouse)

        while True:
            cv2.imshow(WINDOW_SESSION, self.render())

            key = cv2.waitKey(20) & 0xFF

            if key in (ord("q"), 27):
                break

            elif key == ord("b"):
                self.draw_box()

            elif key == ord("p"):
                self.predict_current()

            elif key == ord("n"):
                self.current.clear()
                print("[INFO] New object prompt ready.")

            elif key == ord("s"):
                self.save_current_object()

            elif key == ord("c"):
                self.current.clear()
                print("[INFO] Cleared current prompt.")

            elif key == ord("z"):
                self.finish()
                break

        cv2.destroyWindow(WINDOW_SESSION)


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ArUco-guided RAFT+SAM workspace-frame point cloud capture."
    )

    parser.add_argument("--use-raft", action=argparse.BooleanOptionalAction, default=USE_RAFT)
    parser.add_argument("--raft-iters", type=int, default=RAFT_ITERS)
    parser.add_argument("--raft-repo-dir", type=Path, default=RAFT_REPO_DIR)
    parser.add_argument("--raft-checkpoint", type=Path, default=RAFT_CHECKPOINT)

    parser.add_argument(
        "--raft-corr-implementation",
        choices=["reg", "alt", "reg_cuda", "alt_cuda"],
        default=RAFT_CORR_IMPLEMENTATION,
    )

    parser.add_argument(
        "--raft-mixed-precision",
        action=argparse.BooleanOptionalAction,
        default=RAFT_MIXED_PRECISION,
    )

    parser.add_argument("--sam-model-path", type=Path, default=SAM_MODEL_PATH)
    parser.add_argument("--sam-legacy-path", type=Path, default=SAM_MODEL_LEGACY_PATH)
    parser.add_argument(
        "--sam-model-type",
        choices=["vit_b", "vit_l", "vit_h"],
        default=SAM_MODEL_TYPE,
    )

    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    parser.add_argument("--display-scale", type=float, default=DISPLAY_SCALE)
    parser.add_argument("--panel-height", type=int, default=PANEL_HEIGHT)
    parser.add_argument("--guide-spacing", type=int, default=GUIDE_SPACING_PX)
    parser.add_argument("--min-disparity", type=float, default=MIN_DISPARITY)
    parser.add_argument("--scale-to-meters", type=float, default=SCALE_TO_METERS)
    parser.add_argument("--max-points", type=int, default=MAX_OBJECT_POINTS)
    parser.add_argument("--voxel-size-m", type=float, default=VOXEL_SIZE_M)

    return parser.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    args = parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)

    if not args.use_raft:
        raise ValueError("This script requires RAFT disparity. Use --use-raft.")

    calib = load_calibration()

    q_matrix = calib["disparity_to_depth_Q"]
    proj_l = np.asarray(calib["projection_left_rectified"], dtype=np.float64)
    proj_r = np.asarray(calib["projection_right_rectified"], dtype=np.float64)

    detector = _build_detector(ARUCO_DICT_TYPE)

    sam_path = resolve_sam_checkpoint(args.sam_model_path, args.sam_legacy_path)
    args.sam_model_path = sam_path

    predictor = load_sam_predictor(sam_path, args.sam_model_type)

    # ------------------------------------------------------------------
    # Phase 1 — Live rectified preview with ArUco overlay
    # ------------------------------------------------------------------

    print("[INFO] Live preview: position board so all 4 markers are visible, then press SPACE.")

    with StereoCamera() as stereo:
        stereo.warmup()

        ok, left0, right0 = stereo.read_pair()

        if not ok or left0 is None or right0 is None:
            raise RuntimeError("Could not read initial stereo pair from camera.")

        h, w = left0.shape[:2]
        rect_maps = build_rectification_maps(calib, h, w)

        cv2.namedWindow(WINDOW_LIVE, cv2.WINDOW_NORMAL)

        cv2.resizeWindow(
            WINDOW_LIVE,
            int((w * 2 + 8) * args.display_scale),
            int((h + args.panel_height) * args.display_scale),
        )

        left_rect: np.ndarray | None = None
        right_rect: np.ndarray | None = None

        frozen_det_l: dict[int, np.ndarray] = {}
        frozen_det_r: dict[int, np.ndarray] = {}
        capture_armed = False

        while True:
            ok, left, right = stereo.read_pair()

            if not ok or left is None or right is None:
                continue

            lr, rr = rectify_pair(left, right, rect_maps)

            det_l = _detect_aruco(detector, lr)
            det_r = _detect_aruco(detector, rr)

            both = sum(1 for mid in MARKER_IDS_CW if mid in det_l and mid in det_r)
            ready_to_freeze = both == len(MARKER_IDS_CW)

            left_view = _draw_aruco_overlay(lr, det_l)
            right_view = _draw_aruco_overlay(rr, det_r)

            preview = stereo_preview(left_view, right_view, int(args.guide_spacing))

            status_color = (
                (0, 220, 80)
                if both == 4
                else (0, 180, 255)
                if both >= 2
                else (0, 80, 255)
            )

            cv2.putText(
                preview,
                f"Markers visible in both eyes: {both}/4",
                (14, preview.shape[0] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                status_color,
                2,
                cv2.LINE_AA,
            )

            freeze_hint = "SPACE to freeze" if both < 4 else "All 4 found — SPACE to freeze!"

            if capture_armed:
                freeze_hint = "Waiting for all 4 corners..."
            elif ready_to_freeze:
                freeze_hint = "All 4 found - SPACE to freeze!"
            else:
                freeze_hint = "SPACE to arm capture"

            preview = draw_ui_panel(
                preview,
                f"Live stereo — {freeze_hint}",
                [
                    "Position the ArUco board so all 4 corner markers are visible in both views.",
                    "SPACE waits for all 4 corners, then freezes the first complete pair.",
                    "q / Esc quits without saving.",
                ],
                int(args.panel_height),
            )

            cv2.imshow(WINDOW_LIVE, preview)

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                cv2.destroyWindow(WINDOW_LIVE)
                return

            if key == ord(" "):
                capture_armed = True

                if not ready_to_freeze:
                    print(
                        f"[INFO] Capture armed. Waiting for all 4 markers in both eyes "
                        f"(currently {both}/4)."
                    )

            if capture_armed and ready_to_freeze:
                left_rect = lr
                right_rect = rr
                frozen_det_l = det_l
                frozen_det_r = det_r

                print("[INFO] All 4 markers found in both eyes. Freezing pair.")

                cv2.destroyWindow(WINDOW_LIVE)
                break

    if left_rect is None or right_rect is None:
        return

    # ------------------------------------------------------------------
    # Phase 2 — Triangulate ArUco corners and build workspace frame
    # ------------------------------------------------------------------

    print("[INFO] Triangulating ArUco corners from frozen rectified pair...")

    corners_m, pixels_left, pixels_right = triangulate_aruco_corners(
        frozen_det_l,
        frozen_det_r,
        proj_l,
        proj_r,
        float(args.scale_to_meters),
    )

    n_corners = len(corners_m)

    print(f"[ARUCO] Triangulated {n_corners}/4 corners: {list(corners_m.keys())}")

    for role, pt in corners_m.items():
        print(
            f"  {role}: "
            f"X={pt[0]*1000:+8.2f} mm  "
            f"Y={pt[1]*1000:+8.2f} mm  "
            f"Z={pt[2]*1000:+8.2f} mm"
        )

    if n_corners < 3:
        print("[ERROR] Need at least 3 triangulated corners to fit a plane. Aborting.")
        return

    plane_pts = np.vstack([corners_m[r] for r in _ROLE_NAMES if r in corners_m])
    plane = fit_plane_from_points(plane_pts)

    print(f"[PLANE] normal={plane.normal.round(4).tolist()}  offset={plane.offset:.6f} m")

    # CRITICAL FIX:
    # This is the fixed world/workspace frame used by all saved clouds.
    workspace_frame = build_workspace_frame_from_corners(corners_m)

    corners_workspace_m = {
        role: workspace_frame.camera_to_workspace_point(pt)
        for role, pt in corners_m.items()
    }

    print("[WORKSPACE FRAME] ArUco corners in workspace coordinates:")

    for role, pt in corners_workspace_m.items():
        print(
            f"  {role}: "
            f"X={pt[0]*1000:+8.2f} mm  "
            f"Y={pt[1]*1000:+8.2f} mm  "
            f"Z={pt[2]*1000:+8.2f} mm"
        )

    if "BR" in corners_workspace_m:
        br = corners_workspace_m["BR"]
        print(
            "[CHECK] BR should be roughly "
            f"X=workspace width, Y=workspace depth, Z≈0. "
            f"Measured BR = {br.tolist()}"
        )

    # ------------------------------------------------------------------
    # Phase 3 — RAFT disparity and point-cloud reprojection
    # ------------------------------------------------------------------

    run_dir = args.output_root / f"run_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    run_dir.mkdir(parents=True, exist_ok=True)

    cv2.imwrite(str(run_dir / "left_rect.png"), left_rect)
    cv2.imwrite(str(run_dir / "right_rect.png"), right_rect)

    aruco_overlay = side_by_side(
        _draw_aruco_overlay(left_rect, frozen_det_l),
        _draw_aruco_overlay(right_rect, frozen_det_r),
    )

    cv2.imwrite(str(run_dir / "aruco_detection_frozen.png"), aruco_overlay)

    print("[INFO] Running RAFT-Stereo on frozen pair...")

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

    cv2.imwrite(
        str(run_dir / "raft_disparity_color.png"),
        disparity_colormap(disparity, float(args.min_disparity)),
    )

    points_3d_m = reproject_disparity_to_points(
        disparity,
        q_matrix,
        float(args.scale_to_meters),
    )

    # ------------------------------------------------------------------
    # Phase 4 — Interactive SAM object session
    # ------------------------------------------------------------------

    session = ArUcoWorkspaceSession(
        left_rect=left_rect,
        right_rect=right_rect,
        disparity=disparity,
        points_3d_m=points_3d_m,
        plane=plane,
        corners_m=corners_m,
        pixels_left=pixels_left,
        pixels_right=pixels_right,
        predictor=predictor,
        run_dir=run_dir,
        args=args,
        workspace_frame=workspace_frame,
    )

    session.write_metadata()
    session.run()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
