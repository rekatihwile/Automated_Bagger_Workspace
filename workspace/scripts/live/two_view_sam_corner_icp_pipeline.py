"""
One-pass two-view ArUco + SAM + RAFT + corner-warp + limited-ICP pipeline.

Save this as:
    workspace/scripts/offline/two_view_sam_corner_icp_pipeline.py

Run:
    python .\two_view_sam_corner_icp_pipeline.py

What it does:
    1. Opens stereo cameras once.
    2. Captures view 0 when all 4 markers are visible.
    3. Keeps cameras open.
    4. Captures view 1 from a different angle when all 4 markers are visible.
    5. Runs RAFT for both frozen stereo pairs.
    6. Opens SAM for view 0. Segment object_00, object_01, object_02 in order.
    7. Opens SAM for view 1. Segment same objects in same order.
    8. Saves only useful merged outputs:
        - workspace_plane_mesh.ply
        - object_XX_merged_corners_only.ply
        - object_XX_merged_limited_icp.ply
        - all_objects_merged_corners_only.ply
        - all_objects_merged_limited_icp.ply
        - scene_corners_only_with_plane.ply
        - scene_limited_icp_with_plane.ply
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, cast

import cv2
import numpy as np
import torch

# =============================================================================
# PROJECT IMPORT SETUP
# =============================================================================

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = WORKSPACE_ROOT.parent

if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import config
from camera import StereoCamera
from stereo_raft import compute_raft_disparity

# =============================================================================
# CONFIG
# =============================================================================

ACTIVE_TUNER = True  # Whether to launch the interactive merge tuner after saving the one-pass output/cache.

MARKER_IDS_CW: list[int] = [0, 1, 3, 2]  # TL, TR, BR, BL
ARUCO_DICT_TYPE: int = cv2.aruco.DICT_ARUCO_ORIGINAL

NUM_VIEWS = 2
TARGET_OBJECT_COUNT = 6

RAFT_ITERS = 32
RAFT_REPO_DIR = PROJECT_ROOT / "external" / "RAFT-Stereo"
RAFT_CHECKPOINT = WORKSPACE_ROOT / "models" / "raft_stereo" / "raftstereo-middlebury.pth"
RAFT_CORR_IMPLEMENTATION = "alt"
RAFT_MIXED_PRECISION = True

SAM_MODEL_PATH = WORKSPACE_ROOT / "models" / "sam" / "sam_vit_b_01ec64.pth"
SAM_MODEL_LEGACY_PATH = WORKSPACE_ROOT / "models" / "sam_vit_b_01ec64.pth"
SAM_MODEL_TYPE = "vit_b"

OUTPUT_ROOT = config.CALIBRATION_ROOT / "one_pass_two_view_reconstruction"

DISPLAY_SCALE = 1.0
PANEL_HEIGHT = 150
GUIDE_SPACING_PX = 40

MIN_DISPARITY = 1.0
SCALE_TO_METERS = 0.001
MAX_OBJECT_POINTS = 100_000
VOXEL_SIZE_OBJECT_M = 0.0025

WARP_MODE = "similarity_3d"  # "homography_plane" or "similarity_3d"
Z_SCALE_MODE = "average_xy"     # "average_xy" or "none"
GLOBAL_FINAL_SHIFT_M = np.array([0.0, 0.0, 0.0], dtype=np.float64)

RUN_LIMITED_ICP = True
ICP_ONLY_TRANSLATION = False
LIMIT_ICP_Z_ROTATION = True
MAX_ICP_Z_ROTATION_DEG = 5.0
ICP_DISTANCE_THRESHOLD = 0.006
ICP_VOXEL_SIZE = 0.003
ICP_METHOD = "point_to_plane"  # "point_to_point" or "point_to_plane"
ALLOW_SCALE_IN_ICP = False

REJECT_BAD_ICP = True
MIN_ACCEPTABLE_ICP_FITNESS = 0.03
MAX_ACCEPTABLE_ICP_RMSE = 0.05
MAX_ICP_TRANSLATION_M = 0.04
MAX_ICP_TOTAL_ROTATION_DEG = 15.0

VOXEL_SIZE_FINAL_PER_OBJECT = 0.0015
VOXEL_SIZE_FINAL_ALL_OBJECTS = 0.0015
FINAL_OUTLIER_NB_NEIGHBORS = 25
FINAL_OUTLIER_STD_RATIO = 2.0

TUNED_Z_SCALE = 1.0  # Applied to source z-axis before final shift; overridden by loaded ICP config.

USE_ICP_MERGE_CONFIG = True   # If True, load ICP merge config from file when available.
PREFER_BEST_ICP_CONFIG = True  # If True, prefer config_best/ over latest auto/ config.
ICP_MERGE_CONFIG_ROOT = OUTPUT_ROOT / "ICP_merge_config"
ICP_MERGE_CONFIG_AUTO_DIR = ICP_MERGE_CONFIG_ROOT / "auto"
ICP_MERGE_CONFIG_BEST_PATH = ICP_MERGE_CONFIG_ROOT / "config_best" / "tuned_params_debug.json"

SAVE_CAPTURE_IMAGES = True
SAVE_MASK_OVERLAYS = True
SAVE_DEBUG_JSON = True

_ROLE_NAMES = ["TL", "TR", "BR", "BL"]
ROLE_BY_ID: dict[int, str] = {mid: name for mid, name in zip(MARKER_IDS_CW, _ROLE_NAMES)}
_BGR_BY_ROLE = {"TL": (0, 255, 0), "TR": (255, 0, 0), "BR": (0, 0, 255), "BL": (0, 255, 255)}
WINDOW_LIVE = "Two-view ArUco capture"
WINDOW_SESSION = "SAM object segmentation"

# =============================================================================
# DATA TYPES
# =============================================================================

@dataclass
class PlaneModel:
    normal: np.ndarray
    offset: float
    points_m: np.ndarray

    def as_dict(self) -> dict[str, Any]:
        return {"normal": self.normal.tolist(), "offset": float(self.offset), "points_m": self.points_m.tolist()}


@dataclass
class WorkspaceFrame:
    origin_cam_m: np.ndarray
    R_cw: np.ndarray

    def camera_to_workspace_points(self, points_cam_m: np.ndarray) -> np.ndarray:
        return (np.asarray(points_cam_m, dtype=np.float64) - self.origin_cam_m[None, :]) @ self.R_cw

    def camera_to_workspace_point(self, point_cam_m: np.ndarray) -> np.ndarray:
        return (np.asarray(point_cam_m, dtype=np.float64) - self.origin_cam_m) @ self.R_cw

    def as_dict(self) -> dict[str, Any]:
        return {"origin_cam_m": self.origin_cam_m.tolist(), "R_cw": self.R_cw.tolist()}


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
class ObjectCloud:
    object_id: int
    mask: np.ndarray
    points_cam_m: np.ndarray
    colors_bgr: np.ndarray
    points_workspace_m: np.ndarray


@dataclass
class ScanData:
    view_id: int
    left_rect: np.ndarray
    right_rect: np.ndarray
    det_left: dict[int, np.ndarray]
    det_right: dict[int, np.ndarray]
    corners_cam_m: dict[str, np.ndarray]
    pixels_left: dict[str, tuple[int, int]]
    pixels_right: dict[str, tuple[int, int]]
    plane_cam: PlaneModel
    workspace_frame: WorkspaceFrame
    corners_workspace_m: dict[str, np.ndarray]
    disparity: np.ndarray | None = None
    points_3d_cam_m: np.ndarray | None = None
    objects: list[ObjectCloud] = field(default_factory=list)

# =============================================================================
# CALIBRATION / GEOMETRY
# =============================================================================

def load_calibration() -> dict[str, np.ndarray]:
    calib = config.load_stereo_calibration()
    if calib is None:
        raise FileNotFoundError(f"No calibration found at: {config.ACTIVE_CALIBRATION_NPZ}")
    required = [
        "left_camera_matrix", "left_distortion_coefficients",
        "right_camera_matrix", "right_distortion_coefficients",
        "rectification_left", "rectification_right",
        "projection_left_rectified", "projection_right_rectified",
        "disparity_to_depth_Q",
    ]
    missing = [k for k in required if k not in calib]
    if missing:
        raise KeyError("Calibration missing keys:\n" + "\n".join(f"  {m}" for m in missing))
    return calib


def build_rectification_maps(calib: dict[str, np.ndarray], h: int, w: int):
    map_lx, map_ly = cv2.initUndistortRectifyMap(
        calib["left_camera_matrix"], calib["left_distortion_coefficients"],
        calib["rectification_left"], calib["projection_left_rectified"], (w, h), cv2.CV_32FC1)
    map_rx, map_ry = cv2.initUndistortRectifyMap(
        calib["right_camera_matrix"], calib["right_distortion_coefficients"],
        calib["rectification_right"], calib["projection_right_rectified"], (w, h), cv2.CV_32FC1)
    return map_lx, map_ly, map_rx, map_ry


def rectify_pair(left_bgr: np.ndarray, right_bgr: np.ndarray, maps):
    map_lx, map_ly, map_rx, map_ry = maps
    return (
        cv2.remap(left_bgr, map_lx, map_ly, cv2.INTER_LINEAR),
        cv2.remap(right_bgr, map_rx, map_ry, cv2.INTER_LINEAR),
    )


def _build_detector(dict_type: int) -> cv2.aruco.ArucoDetector:
    dictionary = cv2.aruco.getPredefinedDictionary(dict_type)
    params = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(dictionary, params)


def _detect_aruco(detector: cv2.aruco.ArucoDetector, image_bgr: np.ndarray) -> dict[int, np.ndarray]:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    result: dict[int, np.ndarray] = {}
    if ids is None:
        return result
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        center = marker_corners.reshape(4, 2).mean(axis=0)
        result[int(marker_id)] = center.astype(np.float32)
    return result


def triangulate_rectified_point(P_l, P_r, left_xy, right_xy, scale_to_meters: float) -> np.ndarray:
    left_pts = np.array([[left_xy[0]], [left_xy[1]]], dtype=np.float64)
    right_pts = np.array([[right_xy[0]], [right_xy[1]]], dtype=np.float64)
    homog = cv2.triangulatePoints(P_l, P_r, left_pts, right_pts)
    w = float(homog[3, 0])
    if abs(w) < 1e-12:
        raise ValueError("Cannot triangulate point; homogeneous W is near zero.")
    return (homog[:3, 0] / w) * float(scale_to_meters)


def triangulate_aruco_corners(det_left, det_right, P_l, P_r):
    corners_m: dict[str, np.ndarray] = {}
    pixels_left: dict[str, tuple[int, int]] = {}
    pixels_right: dict[str, tuple[int, int]] = {}
    for mid in MARKER_IDS_CW:
        role = ROLE_BY_ID.get(mid)
        if role and mid in det_left and mid in det_right:
            cl, cr = det_left[mid], det_right[mid]
            pt_m = triangulate_rectified_point(P_l, P_r, (float(cl[0]), float(cl[1])), (float(cr[0]), float(cr[1])), SCALE_TO_METERS)
            corners_m[role] = pt_m
            pixels_left[role] = (int(round(cl[0])), int(round(cl[1])))
            pixels_right[role] = (int(round(cr[0])), int(round(cr[1])))
    return corners_m, pixels_left, pixels_right


def fit_plane_from_points(points_m: np.ndarray) -> PlaneModel:
    center = np.mean(points_m, axis=0)
    _, _, vh = np.linalg.svd(points_m - center)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    offset = -float(np.dot(normal, center))
    return PlaneModel(normal=normal, offset=offset, points_m=points_m.copy())


def build_workspace_frame_from_corners(corners_m: dict[str, np.ndarray]) -> WorkspaceFrame:
    TL, TR, BL = np.asarray(corners_m["TL"], float), np.asarray(corners_m["TR"], float), np.asarray(corners_m["BL"], float)
    x_hat = (TR - TL) / np.linalg.norm(TR - TL)
    y_raw = BL - TL
    y_axis = y_raw - np.dot(y_raw, x_hat) * x_hat
    y_hat = y_axis / np.linalg.norm(y_axis)
    z_hat = np.cross(x_hat, y_hat)
    z_hat = z_hat / np.linalg.norm(z_hat)
    y_hat = np.cross(z_hat, x_hat)
    y_hat = y_hat / np.linalg.norm(y_hat)
    return WorkspaceFrame(origin_cam_m=TL, R_cw=np.column_stack([x_hat, y_hat, z_hat]))


def reproject_disparity_to_points(disparity: np.ndarray, q_matrix: np.ndarray) -> np.ndarray:
    points_raw = cv2.reprojectImageTo3D(disparity.astype(np.float32), q_matrix)
    return points_raw.astype(np.float64) * SCALE_TO_METERS

# =============================================================================
# UI HELPERS
# =============================================================================

def _draw_aruco_overlay(image_bgr: np.ndarray, detections: dict[int, np.ndarray]) -> np.ndarray:
    out = image_bgr.copy()
    for mid, center in detections.items():
        role = ROLE_BY_ID.get(mid)
        color = _BGR_BY_ROLE.get(role, (180, 180, 180)) if role else (180, 180, 180)
        cx, cy = int(round(center[0])), int(round(center[1]))
        cv2.circle(out, (cx, cy), 8, color, 2, cv2.LINE_AA)
        label = role if role else f"ID{mid}"
        cv2.putText(out, label, (cx + 10, cy - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, label, (cx + 10, cy - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
    return out


def draw_horizontal_guides(image_bgr: np.ndarray, spacing: int) -> np.ndarray:
    out = image_bgr.copy()
    h, w = out.shape[:2]
    for y in range(spacing, h, spacing):
        cv2.line(out, (0, y), (w - 1, y), (60, 210, 210), 1, cv2.LINE_AA)
    return out


def side_by_side(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    divider = np.full((left.shape[0], 8, 3), 28, dtype=np.uint8)
    return np.hstack([left, divider, right])


def stereo_preview(left_bgr: np.ndarray, right_bgr: np.ndarray) -> np.ndarray:
    left = draw_horizontal_guides(left_bgr, GUIDE_SPACING_PX)
    right = draw_horizontal_guides(right_bgr, GUIDE_SPACING_PX)
    cv2.putText(left, "LEFT", (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(right, "RIGHT", (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
    return side_by_side(left, right)


def draw_ui_panel(image_bgr: np.ndarray, title: str, lines: list[str], panel_height: int = PANEL_HEIGHT) -> np.ndarray:
    h, w = image_bgr.shape[:2]
    panel = np.full((panel_height, w, 3), (24, 26, 30), dtype=np.uint8)
    cv2.rectangle(panel, (0, 0), (w - 1, panel_height - 1), (70, 75, 82), 1)
    x, y = 16, 28
    cv2.putText(panel, title, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.64, (240, 240, 240), 2, cv2.LINE_AA)
    y += 28
    for line in lines:
        cv2.putText(panel, line[:120], (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (214, 219, 224), 1, cv2.LINE_AA)
        y += 22
        if y > panel_height - 12:
            break
    return np.vstack([image_bgr, panel])


def draw_object_overlay(left_bgr: np.ndarray, objects: list[ObjectCloud], current: ObjectPrompt | None = None) -> np.ndarray:
    out = left_bgr.copy()
    palette = [(0, 220, 120), (255, 170, 0), (80, 140, 255), (220, 90, 220), (80, 220, 255)]
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
            cv2.rectangle(out, current.box[:2], current.box[2:], (255, 190, 0), 2)
        for x, y in current.pos_pts:
            cv2.circle(out, (x, y), 5, (0, 255, 0), -1)
        for x, y in current.neg_pts:
            cv2.circle(out, (x, y), 5, (0, 0, 255), -1)
    return out

# =============================================================================
# POINT CLOUD / PLY
# =============================================================================

def object_cloud_from_mask(mask: np.ndarray, disparity: np.ndarray, points_3d_m: np.ndarray, left_bgr: np.ndarray):
    h, w = disparity.shape
    valid = (mask > 0) & np.isfinite(disparity) & (disparity > MIN_DISPARITY) & (disparity < float(w))
    points = points_3d_m[valid].reshape(-1, 3)
    colors = left_bgr[valid].reshape(-1, 3)
    finite = np.all(np.isfinite(points), axis=1)
    points, colors = points[finite], colors[finite]
    try:
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors[:, ::-1].astype(np.float64) / 255.0)
        if len(points) >= 30:
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        if VOXEL_SIZE_OBJECT_M > 0:
            pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE_OBJECT_M)
        points = np.asarray(pcd.points, dtype=np.float64)
        colors_rgb = (np.asarray(pcd.colors) * 255.0).clip(0, 255).astype(np.uint8)
        colors = colors_rgb[:, ::-1].copy()
    except Exception:
        pass
    if MAX_OBJECT_POINTS > 0 and len(points) > MAX_OBJECT_POINTS:
        chosen = np.random.choice(len(points), int(MAX_OBJECT_POINTS), replace=False)
        points, colors = points[chosen], colors[chosen]
    return points, colors


def np_to_o3d_cloud(points_xyz: np.ndarray, colors_bgr: np.ndarray | None = None):
    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points_xyz, dtype=np.float64))
    if colors_bgr is not None and len(colors_bgr) == len(points_xyz):
        pcd.colors = o3d.utility.Vector3dVector(colors_bgr[:, ::-1].astype(np.float64) / 255.0)
    return pcd


def remove_outliers_pcd(pcd, nb_neighbors: int, std_ratio: float):
    if len(pcd.points) < nb_neighbors:
        return pcd
    clean, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return clean


def downsample_pcd(pcd, voxel_size: float):
    if voxel_size <= 0:
        return pcd
    return pcd.voxel_down_sample(voxel_size)


def write_scene_with_floor_ply(path: Path, floor_corners_xyz: np.ndarray, object_pcds: list[Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    floor = np.asarray(floor_corners_xyz, dtype=np.float64)
    floor_rgb = np.tile(np.array([[90, 110, 130]], dtype=np.uint8), (4, 1))
    xyz_parts = [floor]
    rgb_parts = [floor_rgb]
    for pcd in object_pcds:
        pts = np.asarray(pcd.points, dtype=np.float64)
        if pcd.has_colors():
            rgb = (np.asarray(pcd.colors) * 255).clip(0, 255).astype(np.uint8)
        else:
            rgb = np.tile(np.array([[230, 180, 30]], dtype=np.uint8), (len(pts), 1))
        xyz_parts.append(pts)
        rgb_parts.append(rgb)
    xyz_all = np.vstack(xyz_parts)
    rgb_all = np.vstack(rgb_parts)
    faces = [(0, 1, 2), (0, 2, 3)]
    with path.open("w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(xyz_all)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write(f"element face {len(faces)}\n")
        f.write("property list uchar int vertex_indices\nend_header\n")
        for p, c in zip(xyz_all, rgb_all):
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {int(c[0])} {int(c[1])} {int(c[2])}\n")
        for a, b, c in faces:
            f.write(f"3 {a} {b} {c}\n")

# =============================================================================
# SAM
# =============================================================================

def resolve_sam_checkpoint(path: Path, legacy_path: Path) -> Path:
    if path.exists():
        return path
    if legacy_path.exists():
        return legacy_path
    raise FileNotFoundError(f"SAM checkpoint not found:\n  {path}\n  {legacy_path}")


def load_sam_predictor(checkpoint: Path, model_type: str) -> Any:
    from segment_anything import SamPredictor, sam_model_registry
    device = "cuda" if torch.cuda.is_available() else "cpu"
    sam = sam_model_registry[model_type](checkpoint=str(checkpoint))
    sam.to(device=device)
    print(f"[INFO] SAM device: {device}")
    return SamPredictor(sam)


class SamObjectSession:
    def __init__(self, scan: ScanData, predictor: Any, target_object_count: int) -> None:
        self.scan = scan
        self.predictor = predictor
        self.target_object_count = target_object_count
        self.current = ObjectPrompt()
        print(f"[INFO] Setting SAM image embedding for view {scan.view_id}...")
        self.predictor.set_image(cv2.cvtColor(scan.left_rect, cv2.COLOR_BGR2RGB))

    def mouse(self, event: int, x: int, y: int, flags: int, param: object) -> None:
        h, w = self.scan.left_rect.shape[:2]
        if not (0 <= x < w and 0 <= y < h):
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            self.current.pos_pts.append((x, y))
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.current.neg_pts.append((x, y))

    def draw_box(self) -> None:
        roi = cv2.selectROI("Draw object box — Enter confirm, Esc cancel", self.scan.left_rect, False, False)
        cv2.destroyWindow("Draw object box — Enter confirm, Esc cancel")
        x, y, w, h = roi
        if w > 0 and h > 0:
            self.current.box = (int(x), int(y), int(x + w), int(y + h))
            print(f"[PROMPT] box: {self.current.box}")

    def predict_current(self) -> bool:
        coords = self.current.pos_pts + self.current.neg_pts
        point_coords = np.array(coords, dtype=np.float32) if coords else None
        point_labels = np.array([1] * len(self.current.pos_pts) + [0] * len(self.current.neg_pts), dtype=np.int32) if coords else None
        box_np = np.array(self.current.box, dtype=np.float32) if self.current.box is not None else None
        if point_coords is None and box_np is None:
            print("[WARN] Draw a box or add point prompts before segmenting.")
            return False
        masks_raw, scores_raw, _ = self.predictor.predict(point_coords=point_coords, point_labels=point_labels, box=box_np, multimask_output=True)
        masks = cast(np.ndarray, masks_raw)
        scores = cast(np.ndarray, scores_raw)
        self.current.masks = masks
        self.current.scores = scores
        self.current.mask_idx = int(np.argmax(scores))
        print(f"[SAM] view={self.scan.view_id} mask={self.current.mask_idx}, score={scores[self.current.mask_idx]:.4f}")
        return True

    def save_current_object(self) -> None:
        if self.current.current_mask is None and not self.predict_current():
            return
        mask = self.current.current_mask
        if mask is None:
            return
        if self.scan.disparity is None or self.scan.points_3d_cam_m is None:
            raise RuntimeError("Scan has no disparity/points yet.")
        points_cam_m, colors_bgr = object_cloud_from_mask(mask, self.scan.disparity, self.scan.points_3d_cam_m, self.scan.left_rect)
        if len(points_cam_m) == 0:
            print("[WARN] Object mask produced no valid 3D points.")
            return
        object_id = len(self.scan.objects)
        points_workspace_m = self.scan.workspace_frame.camera_to_workspace_points(points_cam_m)
        self.scan.objects.append(ObjectCloud(object_id, (mask > 0).astype(np.uint8) * 255, points_cam_m, colors_bgr, points_workspace_m))
        self.current.clear()
        print(f"[OBJECT SAVED] view={self.scan.view_id} object_{object_id:02d} pts={len(points_cam_m)}")

    def undo_last_object(self) -> None:
        if not self.scan.objects:
            print("[WARN] No saved objects to undo.")
            return
        removed = self.scan.objects.pop()
        print(f"[UNDO] Removed object_{removed.object_id:02d} from view {self.scan.view_id}")

    def render(self) -> np.ndarray:
        left_view = draw_object_overlay(self.scan.left_rect, self.scan.objects, self.current)
        view = side_by_side(left_view, self.scan.right_rect)
        title = f"View {self.scan.view_id}: saved {len(self.scan.objects)}/{self.target_object_count} objects"
        lines = [
            "Segment objects in the SAME ORDER for both views: object_00, object_01, object_02.",
            "b box | left/right click pos/neg | p preview | s save | u undo | n new | c clear | z finish view",
        ]
        return draw_ui_panel(view, title, lines, PANEL_HEIGHT)

    def run(self) -> None:
        h, w = self.scan.left_rect.shape[:2]
        window_name = f"{WINDOW_SESSION} — view {self.scan.view_id}"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, int((w * 2 + 8) * DISPLAY_SCALE), int((h + PANEL_HEIGHT) * DISPLAY_SCALE))
        cv2.setMouseCallback(window_name, self.mouse)
        while True:
            cv2.imshow(window_name, self.render())
            key = cv2.waitKey(20) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord("b"):
                self.draw_box()
            elif key == ord("p"):
                self.predict_current()
            elif key == ord("n"):
                self.current.clear()
            elif key == ord("s"):
                self.save_current_object()
            elif key == ord("u"):
                self.undo_last_object()
            elif key == ord("c"):
                self.current.clear()
            elif key == ord("z"):
                if len(self.scan.objects) < self.target_object_count:
                    print(f"[WARN] Saved {len(self.scan.objects)}/{self.target_object_count}. Save all first.")
                    continue
                break
        cv2.destroyWindow(window_name)

# =============================================================================
# WARP / ICP
# =============================================================================

def build_marker_frame(corners: dict[str, np.ndarray]) -> dict:
    TL, TR, BL, BR = corners["TL"], corners["TR"], corners["BL"], corners["BR"]
    x_hat = (TR - TL) / np.linalg.norm(TR - TL)
    y_raw = BL - TL
    y_axis = y_raw - np.dot(y_raw, x_hat) * x_hat
    y_hat = y_axis / np.linalg.norm(y_axis)
    z_hat = np.cross(x_hat, y_hat)
    z_hat = z_hat / np.linalg.norm(z_hat)
    y_hat = np.cross(z_hat, x_hat)
    y_hat = y_hat / np.linalg.norm(y_hat)
    width_avg = 0.5 * (np.linalg.norm(TR - TL) + np.linalg.norm(BR - BL))
    depth_avg = 0.5 * (np.linalg.norm(BL - TL) + np.linalg.norm(BR - TR))
    return {"origin": TL, "R": np.column_stack([x_hat, y_hat, z_hat]), "width_avg": width_avg, "depth_avg": depth_avg}


def world_to_local(points_xyz: np.ndarray, frame: dict) -> np.ndarray:
    return (np.asarray(points_xyz, dtype=np.float64) - frame["origin"][None, :]) @ frame["R"]


def local_to_world(local_xyz: np.ndarray, frame: dict) -> np.ndarray:
    return np.asarray(local_xyz, dtype=np.float64) @ frame["R"].T + frame["origin"][None, :]


def corners_to_array(corners: dict[str, np.ndarray]) -> np.ndarray:
    return np.vstack([corners[r] for r in _ROLE_NAMES]).astype(np.float64)


def get_corner_local_xy(corners: dict[str, np.ndarray], frame: dict) -> np.ndarray:
    return world_to_local(corners_to_array(corners), frame)[:, :2].astype(np.float64)


def compute_homography_2d(src_xy: np.ndarray, dst_xy: np.ndarray) -> np.ndarray:
    H, _ = cv2.findHomography(src_xy.astype(np.float64), dst_xy.astype(np.float64), method=0)
    if H is None:
        raise RuntimeError("cv2.findHomography failed.")
    return H.astype(np.float64)


def apply_homography_xy(xy: np.ndarray, H: np.ndarray) -> np.ndarray:
    homog = np.hstack([np.asarray(xy, dtype=np.float64), np.ones((len(xy), 1), dtype=np.float64)])
    warped = homog @ H.T
    denom = np.where(np.abs(warped[:, 2:3]) < 1e-12, np.nan, warped[:, 2:3])
    return warped[:, :2] / denom


def compute_z_scale(src_frame: dict, ref_frame: dict) -> float:
    if Z_SCALE_MODE == "none":
        return 1.0
    sx = ref_frame["width_avg"] / max(src_frame["width_avg"], 1e-12)
    sy = ref_frame["depth_avg"] / max(src_frame["depth_avg"], 1e-12)
    return float(0.5 * (sx + sy))


def kabsch_similarity_transform(src_pts: np.ndarray, dst_pts: np.ndarray, allow_scale: bool = True) -> np.ndarray:
    src, dst = np.asarray(src_pts, float), np.asarray(dst_pts, float)
    src_c, dst_c = src.mean(axis=0), dst.mean(axis=0)
    src0, dst0 = src - src_c, dst - dst_c
    H = src0.T @ dst0
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    scale = float(np.sum(S) / max(np.sum(src0 ** 2), 1e-12)) if allow_scale else 1.0
    t = dst_c - scale * (R @ src_c)
    T = np.eye(4)
    T[:3, :3] = scale * R
    T[:3, 3] = t
    return T


def apply_transform_xyz(points_xyz: np.ndarray, T: np.ndarray) -> np.ndarray:
    homog = np.hstack([np.asarray(points_xyz, float), np.ones((len(points_xyz), 1), float)])
    return (homog @ T.T)[:, :3]


def corner_warp_points_to_reference_camera(points_src_cam: np.ndarray, src_corners_cam: dict[str, np.ndarray], ref_corners_cam: dict[str, np.ndarray]):
    src_frame = build_marker_frame(src_corners_cam)
    ref_frame = build_marker_frame(ref_corners_cam)
    src_corner_arr = corners_to_array(src_corners_cam)
    ref_corner_arr = corners_to_array(ref_corners_cam)
    if WARP_MODE == "similarity_3d":
        T = kabsch_similarity_transform(src_corner_arr, ref_corner_arr, allow_scale=True)
        return apply_transform_xyz(points_src_cam, T), {"mode": WARP_MODE, "transform_4x4": T.tolist()}
    local_src = world_to_local(points_src_cam, src_frame)
    H = compute_homography_2d(get_corner_local_xy(src_corners_cam, src_frame), get_corner_local_xy(ref_corners_cam, ref_frame))
    warped_xy = apply_homography_xy(local_src[:, :2], H)
    warped_z = local_src[:, 2:3] * compute_z_scale(src_frame, ref_frame)
    local_ref = np.hstack([warped_xy, warped_z])
    return local_to_world(local_ref, ref_frame), {"mode": WARP_MODE, "homography_3x3": H.tolist()}


def yaw_from_rotation_matrix(R: np.ndarray) -> float:
    return float(np.arctan2(R[1, 0], R[0, 0]))


def rotation_angle_from_matrix(R: np.ndarray) -> float:
    return float(np.arccos(np.clip((float(np.trace(R)) - 1.0) / 2.0, -1.0, 1.0)))


def rotation_matrix_z(theta: float) -> np.ndarray:
    c, s = float(np.cos(theta)), float(np.sin(theta))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def clamp_z_rotation_in_transform(T: np.ndarray) -> np.ndarray:
    T = np.asarray(T, dtype=np.float64).copy()
    R, t = T[:3, :3], T[:3, 3].copy()
    yaw = yaw_from_rotation_matrix(R)
    yaw_clamped = float(np.clip(yaw, -np.deg2rad(MAX_ICP_Z_ROTATION_DEG), np.deg2rad(MAX_ICP_Z_ROTATION_DEG)))
    if abs(yaw - yaw_clamped) > np.deg2rad(0.25):
        print(f"[Z LIMIT] ICP yaw {np.rad2deg(yaw):+.2f} -> {np.rad2deg(yaw_clamped):+.2f} deg")
    R_tilt = rotation_matrix_z(yaw).T @ R
    T_new = np.eye(4)
    T_new[:3, :3] = rotation_matrix_z(yaw_clamped) @ R_tilt
    T_new[:3, 3] = t
    return T_new


def run_limited_icp(source, reference):
    import open3d as o3d
    if not RUN_LIMITED_ICP:
        return source, {"used": False}
    source_down = source.voxel_down_sample(ICP_VOXEL_SIZE)
    ref_down = reference.voxel_down_sample(ICP_VOXEL_SIZE)
    if source_down.is_empty() or ref_down.is_empty():
        return source, {"used": False, "reason": "empty_downsample"}
    if ICP_METHOD == "point_to_plane":
        source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=max(ICP_VOXEL_SIZE * 3, 0.005), max_nn=30))
        ref_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=max(ICP_VOXEL_SIZE * 3, 0.005), max_nn=30))
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    else:
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint(with_scaling=ALLOW_SCALE_IN_ICP)
    result = o3d.pipelines.registration.registration_icp(
        source_down, ref_down, ICP_DISTANCE_THRESHOLD, np.eye(4), estimation,
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=80, relative_fitness=1e-7, relative_rmse=1e-7),
    )
    T = result.transformation.copy()
    if ICP_ONLY_TRANSLATION:
        T2 = np.eye(4)
        T2[:3, 3] = T[:3, 3]
        T = T2
    elif LIMIT_ICP_Z_ROTATION:
        T = clamp_z_rotation_in_transform(T)
    trans = float(np.linalg.norm(T[:3, 3]))
    rot = float(np.rad2deg(rotation_angle_from_matrix(T[:3, :3])))
    if REJECT_BAD_ICP and (result.fitness < MIN_ACCEPTABLE_ICP_FITNESS or result.inlier_rmse > MAX_ACCEPTABLE_ICP_RMSE or trans > MAX_ICP_TRANSLATION_M or rot > MAX_ICP_TOTAL_ROTATION_DEG):
        print(f"[ICP REJECTED] fitness={result.fitness:.4f} rmse={result.inlier_rmse:.4f} trans={trans:.4f} rot={rot:.2f}")
        return source, {"used": True, "accepted": False, "fitness": float(result.fitness), "rmse": float(result.inlier_rmse)}
    aligned = copy.deepcopy(source)
    aligned.transform(T)
    print(f"[ICP] accepted fitness={result.fitness:.4f} rmse={result.inlier_rmse:.4f} trans={trans:.4f} rot={rot:.2f}")
    return aligned, {"used": True, "accepted": True, "fitness": float(result.fitness), "rmse": float(result.inlier_rmse), "transform": T.tolist()}

# =============================================================================
# ICP MERGE CONFIG LOAD / APPLY
# =============================================================================

def find_latest_auto_icp_config() -> "Path | None":
    """Return the newest JSON in ICP_MERGE_CONFIG_AUTO_DIR, or None."""
    if not ICP_MERGE_CONFIG_AUTO_DIR.exists():
        return None
    candidates = sorted(
        ICP_MERGE_CONFIG_AUTO_DIR.glob("icp_merge_config_*.json"),
        key=lambda p: p.stat().st_mtime,
    )
    return candidates[-1] if candidates else None


def load_icp_merge_config_if_available() -> "dict | None":
    """Load the best or latest auto ICP merge config, or return None."""
    if not USE_ICP_MERGE_CONFIG:
        print("[ICP CONFIG] USE_ICP_MERGE_CONFIG=False — using built-in defaults.")
        return None

    if PREFER_BEST_ICP_CONFIG and ICP_MERGE_CONFIG_BEST_PATH.exists():
        path = ICP_MERGE_CONFIG_BEST_PATH
        source = "config_best"
    else:
        path = find_latest_auto_icp_config()
        if path is None:
            print("[ICP CONFIG] No saved config found — using built-in defaults.")
            return None
        source = "latest auto"

    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if "params" not in data:
            print(f"[ICP CONFIG] {path} has no 'params' key — using built-in defaults.")
            return None
        print(f"[ICP CONFIG] Loaded {source}: {path}")
        return data
    except Exception as exc:
        print(f"[ICP CONFIG] Failed to load {path}: {exc} — using built-in defaults.")
        return None


def apply_icp_merge_config(params: dict) -> None:
    """Overwrite global ICP/merge settings from a loaded config params dict."""
    global RUN_LIMITED_ICP, ICP_ONLY_TRANSLATION, MAX_ICP_Z_ROTATION_DEG
    global ICP_DISTANCE_THRESHOLD, ICP_VOXEL_SIZE
    global VOXEL_SIZE_FINAL_PER_OBJECT, VOXEL_SIZE_FINAL_ALL_OBJECTS
    global GLOBAL_FINAL_SHIFT_M, TUNED_Z_SCALE

    RUN_LIMITED_ICP = bool(params["use_icp"])
    ICP_ONLY_TRANSLATION = bool(params["translation_only"])
    MAX_ICP_Z_ROTATION_DEG = float(params["max_yaw_deg"])
    ICP_DISTANCE_THRESHOLD = float(params["icp_threshold_m"])
    ICP_VOXEL_SIZE = float(params["icp_voxel_m"])
    VOXEL_SIZE_FINAL_PER_OBJECT = float(params["final_voxel_m"])
    VOXEL_SIZE_FINAL_ALL_OBJECTS = float(params["final_voxel_m"])
    GLOBAL_FINAL_SHIFT_M = np.array(
        [params["x_shift_m"], params["y_shift_m"], params["z_shift_m"]],
        dtype=np.float64,
    )
    TUNED_Z_SCALE = float(params.get("z_scale", 1.0))

    print(
        f"[ICP CONFIG] Applied — icp={RUN_LIMITED_ICP} trans_only={ICP_ONLY_TRANSLATION} "
        f"yaw={MAX_ICP_Z_ROTATION_DEG:.1f}deg th={ICP_DISTANCE_THRESHOLD*1000:.1f}mm "
        f"vox={ICP_VOXEL_SIZE*1000:.1f}mm final_vox={VOXEL_SIZE_FINAL_PER_OBJECT*1000:.1f}mm "
        f"shift=({GLOBAL_FINAL_SHIFT_M[0]*1000:.1f},{GLOBAL_FINAL_SHIFT_M[1]*1000:.1f},"
        f"{GLOBAL_FINAL_SHIFT_M[2]*1000:.1f})mm z_scale={TUNED_Z_SCALE:.3f}"
    )

# =============================================================================
# PIPELINE
# =============================================================================

def capture_two_views(stereo: StereoCamera, rect_maps, detector, P_l, P_r) -> list[ScanData]:
    scans: list[ScanData] = []
    cv2.namedWindow(WINDOW_LIVE, cv2.WINDOW_NORMAL)
    for view_id in range(NUM_VIEWS):
        capture_armed = False
        print(f"\n[CAPTURE] View {view_id}: press SPACE; it will freeze once all 4 markers are visible.")
        while True:
            ok, left, right = stereo.read_pair()
            if not ok or left is None or right is None:
                continue
            lr, rr = rectify_pair(left, right, rect_maps)
            det_l, det_r = _detect_aruco(detector, lr), _detect_aruco(detector, rr)
            both = sum(1 for mid in MARKER_IDS_CW if mid in det_l and mid in det_r)
            ready = both == len(MARKER_IDS_CW)
            preview = stereo_preview(_draw_aruco_overlay(lr, det_l), _draw_aruco_overlay(rr, det_r))
            cv2.putText(preview, f"View {view_id}: {both}/4 markers in both eyes", (14, preview.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0) if ready else (0, 180, 255), 2, cv2.LINE_AA)
            preview = draw_ui_panel(preview, f"Capture view {view_id} — {'ARMED' if capture_armed else 'SPACE to arm'}", ["Move to new angle after view 0.", "q/Esc quits."], PANEL_HEIGHT)
            cv2.imshow(WINDOW_LIVE, preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                raise KeyboardInterrupt("User quit during capture.")
            if key == ord(" "):
                capture_armed = True
            if capture_armed and ready:
                corners_m, pixels_l, pixels_r = triangulate_aruco_corners(det_l, det_r, P_l, P_r)
                if len(corners_m) < 4:
                    capture_armed = False
                    continue
                plane = fit_plane_from_points(np.vstack([corners_m[r] for r in _ROLE_NAMES]))
                frame = build_workspace_frame_from_corners(corners_m)
                scans.append(ScanData(view_id, lr.copy(), rr.copy(), det_l.copy(), det_r.copy(), corners_m, pixels_l, pixels_r, plane, frame, {r: frame.camera_to_workspace_point(p) for r, p in corners_m.items()}))
                print(f"[CAPTURED] view {view_id}")
                break
    cv2.destroyWindow(WINDOW_LIVE)
    return scans


def compute_disparity_for_scans(scans: list[ScanData], q_matrix: np.ndarray) -> None:
    for scan in scans:
        print(f"\n[RAFT] View {scan.view_id}")
        scan.disparity = compute_raft_disparity(
            scan.left_rect, scan.right_rect,
            raft_repo_dir=RAFT_REPO_DIR, checkpoint=RAFT_CHECKPOINT, iters=RAFT_ITERS,
            corr_implementation=RAFT_CORR_IMPLEMENTATION, mixed_precision=RAFT_MIXED_PRECISION,
        )
        scan.points_3d_cam_m = reproject_disparity_to_points(scan.disparity, q_matrix)


def run_sam_for_scans(scans: list[ScanData], predictor: Any) -> None:
    for scan in scans:
        session = SamObjectSession(scan, predictor, TARGET_OBJECT_COUNT)
        session.run()
        if len(scan.objects) != TARGET_OBJECT_COUNT:
            raise RuntimeError(f"View {scan.view_id} has {len(scan.objects)} objects, expected {TARGET_OBJECT_COUNT}.")


def save_minimal_capture_artifacts(scans: list[ScanData], run_dir: Path) -> None:
    for scan in scans:
        view_dir = run_dir / f"view_{scan.view_id:02d}"
        view_dir.mkdir(parents=True, exist_ok=True)
        if SAVE_CAPTURE_IMAGES:
            cv2.imwrite(str(view_dir / "left_rect.png"), scan.left_rect)
            cv2.imwrite(str(view_dir / "right_rect.png"), scan.right_rect)
        if SAVE_MASK_OVERLAYS:
            cv2.imwrite(str(view_dir / "all_object_masks_overlay.png"), draw_object_overlay(scan.left_rect, scan.objects))


def merge_outputs(scans: list[ScanData], run_dir: Path) -> dict[str, Any]:
    import open3d as o3d
    ref_scan, src_scan = scans[0], scans[1]
    ref_corners, src_corners = ref_scan.corners_cam_m, src_scan.corners_cam_m
    ref_frame = build_marker_frame(ref_corners)
    floor_ref_local = world_to_local(corners_to_array(ref_corners), ref_frame)
    all_corner_only, all_icp = [], []
    debug = {"objects": []}
    for object_id in range(TARGET_OBJECT_COUNT):
        print(f"\n[MERGE] object_{object_id:02d}")
        ref_obj, src_obj = ref_scan.objects[object_id], src_scan.objects[object_id]
        ref_local = world_to_local(ref_obj.points_cam_m, ref_frame) + GLOBAL_FINAL_SHIFT_M[None, :]
        ref_pcd = np_to_o3d_cloud(ref_local, ref_obj.colors_bgr)
        src_warped_cam, warp_info = corner_warp_points_to_reference_camera(src_obj.points_cam_m, src_corners, ref_corners)
        src_local = world_to_local(src_warped_cam, ref_frame)
        src_local[:, 2] *= TUNED_Z_SCALE
        src_local = src_local + GLOBAL_FINAL_SHIFT_M[None, :]
        src_pcd_corner = np_to_o3d_cloud(src_local, src_obj.colors_bgr)
        corners_merged = o3d.geometry.PointCloud(); corners_merged += ref_pcd; corners_merged += src_pcd_corner
        corners_merged = downsample_pcd(remove_outliers_pcd(corners_merged, FINAL_OUTLIER_NB_NEIGHBORS, FINAL_OUTLIER_STD_RATIO), VOXEL_SIZE_FINAL_PER_OBJECT)
        corners_path = run_dir / f"object_{object_id:02d}_merged_corners_only.ply"
        o3d.io.write_point_cloud(str(corners_path), corners_merged)
        print(f"[SAVE] {corners_path}")
        all_corner_only.append(corners_merged)
        src_pcd_icp, icp_info = run_limited_icp(src_pcd_corner, ref_pcd)
        icp_merged = o3d.geometry.PointCloud(); icp_merged += ref_pcd; icp_merged += src_pcd_icp
        icp_merged = downsample_pcd(remove_outliers_pcd(icp_merged, FINAL_OUTLIER_NB_NEIGHBORS, FINAL_OUTLIER_STD_RATIO), VOXEL_SIZE_FINAL_PER_OBJECT)
        icp_path = run_dir / f"object_{object_id:02d}_merged_limited_icp.ply"
        o3d.io.write_point_cloud(str(icp_path), icp_merged)
        print(f"[SAVE] {icp_path}")
        all_icp.append(icp_merged)
        debug["objects"].append({"object_id": object_id, "warp": warp_info, "icp": icp_info})
    all_corners = o3d.geometry.PointCloud()
    for p in all_corner_only: all_corners += p
    all_corners = downsample_pcd(remove_outliers_pcd(all_corners, FINAL_OUTLIER_NB_NEIGHBORS, FINAL_OUTLIER_STD_RATIO), VOXEL_SIZE_FINAL_ALL_OBJECTS)
    all_corners_path = run_dir / "all_objects_merged_corners_only.ply"
    o3d.io.write_point_cloud(str(all_corners_path), all_corners)
    all_icp_pcd = o3d.geometry.PointCloud()
    for p in all_icp: all_icp_pcd += p
    all_icp_pcd = downsample_pcd(remove_outliers_pcd(all_icp_pcd, FINAL_OUTLIER_NB_NEIGHBORS, FINAL_OUTLIER_STD_RATIO), VOXEL_SIZE_FINAL_ALL_OBJECTS)
    all_icp_path = run_dir / "all_objects_merged_limited_icp.ply"
    o3d.io.write_point_cloud(str(all_icp_path), all_icp_pcd)
    write_scene_with_floor_ply(run_dir / "workspace_plane_mesh.ply", floor_ref_local, [])
    write_scene_with_floor_ply(run_dir / "scene_corners_only_with_plane.ply", floor_ref_local, all_corner_only)
    write_scene_with_floor_ply(run_dir / "scene_limited_icp_with_plane.ply", floor_ref_local, all_icp)
    debug["outputs"] = {"corners_all": str(all_corners_path), "icp_all": str(all_icp_path)}
    return debug


def write_metadata(run_dir: Path, scans: list[ScanData], merge_debug: dict[str, Any]) -> None:
    metadata = {
        "created_at": datetime.now().isoformat(),
        "calibration": str(config.ACTIVE_CALIBRATION_NPZ),
        "marker_ids_cw_TL_TR_BR_BL": MARKER_IDS_CW,
        "target_object_count": TARGET_OBJECT_COUNT,
        "views": [],
        "merge": merge_debug,
    }
    for scan in scans:
        metadata["views"].append({
            "view_id": scan.view_id,
            "corners_camera_m": {k: v.tolist() for k, v in scan.corners_cam_m.items()},
            "corners_workspace_m": {k: v.tolist() for k, v in scan.corners_workspace_m.items()},
            "objects": [{"object_id": obj.object_id, "point_count": int(len(obj.points_cam_m))} for obj in scan.objects],
        })
    path = run_dir / "pipeline_metadata.json"
    path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(f"[SAVE] {path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="One-pass two-view SAM/RAFT/ArUco reconstruction pipeline.")
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)

    parser.add_argument(
        "--interactive-tuner",
        action="store_true",
        help="After saving the one-pass output/cache, launch the interactive merge tuner.",
    )

    return parser.parse_args()

def save_tuner_cache(scans: list[ScanData], run_dir: Path) -> None:
    """
    Save raw per-view, per-object point clouds for the interactive tuner.

    The tuner uses these cached arrays so it can rerun only:
        corner warp -> limited ICP -> final merge

    It does NOT rerun:
        cameras
        RAFT
        SAM
    """
    cache_dir = run_dir / "tuner_cache"
    cache_dir.mkdir(parents=True, exist_ok=True)

    for scan in scans:
        corners_path = cache_dir / f"view_{scan.view_id:02d}_corners_camera_m.json"
        corners_path.write_text(
            json.dumps(
                {name: pt.tolist() for name, pt in scan.corners_cam_m.items()},
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )

        workspace_corners_path = cache_dir / f"view_{scan.view_id:02d}_corners_workspace_m.json"
        workspace_corners_path.write_text(
            json.dumps(
                {name: pt.tolist() for name, pt in scan.corners_workspace_m.items()},
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )

        for obj in scan.objects:
            np.savez_compressed(
                cache_dir / f"object_{obj.object_id:02d}_view_{scan.view_id:02d}.npz",
                points_cam_m=obj.points_cam_m.astype(np.float64),
                colors_bgr=obj.colors_bgr.astype(np.uint8),
                points_workspace_m=obj.points_workspace_m.astype(np.float64),
                mask=obj.mask.astype(np.uint8),
            )

    print(f"[SAVE] tuner cache: {cache_dir}")

def main() -> None:
    args = parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)
    run_dir = args.output_root / f"pipeline_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    run_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 72)
    print("ONE-PASS TWO-VIEW SAM + ARUCO + RAFT + MERGE PIPELINE")
    print("=" * 72)
    print(f"[OUTPUT] {run_dir}")
    print("\n[ORDER]")
    print("  1. Capture view 0 with all 4 markers")
    print("  2. Capture view 1 with all 4 markers")
    print("  3. Close cameras")
    print("  4. Run RAFT disparity for both frozen pairs")
    print("  5. Run SAM segmentation for both views")
    print("  6. Build point clouds and merge")

    calib = load_calibration()
    q_matrix = calib["disparity_to_depth_Q"]
    P_l = np.asarray(calib["projection_left_rectified"], dtype=np.float64)
    P_r = np.asarray(calib["projection_right_rectified"], dtype=np.float64)
    detector = _build_detector(ARUCO_DICT_TYPE)

    # ------------------------------------------------------------------
    # IMPORTANT ORDER CHANGE:
    # Do NOT load SAM and do NOT run RAFT while the cameras are open.
    # First grab both frozen stereo pairs quickly, then release the cameras.
    # ------------------------------------------------------------------
    with StereoCamera() as stereo:
        stereo.warmup()
        ok, left0, right0 = stereo.read_pair()
        if not ok or left0 is None or right0 is None:
            raise RuntimeError("Could not read initial stereo pair.")
        rect_maps = build_rectification_maps(calib, *left0.shape[:2])
        scans = capture_two_views(stereo, rect_maps, detector, P_l, P_r)

    print("\n[INFO] Both views captured. Cameras are now closed.")
    print("[INFO] Starting RAFT disparity for frozen captures...")

    compute_disparity_for_scans(scans, q_matrix)

    print("\n[INFO] Disparity maps finished. Loading SAM now...")
    sam_path = resolve_sam_checkpoint(SAM_MODEL_PATH, SAM_MODEL_LEGACY_PATH)
    predictor = load_sam_predictor(sam_path, SAM_MODEL_TYPE)

    print("\n[INFO] Starting SAM segmentation sessions...")
    run_sam_for_scans(scans, predictor)

    save_minimal_capture_artifacts(scans, run_dir)
    save_tuner_cache(scans, run_dir)

    _icp_cfg = load_icp_merge_config_if_available()
    if _icp_cfg is not None:
        apply_icp_merge_config(_icp_cfg["params"])

    print("\n[INFO] Building corner-only and limited-ICP merged outputs...")
    merge_debug = merge_outputs(scans, run_dir)
    write_metadata(run_dir, scans, merge_debug)

    if ACTIVE_TUNER:
        print("\n[INFO] Launching interactive merge tuner...")
        try:
            from interactive_merge_tuner_from_cache import run_tuner
            run_tuner(run_dir)
        except Exception as exc:
            print(f"[WARN] Could not launch tuner automatically: {exc}")
            print("You can still run it manually with:")
            print(f'  python .\\interactive_merge_tuner_from_cache.py --run-dir "{run_dir}"')

    cv2.destroyAllWindows()

    print("\n" + "=" * 72)
    print("[DONE] Open these in CloudCompare:")
    print(f"  {run_dir / 'scene_corners_only_with_plane.ply'}")
    print(f"  {run_dir / 'scene_limited_icp_with_plane.ply'}")
    print(f"  {run_dir / 'all_objects_merged_corners_only.ply'}")
    print(f"  {run_dir / 'all_objects_merged_limited_icp.ply'}")
    print("=" * 72)


if __name__ == "__main__":
    main()
