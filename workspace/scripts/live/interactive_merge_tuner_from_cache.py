"""
Interactive merge tuner for the one-pass two-view pipeline.

Save as:
    workspace/scripts/offline/interactive_merge_tuner_from_cache.py

Run latest cached capture:
    python .\interactive_merge_tuner_from_cache.py

Run a specific capture:
    python .\interactive_merge_tuner_from_cache.py --run-dir "C:\path\to\pipeline_YYYYMMDD_HHMMSS"

Controls:
    Tkinter panel: Recompute | Reset View | Save Auto Config + Run Outputs | Save Best Config | Quit
    Keyboard (in Open3D window):
        r = recompute
        v = reset view
        s = save auto config + run outputs
        b = save best config
        q / Esc = quit
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np
import open3d as o3d

# -----------------------------------------------------------------------------
# Project path setup
# -----------------------------------------------------------------------------

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import config

# -----------------------------------------------------------------------------
# Defaults
# -----------------------------------------------------------------------------

DEFAULT_OUTPUT_ROOT = config.CALIBRATION_ROOT / "one_pass_two_view_reconstruction"

ICP_MERGE_CONFIG_ROOT = DEFAULT_OUTPUT_ROOT / "ICP_merge_config"
ICP_MERGE_CONFIG_AUTO_DIR = ICP_MERGE_CONFIG_ROOT / "auto"
ICP_MERGE_CONFIG_BEST_DIR = ICP_MERGE_CONFIG_ROOT / "config_best"

CORNER_ORDER = ["TL", "TR", "BR", "BL"]

CONTROL_WINDOW = "Merge tuner controls"
VIEW_WINDOW = "Open3D tuned merge viewer"

# Only used on FIRST display and when pressing v.
VIEW_FRONT = [0.0, 0.0, -1.0]
VIEW_UP = [0.0, 1.0, 0.0]
VIEW_ZOOM = 0.70

POINT_SIZE = 4.0
AXIS_SIZE_M = 0.10

FLOOR_COLOR = np.array([[0.35, 0.45, 0.55]], dtype=np.float64)

# Workspace grid
WORKSPACE_GRID_SPACING_M = 0.05
WORKSPACE_AXIS_LENGTH_M = 0.15

# Height metrics
HEIGHT_LOW_PERCENTILE = 2.0
HEIGHT_HIGH_PERCENTILE = 95.0
HEIGHT_TOP_PERCENTILE = 95.0
HEIGHT_BOTTOM_PERCENTILE = 5.0
HEIGHT_MIN_POINTS = 50

# Presentation mesh
ENABLE_PRESENTATION_MESH_OUTPUTS = True
ENABLE_RADIUS_OUTLIER_REMOVAL = True
RADIUS_OUTLIER_NB_POINTS = 8
RADIUS_OUTLIER_RADIUS_M = 0.01
ENABLE_NORMAL_ESTIMATION = True
NORMAL_RADIUS_M = 0.015
NORMAL_MAX_NN = 30
ENABLE_MESH_BALL_PIVOTING = True
BALL_PIVOT_RADII_M = [0.004, 0.008, 0.012]
ENABLE_MESH_SMOOTHING = True
MESH_SMOOTHING_ITERATIONS = 3

# -----------------------------------------------------------------------------
# Data containers
# -----------------------------------------------------------------------------

@dataclass
class CachedObjectView:
    object_id: int
    view_id: int
    points_cam_m: np.ndarray
    colors_bgr: np.ndarray


@dataclass
class TunerParams:
    use_icp: bool
    translation_only: bool
    use_colored_icp: bool
    max_yaw_deg: float
    icp_threshold_m: float
    icp_voxel_m: float
    final_voxel_m: float
    x_shift_m: float
    y_shift_m: float
    z_shift_m: float
    z_scale: float
    view_mode_icp: bool


# -----------------------------------------------------------------------------
# Small utilities
# -----------------------------------------------------------------------------

def latest_run_dir(root: Path) -> Path:
    candidates = sorted(
        [
            p for p in root.glob("pipeline_*")
            if p.is_dir() and (p / "tuner_cache").exists()
        ],
        key=lambda p: p.stat().st_mtime,
    )

    if not candidates:
        raise FileNotFoundError(f"No cached pipeline_* runs found in {root}")

    return candidates[-1]


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def save_json(path: Path, obj: Any) -> None:
    path.write_text(json.dumps(obj, indent=2) + "\n", encoding="utf-8")


def build_config_json(
    run_dir: Path,
    params: "TunerParams",
    debug: "dict[str, Any]",
) -> "dict[str, Any]":
    """Build the portable config dict that can be loaded by the pipeline."""
    return {
        "created_at": datetime.now().isoformat(),
        "source_run_dir": str(run_dir),
        "params": params.__dict__,
        "debug": debug,
    }


def load_corners(path: Path) -> dict[str, np.ndarray]:
    raw = load_json(path)

    missing = [k for k in CORNER_ORDER if k not in raw]
    if missing:
        raise KeyError(f"{path} missing corners: {missing}")

    return {k: np.asarray(raw[k], dtype=np.float64) for k in CORNER_ORDER}


def discover_object_ids(cache_dir: Path) -> list[int]:
    ids0 = set()
    ids1 = set()

    for p in cache_dir.glob("object_*_view_00.npz"):
        try:
            ids0.add(int(p.name.split("_")[1]))
        except Exception:
            pass

    for p in cache_dir.glob("object_*_view_01.npz"):
        try:
            ids1.add(int(p.name.split("_")[1]))
        except Exception:
            pass

    ids = sorted(ids0 & ids1)

    if not ids:
        raise RuntimeError(
            f"No matching object_XX_view_00/01 cache files found in {cache_dir}"
        )

    return ids


def load_object_view(cache_dir: Path, object_id: int, view_id: int) -> CachedObjectView:
    path = cache_dir / f"object_{object_id:02d}_view_{view_id:02d}.npz"

    if not path.exists():
        raise FileNotFoundError(path)

    data = np.load(path)

    return CachedObjectView(
        object_id=object_id,
        view_id=view_id,
        points_cam_m=np.asarray(data["points_cam_m"], dtype=np.float64),
        colors_bgr=np.asarray(data["colors_bgr"], dtype=np.uint8),
    )


def np_to_o3d_cloud(
    points_xyz: np.ndarray,
    colors_bgr: np.ndarray | None = None,
) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points_xyz, dtype=np.float64))

    if colors_bgr is not None and len(colors_bgr) == len(points_xyz):
        rgb = colors_bgr[:, ::-1].astype(np.float64) / 255.0
        pcd.colors = o3d.utility.Vector3dVector(rgb)

    return pcd


def merge_clouds(clouds: list[o3d.geometry.PointCloud]) -> o3d.geometry.PointCloud:
    merged = o3d.geometry.PointCloud()

    for pcd in clouds:
        merged += pcd

    return merged


def safe_remove_outliers(
    pcd: o3d.geometry.PointCloud,
    nb_neighbors: int = 25,
    std_ratio: float = 2.0,
) -> o3d.geometry.PointCloud:
    if len(pcd.points) < nb_neighbors:
        return pcd

    clean, _ = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio,
    )

    return clean


def safe_downsample(
    pcd: o3d.geometry.PointCloud,
    voxel_m: float,
) -> o3d.geometry.PointCloud:
    if voxel_m <= 0:
        return pcd

    if len(pcd.points) == 0:
        return pcd

    return pcd.voxel_down_sample(voxel_m)


# -----------------------------------------------------------------------------
# Marker-frame and corner-warp math
# -----------------------------------------------------------------------------

def corners_to_array(corners: dict[str, np.ndarray]) -> np.ndarray:
    return np.vstack([corners[k] for k in CORNER_ORDER]).astype(np.float64)


def build_marker_frame(corners: dict[str, np.ndarray]) -> dict[str, np.ndarray | float]:
    TL = corners["TL"]
    TR = corners["TR"]
    BR = corners["BR"]
    BL = corners["BL"]

    x_hat = TR - TL
    x_hat = x_hat / np.linalg.norm(x_hat)

    y_raw = BL - TL
    y_axis = y_raw - np.dot(y_raw, x_hat) * x_hat
    y_hat = y_axis / np.linalg.norm(y_axis)

    z_hat = np.cross(x_hat, y_hat)
    z_hat = z_hat / np.linalg.norm(z_hat)

    y_hat = np.cross(z_hat, x_hat)
    y_hat = y_hat / np.linalg.norm(y_hat)

    R = np.column_stack([x_hat, y_hat, z_hat])

    width_avg = 0.5 * (np.linalg.norm(TR - TL) + np.linalg.norm(BR - BL))
    depth_avg = 0.5 * (np.linalg.norm(BL - TL) + np.linalg.norm(BR - TR))

    return {
        "origin": TL,
        "R": R,
        "width_avg": float(width_avg),
        "depth_avg": float(depth_avg),
    }


def world_to_local(points_xyz: np.ndarray, frame: dict[str, Any]) -> np.ndarray:
    return (
        np.asarray(points_xyz, dtype=np.float64) - frame["origin"][None, :]
    ) @ frame["R"]


def local_to_world(local_xyz: np.ndarray, frame: dict[str, Any]) -> np.ndarray:
    return np.asarray(local_xyz, dtype=np.float64) @ frame["R"].T + frame["origin"][None, :]


def corner_local_xy(corners: dict[str, np.ndarray], frame: dict[str, Any]) -> np.ndarray:
    return world_to_local(corners_to_array(corners), frame)[:, :2]


def compute_homography(src_xy: np.ndarray, dst_xy: np.ndarray) -> np.ndarray:
    H, _ = cv2.findHomography(
        src_xy.astype(np.float64),
        dst_xy.astype(np.float64),
        method=0,
    )

    if H is None:
        raise RuntimeError("cv2.findHomography failed")

    return H.astype(np.float64)


def apply_homography_xy(xy: np.ndarray, H: np.ndarray) -> np.ndarray:
    xy = np.asarray(xy, dtype=np.float64)

    homog = np.hstack(
        [
            xy,
            np.ones((len(xy), 1), dtype=np.float64),
        ]
    )

    warped = homog @ H.T
    denom = np.where(np.abs(warped[:, 2:3]) < 1e-12, np.nan, warped[:, 2:3])

    return warped[:, :2] / denom


def corner_warp_source_to_reference(
    src_points_cam: np.ndarray,
    src_corners_cam: dict[str, np.ndarray],
    ref_corners_cam: dict[str, np.ndarray],
) -> np.ndarray:
    src_frame = build_marker_frame(src_corners_cam)
    ref_frame = build_marker_frame(ref_corners_cam)

    local_src = world_to_local(src_points_cam, src_frame)

    H = compute_homography(
        corner_local_xy(src_corners_cam, src_frame),
        corner_local_xy(ref_corners_cam, ref_frame),
    )

    xy_ref = apply_homography_xy(local_src[:, :2], H)

    sx = ref_frame["width_avg"] / max(float(src_frame["width_avg"]), 1e-12)
    sy = ref_frame["depth_avg"] / max(float(src_frame["depth_avg"]), 1e-12)
    z_scale = 0.5 * (sx + sy)

    z_ref = local_src[:, 2:3] * z_scale
    local_ref = np.hstack([xy_ref, z_ref])

    return local_to_world(local_ref, ref_frame)


# -----------------------------------------------------------------------------
# Limited ICP
# -----------------------------------------------------------------------------

def yaw_from_rotation_matrix(R: np.ndarray) -> float:
    return float(np.arctan2(R[1, 0], R[0, 0]))


def rotation_angle_from_matrix(R: np.ndarray) -> float:
    val = (float(np.trace(R)) - 1.0) / 2.0
    return float(np.arccos(np.clip(val, -1.0, 1.0)))


def rotation_matrix_z(theta: float) -> np.ndarray:
    c = float(np.cos(theta))
    s = float(np.sin(theta))

    return np.array(
        [
            [c, -s, 0.0],
            [s,  c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def clamp_z_rotation(T: np.ndarray, max_yaw_deg: float) -> np.ndarray:
    T = T.copy()

    R = T[:3, :3]
    t = T[:3, 3].copy()

    yaw = yaw_from_rotation_matrix(R)
    yaw_clamped = float(
        np.clip(
            yaw,
            -np.deg2rad(max_yaw_deg),
            np.deg2rad(max_yaw_deg),
        )
    )

    R_tilt = rotation_matrix_z(yaw).T @ R
    R_new = rotation_matrix_z(yaw_clamped) @ R_tilt

    out = np.eye(4)
    out[:3, :3] = R_new
    out[:3, 3] = t

    return out


def run_limited_icp(
    source: o3d.geometry.PointCloud,
    reference: o3d.geometry.PointCloud,
    params: TunerParams,
) -> tuple[o3d.geometry.PointCloud, dict[str, Any]]:
    if not params.use_icp:
        return source, {"used": False}

    src_down = source.voxel_down_sample(params.icp_voxel_m)
    ref_down = reference.voxel_down_sample(params.icp_voxel_m)

    if src_down.is_empty() or ref_down.is_empty():
        return source, {
            "used": False,
            "reason": "empty_downsample",
        }

    result = None
    method_used = "point_to_point"

    if params.use_colored_icp:
        try:
            src_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=params.icp_voxel_m * 3, max_nn=30))
            ref_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=params.icp_voxel_m * 3, max_nn=30))
            result = o3d.pipelines.registration.registration_colored_icp(
                src_down, ref_down, params.icp_threshold_m, np.eye(4),
                o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=80, relative_fitness=1e-7, relative_rmse=1e-7),
            )
            method_used = "colored_icp"
        except Exception as exc:
            print(f"[WARN] Colored ICP failed ({exc}), falling back.")
            result = None

    if result is None:
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint(False)
        result = o3d.pipelines.registration.registration_icp(
            src_down,
            ref_down,
            params.icp_threshold_m,
            np.eye(4),
            estimation,
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=80,
                relative_fitness=1e-7,
                relative_rmse=1e-7,
            ),
        )
        method_used = "point_to_point"

    T = result.transformation.copy()

    if params.translation_only:
        T2 = np.eye(4)
        T2[:3, 3] = T[:3, 3]
        T = T2
    else:
        T = clamp_z_rotation(T, params.max_yaw_deg)

    translation_m = float(np.linalg.norm(T[:3, 3]))
    rotation_deg = float(np.rad2deg(rotation_angle_from_matrix(T[:3, :3])))

    rejected = (
        result.fitness < 0.01
        or result.inlier_rmse > 0.08
        or translation_m > 0.08
        or rotation_deg > 25.0
    )

    if rejected:
        return source, {
            "used": True,
            "accepted": False,
            "method": method_used,
            "fitness": float(result.fitness),
            "rmse": float(result.inlier_rmse),
            "translation_m": translation_m,
            "rotation_deg": rotation_deg,
        }

    aligned = o3d.geometry.PointCloud(source)
    aligned.transform(T)

    return aligned, {
        "used": True,
        "accepted": True,
        "method": method_used,
        "fitness": float(result.fitness),
        "rmse": float(result.inlier_rmse),
        "translation_m": translation_m,
        "rotation_deg": rotation_deg,
        "transform": T.tolist(),
    }


# -----------------------------------------------------------------------------
# Recompute tuned merge
# -----------------------------------------------------------------------------

def load_cached_dataset(run_dir: Path) -> dict[str, Any]:
    cache_dir = run_dir / "tuner_cache"

    if not cache_dir.exists():
        raise FileNotFoundError(
            f"No tuner_cache folder found in {run_dir}. Run the patched base pipeline first."
        )

    object_ids = discover_object_ids(cache_dir)

    ref_corners = load_corners(cache_dir / "view_00_corners_camera_m.json")
    src_corners = load_corners(cache_dir / "view_01_corners_camera_m.json")

    ref_frame = build_marker_frame(ref_corners)

    objects = {}

    for object_id in object_ids:
        objects[object_id] = {
            0: load_object_view(cache_dir, object_id, 0),
            1: load_object_view(cache_dir, object_id, 1),
        }

    floor_ref_local = world_to_local(corners_to_array(ref_corners), ref_frame)

    return {
        "run_dir": run_dir,
        "cache_dir": cache_dir,
        "object_ids": object_ids,
        "ref_corners": ref_corners,
        "src_corners": src_corners,
        "ref_frame": ref_frame,
        "floor_ref_local": floor_ref_local,
        "objects": objects,
    }


def recompute_merge(
    dataset: dict[str, Any],
    params: TunerParams,
) -> tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud, list, list, dict[str, Any]]:
    """Returns (corner_merged, icp_merged, per_obj_corner_pcds, per_obj_icp_pcds, debug)."""
    ref_corners = dataset["ref_corners"]
    src_corners = dataset["src_corners"]
    ref_frame = dataset["ref_frame"]

    corner_clouds: list[o3d.geometry.PointCloud] = []
    icp_clouds: list[o3d.geometry.PointCloud] = []
    per_obj_corner: list[o3d.geometry.PointCloud] = []
    per_obj_icp: list[o3d.geometry.PointCloud] = []

    debug: dict[str, Any] = {"objects": []}

    for object_id in dataset["object_ids"]:
        ref_obj: CachedObjectView = dataset["objects"][object_id][0]
        src_obj: CachedObjectView = dataset["objects"][object_id][1]

        ref_local = world_to_local(ref_obj.points_cam_m, ref_frame)

        src_warped_cam = corner_warp_source_to_reference(
            src_obj.points_cam_m,
            src_corners,
            ref_corners,
        )

        src_local = world_to_local(src_warped_cam, ref_frame)

        src_local[:, 0] += params.x_shift_m
        src_local[:, 1] += params.y_shift_m
        src_local[:, 2] = src_local[:, 2] * params.z_scale + params.z_shift_m

        ref_pcd = np_to_o3d_cloud(ref_local, ref_obj.colors_bgr)
        src_corner_pcd = np_to_o3d_cloud(src_local, src_obj.colors_bgr)

        corner_pair = safe_downsample(safe_remove_outliers(merge_clouds([ref_pcd, src_corner_pcd])), params.final_voxel_m)
        corner_clouds.append(corner_pair)
        per_obj_corner.append(corner_pair)

        src_icp_pcd, icp_info = run_limited_icp(src_corner_pcd, ref_pcd, params)

        icp_pair = safe_downsample(safe_remove_outliers(merge_clouds([ref_pcd, src_icp_pcd])), params.final_voxel_m)
        icp_clouds.append(icp_pair)
        per_obj_icp.append(icp_pair)

        debug["objects"].append({"object_id": object_id, "icp": icp_info})

    corner_merged = merge_clouds(corner_clouds)
    icp_merged = merge_clouds(icp_clouds)

    return corner_merged, icp_merged, per_obj_corner, per_obj_icp, debug


# -----------------------------------------------------------------------------
# PLY scene saving
# -----------------------------------------------------------------------------

def write_scene_with_floor_ply(
    path: Path,
    floor_corners_xyz: np.ndarray,
    pcd: o3d.geometry.PointCloud,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    floor = np.asarray(floor_corners_xyz, dtype=np.float64)
    floor_rgb = np.tile(np.array([[90, 110, 130]], dtype=np.uint8), (4, 1))

    obj_pts = np.asarray(pcd.points, dtype=np.float64)

    if pcd.has_colors():
        obj_rgb = np.clip(np.asarray(pcd.colors) * 255.0, 0, 255).astype(np.uint8)
    else:
        obj_rgb = np.tile(np.array([[230, 180, 30]], dtype=np.uint8), (len(obj_pts), 1))

    xyz_all = np.vstack([floor, obj_pts])
    rgb_all = np.vstack([floor_rgb, obj_rgb])

    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (2, 1, 0),
        (3, 2, 0),
    ]

    with path.open("w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(xyz_all)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write(f"element face {len(faces)}\n")
        f.write("property list uchar int vertex_indices\n")
        f.write("end_header\n")

        for p, c in zip(xyz_all, rgb_all):
            f.write(
                f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                f"{int(c[0])} {int(c[1])} {int(c[2])}\n"
            )

        for a, b, c in faces:
            f.write(f"3 {a} {b} {c}\n")


def make_floor_mesh(floor_corners_xyz: np.ndarray) -> o3d.geometry.TriangleMesh:
    floor = np.asarray(floor_corners_xyz, dtype=np.float64)

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(floor)

    mesh.triangles = o3d.utility.Vector3iVector(
        np.array(
            [
                [0, 1, 2],
                [0, 2, 3],
                [2, 1, 0],
                [3, 2, 0],
            ],
            dtype=np.int32,
        )
    )

    mesh.vertex_colors = o3d.utility.Vector3dVector(
        np.tile(FLOOR_COLOR, (4, 1))
    )

    mesh.compute_vertex_normals()

    return mesh


def make_axis_gizmo(size: float = AXIS_SIZE_M) -> o3d.geometry.TriangleMesh:
    return o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=size,
        origin=[0.0, 0.0, 0.0],
    )


# -----------------------------------------------------------------------------
# Height metrics helpers
# -----------------------------------------------------------------------------

def compute_object_height_metrics(points_local: np.ndarray, object_id: int) -> dict:
    pts = np.asarray(points_local, dtype=np.float64)
    n = len(pts)
    if n < HEIGHT_MIN_POINTS:
        return {"object_id": object_id, "valid": False, "point_count": n}
    z = pts[:, 2]
    centroid = pts.mean(axis=0)
    bbox_min = pts.min(axis=0)
    bbox_max = pts.max(axis=0)
    dims = bbox_max - bbox_min
    z_low = float(np.percentile(z, HEIGHT_LOW_PERCENTILE))
    z_high = float(np.percentile(z, HEIGHT_HIGH_PERCENTILE))
    z_top = float(np.percentile(z, HEIGHT_TOP_PERCENTILE))
    z_bottom = float(np.percentile(z, HEIGHT_BOTTOM_PERCENTILE))
    robust_h = z_high - z_low
    return {
        "object_id": object_id, "valid": True, "point_count": n,
        "centroid_x_m": float(centroid[0]), "centroid_y_m": float(centroid[1]), "centroid_z_m": float(centroid[2]),
        "bbox_width_x_m": float(dims[0]), "bbox_depth_y_m": float(dims[1]), "bbox_height_z_raw_m": float(dims[2]),
        "robust_height_m": robust_h, "robust_height_cm": robust_h * 100.0,
        "floor_relative_top_height_m": z_top, "floor_relative_top_height_cm": z_top * 100.0,
        "floor_relative_top_height_mm": z_top * 1000.0,
        "floor_relative_bottom_m": z_bottom,
    }


def print_height_metrics_table(label: str, metrics: list) -> None:
    print(f"\n[HEIGHT METRICS - {label}]")
    print(f"{'obj':>4} | {'pts':>6} | {'cx_cm':>6} | {'cy_cm':>6} | {'top_cm':>7} | {'robust_cm':>9} | {'bbox_x_cm':>9} | {'bbox_y_cm':>9}")
    print("-" * 75)
    for m in metrics:
        if not m.get("valid"):
            print(f"{m['object_id']:>4} | INVALID (pts={m['point_count']})")
            continue
        print(
            f"{m['object_id']:>4} | {m['point_count']:>6} | "
            f"{m['centroid_x_m']*100:>6.1f} | {m['centroid_y_m']*100:>6.1f} | "
            f"{m['floor_relative_top_height_cm']:>7.2f} | {m['robust_height_cm']:>9.2f} | "
            f"{m['bbox_width_x_m']*100:>9.2f} | {m['bbox_depth_y_m']*100:>9.2f}"
        )


def save_height_metrics(output_dir: Path, prefix: str, metrics: list) -> None:
    import csv
    json_path = output_dir / f"{prefix}.json"
    csv_path = output_dir / f"{prefix}.csv"
    json_path.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    if not metrics:
        return
    keys = list(metrics[0].keys())
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(metrics)
    print(f"[SAVE] {csv_path}")


# -----------------------------------------------------------------------------
# Workspace grid / axes helpers
# -----------------------------------------------------------------------------

def _make_grid_lineset(floor_corners_local: np.ndarray) -> o3d.geometry.LineSet:
    import math
    pts = np.asarray(floor_corners_local, dtype=np.float64)
    x0, x1 = float(pts[:, 0].min()), float(pts[:, 0].max())
    y0, y1 = float(pts[:, 1].min()), float(pts[:, 1].max())
    z = float(np.median(pts[:, 2]))
    spacing = WORKSPACE_GRID_SPACING_M

    points: list[list[float]] = []
    lines_list: list[list[int]] = []
    colors: list[list[float]] = []

    def add_line(p1: list[float], p2: list[float], color: list[float]) -> None:
        idx = len(points)
        points.append(p1)
        points.append(p2)
        lines_list.append([idx, idx + 1])
        colors.append(color)

    grid_color = [0.55, 0.58, 0.62]
    border_color = [0.9, 0.9, 0.4]

    xs = np.arange(math.ceil(x0 / spacing) * spacing, x1 + spacing * 0.01, spacing)
    for x in xs:
        add_line([float(x), y0, z], [float(x), y1, z], grid_color)
    ys = np.arange(math.ceil(y0 / spacing) * spacing, y1 + spacing * 0.01, spacing)
    for y in ys:
        add_line([x0, float(y), z], [x1, float(y), z], grid_color)
    bdr = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
    for i in range(4):
        a, b = bdr[i], bdr[(i + 1) % 4]
        add_line([a[0], a[1], z], [b[0], b[1], z], border_color)

    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(points)
    ls.lines = o3d.utility.Vector2iVector(lines_list)
    ls.colors = o3d.utility.Vector3dVector(colors)
    return ls


def _make_axes_lineset() -> o3d.geometry.LineSet:
    L = WORKSPACE_AXIS_LENGTH_M
    points = [[0, 0, 0], [L, 0, 0], [0, 0, 0], [0, L, 0], [0, 0, 0], [0, 0, L]]
    lines_idx = [[0, 1], [2, 3], [4, 5]]
    colors = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(points)
    ls.lines = o3d.utility.Vector2iVector(lines_idx)
    ls.colors = o3d.utility.Vector3dVector(colors)
    return ls


# -----------------------------------------------------------------------------
# Presentation mesh helpers
# -----------------------------------------------------------------------------

def _clean_cloud_for_presentation(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    out = pcd
    if ENABLE_RADIUS_OUTLIER_REMOVAL and len(out.points) >= RADIUS_OUTLIER_NB_POINTS:
        out, _ = out.remove_radius_outlier(nb_points=RADIUS_OUTLIER_NB_POINTS, radius=RADIUS_OUTLIER_RADIUS_M)
    if ENABLE_NORMAL_ESTIMATION and len(out.points) > 0:
        out.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=NORMAL_RADIUS_M, max_nn=NORMAL_MAX_NN))
        out.orient_normals_consistent_tangent_plane(30)
    return out


def _save_presentation_outputs(output_dir: Path, prefix: str, pcd: o3d.geometry.PointCloud) -> None:
    if not ENABLE_PRESENTATION_MESH_OUTPUTS:
        return
    try:
        clean = _clean_cloud_for_presentation(pcd)
        o3d.io.write_point_cloud(str(output_dir / f"{prefix}_cleaned_cloud.ply"), clean)
        print(f"[SAVE] {prefix}_cleaned_cloud.ply")
    except Exception as exc:
        print(f"[WARN] Presentation clean cloud failed: {exc}")
        return
    if ENABLE_MESH_BALL_PIVOTING:
        try:
            if not clean.has_normals():
                clean.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=NORMAL_RADIUS_M, max_nn=NORMAL_MAX_NN))
            radii = o3d.utility.DoubleVector(BALL_PIVOT_RADII_M)
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(clean, radii)
            if len(mesh.triangles) > 0:
                if ENABLE_MESH_SMOOTHING:
                    mesh = mesh.filter_smooth_taubin(number_of_iterations=MESH_SMOOTHING_ITERATIONS)
                mesh.compute_vertex_normals()
                o3d.io.write_triangle_mesh(str(output_dir / f"{prefix}_mesh_ball_pivot.ply"), mesh)
                print(f"[SAVE] {prefix}_mesh_ball_pivot.ply")
        except Exception as exc:
            print(f"[WARN] Presentation mesh failed: {exc}")


# -----------------------------------------------------------------------------
# Tkinter control panel
# -----------------------------------------------------------------------------


class MergeTunerControlPanel:
    """Tkinter-based control panel replacing OpenCV trackbars."""

    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("Merge Tuner Controls")
        self.root.resizable(False, False)

        self.flag_recompute = False
        self.flag_reset_view = False
        self.flag_save = False
        self.flag_save_best = False
        self.flag_quit = False

        self.root.protocol("WM_DELETE_WINDOW", self._on_quit)

        frame = ttk.Frame(self.root, padding=10)
        frame.grid(row=0, column=0, sticky="nsew")

        row = 0

        self._view_icp = tk.BooleanVar(value=True)
        self._use_icp = tk.BooleanVar(value=True)
        self._trans_only = tk.BooleanVar(value=False)
        self._colored_icp = tk.BooleanVar(value=False)
        self._auto_recompute = tk.BooleanVar(value=True)

        def cb(text, var):
            nonlocal row
            ttk.Checkbutton(frame, text=text, variable=var).grid(row=row, column=0, columnspan=3, sticky="w", pady=1)
            row += 1

        cb("Show ICP result (vs corners-only)", self._view_icp)
        cb("Use ICP", self._use_icp)
        cb("Translation-only ICP", self._trans_only)
        cb("Use colored ICP (experimental)", self._colored_icp)
        cb("Auto recompute on slider change (0.25 s debounce)", self._auto_recompute)

        ttk.Separator(frame, orient="horizontal").grid(row=row, column=0, columnspan=3, sticky="ew", pady=6)
        row += 1

        self._sliders = {}
        slider_defs = [
            ("ICP threshold [mm]",    "icp_thresh",  1,    60,   6),
            ("ICP voxel size [mm]",   "icp_voxel",   1,    20,   3),
            ("Max yaw rotation [deg]","max_yaw",      0,    45,   5),
            ("Final voxel size [mm]", "final_voxel", 1,    20,   2),
            ("X shift [mm]",          "x_shift",   -100,  100,   0),
            ("Y shift [mm]",          "y_shift",   -100,  100,   0),
            ("Z shift [mm]",          "z_shift",   -100,  100,   0),
            ("Z scale [%]",           "z_scale",     50,  150, 100),
        ]
        for label, key, lo, hi, default in slider_defs:
            var = tk.IntVar(value=default)
            self._sliders[key] = var
            ttk.Label(frame, text=label, width=30, anchor="w").grid(row=row, column=0, sticky="w")
            ttk.Scale(frame, from_=lo, to=hi, orient="horizontal", variable=var, length=200,
                      command=lambda _v, k=key: self._on_slider_change()).grid(row=row, column=1, padx=4)
            ttk.Label(frame, textvariable=var, width=5).grid(row=row, column=2, sticky="w")
            row += 1

        ttk.Separator(frame, orient="horizontal").grid(row=row, column=0, columnspan=3, sticky="ew", pady=6)
        row += 1

        self._status_var = tk.StringVar(value="Ready.")
        ttk.Label(frame, textvariable=self._status_var, foreground="#007acc", wraplength=420, justify="left").grid(
            row=row, column=0, columnspan=3, sticky="w", pady=(0, 6))
        row += 1

        btn_frame = ttk.Frame(frame)
        btn_frame.grid(row=row, column=0, columnspan=3, pady=4)

        ttk.Button(btn_frame, text="Recompute  [r]",       command=self._on_recompute).pack(side="left", padx=4)
        ttk.Button(btn_frame, text="Reset View  [v]",       command=self._on_reset_view).pack(side="left", padx=4)
        ttk.Button(btn_frame, text="Save Auto + Run  [s]",  command=self._on_save).pack(side="left", padx=4)
        ttk.Button(btn_frame, text="Save Best Config  [b]", command=self._on_save_best).pack(side="left", padx=4)
        ttk.Button(btn_frame, text="Quit  [q]",             command=self._on_quit).pack(side="left", padx=4)

        self._last_slider_change = 0.0

    def _on_slider_change(self):
        self._last_slider_change = time.time()

    def _on_recompute(self):
        self.flag_recompute = True

    def _on_reset_view(self):
        self.flag_reset_view = True

    def _on_save(self):
        self.flag_save = True

    def _on_save_best(self):
        self.flag_save_best = True

    def _on_quit(self):
        self.flag_quit = True

    def get_params(self):
        return TunerParams(
            use_icp=self._use_icp.get(),
            translation_only=self._trans_only.get(),
            use_colored_icp=self._colored_icp.get(),
            max_yaw_deg=float(self._sliders["max_yaw"].get()),
            icp_threshold_m=max(1, self._sliders["icp_thresh"].get()) / 1000.0,
            icp_voxel_m=max(1, self._sliders["icp_voxel"].get()) / 1000.0,
            final_voxel_m=max(1, self._sliders["final_voxel"].get()) / 1000.0,
            x_shift_m=self._sliders["x_shift"].get() / 1000.0,
            y_shift_m=self._sliders["y_shift"].get() / 1000.0,
            z_shift_m=self._sliders["z_shift"].get() / 1000.0,
            z_scale=max(50, self._sliders["z_scale"].get()) / 100.0,
            view_mode_icp=self._view_icp.get(),
        )

    def auto_recompute_due(self, debounce_s=0.25):
        if not self._auto_recompute.get():
            return False
        return (time.time() - self._last_slider_change) >= debounce_s and self._last_slider_change > 0

    def consume_auto_recompute(self):
        self._last_slider_change = 0.0

    def set_status(self, text):
        self._status_var.set(text)

    def update(self):
        try:
            self.root.update()
        except Exception:
            self.flag_quit = True


def params_key(params):
    return (
        params.use_icp,
        params.translation_only,
        params.use_colored_icp,
        round(params.max_yaw_deg, 4),
        round(params.icp_threshold_m, 5),
        round(params.icp_voxel_m, 5),
        round(params.final_voxel_m, 5),
        round(params.x_shift_m, 5),
        round(params.y_shift_m, 5),
        round(params.z_shift_m, 5),
        round(params.z_scale, 4),
        params.view_mode_icp,
    )


# -----------------------------------------------------------------------------
# Main interactive loop
# -----------------------------------------------------------------------------

def run_tuner(run_dir):
    dataset = load_cached_dataset(run_dir)
    tuned_dir = run_dir / "interactive_tuned_outputs"
    tuned_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 72)
    print("INTERACTIVE MERGE TUNER")
    print("=" * 72)
    print(f"[RUN] {run_dir}")
    print(f"[OBJECT IDS] {dataset['object_ids']}")
    print("[KEYS] r recompute | v reset view | s save run output + auto config | b save best config | q/Esc quit")
    print("[IMPORTANT] Slider changes do NOT reset the Open3D view. Only v / Reset View button resets it.")

    panel = MergeTunerControlPanel()

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=VIEW_WINDOW, width=1200, height=800, visible=True)

    render_opt = vis.get_render_option()
    render_opt.background_color = np.array([0.02, 0.03, 0.04])
    render_opt.point_size = POINT_SIZE
    render_opt.show_coordinate_frame = True
    render_opt.mesh_show_back_face = True

    floor_mesh = make_floor_mesh(dataset["floor_ref_local"])
    axis_gizmo = make_axis_gizmo(size=AXIS_SIZE_M)
    grid_ls = _make_grid_lineset(dataset["floor_ref_local"])
    axes_ls = _make_axes_lineset()

    static_added = False
    active_cloud = None
    view_initialized = False

    last_pk = None
    last_params = None
    last_debug = {}
    last_corner = None
    last_icp = None
    last_per_obj_corner = []
    last_per_obj_icp = []
    last_recompute_time = 0.0

    def reset_view_to_top():
        if active_cloud is None or len(active_cloud.points) == 0:
            return
        bbox = active_cloud.get_axis_aligned_bounding_box()
        center = bbox.get_center()
        ctr = vis.get_view_control()
        ctr.set_lookat(center)
        ctr.set_front(VIEW_FRONT)
        ctr.set_up(VIEW_UP)
        ctr.set_zoom(VIEW_ZOOM)
        vis.poll_events()
        vis.update_renderer()

    def show_cloud(pcd):
        nonlocal active_cloud, static_added, view_initialized
        if active_cloud is not None:
            vis.remove_geometry(active_cloud, reset_bounding_box=False)
        if not static_added:
            vis.add_geometry(floor_mesh, reset_bounding_box=False)
            vis.add_geometry(grid_ls, reset_bounding_box=False)
            vis.add_geometry(axes_ls, reset_bounding_box=False)
            vis.add_geometry(axis_gizmo, reset_bounding_box=False)
            static_added = True
        active_cloud = o3d.geometry.PointCloud(pcd)
        pts = np.asarray(active_cloud.points)
        print(f"[VIEW] {len(pts)} points")
        first_render = not view_initialized
        vis.add_geometry(active_cloud, reset_bounding_box=first_render)
        vis.update_geometry(active_cloud)
        vis.update_geometry(floor_mesh)
        vis.update_geometry(grid_ls)
        vis.update_geometry(axes_ls)
        vis.update_geometry(axis_gizmo)
        if first_render and len(pts) > 0:
            reset_view_to_top()
            view_initialized = True

    def do_recompute(p):
        nonlocal last_corner, last_icp, last_per_obj_corner, last_per_obj_icp, last_debug
        nonlocal last_params, last_pk, last_recompute_time
        print(
            f"[RECOMPUTE] view={'ICP' if p.view_mode_icp else 'corners'} "
            f"icp={p.use_icp} colored={p.use_colored_icp} "
            f"th={p.icp_threshold_m*1000:.1f}mm vox={p.icp_voxel_m*1000:.1f}mm "
            f"yaw={p.max_yaw_deg:.1f}deg "
            f"shift=({p.x_shift_m*1000:.1f},{p.y_shift_m*1000:.1f},{p.z_shift_m*1000:.1f})mm "
            f"zscale={p.z_scale:.2f}"
        )
        last_corner, last_icp, last_per_obj_corner, last_per_obj_icp, last_debug = recompute_merge(dataset, p)
        show_pcd = last_icp if p.view_mode_icp else last_corner
        show_cloud(show_pcd)
        last_params = p
        last_pk = params_key(p)
        last_recompute_time = time.time()
        panel.set_status(
            f"ICP thr={p.icp_threshold_m*1000:.0f}mm | vox={p.icp_voxel_m*1000:.0f}mm | "
            f"yaw={p.max_yaw_deg:.0f}deg | z scale={p.z_scale*100:.0f}%"
        )

    def do_save(p):
        if last_corner is None or last_icp is None:
            print("[WARN] Nothing to save yet.")
            panel.set_status("Nothing to save yet.")
            return
        corner_path = tuned_dir / "tuned_all_objects_corners_only.ply"
        icp_path = tuned_dir / "tuned_all_objects_limited_icp.ply"
        corner_scene = tuned_dir / "tuned_scene_corners_only_with_plane.ply"
        icp_scene = tuned_dir / "tuned_scene_limited_icp_with_plane.ply"
        debug_path = tuned_dir / "tuned_params_debug.json"
        o3d.io.write_point_cloud(str(corner_path), last_corner)
        o3d.io.write_point_cloud(str(icp_path), last_icp)
        write_scene_with_floor_ply(corner_scene, dataset["floor_ref_local"], last_corner)
        write_scene_with_floor_ply(icp_scene, dataset["floor_ref_local"], last_icp)
        corner_metrics = [compute_object_height_metrics(np.asarray(pc.points), i) for i, pc in enumerate(last_per_obj_corner)]
        icp_metrics = [compute_object_height_metrics(np.asarray(pc.points), i) for i, pc in enumerate(last_per_obj_icp)]
        print_height_metrics_table("TUNED CORNERS ONLY", corner_metrics)
        print_height_metrics_table("TUNED LIMITED ICP", icp_metrics)
        save_height_metrics(tuned_dir, "tuned_object_height_metrics_corners_only", corner_metrics)
        save_height_metrics(tuned_dir, "tuned_object_height_metrics_limited_icp", icp_metrics)
        run_debug_payload = {
            "params": p.__dict__,
            "debug": last_debug,
            "height_metrics": {"corners_only": corner_metrics, "limited_icp": icp_metrics},
            "outputs": {"corners_only": str(corner_path), "limited_icp": str(icp_path),
                        "scene_corners_only": str(corner_scene), "scene_limited_icp": str(icp_scene)},
        }
        save_json(debug_path, run_debug_payload)
        _save_presentation_outputs(tuned_dir, "tuned_all_objects", last_icp)
        ICP_MERGE_CONFIG_AUTO_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        auto_path = ICP_MERGE_CONFIG_AUTO_DIR / f"icp_merge_config_{ts}.json"
        auto_payload = build_config_json(run_dir, p, last_debug)
        auto_payload["height_metrics"] = {"corners_only": corner_metrics, "limited_icp": icp_metrics}
        save_json(auto_path, auto_payload)
        print("[SAVED]")
        print(f"  {corner_scene}")
        print(f"  {icp_scene}")
        print(f"  {auto_path}")
        panel.set_status(f"Saved: {ts}")

    def do_save_best(p):
        icp_metrics = [compute_object_height_metrics(np.asarray(pc.points), i) for i, pc in enumerate(last_per_obj_icp)]
        ICP_MERGE_CONFIG_BEST_DIR.mkdir(parents=True, exist_ok=True)
        best_path = ICP_MERGE_CONFIG_BEST_DIR / "tuned_params_debug.json"
        best_payload = build_config_json(run_dir, p, last_debug)
        best_payload["height_metrics"] = {"limited_icp": icp_metrics}
        save_json(best_path, best_payload)
        print(f"[BEST CONFIG SAVED] {best_path}")
        panel.set_status(f"Best config saved: {best_path.name}")

    try:
        while True:
            panel.update()
            if panel.flag_quit:
                break
            p = panel.get_params()
            pk = params_key(p)
            now = time.time()
            if panel.auto_recompute_due():
                panel.consume_auto_recompute()
                if pk != last_pk and (now - last_recompute_time) > 0.25:
                    do_recompute(p)
            if panel.flag_recompute:
                panel.flag_recompute = False
                do_recompute(p)
            if panel.flag_reset_view:
                panel.flag_reset_view = False
                reset_view_to_top()
            if panel.flag_save:
                panel.flag_save = False
                if last_params is not None:
                    do_save(last_params)
                else:
                    print("[WARN] Recompute first.")
                    panel.set_status("Recompute first.")
            if panel.flag_save_best:
                panel.flag_save_best = False
                if last_params is not None:
                    do_save_best(last_params)
                else:
                    print("[WARN] Recompute first.")
                    panel.set_status("Recompute first.")
            vis.poll_events()
            vis.update_renderer()
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord("r"):
                do_recompute(panel.get_params())
            elif key == ord("v"):
                reset_view_to_top()
            elif key == ord("s"):
                if last_params is not None:
                    do_save(last_params)
            elif key == ord("b"):
                if last_params is not None:
                    do_save_best(last_params)
    finally:
        vis.destroy_window()
        try:
            panel.root.destroy()
        except Exception:
            pass
        cv2.destroyAllWindows()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Interactive tuner for cached two-view reconstruction runs."
    )
    parser.add_argument("--run-dir", type=Path, default=None)
    parser.add_argument("--root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    return parser.parse_args()


def main():
    args = parse_args()
    run_dir = args.run_dir.resolve() if args.run_dir else latest_run_dir(args.root.resolve())
    run_tuner(run_dir)


if __name__ == "__main__":
    main()