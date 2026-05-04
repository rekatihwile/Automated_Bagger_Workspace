"""
Interactive merge tuner for the one-pass two-view pipeline.

Save as:
    workspace/scripts/offline/interactive_merge_tuner_from_cache.py

Run latest cached capture:
    python .\interactive_merge_tuner_from_cache.py

Run a specific capture:
    python .\interactive_merge_tuner_from_cache.py --run-dir "C:\path\to\pipeline_YYYYMMDD_HHMMSS"

Controls:
    r = recompute, KEEP CURRENT VIEW
    v = reset view manually
    s = save tuned outputs
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
) -> tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud, dict[str, Any]]:
    ref_corners = dataset["ref_corners"]
    src_corners = dataset["src_corners"]
    ref_frame = dataset["ref_frame"]

    corner_clouds = []
    icp_clouds = []

    debug = {
        "objects": [],
    }

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

        corner_pair = merge_clouds([ref_pcd, src_corner_pcd])
        corner_clouds.append(corner_pair)

        src_icp_pcd, icp_info = run_limited_icp(
            src_corner_pcd,
            ref_pcd,
            params,
        )

        icp_pair = merge_clouds([ref_pcd, src_icp_pcd])
        icp_clouds.append(icp_pair)

        debug["objects"].append(
            {
                "object_id": object_id,
                "icp": icp_info,
            }
        )

    corner_merged = merge_clouds(corner_clouds)
    icp_merged = merge_clouds(icp_clouds)

    corner_merged = safe_downsample(
        safe_remove_outliers(corner_merged),
        params.final_voxel_m,
    )

    icp_merged = safe_downsample(
        safe_remove_outliers(icp_merged),
        params.final_voxel_m,
    )

    return corner_merged, icp_merged, debug


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
# Trackbar controls
# -----------------------------------------------------------------------------

def setup_controls() -> None:
    cv2.namedWindow(CONTROL_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROL_WINDOW, 520, 520)

    cv2.createTrackbar("view ICP", CONTROL_WINDOW, 1, 1, lambda x: None)
    cv2.createTrackbar("use ICP", CONTROL_WINDOW, 1, 1, lambda x: None)
    cv2.createTrackbar("translation only", CONTROL_WINDOW, 0, 1, lambda x: None)
    cv2.createTrackbar("ICP threshold mm", CONTROL_WINDOW, 6, 60, lambda x: None)
    cv2.createTrackbar("ICP voxel mm", CONTROL_WINDOW, 3, 20, lambda x: None)
    cv2.createTrackbar("max yaw deg", CONTROL_WINDOW, 5, 45, lambda x: None)
    cv2.createTrackbar("final voxel mm", CONTROL_WINDOW, 2, 20, lambda x: None)
    cv2.createTrackbar("x shift mm +100", CONTROL_WINDOW, 100, 200, lambda x: None)
    cv2.createTrackbar("y shift mm +100", CONTROL_WINDOW, 100, 200, lambda x: None)
    cv2.createTrackbar("z shift mm +100", CONTROL_WINDOW, 100, 200, lambda x: None)
    cv2.createTrackbar("z scale percent", CONTROL_WINDOW, 100, 150, lambda x: None)


def read_params() -> TunerParams:
    threshold_mm = max(1, cv2.getTrackbarPos("ICP threshold mm", CONTROL_WINDOW))
    icp_voxel_mm = max(1, cv2.getTrackbarPos("ICP voxel mm", CONTROL_WINDOW))
    final_voxel_mm = max(1, cv2.getTrackbarPos("final voxel mm", CONTROL_WINDOW))
    z_scale_percent = max(50, cv2.getTrackbarPos("z scale percent", CONTROL_WINDOW))

    return TunerParams(
        use_icp=bool(cv2.getTrackbarPos("use ICP", CONTROL_WINDOW)),
        translation_only=bool(cv2.getTrackbarPos("translation only", CONTROL_WINDOW)),
        max_yaw_deg=float(cv2.getTrackbarPos("max yaw deg", CONTROL_WINDOW)),
        icp_threshold_m=threshold_mm / 1000.0,
        icp_voxel_m=icp_voxel_mm / 1000.0,
        final_voxel_m=final_voxel_mm / 1000.0,
        x_shift_m=(cv2.getTrackbarPos("x shift mm +100", CONTROL_WINDOW) - 100) / 1000.0,
        y_shift_m=(cv2.getTrackbarPos("y shift mm +100", CONTROL_WINDOW) - 100) / 1000.0,
        z_shift_m=(cv2.getTrackbarPos("z shift mm +100", CONTROL_WINDOW) - 100) / 1000.0,
        z_scale=z_scale_percent / 100.0,
        view_mode_icp=bool(cv2.getTrackbarPos("view ICP", CONTROL_WINDOW)),
    )


def params_key(params: TunerParams) -> tuple:
    return (
        params.use_icp,
        params.translation_only,
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

def run_tuner(run_dir: Path) -> None:
    dataset = load_cached_dataset(run_dir)
    tuned_dir = run_dir / "interactive_tuned_outputs"
    tuned_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 72)
    print("INTERACTIVE MERGE TUNER")
    print("=" * 72)
    print(f"[RUN] {run_dir}")
    print(f"[OBJECT IDS] {dataset['object_ids']}")
    print("[KEYS] r recompute | v reset view | s save run output + auto config | b save best config | q/Esc quit")
    print("[IMPORTANT] Slider changes and r DO NOT reset the view.")

    setup_controls()

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name=VIEW_WINDOW,
        width=1200,
        height=800,
        visible=True,
    )

    render = vis.get_render_option()
    render.background_color = np.array([0.02, 0.03, 0.04])
    render.point_size = POINT_SIZE
    render.show_coordinate_frame = True
    render.mesh_show_back_face = True

    floor_mesh = make_floor_mesh(dataset["floor_ref_local"])
    axis_gizmo = make_axis_gizmo(size=AXIS_SIZE_M)

    active_cloud: o3d.geometry.PointCloud | None = None
    active_floor_added = False
    active_axis_added = False
    view_initialized = False

    last_key = None
    last_params: TunerParams | None = None
    last_debug: dict[str, Any] = {}
    last_corner: o3d.geometry.PointCloud | None = None
    last_icp: o3d.geometry.PointCloud | None = None

    last_recompute_time = 0.0

    def reset_view_to_top() -> None:
        if active_cloud is None:
            return

        pts = np.asarray(active_cloud.points)
        if len(pts) == 0:
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

    def show_cloud(pcd: o3d.geometry.PointCloud) -> None:
        nonlocal active_cloud, active_floor_added, active_axis_added, view_initialized

        if active_cloud is not None:
            vis.remove_geometry(active_cloud, reset_bounding_box=False)

        if not active_floor_added:
            vis.add_geometry(floor_mesh, reset_bounding_box=False)
            active_floor_added = True

        if not active_axis_added:
            vis.add_geometry(axis_gizmo, reset_bounding_box=False)
            active_axis_added = True

        active_cloud = o3d.geometry.PointCloud(pcd)

        pts = np.asarray(active_cloud.points)
        print(f"[VIEW] showing {len(pts)} points")

        if len(pts) > 0:
            print(f"[VIEW] bbox min={pts.min(axis=0)} max={pts.max(axis=0)}")

        # First render only: let Open3D establish scene bounds.
        # After that: never touch the camera unless user presses v.
        first_render = not view_initialized

        vis.add_geometry(active_cloud, reset_bounding_box=first_render)
        vis.update_geometry(active_cloud)
        vis.update_geometry(floor_mesh)
        vis.update_geometry(axis_gizmo)

        if first_render and len(pts) > 0:
            reset_view_to_top()
            view_initialized = True

    try:
        while True:
            params = read_params()
            key_tuple = params_key(params)
            now = time.time()

            should_recompute = last_key is None or key_tuple != last_key

            if should_recompute and (now - last_recompute_time) > 0.25:
                print(
                    f"[RECOMPUTE] view={'ICP' if params.view_mode_icp else 'corners'} "
                    f"use_icp={params.use_icp} "
                    f"th={params.icp_threshold_m * 1000:.1f}mm "
                    f"vox={params.icp_voxel_m * 1000:.1f}mm "
                    f"yaw={params.max_yaw_deg:.1f}deg "
                    f"shift=({params.x_shift_m * 1000:.1f},"
                    f"{params.y_shift_m * 1000:.1f},"
                    f"{params.z_shift_m * 1000:.1f})mm "
                    f"zscale={params.z_scale:.2f}"
                )

                last_corner, last_icp, last_debug = recompute_merge(dataset, params)
                show_pcd = last_icp if params.view_mode_icp else last_corner

                show_cloud(show_pcd)

                last_key = key_tuple
                last_params = params
                last_recompute_time = now

            vis.poll_events()
            vis.update_renderer()

            key = cv2.waitKey(30) & 0xFF

            if key in (ord("q"), 27):
                break

            if key == ord("r"):
                # Recompute only. View stays pinned.
                last_key = None

            if key == ord("v"):
                # Explicit manual view reset.
                reset_view_to_top()

            if key == ord("s"):
                if last_corner is None or last_icp is None or last_params is None:
                    print("[WARN] Nothing to save yet.")
                    continue

                corner_path = tuned_dir / "tuned_all_objects_corners_only.ply"
                icp_path = tuned_dir / "tuned_all_objects_limited_icp.ply"
                corner_scene = tuned_dir / "tuned_scene_corners_only_with_plane.ply"
                icp_scene = tuned_dir / "tuned_scene_limited_icp_with_plane.ply"
                debug_path = tuned_dir / "tuned_params_debug.json"

                o3d.io.write_point_cloud(str(corner_path), last_corner)
                o3d.io.write_point_cloud(str(icp_path), last_icp)

                write_scene_with_floor_ply(
                    corner_scene,
                    dataset["floor_ref_local"],
                    last_corner,
                )

                write_scene_with_floor_ply(
                    icp_scene,
                    dataset["floor_ref_local"],
                    last_icp,
                )

                run_debug_payload = {
                    "params": last_params.__dict__,
                    "debug": last_debug,
                    "outputs": {
                        "corners_only": str(corner_path),
                        "limited_icp": str(icp_path),
                        "scene_corners_only": str(corner_scene),
                        "scene_limited_icp": str(icp_scene),
                    },
                }
                save_json(debug_path, run_debug_payload)

                # Also save a copy to the global auto config folder.
                ICP_MERGE_CONFIG_AUTO_DIR.mkdir(parents=True, exist_ok=True)
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                auto_path = ICP_MERGE_CONFIG_AUTO_DIR / f"icp_merge_config_{ts}.json"
                save_json(
                    auto_path,
                    build_config_json(run_dir, last_params, last_debug),
                )

                print("[SAVED]")
                print(f"  {corner_scene}")
                print(f"  {icp_scene}")
                print(f"  {auto_path}")

            if key == ord("b"):
                if last_params is None:
                    print("[WARN] Nothing to save yet.")
                    continue

                ICP_MERGE_CONFIG_BEST_DIR.mkdir(parents=True, exist_ok=True)
                best_path = ICP_MERGE_CONFIG_BEST_DIR / "tuned_params_debug.json"
                save_json(
                    best_path,
                    build_config_json(run_dir, last_params, last_debug),
                )
                print(f"[BEST CONFIG SAVED] {best_path}")

    finally:
        vis.destroy_window()
        cv2.destroyWindow(CONTROL_WINDOW)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactive tuner for cached two-view reconstruction runs."
    )

    parser.add_argument(
        "--run-dir",
        type=Path,
        default=None,
        help="Specific pipeline_YYYYMMDD_HHMMSS folder to tune.",
    )

    parser.add_argument(
        "--root",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
        help="Root folder containing pipeline_* runs.",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_dir = args.run_dir.resolve() if args.run_dir else latest_run_dir(args.root.resolve())
    run_tuner(run_dir)


if __name__ == "__main__":
    main()