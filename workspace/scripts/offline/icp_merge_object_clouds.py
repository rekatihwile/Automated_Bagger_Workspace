"""
Multi-object corner-warp + limited-ICP merge for ArUco/ChArUco workspace point clouds.

No command-line arguments.
Edit CONFIG below, then run:

    python .\corner_warp_multi_object_config.py

Goal:
    Process multiple object clouds object-by-object.

Assumption:
    You masked objects in the same order for each scan.
    Therefore object_00 in scan A corresponds to object_00 in scan B,
    object_01 corresponds to object_01, etc.

Pipeline:
    1. Find run folders.
    2. Load metadata.json from each run.
    3. Read marker corners from metadata.
    4. Find object IDs from metadata and/or PLY filenames.
    5. For each object ID:
        a. Load that object's cloud from every run.
        b. Corner-warp each cloud into the reference run's marker frame.
        c. Optionally run small limited ICP against the same object ID in the reference run.
        d. Save per-run warped object clouds.
        e. Save merged object cloud.
    6. Save one all-object scene with reference floor.
"""

from __future__ import annotations

import copy
import json
import re
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d


# =============================================================================
# CONFIG — EDIT THESE
# =============================================================================

ROOT_RUNS_DIR = Path(
    r"C:\Users\elipp\OneDrive\Documents\Grocery Bagger\calibration\Same_Set"
)

OUTPUT_DIR = ROOT_RUNS_DIR / "corner_warp_multi_object_output"

USE_MANUAL_RUN_DIR_LIST = False

MANUAL_RUN_DIRS = [
    Path(r"C:\Users\elipp\OneDrive\Documents\Grocery Bagger\calibration\Same_Set\run_20260504_120307"),
    Path(r"C:\Users\elipp\OneDrive\Documents\Grocery Bagger\calibration\Same_Set\run_20260504_120737"),
]

REFERENCE_INDEX = 0

# Set to None to auto-detect object IDs.
# Or manually use something like [0, 1, 2].
OBJECT_IDS_TO_PROCESS: list[int] | None = None

# If True, only process object IDs present in every run.
# If False, process anything found, skipping missing clouds.
REQUIRE_OBJECT_IN_ALL_RUNS = True

# Best case: use camera-frame clouds because the marker corners and object cloud
# are still tied together in the same camera coordinate frame.
CLOUD_COORDINATE_FRAME = "camera"  # "camera" or "workspace"

# Object cloud filename priority.
# The {id:02d} part becomes 00, 01, 02, etc.
OBJECT_FILE_PATTERNS_BY_ID = [
    "object_{id:02d}_cloud_camera_debug.ply",
    "object_{id:02d}_cloud.ply",
    "object_{id:02d}_cloud_workspace.ply",
    "object_{id:02d}_cloud_workspace_aligned_to_reference.ply",
]

CORNER_ORDER = ["TL", "TR", "BR", "BL"]

# Marker/corner warp.
#   "homography_plane" forces 4 corners to match exactly in the plane.
#   "similarity_3d" uses rotation + translation + uniform scale.
WARP_MODE = "homography_plane"

# For homography mode:
#   "average_xy" scales object height by average XY marker scale.
#   "none" preserves Z.
Z_SCALE_MODE = "average_xy"

# Optional global adjustment applied after corner warp and ICP.
# This applies to EVERY object, same shift.
GLOBAL_FINAL_SHIFT_M = np.array([0.0, 0.0, 0.0], dtype=np.float64)

# Cleaning.
ENABLE_PERCENTILE_CROP = True
PERCENTILE_LOW = 0.5
PERCENTILE_HIGH = 99.5

ENABLE_STATISTICAL_OUTLIER_REMOVAL = True
OUTLIER_NB_NEIGHBORS = 20
OUTLIER_STD_RATIO = 2.5

# Final output cleanup.
VOXEL_SIZE_FINAL_PER_OBJECT = 0.0015
VOXEL_SIZE_FINAL_ALL_OBJECTS = 0.0015

FINAL_OUTLIER_NB_NEIGHBORS = 25
FINAL_OUTLIER_STD_RATIO = 2.0

# =============================================================================
# LIMITED ICP SETTINGS — APPLIED OBJECT-BY-OBJECT
# =============================================================================

RUN_SMALL_ICP_AFTER_CORNER_WARP = True

# If True, ICP only applies translation and cannot rotate.
ICP_ONLY_TRANSLATION = False

# If True, clamp ICP yaw / Z rotation.
LIMIT_ICP_Z_ROTATION = True
MAX_ICP_Z_ROTATION_DEG = 5.0

ICP_DISTANCE_THRESHOLD = 0.006
ICP_VOXEL_SIZE = 0.003

ICP_METHOD = "point_to_point"  # "point_to_point" or "point_to_plane"
ALLOW_SCALE_IN_ICP = False

REJECT_BAD_ICP = True
MIN_ACCEPTABLE_ICP_FITNESS = 0.05
MAX_ACCEPTABLE_ICP_RMSE = 0.05
MAX_ICP_TRANSLATION_M = 0.04
MAX_ICP_TOTAL_ROTATION_DEG = 15.0

# Output/debug.
SAVE_PER_SCAN_OUTPUTS = True
SAVE_REFERENCE_FLOOR = True
SAVE_MERGED_SCENE_WITH_FLOOR = True
SAVE_DEBUG_JSON = True
PRINT_VERBOSE = True


# =============================================================================
# BASIC HELPERS
# =============================================================================

def log(msg: str) -> None:
    if PRINT_VERBOSE:
        print(msg)


def copy_cloud(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    return copy.deepcopy(pcd)


def find_run_dirs() -> list[Path]:
    if USE_MANUAL_RUN_DIR_LIST:
        run_dirs = [p for p in MANUAL_RUN_DIRS if (p / "metadata.json").exists()]
        missing = [p for p in MANUAL_RUN_DIRS if not (p / "metadata.json").exists()]
        for p in missing:
            print(f"[WARN] Manual run dir missing metadata.json: {p}")
    else:
        run_dirs = sorted(
            p for p in ROOT_RUNS_DIR.iterdir()
            if p.is_dir() and (p / "metadata.json").exists()
        )

    if not run_dirs:
        raise FileNotFoundError(f"No run folders with metadata.json found in:\n{ROOT_RUNS_DIR}")

    return run_dirs


def load_metadata(run_dir: Path) -> dict:
    metadata_path = run_dir / "metadata.json"
    if not metadata_path.exists():
        raise FileNotFoundError(f"Missing metadata.json: {metadata_path}")

    return json.loads(metadata_path.read_text(encoding="utf-8"))


def read_cloud(path: Path) -> o3d.geometry.PointCloud:
    pcd = o3d.io.read_point_cloud(str(path))

    if pcd.is_empty() or len(pcd.points) == 0:
        raise ValueError(f"Loaded empty point cloud: {path}")

    return pcd


def save_cloud(path: Path, pcd: o3d.geometry.PointCloud) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok = o3d.io.write_point_cloud(str(path), pcd)

    if ok:
        log(f"[SAVE] {path}")
    else:
        print(f"[WARN] Failed to save cloud: {path}")


def cloud_stats(label: str, pcd: o3d.geometry.PointCloud) -> dict:
    pts = np.asarray(pcd.points)

    if len(pts) == 0:
        print(f"[STATS] {label}: EMPTY")
        return {"label": label, "points": 0}

    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    center = pts.mean(axis=0)
    dims = maxs - mins

    print(
        f"[STATS] {label}: "
        f"pts={len(pts)}, "
        f"center=({center[0]:+.4f}, {center[1]:+.4f}, {center[2]:+.4f}), "
        f"dims=({dims[0]:.4f}, {dims[1]:.4f}, {dims[2]:.4f})"
    )

    return {
        "label": label,
        "points": int(len(pts)),
        "min_xyz": mins.tolist(),
        "max_xyz": maxs.tolist(),
        "center_xyz": center.tolist(),
        "dims_xyz": dims.tolist(),
    }


def add_global_shift(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    shifted = copy_cloud(pcd)
    pts = np.asarray(shifted.points, dtype=np.float64)

    if len(pts) > 0:
        pts = pts + GLOBAL_FINAL_SHIFT_M[None, :]
        shifted.points = o3d.utility.Vector3dVector(pts)

    return shifted


# =============================================================================
# OBJECT ID / CLOUD DISCOVERY
# =============================================================================

_OBJECT_FILE_RE = re.compile(r"object_(\d+)_cloud.*\.ply$", re.IGNORECASE)


def object_ids_from_metadata(metadata: dict) -> set[int]:
    ids: set[int] = set()

    objects = metadata.get("objects", [])
    if isinstance(objects, list):
        for obj in objects:
            if isinstance(obj, dict) and "object_id" in obj:
                try:
                    ids.add(int(obj["object_id"]))
                except Exception:
                    pass

    return ids


def object_ids_from_files(run_dir: Path) -> set[int]:
    ids: set[int] = set()

    for p in run_dir.glob("object_*_cloud*.ply"):
        m = _OBJECT_FILE_RE.match(p.name)
        if m:
            try:
                ids.add(int(m.group(1)))
            except Exception:
                pass

    return ids


def find_object_cloud_for_run_and_id(run_dir: Path, object_id: int) -> Path | None:
    for pattern in OBJECT_FILE_PATTERNS_BY_ID:
        candidate = run_dir / pattern.format(id=object_id)
        if candidate.exists():
            return candidate

    return None


def determine_object_ids(run_records: list[dict]) -> list[int]:
    if OBJECT_IDS_TO_PROCESS is not None:
        return sorted(int(x) for x in OBJECT_IDS_TO_PROCESS)

    per_run_ids: list[set[int]] = []

    for record in run_records:
        run_dir = record["run_dir"]
        metadata = record["metadata"]

        ids = set()
        ids |= object_ids_from_metadata(metadata)
        ids |= object_ids_from_files(run_dir)

        per_run_ids.append(ids)

        print(f"[OBJECT IDS] {run_dir.name}: {sorted(ids)}")

    if not per_run_ids:
        return []

    if REQUIRE_OBJECT_IN_ALL_RUNS:
        common = set.intersection(*per_run_ids)
        return sorted(common)

    union = set.union(*per_run_ids)
    return sorted(union)


# =============================================================================
# CORNER LOADING
# =============================================================================

def get_corner_dict(metadata: dict) -> dict[str, np.ndarray]:
    if CLOUD_COORDINATE_FRAME == "camera":
        possible_keys = ["corners_camera_m", "corners_m"]
    elif CLOUD_COORDINATE_FRAME == "workspace":
        possible_keys = ["corners_workspace_m"]
    else:
        raise ValueError(f"Unknown CLOUD_COORDINATE_FRAME: {CLOUD_COORDINATE_FRAME}")

    corner_source = None
    used_key = None

    for key in possible_keys:
        if key in metadata:
            corner_source = metadata[key]
            used_key = key
            break

    if corner_source is None:
        raise KeyError(
            f"Could not find corners for CLOUD_COORDINATE_FRAME={CLOUD_COORDINATE_FRAME}. "
            f"Tried keys: {possible_keys}. Available keys: {list(metadata.keys())}"
        )

    missing = [name for name in CORNER_ORDER if name not in corner_source]
    if missing:
        raise KeyError(f"Missing required corners {missing} in metadata key {used_key}")

    return {
        name: np.asarray(corner_source[name], dtype=np.float64)
        for name in CORNER_ORDER
    }


def corners_to_array(corners: dict[str, np.ndarray]) -> np.ndarray:
    return np.vstack([corners[name] for name in CORNER_ORDER]).astype(np.float64)


# =============================================================================
# SCENE WRITER
# =============================================================================

def pcd_to_xyz_rgb(pcd: o3d.geometry.PointCloud) -> tuple[np.ndarray, np.ndarray]:
    xyz = np.asarray(pcd.points, dtype=np.float64)

    if pcd.has_colors():
        rgb_float = np.asarray(pcd.colors)
        rgb = np.clip(rgb_float * 255.0, 0, 255).astype(np.uint8)
    else:
        rgb = np.tile(np.array([[230, 180, 30]], dtype=np.uint8), (len(xyz), 1))

    return xyz, rgb


def write_scene_with_floor_ply(
    path: Path,
    floor_corners_xyz: np.ndarray,
    object_clouds: list[o3d.geometry.PointCloud],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    floor = np.asarray(floor_corners_xyz, dtype=np.float64)
    if floor.shape != (4, 3):
        raise ValueError(f"floor_corners_xyz must be 4x3, got {floor.shape}")

    floor_rgb = np.tile(np.array([[90, 110, 130]], dtype=np.uint8), (4, 1))

    all_xyz = [floor]
    all_rgb = [floor_rgb]

    for pcd in object_clouds:
        xyz, rgb = pcd_to_xyz_rgb(pcd)
        all_xyz.append(xyz)
        all_rgb.append(rgb)

    xyz_all = np.vstack(all_xyz)
    rgb_all = np.vstack(all_rgb)

    faces = [(0, 1, 2), (0, 2, 3)]

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

    log(f"[SAVE] {path}")


# =============================================================================
# FRAME / WARP MATH
# =============================================================================

def build_marker_frame(corners: dict[str, np.ndarray]) -> dict:
    TL = corners["TL"]
    TR = corners["TR"]
    BL = corners["BL"]
    BR = corners["BR"]

    x_axis = TR - TL
    x_len = float(np.linalg.norm(x_axis))
    if x_len < 1e-12:
        raise ValueError("TL and TR are too close.")

    x_hat = x_axis / x_len

    y_raw = BL - TL
    y_axis = y_raw - np.dot(y_raw, x_hat) * x_hat
    y_len = float(np.linalg.norm(y_axis))
    if y_len < 1e-12:
        raise ValueError("TL, TR, BL are nearly collinear.")

    y_hat = y_axis / y_len

    z_hat = np.cross(x_hat, y_hat)
    z_len = float(np.linalg.norm(z_hat))
    if z_len < 1e-12:
        raise ValueError("Bad marker plane normal.")

    z_hat = z_hat / z_len

    y_hat = np.cross(z_hat, x_hat)
    y_hat = y_hat / np.linalg.norm(y_hat)

    R = np.column_stack([x_hat, y_hat, z_hat])

    width_top = float(np.linalg.norm(TR - TL))
    width_bottom = float(np.linalg.norm(BR - BL))
    depth_left = float(np.linalg.norm(BL - TL))
    depth_right = float(np.linalg.norm(BR - TR))

    width_avg = 0.5 * (width_top + width_bottom)
    depth_avg = 0.5 * (depth_left + depth_right)

    return {
        "origin": TL,
        "R": R,
        "x_hat": x_hat,
        "y_hat": y_hat,
        "z_hat": z_hat,
        "width_avg": width_avg,
        "depth_avg": depth_avg,
    }


def world_to_local(points_xyz: np.ndarray, frame: dict) -> np.ndarray:
    points_xyz = np.asarray(points_xyz, dtype=np.float64)
    origin = frame["origin"]
    R = frame["R"]
    return (points_xyz - origin[None, :]) @ R


def local_to_world(local_xyz: np.ndarray, frame: dict) -> np.ndarray:
    local_xyz = np.asarray(local_xyz, dtype=np.float64)
    origin = frame["origin"]
    R = frame["R"]
    return local_xyz @ R.T + origin[None, :]


def get_corner_local_xy(corners: dict[str, np.ndarray], frame: dict) -> np.ndarray:
    arr = corners_to_array(corners)
    local = world_to_local(arr, frame)
    return local[:, :2].astype(np.float64)


def compute_homography_2d(src_xy: np.ndarray, dst_xy: np.ndarray) -> np.ndarray:
    H, status = cv2.findHomography(
        src_xy.astype(np.float64),
        dst_xy.astype(np.float64),
        method=0,
    )

    if H is None:
        raise RuntimeError("cv2.findHomography failed.")

    return H.astype(np.float64)


def apply_homography_xy(xy: np.ndarray, H: np.ndarray) -> np.ndarray:
    xy = np.asarray(xy, dtype=np.float64)

    ones = np.ones((len(xy), 1), dtype=np.float64)
    homog = np.hstack([xy, ones])

    warped = homog @ H.T
    denom = warped[:, 2:3]

    denom = np.where(np.abs(denom) < 1e-12, np.nan, denom)

    return warped[:, :2] / denom


def kabsch_similarity_transform(
    src_pts: np.ndarray,
    dst_pts: np.ndarray,
    allow_scale: bool = True,
) -> np.ndarray:
    src = np.asarray(src_pts, dtype=np.float64)
    dst = np.asarray(dst_pts, dtype=np.float64)

    src_centroid = src.mean(axis=0)
    dst_centroid = dst.mean(axis=0)

    src_centered = src - src_centroid
    dst_centered = dst - dst_centroid

    H = src_centered.T @ dst_centered
    U, S, Vt = np.linalg.svd(H)

    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    if allow_scale:
        src_var = np.sum(src_centered ** 2)
        scale = float(np.sum(S) / max(src_var, 1e-12))
    else:
        scale = 1.0

    t = dst_centroid - scale * (R @ src_centroid)

    T = np.eye(4)
    T[:3, :3] = scale * R
    T[:3, 3] = t

    return T


def apply_transform_xyz(points_xyz: np.ndarray, T: np.ndarray) -> np.ndarray:
    points_xyz = np.asarray(points_xyz, dtype=np.float64)
    homog = np.hstack([points_xyz, np.ones((len(points_xyz), 1), dtype=np.float64)])
    out = homog @ T.T
    return out[:, :3]


def compute_z_scale(src_frame: dict, ref_frame: dict) -> float:
    if Z_SCALE_MODE == "none":
        return 1.0

    if Z_SCALE_MODE == "average_xy":
        sx = ref_frame["width_avg"] / max(src_frame["width_avg"], 1e-12)
        sy = ref_frame["depth_avg"] / max(src_frame["depth_avg"], 1e-12)
        return float(0.5 * (sx + sy))

    raise ValueError(f"Unknown Z_SCALE_MODE: {Z_SCALE_MODE}")


def corner_warp_points_to_reference(
    points_xyz: np.ndarray,
    src_corners: dict[str, np.ndarray],
    ref_corners: dict[str, np.ndarray],
) -> tuple[np.ndarray, dict]:
    src_frame = build_marker_frame(src_corners)
    ref_frame = build_marker_frame(ref_corners)

    src_corner_arr = corners_to_array(src_corners)
    ref_corner_arr = corners_to_array(ref_corners)

    if WARP_MODE == "similarity_3d":
        T = kabsch_similarity_transform(
            src_corner_arr,
            ref_corner_arr,
            allow_scale=True,
        )

        warped = apply_transform_xyz(points_xyz, T)
        warped_corners = apply_transform_xyz(src_corner_arr, T)
        corner_errors = np.linalg.norm(warped_corners - ref_corner_arr, axis=1)

        return warped, {
            "mode": WARP_MODE,
            "transform_4x4": T.tolist(),
            "corner_errors_m": {
                name: float(err)
                for name, err in zip(CORNER_ORDER, corner_errors)
            },
            "corner_error_rms_m": float(np.sqrt(np.mean(corner_errors ** 2))),
        }

    if WARP_MODE == "homography_plane":
        local_src = world_to_local(points_xyz, src_frame)

        src_corner_xy = get_corner_local_xy(src_corners, src_frame)
        ref_corner_xy = get_corner_local_xy(ref_corners, ref_frame)

        H = compute_homography_2d(src_corner_xy, ref_corner_xy)

        warped_xy = apply_homography_xy(local_src[:, :2], H)

        z_scale = compute_z_scale(src_frame, ref_frame)
        warped_z = local_src[:, 2:3] * z_scale

        local_ref = np.hstack([warped_xy, warped_z])
        warped_world = local_to_world(local_ref, ref_frame)

        src_corners_local = world_to_local(src_corner_arr, src_frame)
        warped_corner_xy = apply_homography_xy(src_corners_local[:, :2], H)
        warped_corner_z = src_corners_local[:, 2:3] * z_scale
        warped_corners_local_ref = np.hstack([warped_corner_xy, warped_corner_z])
        warped_corners_world = local_to_world(warped_corners_local_ref, ref_frame)

        corner_errors = np.linalg.norm(warped_corners_world - ref_corner_arr, axis=1)

        return warped_world, {
            "mode": WARP_MODE,
            "homography_3x3": H.tolist(),
            "z_scale": float(z_scale),
            "corner_errors_m": {
                name: float(err)
                for name, err in zip(CORNER_ORDER, corner_errors)
            },
            "corner_error_rms_m": float(np.sqrt(np.mean(corner_errors ** 2))),
        }

    raise ValueError(f"Unknown WARP_MODE: {WARP_MODE}")


# =============================================================================
# CLEANING
# =============================================================================

def percentile_crop(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    if not ENABLE_PERCENTILE_CROP:
        return pcd

    pts = np.asarray(pcd.points)

    if len(pts) < 20:
        return pcd

    lo = np.percentile(pts, PERCENTILE_LOW, axis=0)
    hi = np.percentile(pts, PERCENTILE_HIGH, axis=0)

    keep = np.all((pts >= lo) & (pts <= hi), axis=1)
    idx = np.where(keep)[0]

    return pcd.select_by_index(idx)


def remove_outliers(
    pcd: o3d.geometry.PointCloud,
    nb_neighbors: int,
    std_ratio: float,
) -> o3d.geometry.PointCloud:
    if len(pcd.points) < nb_neighbors:
        return pcd

    clean, _ = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio,
    )

    return clean


def clean_object_cloud(
    pcd: o3d.geometry.PointCloud,
    label: str,
) -> o3d.geometry.PointCloud:
    log(f"\n[CLEAN] {label}")
    cloud_stats("before_clean", pcd)

    out = copy_cloud(pcd)

    out = percentile_crop(out)
    cloud_stats("after_percentile_crop", out)

    if ENABLE_STATISTICAL_OUTLIER_REMOVAL:
        out = remove_outliers(
            out,
            OUTLIER_NB_NEIGHBORS,
            OUTLIER_STD_RATIO,
        )
        cloud_stats("after_outlier_removal", out)

    if out.is_empty() or len(out.points) == 0:
        raise RuntimeError(f"{label} became empty during cleaning.")

    return out


# =============================================================================
# LIMITED ICP
# =============================================================================

def estimate_normals_safe(
    pcd: o3d.geometry.PointCloud,
    radius: float,
) -> o3d.geometry.PointCloud:
    if pcd.is_empty() or len(pcd.points) == 0:
        return pcd

    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius,
            max_nn=30,
        )
    )

    return pcd


def yaw_from_rotation_matrix(R: np.ndarray) -> float:
    return float(np.arctan2(R[1, 0], R[0, 0]))


def rotation_angle_from_matrix(R: np.ndarray) -> float:
    tr = float(np.trace(R))
    val = (tr - 1.0) / 2.0
    val = float(np.clip(val, -1.0, 1.0))
    return float(np.arccos(val))


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


def clamp_z_rotation_in_transform(
    T: np.ndarray,
    max_z_rotation_deg: float,
) -> np.ndarray:
    T = np.asarray(T, dtype=np.float64).copy()

    R = T[:3, :3]
    t = T[:3, 3].copy()

    yaw = yaw_from_rotation_matrix(R)
    max_yaw = float(np.deg2rad(max_z_rotation_deg))
    yaw_clamped = float(np.clip(yaw, -max_yaw, max_yaw))

    if abs(yaw - yaw_clamped) > np.deg2rad(0.25):
        print(
            f"[Z LIMIT] ICP wanted yaw={np.rad2deg(yaw):+.2f} deg. "
            f"Clamped to {np.rad2deg(yaw_clamped):+.2f} deg."
        )

    Rz_original = rotation_matrix_z(yaw)
    Rz_clamped = rotation_matrix_z(yaw_clamped)

    R_tilt = Rz_original.T @ R
    R_new = Rz_clamped @ R_tilt

    T_new = np.eye(4)
    T_new[:3, :3] = R_new
    T_new[:3, 3] = t

    return T_new


def translation_only_transform(T: np.ndarray) -> np.ndarray:
    T = np.asarray(T, dtype=np.float64)
    out = np.eye(4)
    out[:3, 3] = T[:3, 3]
    return out


def should_reject_icp_transform(
    result: o3d.pipelines.registration.RegistrationResult,
    T_limited: np.ndarray,
) -> tuple[bool, str]:
    if not REJECT_BAD_ICP:
        return False, "disabled"

    if result.fitness < MIN_ACCEPTABLE_ICP_FITNESS:
        return True, f"fitness too low: {result.fitness:.6f}"

    if result.inlier_rmse > MAX_ACCEPTABLE_ICP_RMSE:
        return True, f"rmse too high: {result.inlier_rmse:.6f}"

    translation_mag = float(np.linalg.norm(T_limited[:3, 3]))
    if translation_mag > MAX_ICP_TRANSLATION_M:
        return True, f"translation too large: {translation_mag:.6f} m"

    R = T_limited[:3, :3]
    total_rot_deg = float(np.rad2deg(rotation_angle_from_matrix(R)))
    if total_rot_deg > MAX_ICP_TOTAL_ROTATION_DEG:
        return True, f"rotation too large: {total_rot_deg:.3f} deg"

    return False, "accepted"


def run_small_icp_to_reference(
    source: o3d.geometry.PointCloud,
    reference: o3d.geometry.PointCloud,
) -> tuple[o3d.geometry.PointCloud, dict]:
    source_down = source.voxel_down_sample(ICP_VOXEL_SIZE)
    ref_down = reference.voxel_down_sample(ICP_VOXEL_SIZE)

    if source_down.is_empty() or len(source_down.points) == 0:
        return source, {"used": False, "reason": "empty_source_downsample"}

    if ref_down.is_empty() or len(ref_down.points) == 0:
        return source, {"used": False, "reason": "empty_reference_downsample"}

    if ICP_METHOD == "point_to_plane":
        source_down = estimate_normals_safe(
            source_down,
            radius=max(ICP_VOXEL_SIZE * 3.0, 0.005),
        )
        ref_down = estimate_normals_safe(
            ref_down,
            radius=max(ICP_VOXEL_SIZE * 3.0, 0.005),
        )
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()

    elif ICP_METHOD == "point_to_point":
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint(
            with_scaling=ALLOW_SCALE_IN_ICP
        )

    else:
        raise ValueError(f"Unknown ICP_METHOD: {ICP_METHOD}")

    result = o3d.pipelines.registration.registration_icp(
        source_down,
        ref_down,
        ICP_DISTANCE_THRESHOLD,
        np.eye(4),
        estimation,
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=80,
            relative_fitness=1e-7,
            relative_rmse=1e-7,
        ),
    )

    T_raw = result.transformation
    T_limited = T_raw.copy()

    if ICP_ONLY_TRANSLATION:
        print("[ICP LIMIT] Using translation only. Discarding ICP rotation.")
        T_limited = translation_only_transform(T_raw)

    elif LIMIT_ICP_Z_ROTATION:
        T_limited = clamp_z_rotation_in_transform(
            T_raw,
            MAX_ICP_Z_ROTATION_DEG,
        )

    reject, reason = should_reject_icp_transform(result, T_limited)

    if reject:
        print(f"[ICP REJECTED] {reason}. Keeping corner-warp-only cloud.")
        return source, {
            "used": True,
            "accepted": False,
            "reject_reason": reason,
            "method": ICP_METHOD,
            "fitness": float(result.fitness),
            "inlier_rmse": float(result.inlier_rmse),
            "raw_transform": T_raw.tolist(),
            "limited_transform": T_limited.tolist(),
        }

    aligned = copy_cloud(source)
    aligned.transform(T_limited)

    raw_yaw_deg = float(np.rad2deg(yaw_from_rotation_matrix(T_raw[:3, :3])))
    limited_yaw_deg = float(np.rad2deg(yaw_from_rotation_matrix(T_limited[:3, :3])))
    translation_mag = float(np.linalg.norm(T_limited[:3, 3]))
    total_rot_deg = float(np.rad2deg(rotation_angle_from_matrix(T_limited[:3, :3])))

    print(
        f"[SMALL ICP] accepted | "
        f"fitness={result.fitness:.6f}, "
        f"rmse={result.inlier_rmse:.6f}, "
        f"raw_yaw={raw_yaw_deg:+.2f} deg, "
        f"limited_yaw={limited_yaw_deg:+.2f} deg, "
        f"translation={translation_mag:.4f} m, "
        f"total_rot={total_rot_deg:.2f} deg"
    )

    return aligned, {
        "used": True,
        "accepted": True,
        "method": ICP_METHOD,
        "fitness": float(result.fitness),
        "inlier_rmse": float(result.inlier_rmse),
        "raw_yaw_deg": raw_yaw_deg,
        "limited_yaw_deg": limited_yaw_deg,
        "translation_magnitude_m": translation_mag,
        "total_rotation_deg": total_rot_deg,
        "raw_transform": T_raw.tolist(),
        "limited_transform": T_limited.tolist(),
    }


# =============================================================================
# OBJECT PROCESSING
# =============================================================================

def process_single_object_id(
    object_id: int,
    run_records: list[dict],
    ref_record: dict,
) -> tuple[list[o3d.geometry.PointCloud], dict]:
    print("\n" + "=" * 72)
    print(f"[OBJECT {object_id:02d}] Processing")
    print("=" * 72)

    object_output_dir = OUTPUT_DIR / f"object_{object_id:02d}"
    object_output_dir.mkdir(parents=True, exist_ok=True)

    ref_cloud_path = find_object_cloud_for_run_and_id(ref_record["run_dir"], object_id)

    if ref_cloud_path is None:
        raise FileNotFoundError(
            f"Reference run does not have object {object_id:02d}: {ref_record['run_dir']}"
        )

    ref_cloud = read_cloud(ref_cloud_path)
    ref_cloud = clean_object_cloud(ref_cloud, f"reference object_{object_id:02d}")
    ref_cloud = add_global_shift(ref_cloud)

    if SAVE_PER_SCAN_OUTPUTS:
        save_cloud(object_output_dir / f"object_{object_id:02d}_reference_clean.ply", ref_cloud)

    ref_corners = ref_record["corners"]

    warped_clouds_for_this_object: list[o3d.geometry.PointCloud] = [copy_cloud(ref_cloud)]

    debug = {
        "object_id": object_id,
        "reference_run": str(ref_record["run_dir"]),
        "reference_cloud": str(ref_cloud_path),
        "scans": [],
    }

    debug["scans"].append(
        {
            "is_reference": True,
            "run_dir": str(ref_record["run_dir"]),
            "cloud_path": str(ref_cloud_path),
            "corner_warp": None,
            "small_icp": None,
        }
    )

    for record in run_records:
        if record is ref_record:
            continue

        run_dir = record["run_dir"]
        src_cloud_path = find_object_cloud_for_run_and_id(run_dir, object_id)

        if src_cloud_path is None:
            msg = f"[WARN] Missing object_{object_id:02d} in {run_dir.name}. Skipping."
            print(msg)
            debug["scans"].append(
                {
                    "is_reference": False,
                    "run_dir": str(run_dir),
                    "cloud_path": None,
                    "skipped": True,
                    "reason": "missing_cloud",
                }
            )
            continue

        print("\n" + "-" * 72)
        print(f"[OBJECT {object_id:02d}] Warp {run_dir.name} -> reference")

        src_cloud = read_cloud(src_cloud_path)
        src_cloud = clean_object_cloud(src_cloud, f"{run_dir.name} object_{object_id:02d}")

        src_corners = record["corners"]

        src_xyz = np.asarray(src_cloud.points, dtype=np.float64)
        warped_xyz, warp_info = corner_warp_points_to_reference(
            src_xyz,
            src_corners,
            ref_corners,
        )

        warped_cloud = copy_cloud(src_cloud)
        warped_cloud.points = o3d.utility.Vector3dVector(warped_xyz)

        print(f"[CORNER WARP] object_{object_id:02d} corner RMS error: {warp_info['corner_error_rms_m']:.9f} m")

        if SAVE_PER_SCAN_OUTPUTS:
            save_cloud(
                object_output_dir / f"{run_dir.name}_object_{object_id:02d}_after_corner_warp_no_icp.ply",
                warped_cloud,
            )

        small_icp_info = {"used": False}

        if RUN_SMALL_ICP_AFTER_CORNER_WARP:
            warped_cloud, small_icp_info = run_small_icp_to_reference(
                warped_cloud,
                ref_cloud,
            )

            if SAVE_PER_SCAN_OUTPUTS:
                save_cloud(
                    object_output_dir / f"{run_dir.name}_object_{object_id:02d}_after_limited_icp.ply",
                    warped_cloud,
                )

        warped_cloud = add_global_shift(warped_cloud)

        warped_clouds_for_this_object.append(warped_cloud)

        debug["scans"].append(
            {
                "is_reference": False,
                "run_dir": str(run_dir),
                "cloud_path": str(src_cloud_path),
                "corner_warp": warp_info,
                "small_icp": small_icp_info,
            }
        )

    merged = o3d.geometry.PointCloud()
    for cloud in warped_clouds_for_this_object:
        merged += cloud

    save_cloud(object_output_dir / f"object_{object_id:02d}_merged_raw.ply", merged)

    merged_clean = remove_outliers(
        merged,
        FINAL_OUTLIER_NB_NEIGHBORS,
        FINAL_OUTLIER_STD_RATIO,
    )
    save_cloud(object_output_dir / f"object_{object_id:02d}_merged_clean.ply", merged_clean)

    if VOXEL_SIZE_FINAL_PER_OBJECT > 0:
        merged_down = merged_clean.voxel_down_sample(VOXEL_SIZE_FINAL_PER_OBJECT)
    else:
        merged_down = merged_clean

    save_cloud(object_output_dir / f"object_{object_id:02d}_merged_downsampled.ply", merged_down)

    debug["merged_outputs"] = {
        "raw": str(object_output_dir / f"object_{object_id:02d}_merged_raw.ply"),
        "clean": str(object_output_dir / f"object_{object_id:02d}_merged_clean.ply"),
        "downsampled": str(object_output_dir / f"object_{object_id:02d}_merged_downsampled.ply"),
    }

    if SAVE_DEBUG_JSON:
        debug_path = object_output_dir / f"object_{object_id:02d}_debug.json"
        debug_path.write_text(json.dumps(debug, indent=2) + "\n", encoding="utf-8")
        print(f"[SAVE] {debug_path}")

    # Return downsampled single merged cloud for all-object scene.
    return [merged_down], debug


# =============================================================================
# MAIN
# =============================================================================

def main() -> None:
    print("=" * 72)
    print("Multi-Object Corner-Warp + Limited ICP Cloud Merger")
    print("=" * 72)

    print(f"[CONFIG] ROOT_RUNS_DIR = {ROOT_RUNS_DIR}")
    print(f"[CONFIG] OUTPUT_DIR = {OUTPUT_DIR}")
    print(f"[CONFIG] CLOUD_COORDINATE_FRAME = {CLOUD_COORDINATE_FRAME}")
    print(f"[CONFIG] WARP_MODE = {WARP_MODE}")
    print(f"[CONFIG] RUN_SMALL_ICP_AFTER_CORNER_WARP = {RUN_SMALL_ICP_AFTER_CORNER_WARP}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    run_dirs = find_run_dirs()

    print("\n[INFO] Found run dirs:")
    for i, d in enumerate(run_dirs):
        print(f"  [{i}] {d}")

    if REFERENCE_INDEX < 0 or REFERENCE_INDEX >= len(run_dirs):
        raise IndexError(f"REFERENCE_INDEX={REFERENCE_INDEX} invalid for {len(run_dirs)} runs.")

    run_records: list[dict] = []

    for i, run_dir in enumerate(run_dirs):
        metadata = load_metadata(run_dir)
        corners = get_corner_dict(metadata)

        record = {
            "index": i,
            "run_dir": run_dir,
            "metadata": metadata,
            "corners": corners,
        }
        run_records.append(record)

        print(f"\n[RUN {i}] {run_dir.name}")
        print(f"  metadata objects: {sorted(object_ids_from_metadata(metadata))}")
        print(f"  file objects:     {sorted(object_ids_from_files(run_dir))}")
        for name in CORNER_ORDER:
            p = corners[name]
            print(f"  {name}: [{p[0]:+.6f}, {p[1]:+.6f}, {p[2]:+.6f}]")

    ref_record = run_records[REFERENCE_INDEX]

    object_ids = determine_object_ids(run_records)

    if not object_ids:
        raise RuntimeError(
            "No object IDs found. Check metadata['objects'] or object_XX_cloud*.ply filenames."
        )

    print("\n[INFO] Object IDs to process:")
    for oid in object_ids:
        print(f"  object_{oid:02d}")

    all_merged_object_clouds: list[o3d.geometry.PointCloud] = []

    full_debug = {
        "config": {
            "ROOT_RUNS_DIR": str(ROOT_RUNS_DIR),
            "OUTPUT_DIR": str(OUTPUT_DIR),
            "REFERENCE_INDEX": REFERENCE_INDEX,
            "CLOUD_COORDINATE_FRAME": CLOUD_COORDINATE_FRAME,
            "WARP_MODE": WARP_MODE,
            "Z_SCALE_MODE": Z_SCALE_MODE,
            "GLOBAL_FINAL_SHIFT_M": GLOBAL_FINAL_SHIFT_M.tolist(),
            "RUN_SMALL_ICP_AFTER_CORNER_WARP": RUN_SMALL_ICP_AFTER_CORNER_WARP,
            "ICP_ONLY_TRANSLATION": ICP_ONLY_TRANSLATION,
            "LIMIT_ICP_Z_ROTATION": LIMIT_ICP_Z_ROTATION,
            "MAX_ICP_Z_ROTATION_DEG": MAX_ICP_Z_ROTATION_DEG,
            "ICP_DISTANCE_THRESHOLD": ICP_DISTANCE_THRESHOLD,
        },
        "runs": [str(r["run_dir"]) for r in run_records],
        "reference_run": str(ref_record["run_dir"]),
        "objects": [],
    }

    for object_id in object_ids:
        merged_list, object_debug = process_single_object_id(
            object_id=object_id,
            run_records=run_records,
            ref_record=ref_record,
        )

        all_merged_object_clouds.extend(merged_list)
        full_debug["objects"].append(object_debug)

    print("\n" + "=" * 72)
    print("[ALL OBJECTS] Building combined scene")
    print("=" * 72)

    all_objects_merged = o3d.geometry.PointCloud()
    for pcd in all_merged_object_clouds:
        all_objects_merged += pcd

    save_cloud(OUTPUT_DIR / "all_objects_merged_raw.ply", all_objects_merged)

    all_objects_clean = remove_outliers(
        all_objects_merged,
        FINAL_OUTLIER_NB_NEIGHBORS,
        FINAL_OUTLIER_STD_RATIO,
    )
    save_cloud(OUTPUT_DIR / "all_objects_merged_clean.ply", all_objects_clean)

    if VOXEL_SIZE_FINAL_ALL_OBJECTS > 0:
        all_objects_down = all_objects_clean.voxel_down_sample(VOXEL_SIZE_FINAL_ALL_OBJECTS)
    else:
        all_objects_down = all_objects_clean

    save_cloud(OUTPUT_DIR / "all_objects_merged_downsampled.ply", all_objects_down)

    ref_floor = corners_to_array(ref_record["corners"])

    if SAVE_REFERENCE_FLOOR:
        floor_cloud = o3d.geometry.PointCloud()
        floor_cloud.points = o3d.utility.Vector3dVector(ref_floor)
        floor_cloud.colors = o3d.utility.Vector3dVector(
            np.array(
                [
                    [0.1, 1.0, 0.1],
                    [0.1, 0.3, 1.0],
                    [1.0, 0.1, 0.1],
                    [1.0, 1.0, 0.1],
                ],
                dtype=np.float64,
            )
        )
        save_cloud(OUTPUT_DIR / "reference_marker_corners_points.ply", floor_cloud)

    if SAVE_MERGED_SCENE_WITH_FLOOR:
        write_scene_with_floor_ply(
            OUTPUT_DIR / "all_objects_scene_with_reference_floor.ply",
            ref_floor,
            all_merged_object_clouds,
        )

    if SAVE_DEBUG_JSON:
        debug_path = OUTPUT_DIR / "multi_object_debug.json"
        debug_path.write_text(json.dumps(full_debug, indent=2) + "\n", encoding="utf-8")
        print(f"[SAVE] {debug_path}")

    print("\n" + "=" * 72)
    print("[DONE]")
    print("Open these in CloudCompare:")
    print(f"  {OUTPUT_DIR / 'all_objects_scene_with_reference_floor.ply'}")
    print(f"  {OUTPUT_DIR / 'all_objects_merged_downsampled.ply'}")
    print("\nPer-object outputs are here:")
    for object_id in object_ids:
        print(f"  {OUTPUT_DIR / f'object_{object_id:02d}'}")
    print("=" * 72)


if __name__ == "__main__":
    main()