"""
Offline object-cloud rebuild and height validation for saved stereo captures.

Set RUN_DIR in the USER SETTINGS section, then run:

    python .\recompute_object_clouds_and_validate_height.py

This script does not use live cameras and does not open SAM. It rebuilds
per-object point clouds from saved rectified stereo images, saved masks,
calibration, and saved or recomputed disparity maps.
"""

from __future__ import annotations

import csv
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

o3d: Any = None


# =============================================================================
# USER SETTINGS
# =============================================================================

RUN_DIR = Path(r"C:\Users\elipp\OneDrive\Documents\Grocery Bagger\calibration\one_pass_two_view_reconstruction\pipeline_20260504_152132")  # folder containing saved rectified images/masks/metadata
OUTPUT_DIR = None  # if None, save to RUN_DIR / "recomputed_object_validation"

OBJECT_IDS_TO_PROCESS = None  # None = auto-detect masks; or [0, 1, 2, 3]

TRUE_DIMS_BY_OBJECT_ID = {
    0: {"x": 0.050, "y": 0.050, "z": 0.100},  # example known 3D printed object
}

USE_RAFT_IF_DISPARITY_MISSING = True
USE_SGBM_IF_RAFT_FAILS = True

HEIGHT_LOW_PERCENTILE = 2.0
HEIGHT_BOTTOM_PERCENTILE = 5.0
HEIGHT_TOP_PERCENTILE = 95.0
HEIGHT_HIGH_PERCENTILE = 98.0
HEIGHT_MIN_POINTS = 50

MIN_DISPARITY = 1.0
SCALE_TO_METERS = 0.001

ERODE_MASK_PIXELS = 2
ENABLE_MASK_LARGEST_COMPONENT_ONLY = True
ENABLE_OUTLIER_REMOVAL = True
OUTLIER_NB_NEIGHBORS = 25
OUTLIER_STD_RATIO = 2.0
ENABLE_RADIUS_OUTLIER_REMOVAL = True
RADIUS_NB_POINTS = 8
RADIUS_RADIUS_M = 0.01

SAVE_OBJECT_CLOUDS = True
SAVE_CLEANED_OBJECT_CLOUDS = True
SAVE_HEIGHT_METRICS_CSV = True
SAVE_HEIGHT_METRICS_JSON = True
SAVE_DEBUG_OVERLAYS = True


# =============================================================================
# PROJECT IMPORT SETUP
# =============================================================================

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = WORKSPACE_ROOT.parent

if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))


# =============================================================================
# CONSTANTS / DATA
# =============================================================================

RAFT_REPO_DIR = PROJECT_ROOT / "external" / "RAFT-Stereo"
RAFT_CHECKPOINT = WORKSPACE_ROOT / "models" / "raft_stereo" / "raftstereo-middlebury.pth"
RAFT_ITERS = 32
RAFT_CORR_IMPLEMENTATION = "alt"
RAFT_MIXED_PRECISION = True

CORNER_ORDER = ["TL", "TR", "BR", "BL"]
SKIP_MASK_NAME_PARTS = ("overlay", "debug", "color", "rgb", "left", "right")
OBJECT_ID_RE = re.compile(
    r"(?:^|[_\-])(?:object|obj|mask_object)[_\-]?(\d+)|object[_\-]?(\d+)",
    re.IGNORECASE,
)


@dataclass
class MaskRecord:
    object_id: int
    source: Path
    mask: np.ndarray | None = None


@dataclass
class ViewRecord:
    view_id: int
    view_dir: Path
    left_path: Path
    right_path: Path
    masks: dict[int, MaskRecord]
    disparity_path: Path | None = None
    corners_camera_m: dict[str, np.ndarray] | None = None
    corners_workspace_m: dict[str, np.ndarray] | None = None


def log(message: str) -> None:
    print(message)


def warn(message: str) -> None:
    print(f"[WARN] {message}")


def ensure_open3d() -> Any:
    global o3d
    if o3d is not None:
        return o3d
    try:
        import open3d as open3d_module
    except Exception as exc:
        raise ModuleNotFoundError(
            "Open3D is required to build and save object clouds. "
            "Install it in this Python environment, then rerun this script."
        ) from exc
    o3d = open3d_module
    return o3d


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def array_corner_dict(data: Any) -> dict[str, np.ndarray] | None:
    if not isinstance(data, dict):
        return None
    out: dict[str, np.ndarray] = {}
    for name in CORNER_ORDER:
        if name not in data:
            return None
        arr = np.asarray(data[name], dtype=np.float64).reshape(-1)
        if arr.size != 3 or not np.all(np.isfinite(arr)):
            return None
        out[name] = arr
    return out


def clean_mask(mask: np.ndarray) -> np.ndarray:
    if mask.ndim == 3:
        mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    binary = (mask > 0).astype(np.uint8)

    if ENABLE_MASK_LARGEST_COMPONENT_ONLY:
        count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
        if count > 1:
            largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
            binary = (labels == largest).astype(np.uint8)

    if ERODE_MASK_PIXELS > 0:
        k = 2 * int(ERODE_MASK_PIXELS) + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        binary = cv2.erode(binary, kernel, iterations=1)

    return binary


def object_id_from_path(path: Path) -> int | None:
    m = OBJECT_ID_RE.search(path.stem)
    if not m:
        return None
    value = m.group(1) or m.group(2)
    try:
        return int(value)
    except Exception:
        return None


def is_mask_candidate(path: Path) -> bool:
    name = path.name.lower()
    if path.suffix.lower() not in {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".npz"}:
        return False
    if any(part in name for part in SKIP_MASK_NAME_PARTS):
        return False
    return object_id_from_path(path) is not None


def unique_existing(paths: list[Path]) -> list[Path]:
    seen: set[Path] = set()
    out: list[Path] = []
    for path in paths:
        try:
            resolved = path.resolve()
        except Exception:
            resolved = path
        if path.exists() and resolved not in seen:
            seen.add(resolved)
            out.append(path)
    return out


def find_first_image(base: Path, patterns: list[str]) -> Path | None:
    candidates: list[Path] = []
    for pattern in patterns:
        candidates.extend(sorted(base.glob(pattern)))
    found = unique_existing([p for p in candidates if p.is_file()])
    return found[0] if found else None


def find_left_right_for_base(base: Path) -> tuple[Path, Path] | None:
    left = find_first_image(base, ["left_rect.png", "*_left_rect*.png", "*left_rect*.png"])
    right = find_first_image(base, ["right_rect.png", "*_right_rect*.png", "*right_rect*.png"])
    if left is not None and right is not None:
        return left, right
    return None


def find_disparity_for_base(base: Path) -> Path | None:
    patterns = ["disparity.npy", "disparity_raw.npy", "raft_disparity.npy", "*disparity*.npy"]
    path = find_first_image(base, patterns)
    return path


def load_mask_record(record: MaskRecord) -> np.ndarray | None:
    if record.mask is not None:
        return record.mask
    if record.source.suffix.lower() == ".npz":
        try:
            with np.load(record.source, allow_pickle=False) as data:
                if "mask" in data:
                    return np.asarray(data["mask"])
        except Exception as exc:
            warn(f"Could not read mask from {record.source}: {exc}")
            return None
    img = cv2.imread(str(record.source), cv2.IMREAD_UNCHANGED)
    if img is None:
        warn(f"Could not read mask image: {record.source}")
    return img


def find_masks_for_view(run_dir: Path, view: Path, view_id: int) -> dict[int, MaskRecord]:
    search_roots = [view, view / "masks"]
    if view == run_dir:
        search_roots.append(run_dir / "masks")
    else:
        search_roots.extend([run_dir / "masks", run_dir / "tuner_cache"])

    candidates: list[Path] = []
    for root in search_roots:
        if not root.exists():
            continue
        for pattern in ("object_*mask*.png", "object_*.png", "mask_object_*.png", "object_*.npz"):
            candidates.extend(sorted(root.glob(pattern)))

    # Tuner cache filenames are object_00_view_00.npz.
    cache = run_dir / "tuner_cache"
    if cache.exists():
        candidates.extend(sorted(cache.glob(f"object_*_view_{view_id:02d}.npz")))

    masks: dict[int, MaskRecord] = {}
    for path in unique_existing([p for p in candidates if p.is_file()]):
        if not is_mask_candidate(path):
            continue
        oid = object_id_from_path(path)
        if oid is None:
            continue
        if path.suffix.lower() == ".npz" and f"view_{view_id:02d}" not in path.stem:
            continue
        masks.setdefault(oid, MaskRecord(object_id=oid, source=path))
    return masks


def discover_views(run_dir: Path) -> list[ViewRecord]:
    views: list[ViewRecord] = []
    seen_pairs: set[tuple[Path, Path]] = set()

    bases = [run_dir / "view_00", run_dir / "view_01", run_dir]
    bases.extend(sorted(p for p in run_dir.glob("view_*") if p.is_dir()))

    for base in unique_existing([p for p in bases if p.is_dir()]):
        pair = find_left_right_for_base(base)
        if pair is None:
            continue
        left, right = pair
        key = (left.resolve(), right.resolve())
        if key in seen_pairs:
            continue
        seen_pairs.add(key)

        m = re.search(r"view[_\-]?(\d+)", base.name, re.IGNORECASE)
        view_id = int(m.group(1)) if m else len(views)
        views.append(
            ViewRecord(
                view_id=view_id,
                view_dir=base,
                left_path=left,
                right_path=right,
                masks=find_masks_for_view(run_dir, base, view_id),
                disparity_path=find_disparity_for_base(base),
            )
        )

    return sorted(views, key=lambda v: v.view_id)


def load_calibration() -> dict[str, np.ndarray]:
    try:
        import config

        calib = config.load_stereo_calibration()
        if calib is not None:
            log(f"[CALIB] Loaded via config: {config.ACTIVE_CALIBRATION_NPZ}")
            return calib
        warn(f"config.load_stereo_calibration returned None: {config.ACTIVE_CALIBRATION_NPZ}")
    except Exception as exc:
        warn(f"Could not import/use config calibration loader: {exc}")

    fallback = PROJECT_ROOT / "calibration" / "matrices" / "latest" / "stereo_calibration.npz"
    if not fallback.exists():
        raise FileNotFoundError(f"Calibration not found: {fallback}")
    with np.load(fallback, allow_pickle=False) as data:
        log(f"[CALIB] Loaded direct: {fallback}")
        return {name: data[name] for name in data.files}


def load_metadata_for_views(run_dir: Path, views: list[ViewRecord]) -> dict[str, Any]:
    summary: dict[str, Any] = {"metadata_paths": [], "workspace_frame_found": False}

    metadata_path = run_dir / "pipeline_metadata.json"
    if metadata_path.exists():
        try:
            metadata = read_json(metadata_path)
            summary["metadata_paths"].append(str(metadata_path))
            by_id = {int(v.get("view_id", i)): v for i, v in enumerate(metadata.get("views", []))}
            for view in views:
                data = by_id.get(view.view_id)
                if not isinstance(data, dict):
                    continue
                view.corners_camera_m = array_corner_dict(data.get("corners_camera_m"))
                view.corners_workspace_m = array_corner_dict(data.get("corners_workspace_m"))
        except Exception as exc:
            warn(f"Could not load {metadata_path}: {exc}")

    for view in views:
        cache_cam = run_dir / "tuner_cache" / f"view_{view.view_id:02d}_corners_camera_m.json"
        cache_ws = run_dir / "tuner_cache" / f"view_{view.view_id:02d}_corners_workspace_m.json"
        if view.corners_camera_m is None and cache_cam.exists():
            try:
                view.corners_camera_m = array_corner_dict(read_json(cache_cam))
                summary["metadata_paths"].append(str(cache_cam))
            except Exception as exc:
                warn(f"Could not load {cache_cam}: {exc}")
        if view.corners_workspace_m is None and cache_ws.exists():
            try:
                view.corners_workspace_m = array_corner_dict(read_json(cache_ws))
                summary["metadata_paths"].append(str(cache_ws))
            except Exception as exc:
                warn(f"Could not load {cache_ws}: {exc}")

    summary["workspace_frame_found"] = any(v.corners_camera_m is not None for v in views)
    return summary


def build_marker_frame(corners: dict[str, np.ndarray]) -> dict[str, Any]:
    tl, tr, br, bl = (np.asarray(corners[name], dtype=np.float64) for name in CORNER_ORDER)
    x_axis = tr - tl
    x_norm = float(np.linalg.norm(x_axis))
    if x_norm < 1e-12:
        raise ValueError("TL and TR marker corners are too close")
    x_hat = x_axis / x_norm

    y_raw = bl - tl
    y_axis = y_raw - np.dot(y_raw, x_hat) * x_hat
    y_norm = float(np.linalg.norm(y_axis))
    if y_norm < 1e-12:
        raise ValueError("Marker corners are nearly collinear")
    y_hat = y_axis / y_norm

    z_hat = np.cross(x_hat, y_hat)
    z_hat = z_hat / max(float(np.linalg.norm(z_hat)), 1e-12)
    y_hat = np.cross(z_hat, x_hat)
    y_hat = y_hat / max(float(np.linalg.norm(y_hat)), 1e-12)

    return {
        "origin": tl,
        "R": np.column_stack([x_hat, y_hat, z_hat]),
        "width_avg": 0.5 * (np.linalg.norm(tr - tl) + np.linalg.norm(br - bl)),
        "depth_avg": 0.5 * (np.linalg.norm(bl - tl) + np.linalg.norm(br - tr)),
    }


def camera_to_workspace(points_cam_m: np.ndarray, corners_camera_m: dict[str, np.ndarray]) -> np.ndarray:
    frame = build_marker_frame(corners_camera_m)
    return (np.asarray(points_cam_m, dtype=np.float64) - frame["origin"][None, :]) @ frame["R"]


def compute_sgbm_disparity(left_bgr: np.ndarray, right_bgr: np.ndarray) -> np.ndarray:
    h, w = left_bgr.shape[:2]
    num_disparities = max(16, int(math.ceil(w / 8 / 16)) * 16)
    block_size = 7
    matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disparities,
        blockSize=block_size,
        P1=8 * 3 * block_size**2,
        P2=32 * 3 * block_size**2,
        disp12MaxDiff=1,
        uniquenessRatio=8,
        speckleWindowSize=100,
        speckleRange=2,
        preFilterCap=31,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )
    gray_l = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)
    return matcher.compute(gray_l, gray_r).astype(np.float32) / 16.0


def get_disparity(view: ViewRecord, left_bgr: np.ndarray, right_bgr: np.ndarray) -> tuple[np.ndarray, dict[str, Any]]:
    info: dict[str, Any] = {"view_id": view.view_id, "source": None, "path": None, "method": None}

    if view.disparity_path is not None and view.disparity_path.exists():
        disparity = np.load(view.disparity_path).astype(np.float32)
        if disparity.shape[:2] == left_bgr.shape[:2]:
            info.update({"source": "loaded", "path": str(view.disparity_path), "method": "saved_npy"})
            return disparity, info
        warn(
            f"Saved disparity shape {disparity.shape} does not match image shape "
            f"{left_bgr.shape[:2]} for {view.disparity_path}; recomputing."
        )

    if USE_RAFT_IF_DISPARITY_MISSING:
        try:
            from stereo_raft import compute_raft_disparity

            disparity = compute_raft_disparity(
                left_bgr,
                right_bgr,
                raft_repo_dir=RAFT_REPO_DIR,
                checkpoint=RAFT_CHECKPOINT,
                iters=RAFT_ITERS,
                corr_implementation=RAFT_CORR_IMPLEMENTATION,
                mixed_precision=RAFT_MIXED_PRECISION,
            )
            info.update({"source": "recomputed", "path": None, "method": "raft"})
            return disparity.astype(np.float32), info
        except Exception as exc:
            warn(f"RAFT disparity failed for view_{view.view_id:02d}: {exc}")

    if USE_SGBM_IF_RAFT_FAILS:
        disparity = compute_sgbm_disparity(left_bgr, right_bgr)
        info.update({"source": "recomputed", "path": None, "method": "sgbm"})
        return disparity.astype(np.float32), info

    raise RuntimeError(f"No disparity available for view_{view.view_id:02d}")


def pcd_from_arrays(points: np.ndarray, colors_bgr: np.ndarray | None = None) -> o3d.geometry.PointCloud:
    ensure_open3d()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=np.float64))
    if colors_bgr is not None and len(colors_bgr) == len(points):
        rgb = colors_bgr[:, ::-1].astype(np.float64) / 255.0
        pcd.colors = o3d.utility.Vector3dVector(rgb)
    return pcd


def clean_cloud(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    out = pcd
    if ENABLE_OUTLIER_REMOVAL and len(out.points) >= OUTLIER_NB_NEIGHBORS:
        out, _ = out.remove_statistical_outlier(
            nb_neighbors=OUTLIER_NB_NEIGHBORS,
            std_ratio=OUTLIER_STD_RATIO,
        )
    if ENABLE_RADIUS_OUTLIER_REMOVAL and len(out.points) >= RADIUS_NB_POINTS:
        out, _ = out.remove_radius_outlier(
            nb_points=RADIUS_NB_POINTS,
            radius=RADIUS_RADIUS_M,
        )
    return out


def write_cloud(path: Path, pcd: o3d.geometry.PointCloud) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok = o3d.io.write_point_cloud(str(path), pcd)
    if ok:
        log(f"[SAVE] {path}")
    else:
        warn(f"Open3D failed to save cloud: {path}")


def save_overlay(path: Path, image_bgr: np.ndarray, mask: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    overlay = image_bgr.copy()
    mask_bool = mask > 0
    color = np.zeros_like(overlay)
    color[mask_bool] = (0, 220, 120)
    overlay = cv2.addWeighted(overlay, 0.78, color, 0.22, 0.0)
    contours, _ = cv2.findContours(mask_bool.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay, contours, -1, (0, 255, 0), 2)
    cv2.imwrite(str(path), overlay)


def percentile_or_nan(values: np.ndarray, percentile: float) -> float:
    if len(values) == 0:
        return float("nan")
    return float(np.percentile(values, percentile))


def pca_xy_dims(points: np.ndarray) -> tuple[float, float]:
    if len(points) < 3:
        return float("nan"), float("nan")
    xy = np.asarray(points[:, :2], dtype=np.float64)
    center = xy.mean(axis=0)
    cov = np.cov((xy - center).T)
    vals, vecs = np.linalg.eigh(cov)
    order = np.argsort(vals)[::-1]
    axes = vecs[:, order]
    proj = (xy - center) @ axes
    d1 = percentile_or_nan(proj[:, 0], 98.0) - percentile_or_nan(proj[:, 0], 2.0)
    d2 = percentile_or_nan(proj[:, 1], 98.0) - percentile_or_nan(proj[:, 1], 2.0)
    return float(d1), float(d2)


def add_known_dim_metrics(metrics: dict[str, Any], object_id: int) -> None:
    dims = TRUE_DIMS_BY_OBJECT_ID.get(object_id)
    if not dims:
        return

    true_x = float(dims["x"])
    true_y = float(dims["y"])
    true_z = float(dims["z"])
    metrics.update({"true_x_m": true_x, "true_y_m": true_y, "true_z_m": true_z})

    top_h = metrics.get("floor_relative_top_height_m", float("nan"))
    robust_h = metrics.get("robust_height_m", float("nan"))
    metrics["height_error_m"] = top_h - true_z if np.isfinite(top_h) else float("nan")
    metrics["height_error_mm"] = metrics["height_error_m"] * 1000.0 if np.isfinite(metrics["height_error_m"]) else float("nan")
    metrics["height_percent_error"] = (
        metrics["height_error_m"] / true_z * 100.0 if true_z > 0 and np.isfinite(metrics["height_error_m"]) else float("nan")
    )

    for axis, true_value in (("x", true_x), ("y", true_y), ("z", true_z)):
        measured = float(metrics.get(f"bbox_dim_{axis}_m", float("nan")))
        err = measured - true_value if np.isfinite(measured) else float("nan")
        metrics[f"bbox_{axis}_error_m"] = err
        metrics[f"bbox_{axis}_error_mm"] = err * 1000.0 if np.isfinite(err) else float("nan")
        metrics[f"bbox_{axis}_percent_error"] = err / true_value * 100.0 if true_value > 0 and np.isfinite(err) else float("nan")

    metrics["suggested_z_scale_from_top_height"] = true_z / max(float(top_h), 1e-9) if np.isfinite(top_h) else float("nan")
    metrics["suggested_z_scale_from_robust_height"] = true_z / max(float(robust_h), 1e-9) if np.isfinite(robust_h) else float("nan")

    pca_dims = sorted(
        [metrics.get("pca_dim_1_m", float("nan")), metrics.get("pca_dim_2_m", float("nan"))],
        reverse=True,
    )
    true_xy = sorted([true_x, true_y], reverse=True)
    for i, (measured, true_value) in enumerate(zip(pca_dims, true_xy), start=1):
        err = float(measured) - true_value if np.isfinite(measured) else float("nan")
        metrics[f"pca_dim_{i}_sorted_error_m"] = err
        metrics[f"pca_dim_{i}_sorted_error_mm"] = err * 1000.0 if np.isfinite(err) else float("nan")
        metrics[f"pca_dim_{i}_sorted_percent_error"] = err / true_value * 100.0 if true_value > 0 and np.isfinite(err) else float("nan")


def compute_metrics(
    object_id: int,
    view_id: int,
    frame_type: str,
    raw_count: int,
    clean_points: np.ndarray,
) -> dict[str, Any]:
    count = int(len(clean_points))
    valid = count >= HEIGHT_MIN_POINTS
    metrics: dict[str, Any] = {
        "object_id": int(object_id),
        "view_id": int(view_id),
        "frame_type": frame_type,
        "point_count_raw": int(raw_count),
        "point_count_clean": count,
        "valid": valid,
    }

    fields = [
        "centroid_x_m", "centroid_y_m", "centroid_z_m",
        "bbox_x_min_m", "bbox_x_max_m", "bbox_y_min_m", "bbox_y_max_m", "bbox_z_min_m", "bbox_z_max_m",
        "bbox_dim_x_m", "bbox_dim_y_m", "bbox_dim_z_raw_m",
        "z_p02_m", "z_p05_m", "z_p95_m", "z_p98_m",
        "robust_height_m", "floor_relative_top_height_m", "floor_relative_bottom_m",
        "robust_height_cm", "floor_relative_top_height_cm", "floor_relative_top_height_mm",
        "pca_dim_1_m", "pca_dim_2_m",
    ]

    if count == 0:
        metrics.update({field: float("nan") for field in fields})
        add_known_dim_metrics(metrics, object_id)
        return metrics

    pts = np.asarray(clean_points, dtype=np.float64)
    centroid = pts.mean(axis=0)
    bbox_min = pts.min(axis=0)
    bbox_max = pts.max(axis=0)
    dims = bbox_max - bbox_min
    z = pts[:, 2]
    z_p02 = percentile_or_nan(z, HEIGHT_LOW_PERCENTILE)
    z_p05 = percentile_or_nan(z, HEIGHT_BOTTOM_PERCENTILE)
    z_p95 = percentile_or_nan(z, HEIGHT_TOP_PERCENTILE)
    z_p98 = percentile_or_nan(z, HEIGHT_HIGH_PERCENTILE)
    robust_height = z_p95 - z_p02
    pca_1, pca_2 = pca_xy_dims(pts)

    floor_top = z_p95 if frame_type == "workspace" else float("nan")
    floor_bottom = z_p05 if frame_type == "workspace" else float("nan")

    metrics.update(
        {
            "centroid_x_m": float(centroid[0]),
            "centroid_y_m": float(centroid[1]),
            "centroid_z_m": float(centroid[2]),
            "bbox_x_min_m": float(bbox_min[0]),
            "bbox_x_max_m": float(bbox_max[0]),
            "bbox_y_min_m": float(bbox_min[1]),
            "bbox_y_max_m": float(bbox_max[1]),
            "bbox_z_min_m": float(bbox_min[2]),
            "bbox_z_max_m": float(bbox_max[2]),
            "bbox_dim_x_m": float(dims[0]),
            "bbox_dim_y_m": float(dims[1]),
            "bbox_dim_z_raw_m": float(dims[2]),
            "z_p02_m": z_p02,
            "z_p05_m": z_p05,
            "z_p95_m": z_p95,
            "z_p98_m": z_p98,
            "robust_height_m": float(robust_height),
            "floor_relative_top_height_m": float(floor_top),
            "floor_relative_bottom_m": float(floor_bottom),
            "robust_height_cm": float(robust_height * 100.0),
            "floor_relative_top_height_cm": float(floor_top * 100.0) if np.isfinite(floor_top) else float("nan"),
            "floor_relative_top_height_mm": float(floor_top * 1000.0) if np.isfinite(floor_top) else float("nan"),
            "pca_dim_1_m": pca_1,
            "pca_dim_2_m": pca_2,
        }
    )
    add_known_dim_metrics(metrics, object_id)
    return metrics


def nan_to_none(obj: Any) -> Any:
    if isinstance(obj, dict):
        return {k: nan_to_none(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [nan_to_none(v) for v in obj]
    if isinstance(obj, float) and (math.isnan(obj) or math.isinf(obj)):
        return None
    return obj


def save_metrics(output_dir: Path, metrics: list[dict[str, Any]]) -> None:
    if SAVE_HEIGHT_METRICS_JSON:
        path = output_dir / "recomputed_height_metrics.json"
        path.write_text(json.dumps(nan_to_none(metrics), indent=2) + "\n", encoding="utf-8")
        log(f"[SAVE] {path}")

    if SAVE_HEIGHT_METRICS_CSV:
        path = output_dir / "recomputed_height_metrics.csv"
        keys: list[str] = []
        for row in metrics:
            for key in row:
                if key not in keys:
                    keys.append(key)
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(metrics)
        log(f"[SAVE] {path}")


def print_table(metrics: list[dict[str, Any]]) -> None:
    print("\nVIEW | OBJ | pts | centroid_x_cm | centroid_y_cm | top_h_mm | robust_h_mm | bbox_x_mm | bbox_y_mm | bbox_z_mm | top_err_mm | suggested_z_scale")
    print("-" * 145)
    for m in metrics:
        def fmt(key: str, scale: float = 1.0, width: int = 10) -> str:
            value = m.get(key, float("nan"))
            if value is None or not np.isfinite(float(value)):
                return f"{'NaN':>{width}}"
            return f"{float(value) * scale:>{width}.2f}"

        print(
            f"{int(m['view_id']):>4} | {int(m['object_id']):>3} | {int(m['point_count_clean']):>3} | "
            f"{fmt('centroid_x_m', 100.0, 13)} | {fmt('centroid_y_m', 100.0, 13)} | "
            f"{fmt('floor_relative_top_height_m', 1000.0, 8)} | {fmt('robust_height_m', 1000.0, 11)} | "
            f"{fmt('bbox_dim_x_m', 1000.0, 9)} | {fmt('bbox_dim_y_m', 1000.0, 9)} | {fmt('bbox_dim_z_raw_m', 1000.0, 9)} | "
            f"{fmt('height_error_m', 1000.0, 10)} | {fmt('suggested_z_scale_from_top_height', 1.0, 17)}"
        )


def process_view(
    view: ViewRecord,
    q_matrix: np.ndarray,
    object_ids: list[int],
    output_dir: Path,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    ensure_open3d()
    log(f"\n[VIEW {view.view_id:02d}]")
    log(f"  left:      {view.left_path}")
    log(f"  right:     {view.right_path}")
    log(f"  masks:     {sorted(view.masks)}")
    log(f"  disparity: {view.disparity_path if view.disparity_path else 'not found'}")

    left = cv2.imread(str(view.left_path), cv2.IMREAD_COLOR)
    right = cv2.imread(str(view.right_path), cv2.IMREAD_COLOR)
    if left is None or right is None:
        raise RuntimeError(f"Could not read rectified stereo pair for view_{view.view_id:02d}")
    if left.shape[:2] != right.shape[:2]:
        raise ValueError(f"Left/right shapes differ for view_{view.view_id:02d}: {left.shape} vs {right.shape}")

    disparity, disparity_info = get_disparity(view, left, right)
    points_cam = cv2.reprojectImageTo3D(disparity.astype(np.float32), q_matrix).astype(np.float64) * SCALE_TO_METERS

    frame_type = "workspace" if view.corners_camera_m is not None else "camera"
    if frame_type == "camera":
        warn(f"view_{view.view_id:02d}: no camera marker corners found; using camera frame metrics.")

    metrics: list[dict[str, Any]] = []
    object_output_dir = output_dir / "recomputed_object_clouds"
    overlay_output_dir = output_dir / "debug_overlays"

    for object_id in object_ids:
        record = view.masks.get(object_id)
        if record is None:
            warn(f"view_{view.view_id:02d}: object_{object_id:02d} mask missing; skipping.")
            continue

        mask_raw = load_mask_record(record)
        if mask_raw is None:
            continue
        mask = clean_mask(mask_raw)
        if mask.shape[:2] != disparity.shape[:2]:
            warn(
                f"view_{view.view_id:02d} object_{object_id:02d}: mask shape {mask.shape[:2]} "
                f"does not match disparity {disparity.shape[:2]}; skipping."
            )
            continue

        valid = (
            (mask > 0)
            & np.isfinite(disparity)
            & (disparity > MIN_DISPARITY)
            & (disparity < float(disparity.shape[1]))
            & np.all(np.isfinite(points_cam), axis=2)
        )
        pts_cam = points_cam[valid].reshape(-1, 3)
        colors = left[valid].reshape(-1, 3)

        if frame_type == "workspace" and view.corners_camera_m is not None and len(pts_cam) > 0:
            pts_out = camera_to_workspace(pts_cam, view.corners_camera_m)
        else:
            pts_out = pts_cam

        raw_pcd = pcd_from_arrays(pts_out, colors)
        clean_pcd = clean_cloud(raw_pcd)
        clean_points = np.asarray(clean_pcd.points, dtype=np.float64)

        if SAVE_OBJECT_CLOUDS:
            write_cloud(object_output_dir / f"view_{view.view_id:02d}_object_{object_id:02d}_raw.ply", raw_pcd)
        if SAVE_CLEANED_OBJECT_CLOUDS:
            write_cloud(object_output_dir / f"view_{view.view_id:02d}_object_{object_id:02d}_clean.ply", clean_pcd)
        if SAVE_DEBUG_OVERLAYS:
            save_overlay(overlay_output_dir / f"view_{view.view_id:02d}_object_{object_id:02d}_mask_overlay.png", left, mask)

        row = compute_metrics(
            object_id=object_id,
            view_id=view.view_id,
            frame_type=frame_type,
            raw_count=len(pts_out),
            clean_points=clean_points,
        )
        row["mask_path"] = str(record.source)
        row["left_rect_path"] = str(view.left_path)
        row["disparity_method"] = disparity_info.get("method")
        metrics.append(row)

    return metrics, disparity_info


def determine_object_ids(views: list[ViewRecord]) -> list[int]:
    if OBJECT_IDS_TO_PROCESS is not None:
        return sorted(int(x) for x in OBJECT_IDS_TO_PROCESS)
    ids: set[int] = set()
    for view in views:
        ids.update(view.masks.keys())
    return sorted(ids)


def print_best_known(metrics: list[dict[str, Any]]) -> None:
    known = [
        m for m in metrics
        if m.get("object_id") in TRUE_DIMS_BY_OBJECT_ID and np.isfinite(float(m.get("height_error_m", float("nan"))))
    ]
    if not known:
        print("\n[BEST] No known-dimension object had a finite workspace top-height error.")
        return
    best = min(known, key=lambda m: abs(float(m["height_error_m"])))
    print(
        "\n[BEST] "
        f"view_{int(best['view_id']):02d} object_{int(best['object_id']):02d} "
        f"height_error={float(best['height_error_m']) * 1000.0:+.2f} mm "
        f"mask={best.get('mask_path')}"
    )


def main() -> None:
    print("Set RUN_DIR at the top of this file, then run python .\\recompute_object_clouds_and_validate_height.py")
    print("=" * 90)
    print("Offline Recompute Object Clouds + Validate Height")
    print("=" * 90)

    run_dir = RUN_DIR.expanduser()
    if str(run_dir) in ("", "."):
        print("[CONFIG] RUN_DIR is not set. Edit RUN_DIR in the USER SETTINGS section and run again.")
        return
    run_dir = run_dir.resolve()
    if not run_dir.exists():
        raise FileNotFoundError(f"RUN_DIR does not exist: {run_dir}")

    output_dir = Path(OUTPUT_DIR).expanduser().resolve() if OUTPUT_DIR is not None else run_dir / "recomputed_object_validation"
    output_dir.mkdir(parents=True, exist_ok=True)

    calib = load_calibration()
    if "disparity_to_depth_Q" not in calib:
        raise KeyError("Calibration is missing disparity_to_depth_Q")
    q_matrix = np.asarray(calib["disparity_to_depth_Q"], dtype=np.float64)

    views = discover_views(run_dir)
    if not views:
        raise FileNotFoundError(f"No rectified stereo pairs found in: {run_dir}")

    metadata_summary = load_metadata_for_views(run_dir, views)
    object_ids = determine_object_ids(views)
    if not object_ids:
        raise FileNotFoundError(f"No object masks found in: {run_dir}")

    print("\n[DISCOVERY]")
    print(f"  run_dir:    {run_dir}")
    print(f"  output_dir: {output_dir}")
    print(f"  views:      {[f'view_{v.view_id:02d}' for v in views]}")
    print(f"  object_ids: {[f'object_{i:02d}' for i in object_ids]}")
    print(f"  metadata:   {metadata_summary['metadata_paths'] or 'none'}")

    all_metrics: list[dict[str, Any]] = []
    disparity_infos: list[dict[str, Any]] = []
    for view in views:
        view_metrics, disparity_info = process_view(view, q_matrix, object_ids, output_dir)
        all_metrics.extend(view_metrics)
        disparity_infos.append(disparity_info)

    save_metrics(output_dir, all_metrics)
    print_table(all_metrics)
    print_best_known(all_metrics)

    workspace_found = any(m.get("frame_type") == "workspace" for m in all_metrics)
    print(f"\n[FRAME] Workspace frame found: {workspace_found}")
    print("[DISPARITY]")
    for info in disparity_infos:
        print(
            f"  view_{int(info['view_id']):02d}: "
            f"{info.get('source')} via {info.get('method')}"
            + (f" ({info.get('path')})" if info.get("path") else "")
        )

    summary = {
        "run_dir": str(run_dir),
        "output_dir": str(output_dir),
        "views": [
            {
                "view_id": v.view_id,
                "view_dir": str(v.view_dir),
                "left_rect": str(v.left_path),
                "right_rect": str(v.right_path),
                "mask_object_ids": sorted(v.masks.keys()),
                "disparity_path": str(v.disparity_path) if v.disparity_path else None,
                "has_corners_camera_m": v.corners_camera_m is not None,
                "has_corners_workspace_m": v.corners_workspace_m is not None,
            }
            for v in views
        ],
        "object_ids_processed": object_ids,
        "workspace_frame_found": workspace_found,
        "metadata": metadata_summary,
        "disparity": disparity_infos,
        "settings": {
            "HEIGHT_LOW_PERCENTILE": HEIGHT_LOW_PERCENTILE,
            "HEIGHT_BOTTOM_PERCENTILE": HEIGHT_BOTTOM_PERCENTILE,
            "HEIGHT_TOP_PERCENTILE": HEIGHT_TOP_PERCENTILE,
            "HEIGHT_HIGH_PERCENTILE": HEIGHT_HIGH_PERCENTILE,
            "HEIGHT_MIN_POINTS": HEIGHT_MIN_POINTS,
            "MIN_DISPARITY": MIN_DISPARITY,
            "SCALE_TO_METERS": SCALE_TO_METERS,
            "ERODE_MASK_PIXELS": ERODE_MASK_PIXELS,
            "ENABLE_MASK_LARGEST_COMPONENT_ONLY": ENABLE_MASK_LARGEST_COMPONENT_ONLY,
            "ENABLE_OUTLIER_REMOVAL": ENABLE_OUTLIER_REMOVAL,
            "ENABLE_RADIUS_OUTLIER_REMOVAL": ENABLE_RADIUS_OUTLIER_REMOVAL,
        },
    }
    summary_path = output_dir / "run_summary.json"
    summary_path.write_text(json.dumps(nan_to_none(summary), indent=2) + "\n", encoding="utf-8")
    log(f"[SAVE] {summary_path}")

    print(f"\n[DONE] Outputs saved to: {output_dir}")


if __name__ == "__main__":
    main()
