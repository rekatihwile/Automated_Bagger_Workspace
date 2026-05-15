"""
Offline YOLO-masked stereo point cloud reconstruction from saved left/right image pairs.

Pipeline:
  saved left/right pair
  -> rectify both images
  -> run YOLO segmentation on rectified left image
  -> compute StereoSGBM disparity
  -> reproject disparity to 3D using Q
  -> keep only points inside YOLO mask
  -> save object-only point cloud as .ply
  -> save debug visualization images

Run from project root:
  python workspace/scripts/offline/reconstruct_yolo_cloud_from_pairs.py --index 0

Useful:
  python workspace/scripts/offline/reconstruct_yolo_cloud_from_pairs.py --index 40
  python workspace/scripts/offline/reconstruct_yolo_cloud_from_pairs.py --all --step 10
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np


# -------------------------------------------------------------------------
# Import workspace modules
# -------------------------------------------------------------------------

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = WORKSPACE_ROOT.parent

if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import config
from ai_topface import TopFaceSegmenter


# -------------------------------------------------------------------------
# Basic utilities
# -------------------------------------------------------------------------

def load_calibration() -> dict[str, np.ndarray]:
    calib = config.load_stereo_calibration()

    if calib is None:
        raise FileNotFoundError(
            f"No calibration file found at:\n"
            f"  {config.ACTIVE_CALIBRATION_NPZ}\n\n"
            f"Run:\n"
            f"  python workspace/scripts/camera/calibrate.py"
        )

    required = [
        "image_size",
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
        raise KeyError(
            "Calibration file is missing keys:\n"
            + "\n".join(f"  - {k}" for k in missing)
        )

    return calib


def make_rectification_maps(
    calib: dict[str, np.ndarray],
    image_size: tuple[int, int],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    image_size = (width, height)
    """
    w, h = image_size

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


def make_sgbm(num_disparities: int = 128, block_size: int = 7) -> cv2.StereoSGBM:
    num_disparities = max(16, int(round(num_disparities / 16)) * 16)

    block_size = int(block_size)
    if block_size % 2 == 0:
        block_size += 1
    block_size = max(3, block_size)

    return cv2.StereoSGBM_create(  # type: ignore[attr-defined]
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


def compute_disparity(
    left_rect: np.ndarray,
    right_rect: np.ndarray,
    matcher: cv2.StereoSGBM,
) -> np.ndarray:
    gray_l = cv2.cvtColor(left_rect, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right_rect, cv2.COLOR_BGR2GRAY)

    disp_raw = matcher.compute(gray_l, gray_r)
    disp = disp_raw.astype(np.float32) / 16.0
    return disp


def disparity_to_color(disp: np.ndarray, valid: np.ndarray | None = None) -> np.ndarray:
    if valid is None:
        valid = disp > 0

    if np.any(valid):
        lo = np.percentile(disp[valid], 2)
        hi = np.percentile(disp[valid], 98)
        norm = np.clip((disp - lo) / max(hi - lo, 1e-6), 0, 1)
    else:
        norm = np.zeros_like(disp, dtype=np.float32)

    u8 = (norm * 255).astype(np.uint8)
    color = cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)
    color[~valid] = (0, 0, 0)
    return color


def clean_mask(mask: np.ndarray, open_size: int = 5, close_size: int = 9) -> np.ndarray:
    mask = (mask > 0).astype(np.uint8) * 255

    if open_size > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_size, open_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)

    if close_size > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_size, close_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    return mask


def largest_component(mask: np.ndarray) -> np.ndarray:
    mask_bool = mask > 0
    n, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask_bool.astype(np.uint8),
        connectivity=8,
    )

    if n <= 1:
        return mask

    areas = stats[1:, cv2.CC_STAT_AREA]
    best = 1 + int(np.argmax(areas))
    out = (labels == best).astype(np.uint8) * 255
    return out


def write_ply(
    path: Path,
    points_xyz: np.ndarray,
    colors_bgr: np.ndarray | None = None,
    scale: float = 1.0,
) -> None:
    """
    Write ASCII PLY.

    points_xyz shape: (N, 3)
    colors_bgr shape: (N, 3), uint8
    scale: multiply coordinates before saving.
           If calibration is in mm and you want meters, use scale=0.001.
    """
    path.parent.mkdir(parents=True, exist_ok=True)

    pts = np.asarray(points_xyz, dtype=np.float64) * float(scale)

    if colors_bgr is None:
        colors_rgb = np.full((len(pts), 3), 200, dtype=np.uint8)
    else:
        colors_rgb = np.asarray(colors_bgr, dtype=np.uint8)[:, ::-1]

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
        f.write("end_header\n")

        for p, c in zip(pts, colors_rgb):
            f.write(
                f"{p[0]:.8f} {p[1]:.8f} {p[2]:.8f} "
                f"{int(c[0])} {int(c[1])} {int(c[2])}\n"
            )


def summarize_points(points_xyz: np.ndarray, scale: float) -> dict[str, float]:
    """
    Summary in scaled units, usually meters if scale=0.001.
    """
    pts = points_xyz.astype(np.float64) * float(scale)

    med = np.median(pts, axis=0)
    p05 = np.percentile(pts, 5, axis=0)
    p95 = np.percentile(pts, 95, axis=0)
    dims = p95 - p05

    return {
        "n_points": int(len(pts)),
        "median_x": float(med[0]),
        "median_y": float(med[1]),
        "median_z": float(med[2]),
        "p05_x": float(p05[0]),
        "p05_y": float(p05[1]),
        "p05_z": float(p05[2]),
        "p95_x": float(p95[0]),
        "p95_y": float(p95[1]),
        "p95_z": float(p95[2]),
        "dim_x": float(dims[0]),
        "dim_y": float(dims[1]),
        "dim_z": float(dims[2]),
    }


def overlay_text(img: np.ndarray, lines: list[str]) -> np.ndarray:
    out = img.copy()
    y = 28

    for line in lines:
        cv2.putText(
            out,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            out,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        y += 26

    return out


def list_pairs(left_dir: Path, right_dir: Path) -> list[tuple[Path, Path]]:
    lefts = sorted(left_dir.glob("*.png")) + sorted(left_dir.glob("*.jpg")) + sorted(left_dir.glob("*.jpeg"))
    rights = sorted(right_dir.glob("*.png")) + sorted(right_dir.glob("*.jpg")) + sorted(right_dir.glob("*.jpeg"))

    if not lefts:
        raise FileNotFoundError(f"No left images found in: {left_dir}")
    if not rights:
        raise FileNotFoundError(f"No right images found in: {right_dir}")

    n = min(len(lefts), len(rights))
    return list(zip(lefts[:n], rights[:n]))


# -------------------------------------------------------------------------
# Core processing
# -------------------------------------------------------------------------

def process_pair(
    left_path: Path,
    right_path: Path,
    pair_index: int,
    calib: dict[str, np.ndarray],
    segmenter: TopFaceSegmenter,
    matcher: cv2.StereoSGBM,
    out_dir: Path,
    conf: float,
    scale_to_meters: float,
    min_disp: float,
    max_points: int,
) -> dict[str, float] | None:
    left = cv2.imread(str(left_path), cv2.IMREAD_COLOR)
    right = cv2.imread(str(right_path), cv2.IMREAD_COLOR)

    if left is None:
        print(f"[WARN] Could not read left image: {left_path}")
        return None
    if right is None:
        print(f"[WARN] Could not read right image: {right_path}")
        return None

    if left.shape[:2] != right.shape[:2]:
        raise ValueError(
            f"Left/right image sizes do not match:\n"
            f"  left:  {left.shape}\n"
            f"  right: {right.shape}"
        )

    h, w = left.shape[:2]
    image_size = (w, h)

    calib_size = tuple(int(v) for v in np.asarray(calib["image_size"]).reshape(-1))
    if calib_size != image_size:
        print(
            f"[WARN] Pair {pair_index}: calibration size {calib_size} != image size {image_size}. "
            f"Trying anyway, but rectification may be wrong."
        )

    map_lx, map_ly, map_rx, map_ry = make_rectification_maps(calib, image_size)

    left_rect = cv2.remap(left, map_lx, map_ly, cv2.INTER_LINEAR)
    right_rect = cv2.remap(right, map_rx, map_ry, cv2.INTER_LINEAR)

    seg = segmenter.detect(left_rect, conf=conf)
    if seg is None:
        print(f"[WARN] Pair {pair_index}: YOLO found no mask.")
        return None

    mask = clean_mask(seg.mask)
    mask = largest_component(mask)

    disp = compute_disparity(left_rect, right_rect, matcher)

    valid = disp > float(min_disp)
    valid &= mask > 0

    # Reproject all pixels to 3D using stereo Q.
    Q = calib["disparity_to_depth_Q"]
    points_3d = cv2.reprojectImageTo3D(disp, Q)

    valid &= np.isfinite(points_3d).all(axis=2)

    object_points = points_3d[valid]
    object_colors = left_rect[valid]

    if len(object_points) < 50:
        print(f"[WARN] Pair {pair_index}: too few valid object points: {len(object_points)}")
        return None

    # Remove extreme 3D outliers using percentile clipping.
    pts = object_points.astype(np.float64)
    p01 = np.percentile(pts, 1, axis=0)
    p99 = np.percentile(pts, 99, axis=0)

    keep = np.all((pts >= p01) & (pts <= p99), axis=1)
    object_points = object_points[keep]
    object_colors = object_colors[keep]

    if len(object_points) > max_points:
        rng = np.random.default_rng(pair_index)
        idx = rng.choice(len(object_points), size=max_points, replace=False)
        object_points = object_points[idx]
        object_colors = object_colors[idx]

    summary = summarize_points(object_points, scale=scale_to_meters)
    summary["pair_index"] = int(pair_index)
    summary["yolo_confidence"] = float(seg.confidence)

    stem = f"pair_{pair_index:05d}"
    pair_out = out_dir / stem
    pair_out.mkdir(parents=True, exist_ok=True)

    ply_path = pair_out / f"{stem}_object_cloud_meters.ply"
    write_ply(
        ply_path,
        object_points,
        object_colors,
        scale=scale_to_meters,
    )

    # Debug images
    disp_valid = disp > min_disp
    disp_color = disparity_to_color(disp, valid=disp_valid)

    mask_color = np.zeros_like(left_rect)
    mask_color[:, :, 1] = mask

    overlay = left_rect.copy()
    overlay[mask > 0] = cv2.addWeighted(
        overlay[mask > 0],
        0.65,
        np.full_like(overlay[mask > 0], (0, 255, 0)),
        0.35,
        0,
    )

    masked_disp = disp_color.copy()
    masked_disp[mask == 0] = (0, 0, 0)

    lines = [
        f"pair {pair_index} | YOLO conf={seg.confidence:.2f}",
        f"N={summary['n_points']}",
        f"median XYZ=({summary['median_x']:.4f}, {summary['median_y']:.4f}, {summary['median_z']:.4f}) m",
        f"dims p05-p95=({summary['dim_x']:.4f}, {summary['dim_y']:.4f}, {summary['dim_z']:.4f}) m",
    ]

    overlay_text_img = overlay_text(overlay, lines)

    cv2.imwrite(str(pair_out / f"{stem}_left_rect.png"), left_rect)
    cv2.imwrite(str(pair_out / f"{stem}_right_rect.png"), right_rect)
    cv2.imwrite(str(pair_out / f"{stem}_mask.png"), mask)
    cv2.imwrite(str(pair_out / f"{stem}_overlay.png"), overlay_text_img)
    cv2.imwrite(str(pair_out / f"{stem}_disparity.png"), disp_color)
    cv2.imwrite(str(pair_out / f"{stem}_masked_disparity.png"), masked_disp)

    # Also save raw numeric data if you want to inspect later.
    np.save(pair_out / f"{stem}_object_points_raw_units.npy", object_points)
    np.save(pair_out / f"{stem}_object_points_meters.npy", object_points.astype(np.float64) * scale_to_meters)

    print()
    print(f"[OK] Pair {pair_index}: {left_path.name} / {right_path.name}")
    print(f"     YOLO confidence: {seg.confidence:.3f}")
    print(f"     object points:   {summary['n_points']}")
    print(
        f"     median XYZ:      "
        f"{summary['median_x']:.4f}, "
        f"{summary['median_y']:.4f}, "
        f"{summary['median_z']:.4f} m"
    )
    print(
        f"     rough dims:      "
        f"{summary['dim_x']:.4f}, "
        f"{summary['dim_y']:.4f}, "
        f"{summary['dim_z']:.4f} m"
    )
    print(f"     saved PLY:       {ply_path}")

    return summary


def main() -> None:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--left-dir",
        type=Path,
        default=config.CAPTURE_SESSION_DIR,
        help="Folder containing left_XXXXX images.",
    )
    parser.add_argument(
        "--right-dir",
        type=Path,
        default=config.CAPTURE_SESSION_DIR,
        help="Folder containing right_XXXXX images.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=config.CALIBRATION_ROOT / "object_cloud_outputs",
        help="Output folder for PLY files and debug images.",
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=WORKSPACE_ROOT / "models" / "1000EpochModel.pt",
        help="YOLO segmentation model path.",
    )
    parser.add_argument(
        "--index",
        type=int,
        default=0,
        help="Single pair index to process.",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Process many pairs instead of one.",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=10,
        help="Step size when using --all.",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.25,
        help="YOLO confidence threshold.",
    )
    parser.add_argument(
        "--num-disparities",
        type=int,
        default=128,
        help="StereoSGBM numDisparities. Must be divisible by 16.",
    )
    parser.add_argument(
        "--block-size",
        type=int,
        default=7,
        help="StereoSGBM blockSize. Must be odd.",
    )
    parser.add_argument(
        "--min-disp",
        type=float,
        default=1.0,
        help="Minimum valid disparity in pixels.",
    )
    parser.add_argument(
        "--scale-to-meters",
        type=float,
        default=0.001,
        help=(
            "Scale factor applied to saved/printed 3D points. "
            "Your calibration uses BOARD_SQUARE_SIZE_MM, so 0.001 converts mm to meters."
        ),
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=50000,
        help="Max saved point count per object cloud.",
    )

    args = parser.parse_args()

    print("[INFO] Project root:   ", PROJECT_ROOT)
    print("[INFO] Workspace root: ", WORKSPACE_ROOT)
    print("[INFO] Left dir:       ", args.left_dir)
    print("[INFO] Right dir:      ", args.right_dir)
    print("[INFO] Output dir:     ", args.out_dir)
    print("[INFO] Model:          ", args.model)

    if not args.model.exists():
        raise FileNotFoundError(f"YOLO model not found: {args.model}")

    pairs = list_pairs(args.left_dir, args.right_dir)
    print(f"[INFO] Found {len(pairs)} stereo pairs.")

    calib = load_calibration()
    print(f"[INFO] Calibration: {config.ACTIVE_CALIBRATION_NPZ}")

    matcher = make_sgbm(args.num_disparities, args.block_size)

    segmenter = TopFaceSegmenter(
        model_path=args.model,
        sample_count=config.AI_BOUNDARY_SAMPLE_COUNT,
        input_size=config.AI_INPUT_SIZE,
    )

    args.out_dir.mkdir(parents=True, exist_ok=True)

    summaries: list[dict[str, float]] = []

    if args.all:
        indices = list(range(0, len(pairs), max(1, args.step)))
    else:
        if args.index < 0 or args.index >= len(pairs):
            raise IndexError(f"--index {args.index} outside range 0..{len(pairs)-1}")
        indices = [args.index]

    for i in indices:
        left_path, right_path = pairs[i]
        summary = process_pair(
            left_path=left_path,
            right_path=right_path,
            pair_index=i,
            calib=calib,
            segmenter=segmenter,
            matcher=matcher,
            out_dir=args.out_dir,
            conf=args.conf,
            scale_to_meters=args.scale_to_meters,
            min_disp=args.min_disp,
            max_points=args.max_points,
        )

        if summary is not None:
            summaries.append(summary)

    if summaries:
        csv_path = args.out_dir / "summary.csv"

        fieldnames = list(summaries[0].keys())
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(summaries)

        print()
        print(f"[DONE] Saved summary CSV: {csv_path}")
    else:
        print()
        print("[DONE] No valid object clouds were produced.")


if __name__ == "__main__":
    main()
