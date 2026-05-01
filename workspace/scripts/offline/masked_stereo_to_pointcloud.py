"""
Masked stereo disparity → point cloud.

Takes a rectified left image + mask and rectified right image + mask,
computes SGBM disparity restricted to the masked region, reprojects to
3-D using the calibration Q matrix, and saves a coloured ASCII PLY file.

The images are assumed to already be rectified (e.g. output of
test_rectified_disparity_live.py or test_sam_live_snapshot.py --rectify).
If they are not rectified, pass --rectify to apply the stored calibration.

Outputs saved to  <out-dir>/<stamp>_<label>_cloud.ply
                  <out-dir>/<stamp>_<label>_disparity.png   (colourmap)
                  <out-dir>/<stamp>_<label>_summary.json

Usage examples
--------------
# Minimal – just point at the four files:
  python workspace/scripts/offline/masked_stereo_to_pointcloud.py \\
      --left  calibration/sam_live_snapshot_tests/20260430_120000_left.png \\
      --right calibration/sam_live_snapshot_tests/20260430_120000_right.png \\
      --left-mask  calibration/sam_live_snapshot_tests/20260430_120000_left_mask.png \\
      --right-mask calibration/sam_live_snapshot_tests/20260430_120000_right_mask.png

# With an optional label and custom output folder:
  python workspace/scripts/offline/masked_stereo_to_pointcloud.py \\
      --left ...  --right ...  --left-mask ... --right-mask ... \\
      --label cereal_box  --out-dir calibration/object_cloud_outputs

# If images are NOT yet rectified:
  python workspace/scripts/offline/masked_stereo_to_pointcloud.py \\
      --left ...  --right ...  --left-mask ... --right-mask ... \\
      --rectify
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Workspace path setup
# ---------------------------------------------------------------------------

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import config


# ---------------------------------------------------------------------------
# Helpers – rectification
# ---------------------------------------------------------------------------

def _build_rect_maps(
    h: int, w: int
) -> tuple[tuple[np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]]:
    calib = config.load_stereo_calibration()
    if calib is None:
        raise FileNotFoundError(
            f"No calibration file found at:\n  {config.ACTIVE_CALIBRATION_NPZ}"
        )
    required = [
        "left_camera_matrix", "left_distortion_coefficients",
        "rectification_left", "projection_left_rectified",
        "right_camera_matrix", "right_distortion_coefficients",
        "rectification_right", "projection_right_rectified",
    ]
    missing = [k for k in required if k not in calib]
    if missing:
        raise KeyError("Calibration missing keys:\n" + "\n".join(f"  {k}" for k in missing))

    left_maps = cv2.initUndistortRectifyMap(
        calib["left_camera_matrix"],
        calib["left_distortion_coefficients"],
        calib["rectification_left"],
        calib["projection_left_rectified"],
        (w, h),
        cv2.CV_32FC1,
    )
    right_maps = cv2.initUndistortRectifyMap(
        calib["right_camera_matrix"],
        calib["right_distortion_coefficients"],
        calib["rectification_right"],
        calib["projection_right_rectified"],
        (w, h),
        cv2.CV_32FC1,
    )
    return left_maps, right_maps


# ---------------------------------------------------------------------------
# Helpers – SGBM
# ---------------------------------------------------------------------------

def _make_sgbm(num_disparities: int = 128, block_size: int = 7) -> cv2.StereoSGBM:
    num_disparities = max(16, int(round(num_disparities / 16)) * 16)
    block_size = max(3, block_size | 1)  # ensure odd, >=3
    return cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disparities,
        blockSize=block_size,
        P1=8 * 3 * block_size ** 2,
        P2=32 * 3 * block_size ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=8,
        speckleWindowSize=100,
        speckleRange=2,
        preFilterCap=31,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )


# ---------------------------------------------------------------------------
# Helpers – disparity
# ---------------------------------------------------------------------------

def _compute_disparity(
    left_bgr: np.ndarray,
    right_bgr: np.ndarray,
    matcher: cv2.StereoSGBM,
) -> np.ndarray:
    gray_l = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)
    disp_raw = matcher.compute(gray_l, gray_r)
    return disp_raw.astype(np.float32) / 16.0


def _disparity_colormap(disp: np.ndarray, valid: np.ndarray) -> np.ndarray:
    if np.any(valid):
        lo = float(np.percentile(disp[valid], 2))
        hi = float(np.percentile(disp[valid], 98))
        norm = np.clip((disp - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    else:
        norm = np.zeros_like(disp, dtype=np.float32)

    u8 = (norm * 255).astype(np.uint8)
    color = cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)
    color[~valid] = (30, 30, 30)
    return color


# ---------------------------------------------------------------------------
# Helpers – PLY
# ---------------------------------------------------------------------------

def _write_ply(path: Path, xyz: np.ndarray, colors_bgr: np.ndarray) -> None:
    """Write an ASCII PLY with XYZ + RGB (uint8 colour from bgr)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    rgb = colors_bgr[:, ::-1]  # BGR → RGB

    with path.open("w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(xyz)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        for p, c in zip(xyz, rgb):
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {int(c[0])} {int(c[1])} {int(c[2])}\n")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _find_latest_snapshot(
    snap_dir: Path,
) -> tuple[Path, Path, Path, Path] | None:
    """
    Scan snap_dir for the most recent set of files that has all four of:
      *_left_rect.png  *_right_rect.png  *_left_mask.png  *_right_mask.png
    OR (fallback for test_sam_live_snapshot.py output):
      *_left.png  *_right.png  *_left_mask.png  *_right_mask.png

    Returns (left, right, left_mask, right_mask) paths or None.
    """
    # Gather timestamps that have all four files for each naming convention
    suffixes = [
        ("_left_rect.png", "_right_rect.png", "_left_mask.png", "_right_mask.png"),
        ("_left.png",      "_right.png",      "_left_mask.png", "_right_mask.png"),
    ]

    candidates: list[tuple[str, Path, Path, Path, Path]] = []

    for ls, rs, lms, rms in suffixes:
        for lf in sorted(snap_dir.glob(f"*{ls}"), reverse=True):
            stamp = lf.name[: -len(ls)]
            rf  = snap_dir / f"{stamp}{rs}"
            lmf = snap_dir / f"{stamp}{lms}"
            rmf = snap_dir / f"{stamp}{rms}"
            if rf.exists() and lmf.exists() and rmf.exists():
                candidates.append((stamp, lf, rf, lmf, rmf))
                break  # found the latest for this suffix pattern

    if not candidates:
        return None

    # Pick the most-recent stamp across patterns
    candidates.sort(key=lambda t: t[0], reverse=True)
    _, lf, rf, lmf, rmf = candidates[0]
    return lf, rf, lmf, rmf


def main() -> None:
    default_out     = config.CALIBRATION_ROOT / "object_cloud_outputs"
    default_snap    = config.CALIBRATION_ROOT / "sam_live_snapshot_tests"

    parser = argparse.ArgumentParser(
        description=(
            "Compute a disparity-based point cloud from a rectified stereo pair "
            "and per-image SAM masks, then save a PLY file."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--left",       type=Path, default=None, help="Rectified left image.")
    parser.add_argument("--right",      type=Path, default=None, help="Rectified right image.")
    parser.add_argument("--left-mask",  type=Path, default=None,
                        help="Left mask PNG (white = object, black = background).")
    parser.add_argument("--right-mask", type=Path, default=None,
                        help="Right mask PNG (white = object, black = background).")
    parser.add_argument("--latest",     action="store_true",
                        help="Auto-select the most recent snapshot set from --snap-dir.")
    parser.add_argument("--snap-dir",   type=Path, default=default_snap,
                        help=f"Directory to search when using --latest "
                             f"(default: {default_snap}).")
    parser.add_argument("--label",      type=str,  default="",
                        help="Optional label appended to output filenames.")
    parser.add_argument("--out-dir",    type=Path, default=default_out,
                        help=f"Output directory (default: {default_out}).")
    parser.add_argument("--rectify",    action="store_true",
                        help="Rectify the input images before processing.")
    parser.add_argument("--num-disparities", type=int, default=128,
                        help="SGBM numDisparities (must be divisible by 16, default: 128).")
    parser.add_argument("--block-size",      type=int, default=7,
                        help="SGBM blockSize (must be odd, default: 7).")
    parser.add_argument("--min-depth",       type=float, default=0.0,
                        help="Discard points closer than this depth (mm, default: 0).")
    parser.add_argument("--max-depth",       type=float, default=5000.0,
                        help="Discard points farther than this depth (mm, default: 5000).")
    parser.add_argument("--scale",           type=float, default=1.0,
                        help="Scale factor applied to XYZ before saving "
                             "(e.g. 0.001 to convert mm→m, default: 1.0).")
    args = parser.parse_args()

    # ------------------------------------------------------------------
    # Resolve --latest
    # ------------------------------------------------------------------
    if args.latest:
        result = _find_latest_snapshot(args.snap_dir)
        if result is None:
            raise FileNotFoundError(
                f"No complete snapshot set found in: {args.snap_dir}\n"
                "Expected files matching  TIMESTAMP_left(_rect).png, "
                "TIMESTAMP_right(_rect).png, TIMESTAMP_left_mask.png, "
                "TIMESTAMP_right_mask.png"
            )
        args.left, args.right, args.left_mask, args.right_mask = result
        print(f"[INFO] --latest resolved to stamp: {args.left.name.split('_left')[0]}")
    else:
        missing = [n for n, v in [("--left", args.left), ("--right", args.right),
                                   ("--left-mask", args.left_mask),
                                   ("--right-mask", args.right_mask)] if v is None]
        if missing:
            parser.error(
                f"The following arguments are required (or use --latest): "
                + ", ".join(missing)
            )

    # ------------------------------------------------------------------
    # Load images
    # ------------------------------------------------------------------
    def _load(path: Path, name: str, flags: int = cv2.IMREAD_COLOR) -> np.ndarray:
        img = cv2.imread(str(path), flags)
        if img is None:
            raise FileNotFoundError(f"Could not read {name}: {path}")
        return img

    left_bgr  = _load(args.left,       "left image")
    right_bgr = _load(args.right,      "right image")
    left_mask = _load(args.left_mask,  "left mask",  cv2.IMREAD_GRAYSCALE)
    right_mask = _load(args.right_mask, "right mask", cv2.IMREAD_GRAYSCALE)

    print(f"[INFO] Left:        {args.left}  {left_bgr.shape}")
    print(f"[INFO] Right:       {args.right}  {right_bgr.shape}")
    print(f"[INFO] Left mask:   {args.left_mask}")
    print(f"[INFO] Right mask:  {args.right_mask}")

    if left_bgr.shape[:2] != right_bgr.shape[:2]:
        raise ValueError(
            f"Left and right images must have the same resolution.\n"
            f"  left:  {left_bgr.shape[:2]}\n"
            f"  right: {right_bgr.shape[:2]}"
        )

    h, w = left_bgr.shape[:2]

    # Resize masks to match image if needed
    for name, mask_arr, ref in [("left mask", left_mask, left_bgr),
                                  ("right mask", right_mask, right_bgr)]:
        if mask_arr.shape[:2] != ref.shape[:2]:
            print(f"[WARN] {name} size {mask_arr.shape[:2]} != image size {ref.shape[:2]}; resizing.")
    left_mask  = cv2.resize(left_mask,  (w, h), interpolation=cv2.INTER_NEAREST)
    right_mask = cv2.resize(right_mask, (w, h), interpolation=cv2.INTER_NEAREST)

    # ------------------------------------------------------------------
    # Optional rectification
    # ------------------------------------------------------------------
    if args.rectify:
        print("[INFO] Building rectification maps …")
        (lx, ly), (rx, ry) = _build_rect_maps(h, w)
        left_bgr   = cv2.remap(left_bgr,   lx, ly, cv2.INTER_LINEAR)
        right_bgr  = cv2.remap(right_bgr,  rx, ry, cv2.INTER_LINEAR)
        left_mask  = cv2.remap(left_mask,  lx, ly, cv2.INTER_NEAREST)
        right_mask = cv2.remap(right_mask, rx, ry, cv2.INTER_NEAREST)
        print("[INFO] Rectification done.")

    # ------------------------------------------------------------------
    # Load calibration Q matrix
    # ------------------------------------------------------------------
    calib = config.load_stereo_calibration()
    if calib is None:
        raise FileNotFoundError(
            f"No calibration file found at:\n  {config.ACTIVE_CALIBRATION_NPZ}"
        )
    if "disparity_to_depth_Q" not in calib:
        raise KeyError(
            "Calibration does not contain 'disparity_to_depth_Q'.\n"
            "Re-run calibrate.py to regenerate the calibration file."
        )
    Q = calib["disparity_to_depth_Q"]
    print(f"[INFO] Q matrix loaded from: {config.ACTIVE_CALIBRATION_NPZ}")

    # ------------------------------------------------------------------
    # Compute disparity on full images (SGBM needs context outside mask).
    # Apply a dilated right mask to blank out background in the right image
    # so background texture doesn't produce false matches inside the object.
    # ------------------------------------------------------------------
    print(f"[INFO] Computing SGBM disparity (numDisparities={args.num_disparities}, "
          f"blockSize={args.block_size}) …")

    right_bgr_masked = right_bgr.copy()
    if right_mask is not None:
        # Dilate generously to account for disparity shift
        dil_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (61, 61))
        right_mask_dilated = cv2.dilate((right_mask > 0).astype(np.uint8) * 255, dil_k)
        right_bgr_masked[right_mask_dilated == 0] = 0  # zero out background

    matcher = _make_sgbm(args.num_disparities, args.block_size)
    disp = _compute_disparity(left_bgr, right_bgr_masked, matcher)

    # ------------------------------------------------------------------
    # Build final mask.
    #
    # Disparity is computed in LEFT image coordinates: each pixel (x,y) in
    # the left image is matched to pixel (x - disparity, y) in the right.
    # The right mask therefore covers a HORIZONTALLY SHIFTED region and must
    # NOT be AND-ed with the left mask in pixel space — doing so produces
    # only the narrow overlap sliver.
    #
    # Correct approach:
    #   • Point selection  → left mask only (disparity lives in left coords)
    #   • Right mask use   → dilate it and apply to right image before SGBM
    #                        so background texture doesn't bleed into matches
    # ------------------------------------------------------------------
    left_bool  = left_mask > 0
    disp_valid = (disp > 0) & (disp < args.num_disparities)
    final_mask = left_bool & disp_valid

    n_masked = int(np.count_nonzero(left_bool))
    n_valid  = int(np.count_nonzero(final_mask))
    print(f"[INFO] Left-mask pixels: {n_masked}  valid disparity in mask: {n_valid}")

    if n_valid == 0:
        print("[WARN] No valid disparity pixels inside the mask. "
              "Check that both masks cover the same object area and the "
              "images are rectified.")

    # ------------------------------------------------------------------
    # Reproject to 3-D
    # ------------------------------------------------------------------
    # Compute 3-D for all pixels, then select masked ones
    points_3d = cv2.reprojectImageTo3D(disp, Q)  # (h, w, 3)  units = same as calibration (mm)

    xyz    = points_3d[final_mask]                  # (N, 3)
    colors = left_bgr[final_mask]                   # (N, 3) BGR

    # Depth filter
    depth = xyz[:, 2]
    depth_ok = (depth >= args.min_depth) & (depth <= args.max_depth) & np.isfinite(depth)
    xyz    = xyz[depth_ok]
    colors = colors[depth_ok]
    print(f"[INFO] Points after depth filter [{args.min_depth:.0f}, {args.max_depth:.0f}] mm: "
          f"{len(xyz)}")

    if args.scale != 1.0:
        xyz = xyz * args.scale
        print(f"[INFO] Coordinates scaled by {args.scale}.")

    # ------------------------------------------------------------------
    # Save outputs
    # ------------------------------------------------------------------
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp  = datetime.now().strftime("%Y%m%d_%H%M%S")
    suffix = f"_{args.label}" if args.label else ""
    base   = args.out_dir / f"{stamp}{suffix}"

    ply_path  = Path(f"{base}_cloud.ply")
    disp_path = Path(f"{base}_disparity.png")
    json_path = Path(f"{base}_summary.json")

    # PLY
    if len(xyz) > 0:
        _write_ply(ply_path, xyz, colors)
        print(f"[SAVE] Point cloud: {ply_path}  ({len(xyz)} points)")
    else:
        print("[WARN] No points to save – PLY file not written.")

    # Disparity colourmap (masked)
    disp_color = _disparity_colormap(disp, final_mask)
    cv2.imwrite(str(disp_path), disp_color)
    print(f"[SAVE] Disparity:   {disp_path}")

    # Summary JSON
    summary: dict = {
        "timestamp": stamp,
        "label": args.label,
        "left_image":  str(args.left),
        "right_image": str(args.right),
        "left_mask":   str(args.left_mask),
        "right_mask":  str(args.right_mask),
        "calibration": str(config.ACTIVE_CALIBRATION_NPZ),
        "rectified_input": args.rectify,
        "sgbm_num_disparities": args.num_disparities,
        "sgbm_block_size": args.block_size,
        "depth_range_mm": [args.min_depth, args.max_depth],
        "scale_applied": args.scale,
        "n_masked_pixels": n_masked,
        "n_valid_disparity_pixels": n_valid,
        "n_points_saved": int(len(xyz)),
    }

    if len(xyz) > 0:
        med   = np.median(xyz, axis=0).tolist()
        p05   = np.percentile(xyz, 5, axis=0).tolist()
        p95   = np.percentile(xyz, 95, axis=0).tolist()
        dims  = [p95[i] - p05[i] for i in range(3)]
        unit  = "m" if args.scale == 0.001 else ("mm" if args.scale == 1.0 else f"(×{args.scale} mm)")
        summary.update({
            "units": unit,
            "median_xyz": med,
            "p05_xyz": p05,
            "p95_xyz": p95,
            "dims_p05_p95": dims,
        })

    with json_path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
        f.write("\n")
    print(f"[SAVE] Summary:     {json_path}")
    print("[INFO] Done.")


if __name__ == "__main__":
    main()
