"""
Stereo SAM disparity → point cloud from an indexed pair.

Takes a rectified stereo pair + per-image SAM masks from:
  calibration/sam_indexed_pairs/pair_XXXXXX/

Computes SGBM disparity on the FULL (unmasked) images for maximum context,
then validates candidate points with one of three modes, reprojects to 3-D
with the calibration Q matrix, and saves a PLY point cloud + diagnostics.

Right-mask correspondence logic (strict / dilated)
----------------------------------------------------
After stereo rectification, matched pixels lie on the same image row.
For left pixel (xL, y) with disparity d:
    xR = round(xL - d)          ← matched column in the RIGHT image

A point is kept if ALL of:
  * d  > min_disp
  * left_mask[y, xL]  > 0
  * xR in [0, image_width)
  * right_mask[y, xR] > 0
  * reprojected XYZ is finite

Validation modes
-----------------
  strict    — use the original SAM masks exactly as saved.
  dilated   — dilate left mask by --left-mask-dilate and right mask by
              --right-mask-dilate before validation.  Tolerates small
              SAM boundary mismatch between views.  (DEFAULT)
  left-only — only require a valid disparity inside the left mask; ignore
              the right mask.  Useful for diagnosing how much the right-mask
              check is rejecting.

Outputs (calibration/masked_stereo_cloud_outputs/pair_XXXXXX/)
---------------------------------------------------------------
  object_cloud.ply
  disparity_color.png
  valid_left_pixels.png
  left_masked_disparity.png
  debug_right_mask_correspondence.png
  rejected_by_right_mask.png          ← green=accepted / red=rejected by right mask
  validation_comparison.png           ← 3-panel: left-only / strict / dilated
  summary.txt
  object_points_raw_units.npy
  object_points_meters.npy

Usage:
  python workspace/scripts/offline/masked_stereo_to_cloud.py --latest
  python workspace/scripts/offline/masked_stereo_to_cloud.py --index 0
  python workspace/scripts/offline/masked_stereo_to_cloud.py --latest \\
      --validation-mode dilated --right-mask-dilate 11 --left-mask-dilate 5
  python workspace/scripts/offline/masked_stereo_to_cloud.py --latest \\
      --validation-mode left-only
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import config

DEFAULT_PAIRS_DIR = config.CALIBRATION_ROOT / "sam_indexed_pairs"
DEFAULT_OUT_ROOT  = config.CALIBRATION_ROOT / "masked_stereo_cloud_outputs"

# ================= USER SETTINGS =================
PAIR_INDEX = 1
USE_LATEST = False
STEREO_BACKEND = "raft"  # "sgbm" or "raft"

SGBM_NUM_DISPARITIES = 128
SGBM_BLOCK_SIZE = 7
MIN_DISP = 1.0

RAFT_REPO_DIR = "external/RAFT-Stereo"
RAFT_CHECKPOINT = "workspace/models/raft_stereo/raftstereo-middlebury.pth"
RAFT_ITERS = 32
RAFT_CORR_IMPLEMENTATION = "alt"
RAFT_MIXED_PRECISION = True

VALIDATION_MODE = "dilated"  # "strict", "dilated", or "left-only"
LEFT_MASK_DILATE = 3
RIGHT_MASK_DILATE = 7

OUTLIER_MODE = "xyz"  # "z-only", "xyz", or "none"
CLIP_LOW = 1.0
CLIP_HIGH = 99.0
MAX_POINTS = 100_000
SCALE_TO_METERS = 0.001
# =================================================


# ---------------------------------------------------------------------------
# SGBM
# ---------------------------------------------------------------------------

def _make_sgbm(num_disparities: int, block_size: int) -> cv2.StereoSGBM:
    num_disparities = max(16, int(round(num_disparities / 16)) * 16)
    block_size      = max(3, block_size | 1)   # ensure odd and >= 3
    return cv2.StereoSGBM_create(  # type: ignore[attr-defined]
        minDisparity=0,
        numDisparities=num_disparities,
        blockSize=block_size,
        P1=8  * 3 * block_size ** 2,
        P2=32 * 3 * block_size ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=8,
        speckleWindowSize=100,
        speckleRange=2,
        preFilterCap=31,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )


def _compute_disparity(
    left_bgr: np.ndarray,
    right_bgr: np.ndarray,
    matcher: cv2.StereoSGBM,
) -> np.ndarray:
    gray_l = cv2.cvtColor(left_bgr,  cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)
    raw    = matcher.compute(gray_l, gray_r)
    return raw.astype(np.float32) / 16.0


# ---------------------------------------------------------------------------
# Mask helpers
# ---------------------------------------------------------------------------

def _dilate_mask(mask: np.ndarray, radius: int) -> np.ndarray:
    """Dilate a binary mask using an elliptical kernel of the given radius."""
    if radius <= 0:
        return mask.copy()
    k = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1)
    )
    return cv2.dilate(mask, k, iterations=1)


# ---------------------------------------------------------------------------
# Validation functions
# ---------------------------------------------------------------------------

def _validate_shifted_right_mask(
    disp:       np.ndarray,
    left_mask:  np.ndarray,
    right_mask: np.ndarray,
    min_disp:   float,
) -> np.ndarray:
    """
    Boolean map of valid object pixels using shifted right-mask correspondence.

    For each candidate left pixel (xL, y) with disparity d:
        xR = round(xL - d)   ← matched column in the right image after rectification.

    Kept if:
        d > min_disp
        left_mask[y, xL]  > 0
        xR in [0, W)
        right_mask[y, xR] > 0
    """
    h, w = disp.shape
    candidates = (left_mask > 0) & (disp > min_disp) & (disp < float(w))

    ys, xs = np.where(candidates)
    if len(ys) == 0:
        return np.zeros((h, w), dtype=bool)

    ds  = disp[ys, xs]
    xrs = np.round(xs.astype(np.float64) - ds).astype(np.int32)

    in_bounds      = (xrs >= 0) & (xrs < w)
    ys2, xs2, xrs2 = ys[in_bounds], xs[in_bounds], xrs[in_bounds]

    right_hit = (right_mask > 0)[ys2, xrs2]

    result = np.zeros((h, w), dtype=bool)
    result[ys2[right_hit], xs2[right_hit]] = True
    return result


def _validate_left_only(
    disp:      np.ndarray,
    left_mask: np.ndarray,
    min_disp:  float,
) -> np.ndarray:
    """Accept all left-mask pixels with valid disparity, ignoring the right mask."""
    h, w = disp.shape
    return (left_mask > 0) & (disp > min_disp) & (disp < float(w))


# ---------------------------------------------------------------------------
# Visualization helpers
# ---------------------------------------------------------------------------

def _disparity_colormap(disp: np.ndarray, mask: np.ndarray) -> np.ndarray:
    valid = (mask > 0) & (disp > 0)
    if np.any(valid):
        lo   = float(np.percentile(disp[valid], 2))
        hi   = float(np.percentile(disp[valid], 98))
        norm = np.clip((disp - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    else:
        norm = np.zeros_like(disp)
    u8    = (norm * 255).astype(np.uint8)
    color = cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)
    color[~valid] = (20, 20, 20)
    return color


def _write_ply(path: Path, xyz: np.ndarray, colors_bgr: np.ndarray) -> None:
    """Write ASCII PLY with XYZ + RGB."""
    rgb = colors_bgr[:, ::-1]   # BGR → RGB
    with path.open("w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(xyz)}\n")
        for prop in ("x", "y", "z"):
            f.write(f"property float {prop}\n")
        for prop in ("red", "green", "blue"):
            f.write(f"property uchar {prop}\n")
        f.write("end_header\n")
        for p, c in zip(xyz, rgb):
            f.write(
                f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                f"{int(c[0])} {int(c[1])} {int(c[2])}\n"
            )


def _debug_correspondence_image(
    left_bgr:   np.ndarray,
    right_bgr:  np.ndarray,
    disp:       np.ndarray,
    left_mask:  np.ndarray,
    right_mask: np.ndarray | None,
    valid:      np.ndarray,
) -> np.ndarray:
    """
    Side-by-side debug image (always uses ORIGINAL masks for contour display).
    Left  — cyan contour = left mask boundary; green pixels = valid kept points.
    Right — yellow tint  = right mask region;  green pixels = matched right pixels.
    """
    h, w    = left_bgr.shape[:2]
    left_out  = left_bgr.copy()
    right_out = right_bgr.copy()

    # Left: original mask boundary (cyan)
    lm_u8 = (left_mask > 0).astype(np.uint8) * 255
    ctrs, _ = cv2.findContours(lm_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(left_out, ctrs, -1, (255, 255, 0), 1)
    left_out[valid] = (0, 255, 0)

    if right_mask is not None:
        rm_u8 = (right_mask > 0).astype(np.uint8) * 255
        tint  = np.zeros_like(right_out)
        tint[rm_u8 > 0] = (0, 200, 200)
        right_out = cv2.addWeighted(right_out, 0.7, tint, 0.3, 0)
        ctrs2, _ = cv2.findContours(rm_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(right_out, ctrs2, -1, (0, 255, 255), 1)

    # Mark matched right pixels (green)
    ys, xs = np.where(valid)
    if len(ys) > 0:
        ds   = disp[ys, xs]
        xrs  = np.round(xs.astype(np.float64) - ds).astype(np.int32)
        ok   = (xrs >= 0) & (xrs < w)
        for y, xr in zip(ys[ok], xrs[ok]):
            right_out[y, xr] = (0, 255, 0)

    right_label = (
        "RIGHT (yellow=mask, green=matched pixels)"
        if right_mask is not None
        else "RIGHT (no mask — left-only mode)"
    )
    for img, txt in [
        (left_out,  "LEFT (green=valid, cyan=mask boundary)"),
        (right_out, right_label),
    ]:
        cv2.putText(img, txt, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, txt, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 1, cv2.LINE_AA)

    return np.hstack([left_out, right_out])


def _rejected_by_right_mask_image(
    left_bgr:  np.ndarray,
    disp:      np.ndarray,
    left_mask: np.ndarray,
    valid:     np.ndarray,
    min_disp:  float,
) -> np.ndarray:
    """
    Left image with candidate left-mask pixels coloured:
      green = accepted (in valid)
      red   = rejected by the right-mask check
    """
    h, w   = disp.shape
    result = left_bgr.copy()

    candidates = (left_mask > 0) & (disp > min_disp) & (disp < float(w))
    rejected   = candidates & ~valid

    result[valid]    = (0,   255, 0)    # green
    result[rejected] = (0,   0,   255)  # red (BGR)

    n_acc = int(np.count_nonzero(valid))
    n_rej = int(np.count_nonzero(rejected))
    label = f"green={n_acc} accepted   red={n_rej} rejected by right-mask"
    cv2.putText(result, label, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(result, label, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return result


def _validation_comparison_image(
    left_bgr:        np.ndarray,
    valid_left_only: np.ndarray,
    valid_strict:    np.ndarray | None,
    valid_dilated:   np.ndarray | None,
) -> np.ndarray:
    """Three-panel side-by-side: left-only / strict / dilated."""
    def _panel(label: str, valid: np.ndarray | None) -> np.ndarray:
        out = left_bgr.copy()
        if valid is not None:
            out[valid] = (0, 255, 0)
            n = int(np.count_nonzero(valid))
        else:
            n = 0
        txt = f"{label}  n={n}"
        cv2.putText(out, txt, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, txt, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 1, cv2.LINE_AA)
        return out

    return np.hstack([
        _panel("left-only", valid_left_only),
        _panel("strict",    valid_strict),
        _panel("dilated",   valid_dilated),
    ])


# ---------------------------------------------------------------------------
# Index resolution
# ---------------------------------------------------------------------------

def _resolve_index(pairs_dir: Path, index: int | None, use_latest: bool) -> int:
    existing = [d for d in pairs_dir.glob("pair_*") if d.is_dir()]
    if not existing:
        raise FileNotFoundError(f"No pair folders found in: {pairs_dir}")

    indices: list[int] = []
    for d in existing:
        try:
            indices.append(int(d.name.split("_")[1]))
        except (IndexError, ValueError):
            pass

    if not indices:
        raise FileNotFoundError(f"No valid pair_XXXXXX directories in: {pairs_dir}")

    if use_latest:
        return max(indices)

    if index is None:
        raise ValueError("Provide --index N or --latest.")

    if index in indices:
        return index

    raise ValueError(
        f"Index {index} not found in {pairs_dir}.\n"
        f"Available indices: {sorted(indices)}"
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compute a disparity point cloud from an indexed SAM stereo pair.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    # -- pair selection --
    parser.add_argument("--index",     type=int,  default=PAIR_INDEX,
                        help="Pair index to process.")
    parser.add_argument("--latest",    action=argparse.BooleanOptionalAction,
                        default=USE_LATEST,
                        help="Use the highest-numbered indexed pair.")
    parser.add_argument("--pairs-dir", type=Path, default=DEFAULT_PAIRS_DIR,
                        help=f"Root of indexed pairs (default: {DEFAULT_PAIRS_DIR}).")
    parser.add_argument("--out-root",  type=Path, default=DEFAULT_OUT_ROOT,
                        help=(
                            f"Root for outputs (default: {DEFAULT_OUT_ROOT}). "
                            "Backend and pair subfolders are added automatically."
                        ))

    # -- stereo backend --
    parser.add_argument("--stereo-backend", choices=["sgbm", "raft"],
                        default=STEREO_BACKEND,
                        help=f"Stereo disparity backend (default: {STEREO_BACKEND}).")

    # -- SGBM --
    parser.add_argument("--num-disparities", type=int,   default=SGBM_NUM_DISPARITIES,
                        help="SGBM numDisparities, multiple of 16 (default: 128).")
    parser.add_argument("--block-size",      type=int,   default=SGBM_BLOCK_SIZE,
                        help="SGBM blockSize, odd number (default: 7).")
    parser.add_argument("--min-disp",        type=float, default=MIN_DISP,
                        help="Minimum valid disparity value (default: 1.0).")

    # -- RAFT-Stereo --
    parser.add_argument("--raft-repo-dir", type=Path, default=Path(RAFT_REPO_DIR),
                        help=f"Path to RAFT-Stereo repo (default: {RAFT_REPO_DIR}).")
    parser.add_argument("--raft-checkpoint", type=Path, default=Path(RAFT_CHECKPOINT),
                        help=f"Path to RAFT-Stereo checkpoint (default: {RAFT_CHECKPOINT}).")
    parser.add_argument("--raft-iters", type=int, default=RAFT_ITERS,
                        help=f"RAFT update iterations (default: {RAFT_ITERS}).")
    parser.add_argument("--raft-corr-implementation",
                        choices=["reg", "alt", "reg_cuda", "alt_cuda"],
                        default=RAFT_CORR_IMPLEMENTATION,
                        help=f"RAFT correlation implementation (default: {RAFT_CORR_IMPLEMENTATION}).")
    parser.add_argument("--raft-mixed-precision", action=argparse.BooleanOptionalAction,
                        default=RAFT_MIXED_PRECISION,
                        help=f"Use RAFT mixed precision when CUDA is available (default: {RAFT_MIXED_PRECISION}).")

    # -- mask dilation --
    parser.add_argument("--left-mask-dilate",  type=int, default=LEFT_MASK_DILATE,
                        help="Dilate left mask by this radius before dilated-mode validation "
                             f"(default: {LEFT_MASK_DILATE}).")
    parser.add_argument("--right-mask-dilate", type=int, default=RIGHT_MASK_DILATE,
                        help="Dilate right mask by this radius before dilated-mode validation "
                             f"(default: {RIGHT_MASK_DILATE}).")

    # -- validation mode --
    parser.add_argument(
        "--validation-mode",
        choices=["strict", "dilated", "left-only"],
        default=VALIDATION_MODE,
        help=(
            f"Point acceptance strategy (default: {VALIDATION_MODE}).\n"
            "  strict    — original SAM masks, exact shifted right-mask check.\n"
            "  dilated   — dilated masks, tolerates SAM boundary mismatch.\n"
            "  left-only — ignore right mask; keep all left-mask pixels with valid disparity."
        ),
    )

    # -- outlier removal --
    parser.add_argument(
        "--outlier-mode",
        choices=["z-only", "xyz", "none"],
        default=OUTLIER_MODE,
        help=(
            f"Percentile outlier removal after reprojection (default: {OUTLIER_MODE}).\n"
            "  z-only — clip on Z axis only.\n"
            "  xyz    — clip independently on X, Y, and Z.\n"
            "  none   — no outlier removal (only finite-value filter)."
        ),
    )
    parser.add_argument("--clip-low",  type=float, default=CLIP_LOW,
                        help=f"Lower percentile for outlier clipping (default: {CLIP_LOW}).")
    parser.add_argument("--clip-high", type=float, default=CLIP_HIGH,
                        help=f"Upper percentile for outlier clipping (default: {CLIP_HIGH}).")

    # -- misc --
    parser.add_argument("--max-points",      type=int,   default=MAX_POINTS,
                        help=f"Downsample to at most this many points (default: {MAX_POINTS}).")
    parser.add_argument("--scale-to-meters", type=float, default=SCALE_TO_METERS,
                        help=f"Multiply XYZ by this before saving (default: {SCALE_TO_METERS} → mm→m).")

    args = parser.parse_args()
    cli_args = sys.argv[1:]

    # If USER SETTINGS have USE_LATEST=True, an explicit CLI --index should still
    # select that index unless the CLI also explicitly asks for latest.
    if "--index" in cli_args and "--latest" not in cli_args and "--no-latest" not in cli_args:
        args.latest = False

    if args.index is None and not args.latest:
        parser.error("Provide --index N or --latest.")

    # ------------------------------------------------------------------
    # Resolve pair
    # ------------------------------------------------------------------
    idx      = _resolve_index(args.pairs_dir, args.index, args.latest)
    pair_dir = args.pairs_dir / f"pair_{idx:06d}"

    print(f"[INFO] Pair index      : {idx:06d}")
    print(f"[INFO] Pair dir        : {pair_dir}")
    print(f"[INFO] Stereo backend  : {args.stereo_backend}")
    print(f"[INFO] Validation mode : {args.validation_mode}")
    print(f"[INFO] Outlier mode    : {args.outlier_mode}  "
          f"[{args.clip_low}–{args.clip_high} pct]")
    print()

    # ------------------------------------------------------------------
    # Load images
    # ------------------------------------------------------------------
    def _load(name: str, flags: int = cv2.IMREAD_COLOR,
              required: bool = True) -> np.ndarray | None:
        p   = pair_dir / name
        img = cv2.imread(str(p), flags)
        if img is None and required:
            raise FileNotFoundError(f"Could not read '{name}' from {pair_dir}")
        return img

    left_bgr  = _load("left_rect.png")
    right_bgr = _load("right_rect.png")
    left_mask_orig = _load("left_mask.png", cv2.IMREAD_GRAYSCALE)
    if left_bgr is None:
        raise FileNotFoundError(f"Could not read left image: {pair_dir / 'left_rect.png'}")
    if right_bgr is None:
        raise FileNotFoundError(f"Could not read right image: {pair_dir / 'right_rect.png'}")
    if left_mask_orig is None:
        raise FileNotFoundError(f"Could not read left mask: {pair_dir / 'left_mask.png'}")

    # right_mask is optional in left-only mode
    right_mask_orig: np.ndarray | None
    if args.validation_mode == "left-only":
        right_mask_orig = _load("right_mask.png", cv2.IMREAD_GRAYSCALE, required=False)
    else:
        right_mask_orig = _load("right_mask.png", cv2.IMREAD_GRAYSCALE, required=True)
        if right_mask_orig is None:
            raise FileNotFoundError(f"Could not read right mask: {pair_dir / 'right_mask.png'}")

    h, w = left_bgr.shape[:2]
    print(f"[INFO] Image size      : {w}×{h}")
    if args.stereo_backend == "sgbm":
        print(f"[INFO] SGBM params     : numDisparities={args.num_disparities}"
              f"  blockSize={args.block_size}  minDisp={args.min_disp}")
    else:
        print(f"[INFO] RAFT params     : iters={args.raft_iters}"
              f"  corr={args.raft_corr_implementation}"
              f"  mixedPrecision={args.raft_mixed_precision}"
              f"  minDisp={args.min_disp}")
    print()

    if left_bgr.shape[:2] != right_bgr.shape[:2]:
        raise ValueError(
            f"Left and right images must be the same size.\n"
            f"  left:  {left_bgr.shape[:2]}\n"
            f"  right: {right_bgr.shape[:2]}"
        )

    # Resize masks to image dimensions if needed
    left_mask_orig = cv2.resize(left_mask_orig, (w, h), interpolation=cv2.INTER_NEAREST)
    if right_mask_orig is not None:
        right_mask_orig = cv2.resize(right_mask_orig, (w, h), interpolation=cv2.INTER_NEAREST)

    # Dilated variants
    left_mask_dil  = _dilate_mask(left_mask_orig,  args.left_mask_dilate)
    right_mask_dil = (
        _dilate_mask(right_mask_orig, args.right_mask_dilate)
        if right_mask_orig is not None else None
    )

    # ------------------------------------------------------------------
    # Mask diagnostics
    # ------------------------------------------------------------------
    n_lm_orig = int(np.count_nonzero(left_mask_orig  > 0))
    n_lm_dil  = int(np.count_nonzero(left_mask_dil   > 0))
    n_rm_orig = int(np.count_nonzero(right_mask_orig  > 0)) if right_mask_orig is not None else 0
    n_rm_dil  = int(np.count_nonzero(right_mask_dil   > 0)) if right_mask_dil  is not None else 0

    print(f"[MASK] Left  orig / dilated(r={args.left_mask_dilate})  : "
          f"{n_lm_orig:6d} / {n_lm_dil:6d} px")
    suffix = "" if right_mask_orig is not None else "  (not loaded — left-only mode)"
    print(f"[MASK] Right orig / dilated(r={args.right_mask_dilate})  : "
          f"{n_rm_orig:6d} / {n_rm_dil:6d} px{suffix}")
    print()

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

    # ------------------------------------------------------------------
    # Compute disparity on FULL unmasked images
    # ------------------------------------------------------------------
    if args.stereo_backend == "sgbm":
        print("[INFO] Computing SGBM disparity on full rectified images …")
        matcher = _make_sgbm(args.num_disparities, args.block_size)
        disp    = _compute_disparity(left_bgr, right_bgr, matcher)
    else:
        print("[INFO] Computing RAFT-Stereo disparity on full rectified images …")
        from stereo_raft import compute_raft_disparity

        disp = compute_raft_disparity(
            left_bgr,
            right_bgr,
            raft_repo_dir=args.raft_repo_dir,
            checkpoint=args.raft_checkpoint,
            iters=args.raft_iters,
            corr_implementation=args.raft_corr_implementation,
            mixed_precision=args.raft_mixed_precision,
        )

    n_disp_valid = int(np.count_nonzero(disp > args.min_disp))
    print(f"[DISP] Pixels with disparity > {args.min_disp}: {n_disp_valid}")
    print()

    # ------------------------------------------------------------------
    # Compute all three validation maps (always — needed for comparison image)
    # ------------------------------------------------------------------
    valid_left_only: np.ndarray = _validate_left_only(
        disp, left_mask_orig, args.min_disp
    )

    valid_strict: np.ndarray | None = None
    valid_dilated: np.ndarray | None = None

    if right_mask_orig is not None:
        valid_strict = _validate_shifted_right_mask(
            disp, left_mask_orig, right_mask_orig, args.min_disp
        )
    if right_mask_dil is not None:
        valid_dilated = _validate_shifted_right_mask(
            disp, left_mask_dil, right_mask_dil, args.min_disp
        )

    n_lo = int(np.count_nonzero(valid_left_only))
    n_st = int(np.count_nonzero(valid_strict))  if valid_strict  is not None else 0
    n_di = int(np.count_nonzero(valid_dilated)) if valid_dilated is not None else 0

    print(f"[VALID] left-only candidates : {n_lo:6d} px")
    print(f"[VALID] strict (orig masks)  : {n_st:6d} px"
          + ("" if right_mask_orig is not None else "  (skipped — no right mask)"))
    print(f"[VALID] dilated              : {n_di:6d} px"
          + ("" if right_mask_dil  is not None else "  (skipped — no right mask)"))

    # Choose active validation map
    if args.validation_mode == "left-only":
        valid = valid_left_only
    elif args.validation_mode == "strict":
        if valid_strict is None:
            print("[WARN] strict mode requires right_mask.png — falling back to left-only.")
            valid = valid_left_only
        else:
            valid = valid_strict
    else:  # dilated
        if valid_dilated is None:
            print("[WARN] dilated mode requires right_mask.png — falling back to left-only.")
            valid = valid_left_only
        else:
            valid = valid_dilated

    n_chosen = int(np.count_nonzero(valid))
    print(f"[VALID] CHOSEN ({args.validation_mode:9s})   : {n_chosen:6d} px")
    print()

    # ------------------------------------------------------------------
    # Reproject to 3-D
    # ------------------------------------------------------------------
    points_3d = cv2.reprojectImageTo3D(disp, Q)   # (H, W, 3)

    xyz    = points_3d[valid].astype(np.float64)
    colors = left_bgr[valid]

    # Non-finite filter
    finite_ok = np.all(np.isfinite(xyz), axis=1)
    xyz    = xyz[finite_ok]
    colors = colors[finite_ok]

    # Outlier removal
    if args.outlier_mode == "z-only" and len(xyz) > 0:
        z  = xyz[:, 2]
        lo = np.percentile(z, args.clip_low)
        hi = np.percentile(z, args.clip_high)
        ok = (z >= lo) & (z <= hi)
        xyz, colors = xyz[ok], colors[ok]
        print(f"[CLIP] z-only  [{args.clip_low}–{args.clip_high} pct]  "
              f"z=[{lo:.1f}, {hi:.1f}]  remaining: {len(xyz)}")

    elif args.outlier_mode == "xyz" and len(xyz) > 0:
        ok = np.ones(len(xyz), dtype=bool)
        for axis, name in enumerate(("X", "Y", "Z")):
            col = xyz[:, axis]
            lo  = np.percentile(col, args.clip_low)
            hi  = np.percentile(col, args.clip_high)
            ok &= (col >= lo) & (col <= hi)
            print(f"[CLIP] {name}  [{args.clip_low}–{args.clip_high} pct]  [{lo:.1f}, {hi:.1f}]")
        xyz, colors = xyz[ok], colors[ok]
        print(f"[CLIP] After xyz clip: {len(xyz)} points remaining")

    # Optional downsample
    if len(xyz) > args.max_points:
        chosen = np.random.choice(len(xyz), args.max_points, replace=False)
        xyz    = xyz[chosen]
        colors = colors[chosen]
        print(f"[INFO] Downsampled to {args.max_points} points.")

    n_final = len(xyz)
    print(f"[INFO] Final point count: {n_final}")
    print()

    # ------------------------------------------------------------------
    # Diagnostic if very few points
    # ------------------------------------------------------------------
    if n_final < 50:
        print("[WARN] Very few valid points. Diagnostics:")
        print(f"  Left mask orig / dilated      : {n_lm_orig} / {n_lm_dil} px")
        print(f"  Right mask orig / dilated     : {n_rm_orig} / {n_rm_dil} px")
        print(f"  Disparity valid (>{args.min_disp}): {n_disp_valid} px")
        print(f"  left-only candidates          : {n_lo} px")
        print(f"  strict shifted-right-mask     : {n_st} px")
        print(f"  dilated shifted-right-mask    : {n_di} px")
        print(f"  Chosen ({args.validation_mode}): {n_chosen} px")
        print("Suggestions:")
        print("  1. Try --validation-mode left-only to bypass the right-mask check.")
        print("  2. Increase --right-mask-dilate (e.g. 15) for larger boundary tolerance.")
        print("  3. Confirm images are rectified (epipolar rows must align).")
        print("  4. Increase --num-disparities for close objects.")
        print("  5. Add texture — plain surfaces confuse SGBM.")
        print("  6. Decrease --min-disp for distant objects.")
        print()

    # ------------------------------------------------------------------
    # Save outputs
    # ------------------------------------------------------------------
    backend_out_dir = args.stereo_backend.upper()
    out_dir = args.out_root / backend_out_dir / f"pair_{idx:06d}"
    out_dir.mkdir(parents=True, exist_ok=True)

    xyz_m = xyz * args.scale_to_meters

    # PLY
    ply_path = out_dir / "object_cloud.ply"
    if n_final > 0:
        _write_ply(ply_path, xyz_m, colors)
        print(f"[SAVE] PLY:                    {ply_path}")
    else:
        print("[WARN] No points — PLY not written.")

    # NumPy arrays
    np.save(str(out_dir / "object_points_raw_units.npy"), xyz)
    np.save(str(out_dir / "object_points_meters.npy"),    xyz_m)
    print(f"[SAVE] NPY (raw):               {out_dir / 'object_points_raw_units.npy'}")
    print(f"[SAVE] NPY (meters):            {out_dir / 'object_points_meters.npy'}")

    # Disparity colourmap over original left-mask region
    disp_color = _disparity_colormap(disp, left_mask_orig)
    cv2.imwrite(str(out_dir / "disparity_color.png"), disp_color)
    print(f"[SAVE] Disparity color:         {out_dir / 'disparity_color.png'}")

    # Masked disparity (chosen valid pixels only)
    left_masked_disp = _disparity_colormap(disp, valid)
    cv2.imwrite(str(out_dir / "left_masked_disparity.png"), left_masked_disp)
    print(f"[SAVE] Masked disparity:        {out_dir / 'left_masked_disparity.png'}")

    # Valid left pixels
    valid_img = left_bgr.copy()
    valid_img[valid] = (0, 255, 0)
    cv2.imwrite(str(out_dir / "valid_left_pixels.png"), valid_img)
    print(f"[SAVE] Valid pixels:            {out_dir / 'valid_left_pixels.png'}")

    # Debug correspondence (uses original masks for contours)
    debug_img = _debug_correspondence_image(
        left_bgr, right_bgr, disp, left_mask_orig, right_mask_orig, valid
    )
    cv2.imwrite(str(out_dir / "debug_right_mask_correspondence.png"), debug_img)
    print(f"[SAVE] Debug correspondence:    {out_dir / 'debug_right_mask_correspondence.png'}")

    # Rejected-by-right-mask image
    rej_img = _rejected_by_right_mask_image(
        left_bgr, disp, left_mask_orig, valid, args.min_disp
    )
    cv2.imwrite(str(out_dir / "rejected_by_right_mask.png"), rej_img)
    print(f"[SAVE] Rejected mask:           {out_dir / 'rejected_by_right_mask.png'}")

    # Validation comparison (3-panel: left-only / strict / dilated)
    comp_img = _validation_comparison_image(
        left_bgr, valid_left_only, valid_strict, valid_dilated
    )
    cv2.imwrite(str(out_dir / "validation_comparison.png"), comp_img)
    print(f"[SAVE] Validation comparison:   {out_dir / 'validation_comparison.png'}")

    # Summary
    lines: list[str] = [
        f"Pair index        : {idx}",
        f"Input dir         : {pair_dir}",
        f"Output dir        : {out_dir}",
        f"Image size        : {w}×{h}",
        f"Stereo backend    : {args.stereo_backend}",
        f"Validation mode   : {args.validation_mode}",
        f"Outlier mode      : {args.outlier_mode}  [{args.clip_low}–{args.clip_high} pct]",
        f"Left mask dilate  : r={args.left_mask_dilate}",
        f"Right mask dilate : r={args.right_mask_dilate}",
        f"",
        f"Left mask orig / dilated   : {n_lm_orig} / {n_lm_dil} px",
        f"Right mask orig / dilated  : {n_rm_orig} / {n_rm_dil} px",
        f"Disparity valid (>{args.min_disp}) : {n_disp_valid} px",
        f"",
        f"Validation candidates:",
        f"  left-only              : {n_lo} px",
        f"  strict (orig masks)    : {n_st} px",
        f"  dilated                : {n_di} px",
        f"  CHOSEN ({args.validation_mode}) : {n_chosen} px",
        f"",
        f"Final point count : {n_final}",
    ]
    if args.stereo_backend == "sgbm":
        lines.insert(
            7,
            f"SGBM params       : numDisparities={args.num_disparities}"
            f"  blockSize={args.block_size}  minDisp={args.min_disp}",
        )
    else:
        lines.insert(
            7,
            f"RAFT params       : iters={args.raft_iters}"
            f"  corr={args.raft_corr_implementation}"
            f"  mixedPrecision={args.raft_mixed_precision}"
            f"  minDisp={args.min_disp}",
        )
    if n_final > 0:
        med  = np.median(xyz_m, axis=0)
        p05  = np.percentile(xyz_m,  5, axis=0)
        p95  = np.percentile(xyz_m, 95, axis=0)
        dims = p95 - p05
        lines += [
            f"",
            f"Median XYZ (m)    : {med[0]:.4f}  {med[1]:.4f}  {med[2]:.4f}",
            f"5th pct XYZ (m)   : {p05[0]:.4f}  {p05[1]:.4f}  {p05[2]:.4f}",
            f"95th pct XYZ (m)  : {p95[0]:.4f}  {p95[1]:.4f}  {p95[2]:.4f}",
            f"Dimensions (m)    : {dims[0]:.4f}  {dims[1]:.4f}  {dims[2]:.4f}",
            f"",
            f"PLY : {ply_path}",
        ]

    (out_dir / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[SAVE] Summary:                 {out_dir / 'summary.txt'}")

    print()
    print("━━━ Point cloud summary ━━━")
    for line in lines:
        print(f"  {line}")
    print()


if __name__ == "__main__":
    main()
