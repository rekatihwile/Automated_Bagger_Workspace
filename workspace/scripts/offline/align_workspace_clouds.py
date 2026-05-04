"""
Align multiple ArUco workspace point clouds by forcing their 4 marker corners
to match a common reference frame.

Use case:
    You captured the same object from multiple angles.
    Each run produced:
        metadata.json
        object_00_cloud_workspace.ply
        combined_objects_cloud.ply
        workspace_floor_and_objects.ply

Problem:
    The clouds face the right way, but their floor planes / ArUco rectangles
    do not perfectly align because stereo scale/depth varies between captures.

This script:
    1. Finds run_* folders.
    2. Reads each run's metadata.json.
    3. Uses corners_workspace_m from metadata.
    4. Uses the first run as the reference unless --reference is provided.
    5. Computes a planar homography from each run's four ArUco corners to
       the reference run's four ArUco corners.
    6. Applies that warp to all workspace PLY files.
    7. Saves aligned PLY files into an aligned_output folder.

Important:
    This is NOT physically perfect 3D reconstruction.
    It is a practical visualization alignment that forces the workspace plane
    to match exactly.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


CORNER_ORDER = ["TL", "TR", "BR", "BL"]


# ---------------------------------------------------------------------------
# PLY IO
# ---------------------------------------------------------------------------

def read_ascii_ply(path: Path) -> tuple[np.ndarray, np.ndarray | None, list[str], list[str]]:
    """
    Read a simple ASCII PLY containing vertex x y z and optional RGB.

    Returns:
        xyz: Nx3 float64
        rgb: Nx3 uint8 or None
        header_lines: original header lines
        extra_lines_after_vertices: face lines or other body lines after vertices
    """
    text = path.read_text(encoding="utf-8", errors="replace").splitlines()

    if not text or text[0].strip() != "ply":
        raise ValueError(f"{path} is not a PLY file.")

    if "format ascii" not in "\n".join(text[:10]):
        raise ValueError(f"{path} is not ASCII PLY. This script only supports ASCII PLY.")

    header_end_idx = None
    vertex_count = None
    has_rgb = False

    for i, line in enumerate(text):
        s = line.strip()

        if s.startswith("element vertex"):
            vertex_count = int(s.split()[-1])

        if s == "property uchar red":
            has_rgb = True

        if s == "end_header":
            header_end_idx = i
            break

    if header_end_idx is None:
        raise ValueError(f"{path} has no end_header.")

    if vertex_count is None:
        raise ValueError(f"{path} has no element vertex line.")

    header_lines = text[: header_end_idx + 1]
    vertex_lines = text[header_end_idx + 1 : header_end_idx + 1 + vertex_count]
    extra_lines = text[header_end_idx + 1 + vertex_count :]

    xyz = np.zeros((vertex_count, 3), dtype=np.float64)
    rgb = np.zeros((vertex_count, 3), dtype=np.uint8) if has_rgb else None

    for i, line in enumerate(vertex_lines):
        parts = line.split()

        if len(parts) < 3:
            raise ValueError(f"Bad vertex line in {path}: {line}")

        xyz[i, 0] = float(parts[0])
        xyz[i, 1] = float(parts[1])
        xyz[i, 2] = float(parts[2])

        if has_rgb and rgb is not None and len(parts) >= 6:
            rgb[i, 0] = int(float(parts[3]))
            rgb[i, 1] = int(float(parts[4]))
            rgb[i, 2] = int(float(parts[5]))

    return xyz, rgb, header_lines, extra_lines


def write_ascii_ply(
    path: Path,
    xyz: np.ndarray,
    rgb: np.ndarray | None,
    extra_lines: list[str] | None = None,
) -> None:
    """
    Write ASCII PLY with x y z and optional RGB.
    Preserves face lines if extra_lines are passed.
    """
    path.parent.mkdir(parents=True, exist_ok=True)

    xyz = np.asarray(xyz, dtype=np.float64)

    has_rgb = rgb is not None
    if has_rgb:
        rgb = np.asarray(rgb, dtype=np.uint8)
        if rgb.shape != xyz.shape:
            raise ValueError("RGB shape must match XYZ shape.")

    extra_lines = extra_lines or []

    face_lines = [line for line in extra_lines if line.strip()]
    face_count = len(face_lines)

    with path.open("w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(xyz)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")

        if has_rgb:
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")

        if face_count > 0:
            f.write(f"element face {face_count}\n")
            f.write("property list uchar int vertex_indices\n")

        f.write("end_header\n")

        if has_rgb:
            assert rgb is not None
            for p, c in zip(xyz, rgb):
                f.write(
                    f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                    f"{int(c[0])} {int(c[1])} {int(c[2])}\n"
                )
        else:
            for p in xyz:
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

        for line in face_lines:
            f.write(line.rstrip() + "\n")


# ---------------------------------------------------------------------------
# Metadata / corner handling
# ---------------------------------------------------------------------------

def load_metadata(run_dir: Path) -> dict[str, Any]:
    metadata_path = run_dir / "metadata.json"

    if not metadata_path.exists():
        raise FileNotFoundError(f"No metadata.json found in {run_dir}")

    return json.loads(metadata_path.read_text(encoding="utf-8"))


def get_workspace_corners_xy(metadata: dict[str, Any]) -> np.ndarray:
    """
    Return 4x2 array of TL, TR, BR, BL corner XY positions.

    Prefers corners_workspace_m, which your latest script writes.
    """
    if "corners_workspace_m" not in metadata:
        raise KeyError(
            "metadata.json does not contain corners_workspace_m. "
            "You need to run the newer capture script that saves workspace-frame corners."
        )

    corners = metadata["corners_workspace_m"]

    missing = [name for name in CORNER_ORDER if name not in corners]
    if missing:
        raise KeyError(f"Missing required corners in metadata: {missing}")

    pts = []

    for name in CORNER_ORDER:
        p = corners[name]
        pts.append([float(p[0]), float(p[1])])

    return np.asarray(pts, dtype=np.float64)


def edge_lengths(corners_xy: np.ndarray) -> dict[str, float]:
    TL, TR, BR, BL = corners_xy

    return {
        "top": float(np.linalg.norm(TR - TL)),
        "right": float(np.linalg.norm(BR - TR)),
        "bottom": float(np.linalg.norm(BR - BL)),
        "left": float(np.linalg.norm(BL - TL)),
        "width_avg": float(0.5 * (np.linalg.norm(TR - TL) + np.linalg.norm(BR - BL))),
        "depth_avg": float(0.5 * (np.linalg.norm(BL - TL) + np.linalg.norm(BR - TR))),
    }


# ---------------------------------------------------------------------------
# Alignment math
# ---------------------------------------------------------------------------

def compute_homography(src_xy: np.ndarray, dst_xy: np.ndarray) -> np.ndarray:
    """
    Compute 3x3 homography mapping source corner XY to destination corner XY.
    """
    if src_xy.shape != (4, 2):
        raise ValueError(f"src_xy must be 4x2, got {src_xy.shape}")

    if dst_xy.shape != (4, 2):
        raise ValueError(f"dst_xy must be 4x2, got {dst_xy.shape}")

    H, status = cv2.findHomography(
        src_xy.astype(np.float64),
        dst_xy.astype(np.float64),
        method=0,
    )

    if H is None:
        raise RuntimeError("cv2.findHomography failed.")

    return H.astype(np.float64)


def apply_homography_to_xy(xy: np.ndarray, H: np.ndarray) -> np.ndarray:
    """
    Apply planar homography to Nx2 XY points.
    """
    xy = np.asarray(xy, dtype=np.float64)

    ones = np.ones((xy.shape[0], 1), dtype=np.float64)
    homog = np.hstack([xy, ones])

    warped = homog @ H.T

    denom = warped[:, 2:3]
    bad = np.abs(denom[:, 0]) < 1e-12

    if np.any(bad):
        print(f"[WARN] {np.count_nonzero(bad)} points had near-zero homography denominator.")

    denom = np.where(np.abs(denom) < 1e-12, np.nan, denom)

    return warped[:, :2] / denom


def compute_z_scale(src_xy: np.ndarray, dst_xy: np.ndarray) -> float:
    """
    Estimate a reasonable Z scale from average XY edge-length scale.

    Because the homography only acts on the workspace plane, Z needs a separate
    scale factor. This uses average width/depth ratio.
    """
    src_lengths = edge_lengths(src_xy)
    dst_lengths = edge_lengths(dst_xy)

    sx = dst_lengths["width_avg"] / max(src_lengths["width_avg"], 1e-12)
    sy = dst_lengths["depth_avg"] / max(src_lengths["depth_avg"], 1e-12)

    return float(0.5 * (sx + sy))


def transform_cloud_to_reference(
    xyz: np.ndarray,
    H: np.ndarray,
    z_scale: float,
) -> np.ndarray:
    """
    Transform workspace cloud into reference workspace frame.

    XY:
        homography so the 4 marker corners match exactly.

    Z:
        scaled by average XY scale.
    """
    xyz = np.asarray(xyz, dtype=np.float64)

    out = xyz.copy()
    out[:, :2] = apply_homography_to_xy(xyz[:, :2], H)
    out[:, 2] = xyz[:, 2] * z_scale

    return out


# ---------------------------------------------------------------------------
# Main processing
# ---------------------------------------------------------------------------

def find_run_dirs(root: Path) -> list[Path]:
    candidates = []

    for p in root.iterdir():
        if p.is_dir() and (p / "metadata.json").exists():
            candidates.append(p)

    candidates = sorted(candidates)

    if not candidates:
        raise FileNotFoundError(f"No run folders with metadata.json found in {root}")

    return candidates


def align_run(
    run_dir: Path,
    output_dir: Path,
    src_corners_xy: np.ndarray,
    dst_corners_xy: np.ndarray,
    ply_patterns: list[str],
) -> None:
    H = compute_homography(src_corners_xy, dst_corners_xy)
    z_scale = compute_z_scale(src_corners_xy, dst_corners_xy)

    src_lengths = edge_lengths(src_corners_xy)
    dst_lengths = edge_lengths(dst_corners_xy)

    print(f"\n[RUN] {run_dir.name}")
    print(f"  source width/depth: {src_lengths['width_avg']:.6f} m / {src_lengths['depth_avg']:.6f} m")
    print(f"  target width/depth: {dst_lengths['width_avg']:.6f} m / {dst_lengths['depth_avg']:.6f} m")
    print(f"  z_scale: {z_scale:.6f}")

    run_out = output_dir / run_dir.name
    run_out.mkdir(parents=True, exist_ok=True)

    # Save the warped corner positions for debugging.
    warped_corners = apply_homography_to_xy(src_corners_xy, H)
    corner_debug = {
        name: {
            "source_xy_m": src_corners_xy[i].tolist(),
            "target_xy_m": dst_corners_xy[i].tolist(),
            "warped_xy_m": warped_corners[i].tolist(),
            "error_m": float(np.linalg.norm(warped_corners[i] - dst_corners_xy[i])),
        }
        for i, name in enumerate(CORNER_ORDER)
    }

    (run_out / "corner_alignment_debug.json").write_text(
        json.dumps(
            {
                "run": run_dir.name,
                "homography": H.tolist(),
                "z_scale": z_scale,
                "corners": corner_debug,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    ply_files: list[Path] = []

    for pattern in ply_patterns:
        ply_files.extend(sorted(run_dir.glob(pattern)))

    # Remove duplicates while preserving order.
    seen = set()
    unique_ply_files = []

    for p in ply_files:
        if p not in seen:
            unique_ply_files.append(p)
            seen.add(p)

    if not unique_ply_files:
        print("  [WARN] No matching PLY files found.")
        return

    for ply_path in unique_ply_files:
        try:
            xyz, rgb, header, extra_lines = read_ascii_ply(ply_path)
            aligned_xyz = transform_cloud_to_reference(xyz, H, z_scale)

            out_path = run_out / f"{ply_path.stem}_aligned_to_reference.ply"

            write_ascii_ply(
                out_path,
                aligned_xyz,
                rgb,
                extra_lines=extra_lines,
            )

            print(f"  saved: {out_path}")

        except Exception as exc:
            print(f"  [ERROR] Failed on {ply_path.name}: {exc}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Align ArUco workspace point clouds by forcing marker corners to match."
    )

    parser.add_argument(
        "root",
        type=Path,
        help="Folder containing run_* folders, each with metadata.json and PLY files.",
    )

    parser.add_argument(
        "--reference",
        type=str,
        default=None,
        help=(
            "Reference run folder name. Example: run_20260504_153000. "
            "If omitted, the first run folder is used."
        ),
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output folder. Default: <root>/aligned_output",
    )

    parser.add_argument(
        "--ply-pattern",
        action="append",
        default=None,
        help=(
            "PLY glob pattern to align. Can be passed multiple times. "
            "Default aligns object_*_cloud_workspace.ply, combined_objects_cloud.ply, "
            "and workspace_floor_and_objects.ply."
        ),
    )

    args = parser.parse_args()

    root = args.root.resolve()
    output_dir = args.output.resolve() if args.output else root / "aligned_output"

    ply_patterns = args.ply_pattern or [
        "object_*_cloud_workspace.ply",
        "combined_objects_cloud.ply",
        "workspace_floor_and_objects.ply",
    ]

    run_dirs = find_run_dirs(root)

    print("[INFO] Found run folders:")
    for r in run_dirs:
        print(f"  {r.name}")

    if args.reference is None:
        ref_dir = run_dirs[0]
    else:
        matches = [r for r in run_dirs if r.name == args.reference]
        if not matches:
            raise FileNotFoundError(f"Reference run not found: {args.reference}")
        ref_dir = matches[0]

    print(f"\n[INFO] Reference run: {ref_dir.name}")

    ref_metadata = load_metadata(ref_dir)
    ref_corners_xy = get_workspace_corners_xy(ref_metadata)

    print("[INFO] Reference corner XY positions:")
    for name, p in zip(CORNER_ORDER, ref_corners_xy):
        print(f"  {name}: x={p[0]:+.6f} m, y={p[1]:+.6f} m")

    output_dir.mkdir(parents=True, exist_ok=True)

    for run_dir in run_dirs:
        metadata = load_metadata(run_dir)
        src_corners_xy = get_workspace_corners_xy(metadata)

        align_run(
            run_dir=run_dir,
            output_dir=output_dir,
            src_corners_xy=src_corners_xy,
            dst_corners_xy=ref_corners_xy,
            ply_patterns=ply_patterns,
        )

    print(f"\n[DONE] Aligned files written to:\n  {output_dir}")


if __name__ == "__main__":
    main()