"""Offline workspace height analysis from a saved stereo pair."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

# ── Edit these before running ────────────────────────────────────────────────
LEFT_IMAGE    = Path(r"C:\Users\elipp\OneDrive\Documents\Grocery Bagger\YOLO_Bandaid\left\left_00003.png")
RIGHT_IMAGE   = Path(r"C:\Users\elipp\OneDrive\Documents\Grocery Bagger\YOLO_Bandaid\right\right_00003.png")
REQUIRE_PLANE = False  # False → skip plane clicks; heights are raw camera-frame Z (m)
# ─────────────────────────────────────────────────────────────────────────────

_WORKSPACE = Path(__file__).resolve().parents[2]
_LIVE_DIR  = Path(__file__).resolve().parent.parent / "live"
for _p in (_WORKSPACE, _LIVE_DIR):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from test_workspace_plane_heights import (  # noqa: E402
    DISPLAY_SCALE, MAX_OBJECT_POINTS, MIN_DISPARITY, OUTPUT_ROOT, PANEL_HEIGHT,
    RAFT_CHECKPOINT, RAFT_CORR_IMPLEMENTATION, RAFT_ITERS, RAFT_MIXED_PRECISION,
    RAFT_REPO_DIR, SAM_MODEL_LEGACY_PATH, SAM_MODEL_PATH, SAM_MODEL_TYPE,
    SCALE_TO_METERS, VOXEL_SIZE_M,
    PlaneModel, WorkspacePlaneHeightSession,
    build_rectification_maps, create_run_dir, disparity_colormap,
    load_calibration, load_sam_predictor, rectify_pair,
    reproject_disparity_to_points, resolve_sam_checkpoint,
)
from stereo_raft import compute_raft_disparity


class _OfflineSession(WorkspacePlaneHeightSession):
    """WorkspacePlaneHeightSession that optionally skips the plane-click stage."""

    def __init__(self, *args, require_plane: bool = True, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        if not require_plane:
            self.stage = "object"
            # normal=(0,0,1), offset=0 → distance_to_plane == raw Z depth in meters
            self.plane = PlaneModel(np.array([0.0, 0.0, 1.0]), 0.0, np.zeros((0, 3)))
            print("[INFO] REQUIRE_PLANE=False: heights reported as raw camera-frame Z (m).")


def main() -> None:
    left_bgr = cv2.imread(str(LEFT_IMAGE))
    right_bgr = cv2.imread(str(RIGHT_IMAGE))
    if left_bgr is None:
        raise FileNotFoundError(f"Cannot read LEFT_IMAGE: {LEFT_IMAGE}")
    if right_bgr is None:
        raise FileNotFoundError(f"Cannot read RIGHT_IMAGE: {RIGHT_IMAGE}")

    calib = load_calibration()
    rect_maps = build_rectification_maps(calib, *left_bgr.shape[:2])
    left_rect, right_rect = rectify_pair(left_bgr, right_bgr, rect_maps)

    print("[INFO] Computing RAFT disparity...")
    disparity = compute_raft_disparity(
        left_rect, right_rect,
        raft_repo_dir=RAFT_REPO_DIR, checkpoint=RAFT_CHECKPOINT,
        iters=RAFT_ITERS, corr_implementation=RAFT_CORR_IMPLEMENTATION,
        mixed_precision=RAFT_MIXED_PRECISION,
    )
    q_matrix  = calib["disparity_to_depth_Q"]
    points_3d = reproject_disparity_to_points(disparity, q_matrix, SCALE_TO_METERS)

    sam_path  = resolve_sam_checkpoint(SAM_MODEL_PATH, SAM_MODEL_LEGACY_PATH)
    predictor = load_sam_predictor(sam_path, SAM_MODEL_TYPE)

    run_dir = create_run_dir(OUTPUT_ROOT)
    cv2.imwrite(str(run_dir / "left_rect.png"), left_rect)
    cv2.imwrite(str(run_dir / "right_rect.png"), right_rect)
    np.save(run_dir / "raft_disparity.npy", disparity)
    cv2.imwrite(str(run_dir / "raft_disparity_color.png"), disparity_colormap(disparity, MIN_DISPARITY))

    args = argparse.Namespace(
        scale_to_meters=SCALE_TO_METERS, min_disparity=MIN_DISPARITY,
        max_points=MAX_OBJECT_POINTS, voxel_size_m=VOXEL_SIZE_M,
        raft_checkpoint=RAFT_CHECKPOINT, sam_model_path=sam_path,
        panel_height=PANEL_HEIGHT, display_scale=DISPLAY_SCALE,
    )
    session = _OfflineSession(
        left_rect=left_rect, right_rect=right_rect, disparity=disparity,
        points_3d_m=points_3d, projection_left=calib["projection_left_rectified"],
        projection_right=calib["projection_right_rectified"], q_matrix=q_matrix,
        predictor=predictor, run_dir=run_dir, args=args, require_plane=REQUIRE_PLANE,
    )
    session.write_metadata()
    session.run()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
