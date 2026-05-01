# AGENTS.md - Automated Grocery Bagger Workspace

## Project Context

This project is an automated grocery bagger using an HBVCam USB stereo camera
and a SCARA-like RRPR manipulator. The current computer vision pipeline is:

rectified stereo pair -> SAM masks -> RAFT/SGBM disparity -> object point cloud
-> centroid/height/workspace coordinates.

## Repository Layout

- `workspace/` contains source code.
- `workspace/scripts/live/` contains live camera/UI experiments.
- `workspace/scripts/offline/` contains post-processing scripts.
- `workspace/scripts/utils/` contains shared script helpers.
- `workspace/models/` contains model checkpoints and weights.
- `workspace/models/sam/` contains SAM checkpoints.
- `workspace/models/raft_stereo/` contains RAFT-Stereo checkpoints.
- `calibration/` contains camera data, calibration matrices, indexed SAM pairs,
  point cloud outputs, and experiments.
- `external/` contains third-party cloned repositories.
- `external/RAFT-Stereo/` must remain the RAFT-Stereo checkout.

## Hard Rules

- Inspect files before editing.
- Avoid giant rewrites; make focused changes that preserve current behavior.
- Do not delete calibration data, SAM indexed pairs, point cloud outputs,
  RAFT-Stereo, YOLO weights, or CAD files.
- Do not move `calibration/` into `workspace/`.
- Do not move `external/` into `workspace/`.
- Do not move `calibration/matrices/latest/stereo_calibration.npz`.
- Keep third-party RAFT code in `external/RAFT-Stereo/`.
- Keep project checkpoints in `workspace/models/`.
- Do not edit `external/` unless intentionally patching third-party code.
- Prefer top-of-file `USER SETTINGS` sections in runnable scripts.
- Run `python -m py_compile` after Python edits.
- Explain changed files clearly.

## Stereo Mask Rule

For a left pixel `(xL, y)` with disparity `d`, the corresponding right pixel is:

```text
xR = round(xL - d)
```

Do not compare `left_mask[y, x]` and `right_mask[y, x]` at the same coordinate.
Correct validation uses:

```text
left_mask[y, xL] > 0 and right_mask[y, xR] > 0
```

## Canonical Paths

- Main config: `workspace/config.py`
- Active calibration: `calibration/matrices/latest/stereo_calibration.npz`
- Indexed SAM pairs: `calibration/sam_indexed_pairs/pair_XXXXXX/`
- Current cloud outputs: `calibration/masked_stereo_cloud_outputs/`
- RAFT wrapper: `workspace/stereo_raft.py`
- RAFT repo: `external/RAFT-Stereo/`
- RAFT checkpoint: `workspace/models/raft_stereo/raftstereo-middlebury.pth`
- SAM checkpoint: `workspace/models/sam/sam_vit_b_01ec64.pth`
