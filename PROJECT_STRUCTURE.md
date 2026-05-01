# Automated Grocery Bagger Project Structure

This repository combines source code, calibration data, model checkpoints, and
third-party research code for the HBVCam stereo grocery-bagging pipeline.

## Canonical Top-Level Folders

- `workspace/` contains project source code.
- `workspace/scripts/live/` contains live camera, UI, and capture experiments.
- `workspace/scripts/offline/` contains post-processing and reconstruction scripts.
- `workspace/scripts/utils/` is reserved for shared script helpers.
- `workspace/models/` contains local model checkpoints and weights.
- `workspace/models/sam/` contains SAM checkpoints.
- `workspace/models/raft_stereo/` contains RAFT-Stereo checkpoints used by project wrappers.
- `calibration/` contains generated camera data, calibration matrices, indexed SAM pairs,
  rectification checks, point cloud outputs, and experiments.
- `external/` contains third-party cloned repositories such as RAFT-Stereo.

## Important Paths

- Active stereo calibration: `calibration/matrices/latest/stereo_calibration.npz`
- Indexed SAM stereo pairs: `calibration/sam_indexed_pairs/pair_XXXXXX/`
- Masked stereo cloud outputs: `calibration/masked_stereo_cloud_outputs/`
- RAFT-Stereo repository: `external/RAFT-Stereo/`
- RAFT-Stereo checkpoint: `workspace/models/raft_stereo/raftstereo-middlebury.pth`
- SAM checkpoint: `workspace/models/sam/sam_vit_b_01ec64.pth`

## Working Rules

Do not move `calibration/` into `workspace/`.
Do not move `external/` into `workspace/`.
Do not move `calibration/matrices/latest/stereo_calibration.npz`; it is used by
`workspace/config.py`.

Avoid editing files in `external/` unless intentionally patching a third-party
dependency. Project-specific wrappers should live in `workspace/`.
