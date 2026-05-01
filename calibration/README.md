# Calibration Data

This folder stores generated data and experiment outputs for the stereo camera.
It is intentionally outside `workspace/` so source code and data stay separate.

Important subfolders:

- `matrices/latest/` contains the active stereo calibration used by `workspace/config.py`.
- `pairs/` contains captured stereo calibration pairs.
- `sam_indexed_pairs/` contains rectified stereo pairs and SAM masks.
- `sam_live_snapshot_tests/` contains live SAM capture/debug outputs.
- `masked_stereo_cloud_outputs/` contains current RAFT/SGBM point cloud outputs.
- `rectified_disparity_debug/` contains rectification/disparity diagnostics.
- `rectification_checks/` is reserved for rectification validation outputs.
- `workspace_pose_tests/` is reserved for robot/workspace pose experiments.
- `archived_experiments/` contains old one-off outputs that should be kept but
  are not part of the active pipeline.

Do not delete calibration data unless it has been backed up intentionally.
Do not move `matrices/latest/stereo_calibration.npz`.
