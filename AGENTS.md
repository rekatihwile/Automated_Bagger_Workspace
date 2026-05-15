# AGENTS.md - Automated Grocery Bagger Workspace

## Project Context

This project is an automated grocery bagger using an HBVCam USB stereo camera system and a SCARA-like RRPR manipulator. The current computer vision goal is to reconstruct grocery-item geometry in a shared workspace frame for object centroid, height, and rough grasp/planning information.

The current computer vision pipeline is:

```text
stereo capture
-> rectification
-> ArUco/ChArUco workspace marker detection
-> workspace frame creation
-> RAFT/SGBM disparity
-> SAM object masks
-> per-object point clouds
-> corner-based multi-view alignment
-> optional limited ICP / colored ICP
-> merged scene output
-> object height metrics / workspace grid / presentation mesh outputs
```
