# Scripts

Project scripts are grouped by how they are used.

- `live/` contains scripts that talk to the camera, show UI windows, or capture
  new data.
- `offline/` contains scripts that process saved images, masks, calibration, or
  point cloud outputs.
- `camera/` contains camera setup, scanning, capture, and calibration utilities.
- `utils/` is reserved for shared helper modules used by scripts.
- `sam_backup/` contains older SAM experiments kept for reference.

Prefer a top-of-file `USER SETTINGS` section for scripts that are commonly run
from VS Code. Keep CLI arguments available when they are useful for batch tests.
