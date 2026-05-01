# Grocery Bagger Stereo Workspace

This project uses one USB stereo camera that streams a side-by-side stitched frame.
The left and right images are split from that one frame and used for calibration,
segmentation, triangulation, and post-processing.

## Workspace Organization

```text
Grocery Bagger/
  calibration/                       # outputs: pairs, matrices, recordings
  YOLO_Bandaid/                      # captured image pairs for AI experiments
  camera_test_properties_*.json      # camera inspector snapshots
  workspace/
    config.py                        # main project settings (edit this first)
    ai_topface.py                    # YOLO top-face segmenter wrapper
    record_raw_stitched_video.py     # raw stitched recorder (dynamic fps)
    postprocess_yolo_mesh_3d.py      # offline YOLO -> mesh -> 3D plot
    camera/                          # camera pipeline package
      camera.py
      __init__.py
    models/                          # YOLO weights
      Yolo11_Top.pt
      best.pt
    scripts/
      camera/                        # camera setup + calibration tools
        camera_config_inspector.py
        camera_scan.py
        capture_pairs.py
        calibrate.py
      live/                          # live experiments / debug viewers
        ai_mask_test.py
        test_calibration_live.py
        test_plane_live.py
        Chat_Wep_Test_Plane_Live.py
    manipulator/
    _archive_old_python_20260420/
    .vscode/
```

## YOLO Weights (Important)

Put your `.pt` files in:

- `workspace/models/`

Default behavior:

- Live scripts (`workspace/scripts/live/*.py`) and `ai_topface.py` auto-load:
`workspace/models/Yolo11_Top.pt`
- If that file is missing, `ai_topface.py` falls back to:
`workspace/Yolo11_Top.pt`

For postprocess:

- Edit `workspace/postprocess_yolo_mesh_3d.py`:
`YOLO_MODEL_PATH = r""`
- Set it to your exact weight file path if you want a different model, for example:
`YOLO_MODEL_PATH = r"C:\Users\elipp\OneDrive\Documents\Grocery Bagger\workspace\models\best.pt"`
- Default mesh outputs are saved to:
`calibration/mesh3d_outputs/`

## Quick Start

1. Install dependencies

```powershell
pip install -r requirements.txt
```

2. Scan camera index/modes

```powershell
python workspace/scripts/camera/camera_scan.py
```

3. Capture checkerboard pairs

```powershell
python workspace/scripts/camera/capture_pairs.py
```

4. Calibrate stereo

```powershell
python workspace/scripts/camera/calibrate.py
```

5. Record raw stitched video

```powershell
python workspace/record_raw_stitched_video.py
```

6. Offline YOLO post-process to 3D mesh

```powershell
python workspace/postprocess_yolo_mesh_3d.py
```

The post-process script prompts for the video path if you do not pass one.
Default preview controls:
- `[` / `]`: lower/raise confidence for the current frame
- `0` or `r`: reset current-frame confidence override
- `Space`: toggle play/pause
- `n` or `.`: advance one frame when paused
- `q` / `Esc`: quit

## Settings Guide (workspace/config.py)

All important settings are in `workspace/config.py`.

### 1) Camera stream and transport

- `CAMERA_INDEX`: default camera index (overridden by `camera_ids.json` if present)
- `CAMERA_BACKEND`: OpenCV backend (`"DSHOW"` recommended on Windows)
- `CAMERA_FOURCC`: requested codec (currently `"MJPG"`)
- `CAMERA_FPS`: requested camera FPS
- `STREAM_WIDTH`, `STREAM_HEIGHT`: requested stitched frame size
- `STEREO_STREAM_MODES`: candidate modes for `camera_scan.py`

### 2) Camera image controls

- `CAMERA_SETTINGS`: brightness/contrast/saturation/hue/gain/exposure/sharpness/gamma
- `AUTO_EXPOSURE_MODE`: convenience override (`None`, `"auto"`, `"manual"`)
- `EXPOSURE_VALUE`: optional override when `AUTO_EXPOSURE_MODE == "manual"`

Current defaults were applied from:
`camera_test_properties_20260423_121205_4961.json`.

### 3) Preprocessing

- `PREPROCESS["apply_to"]`: `"none"`, `"preview"`, `"saved"`, or `"both"`
- Gamma/CLAHE/blur/sharpen live in the same `PREPROCESS` block

### 4) AI and recording

- `AI_INPUT_SIZE`: YOLO inference image size
- `AI_BOUNDARY_SAMPLE_COUNT`: sampled mask boundary points
- `AI_TOPFACE_RECORD_*`: live AI recording options
- `RECORD_TEST_VIDEO`, `TEST_RECORDINGS_DIR`: live test recording controls

### 5) Calibration and path outputs

- `CALIBRATION_ROOT`: base output dir for calibration assets
- `STEREO_PAIR_DIR`: checkerboard pair directory
- `ACTIVE_CALIBRATION_NPZ`: triangulation matrices used by runtime scripts
- `CAPTURE_SESSION_FOLDER`: where `capture_pairs.py` writes left/right frames

## Camera Inspector Workflow

Use inspector to probe what your camera actually accepts:

```powershell
python workspace/scripts/camera/camera_config_inspector.py
```

Press `s` in the inspector window to save a `camera_test_properties_*.json` file,
then copy those values into `CAMERA_SETTINGS` in `workspace/config.py`.

## FPS Behavior

- `record_raw_stitched_video.py`: default output FPS is auto-derived from real capture timing.
- `postprocess_yolo_mesh_3d.py`: default output FPS is auto-derived from source timing.
- Both scripts support `--fps` if you want a manual override.
