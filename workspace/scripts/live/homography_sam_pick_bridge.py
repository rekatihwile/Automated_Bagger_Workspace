"""
Single-webcam homography + SAM bridge for RRPR pick-and-place.

Workflow:
  1. Click the 4 workspace corners in the raw camera window, using the same
     order and workspace coordinates as homography_test.py.
  2. After the homography is computed, use multiple prompts on one object in
     the transformed workspace window.
  3. Press p to preview/update that object's SAM mask.
  4. Press s to save the current object mask and append one
     {"x", "y", "z", "phi"} entry to JSON_OUTPUT_PATH.

This script intentionally uses a plain cv2.VideoCapture webcam path. It does
not use the StereoCamera class.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, cast

import cv2
import numpy as np
import torch


# ---------------------------------------------------------------------------
# User-tuned values
# ---------------------------------------------------------------------------

CAMERA_INDEX = 1
# 0 means unlimited saves. Set to a positive integer if you want the script to
# stop after a fixed number of objects.
NUM_OBJECTS = 0
DEFAULT_Z = 0

DISPLAY_SCALE = 3.0

# Same click-to-workspace pairing as homography_test.py.
WORKSPACE_CORNERS = [
    (400.0, 400.0),
    (400.0, 100.0),
    (100.0, 100.0),
    (100.0, 400.0),
]


# ---------------------------------------------------------------------------
# Paths and SAM setup
# ---------------------------------------------------------------------------

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = WORKSPACE_ROOT.parent
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

DEFAULT_CHECKPOINT = WORKSPACE_ROOT / "models" / "sam" / "sam_vit_b_01ec64.pth"
LEGACY_CHECKPOINT = WORKSPACE_ROOT / "models" / "sam_vit_b_01ec64.pth"
DEFAULT_OUT_DIR = PROJECT_ROOT / "calibration" / "homography_sam_pick_bridge"
JSON_OUTPUT_PATH = WORKSPACE_ROOT / "scripts/live" / "cv_target.json"

try:
    from segment_anything import SamPredictor, sam_model_registry
except ImportError as exc:
    raise ImportError(
        "\nCould not import segment_anything.\n"
        "Install it with:\n"
        "  pip install git+https://github.com/facebookresearch/segment-anything.git\n"
    ) from exc


WINDOW_CAMERA = "Raw Webcam - click 4 workspace corners"
WINDOW_WORKSPACE = "Homography Workspace - click objects"


@dataclass
class ObjectPrompt:
    pos_pts: list[tuple[int, int]] = field(default_factory=list)
    neg_pts: list[tuple[int, int]] = field(default_factory=list)
    history: list[tuple[str, tuple[int, int]]] = field(default_factory=list)
    masks: np.ndarray | None = None
    scores: np.ndarray | None = None
    mask_idx: int = 0
    metric: dict[str, Any] | None = None
    predicted_warped_bgr: np.ndarray | None = None

    @property
    def current_mask(self) -> np.ndarray | None:
        if self.masks is None:
            return None
        return self.masks[self.mask_idx]

    def add_point(self, kind: str, pt: tuple[int, int]) -> None:
        if kind == "pos":
            self.pos_pts.append(pt)
        elif kind == "neg":
            self.neg_pts.append(pt)
        else:
            raise ValueError(f"Unknown prompt kind: {kind}")
        self.history.append((kind, pt))
        self.clear_prediction()

    def undo_last_point(self) -> None:
        if not self.history:
            print("[WARN] No current-object prompt points to undo.")
            return
        kind, pt = self.history.pop()
        points = self.pos_pts if kind == "pos" else self.neg_pts
        if points:
            points.pop()
        self.clear_prediction()
        print(f"[UNDO] Removed {kind} point {pt}")

    def clear_prediction(self) -> None:
        self.masks = None
        self.scores = None
        self.mask_idx = 0
        self.metric = None
        self.predicted_warped_bgr = None

    def clear(self) -> None:
        self.pos_pts.clear()
        self.neg_pts.clear()
        self.history.clear()
        self.clear_prediction()


@dataclass
class SavedObject:
    mask: np.ndarray
    metric: dict[str, Any]


@dataclass
class HomographyState:
    corner_clicks: list[tuple[int, int]]
    current: ObjectPrompt = field(default_factory=ObjectPrompt)
    h_display: np.ndarray | None = None
    crop_min_x: float = 0.0
    crop_min_y: float = 0.0
    workspace_width: int = 300
    workspace_height: int = 300
    saved_objects: list[SavedObject] = field(default_factory=list)

    def reset_homography(self) -> None:
        self.corner_clicks.clear()
        self.current.clear()
        self.h_display = None
        self.saved_objects.clear()

    def clear_objects(self) -> None:
        self.current.clear()


def open_camera(index: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open webcam at index {index}.")
    return cap


def draw_hud(image: np.ndarray, lines: list[str]) -> np.ndarray:
    out = image.copy()
    y = 26
    for line in lines:
        cv2.putText(out, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(out, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)
        y += 24
    return out


def camera_mouse(event: int, x: int, y: int, flags: int, param: Any) -> None:
    state: HomographyState = param["state"]
    if state.h_display is not None:
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        if len(state.corner_clicks) < 4:
            state.corner_clicks.append((x, y))
            print(f"[CORNER] {len(state.corner_clicks)}/4 raw pixel ({x}, {y})")
        else:
            print("[INFO] Already have 4 corners. Press r to redo homography.")
    elif event == cv2.EVENT_RBUTTONDOWN and state.corner_clicks:
        removed = state.corner_clicks.pop()
        print(f"[CORNER] Removed {removed}")


def workspace_mouse(event: int, x: int, y: int, flags: int, param: Any) -> None:
    state: HomographyState = param["state"]
    display_scale: float = param["display_scale"]
    if state.h_display is None:
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        state.current.add_point("pos", (x, y))
        wx = x / display_scale + state.crop_min_x
        wy = y / display_scale + state.crop_min_y
        print(
            f"[PROMPT] + point workspace ({wx:.3f}, {wy:.3f}) "
            f"from display pixel ({x}, {y})"
        )
    elif event == cv2.EVENT_RBUTTONDOWN:
        state.current.add_point("neg", (x, y))
        print(f"[PROMPT] - point display pixel ({x}, {y})")


def compute_homography(state: HomographyState, display_scale: float) -> bool:
    if len(state.corner_clicks) != 4:
        return False

    pixel_points = np.array(state.corner_clicks, dtype=np.float32)
    workspace_points = np.array(WORKSPACE_CORNERS, dtype=np.float32)

    h_robot, status = cv2.findHomography(pixel_points, workspace_points)
    if h_robot is None:
        print("[ERROR] cv2.findHomography failed. Press r and click corners again.")
        return False

    state.crop_min_x = float(np.min(workspace_points[:, 0]))
    crop_max_x = float(np.max(workspace_points[:, 0]))
    state.crop_min_y = float(np.min(workspace_points[:, 1]))
    crop_max_y = float(np.max(workspace_points[:, 1]))

    state.workspace_width = int(round((crop_max_x - state.crop_min_x) * display_scale)) + 1
    state.workspace_height = int(round((crop_max_y - state.crop_min_y) * display_scale)) + 1

    scale_crop = np.array(
        [
            [display_scale, 0.0, -state.crop_min_x * display_scale],
            [0.0, display_scale, -state.crop_min_y * display_scale],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    state.h_display = scale_crop @ h_robot

    cv2.namedWindow(WINDOW_WORKSPACE, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_WORKSPACE, state.workspace_width, state.workspace_height)

    print("[INFO] Homography ready.")
    print("[INFO] Display homography matrix:")
    print(state.h_display)
    return True


def largest_component(mask: np.ndarray) -> np.ndarray:
    mask_u8 = (mask > 0).astype(np.uint8)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask_u8, connectivity=8)
    if num_labels <= 1:
        return mask_u8

    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return (labels == largest).astype(np.uint8)


def compute_mask_metrics(
    mask: np.ndarray,
    state: HomographyState,
    object_index: int,
    z_value: float,
    display_scale: float,
) -> dict[str, Any] | None:
    component = largest_component(mask)
    ys, xs = np.nonzero(component)
    if len(xs) < 10:
        print(f"[WARN] Object {object_index}: mask too small for centroid/axis metrics.")
        return None

    moments = cv2.moments(component, binaryImage=True)
    if abs(moments["m00"]) < 1e-9:
        print(f"[WARN] Object {object_index}: zero-area mask moments.")
        return None

    cx = float(moments["m10"] / moments["m00"])
    cy = float(moments["m01"] / moments["m00"])

    coords = np.column_stack([xs.astype(np.float64), ys.astype(np.float64)])
    centered = coords - np.array([cx, cy], dtype=np.float64)
    cov = np.cov(centered, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    order = np.argsort(eigenvalues)[::-1]
    eigenvectors = eigenvectors[:, order]

    major_vec = eigenvectors[:, 0]
    minor_vec = eigenvectors[:, 1]
    projections = centered @ eigenvectors
    major_px = float(np.ptp(projections[:, 0]))
    minor_px = float(np.ptp(projections[:, 1]))

    phi = float(np.degrees(np.arctan2(major_vec[1], major_vec[0])) % 180.0)
    minor_phi = float((phi + 90.0) % 180.0)

    workspace_x = cx / display_scale + state.crop_min_x
    workspace_y = cy / display_scale + state.crop_min_y

    return {
        "object_index": object_index,
        "json_entry": {
            "x": round(float(workspace_x), 3),
            "y": round(float(workspace_y), 3),
            "z": round(float(z_value), 3),
            "phi": round(phi, 3),
        },
        "centroid_display_px": [round(cx, 3), round(cy, 3)],
        "major_axis_length": round(major_px / display_scale, 3),
        "minor_axis_length": round(minor_px / display_scale, 3),
        "major_axis_angle_deg": round(phi, 3),
        "minor_axis_angle_deg": round(minor_phi, 3),
        "mask_area_px": int(len(xs)),
    }


def draw_results_overlay(
    warped_bgr: np.ndarray,
    state: HomographyState,
    masks: list[np.ndarray] | None = None,
    metrics: list[dict[str, Any]] | None = None,
    display_scale: float = DISPLAY_SCALE,
) -> np.ndarray:
    out = warped_bgr.copy()

    if masks:
        colors = [(0, 255, 0), (255, 0, 255), (255, 180, 0), (0, 255, 255)]
        for i, mask in enumerate(masks):
            component = largest_component(mask)
            color = colors[i % len(colors)]
            layer = np.zeros_like(out)
            layer[component > 0] = color
            out = cv2.addWeighted(out, 0.76, layer, 0.24, 0)
            contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(out, contours, -1, color, 2)

    current_mask = state.current.current_mask
    if current_mask is not None:
        component = largest_component(current_mask)
        layer = np.zeros_like(out)
        layer[component > 0] = (0, 255, 0)
        out = cv2.addWeighted(out, 0.68, layer, 0.32, 0)
        contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(out, contours, -1, (0, 255, 0), 2)

    for i, (x, y) in enumerate(state.current.pos_pts, start=1):
        cv2.circle(out, (x, y), 5, (0, 255, 0), -1)
        cv2.circle(out, (x, y), 9, (0, 0, 0), 1)
        cv2.putText(out, f"+{i}", (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, f"+{i}", (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)

    for i, (x, y) in enumerate(state.current.neg_pts, start=1):
        cv2.circle(out, (x, y), 5, (0, 0, 255), -1)
        cv2.circle(out, (x, y), 9, (0, 0, 0), 1)
        cv2.putText(out, f"-{i}", (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, f"-{i}", (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)

    draw_metrics = list(metrics or [])
    if state.current.metric is not None:
        draw_metrics.append(state.current.metric)

    if draw_metrics:
        for metric in draw_metrics:
            cx, cy = metric["centroid_display_px"]
            phi_rad = np.radians(metric["major_axis_angle_deg"])
            major_half = metric["major_axis_length"] * display_scale * 0.5
            minor_half = metric["minor_axis_length"] * display_scale * 0.5

            major_vec = np.array([np.cos(phi_rad), np.sin(phi_rad)])
            minor_vec = np.array([-np.sin(phi_rad), np.cos(phi_rad)])
            center = np.array([cx, cy])

            p1 = tuple(np.round(center - major_vec * major_half).astype(int))
            p2 = tuple(np.round(center + major_vec * major_half).astype(int))
            q1 = tuple(np.round(center - minor_vec * minor_half).astype(int))
            q2 = tuple(np.round(center + minor_vec * minor_half).astype(int))
            c = tuple(np.round(center).astype(int))

            cv2.circle(out, c, 5, (0, 0, 255), -1)
            cv2.line(out, p1, p2, (0, 0, 255), 2)
            cv2.line(out, q1, q2, (255, 255, 255), 1)

    return out


def save_mask_outputs(
    out_dir: Path,
    warped_bgr: np.ndarray,
    masks: list[np.ndarray],
    overlay_bgr: np.ndarray,
    metrics: list[dict[str, Any]],
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

    cv2.imwrite(str(out_dir / f"{stamp}_workspace.png"), warped_bgr)
    cv2.imwrite(str(out_dir / f"{stamp}_overlay.png"), overlay_bgr)

    combined = np.zeros(warped_bgr.shape[:2], dtype=np.uint8)
    for i, mask in enumerate(masks, start=1):
        component = largest_component(mask)
        combined[component > 0] = 255
        cv2.imwrite(str(out_dir / f"{stamp}_object_{i:02d}_mask.png"), component * 255)
    cv2.imwrite(str(out_dir / f"{stamp}_combined_mask.png"), combined)

    debug_path = out_dir / f"{stamp}_metrics_debug.json"
    debug_path.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    print(f"[SAVE] Mask outputs: {out_dir}")


def append_json_entries(json_path: Path, entries: list[dict[str, float]]) -> None:
    json_path.parent.mkdir(parents=True, exist_ok=True)
    if json_path.exists():
        with json_path.open("r", encoding="utf-8") as f:
            existing = json.load(f)
        if isinstance(existing, list):
            data = existing
        elif isinstance(existing, dict):
            data = [existing]
        else:
            raise ValueError(f"Expected JSON list or object in {json_path}, got {type(existing).__name__}.")
    else:
        data = []

    data.extend(entries)
    tmp_path = json_path.with_suffix(json_path.suffix + ".tmp")
    tmp_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    tmp_path.replace(json_path)
    print(f"[JSON] Appended {len(entries)} entr{'y' if len(entries) == 1 else 'ies'} to {json_path}")


def object_limit_label(num_objects: int) -> str:
    return "unlimited" if num_objects <= 0 else str(num_objects)


def object_limit_reached(state: HomographyState, num_objects: int) -> bool:
    return num_objects > 0 and len(state.saved_objects) >= num_objects


def remove_last_json_entry_if_match(json_path: Path, expected_entry: dict[str, float]) -> bool:
    if not json_path.exists():
        print(f"[WARN] JSON file does not exist yet: {json_path}")
        return False

    with json_path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, list) or not data:
        print(f"[WARN] JSON file has no list entry to undo: {json_path}")
        return False
    if data[-1] != expected_entry:
        print("[WARN] Last JSON entry does not match the last saved object from this run; leaving JSON unchanged.")
        return False

    data.pop()
    tmp_path = json_path.with_suffix(json_path.suffix + ".tmp")
    tmp_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    tmp_path.replace(json_path)
    print(f"[JSON] Removed last entry from {json_path}")
    return True


def predict_current_object(
    warped_bgr: np.ndarray,
    state: HomographyState,
    predictor: SamPredictor,
    z_value: float,
    display_scale: float,
) -> bool:
    coords = state.current.pos_pts + state.current.neg_pts
    if not coords:
        print("[WARN] Add at least one positive or negative point before pressing p.")
        return False

    print("[INFO] Setting SAM image embedding for current warped workspace frame...")
    predictor.set_image(cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2RGB))

    point_coords = np.array(coords, dtype=np.float32)
    point_labels = np.array(
        [1] * len(state.current.pos_pts) + [0] * len(state.current.neg_pts),
        dtype=np.int32,
    )
    masks_raw, scores_raw, _ = predictor.predict(
        point_coords=point_coords,
        point_labels=point_labels,
        multimask_output=True,
    )
    masks = cast(np.ndarray, masks_raw)
    scores = cast(np.ndarray, scores_raw)
    state.current.masks = masks
    state.current.scores = scores
    state.current.mask_idx = int(np.argmax(scores))
    state.current.predicted_warped_bgr = warped_bgr.copy()

    object_index = len(state.saved_objects) + 1
    mask = state.current.current_mask
    if mask is None:
        print("[WARN] SAM did not return a usable mask.")
        return False

    metric = compute_mask_metrics(mask, state, object_index, z_value, display_scale)
    if metric is None:
        state.current.metric = None
        return False
    state.current.metric = metric

    for i, score in enumerate(scores):
        marker = " <--" if i == state.current.mask_idx else ""
        print(f"[SAM] mask {i}: score={score:.4f}{marker}")

    entry = metric["json_entry"]
    print(
        f"[PREVIEW object_{object_index:02d}] x={entry['x']} y={entry['y']} z={entry['z']} "
        f"phi={entry['phi']} deg | major={metric['major_axis_length']} "
        f"minor={metric['minor_axis_length']}"
    )
    return True


def cycle_current_mask(state: HomographyState, z_value: float, display_scale: float) -> None:
    if state.current.masks is None or state.current.scores is None:
        print("[WARN] No current mask candidates yet. Press p first.")
        return
    state.current.mask_idx = (state.current.mask_idx + 1) % len(state.current.masks)
    mask = state.current.current_mask
    if mask is None:
        return
    metric = compute_mask_metrics(mask, state, len(state.saved_objects) + 1, z_value, display_scale)
    state.current.metric = metric
    score = state.current.scores[state.current.mask_idx]
    print(f"[SAM] Selected mask {state.current.mask_idx}/{len(state.current.masks) - 1}, score={score:.4f}")


def save_current_object(
    warped_bgr: np.ndarray,
    state: HomographyState,
    predictor: SamPredictor,
    num_objects: int,
    z_value: float,
    out_dir: Path,
    json_out: Path,
    display_scale: float,
) -> None:
    if object_limit_reached(state, num_objects):
        print(
            f"[WARN] Already saved {len(state.saved_objects)}/{num_objects} objects. "
            "Increase NUM_OBJECTS, run with --num-objects N, or use --num-objects 0 for unlimited. "
            "Press d to undo last saved object."
        )
        return

    if state.current.current_mask is None or state.current.metric is None:
        if not predict_current_object(warped_bgr, state, predictor, z_value, display_scale):
            return

    mask = state.current.current_mask
    metric = state.current.metric
    if mask is None or metric is None:
        print("[WARN] No current segmentation to save.")
        return

    save_frame = state.current.predicted_warped_bgr
    if save_frame is None:
        save_frame = warped_bgr

    overlay = draw_results_overlay(save_frame, state, [], [], display_scale)
    save_mask_outputs(out_dir, save_frame, [mask], overlay, [metric])
    append_json_entries(json_out, [metric["json_entry"]])

    state.saved_objects.append(SavedObject(mask=mask.copy(), metric=metric))
    print(f"[OBJECT SAVED] object_{len(state.saved_objects):02d}/{object_limit_label(num_objects)}")
    state.current.clear()


def undo_last_saved_object(state: HomographyState, json_out: Path) -> None:
    if not state.saved_objects:
        print("[WARN] No saved objects from this run to undo.")
        return

    removed = state.saved_objects.pop()
    remove_last_json_entry_if_match(json_out, removed.metric["json_entry"])
    print(f"[UNDO] Removed saved object_{len(state.saved_objects) + 1:02d} from this session.")


def draw_camera_view(frame: np.ndarray, state: HomographyState) -> np.ndarray:
    out = frame.copy()
    for i, pt in enumerate(state.corner_clicks, start=1):
        cv2.circle(out, pt, 5, (0, 255, 255), -1)
        cv2.putText(out, str(i), (pt[0] + 8, pt[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, str(i), (pt[0] + 8, pt[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    if len(state.corner_clicks) == 4:
        cv2.polylines(out, [np.array(state.corner_clicks, dtype=np.int32)], isClosed=True, color=(0, 255, 255), thickness=2)

    if state.h_display is None:
        lines = [
            "L-click 4 corners in homography_test.py order",
            "R-click undo corner | r reset | q quit",
            f"corners: {len(state.corner_clicks)}/4",
        ]
    else:
        lines = [
            "Homography active. Use the workspace window for object prompts.",
            "p preview | s save current object | u undo point | c clear current | q quit",
        ]
    return draw_hud(out, lines)


def draw_workspace_view(
    warped: np.ndarray,
    state: HomographyState,
    num_objects: int,
    display_scale: float,
) -> np.ndarray:
    saved_masks = [obj.mask for obj in state.saved_objects]
    saved_metrics = [obj.metric for obj in state.saved_objects]
    out = draw_results_overlay(warped, state, saved_masks, saved_metrics, display_scale)
    mask_info = "no preview"
    if state.current.scores is not None:
        score = state.current.scores[state.current.mask_idx]
        mask_info = f"mask {state.current.mask_idx}/{len(state.current.scores) - 1} score={score:.3f}"
    lines = [
        f"saved {len(state.saved_objects)}/{object_limit_label(num_objects)} | current +pts={len(state.current.pos_pts)} -pts={len(state.current.neg_pts)} | {mask_info}",
        "L-click +point | R-click -point | p preview | m cycle mask | s save JSON",
        "u undo point | c clear current | d undo saved | r redo corners | q quit",
    ]
    return draw_hud(out, lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Single webcam -> homography -> SAM masks -> pick JSON bridge."
    )
    parser.add_argument("--camera-index", type=int, default=CAMERA_INDEX)
    parser.add_argument(
        "--num-objects",
        type=int,
        default=NUM_OBJECTS,
        help="Maximum objects to save this run. Use 0 for unlimited.",
    )
    parser.add_argument("--z", type=float, default=DEFAULT_Z, help="Constant z value written to each JSON entry.")
    parser.add_argument("--json-out", type=Path, default=JSON_OUTPUT_PATH)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--display-scale", type=float, default=DISPLAY_SCALE)
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--model-type", choices=["vit_b", "vit_l", "vit_h"], default="vit_b")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.num_objects < 0:
        raise ValueError("--num-objects must be 0 for unlimited, or a positive integer.")

    if not args.checkpoint.exists() and args.checkpoint == DEFAULT_CHECKPOINT and LEGACY_CHECKPOINT.exists():
        args.checkpoint = LEGACY_CHECKPOINT
    if not args.checkpoint.exists():
        raise FileNotFoundError(
            f"SAM checkpoint not found: {args.checkpoint}\n"
            "Download vit_b from:\n"
            "  https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth"
        )

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[INFO] Device:     {device}")
    print(f"[INFO] Model:      {args.model_type}")
    print(f"[INFO] Checkpoint: {args.checkpoint}")
    print(f"[INFO] JSON out:   {args.json_out}")
    print(f"[INFO] Mask dir:   {args.out_dir}")
    print()

    sam = sam_model_registry[args.model_type](checkpoint=str(args.checkpoint))
    sam.to(device=device)
    predictor = SamPredictor(sam)

    state = HomographyState(corner_clicks=[])
    cap = open_camera(args.camera_index)

    cv2.namedWindow(WINDOW_CAMERA, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW_CAMERA, camera_mouse, {"state": state})

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("[WARN] Camera read failed.")
                continue

            if state.h_display is None and len(state.corner_clicks) == 4:
                if compute_homography(state, args.display_scale):
                    cv2.setMouseCallback(
                        WINDOW_WORKSPACE,
                        workspace_mouse,
                        {
                            "state": state,
                            "display_scale": args.display_scale,
                        },
                    )

            warped = None
            if state.h_display is not None:
                warped = cv2.warpPerspective(
                    frame,
                    state.h_display,
                    (state.workspace_width, state.workspace_height),
                )
                cv2.imshow(
                    WINDOW_WORKSPACE,
                    draw_workspace_view(warped, state, args.num_objects, args.display_scale),
                )

            cv2.imshow(WINDOW_CAMERA, draw_camera_view(frame, state))
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                state.reset_homography()
                try:
                    cv2.destroyWindow(WINDOW_WORKSPACE)
                except cv2.error:
                    pass
                print("[INFO] Homography reset. Click 4 corners again.")
            elif key == ord("c"):
                if state.h_display is None:
                    state.corner_clicks.clear()
                    print("[INFO] Corner clicks cleared.")
                else:
                    state.clear_objects()
                    print("[INFO] Current object prompts and preview mask cleared.")
            elif key in (ord("u"), 8):
                if state.h_display is None:
                    if state.corner_clicks:
                        removed = state.corner_clicks.pop()
                        print(f"[UNDO] Removed corner {removed}")
                    else:
                        print("[WARN] No corner click to undo.")
                else:
                    state.current.undo_last_point()
            elif key == ord("m"):
                if state.h_display is None:
                    print("[WARN] Compute homography first by clicking 4 corners.")
                else:
                    cycle_current_mask(state, args.z, args.display_scale)
            elif key == ord("d"):
                if state.h_display is None:
                    print("[WARN] No saved object to undo before homography is active.")
                else:
                    undo_last_saved_object(state, args.json_out)
            elif key == ord("p"):
                if state.h_display is None or warped is None:
                    print("[WARN] Compute homography first by clicking 4 corners.")
                else:
                    predict_current_object(
                        warped,
                        state,
                        predictor,
                        args.z,
                        args.display_scale,
                    )
            elif key == ord("s"):
                if state.h_display is None or warped is None:
                    print("[WARN] Compute homography first by clicking 4 corners.")
                else:
                    save_current_object(
                        warped,
                        state,
                        predictor,
                        args.num_objects,
                        args.z,
                        args.out_dir,
                        args.json_out,
                        args.display_scale,
                    )
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[INFO] Done.")


if __name__ == "__main__":
    main()
