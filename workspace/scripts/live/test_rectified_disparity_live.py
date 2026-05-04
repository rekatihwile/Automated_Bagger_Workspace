"""
Live SAM camera snapshot tool with rectified stereo preview and LEFT/RIGHT masks.

Workflow:
  1. A live stereo camera feed opens.
  2. Live view shows BOTH left and right frames stitched side-by-side.
     If --rectify is passed, both are rectified before display.
  3. Press SPACE to freeze the current stereo pair.
  4. In snapshot mode, you can switch between LEFT and RIGHT images.
  5. SAM can be run independently on the LEFT and RIGHT rectified images.
  6. Saving writes:
       left_rect.png
       right_rect.png
       stereo_rect_preview.png
       left_mask.png          if left mask exists
       right_mask.png         if right mask exists
       left_overlay.png
       right_overlay.png
       left_masked.png        if left mask exists
       right_masked.png       if right mask exists
       stereo_mask_overlay_preview.png

Controls in snapshot mode:
  v                 = switch active image LEFT/RIGHT
  Left click        = positive SAM point prompt on active image
  Right click       = negative SAM point prompt on active image
  b                 = draw box prompt on active image
  p                 = run SAM on active image
  n                 = cycle mask candidates on active image
  t                 = enter text label / note
  s                 = save both images and any existing masks
  c                 = clear prompts/mask on active image
  C                 = clear prompts/masks on BOTH images
  SPACE             = discard snapshot, return to live view
  q / Esc           = quit

Note:
  Regular SAM does not accept text prompts.
  The label entered with 't' is only used as a filename suffix when saving outputs.

Usage:
  python workspace/scripts/live/test_sam_live_snapshot.py --rectify
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import cast

import cv2
import numpy as np
import torch


# ---------------------------------------------------------------------------
# Workspace path setup
# ---------------------------------------------------------------------------

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import config
from camera import StereoCamera

try:
    from segment_anything import sam_model_registry, SamPredictor
except ImportError as exc:
    raise ImportError(
        "\nCould not import segment_anything.\n"
        "Install it with:\n"
        "  pip install git+https://github.com/facebookresearch/segment-anything.git\n"
    ) from exc


DEFAULT_CHECKPOINT = WORKSPACE_ROOT / "models" / "sam" / "sam_vit_b_01ec64.pth"
LEGACY_CHECKPOINT = WORKSPACE_ROOT / "models" / "sam_vit_b_01ec64.pth"
DEFAULT_OUT_DIR = config.CALIBRATION_ROOT / "sam_live_snapshot_tests"

WINDOW_LIVE = "SAM Live Stereo Preview — SPACE=snapshot  q=quit"
WINDOW_SNAP = "SAM Snapshot — v=switch LEFT/RIGHT  p=predict  s=save"


# ---------------------------------------------------------------------------
# Rectification
# ---------------------------------------------------------------------------

def _build_rect_maps(h: int, w: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Build rectification maps for BOTH left and right images.

    h, w are the dimensions of each individual eye image, not the full stitched stream.
    """
    calib = config.load_stereo_calibration()

    if calib is None:
        raise FileNotFoundError(
            f"No calibration file found at:\n  {config.ACTIVE_CALIBRATION_NPZ}"
        )

    required = [
        "left_camera_matrix",
        "left_distortion_coefficients",
        "right_camera_matrix",
        "right_distortion_coefficients",
        "rectification_left",
        "rectification_right",
        "projection_left_rectified",
        "projection_right_rectified",
    ]

    missing = [k for k in required if k not in calib]
    if missing:
        raise KeyError(
            "Calibration missing keys:\n" + "\n".join(f"  {k}" for k in missing)
        )

    map_lx, map_ly = cv2.initUndistortRectifyMap(
        calib["left_camera_matrix"],
        calib["left_distortion_coefficients"],
        calib["rectification_left"],
        calib["projection_left_rectified"],
        (w, h),
        cv2.CV_32FC1,
    )

    map_rx, map_ry = cv2.initUndistortRectifyMap(
        calib["right_camera_matrix"],
        calib["right_distortion_coefficients"],
        calib["rectification_right"],
        calib["projection_right_rectified"],
        (w, h),
        cv2.CV_32FC1,
    )

    return map_lx, map_ly, map_rx, map_ry


def rectify_pair(
    left_bgr: np.ndarray,
    right_bgr: np.ndarray,
    maps: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    map_lx, map_ly, map_rx, map_ry = maps

    left_rect = cv2.remap(left_bgr, map_lx, map_ly, cv2.INTER_LINEAR)
    right_rect = cv2.remap(right_bgr, map_rx, map_ry, cv2.INTER_LINEAR)

    return left_rect, right_rect


# ---------------------------------------------------------------------------
# Visualization helpers
# ---------------------------------------------------------------------------

def _draw_horizontal_guides(
    img: np.ndarray,
    spacing: int = 40,
    color: tuple[int, int, int] = (0, 255, 255),
) -> np.ndarray:
    out = img.copy()
    h, w = out.shape[:2]

    for y in range(spacing, h, spacing):
        cv2.line(out, (0, y), (w - 1, y), color, 1, cv2.LINE_AA)

    return out


def _make_stereo_preview(
    left_bgr: np.ndarray,
    right_bgr: np.ndarray,
    label: str = "",
    draw_guides: bool = True,
) -> np.ndarray:
    """
    Make stitched left/right stereo preview.

    This is only for preview/saving.
    SAM operates on each eye image independently.
    """
    left_view = left_bgr.copy()
    right_view = right_bgr.copy()

    if draw_guides:
        left_view = _draw_horizontal_guides(left_view)
        right_view = _draw_horizontal_guides(right_view)

    cv2.putText(
        left_view,
        "LEFT",
        (15, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    cv2.putText(
        right_view,
        "RIGHT",
        (15, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    divider = np.full((left_view.shape[0], 8, 3), 35, dtype=np.uint8)
    stitched = cv2.hconcat([left_view, divider, right_view])

    if label:
        cv2.putText(
            stitched,
            label,
            (15, stitched.shape[0] - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            stitched,
            label,
            (15, stitched.shape[0] - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    return stitched


def _mask_overlay(image_bgr: np.ndarray, mask: np.ndarray | None) -> np.ndarray:
    out = image_bgr.copy()

    if mask is None:
        return out

    mask_bool = mask > 0
    color_layer = np.zeros_like(out)
    color_layer[mask_bool] = (0, 255, 0)
    out = cv2.addWeighted(out, 0.72, color_layer, 0.28, 0)

    contours, _ = cv2.findContours(
        mask_bool.astype(np.uint8),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    cv2.drawContours(out, contours, -1, (0, 255, 0), 2)

    return out


def _make_masked_image(image_bgr: np.ndarray, mask: np.ndarray) -> np.ndarray:
    out = np.zeros_like(image_bgr)
    out[mask > 0] = image_bgr[mask > 0]
    return out


def _make_overlay(
    image_bgr: np.ndarray,
    mask: np.ndarray | None,
    pos_pts: list[tuple[int, int]],
    neg_pts: list[tuple[int, int]],
    box: tuple[int, int, int, int] | None,
    hud_lines: list[str],
    active_side: str,
) -> np.ndarray:
    out = _mask_overlay(image_bgr, mask)

    for x, y in pos_pts:
        cv2.circle(out, (x, y), 5, (0, 255, 0), -1)
        cv2.circle(out, (x, y), 8, (0, 0, 0), 1)

    for x, y in neg_pts:
        cv2.circle(out, (x, y), 5, (0, 0, 255), -1)
        cv2.circle(out, (x, y), 8, (0, 0, 0), 1)

    if box is not None:
        x1, y1, x2, y2 = box
        cv2.rectangle(out, (x1, y1), (x2, y2), (255, 180, 0), 2)

    # Active side label at top-left.
    cv2.putText(
        out,
        f"ACTIVE: {active_side.upper()}",
        (12, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, 0),
        4,
        cv2.LINE_AA,
    )
    cv2.putText(
        out,
        f"ACTIVE: {active_side.upper()}",
        (12, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    # Compact HUD at bottom.
    h, _ = out.shape[:2]
    y_start = max(58, h - 24 * len(hud_lines) - 12)

    y_pos = y_start
    for line in hud_lines:
        cv2.putText(
            out,
            line,
            (12, y_pos),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            out,
            line,
            (12, y_pos),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        y_pos += 24

    return out


def _safe_label(label: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in label.strip())


def _save_outputs(
    out_dir: Path,
    left_bgr: np.ndarray,
    right_bgr: np.ndarray,
    left_mask: np.ndarray | None,
    right_mask: np.ndarray | None,
    label: str,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe = _safe_label(label)
    suffix = f"_{safe}" if safe else ""
    stem = f"{stamp}{suffix}"

    left_path = out_dir / f"{stem}_left_rect.png"
    right_path = out_dir / f"{stem}_right_rect.png"
    stereo_path = out_dir / f"{stem}_stereo_rect_preview.png"

    left_overlay_path = out_dir / f"{stem}_left_overlay.png"
    right_overlay_path = out_dir / f"{stem}_right_overlay.png"
    stereo_overlay_path = out_dir / f"{stem}_stereo_mask_overlay_preview.png"

    cv2.imwrite(str(left_path), left_bgr)
    cv2.imwrite(str(right_path), right_bgr)

    stereo_preview = _make_stereo_preview(
        left_bgr,
        right_bgr,
        label="Rectified stereo pair",
        draw_guides=True,
    )
    cv2.imwrite(str(stereo_path), stereo_preview)

    left_overlay = _mask_overlay(left_bgr, left_mask)
    right_overlay = _mask_overlay(right_bgr, right_mask)

    cv2.imwrite(str(left_overlay_path), left_overlay)
    cv2.imwrite(str(right_overlay_path), right_overlay)

    stereo_mask_overlay = _make_stereo_preview(
        left_overlay,
        right_overlay,
        label="Stereo pair with independent SAM masks",
        draw_guides=True,
    )
    cv2.imwrite(str(stereo_overlay_path), stereo_mask_overlay)

    print(f"[SAVE] Left rect:        {left_path}")
    print(f"[SAVE] Right rect:       {right_path}")
    print(f"[SAVE] Stereo preview:   {stereo_path}")
    print(f"[SAVE] Left overlay:     {left_overlay_path}")
    print(f"[SAVE] Right overlay:    {right_overlay_path}")
    print(f"[SAVE] Stereo mask view: {stereo_overlay_path}")

    if left_mask is not None:
        left_mask_path = out_dir / f"{stem}_left_mask.png"
        left_masked_path = out_dir / f"{stem}_left_masked.png"

        cv2.imwrite(str(left_mask_path), (left_mask > 0).astype(np.uint8) * 255)
        cv2.imwrite(str(left_masked_path), _make_masked_image(left_bgr, left_mask))

        print(f"[SAVE] Left mask:        {left_mask_path}")
        print(f"[SAVE] Left masked:      {left_masked_path}")

    if right_mask is not None:
        right_mask_path = out_dir / f"{stem}_right_mask.png"
        right_masked_path = out_dir / f"{stem}_right_masked.png"

        cv2.imwrite(str(right_mask_path), (right_mask > 0).astype(np.uint8) * 255)
        cv2.imwrite(str(right_masked_path), _make_masked_image(right_bgr, right_mask))

        print(f"[SAVE] Right mask:       {right_mask_path}")
        print(f"[SAVE] Right masked:     {right_masked_path}")


def _open_window(name: str, img_w: int, img_h: int, scale: float) -> None:
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(
        name,
        max(320, int(img_w * scale)),
        max(240, int(img_h * scale)),
    )


# ---------------------------------------------------------------------------
# Snapshot session
# ---------------------------------------------------------------------------

@dataclass
class MaskState:
    pos_pts: list[tuple[int, int]]
    neg_pts: list[tuple[int, int]]
    box: tuple[int, int, int, int] | None
    masks: np.ndarray | None
    scores: np.ndarray | None
    mask_idx: int

    @classmethod
    def empty(cls) -> "MaskState":
        return cls(
            pos_pts=[],
            neg_pts=[],
            box=None,
            masks=None,
            scores=None,
            mask_idx=0,
        )

    @property
    def current_mask(self) -> np.ndarray | None:
        if self.masks is None:
            return None
        return self.masks[self.mask_idx]

    def clear(self) -> None:
        self.pos_pts.clear()
        self.neg_pts.clear()
        self.box = None
        self.masks = None
        self.scores = None
        self.mask_idx = 0


class SnapshotSession:
    """
    Handles all interaction on a single frozen stereo snapshot.

    LEFT and RIGHT each get independent prompt/mask state.
    This is the correct behavior because left/right object pixels are shifted by disparity.
    """

    def __init__(
        self,
        left_bgr: np.ndarray,
        right_bgr: np.ndarray,
        sam_model,
        out_dir: Path,
        display_scale: float = 2.0,
    ) -> None:
        self.left_bgr = left_bgr
        self.right_bgr = right_bgr
        self.out_dir = out_dir
        self.display_scale = display_scale

        self.left_predictor = SamPredictor(sam_model)
        self.right_predictor = SamPredictor(sam_model)

        self.left_state = MaskState.empty()
        self.right_state = MaskState.empty()

        self.active_side = "left"
        self.label: str = ""

        print("[INFO] Setting SAM image embedding for LEFT frame...")
        self.left_predictor.set_image(cv2.cvtColor(left_bgr, cv2.COLOR_BGR2RGB))

        print("[INFO] Setting SAM image embedding for RIGHT frame...")
        self.right_predictor.set_image(cv2.cvtColor(right_bgr, cv2.COLOR_BGR2RGB))

        print("[INFO] SAM ready for both LEFT and RIGHT frames.")

    def _state(self) -> MaskState:
        return self.left_state if self.active_side == "left" else self.right_state

    def _image(self) -> np.ndarray:
        return self.left_bgr if self.active_side == "left" else self.right_bgr

    def _predictor(self) -> SamPredictor:
        return self.left_predictor if self.active_side == "left" else self.right_predictor

    def _mouse(self, event: int, x: int, y: int, flags: int, param: object) -> None:
        state = self._state()

        if event == cv2.EVENT_LBUTTONDOWN:
            state.pos_pts.append((x, y))
            print(f"[PROMPT] {self.active_side.upper()} + point ({x}, {y})")

        elif event == cv2.EVENT_RBUTTONDOWN:
            state.neg_pts.append((x, y))
            print(f"[PROMPT] {self.active_side.upper()} - point ({x}, {y})")

    def _toggle_side(self) -> None:
        self.active_side = "right" if self.active_side == "left" else "left"
        print(f"[INFO] Active image: {self.active_side.upper()}")

    def _draw_box(self) -> None:
        image = self._image()
        state = self._state()

        roi = cv2.selectROI(
            f"Draw box on {self.active_side.upper()} — Enter confirm, Esc cancel",
            image,
            showCrosshair=True,
            fromCenter=False,
        )
        cv2.destroyWindow(f"Draw box on {self.active_side.upper()} — Enter confirm, Esc cancel")

        x, y, w, h = roi
        if w > 0 and h > 0:
            state.box = (int(x), int(y), int(x + w), int(y + h))
            print(f"[PROMPT] {self.active_side.upper()} box: {state.box}")
        else:
            print("[INFO] Box canceled.")

    def _predict(self) -> None:
        state = self._state()
        predictor = self._predictor()

        coords = state.pos_pts + state.neg_pts
        point_coords = np.array(coords, dtype=np.float32) if coords else None

        point_labels = (
            np.array(
                [1] * len(state.pos_pts) + [0] * len(state.neg_pts),
                dtype=np.int32,
            )
            if coords
            else None
        )

        box_np = np.array(state.box, dtype=np.float32) if state.box is not None else None

        if point_coords is None and box_np is None:
            print(
                f"[WARN] Add a point or draw a box on {self.active_side.upper()} first: "
                "left click / right click / b."
            )
            return

        masks_raw, scores_raw, _ = predictor.predict(
            point_coords=point_coords,
            point_labels=point_labels,
            box=box_np,
            multimask_output=True,
        )

        masks = cast(np.ndarray, masks_raw)
        scores = cast(np.ndarray, scores_raw)
        state.masks = masks
        state.scores = scores
        state.mask_idx = int(np.argmax(scores))

        for i, s in enumerate(scores):
            marker = " <-- selected" if i == state.mask_idx else ""
            print(f"[SAM] {self.active_side.upper()} mask {i}: score={s:.4f}{marker}")

    def _cycle(self) -> None:
        state = self._state()

        if state.masks is None:
            print(f"[WARN] No {self.active_side.upper()} masks yet — press p first.")
            return

        state.mask_idx = (state.mask_idx + 1) % len(state.masks)
        score = state.scores[state.mask_idx] if state.scores is not None else float("nan")
        print(
            f"[INFO] {self.active_side.upper()} mask "
            f"{state.mask_idx}/{len(state.masks) - 1}, score={score:.4f}"
        )

    def _clear_active(self) -> None:
        self._state().clear()
        print(f"[INFO] Cleared {self.active_side.upper()} prompts and mask.")

    def _clear_both(self) -> None:
        self.left_state.clear()
        self.right_state.clear()
        print("[INFO] Cleared BOTH left/right prompts and masks.")

    def _enter_label(self) -> None:
        try:
            raw = input("[LABEL] Enter text label/note, blank to clear: ").strip()
        except EOFError:
            raw = ""

        self.label = raw
        print(f"[INFO] Label set to: {self.label!r}")

    def _render(self) -> np.ndarray:
        state = self._state()

        if state.masks is not None and state.scores is not None:
            mask_info = (
                f"{self.active_side.upper()} mask {state.mask_idx}/{len(state.masks) - 1}  "
                f"score={state.scores[state.mask_idx]:.3f}"
            )
        else:
            mask_info = f"{self.active_side.upper()} no mask — add prompt then press p"

        label_info = f"label: {self.label!r}" if self.label else "label: (none)"

        left_status = "LEFT mask: yes" if self.left_state.current_mask is not None else "LEFT mask: no"
        right_status = "RIGHT mask: yes" if self.right_state.current_mask is not None else "RIGHT mask: no"

        hud = [
            mask_info,
            f"{left_status}    {right_status}    {label_info}",
            "v=switch LEFT/RIGHT  L-click=+pt  R-click=-pt  b=box  p=predict  n=cycle",
            "t=label  s=save both  c=clear active  C=clear both  SPACE=back  q=quit",
        ]

        return _make_overlay(
            self._image(),
            state.current_mask,
            state.pos_pts,
            state.neg_pts,
            state.box,
            hud,
            self.active_side,
        )

    def _save(self) -> None:
        _save_outputs(
            out_dir=self.out_dir,
            left_bgr=self.left_bgr,
            right_bgr=self.right_bgr,
            left_mask=self.left_state.current_mask,
            right_mask=self.right_state.current_mask,
            label=self.label,
        )

    def run(self) -> bool:
        """
        Run the snapshot interaction loop.

        Returns:
          True  = go back to live view
          False = quit entire program
        """
        h, w = self.left_bgr.shape[:2]

        _open_window(WINDOW_SNAP, w, h, self.display_scale)
        cv2.setMouseCallback(WINDOW_SNAP, self._mouse)

        while True:
            cv2.imshow(WINDOW_SNAP, self._render())
            key = cv2.waitKey(20) & 0xFF

            if key in (ord("q"), 27):
                cv2.destroyWindow(WINDOW_SNAP)
                return False

            elif key == ord(" "):
                cv2.destroyWindow(WINDOW_SNAP)
                return True

            elif key == ord("v"):
                self._toggle_side()

            elif key == ord("b"):
                self._draw_box()

            elif key == ord("p"):
                self._predict()

            elif key == ord("n"):
                self._cycle()

            elif key == ord("t"):
                self._enter_label()

            elif key == ord("c"):
                self._clear_active()

            elif key == ord("C"):
                self._clear_both()

            elif key == ord("s"):
                self._save()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Live stereo camera → rectified stereo preview → snapshot → "
            "interactive LEFT/RIGHT SAM segmentation."
        )
    )

    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=DEFAULT_CHECKPOINT,
        help="SAM checkpoint .pth file.",
    )

    parser.add_argument(
        "--model-type",
        type=str,
        default="vit_b",
        choices=["vit_b", "vit_l", "vit_h"],
        help="SAM model variant. Default: vit_b.",
    )

    parser.add_argument(
        "--rectify",
        action="store_true",
        help="Rectify both left and right images using stored stereo calibration.",
    )

    parser.add_argument(
        "--out-dir",
        type=Path,
        default=DEFAULT_OUT_DIR,
        help="Folder for saved outputs.",
    )

    parser.add_argument(
        "--display-scale",
        type=float,
        default=1.4,
        help="Window display scale multiplier. Does not affect saved image resolution.",
    )

    args = parser.parse_args()

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
    print(f"[INFO] Rectify:    {args.rectify}")
    print(f"[INFO] Output dir: {args.out_dir}")
    print()

    sam = sam_model_registry[args.model_type](checkpoint=str(args.checkpoint))
    sam.to(device=device)

    rect_maps: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None = None

    with StereoCamera() as stereo:
        stereo.require_stereo_shape()
        stereo.warmup()

        ok, left0, right0 = stereo.read_pair()
        if not ok or left0 is None or right0 is None:
            raise RuntimeError("Could not read initial stereo pair from camera.")

        h, w = left0.shape[:2]

        if args.rectify:
            print("[INFO] Building left/right rectification maps...")
            rect_maps = _build_rect_maps(h, w)
            print("[INFO] Rectification maps ready.")

        preview_w = w * 2 + 8
        preview_h = h
        _open_window(WINDOW_LIVE, preview_w, preview_h, args.display_scale)

        print("[INFO] Live stereo preview active.")
        print("[INFO] Press SPACE to snapshot, q/Esc to quit.")

        keep_running = True

        while keep_running:
            ok, left, right = stereo.read_pair()
            if not ok or left is None or right is None:
                continue

            if args.rectify and rect_maps is not None:
                left_show, right_show = rectify_pair(left, right, rect_maps)
                preview_label = "[rectified] SPACE = snapshot   q = quit"
            else:
                left_show, right_show = left, right
                preview_label = "[raw] SPACE = snapshot   q = quit"

            stereo_preview = _make_stereo_preview(
                left_show,
                right_show,
                label=preview_label,
                draw_guides=True,
            )

            cv2.imshow(WINDOW_LIVE, stereo_preview)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            elif key == ord(" "):
                snapshot_left = left_show.copy()
                snapshot_right = right_show.copy()

                if args.rectify:
                    print("[INFO] Snapshot taken: rectified left/right pair.")
                else:
                    print("[INFO] Snapshot taken: raw left/right pair.")

                cv2.destroyWindow(WINDOW_LIVE)

                session = SnapshotSession(
                    left_bgr=snapshot_left,
                    right_bgr=snapshot_right,
                    sam_model=sam,
                    out_dir=args.out_dir,
                    display_scale=args.display_scale,
                )

                keep_running = session.run()

                if keep_running:
                    _open_window(WINDOW_LIVE, preview_w, preview_h, args.display_scale)
                    print("[INFO] Returned to live stereo preview.")

    cv2.destroyAllWindows()
    print("[INFO] Done.")


if __name__ == "__main__":
    main()
