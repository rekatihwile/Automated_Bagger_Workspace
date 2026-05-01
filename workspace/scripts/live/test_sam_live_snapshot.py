"""
Live SAM camera snapshot tool.

Workflow:
  1. A live stereo camera feed opens. Press SPACE to freeze a snapshot.
  2. In snapshot mode:
       Left click        = positive SAM point prompt
       Right click       = negative SAM point prompt
       b                 = draw a box prompt (opens ROI selector window)
       p                 = run SAM with current prompts
       n                 = cycle through the three mask candidates
       t                 = enter a text label / note (typed in the terminal)
       s                 = save input image, overlay, and mask PNG to disk
       c                 = clear all prompts and masks
       SPACE             = discard snapshot, return to live view
       q / Esc           = quit

Note: SAM does not accept text prompts. The label entered with 't' is
      only used as a filename suffix when saving outputs.

Usage:
  python workspace/scripts/live/test_sam_live_snapshot.py
  python workspace/scripts/live/test_sam_live_snapshot.py --rectify
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

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


DEFAULT_CHECKPOINT  = WORKSPACE_ROOT / "models" / "sam_vit_b_01ec64.pth"
DEFAULT_OUT_DIR     = config.CALIBRATION_ROOT / "sam_live_snapshot_tests"
DEFAULT_PAIRS_DIR   = config.CALIBRATION_ROOT / "sam_indexed_pairs"

WINDOW_LIVE = "SAM Live — SPACE=snapshot  q=quit"
WINDOW_SNAP = "SAM Snapshot — p=predict  n=cycle  s=save  t=label  SPACE=back  q=quit"


# ---------------------------------------------------------------------------
# Indexed-pair helpers
# ---------------------------------------------------------------------------

def _next_pair_index(pairs_dir: Path) -> int:
    """Return the next available pair index (0-based)."""
    existing = [d for d in pairs_dir.glob("pair_*") if d.is_dir()]
    if not existing:
        return 0
    indices: list[int] = []
    for d in existing:
        try:
            indices.append(int(d.name.split("_")[1]))
        except (IndexError, ValueError):
            pass
    return max(indices) + 1 if indices else 0


# ---------------------------------------------------------------------------
# Rectification
# ---------------------------------------------------------------------------

RectMaps = tuple[np.ndarray, np.ndarray]


def _build_rect_maps(
    h: int, w: int
) -> tuple[RectMaps, RectMaps]:
    """Return (left_maps, right_maps) where each is (map_x, map_y)."""
    calib = config.load_stereo_calibration()
    if calib is None:
        raise FileNotFoundError(
            f"No calibration file found at:\n  {config.ACTIVE_CALIBRATION_NPZ}"
        )
    required = [
        "left_camera_matrix",
        "left_distortion_coefficients",
        "rectification_left",
        "projection_left_rectified",
        "right_camera_matrix",
        "right_distortion_coefficients",
        "rectification_right",
        "projection_right_rectified",
    ]
    missing = [k for k in required if k not in calib]
    if missing:
        raise KeyError("Calibration missing keys:\n" + "\n".join(f"  {k}" for k in missing))

    left_maps = cv2.initUndistortRectifyMap(
        calib["left_camera_matrix"],
        calib["left_distortion_coefficients"],
        calib["rectification_left"],
        calib["projection_left_rectified"],
        (w, h),
        cv2.CV_32FC1,
    )
    right_maps = cv2.initUndistortRectifyMap(
        calib["right_camera_matrix"],
        calib["right_distortion_coefficients"],
        calib["rectification_right"],
        calib["projection_right_rectified"],
        (w, h),
        cv2.CV_32FC1,
    )
    return left_maps, right_maps


def rectify(image_bgr: np.ndarray, maps: tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    return cv2.remap(image_bgr, maps[0], maps[1], cv2.INTER_LINEAR)


# ---------------------------------------------------------------------------
# Visualization helpers
# ---------------------------------------------------------------------------

def _make_overlay(
    image_bgr: np.ndarray,
    mask: np.ndarray | None,
    pos_pts: list[tuple[int, int]],
    neg_pts: list[tuple[int, int]],
    box: tuple[int, int, int, int] | None,
    hud_lines: list[str],
) -> np.ndarray:
    out = image_bgr.copy()

    if mask is not None:
        color_layer = np.zeros_like(out)
        color_layer[mask > 0] = (0, 255, 0)
        out = cv2.addWeighted(out, 0.72, color_layer, 0.28, 0)
        contours, _ = cv2.findContours(
            (mask > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(out, contours, -1, (0, 255, 0), 2)

    for x, y in pos_pts:
        cv2.circle(out, (x, y), 5, (0, 255, 0), -1)
        cv2.circle(out, (x, y), 8, (0, 0, 0), 1)

    for x, y in neg_pts:
        cv2.circle(out, (x, y), 5, (0, 0, 255), -1)
        cv2.circle(out, (x, y), 8, (0, 0, 0), 1)

    if box is not None:
        x1, y1, x2, y2 = box
        cv2.rectangle(out, (x1, y1), (x2, y2), (255, 180, 0), 2)

    y_pos = 26
    for line in hud_lines:
        cv2.putText(out, line, (12, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(out, line, (12, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)
        y_pos += 24

    return out


def _save_outputs(
    out_dir: Path,
    image_bgr: np.ndarray,
    right_image_bgr: np.ndarray | None,
    overlay_bgr: np.ndarray,
    mask: np.ndarray | None,
    right_mask: np.ndarray | None,
    label: str,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    suffix = f"_{label}" if label else ""

    left_path = out_dir / f"{stamp}{suffix}_left.png"
    ov_path   = out_dir / f"{stamp}{suffix}_overlay.png"

    cv2.imwrite(str(left_path), image_bgr)
    cv2.imwrite(str(ov_path), overlay_bgr)
    print(f"[SAVE] Left:       {left_path}")
    print(f"[SAVE] Overlay:    {ov_path}")

    if right_image_bgr is not None:
        right_path = out_dir / f"{stamp}{suffix}_right.png"
        cv2.imwrite(str(right_path), right_image_bgr)
        print(f"[SAVE] Right:      {right_path}")

    if mask is not None:
        lmk_path = out_dir / f"{stamp}{suffix}_left_mask.png"
        cv2.imwrite(str(lmk_path), (mask > 0).astype(np.uint8) * 255)
        print(f"[SAVE] Left mask:  {lmk_path}")

    if right_mask is not None:
        rmk_path = out_dir / f"{stamp}{suffix}_right_mask.png"
        cv2.imwrite(str(rmk_path), (right_mask > 0).astype(np.uint8) * 255)
        print(f"[SAVE] Right mask: {rmk_path}")


# ---------------------------------------------------------------------------
# Snapshot session
# ---------------------------------------------------------------------------

def _open_window(name: str, img_w: int, img_h: int, scale: float) -> None:
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, max(320, int(img_w * scale)), max(240, int(img_h * scale)))


class SnapshotSession:
    """Handles all interaction on a single frozen snapshot image."""

    def __init__(
        self,
        image_bgr: np.ndarray,
        predictor: SamPredictor,
        out_dir: Path,
        display_scale: float = 2.0,
        right_image_bgr: np.ndarray | None = None,
        pairs_dir: Path = DEFAULT_PAIRS_DIR,
        rectified: bool = False,
    ) -> None:
        self.image_bgr = image_bgr
        self.right_image_bgr = right_image_bgr
        self.predictor = predictor
        self.out_dir = out_dir
        self.pairs_dir = pairs_dir
        self.rectified = rectified
        self.display_scale = display_scale

        self.pos_pts: list[tuple[int, int]] = []
        self.neg_pts: list[tuple[int, int]] = []
        self.box: tuple[int, int, int, int] | None = None
        self.masks: np.ndarray | None = None
        self.scores: np.ndarray | None = None
        self.right_masks: np.ndarray | None = None
        self.mask_idx: int = 0
        self.label: str = ""

        print("[INFO] Setting SAM image embedding (left) …")
        self.predictor.set_image(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB))
        print("[INFO] SAM ready. Interact with the snapshot window.")

    @property
    def current_mask(self) -> np.ndarray | None:
        if self.masks is None:
            return None
        return self.masks[self.mask_idx]

    @property
    def current_right_mask(self) -> np.ndarray | None:
        if self.right_masks is None:
            return None
        return self.right_masks[self.mask_idx]

    def _mouse(self, event: int, x: int, y: int, flags: int, param: object) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pos_pts.append((x, y))
            print(f"[PROMPT] + point ({x}, {y})")
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.neg_pts.append((x, y))
            print(f"[PROMPT] - point ({x}, {y})")

    def _draw_box(self) -> None:
        roi = cv2.selectROI("Draw box — Enter to confirm, Esc to cancel",
                            self.image_bgr, showCrosshair=True, fromCenter=False)
        cv2.destroyWindow("Draw box — Enter to confirm, Esc to cancel")
        x, y, w, h = roi
        if w > 0 and h > 0:
            self.box = (int(x), int(y), int(x + w), int(y + h))
            print(f"[PROMPT] Box: {self.box}")
        else:
            print("[INFO] Box canceled.")

    def _predict(self) -> None:
        coords = self.pos_pts + self.neg_pts
        point_coords = np.array(coords, dtype=np.float32) if coords else None
        point_labels = (
            np.array([1] * len(self.pos_pts) + [0] * len(self.neg_pts), dtype=np.int32)
            if coords else None
        )
        box_np = np.array(self.box, dtype=np.float32) if self.box is not None else None

        if point_coords is None and box_np is None:
            print("[WARN] Add a point or draw a box first (left click / right click / b).")
            return

        masks, scores, _ = self.predictor.predict(
            point_coords=point_coords,
            point_labels=point_labels,
            box=box_np,
            multimask_output=True,
        )
        self.masks = masks
        self.scores = scores
        self.mask_idx = int(np.argmax(scores))
        for i, s in enumerate(scores):
            marker = " <--" if i == self.mask_idx else ""
            print(f"[SAM left]  mask {i}: score={s:.4f}{marker}")

        if self.right_image_bgr is not None:
            print("[INFO] Running SAM on right image …")
            self.predictor.set_image(cv2.cvtColor(self.right_image_bgr, cv2.COLOR_BGR2RGB))
            right_masks, right_scores, _ = self.predictor.predict(
                point_coords=point_coords,
                point_labels=point_labels,
                box=box_np,
                multimask_output=True,
            )
            self.right_masks = right_masks
            for i, s in enumerate(right_scores):
                marker = " <--" if i == self.mask_idx else ""
                print(f"[SAM right] mask {i}: score={s:.4f}{marker}")
            # Restore left image embedding
            print("[INFO] Restoring left image embedding …")
            self.predictor.set_image(cv2.cvtColor(self.image_bgr, cv2.COLOR_BGR2RGB))

    def _cycle(self) -> None:
        if self.masks is None:
            print("[WARN] No masks yet — press p first.")
            return
        self.mask_idx = (self.mask_idx + 1) % len(self.masks)
        score = self.scores[self.mask_idx] if self.scores is not None else float("nan")
        print(f"[INFO] Mask {self.mask_idx}/{len(self.masks)-1}, score={score:.4f}")

    def _clear(self) -> None:
        self.pos_pts.clear()
        self.neg_pts.clear()
        self.box = None
        self.masks = None
        self.scores = None
        self.right_masks = None
        self.mask_idx = 0
        print("[INFO] Prompts and masks cleared.")

    def _enter_label(self) -> None:
        try:
            raw = input("[LABEL] Enter text label/note (leave blank to clear): ").strip()
        except EOFError:
            raw = ""
        self.label = raw
        print(f"[INFO] Label set to: {self.label!r}")

    def _save_indexed_pair(self) -> None:
        """Save rectified images, masks, overlays, and metadata into a new pair_XXXXXX folder."""
        if self.current_mask is None:
            print("[WARN] No left mask yet — SAM has not been run (press p). Saving pair without masks.")
        if self.right_image_bgr is not None and self.current_right_mask is None:
            print("[WARN] No right mask — pair will be saved without right_mask.png.")

        self.pairs_dir.mkdir(parents=True, exist_ok=True)
        idx      = _next_pair_index(self.pairs_dir)
        pair_dir = self.pairs_dir / f"pair_{idx:06d}"
        pair_dir.mkdir(parents=True, exist_ok=True)

        has_left_mask  = self.current_mask       is not None
        has_right_mask = self.current_right_mask is not None

        # Rectified images
        cv2.imwrite(str(pair_dir / "left_rect.png"), self.image_bgr)
        if self.right_image_bgr is not None:
            cv2.imwrite(str(pair_dir / "right_rect.png"), self.right_image_bgr)

        # Binary masks
        if has_left_mask:
            cv2.imwrite(
                str(pair_dir / "left_mask.png"),
                (self.current_mask > 0).astype(np.uint8) * 255,
            )
        if has_right_mask:
            cv2.imwrite(
                str(pair_dir / "right_mask.png"),
                (self.current_right_mask > 0).astype(np.uint8) * 255,
            )

        # Clean overlays (no prompt dots or box — just mask color)
        left_overlay = _make_overlay(self.image_bgr, self.current_mask, [], [], None, [])
        cv2.imwrite(str(pair_dir / "left_overlay.png"), left_overlay)

        if self.right_image_bgr is not None:
            right_overlay = _make_overlay(
                self.right_image_bgr, self.current_right_mask, [], [], None, []
            )
            cv2.imwrite(str(pair_dir / "right_overlay.png"), right_overlay)

            stereo_rect = np.hstack([self.image_bgr, self.right_image_bgr])
            cv2.imwrite(str(pair_dir / "stereo_rect_preview.png"), stereo_rect)

            stereo_mask_preview = np.hstack([left_overlay, right_overlay])
            cv2.imwrite(str(pair_dir / "stereo_mask_overlay_preview.png"), stereo_mask_preview)

        # Metadata
        metadata = {
            "index":          idx,
            "created_at":     datetime.now().isoformat(),
            "source":         "live_sam",
            "original_stem":  datetime.now().strftime("%Y%m%d_%H%M%S"),
            "has_left_mask":  has_left_mask,
            "has_right_mask": has_right_mask,
            "rectified":      self.rectified,
            "label":          self.label,
        }
        (pair_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )

        print(f"[SAVE] Indexed pair {idx:06d} → {pair_dir}")
        print(f"       left_mask={has_left_mask}  right_mask={has_right_mask}  rectified={self.rectified}")
        print(f"       Point cloud: python workspace/scripts/offline/masked_stereo_to_cloud.py --index {idx}")

    def _render(self) -> np.ndarray:
        if self.masks is not None and self.scores is not None:
            mask_info = f"mask {self.mask_idx}/{len(self.masks)-1}  score={self.scores[self.mask_idx]:.3f}"
        else:
            mask_info = "no mask — add prompt then press p"

        label_info = f"label: {self.label!r}" if self.label else "label: (none)"

        hud = [
            mask_info,
            label_info,
            "L-click=+pt  R-click=-pt  b=box  p=predict  n=cycle",
            "t=label  s=save  c=clear  SPACE=back  q=quit",
        ]
        return _make_overlay(
            self.image_bgr, self.current_mask,
            self.pos_pts, self.neg_pts, self.box, hud,
        )

    def run(self) -> bool:
        """Run the snapshot interaction loop. Returns True to keep running, False to quit."""
        h, w = self.image_bgr.shape[:2]
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

            elif key == ord("b"):
                self._draw_box()

            elif key == ord("p"):
                self._predict()

            elif key == ord("n"):
                self._cycle()

            elif key == ord("t"):
                self._enter_label()

            elif key == ord("c"):
                self._clear()

            elif key == ord("s"):
                self._save_indexed_pair()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Live stereo camera → SPACE to snapshot → interactive SAM segmentation."
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
        help="SAM model variant (default: vit_b).",
    )
    parser.add_argument(
        "--rectify",
        action="store_true",
        help="Rectify the left image using the stored stereo calibration before running SAM.",
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
        default=2.0,
        help="Window display scale multiplier (default: 2.0). Does not affect saved image resolution.",
    )
    parser.add_argument(
        "--pairs-dir",
        type=Path,
        default=DEFAULT_PAIRS_DIR,
        help=f"Root folder for indexed pair output (default: {DEFAULT_PAIRS_DIR}).",
    )
    args = parser.parse_args()

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
    predictor = SamPredictor(sam)

    left_rect_maps: tuple[np.ndarray, np.ndarray] | None = None
    right_rect_maps: tuple[np.ndarray, np.ndarray] | None = None

    with StereoCamera() as stereo:
        stereo.warmup()

        if args.rectify:
            ok, left, _ = stereo.read_pair()
            if ok and left is not None:
                h, w = left.shape[:2]
                print("[INFO] Building rectification maps …")
                left_rect_maps, right_rect_maps = _build_rect_maps(h, w)
                print("[INFO] Rectification maps ready.")

        live_w, live_h = config.STREAM_EYE_WIDTH, config.STREAM_EYE_HEIGHT
        _open_window(WINDOW_LIVE, live_w, live_h, args.display_scale)
        print(f"[INFO] Live view active. Press SPACE to snapshot, q/Esc to quit.")

        keep_running = True
        while keep_running:
            ok, left, right = stereo.read_pair()
            if not ok or left is None:
                continue

            display = left.copy()
            hud = ["SPACE = snapshot   q = quit"]
            if args.rectify:
                hud[0] = "[rectified]  " + hud[0]
            y = 26
            for line in hud:
                cv2.putText(display, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(display, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
                y += 24

            cv2.imshow(WINDOW_LIVE, display)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            elif key == ord(" "):
                ok2, left2, right2 = stereo.read_pair()
                snap_left = (left2 if ok2 and left2 is not None else left).copy()
                snap_right = (right2 if ok2 and right2 is not None else None)
                snap_right = snap_right.copy() if snap_right is not None else None

                if args.rectify and left_rect_maps is not None:
                    snap_left = rectify(snap_left, left_rect_maps)
                    if snap_right is not None and right_rect_maps is not None:
                        snap_right = rectify(snap_right, right_rect_maps)
                    print("[INFO] Snapshot taken (rectified).")
                else:
                    print("[INFO] Snapshot taken.")

                cv2.destroyWindow(WINDOW_LIVE)

                session = SnapshotSession(
                    image_bgr=snap_left,
                    predictor=predictor,
                    out_dir=args.out_dir,
                    display_scale=args.display_scale,
                    right_image_bgr=snap_right,
                    pairs_dir=args.pairs_dir,
                    rectified=args.rectify,
                )
                keep_running = session.run()

                if keep_running:
                    _open_window(WINDOW_LIVE, live_w, live_h, args.display_scale)
                    print("[INFO] Returned to live view.")

    cv2.destroyAllWindows()
    print("[INFO] Done.")


if __name__ == "__main__":
    main()
