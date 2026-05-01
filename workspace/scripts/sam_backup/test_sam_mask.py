"""
Test Segment Anything on a saved image or saved stereo pair.

Purpose:
  First SAM sanity check before doing stereo point clouds.

Modes:
  1. Single image:
      python workspace/scripts/offline/test_sam_mask.py --image path/to/image.png --checkpoint workspace/models/sam_vit_b.pth

  2. Stereo pair from your saved capture folders:
      python workspace/scripts/offline/test_sam_mask.py --index 40 --checkpoint workspace/models/sam_vit_b.pth --rectify

Controls:
  Left click       = positive SAM point
  Right click      = negative SAM point
  b                = draw box prompt
  p                = predict mask
  n                = cycle mask candidate
  s                = save current mask + overlay
  c                = clear prompts
  q / Esc          = quit

Recommended first use:
  Draw a box around the object with 'b', then press 'p'.
  Box prompts are usually much better than single-click prompts.
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import torch


# ---------------------------------------------------------------------
# Workspace imports
# ---------------------------------------------------------------------

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = WORKSPACE_ROOT.parent

if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import config


# ---------------------------------------------------------------------
# SAM import
# ---------------------------------------------------------------------

try:
    from segment_anything import sam_model_registry, SamPredictor
except ImportError as exc:
    raise ImportError(
        "\nCould not import segment_anything.\n\n"
        "Install it with:\n"
        "  pip install segment-anything\n"
    ) from exc


WINDOW = "SAM mask test"


# ---------------------------------------------------------------------
# Image loading / rectification
# ---------------------------------------------------------------------

def list_pairs(left_dir: Path, right_dir: Path) -> list[tuple[Path, Path]]:
    exts = ("*.png", "*.jpg", "*.jpeg", "*.bmp")

    lefts = []
    rights = []

    for ext in exts:
        lefts.extend(sorted(left_dir.glob(ext)))
        rights.extend(sorted(right_dir.glob(ext)))

    lefts = sorted(lefts)
    rights = sorted(rights)

    if not lefts:
        raise FileNotFoundError(f"No left images found in: {left_dir}")
    if not rights:
        raise FileNotFoundError(f"No right images found in: {right_dir}")

    n = min(len(lefts), len(rights))
    return list(zip(lefts[:n], rights[:n]))


def load_calibration() -> dict[str, np.ndarray]:
    calib = config.load_stereo_calibration()

    if calib is None:
        raise FileNotFoundError(
            f"No calibration file found at:\n"
            f"  {config.ACTIVE_CALIBRATION_NPZ}\n"
        )

    required = [
        "left_camera_matrix",
        "left_distortion_coefficients",
        "rectification_left",
        "projection_left_rectified",
    ]

    missing = [k for k in required if k not in calib]
    if missing:
        raise KeyError(
            "Calibration file is missing keys:\n"
            + "\n".join(f"  - {k}" for k in missing)
        )

    return calib


def rectify_left_image(left_bgr: np.ndarray) -> np.ndarray:
    calib = load_calibration()

    h, w = left_bgr.shape[:2]

    map_lx, map_ly = cv2.initUndistortRectifyMap(
        calib["left_camera_matrix"],
        calib["left_distortion_coefficients"],
        calib["rectification_left"],
        calib["projection_left_rectified"],
        (w, h),
        cv2.CV_32FC1,
    )

    left_rect = cv2.remap(left_bgr, map_lx, map_ly, cv2.INTER_LINEAR)
    return left_rect


def load_input_image(args: argparse.Namespace) -> tuple[np.ndarray, str]:
    if args.image is not None:
        img = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
        if img is None:
            raise FileNotFoundError(f"Could not read image: {args.image}")

        label = str(args.image)
        return img, label

    pairs = list_pairs(args.left_dir, args.right_dir)

    if args.index < 0 or args.index >= len(pairs):
        raise IndexError(f"--index {args.index} outside range 0..{len(pairs) - 1}")

    left_path, right_path = pairs[args.index]
    img = cv2.imread(str(left_path), cv2.IMREAD_COLOR)

    if img is None:
        raise FileNotFoundError(f"Could not read left image: {left_path}")

    label = f"pair index {args.index}: {left_path.name}"

    if args.rectify:
        print("[INFO] Rectifying left image before SAM...")
        img = rectify_left_image(img)
        label += " [rectified]"

    return img, label


# ---------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------

def colorize_mask(mask: np.ndarray, color_bgr: tuple[int, int, int] = (0, 255, 0)) -> np.ndarray:
    mask_u8 = (mask > 0).astype(np.uint8)
    color = np.zeros((*mask_u8.shape, 3), dtype=np.uint8)
    color[mask_u8 > 0] = color_bgr
    return color


def make_overlay(
    image_bgr: np.ndarray,
    mask: np.ndarray | None,
    pos_points: list[tuple[int, int]],
    neg_points: list[tuple[int, int]],
    box: tuple[int, int, int, int] | None,
    text_lines: list[str],
) -> np.ndarray:
    out = image_bgr.copy()

    if mask is not None:
        mask_color = colorize_mask(mask, (0, 255, 0))
        out = cv2.addWeighted(out, 0.72, mask_color, 0.28, 0)

        contours, _ = cv2.findContours(
            (mask > 0).astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        cv2.drawContours(out, contours, -1, (0, 255, 0), 2)

    for x, y in pos_points:
        cv2.circle(out, (x, y), 5, (0, 255, 0), -1)
        cv2.circle(out, (x, y), 8, (0, 0, 0), 1)

    for x, y in neg_points:
        cv2.circle(out, (x, y), 5, (0, 0, 255), -1)
        cv2.circle(out, (x, y), 8, (0, 0, 0), 1)

    if box is not None:
        x1, y1, x2, y2 = box
        cv2.rectangle(out, (x1, y1), (x2, y2), (255, 180, 0), 2)

    y = 26
    for line in text_lines:
        cv2.putText(out, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(out, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
        y += 25

    return out


def save_outputs(
    out_dir: Path,
    image_bgr: np.ndarray,
    overlay_bgr: np.ndarray,
    mask: np.ndarray | None,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    image_path = out_dir / f"{stamp}_sam_input.png"
    overlay_path = out_dir / f"{stamp}_sam_overlay.png"
    mask_path = out_dir / f"{stamp}_sam_mask.png"

    cv2.imwrite(str(image_path), image_bgr)
    cv2.imwrite(str(overlay_path), overlay_bgr)

    if mask is not None:
        cv2.imwrite(str(mask_path), (mask > 0).astype(np.uint8) * 255)

    print(f"[SAVE] Input:   {image_path}")
    print(f"[SAVE] Overlay: {overlay_path}")
    if mask is not None:
        print(f"[SAVE] Mask:    {mask_path}")


# ---------------------------------------------------------------------
# Main interactive SAM tool
# ---------------------------------------------------------------------

class SamMaskTester:
    def __init__(
        self,
        image_bgr: np.ndarray,
        predictor: SamPredictor,
        out_dir: Path,
        label: str,
    ) -> None:
        self.image_bgr = image_bgr
        self.image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        self.predictor = predictor
        self.out_dir = out_dir
        self.label = label

        self.pos_points: list[tuple[int, int]] = []
        self.neg_points: list[tuple[int, int]] = []
        self.box: tuple[int, int, int, int] | None = None

        self.masks: np.ndarray | None = None
        self.scores: np.ndarray | None = None
        self.current_mask_idx = 0

        print("[INFO] Setting SAM image embedding. This may take a few seconds...")
        self.predictor.set_image(self.image_rgb)
        print("[INFO] SAM is ready.")

    @property
    def current_mask(self) -> np.ndarray | None:
        if self.masks is None:
            return None
        return self.masks[self.current_mask_idx]

    def mouse_callback(self, event, x, y, flags, param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pos_points.append((x, y))
            print(f"[PROMPT] Positive point: ({x}, {y})")

        elif event == cv2.EVENT_RBUTTONDOWN:
            self.neg_points.append((x, y))
            print(f"[PROMPT] Negative point: ({x}, {y})")

    def clear_prompts(self) -> None:
        self.pos_points.clear()
        self.neg_points.clear()
        self.box = None
        self.masks = None
        self.scores = None
        self.current_mask_idx = 0
        print("[INFO] Cleared prompts and masks.")

    def select_box(self) -> None:
        temp = self.image_bgr.copy()
        roi = cv2.selectROI("Draw SAM box prompt", temp, showCrosshair=True, fromCenter=False)
        cv2.destroyWindow("Draw SAM box prompt")

        x, y, w, h = roi
        if w <= 0 or h <= 0:
            print("[INFO] Box selection canceled.")
            return

        self.box = (int(x), int(y), int(x + w), int(y + h))
        print(f"[PROMPT] Box: {self.box}")

    def predict(self) -> None:
        point_coords = None
        point_labels = None
        box_np = None

        points = self.pos_points + self.neg_points
        if points:
            point_coords = np.array(points, dtype=np.float32)
            point_labels = np.array(
                [1] * len(self.pos_points) + [0] * len(self.neg_points),
                dtype=np.int32,
            )

        if self.box is not None:
            box_np = np.array(self.box, dtype=np.float32)

        if point_coords is None and box_np is None:
            print("[WARN] Add a point or draw a box first.")
            return

        masks, scores, logits = self.predictor.predict(
            point_coords=point_coords,
            point_labels=point_labels,
            box=box_np,
            multimask_output=True,
        )

        self.masks = masks
        self.scores = scores
        self.current_mask_idx = int(np.argmax(scores))

        print("[OK] Predicted masks:")
        for i, s in enumerate(scores):
            print(f"     mask {i}: score={s:.4f}")
        print(f"     selected mask {self.current_mask_idx}")

    def cycle_mask(self) -> None:
        if self.masks is None:
            print("[WARN] No masks yet. Press 'p' first.")
            return

        self.current_mask_idx = (self.current_mask_idx + 1) % len(self.masks)
        score = self.scores[self.current_mask_idx] if self.scores is not None else float("nan")
        print(f"[INFO] Current mask: {self.current_mask_idx}, score={score:.4f}")

    def draw(self) -> np.ndarray:
        if self.masks is not None and self.scores is not None:
            score = self.scores[self.current_mask_idx]
            mask_text = f"mask {self.current_mask_idx}/{len(self.masks)-1}, score={score:.3f}"
        else:
            mask_text = "no mask yet"

        lines = [
            self.label,
            mask_text,
            "left click=positive | right click=negative | b=box | p=predict | n=next | s=save | c=clear | q=quit",
        ]

        return make_overlay(
            self.image_bgr,
            self.current_mask,
            self.pos_points,
            self.neg_points,
            self.box,
            lines,
        )

    def run(self) -> None:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW, self.mouse_callback)

        while True:
            view = self.draw()
            cv2.imshow(WINDOW, view)

            key = cv2.waitKey(20) & 0xFF

            if key in (ord("q"), 27):
                break

            elif key == ord("b"):
                self.select_box()

            elif key == ord("p"):
                self.predict()

            elif key == ord("n"):
                self.cycle_mask()

            elif key == ord("c"):
                self.clear_prompts()

            elif key == ord("s"):
                overlay = self.draw()
                save_outputs(self.out_dir, self.image_bgr, overlay, self.current_mask)

        cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser()

    parser.add_argument("--image", type=Path, default=None, help="Single image to segment.")
    parser.add_argument("--index", type=int, default=0, help="Stereo pair index from capture folders.")
    parser.add_argument("--left-dir", type=Path, default=config.CAPTURE_LEFT_DIR)
    parser.add_argument("--right-dir", type=Path, default=config.CAPTURE_RIGHT_DIR)
    parser.add_argument("--rectify", action="store_true", help="Rectify left image before SAM.")
    parser.add_argument("--checkpoint", type=Path, required=True, help="SAM checkpoint .pth file.")
    parser.add_argument(
        "--model-type",
        type=str,
        default="vit_b",
        choices=["vit_b", "vit_l", "vit_h"],
        help="SAM model type. vit_b is fastest/smallest.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=config.CALIBRATION_ROOT / "sam_mask_tests",
        help="Output folder for masks and overlays.",
    )

    args = parser.parse_args()

    if not args.checkpoint.exists():
        raise FileNotFoundError(f"SAM checkpoint not found: {args.checkpoint}")

    image_bgr, label = load_input_image(args)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[INFO] Device: {device}")
    print(f"[INFO] Loading SAM model: {args.model_type}")
    print(f"[INFO] Checkpoint: {args.checkpoint}")

    sam = sam_model_registry[args.model_type](checkpoint=str(args.checkpoint))
    sam.to(device=device)

    predictor = SamPredictor(sam)

    tester = SamMaskTester(
        image_bgr=image_bgr,
        predictor=predictor,
        out_dir=args.out_dir,
        label=label,
    )

    tester.run()


if __name__ == "__main__":
    main()