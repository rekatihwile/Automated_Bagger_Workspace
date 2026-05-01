"""
Live HBVCam snapshot + SAM mask tester.

What this does:
1. Opens your stereo camera.
2. Captures one live stereo frame.
3. Splits left/right.
4. Optionally rectifies the left image.
5. Lets you click or box-prompt the object.
6. Runs SAM.
7. Saves the input image, mask, and overlay.

Controls:
  SPACE  = take new snapshot from camera
  b      = draw box around object
  left click  = positive SAM point
  right click = negative SAM point
  p      = predict SAM mask
  n      = cycle mask candidate
  t      = type a text label/note for saved filename/overlay
  s      = save current mask/overlay
  c      = clear prompts
  q/Esc  = quit

Important:
  Regular SAM does NOT understand text prompts like "pill bottle".
  The text label here is only a saved note, not a segmentation prompt.
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

if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import config
from camera import StereoCamera


try:
    from segment_anything import sam_model_registry, SamPredictor
except ImportError as exc:
    raise ImportError(
        "\nCould not import segment_anything.\n\n"
        "Install it with:\n"
        "  pip install segment-anything\n"
    ) from exc


WINDOW = "Live SAM Snapshot"


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

    return cv2.remap(left_bgr, map_lx, map_ly, cv2.INTER_LINEAR)


def colorize_mask(mask: np.ndarray, color_bgr=(0, 255, 0)) -> np.ndarray:
    out = np.zeros((*mask.shape, 3), dtype=np.uint8)
    out[mask > 0] = color_bgr
    return out


def make_overlay(
    image_bgr: np.ndarray,
    mask: np.ndarray | None,
    pos_points: list[tuple[int, int]],
    neg_points: list[tuple[int, int]],
    box: tuple[int, int, int, int] | None,
    label: str,
    status: str,
) -> np.ndarray:
    out = image_bgr.copy()

    if mask is not None:
        mask_u8 = (mask > 0).astype(np.uint8)
        mask_color = colorize_mask(mask_u8)
        out = cv2.addWeighted(out, 0.72, mask_color, 0.28, 0)

        contours, _ = cv2.findContours(
            mask_u8,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        cv2.drawContours(out, contours, -1, (0, 255, 0), 2)

    for x, y in pos_points:
        cv2.circle(out, (x, y), 6, (0, 255, 0), -1)
        cv2.circle(out, (x, y), 9, (0, 0, 0), 1)

    for x, y in neg_points:
        cv2.circle(out, (x, y), 6, (0, 0, 255), -1)
        cv2.circle(out, (x, y), 9, (0, 0, 0), 1)

    if box is not None:
        x1, y1, x2, y2 = box
        cv2.rectangle(out, (x1, y1), (x2, y2), (255, 180, 0), 2)

    lines = [
        status,
        f"label/note: {label if label else '(none)'}",
        "SPACE=snapshot | b=box | left+=object | right+=background | p=predict | n=next | t=label | s=save | c=clear | q=quit",
    ]

    y = 28
    for line in lines:
        cv2.putText(out, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(out, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
        y += 26

    return out


def save_outputs(
    out_dir: Path,
    image_bgr: np.ndarray,
    overlay_bgr: np.ndarray,
    mask: np.ndarray | None,
    label: str,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_label = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in label.strip())
    if safe_label:
        stem = f"{stamp}_{safe_label}"
    else:
        stem = stamp

    input_path = out_dir / f"{stem}_input.png"
    overlay_path = out_dir / f"{stem}_overlay.png"
    mask_path = out_dir / f"{stem}_mask.png"

    cv2.imwrite(str(input_path), image_bgr)
    cv2.imwrite(str(overlay_path), overlay_bgr)

    if mask is not None:
        cv2.imwrite(str(mask_path), (mask > 0).astype(np.uint8) * 255)

    print(f"[SAVE] Input:   {input_path}")
    print(f"[SAVE] Overlay: {overlay_path}")
    if mask is not None:
        print(f"[SAVE] Mask:    {mask_path}")


class LiveSamSnapshotTool:
    def __init__(
        self,
        predictor: SamPredictor,
        rectify: bool,
        out_dir: Path,
    ) -> None:
        self.predictor = predictor
        self.rectify = rectify
        self.out_dir = out_dir

        self.image_bgr: np.ndarray | None = None
        self.image_rgb: np.ndarray | None = None

        self.pos_points: list[tuple[int, int]] = []
        self.neg_points: list[tuple[int, int]] = []
        self.box: tuple[int, int, int, int] | None = None

        self.masks: np.ndarray | None = None
        self.scores: np.ndarray | None = None
        self.current_mask_idx = 0

        self.label = ""
        self.status = "Press SPACE to take a snapshot."

    @property
    def current_mask(self) -> np.ndarray | None:
        if self.masks is None:
            return None
        return self.masks[self.current_mask_idx]

    def clear_prompts(self) -> None:
        self.pos_points.clear()
        self.neg_points.clear()
        self.box = None
        self.masks = None
        self.scores = None
        self.current_mask_idx = 0
        self.status = "Cleared prompts."

    def set_image(self, left_bgr: np.ndarray) -> None:
        if self.rectify:
            print("[INFO] Rectifying left image...")
            left_bgr = rectify_left_image(left_bgr)

        self.image_bgr = left_bgr
        self.image_rgb = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2RGB)

        self.clear_prompts()
        self.status = "Setting SAM image embedding..."
        print("[INFO] Setting SAM image embedding. This may take a few seconds.")
        self.predictor.set_image(self.image_rgb)
        self.status = "Snapshot ready. Draw box or click points, then press p."
        print("[INFO] Snapshot ready.")

    def take_snapshot(self, cam: StereoCamera) -> None:
        ok, left, right = cam.read_pair()
        if not ok or left is None:
            self.status = "Failed to read camera frame."
            print("[WARN] Failed to read camera frame.")
            return

        self.set_image(left)

    def mouse_callback(self, event, x, y, flags, param) -> None:
        if self.image_bgr is None:
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            self.pos_points.append((x, y))
            self.status = f"Added positive point ({x}, {y})."
            print(f"[PROMPT] Positive point: ({x}, {y})")

        elif event == cv2.EVENT_RBUTTONDOWN:
            self.neg_points.append((x, y))
            self.status = f"Added negative point ({x}, {y})."
            print(f"[PROMPT] Negative point: ({x}, {y})")

    def select_box(self) -> None:
        if self.image_bgr is None:
            self.status = "Take a snapshot first."
            return

        roi = cv2.selectROI(
            "Draw SAM box prompt",
            self.image_bgr,
            showCrosshair=True,
            fromCenter=False,
        )
        cv2.destroyWindow("Draw SAM box prompt")

        x, y, w, h = roi
        if w <= 0 or h <= 0:
            self.status = "Box selection canceled."
            return

        self.box = (int(x), int(y), int(x + w), int(y + h))
        self.status = f"Box selected: {self.box}. Press p to predict."
        print(f"[PROMPT] Box: {self.box}")

    def predict(self) -> None:
        if self.image_bgr is None:
            self.status = "Take a snapshot first."
            return

        point_coords = None
        point_labels = None
        box_np = None

        all_points = self.pos_points + self.neg_points

        if all_points:
            point_coords = np.array(all_points, dtype=np.float32)
            point_labels = np.array(
                [1] * len(self.pos_points) + [0] * len(self.neg_points),
                dtype=np.int32,
            )

        if self.box is not None:
            box_np = np.array(self.box, dtype=np.float32)

        if point_coords is None and box_np is None:
            self.status = "Add a click or draw a box first."
            print("[WARN] Add a click or draw a box first.")
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

        self.status = f"Predicted mask {self.current_mask_idx}, score={scores[self.current_mask_idx]:.3f}"
        print("[OK] Predicted masks:")
        for i, score in enumerate(scores):
            print(f"     mask {i}: score={score:.4f}")
        print(f"     selected mask {self.current_mask_idx}")

    def cycle_mask(self) -> None:
        if self.masks is None:
            self.status = "No masks yet. Press p first."
            return

        self.current_mask_idx = (self.current_mask_idx + 1) % len(self.masks)
        score = self.scores[self.current_mask_idx] if self.scores is not None else float("nan")
        self.status = f"Current mask {self.current_mask_idx}, score={score:.3f}"
        print(f"[INFO] Current mask {self.current_mask_idx}, score={score:.4f}")

    def enter_label(self) -> None:
        label = input("Enter label/note for this object, e.g. pill_bottle: ").strip()
        self.label = label
        self.status = f"Label set to: {label}"
        print(f"[INFO] Label set to: {label}")

    def draw(self) -> np.ndarray:
        if self.image_bgr is None:
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            return make_overlay(
                blank,
                None,
                [],
                [],
                None,
                self.label,
                self.status,
            )

        return make_overlay(
            self.image_bgr,
            self.current_mask,
            self.pos_points,
            self.neg_points,
            self.box,
            self.label,
            self.status,
        )

    def save(self) -> None:
        if self.image_bgr is None:
            self.status = "Nothing to save. Take a snapshot first."
            return

        overlay = self.draw()
        save_outputs(
            out_dir=self.out_dir,
            image_bgr=self.image_bgr,
            overlay_bgr=overlay,
            mask=self.current_mask,
            label=self.label,
        )
        self.status = "Saved input, overlay, and mask."

    def run(self) -> None:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW, self.mouse_callback)

        with StereoCamera() as cam:
            print("[INFO] Opening stereo camera...")
            cam.require_stereo_shape()
            cam.warmup()

            print("[READY]")
            print("  SPACE  = take snapshot")
            print("  b      = box prompt")
            print("  left click  = positive point")
            print("  right click = negative point")
            print("  p      = predict")
            print("  n      = next mask")
            print("  t      = enter text label/note")
            print("  s      = save")
            print("  c      = clear")
            print("  q/Esc  = quit")

            while True:
                view = self.draw()
                cv2.imshow(WINDOW, view)

                key = cv2.waitKey(20) & 0xFF

                if key in (ord("q"), 27):
                    break

                elif key == ord(" "):
                    self.take_snapshot(cam)

                elif key == ord("b"):
                    self.select_box()

                elif key == ord("p"):
                    self.predict()

                elif key == ord("n"):
                    self.cycle_mask()

                elif key == ord("t"):
                    self.enter_label()

                elif key == ord("s"):
                    self.save()

                elif key == ord("c"):
                    self.clear_prompts()

        cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--checkpoint",
        type=Path,
        required=True,
        help="Path to SAM checkpoint, e.g. workspace/models/sam_vit_b.pth",
    )
    parser.add_argument(
        "--model-type",
        type=str,
        default="vit_b",
        choices=["vit_b", "vit_l", "vit_h"],
        help="SAM model type. vit_b is fastest/smallest.",
    )
    parser.add_argument(
        "--rectify",
        action="store_true",
        help="Rectify the left image before SAM.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=config.CALIBRATION_ROOT / "sam_live_snapshot_tests",
        help="Folder for saved masks and overlays.",
    )

    args = parser.parse_args()

    if not args.checkpoint.exists():
        raise FileNotFoundError(f"SAM checkpoint not found: {args.checkpoint}")

    device = "cuda" if torch.cuda.is_available() else "cpu"

    print(f"[INFO] Device: {device}")
    print(f"[INFO] Loading SAM model: {args.model_type}")
    print(f"[INFO] Checkpoint: {args.checkpoint}")

    sam = sam_model_registry[args.model_type](checkpoint=str(args.checkpoint))
    sam.to(device=device)

    predictor = SamPredictor(sam)

    tool = LiveSamSnapshotTool(
        predictor=predictor,
        rectify=args.rectify,
        out_dir=args.out_dir,
    )

    tool.run()


if __name__ == "__main__":
    main()