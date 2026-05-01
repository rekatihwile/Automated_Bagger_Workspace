from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from ultralytics import YOLO


DEFAULT_MODEL_NAME = "1000EpochModel.pt"


@dataclass
class SegmentationResult:
    confidence: float
    contour: np.ndarray          # (N, 2) float32, dense contour
    sampled: np.ndarray          # (M, 2) float32, evenly sampled boundary points
    mask: np.ndarray             # uint8 binary mask full-res


class TopFaceSegmenter:
    def __init__(
        self,
        model_path: str | Path | None = None,
        sample_count: int = 32,
        input_size: int = 342,
    ) -> None:
        if model_path is None:
            base = Path(__file__).resolve().parent
            preferred = base / "models" / DEFAULT_MODEL_NAME
            legacy = base / DEFAULT_MODEL_NAME
            model_path = preferred if preferred.exists() else legacy
        self.model_path = str(model_path)
        self.sample_count = int(sample_count)
        self.input_size = int(input_size)
        self.model = YOLO(self.model_path)

    def detect(self, frame_bgr: np.ndarray, conf: float = 0.35) -> Optional[SegmentationResult]:
        results = self.model.predict(
            source=frame_bgr,
            conf=float(conf),
            imgsz=self.input_size,
            verbose=False,
            retina_masks=True,
        )
        if not results:
            return None

        r = results[0]
        if r.masks is None or r.boxes is None or len(r.boxes) == 0:
            return None

        confs = r.boxes.conf.detach().cpu().numpy().astype(np.float32)
        best_idx = int(np.argmax(confs))
        best_conf = float(confs[best_idx])

        masks = r.masks.data.detach().cpu().numpy()
        mask = masks[best_idx]
        mask = (mask > 0.5).astype(np.uint8) * 255

        if mask.shape[:2] != frame_bgr.shape[:2]:
            mask = cv2.resize(mask, (frame_bgr.shape[1], frame_bgr.shape[0]), interpolation=cv2.INTER_NEAREST)

        contour = self._largest_contour(mask)
        if contour is None or len(contour) < 8:
            return None

        sampled = self._sample_contour_evenly(contour, self.sample_count)
        if sampled is None or len(sampled) < 4:
            return None

        return SegmentationResult(
            confidence=best_conf,
            contour=contour.astype(np.float32),
            sampled=sampled.astype(np.float32),
            mask=mask,
        )

    @staticmethod
    def _largest_contour(mask: np.ndarray) -> Optional[np.ndarray]:
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < 20:
            return None
        return c.reshape(-1, 2)

    @staticmethod
    def _sample_contour_evenly(contour_xy: np.ndarray, sample_count: int) -> Optional[np.ndarray]:
        pts = np.asarray(contour_xy, dtype=np.float32).reshape(-1, 2)
        if len(pts) < 2:
            return None

        closed = np.vstack([pts, pts[0]])
        seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
        total = float(np.sum(seg))
        if total <= 1e-6:
            return None

        targets = np.linspace(0.0, total, int(sample_count), endpoint=False)
        out = []
        acc = 0.0
        j = 0

        for t in targets:
            while j < len(seg) - 1 and acc + seg[j] < t:
                acc += seg[j]
                j += 1
            if seg[j] <= 1e-6:
                out.append(closed[j].copy())
            else:
                alpha = (t - acc) / seg[j]
                p = (1.0 - alpha) * closed[j] + alpha * closed[j + 1]
                out.append(p)
        return np.asarray(out, dtype=np.float32)

    @staticmethod
    def overlay(
        frame_bgr: np.ndarray,
        result: Optional[SegmentationResult],
        color: tuple[int, int, int] = (0, 255, 0),
        point_color: tuple[int, int, int] = (255, 0, 255),
        alpha: float = 0.28,
        show_text: bool = True,
        text_prefix: str = "",
        label: str | None = None,
    ) -> np.ndarray:
        out = frame_bgr.copy()
        if result is None:
            return out

        mask = result.mask
        fill = np.zeros_like(out)
        fill[:] = color
        idx = mask > 0
        out[idx] = cv2.addWeighted(out, 1.0 - alpha, fill, alpha, 0)[idx]

        contour = np.round(result.contour).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(out, [contour], True, color, 2, cv2.LINE_AA)

        for p in result.sampled:
            xy = tuple(np.round(p).astype(int))
            cv2.circle(out, xy, 3, point_color, -1, cv2.LINE_AA)

        if label is not None:
            text_prefix = str(label)

        if show_text:
            area = int(np.count_nonzero(mask))
            label = f"{text_prefix}{result.confidence:.2f} area={area}"
            cv2.putText(out, label, (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(out, label, (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 1, cv2.LINE_AA)

        return out  
