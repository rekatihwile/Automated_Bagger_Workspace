"""
Camera pipeline helpers for the single-USB HBVCAM stereo stream.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import cv2
import numpy as np

import config


BACKENDS = {
    "ANY": cv2.CAP_ANY,
    "DSHOW": cv2.CAP_DSHOW,
    "MSMF": cv2.CAP_MSMF,
}

UVC_PROPERTIES = {
    "Brightness": cv2.CAP_PROP_BRIGHTNESS,
    "Contrast": cv2.CAP_PROP_CONTRAST,
    "Saturation": cv2.CAP_PROP_SATURATION,
    "Hue": cv2.CAP_PROP_HUE,
    "Sharpness": cv2.CAP_PROP_SHARPNESS,
    "Gamma": cv2.CAP_PROP_GAMMA,
    "Gain": cv2.CAP_PROP_GAIN,
    "Exposure": cv2.CAP_PROP_EXPOSURE,
    "Auto Exposure": cv2.CAP_PROP_AUTO_EXPOSURE,
}


@dataclass(frozen=True)
class FrameSize:
    width: int
    height: int


class Camera:
    """Thin OpenCV wrapper for one physical UVC camera stream."""

    def __init__(
        self,
        index: int | None = None,
        name: str = "Camera",
        width: int = config.STREAM_WIDTH,
        height: int = config.STREAM_HEIGHT,
        backend: str = config.CAMERA_BACKEND,
        fourcc: str = config.CAMERA_FOURCC,
        fps: int = config.CAMERA_FPS,
        settings: dict[str, float | int] | None = None,
    ) -> None:
        self.index = config.load_camera_index() if index is None else int(index)
        self.name = name
        self.requested_size = FrameSize(width, height)
        self.backend_name = backend
        self.fourcc = fourcc
        self.fps = fps
        self._cap: cv2.VideoCapture | None = None
        self.capture_width = 0
        self.capture_height = 0
        self.open(settings=settings)

    def open(self, settings: dict[str, float | int] | None = None) -> None:
        backend_id = BACKENDS.get(self.backend_name.upper(), cv2.CAP_ANY)
        self._cap = cv2.VideoCapture(self.index, backend_id)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera index {self.index}. Run camera_scan.py.")

        if self.fourcc:
            code = cv2.VideoWriter_fourcc(*self.fourcc)
            self._cap.set(cv2.CAP_PROP_FOURCC, code)
        if self.fps:
            self._cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.set_resolution(self.requested_size.width, self.requested_size.height)
        self.apply_settings(config.load_camera_settings())
        if settings:
            self.apply_settings(settings)

    def set_resolution(self, width: int, height: int) -> None:
        self._require_open()
        assert self._cap is not None
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        self.capture_width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.capture_height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def apply_settings(self, settings: dict[str, float | int]) -> None:
        self._require_open()
        assert self._cap is not None
        for label, value in settings.items():
            prop = UVC_PROPERTIES.get(label)
            if prop is not None and value is not None:
                # Some drivers expect specific conventions for exposure.
                if label == "Exposure":
                    v = float(value)
                    # Accept either a "trackbar style" 0..12 or a direct UVC negative value.
                    if 0 <= v <= 13:
                        v = v - 13
                    self._cap.set(prop, v)
                elif label == "Auto Exposure" and isinstance(value, str):
                    mode = value.strip().lower()
                    if mode == "auto":
                        self._cap.set(prop, float(config.AUTO_EXPOSURE_VALUE_AUTO))
                    elif mode == "manual":
                        self._cap.set(prop, float(config.AUTO_EXPOSURE_VALUE_MANUAL))
                    else:
                        self._cap.set(prop, float(value))
                else:
                    self._cap.set(prop, float(value))

    def read(self) -> tuple[bool, np.ndarray | None]:
        self._require_open()
        assert self._cap is not None
        return self._cap.read()

    def resize_for_display(self, frame: np.ndarray, scale: float = config.DISPLAY_SCALE) -> np.ndarray:
        if scale == 1.0:
            return frame
        return cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def _require_open(self) -> None:
        if self._cap is None:
            raise RuntimeError(f"{self.name} is not open.")

    def __enter__(self) -> "Camera":
        return self

    def __exit__(self, *_: object) -> None:
        self.release()


class StereoCamera:
    """
    One USB stereo camera that delivers left/right images in a side-by-side frame.
    """

    def __init__(self, index: int | None = None, name: str = "HBVCAM Stereo") -> None:
        self.camera = Camera(index=index, name=name)
        self.name = name

    @property
    def actual_size(self) -> FrameSize:
        return FrameSize(self.camera.capture_width, self.camera.capture_height)

    @property
    def eye_size(self) -> FrameSize:
        return FrameSize(self.camera.capture_width // 2, self.camera.capture_height)

    @property
    def is_stereo_shape(self) -> bool:
        if self.camera.capture_height <= 0:
            return False
        aspect = self.camera.capture_width / self.camera.capture_height
        return aspect >= config.STEREO_MIN_ASPECT_RATIO and self.camera.capture_width % 2 == 0

    def require_stereo_shape(self) -> None:
        if self.is_stereo_shape:
            return
        actual = self.actual_size
        requested = self.camera.requested_size
        if actual == requested:
            return
        raise RuntimeError(
            "The camera did not return a side-by-side stereo frame.\n"
            f"Requested: {requested.width}x{requested.height}\n"
            f"Actual:    {actual.width}x{actual.height}\n"
            "This usually means the driver rejected the requested stereo mode "
            "and fell back to a normal single-view mode. Run python camera_scan.py "
            "and set STREAM_WIDTH/STREAM_HEIGHT in config.py to a stereo-shaped mode."
        )

    def warmup(self, frames: int = config.WARMUP_FRAMES) -> None:
        for _ in range(frames):
            self.camera.read()

    def read_pair(self) -> tuple[bool, np.ndarray | None, np.ndarray | None]:
        ok, frame = self.camera.read()
        if not ok or frame is None:
            return False, None, None
        self.require_stereo_shape()
        left, right = self.split(frame)
        if self._preprocess_targets_saved():
            left = self._preprocess(left)
            right = self._preprocess(right)
        return True, left, right

    def split(self, frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        mid = frame.shape[1] // 2
        return frame[:, :mid].copy(), frame[:, mid:].copy()

    def create_preview_window(self, window_name: str) -> None:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        eye = self.eye_size
        width = max(640, int((eye.width * 2) * config.DISPLAY_SCALE))
        height = max(360, int(eye.height * config.DISPLAY_SCALE))
        cv2.resizeWindow(window_name, width, height)

    def render_preview(
        self,
        left: np.ndarray,
        right: np.ndarray,
        lines: Iterable[str] = (),
        left_ok: bool | None = None,
        right_ok: bool | None = None,
    ) -> np.ndarray:
        if self._preprocess_targets_preview():
            left = self._preprocess(left)
            right = self._preprocess(right)
        left_view = self.camera.resize_for_display(left)
        right_view = self.camera.resize_for_display(right)
        self._label(left_view, "LEFT", left_ok)
        self._label(right_view, "RIGHT", right_ok)
        divider = np.full((left_view.shape[0], 6, 3), 32, dtype=np.uint8)
        preview = cv2.hconcat([left_view, divider, right_view])
        self._overlay(preview, list(lines))
        return preview

    @staticmethod
    def _preprocess_targets_preview() -> bool:
        apply_to = str(config.PREPROCESS.get("apply_to", "preview")).strip().lower()
        return apply_to in {"preview", "both"}

    @staticmethod
    def _preprocess_targets_saved() -> bool:
        apply_to = str(config.PREPROCESS.get("apply_to", "preview")).strip().lower()
        return apply_to in {"saved", "both"}

    @staticmethod
    def _preprocess(frame_bgr: np.ndarray) -> np.ndarray:
        """
        Lightweight preprocessing for visibility / robustness.
        Defaults are effectively no-op unless config.PREPROCESS is changed.
        """
        out = frame_bgr

        gamma = float(config.PREPROCESS.get("gamma", 1.0))
        if gamma > 0 and abs(gamma - 1.0) > 1e-3:
            inv = 1.0 / gamma
            table = (np.linspace(0, 1, 256) ** inv * 255.0).astype(np.uint8)
            out = cv2.LUT(out, table)

        if bool(config.PREPROCESS.get("clahe", False)):
            clip = float(config.PREPROCESS.get("clahe_clip", 2.0))
            grid = config.PREPROCESS.get("clahe_grid", (8, 8))
            try:
                grid_x, grid_y = int(grid[0]), int(grid[1])
            except Exception:
                grid_x, grid_y = 8, 8

            ycrcb = cv2.cvtColor(out, cv2.COLOR_BGR2YCrCb)
            y, cr, cb = cv2.split(ycrcb)
            clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(grid_x, grid_y))
            y = clahe.apply(y)
            out = cv2.cvtColor(cv2.merge([y, cr, cb]), cv2.COLOR_YCrCb2BGR)

        ksize = int(config.PREPROCESS.get("blur_ksize", 0))
        if ksize and ksize >= 3:
            if ksize % 2 == 0:
                ksize += 1
            out = cv2.GaussianBlur(out, (ksize, ksize), 0)

        sharpen = float(config.PREPROCESS.get("sharpen", 0.0))
        if sharpen > 0:
            # Unsharp mask.
            blurred = cv2.GaussianBlur(out, (0, 0), 1.0)
            out = cv2.addWeighted(out, 1.0 + sharpen, blurred, -sharpen, 0)

        return out

    def release(self) -> None:
        self.camera.release()

    def __enter__(self) -> "StereoCamera":
        return self

    def __exit__(self, *_: object) -> None:
        self.release()

    @staticmethod
    def _label(frame: np.ndarray, label: str, ok: bool | None) -> None:
        if ok is not None:
            color = (40, 210, 40) if ok else (40, 40, 230)
            cv2.rectangle(frame, (0, 0), (frame.shape[1] - 1, frame.shape[0] - 1), color, 4)
        cv2.putText(frame, label, (14, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, label, (14, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)

    @staticmethod
    def _overlay(frame: np.ndarray, lines: list[str]) -> None:
        for i, line in enumerate(lines):
            y = frame.shape[0] - 22 - (len(lines) - 1 - i) * 30
            cv2.putText(frame, line, (14, y), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(frame, line, (14, y), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA)
