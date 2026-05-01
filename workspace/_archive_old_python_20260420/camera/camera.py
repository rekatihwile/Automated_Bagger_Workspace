"""
Low-level camera wrapper: capture, resolution, scaling, window display,
and pixel coordinate mapping back to original resolution.
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
from config import NATIVE_WIDTH, NATIVE_HEIGHT, CAPTURE_DOWNSCALE, DISPLAY_SCALE

# UVC property label → cv2 constant (used by apply_settings)
_PROP_MAP = {
    "Brightness":    cv2.CAP_PROP_BRIGHTNESS,
    "Contrast":      cv2.CAP_PROP_CONTRAST,
    "Saturation":    cv2.CAP_PROP_SATURATION,
    "Hue":           cv2.CAP_PROP_HUE,
    "Sharpness":     cv2.CAP_PROP_SHARPNESS,
    "Gamma":         cv2.CAP_PROP_GAMMA,
    "Gain":          cv2.CAP_PROP_GAIN,
    "Exposure":      cv2.CAP_PROP_EXPOSURE,
    "Auto Exposure": cv2.CAP_PROP_AUTO_EXPOSURE,
}


class Camera:
    def __init__(self, index, name="Camera", downscale=CAPTURE_DOWNSCALE):
        self.index = index
        self.name  = name
        self.display_scale = DISPLAY_SCALE

        self._cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera at index {index}")

        self._set_capture_downscale(downscale)

    # ------------------------------------------------------------------
    # Resolution helpers
    # ------------------------------------------------------------------

    def _set_capture_downscale(self, downscale):
        """Compute width/height from native dims * downscale and apply."""
        w = int(NATIVE_WIDTH  * downscale)
        h = int(NATIVE_HEIGHT * downscale)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.capture_width  = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.capture_height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._downscale = downscale

    def set_downscale(self, downscale):
        """Change capture resolution at runtime (aspect ratio preserved)."""
        self._set_capture_downscale(downscale)
        print(f"[{self.name}] capture resolution → {self.capture_width}x{self.capture_height}")

    def set_resolution(self, width, height):
        """Set an explicit capture resolution."""
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture_width  = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.capture_height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def set_display_scale(self, scale):
        self.display_scale = scale

    @property
    def display_size(self):
        return (
            int(self.capture_width  * self.display_scale),
            int(self.capture_height * self.display_scale),
        )

    # ------------------------------------------------------------------
    # UVC settings
    # ------------------------------------------------------------------

    def apply_settings(self, settings: dict):
        """Apply a saved settings dict (from camera_tuner) to this capture."""
        for label, val in settings.items():
            prop = _PROP_MAP.get(label)
            if prop is None:
                continue
            actual = val - 13 if label == "Exposure" else val
            self._cap.set(prop, actual)

    # ------------------------------------------------------------------
    # Frame capture
    # ------------------------------------------------------------------

    def read(self):
        """Return (success, frame) at current capture resolution."""
        return self._cap.read()

    def read_scaled(self):
        """Return (success, frame) downscaled for display."""
        ret, frame = self._cap.read()
        if not ret:
            return False, None
        if self.display_scale != 1.0:
            dw, dh = self.display_size
            frame = cv2.resize(frame, (dw, dh))
        return True, frame

    def read_stereo(self, display=True):
        """
        Split the side-by-side frame into left/right.
        If display=True, applies display_scale. If False, returns full capture-res halves.
        Returns (success, left, right).
        """
        ret, frame = self._cap.read()
        if not ret:
            return False, None, None
        mid   = self.capture_width // 2
        left  = frame[:, :mid]
        right = frame[:, mid:]
        if display and self.display_scale != 1.0:
            w = int(mid * self.display_scale)
            h = int(self.capture_height * self.display_scale)
            left  = cv2.resize(left,  (w, h))
            right = cv2.resize(right, (w, h))
        return True, left, right

    # ------------------------------------------------------------------
    # Pixel coordinate mapping
    # ------------------------------------------------------------------

    def display_to_capture(self, x, y):
        return (int(x / self.display_scale), int(y / self.display_scale))

    def capture_to_display(self, x, y):
        return (int(x * self.display_scale), int(y * self.display_scale))

    # ------------------------------------------------------------------
    # Window management
    # ------------------------------------------------------------------

    def show(self, frame):
        cv2.imshow(self.name, frame)

    def create_window(self):
        cv2.namedWindow(self.name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.name, *self.display_size)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def release(self):
        self._cap.release()
        cv2.destroyWindow(self.name)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.release()
