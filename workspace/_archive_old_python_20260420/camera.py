"""
Low-level camera wrapper: capture, resolution, scaling, window display,
and pixel coordinate mapping back to original resolution.
"""

import cv2
from config import CAPTURE_WIDTH, CAPTURE_HEIGHT, DISPLAY_SCALE


class Camera:
    def __init__(self, index, name="Camera", width=CAPTURE_WIDTH, height=CAPTURE_HEIGHT):
        self.index = index
        self.name = name
        self.display_scale = DISPLAY_SCALE

        self._cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera at index {index}")

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # Read back actual values the device settled on
        self.capture_width  = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.capture_height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # ------------------------------------------------------------------
    # Frame capture
    # ------------------------------------------------------------------

    def read(self):
        """Return (success, frame) at native capture resolution."""
        return self._cap.read()

    def read_scaled(self):
        """Return (success, frame) scaled by display_scale."""
        ret, frame = self._cap.read()
        if not ret:
            return False, None
        if self.display_scale != 1.0:
            w = int(self.capture_width  * self.display_scale)
            h = int(self.capture_height * self.display_scale)
            frame = cv2.resize(frame, (w, h))
        return True, frame

    def read_stereo(self):
        """Return (success, left_frame, right_frame) by splitting the wide side-by-side frame."""
        ret, frame = self._cap.read()
        if not ret:
            return False, None, None
        mid = self.capture_width // 2
        left  = frame[:, :mid]
        right = frame[:, mid:]
        if self.display_scale != 1.0:
            w = int(mid * self.display_scale)
            h = int(self.capture_height * self.display_scale)
            left  = cv2.resize(left,  (w, h))
            right = cv2.resize(right, (w, h))
        return True, left, right

    # ------------------------------------------------------------------
    # Resolution / scale helpers
    # ------------------------------------------------------------------

    def set_resolution(self, width, height):
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture_width  = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.capture_height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def set_scale(self, scale):
        self.display_scale = scale

    @property
    def display_size(self):
        return (
            int(self.capture_width  * self.display_scale),
            int(self.capture_height * self.display_scale),
        )

    # ------------------------------------------------------------------
    # Pixel coordinate mapping
    # ------------------------------------------------------------------

    def display_to_capture(self, x, y):
        """Map a pixel in the scaled display back to the original capture frame."""
        return (int(x / self.display_scale), int(y / self.display_scale))

    def capture_to_display(self, x, y):
        """Map a pixel in the capture frame to the scaled display."""
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
