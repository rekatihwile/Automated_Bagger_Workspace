import cv2
import json
import time
import random
from pathlib import Path
from datetime import datetime
import sys


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

# =========================
# USER SETTINGS
# =========================
CAMERA_INDEX = 1          # change if needed
USE_DSHOW = True          # Windows: usually best for UVC webcams
START_FOURCC = "MJPG"     # "MJPG" or "YUY2"
START_FPS = 30

# Your listed stitched stereo modes
RESOLUTION_MODES = [
    (1280, 480),
    (2160, 1080),
    (2560, 720),
    (3040, 1520),
    (3840, 1080),
]

START_MODE_INDEX = 0

SAVE_DIR = WORKSPACE_ROOT.parent
SAVE_DIR.mkdir(parents=True, exist_ok=True)


# =========================
# PROPERTY MAP
# =========================
PROP_MAP = {
    "FRAME_WIDTH": cv2.CAP_PROP_FRAME_WIDTH,
    "FRAME_HEIGHT": cv2.CAP_PROP_FRAME_HEIGHT,
    "FPS": cv2.CAP_PROP_FPS,
    "FOURCC": cv2.CAP_PROP_FOURCC,
    "FORMAT": cv2.CAP_PROP_FORMAT,
    "MODE": cv2.CAP_PROP_MODE,
    "BRIGHTNESS": cv2.CAP_PROP_BRIGHTNESS,
    "CONTRAST": cv2.CAP_PROP_CONTRAST,
    "SATURATION": cv2.CAP_PROP_SATURATION,
    "HUE": cv2.CAP_PROP_HUE,
    "GAIN": cv2.CAP_PROP_GAIN,
    "EXPOSURE": cv2.CAP_PROP_EXPOSURE,
    "AUTO_EXPOSURE": cv2.CAP_PROP_AUTO_EXPOSURE,
    "SHARPNESS": getattr(cv2, "CAP_PROP_SHARPNESS", -1),
    "GAMMA": getattr(cv2, "CAP_PROP_GAMMA", -1),
    "TEMPERATURE": getattr(cv2, "CAP_PROP_TEMPERATURE", -1),
    "ZOOM": getattr(cv2, "CAP_PROP_ZOOM", -1),
    "FOCUS": getattr(cv2, "CAP_PROP_FOCUS", -1),
    "AUTOFOCUS": getattr(cv2, "CAP_PROP_AUTOFOCUS", -1),
    "BUFFERSIZE": getattr(cv2, "CAP_PROP_BUFFERSIZE", -1),
    "BACKEND": getattr(cv2, "CAP_PROP_BACKEND", -1),
}


def fourcc_to_string(v: float) -> str:
    i = int(v)
    return "".join([chr((i >> 8 * k) & 0xFF) for k in range(4)])


def string_to_fourcc(code: str) -> int:
    return cv2.VideoWriter_fourcc(*code)


def make_filename(prefix: str = "camera_test_properties") -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    rand = random.randint(1000, 9999)
    return SAVE_DIR / f"{prefix}_{stamp}_{rand}.json"


def safe_get(cap, prop_id):
    if prop_id == -1:
        return None
    try:
        return cap.get(prop_id)
    except Exception:
        return None


def safe_set(cap, prop_id, value):
    if prop_id == -1:
        return False
    try:
        return cap.set(prop_id, value)
    except Exception:
        return False


def collect_properties(cap, requested_mode=None, requested_fourcc=None, measured_fps=None):
    data = {}
    for name, prop_id in PROP_MAP.items():
        value = safe_get(cap, prop_id)
        if name == "FOURCC" and value is not None:
            data[name] = {
                "raw": value,
                "decoded": fourcc_to_string(value)
            }
        else:
            data[name] = value

    data["timestamp"] = datetime.now().isoformat()
    data["requested_mode"] = requested_mode
    data["requested_fourcc"] = requested_fourcc
    data["measured_preview_fps"] = measured_fps
    return data


def save_properties_json(data):
    path = make_filename()
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    print(f"[SAVED] {path}")
    return path


def open_camera(index, width, height, fourcc="MJPG", fps=30, use_dshow=True):
    backend = cv2.CAP_DSHOW if use_dshow else cv2.CAP_ANY
    cap = cv2.VideoCapture(index, backend)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {index}")

    # Set codec first
    cap.set(cv2.CAP_PROP_FOURCC, string_to_fourcc(fourcc))
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # Small warmup so returned values settle a bit
    for _ in range(10):
        cap.read()

    return cap


def draw_overlay(frame, lines):
    y = 30
    for line in lines:
        cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        y += 28
    return frame


def print_controls():
    print("\n=== CONTROLS ===")
    print("q  : quit")
    print("p  : print current properties")
    print("s  : save current properties to JSON")
    print("r  : cycle resolution mode")
    print("m  : toggle MJPG / YUY2")
    print("a  : set auto exposure (Windows UVC often uses 3)")
    print("n  : set manual exposure (Windows UVC often uses 1)")
    print("[  : decrease exposure")
    print("]  : increase exposure")
    print("-  : decrease gain")
    print("=  : increase gain")
    print("0  : set FPS request back to 30")
    print("================\n")


def main():
    mode_index = START_MODE_INDEX
    current_fourcc = START_FOURCC
    req_width, req_height = RESOLUTION_MODES[mode_index]

    cap = open_camera(
        CAMERA_INDEX,
        req_width,
        req_height,
        fourcc=current_fourcc,
        fps=START_FPS,
        use_dshow=USE_DSHOW
    )

    print_controls()

    frame_counter = 0
    fps_timer = time.time()
    measured_fps = 0.0

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("[WARN] Failed to read frame.")
            continue

        frame_counter += 1
        now = time.time()
        dt = now - fps_timer
        if dt >= 1.0:
            measured_fps = frame_counter / dt
            frame_counter = 0
            fps_timer = now

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        actual_fourcc = fourcc_to_string(cap.get(cv2.CAP_PROP_FOURCC))
        exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
        auto_exposure = cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
        gain = cap.get(cv2.CAP_PROP_GAIN)

        overlay_lines = [
            f"Requested: {req_width}x{req_height} @ {START_FPS}  FOURCC={current_fourcc}",
            f"Actual:    {actual_w}x{actual_h}  OpenCV_FPS={actual_fps:.2f}  Measured_FPS={measured_fps:.2f}",
            f"FOURCC: {actual_fourcc}",
            f"Exposure: {exposure:.3f}   AutoExposure: {auto_exposure:.3f}   Gain: {gain:.3f}",
            "Keys: q quit | p print | s save json | r res | m codec | a auto exp | n manual exp | [ ] exposure | - = gain",
        ]

        preview = frame.copy()
        preview = draw_overlay(preview, overlay_lines)
        cv2.imshow("Camera Config Inspector", preview)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        elif key == ord("p"):
            props = collect_properties(
                cap,
                requested_mode={"width": req_width, "height": req_height},
                requested_fourcc=current_fourcc,
                measured_fps=measured_fps,
            )
            print(json.dumps(props, indent=2))

        elif key == ord("s"):
            props = collect_properties(
                cap,
                requested_mode={"width": req_width, "height": req_height},
                requested_fourcc=current_fourcc,
                measured_fps=measured_fps,
            )
            save_properties_json(props)

        elif key == ord("r"):
            mode_index = (mode_index + 1) % len(RESOLUTION_MODES)
            req_width, req_height = RESOLUTION_MODES[mode_index]
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, req_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, req_height)
            time.sleep(0.3)
            print(f"[MODE] Requested resolution -> {req_width}x{req_height}")

        elif key == ord("m"):
            current_fourcc = "YUY2" if current_fourcc == "MJPG" else "MJPG"
            cap.set(cv2.CAP_PROP_FOURCC, string_to_fourcc(current_fourcc))
            time.sleep(0.3)
            print(f"[MODE] Requested FOURCC -> {current_fourcc}")

        elif key == ord("a"):
            # Windows UVC convention is often 3 for auto
            ok = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
            print(f"[SET] Auto exposure -> 3  success={ok}")

        elif key == ord("n"):
            # Windows UVC convention is often 1 for manual
            ok = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            print(f"[SET] Manual exposure -> 1  success={ok}")

        elif key == ord("["):
            current = cap.get(cv2.CAP_PROP_EXPOSURE)
            # many UVC drivers use more negative = shorter exposure
            new_val = current - 1
            ok = cap.set(cv2.CAP_PROP_EXPOSURE, new_val)
            print(f"[SET] Exposure {current} -> {new_val}  success={ok}")

        elif key == ord("]"):
            current = cap.get(cv2.CAP_PROP_EXPOSURE)
            new_val = current + 1
            ok = cap.set(cv2.CAP_PROP_EXPOSURE, new_val)
            print(f"[SET] Exposure {current} -> {new_val}  success={ok}")

        elif key == ord("-"):
            current = cap.get(cv2.CAP_PROP_GAIN)
            new_val = current - 1
            ok = cap.set(cv2.CAP_PROP_GAIN, new_val)
            print(f"[SET] Gain {current} -> {new_val}  success={ok}")

        elif key == ord("="):
            current = cap.get(cv2.CAP_PROP_GAIN)
            new_val = current + 1
            ok = cap.set(cv2.CAP_PROP_GAIN, new_val)
            print(f"[SET] Gain {current} -> {new_val}  success={ok}")

        elif key == ord("0"):
            ok = cap.set(cv2.CAP_PROP_FPS, 30)
            print(f"[SET] FPS request -> 30  success={ok}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
