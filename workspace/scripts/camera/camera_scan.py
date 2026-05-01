"""
Find the OpenCV camera index for the single-USB HBVCAM stereo stream.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import cv2

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import config
from camera.camera import BACKENDS


def looks_stereo(width: int, height: int) -> bool:
    return height > 0 and width / height >= config.STEREO_MIN_ASPECT_RATIO and width % 2 == 0


def mode_note(width: int, height: int) -> str:
    if looks_stereo(width, height):
        return "stereo-shaped"
    if height > 0 and 1.6 <= width / height <= 1.9:
        return "mono-shaped, do not split"
    return "normal-shaped"


def _should_print(label: str) -> bool:
    return label in {"stereo-shaped", "normal-shaped"}


def _open_and_probe(index: int, backend: int, requested_w: int, requested_h: int) -> tuple[bool, int, int]:
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        return False, 0, 0

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config.CAMERA_FOURCC))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, requested_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, requested_h)
    cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)

    ok, frame = cap.read()
    if ok and frame is not None:
        actual_h, actual_w = frame.shape[:2]
    else:
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.release()
    return ok, actual_w, actual_h


def brute_force_modes(
    index: int,
    backend: int,
    widths: list[int],
    heights: list[int],
    max_requests: int | None = None,
) -> dict[tuple[int, int], set[tuple[int, int]]]:
    """
    Returns mapping: (requested_w, requested_h) -> {(actual_w, actual_h), ...}
    """
    results: dict[tuple[int, int], set[tuple[int, int]]] = {}
    count = 0
    for w in widths:
        for h in heights:
            if max_requests is not None and count >= max_requests:
                return results
            count += 1
            ok, aw, ah = _open_and_probe(index, backend, w, h)
            if not ok:
                continue
            results.setdefault((w, h), set()).add((aw, ah))
    return results


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bruteforce", action="store_true", help="probe lots of width/height requests")
    parser.add_argument("--max-requests", type=int, default=500, help="limit bruteforce probes for speed")
    args = parser.parse_args()

    backend = BACKENDS.get(config.CAMERA_BACKEND.upper(), cv2.CAP_ANY)
    found: list[int] = []
    stereo_like: list[int] = []

    print("Scanning camera indices 1-10...")
    for index in range(1, 11):
        first_cap = cv2.VideoCapture(index, backend)
        if not first_cap.isOpened():
            print(f"  {index}: unavailable")
            continue
        first_cap.release()

        found.append(index)
        print(f"  {index}: opened")

        current_mode_is_stereo = False
        for requested_w, requested_h in config.STEREO_STREAM_MODES:
            ok, width, height = _open_and_probe(index, backend, requested_w, requested_h)

            stereo = looks_stereo(width, height)
            if stereo and requested_w == config.STREAM_WIDTH and requested_h == config.STREAM_HEIGHT:
                current_mode_is_stereo = True
            label = mode_note(width, height)
            if _should_print(label):
                print(f"      request {requested_w}x{requested_h}: read={ok}, got {width}x{height}, {label}")

        if args.bruteforce:
            # Coarse grids only. The driver typically exposes discrete modes; this is about discovery.
            widths = list(range(320, 4001, 160))
            heights = list(range(180, 1201, 60))
            probed = brute_force_modes(index, backend, widths, heights, max_requests=args.max_requests)

            actual_modes: set[tuple[int, int]] = set()
            for actuals in probed.values():
                actual_modes |= actuals

            actual_sorted = sorted(actual_modes, key=lambda x: (x[1], x[0]))
            print(f"      bruteforce: {len(probed)} requests, {len(actual_sorted)} unique actual mode(s)")
            for aw, ah in actual_sorted:
                label = mode_note(aw, ah)
                if _should_print(label):
                    print(f"        actual {aw}x{ah}: {label}")

        if current_mode_is_stereo:
            stereo_like.append(index)

    if stereo_like:
        chosen = stereo_like[0]
        config.save_camera_index(chosen)
        print(f"\nSaved camera index {chosen} to {config.CAMERA_IDS_PATH}")
        print(f"Configured stream mode is stereo-shaped: {config.STREAM_WIDTH}x{config.STREAM_HEIGHT}")
    elif found:
        print("\nNo stereo-shaped stream was found after requesting the configured resolution.")
        print("Pick a request above that says stereo-shaped, then set STREAM_WIDTH/STREAM_HEIGHT in config.py.")
    else:
        print("\nNo cameras opened.")


if __name__ == "__main__":
    main()
