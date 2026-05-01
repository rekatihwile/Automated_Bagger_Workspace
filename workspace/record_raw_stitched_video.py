from __future__ import annotations

import argparse
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import config
from camera import Camera


WINDOW_NAME = "Raw Stitched Stereo Recorder"


def _default_output_path() -> Path:
    out_dir = config.CALIBRATION_ROOT / "raw_stitched_recordings"
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return out_dir / f"raw_stitched_{stamp}.mp4"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record raw stitched stereo frames (no overlays).")
    parser.add_argument("--out", type=Path, default=None, help="Output video path (.mp4).")
    parser.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="Stop automatically after N seconds. 0 means record until Q/Esc.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=0.0,
        help="Output video fps override. <= 0 means auto (measured from capture timing).",
    )
    parser.add_argument("--no-preview", action="store_true", help="Disable live preview window.")
    parser.add_argument("--codec", type=str, default="mp4v", help="OpenCV FOURCC (default: mp4v).")
    return parser.parse_args()


def _estimate_capture_fps(timestamps_s: list[float], fallback_fps: float) -> float:
    if len(timestamps_s) < 2:
        return max(1.0, float(fallback_fps))

    ts = np.asarray(timestamps_s, dtype=np.float64)
    dt = np.diff(ts)
    dt = dt[np.isfinite(dt) & (dt > 1e-6)]
    if len(dt) == 0:
        return max(1.0, float(fallback_fps))

    fps_med = 1.0 / float(np.median(dt))
    if not np.isfinite(fps_med) or fps_med <= 0.0:
        return max(1.0, float(fallback_fps))
    return float(np.clip(fps_med, 1.0, 240.0))


def _encode_buffered_video(
    out_path: Path,
    frames: list[np.ndarray],
    timestamps_s: list[float],
    codec: str,
    fps_override: float,
    fallback_fps: float,
) -> tuple[float, float]:
    if not frames:
        raise RuntimeError("No frames were captured.")

    captured_fps = _estimate_capture_fps(timestamps_s, fallback_fps=max(1.0, float(fallback_fps)))
    output_fps = float(fps_override) if float(fps_override) > 0 else captured_fps
    output_fps = float(np.clip(output_fps, 1.0, 240.0))

    h, w = frames[0].shape[:2]
    h_even = h - (h % 2)
    w_even = w - (w % 2)

    fourcc = cv2.VideoWriter_fourcc(*str(codec)[:4])
    writer = cv2.VideoWriter(str(out_path), fourcc, output_fps, (w_even, h_even))
    if not writer.isOpened():
        raise RuntimeError(f"Could not open VideoWriter for: {out_path}")

    for frame in frames:
        writer.write(frame[:h_even, :w_even])
    writer.release()
    return captured_fps, output_fps


def main() -> None:
    args = _parse_args()
    out_path = Path(args.out) if args.out else _default_output_path()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    frame_count = 0
    frame_buffer: list[np.ndarray] = []
    frame_times_s: list[float] = []
    t0 = 0.0

    with Camera(width=config.STREAM_WIDTH, height=config.STREAM_HEIGHT, name="HBVCAM Raw") as cam:
        for _ in range(int(config.WARMUP_FRAMES)):
            cam.read()

        ok, frame = cam.read()
        first_ts = time.perf_counter()
        if not ok or frame is None:
            raise RuntimeError("Could not read from camera.")
        t0 = first_ts

        h, w = frame.shape[:2]
        h_even = h - (h % 2)
        w_even = w - (w % 2)

        if not args.no_preview:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW_NAME, max(640, int(w_even * 0.8)), max(360, int(h_even * 0.8)))

        print(f"[REC] Capturing raw stitched frames to memory: {out_path}")
        print("[REC] Controls: Q/Esc to stop")

        try:
            frame_ts = first_ts
            while True:
                if frame is None:
                    break

                raw = frame[:h_even, :w_even]
                frame_buffer.append(raw.copy())
                frame_times_s.append(float(frame_ts))
                frame_count += 1

                if not args.no_preview:
                    preview = cam.resize_for_display(raw)
                    cv2.imshow(WINDOW_NAME, preview)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break

                if args.seconds > 0:
                    elapsed = frame_ts - t0
                    if elapsed >= float(args.seconds):
                        break

                ok, frame = cam.read()
                frame_ts = time.perf_counter()
                if not ok:
                    break
        finally:
            if not args.no_preview:
                cv2.destroyWindow(WINDOW_NAME)

    if frame_times_s:
        elapsed = max(1e-6, frame_times_s[-1] - frame_times_s[0])
    else:
        elapsed = max(1e-6, time.perf_counter() - t0)

    print("[REC] Encoding buffered frames...")
    captured_fps, output_fps = _encode_buffered_video(
        out_path=out_path,
        frames=frame_buffer,
        timestamps_s=frame_times_s,
        codec=str(args.codec),
        fps_override=float(args.fps),
        fallback_fps=float(config.CAMERA_FPS),
    )
    print(
        f"[REC] Saved {frame_count} frame(s) in {elapsed:.2f}s "
        f"(captured ~{captured_fps:.2f} fps, encoded {output_fps:.2f} fps)"
    )


if __name__ == "__main__":
    main()
