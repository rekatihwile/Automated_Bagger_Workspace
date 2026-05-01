import cv2
import os
from pathlib import Path
from datetime import datetime

import sys
from pathlib import Path

# Add workspace root to Python path
ROOT = Path(__file__).resolve().parents[1]  # Weeder_Workspace
sys.path.append(str(ROOT))

from hardware.cameras import StereoCameras


SAVE_DIR = Path("Weeder_Workspace/Triangulation/calib_pairs")
SAVE_DIR.mkdir(parents=True, exist_ok=True)

WARMUP_FRAMES = 10
DISPLAY_SCALE = 0.6  # adjust (0.5–0.8 usually good)

def get_start_index(save_dir: Path):
    existing = list(save_dir.glob("left_*.png"))
    
    if not existing:
        return 0

    indices = []
    for f in existing:
        try:
            idx = int(f.stem.split("_")[1])
            indices.append(idx)
        except:
            continue

    return max(indices) + 1 if indices else 0
def main():
    cams = StereoCameras()
    cams.open()
    print("Warming up cameras...")
    for _ in range(WARMUP_FRAMES):
        left, right = cams.read_pair()

    print("\nPress SPACE to capture pair")
    print("Press Q to quit\n")

    idx = get_start_index(SAVE_DIR)

    while True:
        left, right = cams.read_pair()

        display = cv2.hconcat([left, right])

# Resize ONLY for display
        display_small = cv2.resize(
            display,
            None,
            fx=DISPLAY_SCALE,
            fy=DISPLAY_SCALE,
            interpolation=cv2.INTER_AREA
        )
        cv2.imshow("Stereo Capture", display_small)

        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):
            left_path = SAVE_DIR / f"left_{idx:04d}.png"
            right_path = SAVE_DIR / f"right_{idx:04d}.png"

            cv2.imwrite(str(left_path), left)
            cv2.imwrite(str(right_path), right)

            print(f"Saved pair {idx}")
            idx += 1

        elif key == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()