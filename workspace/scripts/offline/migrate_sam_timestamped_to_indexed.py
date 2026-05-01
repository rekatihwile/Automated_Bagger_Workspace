"""
Migrate timestamped SAM snapshot files into the indexed pair dataset.

Scans:
  calibration/sam_live_snapshot_tests/

Groups files by timestamp stem and copies each complete group into:
  calibration/sam_indexed_pairs/pair_XXXXXX/

Rules
-----
* Requires *_left_rect.png AND *_right_rect.png to migrate a group.
  Groups missing either are skipped with a warning.
* Missing masks are allowed — metadata records has_left_mask / has_right_mask.
* Existing pair_XXXXXX folders are never overwritten.
* Original timestamped files are NOT deleted.

Run:
  python workspace/scripts/offline/migrate_sam_timestamped_to_indexed.py
"""

from __future__ import annotations

import json
import shutil
import sys
from datetime import datetime
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import config

SNAP_DIR  = config.CALIBRATION_ROOT / "sam_live_snapshot_tests"
PAIRS_DIR = config.CALIBRATION_ROOT / "sam_indexed_pairs"

# Map: file suffix (after timestamp stem) → standardized filename inside pair folder
SUFFIX_MAP: dict[str, str] = {
    "_left_rect.png":                   "left_rect.png",
    "_right_rect.png":                  "right_rect.png",
    "_left_mask.png":                   "left_mask.png",
    "_right_mask.png":                  "right_mask.png",
    "_left_overlay.png":                "left_overlay.png",
    "_right_overlay.png":               "right_overlay.png",
    "_stereo_rect_preview.png":         "stereo_rect_preview.png",
    "_stereo_mask_overlay_preview.png": "stereo_mask_overlay_preview.png",
}


def _next_pair_index(pairs_dir: Path) -> int:
    """Return the next available pair index (0-based)."""
    existing = [d for d in pairs_dir.glob("pair_*") if d.is_dir()]
    if not existing:
        return 0
    indices: list[int] = []
    for d in existing:
        try:
            indices.append(int(d.name.split("_")[1]))
        except (IndexError, ValueError):
            pass
    return max(indices) + 1 if indices else 0


def _scan_groups(snap_dir: Path) -> dict[str, dict[str, Path]]:
    """
    Scan snap_dir for timestamped files matching SUFFIX_MAP.
    Returns {stem: {std_name: src_path}}.
    """
    groups: dict[str, dict[str, Path]] = {}
    for f in sorted(snap_dir.iterdir()):
        if not f.is_file():
            continue
        for suffix, std_name in SUFFIX_MAP.items():
            if f.name.endswith(suffix):
                stem = f.name[: -len(suffix)]
                groups.setdefault(stem, {})[std_name] = f
                break
    return groups


def main() -> None:
    if not SNAP_DIR.exists():
        print(f"[ERROR] Snapshot directory not found: {SNAP_DIR}")
        return

    PAIRS_DIR.mkdir(parents=True, exist_ok=True)
    groups = _scan_groups(SNAP_DIR)

    if not groups:
        print(f"[INFO] No matching timestamped files found in {SNAP_DIR}")
        return

    print(f"[INFO] Found {len(groups)} timestamped group(s) in {SNAP_DIR}")
    print()

    n_migrated = 0
    n_skipped  = 0

    for stem in sorted(groups.keys()):
        file_map = groups[stem]

        if "left_rect.png" not in file_map:
            print(f"[SKIP] {stem}: missing left_rect.png")
            n_skipped += 1
            continue

        if "right_rect.png" not in file_map:
            print(f"[SKIP] {stem}: missing right_rect.png")
            n_skipped += 1
            continue

        idx      = _next_pair_index(PAIRS_DIR)
        pair_dir = PAIRS_DIR / f"pair_{idx:06d}"
        pair_dir.mkdir(parents=True, exist_ok=True)

        has_left_mask  = "left_mask.png"  in file_map
        has_right_mask = "right_mask.png" in file_map

        for std_name, src in file_map.items():
            shutil.copy2(str(src), str(pair_dir / std_name))

        metadata = {
            "index":          idx,
            "created_at":     datetime.now().isoformat(),
            "source":         "migrated_timestamp",
            "original_stem":  stem,
            "has_left_mask":  has_left_mask,
            "has_right_mask": has_right_mask,
            "rectified":      True,
        }
        (pair_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )

        print(
            f"[OK] {stem} → {pair_dir.name}  "
            f"files={len(file_map)}  "
            f"left_mask={has_left_mask}  right_mask={has_right_mask}"
        )
        n_migrated += 1

    print()
    print("━━━ Migration summary ━━━")
    print(f"  Migrated : {n_migrated}")
    print(f"  Skipped  : {n_skipped}")
    if n_migrated:
        print(f"  Pairs dir: {PAIRS_DIR}")
        print()
        print("Run point-cloud script on latest pair:")
        print("  python workspace/scripts/offline/masked_stereo_to_cloud.py --latest")
        print()
        print("Run on specific pair (e.g. index 0):")
        print("  python workspace/scripts/offline/masked_stereo_to_cloud.py --index 0")


if __name__ == "__main__":
    main()
