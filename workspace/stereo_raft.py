"""Small inference wrapper for the external RAFT-Stereo checkout.

The public function returns positive disparity in the same convention used by
OpenCV SGBM in this project:

    xR = xL - disparity
"""

from __future__ import annotations

import sys
from argparse import Namespace
from pathlib import Path

import cv2
import numpy as np
import torch


def _resolve_project_path(path: str | Path) -> Path:
    p = Path(path)
    if p.is_absolute():
        return p
    return Path(__file__).resolve().parents[1] / p


def _add_raft_to_path(repo_dir: Path) -> None:
    repo_dir = repo_dir.resolve()
    core_dir = repo_dir / "core"
    for p in (repo_dir, core_dir):
        s = str(p)
        if s not in sys.path:
            sys.path.insert(0, s)


def _make_raft_args(
    checkpoint: Path,
    corr_implementation: str,
    mixed_precision: bool,
) -> Namespace:
    return Namespace(
        restore_ckpt=str(checkpoint),
        mixed_precision=mixed_precision,
        hidden_dims=[128, 128, 128],
        corr_implementation=corr_implementation,
        shared_backbone=False,
        corr_levels=4,
        corr_radius=4,
        n_downsample=2,
        context_norm="batch",
        slow_fast_gru=False,
        n_gru_layers=3,
    )


def _image_to_tensor(bgr: np.ndarray, device: torch.device) -> torch.Tensor:
    if bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError(f"Expected HxWx3 BGR image, got shape {bgr.shape}")

    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    tensor = torch.from_numpy(rgb).permute(2, 0, 1).float()
    return tensor[None].to(device)


def _load_checkpoint(
    model: torch.nn.Module,
    checkpoint_path: Path,
    device: torch.device,
) -> None:
    state = torch.load(str(checkpoint_path), map_location=device)
    if isinstance(state, dict):
        for key in ("state_dict", "model"):
            if key in state and isinstance(state[key], dict):
                state = state[key]
                break

    if not isinstance(state, dict):
        raise TypeError(f"Unsupported checkpoint format: {checkpoint_path}")

    try:
        torch.nn.DataParallel(model).load_state_dict(state, strict=True)
        return
    except RuntimeError:
        pass

    stripped = {
        k.removeprefix("module."): v
        for k, v in state.items()
    }
    model.load_state_dict(stripped, strict=True)


def compute_raft_disparity(
    left_bgr: np.ndarray,
    right_bgr: np.ndarray,
    *,
    raft_repo_dir: str | Path = "external/RAFT-Stereo",
    checkpoint: str | Path = "workspace/models/raft_stereo/raftstereo-middlebury.pth",
    iters: int = 32,
    corr_implementation: str = "alt",
    mixed_precision: bool = True,
    device: str | None = None,
) -> np.ndarray:
    """Compute a positive HxW float32 disparity map with RAFT-Stereo."""
    if left_bgr.shape[:2] != right_bgr.shape[:2]:
        raise ValueError(
            f"Left/right image sizes differ: {left_bgr.shape[:2]} vs {right_bgr.shape[:2]}"
        )

    repo_dir = _resolve_project_path(raft_repo_dir)
    checkpoint_path = _resolve_project_path(checkpoint)

    if not repo_dir.exists():
        raise FileNotFoundError(f"RAFT-Stereo repo not found: {repo_dir}")
    if not checkpoint_path.exists():
        raise FileNotFoundError(f"RAFT-Stereo checkpoint not found: {checkpoint_path}")

    _add_raft_to_path(repo_dir)

    from core.raft_stereo import RAFTStereo  # type: ignore
    from core.utils.utils import InputPadder  # type: ignore

    torch_device = torch.device(
        device if device is not None else ("cuda" if torch.cuda.is_available() else "cpu")
    )
    use_mixed_precision = bool(mixed_precision and torch_device.type == "cuda")

    if corr_implementation.endswith("_cuda") and torch_device.type != "cuda":
        raise ValueError(
            f"corr_implementation={corr_implementation!r} requires a CUDA device"
        )

    args = _make_raft_args(
        checkpoint_path,
        corr_implementation=corr_implementation,
        mixed_precision=use_mixed_precision,
    )

    model = RAFTStereo(args)
    _load_checkpoint(model, checkpoint_path, torch_device)
    model.to(torch_device)
    model.eval()

    image1 = _image_to_tensor(left_bgr, torch_device)
    image2 = _image_to_tensor(right_bgr, torch_device)

    padder = InputPadder(image1.shape, divis_by=32)
    image1, image2 = padder.pad(image1, image2)

    with torch.no_grad():
        _, flow_up = model(image1, image2, iters=iters, test_mode=True)

    flow_up = padder.unpad(flow_up.float()).squeeze(0).squeeze(0)

    # RAFT-Stereo predicts x-flow in the training convention flow_x = -disparity.
    disparity = (-flow_up).detach().cpu().numpy().astype(np.float32, copy=False)
    return np.ascontiguousarray(disparity)
