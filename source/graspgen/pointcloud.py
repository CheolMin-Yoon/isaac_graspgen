"""Point-cloud preparation shared by online completion and grasp inference."""

from __future__ import annotations

import numpy as np


def finite_xyz(points: np.ndarray) -> np.ndarray:
    """Return a contiguous float32 ``(N, 3)`` cloud without NaN/Inf rows."""
    xyz = np.asarray(points, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] < 3:
        raise ValueError(f"points must have shape (N, 3+), got {xyz.shape}")
    xyz = xyz[:, :3]
    return np.ascontiguousarray(xyz[np.isfinite(xyz).all(axis=1)])


def crop_aabb(
    points: np.ndarray,
    minimum: np.ndarray,
    maximum: np.ndarray,
    *,
    padding: float = 0.0,
) -> np.ndarray:
    """Crop world-frame points to an axis-aligned object bound."""
    xyz = finite_xyz(points)
    lower = np.asarray(minimum, dtype=np.float32).reshape(3) - float(padding)
    upper = np.asarray(maximum, dtype=np.float32).reshape(3) + float(padding)
    if np.any(lower > upper):
        raise ValueError(f"invalid bounds: minimum={lower}, maximum={upper}")
    keep = np.logical_and(xyz >= lower, xyz <= upper).all(axis=1)
    return np.ascontiguousarray(xyz[keep])


def sample_fixed(points: np.ndarray, count: int, *, seed: int = 0) -> np.ndarray:
    """Deterministically sample ``count`` points, replacing only when necessary."""
    xyz = finite_xyz(points)
    count = int(count)
    if count <= 0:
        raise ValueError(f"count must be positive, got {count}")
    if len(xyz) == 0:
        raise ValueError("cannot sample an empty point cloud")
    rng = np.random.default_rng(seed)
    indices = rng.choice(len(xyz), size=count, replace=len(xyz) < count)
    return np.ascontiguousarray(xyz[indices])
