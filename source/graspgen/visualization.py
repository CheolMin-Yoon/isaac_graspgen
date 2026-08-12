"""Viewport visualization for world-frame GraspGen poses."""

from __future__ import annotations

import numpy as np


def grasp_axis_lines(
    grasps: np.ndarray,
    confidences: np.ndarray,
    *,
    max_grasps: int = 20,
    axis_length: float = 0.04,
) -> tuple[list[list[float]], list[list[float]], list[list[float]]]:
    """Build RGB axis lines, ordered from highest to lowest confidence."""
    poses = np.asarray(grasps, dtype=np.float32).reshape(-1, 4, 4)
    scores = np.asarray(confidences, dtype=np.float32).reshape(-1)
    if len(poses) != len(scores):
        raise ValueError("grasps and confidences must have the same length")

    starts: list[list[float]] = []
    ends: list[list[float]] = []
    colors: list[list[float]] = []
    axis_colors = (
        [1.0, 0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0, 1.0],
        [0.0, 0.4, 1.0, 1.0],
    )
    order = np.argsort(scores)[::-1][: max(0, int(max_grasps))]
    for index in order:
        pose = poses[index]
        origin = pose[:3, 3]
        for axis in range(3):
            endpoint = origin + pose[:3, axis] * float(axis_length)
            starts.append(origin.tolist())
            ends.append(endpoint.tolist())
            colors.append(axis_colors[axis])
    return starts, ends, colors


def draw_grasps(
    grasps: np.ndarray,
    confidences: np.ndarray,
    *,
    max_grasps: int = 20,
    axis_length: float = 0.04,
) -> None:
    """Draw the best GraspGen poses as X/Y/Z frames in the Isaac viewport."""
    from isaacsim.util.debug_draw import _debug_draw

    starts, ends, colors = grasp_axis_lines(
        grasps,
        confidences,
        max_grasps=max_grasps,
        axis_length=axis_length,
    )
    draw = _debug_draw.acquire_debug_draw_interface()
    draw.clear_lines()
    if starts:
        draw.draw_lines(starts, ends, colors, [2.0] * len(starts))


def draw_pointcloud(points: np.ndarray, *, max_points: int = 4000) -> None:
    """Draw the exact point cloud sent to GraspGen."""
    from isaacsim.util.debug_draw import _debug_draw

    xyz = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    if len(xyz) > max_points:
        xyz = xyz[np.linspace(0, len(xyz) - 1, max_points, dtype=int)]
    draw = _debug_draw.acquire_debug_draw_interface()
    draw.clear_points()
    if len(xyz):
        draw.draw_points(
            xyz.tolist(),
            [[1.0, 0.7, 0.0, 1.0]] * len(xyz),
            [3.0] * len(xyz),
        )
