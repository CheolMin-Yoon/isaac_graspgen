"""Conservative geometric prefilter for executing GraspGen candidates."""

from __future__ import annotations

import numpy as np

from . import config


def select_executable_grasp(
    grasps: np.ndarray,
    confidences: np.ndarray,
    object_points: np.ndarray,
    *,
    gripper_depth: float,
    support_z: float = 0.0,
) -> tuple[int, dict[str, float]]:
    """Choose the highest-confidence table-safe, top-down-ish grasp.

    GraspGen poses locate the gripper base, not its finger center.  The tool
    center used for the object-distance check is therefore
    ``base + gripper_depth*z``, so ``gripper_depth`` must come from the
    GripperSpec actually mounted — passing the wrong gripper's depth silently
    shifts every distance gate.  This is only a cheap execution prefilter;
    collision and IK checks remain future gates.
    """
    poses = np.asarray(grasps, dtype=float).reshape(-1, 4, 4)
    scores = np.asarray(confidences, dtype=float).reshape(-1)
    points = np.asarray(object_points, dtype=float).reshape(-1, 3)
    if len(poses) != len(scores):
        raise ValueError("grasps and confidences must have the same length")
    if len(poses) == 0 or len(points) == 0:
        raise ValueError("grasps and object_points must not be empty")

    object_center = points.mean(axis=0)
    object_radius_xy = float(np.linalg.norm(object_center[:2]))
    if object_radius_xy < 1e-6:
        raise ValueError("object center is too close to the robot base for a radial approach gate")
    outward_xy = object_center[:2] / object_radius_xy
    order = np.argsort(-scores)
    for index in order:
        pose = poses[index]
        base = pose[:3, 3]
        approach = pose[:3, 2]
        tool_center = base + gripper_depth * approach
        tool_distance = float(np.linalg.norm(tool_center - object_center))
        base_clearance = float(base[2] - support_z)
        approach_z = float(approach[2])
        outward_approach = float(np.dot(approach[:2], outward_xy))
        if (
            base_clearance >= config.MIN_GRIPPER_BASE_Z
            and approach_z <= config.MAX_APPROACH_Z
            and tool_distance <= config.MAX_TOOL_TO_OBJECT_DISTANCE
            and outward_approach >= config.MIN_OUTWARD_APPROACH
        ):
            return int(index), {
                "confidence": float(scores[index]),
                "base_clearance": base_clearance,
                "approach_z": approach_z,
                "tool_distance": tool_distance,
                "outward_approach": outward_approach,
            }

    raise RuntimeError(
        "GraspGen returned no execution candidate satisfying the table-clearance, "
        "top-down/outward approach, and tool-to-object distance gates"
    )
