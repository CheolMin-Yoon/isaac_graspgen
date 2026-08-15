"""Conservative geometric prefilter for executing GraspGen candidates."""

from __future__ import annotations

import numpy as np

from . import config


def _closing_axis_offset(points: np.ndarray, pose: np.ndarray, tool_center: np.ndarray) -> float:
    """Signed distance from the jaws' centre to the object's midline.

    The closing axis is the pose's y column -- verified in simulation by dotting
    it against the axis the two finger links physically define, which returned
    -1.000 rather than an assumption.

    The midline is taken as the centre of the observed points' 5th-95th
    percentile span along that axis, not their mean. The mean of a one-sided
    view is pulled toward the camera; the percentile span is symmetric for a
    symmetric object as long as its width is observed, which is exactly the
    direction the jaws travel. Percentiles rather than min/max so a few stray
    points off the object do not define the span. Everything here comes from
    the point cloud, so it holds outside simulation too.
    """
    closing_axis = np.asarray(pose[:3, 1], dtype=float)
    projected = points @ closing_axis
    low, high = np.percentile(projected, [5.0, 95.0])
    return float(np.dot(tool_center, closing_axis) - 0.5 * (low + high))


def select_executable_grasp(
    grasps: np.ndarray,
    confidences: np.ndarray,
    object_points: np.ndarray,
    *,
    gripper_depth: float,
    support_z: float = 0.0,
) -> tuple[int, dict[str, float]]:
    """Choose the best-centred grasp among the table-safe, top-down-ish ones.

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
    feasible: list[tuple[int, dict[str, float]]] = []
    rejected_offsets: list[float] = []
    for rank, index in enumerate(order):
        pose = poses[index]
        base = pose[:3, 3]
        approach = pose[:3, 2]
        tool_center = base + gripper_depth * approach
        tool_distance = float(np.linalg.norm(tool_center - object_center))
        base_clearance = float(base[2] - support_z)
        approach_z = float(approach[2])
        outward_approach = float(np.dot(approach[:2], outward_xy))
        closing_offset = _closing_axis_offset(points, pose, tool_center)
        # Say why a candidate was passed over. These gates are heuristics tuned
        # for one arm and one scene, and a heuristic that silently discards the
        # model's best answer is worse than no heuristic at all -- measured, the
        # two highest-confidence grasps bracketed the can to within 2mm and this
        # filter walked past both.
        failed = [
            name
            for name, ok in (
                (f"base_clearance {base_clearance:.3f}<{config.MIN_GRIPPER_BASE_Z}",
                 base_clearance >= config.MIN_GRIPPER_BASE_Z),
                (f"approach_z {approach_z:.3f}>{config.MAX_APPROACH_Z}",
                 approach_z <= config.MAX_APPROACH_Z),
                (f"tool_distance {tool_distance:.3f}>{config.MAX_TOOL_TO_OBJECT_DISTANCE}",
                 tool_distance <= config.MAX_TOOL_TO_OBJECT_DISTANCE),
                (f"outward {outward_approach:.3f}<{config.MIN_OUTWARD_APPROACH}",
                 outward_approach >= config.MIN_OUTWARD_APPROACH),
            )
            if not ok
        ]
        if failed:
            if rank < 12:
                print(
                    f"[grasp] candidate #{int(index)} conf={scores[index]:.3f} "
                    f"centring={closing_offset * 1000:+6.1f}mm rejected: {', '.join(failed)}"
                )
            rejected_offsets.append(abs(closing_offset))
            continue
        feasible.append(
            (
                int(index),
                {
                    "confidence": float(scores[index]),
                    "base_clearance": base_clearance,
                    "approach_z": approach_z,
                    "tool_distance": tool_distance,
                    "outward_approach": outward_approach,
                    "closing_offset": closing_offset,
                },
            )
        )

    if not feasible:
        raise RuntimeError(
            "GraspGen returned no execution candidate satisfying the table-clearance, "
            "top-down/outward approach, and tool-to-object distance gates"
        )

    # Rank the survivors by centring rather than by confidence. The gates above
    # answer "can the arm execute this at all"; among the candidates that pass,
    # confidence is GraspGen's opinion while the closing-axis offset is a
    # geometric fact we can measure, and it is the one that decides whether the
    # jaws bracket the object. Measured on one can, confidence did not rank it:
    # the top-ranked candidate sat 22mm off its axis while #2 and #3 were inside
    # 5mm, on an object with 7mm of clearance a side.
    best_index, best_metrics = min(feasible, key=lambda item: abs(item[1]["closing_offset"]))
    if rejected_offsets:
        # If the gates are throwing away the well-centred grasps, that is a
        # property of the gates, not of GraspGen, and it is only visible by
        # comparing the two pools.
        print(
            f"[grasp] best centring: {abs(best_metrics['closing_offset']) * 1000:.1f}mm among "
            f"{len(feasible)} feasible, {min(rejected_offsets) * 1000:.1f}mm among "
            f"{len(rejected_offsets)} rejected"
        )
    print(
        f"[grasp] {len(feasible)} candidate(s) passed the gates; "
        f"chose #{best_index} by centring "
        f"({best_metrics['closing_offset'] * 1000:+.1f}mm, conf {best_metrics['confidence']:.3f})"
    )
    return best_index, best_metrics


def report_candidate_centering(
    grasps: np.ndarray,
    confidences: np.ndarray,
    reference_center,
    *,
    gripper_depth: float,
    selected: int,
    limit: int = 8,
) -> None:
    """Print how well each candidate's jaws would bracket ``reference_center``.

    A cylinder can only be grasped where the closing axis passes through its
    axis, so the offset along that axis is the number that decides whether the
    fingers bracket the can or clip it. Which matrix column *is* the closing
    axis is a convention this repo has not verified, so both in-plane columns
    are reported and the caller compares them against the closing axis measured
    off the fingers themselves. Whichever matches, the spread across candidates
    says whether the selection gate is choosing badly or every candidate is off.
    """
    poses = np.asarray(grasps, dtype=float).reshape(-1, 4, 4)
    scores = np.asarray(confidences, dtype=float).reshape(-1)
    center = np.asarray(reference_center, dtype=float)

    print(f"[grasp] candidate centering against {center.round(4).tolist()} (mm):")
    for rank, index in enumerate(np.argsort(-scores)):
        if rank >= limit:
            break
        pose = poses[index]
        tool_center = pose[:3, 3] + gripper_depth * pose[:3, 2]
        offset = center - tool_center
        mark = " <== selected" if int(index) == int(selected) else ""
        print(
            f"[grasp]   #{int(index):<2} conf={scores[index]:.3f} "
            f"along-x={float(np.dot(offset, pose[:3, 0])) * 1000:+7.1f} "
            f"along-y={float(np.dot(offset, pose[:3, 1])) * 1000:+7.1f} "
            f"along-approach={float(np.dot(offset, pose[:3, 2])) * 1000:+7.1f}{mark}"
        )
