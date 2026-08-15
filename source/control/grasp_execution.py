"""Execute a GraspGen world-frame grasp pose with differential IK.

Phase machine: pregrasp -> approach -> close -> lift -> done. Each phase
tracks one TCP target with ``robots.arm_ik.PinkArmIK`` and advances after the
TCP stays within ``config.POSITION_TOL`` for ``config.SETTLE_STEPS`` steps.

The executor is robot-agnostic: it only needs an object with ``go_to`` and
``ee_pose`` plus a gripper with ``open``/``close``.

Example wiring (see the repo-root ``grasp_scene.py`` entrypoint):

    executor = GraspExecutor(ik, gripper)
    executor.start(result.grasps[int(np.argmax(result.confidences))])
    # inside the simulation loop, every step:
    phase = executor.step()

While an executor is active the entrypoint must not issue its own gripper
hold commands — the executor owns the gripper.
"""

from __future__ import annotations

import numpy as np

from . import config


def rotation_to_wxyz(rot) -> np.ndarray:
    """Convert a 3x3 rotation matrix to an Isaac wxyz quaternion."""
    m = np.asarray(rot, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    quat = np.array([w, x, y, z], dtype=float)
    return quat / np.linalg.norm(quat)


class GraspExecutor:
    """Track a grasp pose through pregrasp/approach/close/lift phases."""

    def __init__(self, ik, gripper, cfg=config) -> None:
        self._ik = ik
        self._gripper = gripper
        self._cfg = cfg
        self._phase = "idle"
        self._orientation: np.ndarray | None = None
        self._targets: dict[str, np.ndarray] = {}
        self._approach: np.ndarray | None = None
        self._best_error_vector: np.ndarray | None = None
        self._settle = 0
        self._phase_steps = 0
        self._stalled = 0
        # Best errors seen in the current phase, so a failure can say which
        # gate it failed and how close it got. "Did not reach the pose" is not
        # actionable; "position was within 2mm but orientation plateaued at
        # 0.6 rad" points straight at the cause.
        self._best_position_error = float("inf")
        self._best_orientation_error = float("inf")
        # Best-ever and current are different questions. A phase that touched
        # the target once and then drifted reports a passing "best" while
        # failing, which reads as a contradiction unless both are printed.
        self._position_error = float("inf")
        self._orientation_error = float("inf")

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def active(self) -> bool:
        return self._phase not in ("idle", "done", "failed")

    def start(self, grasp_pose) -> None:
        """Arm the executor with a world-frame 4x4 grasp pose."""
        pose = np.asarray(grasp_pose, dtype=float)
        if pose.shape != (4, 4):
            raise ValueError(f"grasp pose must be 4x4, got {pose.shape}")

        rotation = pose[:3, :3]
        position = pose[:3, 3]
        approach = rotation[:, 2]  # GraspGen convention: +z is the approach axis

        cfg = self._cfg
        grasp_pos = position + (cfg.TCP_OFFSET + cfg.GRASP_DEPTH_OFFSET) * approach
        self._targets = {
            "pregrasp": grasp_pos - cfg.PREGRASP_OFFSET * approach,
            "approach": grasp_pos,
            "lift": grasp_pos + np.array([0.0, 0.0, cfg.LIFT_HEIGHT]),
        }
        self._orientation = rotation_to_wxyz(rotation)
        self._approach = approach / np.linalg.norm(approach)
        self._settle = 0
        self._phase_steps = 0
        self._stalled = 0
        self._best_position_error = float("inf")
        self._best_orientation_error = float("inf")
        self._best_error_vector = None
        self._phase = "pregrasp"
        self._gripper.open()

    def step(self) -> str:
        """Advance one simulation step. Call every ``world.step``."""
        if not self.active:
            return self._phase

        cfg = self._cfg
        self._phase_steps += 1
        if self._phase_steps > cfg.MAX_PHASE_STEPS:
            self._fail(f"exceeded {cfg.MAX_PHASE_STEPS} steps")
            return self._phase

        if self._phase == "close":
            self._gripper.close()
            if self._phase_steps >= cfg.CLOSE_STEPS:
                self._enter("lift")
            return self._phase

        target = self._targets[self._phase]
        self._ik.go_to(target, self._orientation)
        if self._phase in ("close", "lift"):
            self._gripper.close()
        else:
            self._gripper.open()

        ee_pos, ee_quat = self._ik.ee_pose()
        position_error = float(np.linalg.norm(np.asarray(ee_pos, dtype=float) - target))
        # Orientation counts too: reaching the grasp point with the hand still
        # rotated away is not reaching the grasp.
        orientation_error = 2.0 * np.arccos(
            np.clip(
                abs(float(np.dot(self._orientation, np.asarray(ee_quat, dtype=float)))),
                0.0,
                1.0,
            )
        )
        # Progress on either axis counts: the arm often locks position first and
        # then spends hundreds of steps rotating, which is converging, not stuck.
        improved = (
            position_error < self._best_position_error - cfg.STALL_MIN_IMPROVEMENT
            or orientation_error < self._best_orientation_error - cfg.STALL_MIN_IMPROVEMENT
        )
        self._position_error = position_error
        self._orientation_error = orientation_error
        if position_error < self._best_position_error:
            self._best_error_vector = np.asarray(ee_pos, dtype=float) - target
        self._best_position_error = min(self._best_position_error, position_error)
        self._best_orientation_error = min(self._best_orientation_error, orientation_error)

        reached = position_error < cfg.POSITION_TOL and orientation_error < cfg.ORIENTATION_TOL
        self._settle = self._settle + 1 if reached else 0
        self._stalled = 0 if (improved or reached) else self._stalled + 1
        if self._stalled >= cfg.STALL_STEPS:
            self._fail(f"no progress for {cfg.STALL_STEPS} steps")
            return self._phase

        if self._settle >= cfg.SETTLE_STEPS:
            if self._phase == "pregrasp":
                self._enter("approach")
            elif self._phase == "approach":
                self._enter("close")
            elif self._phase == "lift":
                self._phase = "done"
        return self._phase

    def _fail(self, reason: str) -> None:
        cfg = self._cfg
        # Decompose the shortfall along the grasp's own approach axis. A gap
        # that lies along the approach is a depth/frame problem; one that lies
        # across it is not, and the distinction rules out half the candidates
        # without another guess.
        if self._best_error_vector is not None and self._approach is not None:
            along = float(np.dot(self._best_error_vector, self._approach))
            perpendicular = self._best_error_vector - along * self._approach
            print(
                f"[grasp] shortfall decomposition: along approach={along * 1000:+.1f}mm, "
                f"perpendicular={float(np.linalg.norm(perpendicular)) * 1000:.1f}mm, "
                f"world={np.round(self._best_error_vector * 1000, 1).tolist()}mm "
                f"(approach axis={np.round(self._approach, 3).tolist()})"
            )
        self._report_gripper(f"at {self._phase} failure")
        target = self._targets.get(self._phase)
        if target is not None and hasattr(self._ik, "diagnose"):
            self._ik.diagnose(target, self._orientation)
        print(
            f"[grasp] {self._phase} failed ({reason}) at step {self._phase_steps}; "
            f"best position error={self._best_position_error * 1000:.1f}mm "
            f"(tol {cfg.POSITION_TOL * 1000:.0f}mm), "
            f"best orientation error={self._best_orientation_error:.3f}rad "
            f"(tol {cfg.ORIENTATION_TOL:.3f}rad)"
        )
        print(
            f"[grasp]   at failure: position={self._position_error * 1000:.1f}mm "
            f"orientation={self._orientation_error:.3f}rad settle={self._settle}"
        )
        self._phase = "failed"

    def _report_gripper(self, label: str) -> None:
        """Print commanded vs measured finger travel.

        A grasp that never closed and a grasp that closed on air look identical
        from the arm's pose alone, so the width is worth a line at every
        transition rather than a guess afterwards.
        """
        gripper = self._gripper
        try:
            print(
                f"[grasp] gripper {label}: target={gripper.target_position:.4f} "
                f"measured={gripper.hold_position:.4f} effort={gripper.measured_effort:.3f}"
            )
        except Exception as error:  # diagnostics must never break execution
            print(f"[grasp] gripper {label}: unavailable ({error})")

    def _enter(self, phase: str) -> None:
        self._report_gripper(f"entering {phase}")
        self._phase = phase
        self._settle = 0
        self._phase_steps = 0
        self._stalled = 0
        self._best_position_error = float("inf")
        self._best_orientation_error = float("inf")
