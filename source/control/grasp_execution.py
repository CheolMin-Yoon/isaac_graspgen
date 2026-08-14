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
        self._settle = 0
        self._phase_steps = 0

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
        self._settle = 0
        self._phase_steps = 0
        self._phase = "pregrasp"
        self._gripper.open()

    def step(self) -> str:
        """Advance one simulation step. Call every ``world.step``."""
        if not self.active:
            return self._phase

        cfg = self._cfg
        self._phase_steps += 1
        if self._phase_steps > cfg.TIMEOUT_STEPS:
            self._phase = "failed"
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

        ee_pos, _ = self._ik.ee_pose()
        error = float(np.linalg.norm(np.asarray(ee_pos, dtype=float) - target))
        self._settle = self._settle + 1 if error < cfg.POSITION_TOL else 0

        if self._settle >= cfg.SETTLE_STEPS:
            if self._phase == "pregrasp":
                self._enter("approach")
            elif self._phase == "approach":
                self._enter("close")
            elif self._phase == "lift":
                self._phase = "done"
        return self._phase

    def _enter(self, phase: str) -> None:
        self._phase = phase
        self._settle = 0
        self._phase_steps = 0
