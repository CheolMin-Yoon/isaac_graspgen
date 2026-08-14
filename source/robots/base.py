"""Robot and gripper specifications consumed by the scene entrypoint.

A robot is described by data, not by a subclass: ``RobotSpec`` carries the
asset paths, the kinematic facts the PINK controller needs, and two factories
(``spawn`` and ``make_gripper``) for the parts that genuinely differ per arm.
Adding an arm means adding one package under ``robots/`` that exports ``SPEC``
and registering it in ``robots/__init__.py``.

Nothing here imports Isaac Sim — the specs must stay importable from plain
pytest, so every Isaac import lives inside the factory bodies.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Callable, Sequence


@dataclass(frozen=True)
class GripperSpec:
    """A gripper, from both the articulation side and the GraspGen side.

    GraspGen is trained per gripper, so the checkpoint pair travels with the
    gripper rather than with the arm — an Indy7 and a Panda carrying the same
    Robotiq 2F-140 share these values.
    """

    name: str
    # articulation DOF driven for open/close
    joint_name: str
    open_position: float
    # Distance (m) from the gripper base pose GraspGen returns to the tool
    # center, along the grasp local +z. Used to score candidates, not to move.
    graspgen_depth: float
    # GraspGen server checkpoints for this gripper
    graspgen_gripper_config: str
    graspgen_generator_checkpoint: str
    # (articulation, spec) -> gripper controller
    make: Callable


@dataclass(frozen=True)
class RobotSpec:
    """One arm plus the gripper mounted on it."""

    name: str
    usd_path: str
    prim_path: str
    position: Sequence[float]
    # () -> PinkRobot. A callable rather than a URDF path because the two ways
    # of getting a PINK model are not interchangeable: a repo-authored arm uses
    # ``load_pink_robot(path)``, while an arm Isaac already bundles uses
    # ``load_pink_supported_robot(name)``, which also wires up the SRDF and
    # package dirs. Built lazily so Isaac imports stay out of module scope.
    make_pink_robot: Callable
    # Human-readable provenance of that model, for logs and error messages —
    # a filesystem path for repo-authored arms, "bundled:<name>" otherwise.
    kinematics_source: str
    ee_link_name: str
    num_arm_dofs: int
    # Bent, nonsingular seed posture applied before reactive pose tracking
    reach_posture: Sequence[float]
    # Articulation link the wrist camera is mounted under
    wrist_camera_link: str
    gripper: GripperSpec
    # (spec) -> SingleArticulation, already wrapped for world.scene.add
    spawn: Callable

    def make_ik(self, articulation, dt: float):
        """Build the PINK controller for this arm."""
        from robots.arm_ik import PinkArmIK

        return PinkArmIK(articulation, self, dt=dt)

    def make_gripper(self, articulation):
        return self.gripper.make(articulation, self.gripper)
