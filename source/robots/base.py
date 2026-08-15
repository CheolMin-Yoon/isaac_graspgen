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
    # Articulation DOFs commanded for open/close. One entry when the remaining
    # fingers follow through mimic joints (Robotiq 2F-140), one per finger when
    # they are independently actuated (Panda hand).
    joint_names: Sequence[str]
    # Half-width for a parallel jaw, joint angle for a rotary finger.
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
    # Either an absolute local path, or an assets-root-relative path starting
    # with "/Isaac/" for a robot Isaac already ships. Resolve it through
    # ``resolve_usd_path`` rather than reading it directly.
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
    # Arm posture the wrist camera observes from. Currently the same joint
    # values as reach_posture for both arms, but named separately because the
    # two are answerable to different things: reach_posture must be nonsingular
    # for the IK, observation_posture must let the camera see the workspace.
    # Kept explicit so the GraspGen viewpoint is a stated choice rather than
    # wherever the arm happened to be when the trigger step arrived.
    observation_posture: Sequence[float]
    # Wrist camera description, or None for an arm that has none (and so
    # cannot feed GraspGen). Requires a "link" key naming the articulation link
    # it hangs under; every other key overrides sim.config.CAMERA_CONFIG. The
    # mount mode lives here rather than in the scene config because it is a
    # property of the robot's asset: "asset" wraps a RealSense the USD already
    # references, "pinhole" creates a bare camera prim for a USD with none.
    wrist_camera: dict | None
    gripper: GripperSpec
    # (spec) -> SingleArticulation, already wrapped for world.scene.add
    spawn: Callable

    def resolve_usd_path(self) -> str:
        """Absolute USD path, expanding an assets-root-relative "/Isaac/..." path.

        Local assets are checked for existence here so a moved file fails now
        rather than as a confusing USD composition error later; a Nucleus path
        can only be checked by trying to load it.
        """
        if not self.usd_path.startswith("/Isaac/"):
            if not os.path.isfile(self.usd_path):
                raise FileNotFoundError(f"{self.name} USD not found: {self.usd_path}")
            return self.usd_path

        from isaacsim.storage.native import get_assets_root_path

        assets_root = get_assets_root_path()
        if assets_root is None:
            raise RuntimeError(
                f"{self.name} needs the Isaac asset root for {self.usd_path}, but "
                "get_assets_root_path() returned None; check Nucleus/assets access"
            )
        return assets_root + self.usd_path

    def make_ik(self, articulation, dt: float):
        """Build the PINK controller for this arm."""
        from robots.arm_ik import PinkArmIK

        return PinkArmIK(articulation, self, dt=dt)

    def make_gripper(self, articulation):
        return self.gripper.make(articulation, self.gripper)
