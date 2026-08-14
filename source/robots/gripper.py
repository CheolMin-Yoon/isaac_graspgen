"""Gripper controllers and the gripper registry.

``SingleJointGripper`` covers any gripper Isaac exposes as one driven DOF with
the remaining fingers following through mimic joints — the Robotiq 2F-140 as
grafted in ``robots.indy7.spawn``. A gripper whose fingers are independently
actuated (a Panda hand's two prismatic joints) needs its own controller class,
not a wider constructor here.
"""

from __future__ import annotations

import numpy as np

from robots.base import GripperSpec


class SingleJointGripper:
    """Position control for a gripper driven through one articulation DOF."""

    def __init__(self, articulation, spec: GripperSpec) -> None:
        if len(spec.joint_names) != 1:
            raise ValueError(
                f"SingleJointGripper drives exactly one DOF, but {spec.name} declares "
                f"{list(spec.joint_names)}; independently actuated fingers need "
                "ParallelFingerGripper"
            )
        self._articulation = articulation
        self._spec = spec
        self._joint_name = spec.joint_names[0]
        self.open_position = float(spec.open_position)
        self.closed_position = np.deg2rad(45.0)

        dof_names = list(articulation.dof_names)
        self._joint_index = dof_names.index(self._joint_name) if self._joint_name in dof_names else None
        if self._joint_index is None:
            print(f"[gripper] joint '{self._joint_name}' not found; available={dof_names}")
            return

        properties = articulation.dof_properties[self._joint_index]
        # The USD limit is authoritative for "closed" — the class default is
        # only a placeholder for assets that omit it.
        self.closed_position = float(properties["upper"])
        print(
            f"[gripper] {spec.name} joint '{self._joint_name}' @ index {self._joint_index} "
            f"limits=[{properties['lower']:.4f}, {properties['upper']:.4f}] "
            f"stiffness={properties['stiffness']:.3f} damping={properties['damping']:.3f} "
            f"max_effort={properties['maxEffort']:.3f}"
        )

    @property
    def available(self) -> bool:
        return self._joint_index is not None

    def open(self) -> None:
        self.set(self.open_position)

    def close(self) -> None:
        self.set(self.closed_position)

    def set(self, target: float) -> None:
        if self._joint_index is None:
            return
        from isaacsim.core.utils.types import ArticulationAction

        self._articulation.apply_action(
            ArticulationAction(
                joint_positions=[float(target)],
                joint_indices=[self._joint_index],
            )
        )

    @property
    def hold_position(self) -> float:
        if self._joint_index is None:
            return self.open_position
        positions = self._articulation.get_joint_positions()
        return float(positions[self._joint_index])

    @property
    def target_position(self) -> float:
        """Return the currently applied articulation position target."""
        if self._joint_index is None:
            return self.open_position
        action = self._articulation.get_applied_action()
        return float(action.joint_positions[self._joint_index])

    @property
    def measured_effort(self) -> float:
        if self._joint_index is None:
            return 0.0
        return float(self._articulation.get_measured_joint_efforts([self._joint_index])[0])

    def hold(self, target: float | None = None) -> None:
        self.set(self.hold_position if target is None else target)


class ParallelFingerGripper:
    """Position control for a gripper whose fingers are each driven directly.

    The Panda hand has no mimic relation between ``panda_finger_joint1`` and
    ``panda_finger_joint2``: both are prismatic and both must be commanded, so
    ``SingleJointGripper`` would close one finger and leave the other open.
    ``open_position`` and the per-joint upper limit are half-widths — each
    finger travels its own side of the centreline.
    """

    def __init__(self, articulation, spec: GripperSpec) -> None:
        self._articulation = articulation
        self._spec = spec
        self.open_position = float(spec.open_position)
        self.closed_position = 0.0

        dof_names = list(articulation.dof_names)
        self._joint_indices = [dof_names.index(n) for n in spec.joint_names if n in dof_names]
        missing = [n for n in spec.joint_names if n not in dof_names]
        if missing:
            print(f"[gripper] joints {missing} not found; available={dof_names}")
            self._joint_indices = []
            return

        lowers = []
        for index in self._joint_indices:
            properties = articulation.dof_properties[index]
            lowers.append(float(properties["lower"]))
        # A parallel-jaw hand closes toward its lower limit, unlike the
        # Robotiq's rotary finger joint which closes toward its upper limit.
        self.closed_position = max(lowers)
        print(
            f"[gripper] {spec.name} joints {spec.joint_names} @ indices "
            f"{self._joint_indices} open={self.open_position:.4f} "
            f"closed={self.closed_position:.4f}"
        )

    @property
    def available(self) -> bool:
        return bool(self._joint_indices)

    def open(self) -> None:
        self.set(self.open_position)

    def close(self) -> None:
        self.set(self.closed_position)

    def set(self, target: float) -> None:
        if not self._joint_indices:
            return
        from isaacsim.core.utils.types import ArticulationAction

        self._articulation.apply_action(
            ArticulationAction(
                joint_positions=[float(target)] * len(self._joint_indices),
                joint_indices=list(self._joint_indices),
            )
        )

    @property
    def hold_position(self) -> float:
        if not self._joint_indices:
            return self.open_position
        positions = self._articulation.get_joint_positions()
        return float(np.mean([positions[i] for i in self._joint_indices]))

    @property
    def target_position(self) -> float:
        if not self._joint_indices:
            return self.open_position
        action = self._articulation.get_applied_action()
        return float(np.mean([action.joint_positions[i] for i in self._joint_indices]))

    @property
    def measured_effort(self) -> float:
        if not self._joint_indices:
            return 0.0
        efforts = self._articulation.get_measured_joint_efforts(list(self._joint_indices))
        return float(np.mean(np.abs(np.asarray(efforts, dtype=float))))

    def hold(self, target: float | None = None) -> None:
        self.set(self.hold_position if target is None else target)


ROBOTIQ_2F_140 = GripperSpec(
    name="robotiq_2f_140",
    joint_names=("finger_joint",),
    open_position=0.0,
    # The official Robotiq asset places the tool center 0.195 m along local +z
    # from the base pose GraspGen returns.
    graspgen_depth=0.195,
    graspgen_gripper_config="/home/frlab/GraspGenModels/checkpoints/graspgen_robotiq_2f_140.yml",
    graspgen_generator_checkpoint="/home/frlab/GraspGenModels/checkpoints/graspgen_robotiq_2f_140_gen.pth",
    make=SingleJointGripper,
)

PANDA_HAND = GripperSpec(
    name="panda_hand",
    joint_names=("panda_finger_joint1", "panda_finger_joint2"),
    # Half-width per finger, from Isaac's official Franka PINK example.
    open_position=0.035,
    # Isaac's FrankaPinkIKExample offsets panda_hand by this to reach the
    # fingertip midpoint.
    graspgen_depth=0.1034,
    # NOT downloaded yet — GraspGen publishes a Franka Panda checkpoint, but
    # /home/frlab/GraspGenModels/checkpoints only holds the Robotiq pair.
    # scripts/run_graspgen_server.py fails with the missing path until then.
    graspgen_gripper_config="/home/frlab/GraspGenModels/checkpoints/graspgen_franka_panda.yml",
    graspgen_generator_checkpoint="/home/frlab/GraspGenModels/checkpoints/graspgen_franka_panda_gen.pth",
    make=ParallelFingerGripper,
)

GRIPPERS = {g.name: g for g in (ROBOTIQ_2F_140, PANDA_HAND)}


def get_gripper(name: str) -> GripperSpec:
    if name not in GRIPPERS:
        raise KeyError(f"unknown gripper '{name}'; available={sorted(GRIPPERS)}")
    return GRIPPERS[name]
