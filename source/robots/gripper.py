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
        self._articulation = articulation
        self._spec = spec
        self._joint_name = spec.joint_name
        self.open_position = float(spec.open_position)
        self.closed_position = np.deg2rad(45.0)

        dof_names = list(articulation.dof_names)
        self._joint_index = dof_names.index(spec.joint_name) if spec.joint_name in dof_names else None
        if self._joint_index is None:
            print(f"[gripper] joint '{spec.joint_name}' not found; available={dof_names}")
            return

        properties = articulation.dof_properties[self._joint_index]
        # The USD limit is authoritative for "closed" — the class default is
        # only a placeholder for assets that omit it.
        self.closed_position = float(properties["upper"])
        print(
            f"[gripper] {spec.name} joint '{spec.joint_name}' @ index {self._joint_index} "
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


ROBOTIQ_2F_140 = GripperSpec(
    name="robotiq_2f_140",
    joint_name="finger_joint",
    open_position=0.0,
    # The official Robotiq asset places the tool center 0.195 m along local +z
    # from the base pose GraspGen returns.
    graspgen_depth=0.195,
    graspgen_gripper_config="/home/frlab/GraspGenModels/checkpoints/graspgen_robotiq_2f_140.yml",
    graspgen_generator_checkpoint="/home/frlab/GraspGenModels/checkpoints/graspgen_robotiq_2f_140_gen.pth",
    make=SingleJointGripper,
)

GRIPPERS = {ROBOTIQ_2F_140.name: ROBOTIQ_2F_140}


def get_gripper(name: str) -> GripperSpec:
    if name not in GRIPPERS:
        raise KeyError(f"unknown gripper '{name}'; available={sorted(GRIPPERS)}")
    return GRIPPERS[name]
