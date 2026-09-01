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
        for index, name in zip(self._joint_indices, spec.joint_names):
            properties = articulation.dof_properties[index]
            lowers.append(float(properties["lower"]))
            # A finger that will not move is either unpowered or out of effort,
            # and the two look identical from the joint position alone.
            print(
                f"[gripper] {name} @ {index} limits=[{properties['lower']:.4f}, "
                f"{properties['upper']:.4f}] stiffness={properties['stiffness']:.1f} "
                f"damping={properties['damping']:.1f} max_effort={properties['maxEffort']:.1f}"
            )
        # A parallel-jaw hand closes toward its lower limit, unlike the
        # Robotiq's rotary finger joint which closes toward its upper limit.
        self.closed_position = max(lowers)
        self._power_unpowered_fingers(articulation, spec)
        print(
            f"[gripper] {spec.name} joints {spec.joint_names} @ indices "
            f"{self._joint_indices} open={self.open_position:.4f} "
            f"closed={self.closed_position:.4f}"
        )

    def _power_unpowered_fingers(self, articulation, spec: GripperSpec) -> None:
        """Give every finger the drive gains of the strongest one.

        Isaac's ``franka.usd`` authors a drive on ``panda_finger_joint1`` only:
        joint 2 comes through with stiffness, damping and max effort all zero.
        An unpowered finger does not hold its commanded position -- measured, it
        was pushed shut while the gripper was commanded fully open, which reads
        as a grasp closing on the object when it is the object closing the
        gripper. Copying the authored gains is the smallest fix that makes both
        jaws behave like jaws.
        """
        import numpy as np

        properties = [articulation.dof_properties[index] for index in self._joint_indices]
        stiffness = [float(p["stiffness"]) for p in properties]
        strongest = int(np.argmax(stiffness))
        if stiffness[strongest] <= 0.0:
            print(f"[gripper] {spec.name}: no finger has a drive authored; leaving gains alone")
            return

        for position, (index, properties_i) in enumerate(zip(self._joint_indices, properties)):
            if float(properties_i["stiffness"]) > 0.0:
                continue
            reference = properties[strongest]
            try:
                # SingleArticulation exposes gains only through its controller,
                # which takes full-DOF arrays rather than an index subset.
                controller = articulation.get_articulation_controller()
                kps, kds = controller.get_gains()
                kps = np.asarray(kps, dtype=float).copy()
                kds = np.asarray(kds, dtype=float).copy()
                kps[index] = float(reference["stiffness"])
                kds[index] = float(reference["damping"])
                controller.set_gains(kps=kps, kds=kds)
                # The force limit has no runtime setter on SingleArticulation
                # (three plausible names were tried and none exist), so write the
                # USD drive attribute, which is the authority PhysX reads.
                import omni.usd
                from pxr import UsdPhysics

                stage = omni.usd.get_context().get_stage()
                joint_prim = next(
                    (
                        prim
                        for prim in stage.Traverse()
                        if prim.GetName() == spec.joint_names[position]
                    ),
                    None,
                )
                if joint_prim is not None:
                    drive = UsdPhysics.DriveAPI.Get(joint_prim, "linear")
                    if not drive:
                        drive = UsdPhysics.DriveAPI.Apply(joint_prim, "linear")
                    drive.CreateMaxForceAttr().Set(float(reference["maxEffort"]))
                    drive.CreateStiffnessAttr().Set(float(reference["stiffness"]))
                    drive.CreateDampingAttr().Set(float(reference["damping"]))
            except Exception as error:
                print(f"[gripper] could not power {spec.joint_names[position]}: {error}")
                continue
            print(
                f"[gripper] powered {spec.joint_names[position]} from "
                f"{spec.joint_names[strongest]}: stiffness={float(reference['stiffness']):.1f} "
                f"damping={float(reference['damping']):.1f} "
                f"max_effort={float(reference['maxEffort']):.1f}"
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
    graspgen_gripper_config="/home/frlab/Grasp/GraspGenModels/checkpoints/graspgen_robotiq_2f_140.yml",
    graspgen_generator_checkpoint="/home/frlab/Grasp/GraspGenModels/checkpoints/graspgen_robotiq_2f_140_gen.pth",
    make=SingleJointGripper,
)

PANDA_HAND = GripperSpec(
    name="panda_hand",
    joint_names=("panda_finger_joint1", "panda_finger_joint2"),
    # Half-width per finger. The Isaac example uses 0.035 for its 5 cm cubes;
    # this is the joint's full 0.04 because YCB cans are 66 mm across, and at
    # 0.035 the 70 mm opening leaves 2 mm a side — less than the approach's own
    # orientation tolerance, so the fingers hit the can instead of straddling
    # it and the arm stalls short of the grasp.
    open_position=0.04,
    # Isaac's FrankaPinkIKExample offsets panda_hand by this to reach the
    # fingertip midpoint.
    graspgen_depth=0.1034,
    # External model assets are intentionally referenced rather than copied
    # into this repo; the server launcher validates both paths before startup.
    graspgen_gripper_config="/home/frlab/Grasp/GraspGenModels/checkpoints/graspgen_franka_panda.yml",
    graspgen_generator_checkpoint="/home/frlab/Grasp/GraspGenModels/checkpoints/graspgen_franka_panda_gen.pth",
    make=ParallelFingerGripper,
)

GRIPPERS = {g.name: g for g in (ROBOTIQ_2F_140, PANDA_HAND)}


def get_gripper(name: str) -> GripperSpec:
    if name not in GRIPPERS:
        raise KeyError(f"unknown gripper '{name}'; available={sorted(GRIPPERS)}")
    return GRIPPERS[name]
