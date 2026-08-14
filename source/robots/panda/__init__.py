"""Franka Panda with its own hand, from Isaac's official assets.

Unlike the Indy7, nothing here is authored in this repo: the USD, the
kinematics URDF and the controller gains all come from Isaac Sim 6.0.1. The
values below are taken from the official ``FrankaPinkIKExample``
(``exts/isaacsim.robot_motion.pink.examples/.../ik_controller/scenario.py``),
whose gains happen to be the ones ``PinkArmIK`` already uses.

Known gaps, both tracked in ``robots/panda/README.md``:
  * ``franka.usd`` carries no wrist camera mount, so ``wrist_camera_link`` is
    None and ``--graspgen`` is unavailable for this arm.
  * The GraspGen Franka checkpoints are not downloaded yet.
"""

from __future__ import annotations

from robots.base import RobotSpec
from robots.gripper import PANDA_HAND
from robots.panda.spawn import spawn

# Bundled with the PINK extension under robot_configurations/franka/, which
# also supplies the SRDF and package dirs — hence the loader rather than a path.
KINEMATICS_SOURCE = "bundled:franka"


def _make_pink_robot():
    from isaacsim.robot_motion.pink import load_pink_supported_robot

    return load_pink_supported_robot("franka")


SPEC = RobotSpec(
    name="panda",
    usd_path="/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
    prim_path="/World/panda",
    position=(0.0, 0.0, 0.0),
    make_pink_robot=_make_pink_robot,
    kinematics_source=KINEMATICS_SOURCE,
    ee_link_name="panda_hand",
    num_arm_dofs=7,
    # Official example's nominal reach posture, arm joints only — the finger
    # entries in that dict belong to the gripper.
    reach_posture=(0.012, -0.568, 0.0, -2.811, 0.0, 3.037, 0.741),
    wrist_camera_link=None,
    gripper=PANDA_HAND,
    spawn=spawn,
)
