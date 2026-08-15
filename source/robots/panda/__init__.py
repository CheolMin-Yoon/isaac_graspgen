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
    # Same values for now: the wrist camera mount was measured at this posture
    # and sees the workspace from it. Separate field so moving one does not
    # silently move the other.
    observation_posture=(0.012, -0.568, 0.0, -2.811, 0.0, 3.037, 0.741),
    # franka.usd ships no camera, so one is created under panda_hand.
    #
    # The orientation is a 180 degree flip about X, not IsaacLab's offset
    # quaternion: IsaacLab's CameraCfg values are expressed in its own ROS
    # camera convention and are converted by its machinery, so dropping the raw
    # numbers into a USD local pose aims the camera at the sky. Measured over
    # the candidates, the ROS-derived quaternions return 0% finite depth while
    # this returns 100%.
    #
    # A USD camera looks along its local -z, so the flip points it along
    # panda_hand's +z — the direction the fingers close toward. Mounted at the
    # hand origin it sees nothing but its own fingers, so it is offset 10 cm to
    # the side and pitched 20 degrees back toward the grasp region. This is
    # X180 composed with a 20 degree rotation about Y.
    #
    # Chosen by measuring how many instance-mask pixels the target occupies,
    # not by eye: at the reach posture this mount gives 43k target pixels while
    # the un-pitched variants give zero. That count is what decides whether
    # GraspGen gets an input at all.
    #
    # Optics are IsaacLab's Franka visuomotor values, which carry no frame
    # convention and so transfer directly.
    wrist_camera={
        "link": "panda_hand",
        "mode": "pinhole",
        "prim_name": "wrist_cam",
        "mount_translation": (0.10, 0.0, -0.05),
        "mount_orientation": (0.0, 0.98481, 0.0, 0.17365),
        "focal_length": 24.0,
        "horizontal_aperture": 20.955,
        "clipping_range": [0.05, 3.0],
        # DLSS renders at half resolution internally and refuses inputs below
        # 300 px; at 640x480 that is 320x240, and the renderer was crashing
        # while reconfiguring around it. 800x600 keeps the DLSS input at
        # 400x300, on the legal side of that limit.
        "resolution": [800, 600],
    },
    gripper=PANDA_HAND,
    spawn=spawn,
)
