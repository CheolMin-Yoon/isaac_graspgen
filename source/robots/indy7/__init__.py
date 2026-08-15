"""Indy7 (Neuromeka) with a grafted Robotiq 2F-140 and a D455 wrist camera."""

from __future__ import annotations

import os

from robots.base import RobotSpec
from robots.gripper import ROBOTIQ_2F_140
from robots.indy7.spawn import spawn

ASSETS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")

# The d455 mount and its rsd455.usd reference are already baked into this
# variant at link6/d455; WristCamera only re-poses the mount.
ROBOT_USD = os.path.join(ASSETS_DIR, "indy7_v2_with_2f-140_d455.usd")
KINEMATICS_URDF = os.path.join(ASSETS_DIR, "indy7_kinematics.urdf")


def _make_pink_robot():
    """Build the PINK model from this repo's checked URDF.

    Indy7 is not one of the arms Isaac bundles under ``robot_configurations/``
    (only ``franka`` and ``ur10`` are), so the URDF is authored here and
    verified against the USD by PinkArmIK's startup checks.
    """
    from isaacsim.robot_motion.pink import load_pink_robot

    return load_pink_robot(KINEMATICS_URDF)


SPEC = RobotSpec(
    name="indy7",
    usd_path=ROBOT_USD,
    prim_path="/World/indy7",
    position=(0.0, 0.0, 0.0),
    make_pink_robot=_make_pink_robot,
    kinematics_source=KINEMATICS_URDF,
    ee_link_name="tcp",
    num_arm_dofs=6,
    reach_posture=(0.0, 0.8, -1.6, 0.0, -0.8, 0.0),
    observation_posture=(0.0, 0.8, -1.6, 0.0, -0.8, 0.0),
    # The d455 mount and its rsd455.usd reference are baked into ROBOT_USD
    # under link6; WristCamera only re-poses that mount.
    wrist_camera={"link": "link6", "mode": "asset"},
    gripper=ROBOTIQ_2F_140,
    spawn=spawn,
)
