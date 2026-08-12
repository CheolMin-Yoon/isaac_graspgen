"""Grasp execution parameters.

GraspGen returns world-frame 4x4 gripper poses following the Robotiq 2F-140
convention used at training time: +z is the approach direction and the origin
sits at the gripper base frame. If grasps look offset along the approach axis,
tune TCP_OFFSET first — it maps the GraspGen gripper frame onto the Indy7 TCP
frame used by ``control.ik.Indy7IK``.
"""

from __future__ import annotations

# distance (m) behind the grasp pose, along -approach, where the TCP goes first
PREGRASP_OFFSET = 0.10
# extra travel (m) along +approach from the grasp pose; >0 moves deeper
GRASP_DEPTH_OFFSET = 0.0
# offset (m) along the grasp +z mapping GraspGen's gripper base to the TCP
TCP_OFFSET = 0.0
# lift height (m) straight up in world frame after closing
LIFT_HEIGHT = 0.15

# phase transition thresholds
POSITION_TOL = 0.01     # m, TCP-to-target distance to consider a phase reached
SETTLE_STEPS = 30       # consecutive in-tolerance steps before advancing
CLOSE_STEPS = 90        # steps to hold the close command before lifting
TIMEOUT_STEPS = 600     # per-phase step budget before aborting as unreachable
