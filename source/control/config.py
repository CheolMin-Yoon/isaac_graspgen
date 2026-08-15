"""Grasp execution parameters.

GraspGen returns world-frame 4x4 gripper poses following the convention used
at training time: +z is the approach direction and the origin sits at the
gripper base frame. If grasps look offset along the approach axis, tune
TCP_OFFSET first — it maps the GraspGen gripper frame onto the arm's TCP frame
tracked by ``robots.arm_ik.PinkArmIK``.
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

# Phase transition thresholds.
#
# These are set from the task's clearance, not from what the arm finds easy. A
# 66 mm can inside an 80 mm grip leaves 7 mm a side, so a gate that passes at
# 10 mm admits a guaranteed miss -- and it did: the arm tracked to 4.3 mm and
# 0.058 rad, well inside the old gates, and the fingers still closed 14 mm off
# the can's axis.
#
# Orientation is the dominant term, which is not obvious. The fingertips sit
# 103 mm beyond ``panda_hand``, the frame the IK actually tracks, so an
# orientation error is levered out to 103 mm x theta at the point that has to
# clear the can: 0.058 rad became 6.0 mm there, larger than the whole position
# error. Budget the two together against the 7 mm: 4 mm + 103 mm x 0.025 rad
# leaves ~1.4 mm of margin.
POSITION_TOL = 0.004    # m, TCP-to-target distance to consider a phase reached
# rad, TCP-to-target angle. A phase used to advance on position alone, which
# let the arm enter approach with the hand still turned away from the grasp:
# the fingers then swept the object aside and closed on nothing (measured with
# the Panda, whose 7th DOF lets orientation lag position by hundreds of steps).
ORIENTATION_TOL = 0.025
SETTLE_STEPS = 30       # consecutive in-tolerance steps before advancing
CLOSE_STEPS = 90        # steps to hold the close command before lifting
# A phase gives up when it stops making progress, not when a step budget runs
# out. A fixed budget forces a number that is wrong in both directions: too low
# kills a slow-but-converging arm (the Panda's orientation legitimately takes
# ~960 steps), too high spends that long confirming a failure that was already
# decided. Progress is self-scaling — a fast arm fails fast, a slow one is left
# alone as long as it keeps closing.
STALL_STEPS = 240           # steps with no improvement before declaring failure
STALL_MIN_IMPROVEMENT = 1e-4  # m / rad; smaller than this is noise, not progress
# Absolute backstop, only for a phase that somehow keeps inching forever.
MAX_PHASE_STEPS = 12_000
