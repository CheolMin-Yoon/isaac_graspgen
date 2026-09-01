"""GraspGen client/server configuration."""

from __future__ import annotations

# --- client (Isaac side) ---
HOST = "127.0.0.1"
PORT = 5556
TIMEOUT_MS = 60_000

TRIGGER_STEP = 180      # simulation step at which GraspGen is called once
POINT_COUNT = 2048      # points sent to the server
NUM_GRASPS = 200
# Candidates the server returns, ranked by its own confidence. Sampling stays at
# NUM_GRASPS either way, so a larger TOPK costs nothing at inference and only
# widens the pool the geometric selection gets to choose from. At 20 the pool
# regularly held a single gate-passing grasp, which left no choice to make.
TOPK = 60
SEED = 0

# --- execution prefilter (scene policy, not gripper geometry) ---
# GraspGen returns the gripper base pose; the tool center is base + depth*z,
# where the depth is a property of the gripper and so lives on its GripperSpec
# in ``robots/gripper.py``.  The gates below encode a table-mounted arm
# reaching outward, and are retuned per scene rather than per gripper.
MIN_GRIPPER_BASE_Z = 0.10
MAX_APPROACH_Z = -0.25
MAX_TOOL_TO_OBJECT_DISTANCE = 0.08
# Heading test for the approach, as a cosine against the outward radial
# direction, applied only when the approach has a real heading to speak of.
#
# The previous form gated the *unnormalised* dot product at 0.20, which coupled
# the heading requirement to how steeply the grasp came down: a top-down grasp
# has a horizontal component of about 0.5 whatever its azimuth, so the test
# demanded it also lean outward by 40 degrees. Measured on one can, that
# discarded 57 of 60 candidates, and the discarded pool contained a grasp
# centred to 0.1mm while the best survivor was 6.1mm off.
#
# What the gate is actually for is refusing to approach a table-mounted arm's
# workspace from behind the object. Reject only headings that point back toward
# the base more than they point away; a grasp coming almost straight down has no
# meaningful heading, so do not ask it for one.
MIN_OUTWARD_COS = -0.5
MIN_HEADING_MAGNITUDE = 0.15
# m, how far the jaws' centre may sit from the object's midline along the axis
# they close on. GraspGen's confidence does not rank this: measured on one can,
# the top-ranked candidate was 22mm off while #2 and #3 were within 5mm, and the
# 66mm can inside an 80mm grip only has 7mm a side. Without this gate the choice
# among equally confident candidates is a coin flip that decides the grasp.
MAX_CLOSING_AXIS_OFFSET = 0.008

# --- server (launched by scripts/run_graspgen_server.py) ---
# NOTE: SERVER_ROOT is the upstream NVIDIA GraspGen checkout — a *different*
# repo from this workspace.  This workspace is /home/frlab/Grasp/isaac_graspgen
# and only speaks to that server over ZMQ; do not conflate the two paths.
SERVER_ROOT = "/home/frlab/Grasp/GraspGen"
SERVER_PYTHON = "/home/frlab/anaconda3/envs/graspgen/bin/python"
# Per-gripper checkpoints live on the GripperSpec (robots/gripper.py) because
# GraspGen is trained per gripper, not per arm.
