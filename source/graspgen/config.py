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
MIN_OUTWARD_APPROACH = 0.20
# m, how far the jaws' centre may sit from the object's midline along the axis
# they close on. GraspGen's confidence does not rank this: measured on one can,
# the top-ranked candidate was 22mm off while #2 and #3 were within 5mm, and the
# 66mm can inside an 80mm grip only has 7mm a side. Without this gate the choice
# among equally confident candidates is a coin flip that decides the grasp.
MAX_CLOSING_AXIS_OFFSET = 0.008

# --- server (launched by scripts/run_graspgen_server.py) ---
# NOTE: SERVER_ROOT is the upstream NVIDIA GraspGen checkout — a *different*
# repo from this workspace.  This workspace is /home/frlab/isaac_graspgen and
# only speaks to that server over ZMQ; do not conflate the two paths.
SERVER_ROOT = "/home/frlab/GraspGen"
SERVER_PYTHON = "/home/frlab/anaconda3/envs/graspgen/bin/python"
# Per-gripper checkpoints live on the GripperSpec (robots/gripper.py) because
# GraspGen is trained per gripper, not per arm.
