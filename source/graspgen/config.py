"""GraspGen client/server configuration."""

from __future__ import annotations

# --- client (Isaac side) ---
HOST = "127.0.0.1"
PORT = 5556
TIMEOUT_MS = 60_000

TRIGGER_STEP = 180      # simulation step at which GraspGen is called once
POINT_COUNT = 2048      # points sent to the server
NUM_GRASPS = 200
TOPK = 20
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

# --- server (launched by scripts/run_graspgen_server.py) ---
# NOTE: SERVER_ROOT is the upstream NVIDIA GraspGen checkout — a *different*
# repo from this workspace.  This workspace is /home/frlab/isaac_graspgen and
# only speaks to that server over ZMQ; do not conflate the two paths.
SERVER_ROOT = "/home/frlab/GraspGen"
SERVER_PYTHON = "/home/frlab/anaconda3/envs/graspgen/bin/python"
# Per-gripper checkpoints live on the GripperSpec (robots/gripper.py) because
# GraspGen is trained per gripper, not per arm.
