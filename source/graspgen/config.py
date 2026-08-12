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

# --- execution prefilter (Robotiq 2F-140 GraspGen convention) ---
# GraspGen returns the gripper base pose.  The official asset places the tool
# center 0.195 m along local +z; the Indy7 TCP is mounted at that base pose.
GRIPPER_DEPTH = 0.195
MIN_GRIPPER_BASE_Z = 0.10
MAX_APPROACH_Z = -0.25
MAX_TOOL_TO_OBJECT_DISTANCE = 0.08
MIN_OUTWARD_APPROACH = 0.20

# --- server (launched by scripts/run_graspgen_server.py) ---
SERVER_ROOT = "/home/frlab/GraspGen"
SERVER_PYTHON = "/home/frlab/anaconda3/envs/graspgen/bin/python"
GRIPPER_CONFIG = "/home/frlab/GraspGenModels/checkpoints/graspgen_robotiq_2f_140.yml"
GENERATOR_CHECKPOINT = "/home/frlab/GraspGenModels/checkpoints/graspgen_robotiq_2f_140_gen.pth"
