#!/usr/bin/env python3
"""Restore the pin-pink version Isaac Sim's pink extension expects.

Isaac Sim 6.0.1 ships pin-pink 4.2.0 in
``~/isaacsim/exts/isaacsim.robot_motion.pink/pip_prebundle``. That directory is
on Isaac's PYTHONPATH, so ``pip install isaaclab`` (which pins pin-pink==3.1.0)
uninstalled it and left 3.1.0 in site-packages. 3.1.0 has no
``pink.exceptions.NoSolutionFound`` and ``isaacsim.robot_motion.pink`` fails to
import, which silently disables every IK path in this workspace.

This installs 4.2.0 (no deps; pinocchio/qpsolvers already exist) into
``<repo>/.pink_overlay``. ``scripts/run_scene.py`` prepends that directory to
PYTHONPATH, so the shared Isaac/IsaacLab site-packages stay untouched.
"""

from __future__ import annotations

import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ISAACSIM_PYTHON = os.path.expanduser("~/isaacsim/python.sh")
OVERLAY = os.path.join(PROJECT_ROOT, ".pink_overlay")
PINK_VERSION = "4.2.0"

os.execv(
    ISAACSIM_PYTHON,
    [ISAACSIM_PYTHON, "-m", "pip", "install", "--no-deps", "--target", OVERLAY, f"pin-pink=={PINK_VERSION}"],
)
