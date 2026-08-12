#!/usr/bin/env python3
"""Install the GraspGen client deps into Isaac Sim's python."""

from __future__ import annotations

import os

ISAACSIM_PYTHON = os.path.expanduser("~/isaacsim/python.sh")

os.execv(ISAACSIM_PYTHON, [ISAACSIM_PYTHON, "-m", "pip", "install", "pyzmq", "msgpack", "msgpack-numpy"])
