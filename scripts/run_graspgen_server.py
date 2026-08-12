#!/usr/bin/env python3
"""Launch the GraspGen ZMQ server (paths/ports in source/graspgen/config.py).

Usage:
    ./scripts/run_graspgen_server.py [graspgen_server.py args...]
"""

from __future__ import annotations

import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "source"))

from graspgen import config  # noqa: E402


def main() -> None:
    for required in (config.GRIPPER_CONFIG, config.GENERATOR_CHECKPOINT):
        if not os.path.isfile(required):
            sys.exit(f"Missing GraspGen file: {required}")

    server = os.path.join(config.SERVER_ROOT, "client-server", "graspgen_server.py")
    os.chdir(config.SERVER_ROOT)
    os.execv(
        config.SERVER_PYTHON,
        [
            config.SERVER_PYTHON,
            server,
            "--gripper_config",
            config.GRIPPER_CONFIG,
            "--host",
            config.HOST,
            "--port",
            str(config.PORT),
            *sys.argv[1:],
        ],
    )


if __name__ == "__main__":
    main()
