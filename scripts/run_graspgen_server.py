#!/usr/bin/env python3
"""Launch the GraspGen ZMQ server (host/port in source/graspgen/config.py).

The checkpoints are chosen by gripper, not by arm — GraspGen is trained per
gripper — so the pair comes from the GripperSpec in source/robots/gripper.py.

Usage:
    ./scripts/run_graspgen_server.py [graspgen_server.py args...]
    ./scripts/run_graspgen_server.py --gripper robotiq_2f_140
"""

from __future__ import annotations

import argparse
import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "source"))

from graspgen import config  # noqa: E402
from robots.gripper import GRIPPERS, get_gripper  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--gripper", default="robotiq_2f_140", choices=sorted(GRIPPERS))
    args, passthrough = parser.parse_known_args()
    gripper = get_gripper(args.gripper)

    for required in (gripper.graspgen_gripper_config, gripper.graspgen_generator_checkpoint):
        if not os.path.isfile(required):
            sys.exit(f"Missing GraspGen file for {gripper.name}: {required}")

    server = os.path.join(config.SERVER_ROOT, "client-server", "graspgen_server.py")
    os.chdir(config.SERVER_ROOT)
    os.execv(
        config.SERVER_PYTHON,
        [
            config.SERVER_PYTHON,
            server,
            "--gripper_config",
            gripper.graspgen_gripper_config,
            "--host",
            config.HOST,
            "--port",
            str(config.PORT),
            *passthrough,
        ],
    )


if __name__ == "__main__":
    main()
