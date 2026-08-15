"""Bootstrap the standalone robot + YCB + GraspGen scene.

Isaac requires ``SimulationApp`` to exist before most of its modules are
imported. The process boundary stays here; CLI parsing and runtime orchestration
live in ``sim.cli`` and ``sim.runner`` respectively.

Examples:
    ./scripts/run_scene.py
    ./scripts/run_scene.py --robot indy7
    ./scripts/run_scene.py --robot panda --ycb-only 005_tomato_soup_can \
        --ycb-radius 0.55 --graspgen --execute-grasp
"""

from __future__ import annotations

import os
import sys

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
SOURCE_DIR = os.path.join(ROOT_DIR, "source")
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

from sim.cli import parse_args


def _cpu_thread_limit() -> int:
    value = int(os.environ.get("ISAAC_GRASPGEN_CPU_THREADS", "8"))
    if value < 1:
        raise ValueError("ISAAC_GRASPGEN_CPU_THREADS must be >= 1")
    return value


def main() -> None:
    args = parse_args()

    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {
            "headless": args.headless,
            "limit_cpu_threads": _cpu_thread_limit(),
            "multi_gpu": False,
        }
    )
    try:
        # This import must remain after SimulationApp construction.
        from sim.runner import GraspSceneRunner

        GraspSceneRunner(args, simulation_app, ROOT_DIR).run()
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
