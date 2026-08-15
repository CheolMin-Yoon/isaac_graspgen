#!/usr/bin/env python3
"""Launch the grasp scene inside Isaac Sim.

Usage:
    ./scripts/run_scene.py [grasp_scene.py args...]
    ./scripts/run_scene.py --robot indy7 --graspgen --execute-grasp

Sets up the ROS2 environment expected by isaacsim.ros2.bridge, then replaces
itself with Isaac Sim's python running the repo-root ``grasp_scene.py``. The
default Kit options cap the app at 60 Hz, limit CPU worker pools to 8 threads,
and use one GPU. A matching option supplied by the user later on the command
line wins.
"""

from __future__ import annotations

import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ISAACSIM_ROOT = os.path.expanduser("~/isaacsim")
ISAACSIM_ROS2_LIB = os.path.join(ISAACSIM_ROOT, "exts", "isaacsim.ros2.core", "jazzy", "lib")
DEFAULT_CPU_THREADS = "8"
DEFAULT_KIT_ARGS = [
    "--/app/runLoops/main/rateLimitEnabled=true",
    "--/app/runLoops/main/rateLimitFrequency=60",
    "--/renderer/multiGpu/enabled=false",
]


def main() -> None:
    env = dict(os.environ)
    env.setdefault("ISAAC_GRASPGEN_CPU_THREADS", DEFAULT_CPU_THREADS)
    env["ROS_DISTRO"] = "jazzy"
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    # Isaac ships its own jazzy tree and the system has one at /opt/ros/jazzy.
    # Putting both in reach means the same SONAMEs come from two places, which
    # is a known way to make ld.so fail its dl-close assertion. Skip the
    # injection entirely when the bridge is not being started.
    if "--no-ros2" not in sys.argv[1:]:
        prior = env.get("LD_LIBRARY_PATH")
        env["LD_LIBRARY_PATH"] = f"{prior}:{ISAACSIM_ROS2_LIB}" if prior else ISAACSIM_ROS2_LIB
    cpu_threads = env["ISAAC_GRASPGEN_CPU_THREADS"]
    env["PXR_WORK_THREAD_LIMIT"] = cpu_threads
    env["OPENBLAS_NUM_THREADS"] = cpu_threads
    env["OMP_NUM_THREADS"] = cpu_threads
    env["MKL_NUM_THREADS"] = cpu_threads
    kit_args = [
        *DEFAULT_KIT_ARGS,
        f"--/plugins/carb.tasking.plugin/threadCount={cpu_threads}",
        f"--/plugins/omni.tbb.globalcontrol/maxThreadCount={cpu_threads}",
    ]

    launcher = os.path.join(ISAACSIM_ROOT, "python.sh")
    entry = os.path.join(PROJECT_ROOT, "grasp_scene.py")
    os.execve(launcher, [launcher, entry, *kit_args, *sys.argv[1:]], env)


if __name__ == "__main__":
    main()
