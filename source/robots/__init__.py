"""Robot registry.

``get_robot(name)`` is the only way the scene learns which arm it is driving.
Robots are imported lazily so that importing this package stays free of Isaac
Sim — the specs themselves are plain data, but a robot module may pull in
assets or helpers that are not worth loading for the arm you did not pick.

To add an arm, see ``robots/panda/README.md``.
"""

from __future__ import annotations

from importlib import import_module

from robots.base import GripperSpec, RobotSpec

# name -> module exporting SPEC
_ROBOT_MODULES = {
    "panda": "robots.panda",
    "indy7": "robots.indy7",
}

DEFAULT_ROBOT = "panda"

__all__ = ["DEFAULT_ROBOT", "GripperSpec", "RobotSpec", "available_robots", "get_robot"]


def available_robots() -> list[str]:
    return sorted(_ROBOT_MODULES)


def get_robot(name: str) -> RobotSpec:
    if name not in _ROBOT_MODULES:
        raise KeyError(
            f"unknown robot '{name}'; available={available_robots()}. "
            "See source/robots/panda/README.md for what a new arm must provide."
        )

    spec = import_module(_ROBOT_MODULES[name]).SPEC
    if spec.name != name:
        raise RuntimeError(f"robot module '{_ROBOT_MODULES[name]}' exports SPEC.name={spec.name!r}")
    return spec
