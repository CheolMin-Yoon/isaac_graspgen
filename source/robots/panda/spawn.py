"""Franka Panda asset spawn."""

from __future__ import annotations

import numpy as np


def spawn(spec):
    """Add the official Franka Panda USD and wrap it as a SingleArticulation.

    Much shorter than the Indy7 equivalent on purpose: the Panda asset ships
    with its hand already attached and its articulation graph intact, so there
    is no gripper subtree to graft. The solver settings are still forced here
    rather than trusted from the asset, for the same reason they are on the
    Indy7 — a composed articulation's self-collision and iteration counts are
    a property of the final plant, not of the asset it came from.
    """
    from pxr import PhysxSchema, Usd, UsdPhysics

    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage

    usd_path = spec.resolve_usd_path()
    prim_path = spec.prim_path

    add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
    stage = get_current_stage()
    root_prim = stage.GetPrimAtPath(prim_path)
    articulation_prim = next(
        (p for p in Usd.PrimRange(root_prim) if p.HasAPI(UsdPhysics.ArticulationRootAPI)),
        None,
    )
    if articulation_prim is None:
        raise RuntimeError(f"No ArticulationRootAPI prim found under {prim_path} in {usd_path}")

    articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(articulation_prim)
    articulation_api.CreateEnabledSelfCollisionsAttr(False)
    articulation_api.CreateSolverPositionIterationCountAttr(16)
    articulation_api.CreateSolverVelocityIterationCountAttr(1)

    return SingleArticulation(
        prim_path=str(articulation_prim.GetPath()),
        name=spec.name,
        position=np.asarray(spec.position, dtype=float),
    )
