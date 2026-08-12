"""YCB object spawning helpers."""

from __future__ import annotations

import math
import os
import random

import numpy as np


def yaw_to_quat(yaw: float) -> np.ndarray:
    """Convert yaw around world Z to an Isaac wxyz quaternion."""
    half = yaw * 0.5
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)])


def _world_bounds(prim) -> tuple[np.ndarray, np.ndarray]:
    """Return fresh world-aligned render bounds for a USD prim."""
    from pxr import Usd, UsdGeom

    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=True,
    )
    bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    minimum = np.asarray(bounds.GetMin(), dtype=np.float64)
    maximum = np.asarray(bounds.GetMax(), dtype=np.float64)
    return minimum, maximum


def spawn_ycb(cfg: dict, base_position=(0.0, 0.0, 0.0)) -> list[str]:
    """Spawn support-aligned YCB objects, kinematic until grasp approach."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    from isaacsim.core.experimental.utils.semantics import add_labels
    from isaacsim.core.prims import SingleXFormPrim
    from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
    from isaacsim.storage.native import get_assets_root_path

    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("get_assets_root_path() returned None; check Nucleus/assets access")

    objects = list(cfg["objects"])
    if not objects:
        return []
    spawn = cfg["spawn"]
    scale = float(spawn.get("scale", 1.0))

    rng = random.Random(cfg.get("seed", None))
    base = np.asarray(base_position, dtype=float)
    spawned: list[str] = []
    stage = get_current_stage()
    angle_min = float(spawn["angle_min"])
    angle_max = float(spawn["angle_max"])
    radius = float(spawn["radius"])
    support_z = base[2] + float(spawn.get("support_z", 0.0))
    clearance = float(spawn.get("clearance", 0.0))

    for i, obj in enumerate(objects):
        name = str(obj["name"])
        configured_path = str(obj["usd"])
        usd_path = assets_root + configured_path if configured_path.startswith("/Isaac/") else configured_path
        if not configured_path.startswith("/Isaac/") and not os.path.isfile(usd_path):
            raise FileNotFoundError(
                f"local YCB asset is missing: {usd_path}; "
                "run ~/isaacsim/python.sh scripts/prepare_ycb_overlap_assets.py"
            )
        prim_path = f"/World/ycb/obj_{name}"

        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

        fraction = 0.5 if len(objects) == 1 else i / (len(objects) - 1)
        theta = angle_min + fraction * (angle_max - angle_min)
        position = np.array(
            [
                base[0] + radius * math.cos(theta),
                base[1] + radius * math.sin(theta),
                support_z,
            ],
            dtype=float,
        )
        yaw = rng.uniform(-math.pi, math.pi)
        orientation = yaw_to_quat(yaw)

        xform = SingleXFormPrim(
            prim_path=prim_path,
            position=position,
            orientation=orientation,
            scale=np.array([scale, scale, scale]),
        )

        root = stage.GetPrimAtPath(prim_path)
        add_labels(root, labels=name)
        minimum, _ = _world_bounds(root)
        position[2] += support_z + clearance - minimum[2]
        xform.set_world_pose(position=position, orientation=orientation)
        aligned_minimum, _ = _world_bounds(root)

        rigid_body = UsdPhysics.RigidBodyAPI.Apply(root)
        rigid_body.CreateKinematicEnabledAttr(True)
        rigid_body.CreateVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        rigid_body.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        UsdPhysics.MassAPI.Apply(root).CreateMassAttr(float(obj["mass"]))
        mesh_count = 0
        for prim in Usd.PrimRange(root):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set("convexHull")
            mesh_count += 1
        if mesh_count == 0:
            raise RuntimeError(f"YCB asset contains no mesh prim: {usd_path}")

        spawned.append(prim_path)
        print(
            f"[ycb] spawned {name} -> {prim_path} @ {position.round(3).tolist()} "
            f"(bottom_z={aligned_minimum[2]:.6f}, kinematic=True)"
        )

    return spawned


def print_ycb_centers(ycb_paths: list[str]) -> None:
    from isaacsim.core.prims import SingleXFormPrim

    for path in ycb_paths:
        pos, quat = SingleXFormPrim(path).get_world_pose()
        print(
            f"[ycb] center {path}: "
            f"pos={np.asarray(pos).round(4).tolist()}, "
            f"quat={np.asarray(quat).round(4).tolist()}"
        )


def get_world_bounds(prim_path: str) -> tuple[np.ndarray, np.ndarray]:
    """Return the current world-aligned render bounds of a spawned object."""
    from isaacsim.core.utils.stage import get_current_stage

    prim = get_current_stage().GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise ValueError(f"invalid prim path: {prim_path}")
    return _world_bounds(prim)


def set_ycb_kinematic(prim_path: str, enabled: bool) -> None:
    """Freeze an object for observation or release it for grasp/contact physics."""
    from pxr import Gf, UsdPhysics

    from isaacsim.core.utils.stage import get_current_stage

    prim = get_current_stage().GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise ValueError(f"invalid prim path: {prim_path}")
    body = UsdPhysics.RigidBodyAPI(prim)
    body.GetKinematicEnabledAttr().Set(bool(enabled))
    if not enabled:
        body.GetVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        body.GetAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
