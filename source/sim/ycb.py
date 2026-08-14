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


def quat_mul(a, b) -> np.ndarray:
    """Hamilton product of two Isaac wxyz quaternions."""
    aw, ax, ay, az = (float(v) for v in a)
    bw, bx, by, bz = (float(v) for v in b)
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


# +90 deg about X maps the asset's +Y onto world +Z.
_Y_UP_TO_Z_UP = np.array([math.cos(math.pi / 4.0), math.sin(math.pi / 4.0), 0.0, 0.0])


def up_axis_correction(up_axis: str) -> np.ndarray:
    """Rotation standing an asset authored ``up_axis``-up onto the Z-up floor.

    Isaac's ``/Isaac/Props/YCB/Axis_Aligned/`` meshes declare ``upAxis=Z`` in
    stage metadata but are geometrically Y-up — a tomato soup can measures
    66 mm across X and Z and 101 mm along Y. Referencing such a layer does not
    rotate it, so without this correction every one of those objects is spawned
    lying on its side, and the scene only *looks* fine while the objects are
    pinned kinematic.
    """
    if up_axis == "Z":
        return np.array([1.0, 0.0, 0.0, 0.0])
    if up_axis == "Y":
        return _Y_UP_TO_Z_UP
    raise ValueError(f"unsupported up_axis {up_axis!r}; expected 'Y' or 'Z'")


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


def spawn_ycb(
    cfg: dict,
    base_position=(0.0, 0.0, 0.0),
    kinematic: bool = True,
    collision_approximation: str | None = None,
) -> list[str]:
    """Spawn support-aligned YCB objects.

    ``kinematic=True`` pins every object so the observation phase is
    reproducible. That pin also hides whether the spawn pose is physically
    stable at all — the alignment uses render bounds while the collider is a
    per-mesh convex hull, and the two need not agree. Pass ``kinematic=False``
    to let physics judge the placement.

    ``collision_approximation`` overrides the per-object setting for every
    object, for single-variable A/B tests. A convex hull is wrong for concave
    geometry: a bowl's hull is a solid dome, so it rolls instead of resting.
    """
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
        # Isaac's YCB props are geometrically Y-up despite declaring upAxis=Z,
        # so they must be stood up before the yaw is applied. Assets converted
        # by scripts/prepare_ycb_overlap_assets.py are genuinely Z-up.
        up_axis = str(obj.get("up_axis", "Y" if configured_path.startswith("/Isaac/") else "Z"))
        yaw = rng.uniform(-math.pi, math.pi)
        orientation = quat_mul(yaw_to_quat(yaw), up_axis_correction(up_axis))

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
        rigid_body.CreateKinematicEnabledAttr(bool(kinematic))
        rigid_body.CreateVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        rigid_body.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        UsdPhysics.MassAPI.Apply(root).CreateMassAttr(float(obj["mass"]))
        approximation = collision_approximation or str(
            obj.get("collision", spawn.get("collision", cfg.get("collision", "convexHull")))
        )
        mesh_count = 0
        for prim in Usd.PrimRange(root):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set(approximation)
            mesh_count += 1
        if mesh_count == 0:
            raise RuntimeError(f"YCB asset contains no mesh prim: {usd_path}")

        spawned.append(prim_path)
        print(
            f"[ycb] spawned {name} -> {prim_path} @ {position.round(3).tolist()} "
            f"(bottom_z={aligned_minimum[2]:.6f}, up_axis={up_axis}, "
            f"kinematic={bool(kinematic)}, collision={approximation})"
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


def ycb_pose_snapshot(ycb_paths: list[str]) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Capture world poses so a later call can measure how far physics moved them."""
    from isaacsim.core.prims import SingleXFormPrim

    snapshot = {}
    for path in ycb_paths:
        pos, quat = SingleXFormPrim(path).get_world_pose()
        snapshot[path] = (np.asarray(pos, dtype=float), np.asarray(quat, dtype=float))
    return snapshot


def _rotation_angle_degrees(quat0, quat1) -> float:
    """Magnitude of the rotation carrying ``quat0`` to ``quat1``, in degrees.

    Deliberately measured relative to the spawn pose rather than against world
    +z: once an asset is stood up by ``up_axis_correction`` its local +z is no
    longer its geometric up, so any "angle from vertical" reading would be an
    artifact of the asset's authoring convention. "How far did it rotate from
    where I placed it" is the question that survives that.
    """
    a = np.asarray(quat0, dtype=float)
    b = np.asarray(quat1, dtype=float)
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    return float(np.degrees(2.0 * np.arccos(np.clip(abs(float(np.dot(a, b))), 0.0, 1.0))))


def report_ycb_drift(before: dict, after: dict, label: str = "drift") -> None:
    """Print how far each object translated and rotated away from its spawn pose."""
    print(f"[ycb] --- {label}: displacement / rotation from spawn pose ---")
    worst = 0.0
    unstable = []
    for path, (pos0, quat0) in before.items():
        if path not in after:
            continue
        pos1, quat1 = after[path]
        moved = float(np.linalg.norm(pos1 - pos0))
        rotated = _rotation_angle_degrees(quat0, quat1)
        worst = max(worst, rotated)
        flag = ""
        if rotated > 15.0:
            flag = "  <== TIPPED"
            unstable.append(path.rsplit("/", 1)[-1])
        elif moved > 0.01:
            flag = "  <== SLID"
            unstable.append(path.rsplit("/", 1)[-1])
        print(
            f"[ycb] {path.rsplit('/', 1)[-1]:28s} "
            f"moved={moved * 1000:7.2f}mm  rotated={rotated:7.2f}deg{flag}"
        )
    print(f"[ycb] --- worst rotation: {worst:.2f} deg | unstable: {len(unstable)}/{len(before)} ---")
    if unstable:
        print(f"[ycb] --- {', '.join(unstable)} ---")


def settle_ycb(
    world,
    ycb_paths: list[str],
    *,
    max_steps: int = 250,
    position_tol: float = 1e-4,
    rotation_tol: float = 0.05,
    pin: bool = True,
) -> int | None:
    """Step physics until the objects stop moving, then optionally freeze them.

    Freezing a settled pose and freezing a hand-authored pose are
    indistinguishable in a screenshot and are not the same thing: only the
    first is a state the solver agrees with. Isaac's own
    ``volume_stack_randomizer`` ends its drop the same way, disabling rigid
    body dynamics once the assets have come to rest.

    Returns the step the scene settled on, or None if it never did — that
    return value is the signal that the placement is still wrong.
    """
    if not ycb_paths:
        return 0

    previous = ycb_pose_snapshot(ycb_paths)
    moved = rotated = float("inf")
    settled_at = None
    for step in range(1, max_steps + 1):
        world.step(render=False)
        current = ycb_pose_snapshot(ycb_paths)
        moved = max(float(np.linalg.norm(current[p][0] - previous[p][0])) for p in ycb_paths)
        rotated = max(_rotation_angle_degrees(previous[p][1], current[p][1]) for p in ycb_paths)
        previous = current
        if moved < position_tol and rotated < rotation_tol:
            settled_at = step
            break

    if settled_at is None:
        print(
            f"[ycb] settle: NOT at rest after {max_steps} steps "
            f"(last step moved {moved * 1000:.3f}mm, rotated {rotated:.3f}deg) — "
            "the spawn placement is still unstable"
        )
    else:
        print(f"[ycb] settle: at rest after {settled_at} steps")

    if pin:
        for path in ycb_paths:
            set_ycb_kinematic(path, True)
        print(f"[ycb] settle: pinned {len(ycb_paths)} objects at their settled pose")
    return settled_at


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
