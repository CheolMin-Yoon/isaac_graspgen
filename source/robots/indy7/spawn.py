"""Indy7 asset spawn helpers."""

from __future__ import annotations

import numpy as np


def spawn(spec):
    """Add the Indy7 USD reference and wrap it as a SingleArticulation.

    The combined asset USDs (gripper/camera variants) nest the actual
    ArticulationRootAPI prim one or more levels below their defaultPrim, so
    ``spec.prim_path`` itself is never the articulation root after referencing
    — the root is found by scanning the referenced subtree instead of assuming
    it sits exactly at ``spec.prim_path``.
    """
    usd_path = spec.resolve_usd_path()
    prim_path = spec.prim_path

    import omni.usd
    from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
    from isaacsim.robot_setup.assembler.robot_assembler import set_opposite_body_transform
    from isaacsim.storage.native import get_assets_root_path

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

    # The gripper embedded in the legacy combined asset mixes the Z-axis
    # Robotiq configuration with only one rotX mimic relation. Replace only
    # that subtree with the exact configured Robotiq graph used by Isaac Sim's
    # official UR10e gripper example (X-axis joints + five mimic relations).
    old_gripper = next(
        (p for p in Usd.PrimRange(root_prim) if p.GetName() == "Robotiq_2F_140_config"),
        None,
    )
    tcp_prim = next(
        (
            p
            for p in Usd.PrimRange(root_prim)
            if p.GetName() == "tcp" and p.HasAPI(UsdPhysics.RigidBodyAPI)
        ),
        None,
    )
    if old_gripper is None or tcp_prim is None:
        raise RuntimeError("combined Indy7 asset is missing its Robotiq subtree or rigid TCP")

    official_asset = (
        get_assets_root_path()
        + "/Isaac/Samples/Rigging/Manipulator/configure_manipulator/ur10e/ur/ur_gripper.usd"
    )
    official_root_path = f"{prim_path}/Robotiq_2F_140_official"
    official_root = stage.DefinePrim(official_root_path, "Xform")
    official_root.GetReferences().AddReference(official_asset, Sdf.Path("/ur/ee_link"))
    official_base = stage.GetPrimAtPath(f"{official_root_path}/robotiq_arg2f_base_link")
    if not official_base.IsValid():
        raise RuntimeError(f"failed to compose official Robotiq subtree from {official_asset}")

    # Align the official gripper base frame to the live Indy7 TCP frame.
    target_world = omni.usd.get_world_transform_matrix(tcp_prim)
    base_local = omni.usd.get_local_transform_matrix(official_base)
    parent_world = omni.usd.get_world_transform_matrix(official_root.GetParent())
    root_local = base_local.GetInverse() * target_world * parent_world.GetInverse()
    root_xform = UsdGeom.Xformable(official_root)
    root_xform.ClearXformOpOrder()
    root_xform.AddTransformOp().Set(root_local)
    aligned_world = omni.usd.get_world_transform_matrix(official_base)
    alignment_error = float(np.linalg.norm(np.asarray(aligned_world) - np.asarray(target_world)))
    print(f"[gripper] official base-to-TCP alignment error={alignment_error:.3e}")

    # Disable the fixed joint that belonged to UR10e and weld the same base
    # rigid body to Indy7's TCP instead.
    old_fixed_joint = stage.GetPrimAtPath(
        f"{official_root_path}/robotiq_arg2f_base_link/AssemblerFixedJoint"
    )
    if old_fixed_joint.IsValid():
        old_fixed_joint.SetActive(False)
    fixed_joint = UsdPhysics.FixedJoint.Define(
        stage,
        f"{official_root_path}/robotiq_arg2f_base_link/Indy7AssemblerFixedJoint",
    )
    fixed_joint.GetBody0Rel().SetTargets([tcp_prim.GetPath()])
    fixed_joint.GetBody1Rel().SetTargets([official_base.GetPath()])
    cache = UsdGeom.XformCache()
    set_opposite_body_transform(
        stage,
        cache,
        fixed_joint.GetPrim(),
        body0base=False,
        fixpos=True,
        fixrot=True,
    )
    set_opposite_body_transform(
        stage,
        cache,
        fixed_joint.GetPrim(),
        body0base=True,
        fixpos=True,
        fixrot=True,
    )

    old_gripper.SetActive(False)

    return SingleArticulation(
        prim_path=str(articulation_prim.GetPath()),
        name=spec.name,
        position=np.asarray(spec.position, dtype=float),
    )
