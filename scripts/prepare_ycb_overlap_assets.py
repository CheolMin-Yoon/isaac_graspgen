#!/usr/bin/env python3
"""Build the three YCB overlap assets absent from Isaac Sim 6.0."""

from __future__ import annotations

import argparse
import asyncio
import os
import shutil
import tarfile
import tempfile
import urllib.request

from isaacsim import SimulationApp


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "source", "assets", "ycb_overlap")
YCB_BASE_URL = "https://ycb-benchmarks.s3.amazonaws.com/data"
SOURCES = {
    "001_chips_can": (
        f"{YCB_BASE_URL}/berkeley/001_chips_can/001_chips_can_berkeley_meshes.tgz",
        "001_chips_can/poisson/nontextured.stl",
    ),
    "022_windex_bottle": (
        f"{YCB_BASE_URL}/google/022_windex_bottle_google_16k.tgz",
        "022_windex_bottle/google_16k/nontextured.stl",
    ),
    "032_knife": (
        f"{YCB_BASE_URL}/google/032_knife_google_16k.tgz",
        "032_knife/google_16k/nontextured.stl",
    ),
}


def download_sources(directory: str) -> dict[str, str]:
    """Download official YCB archives and extract one non-textured STL each."""
    extracted: dict[str, str] = {}
    for name, (url, member) in SOURCES.items():
        archive_path = os.path.join(directory, f"{name}.tgz")
        stl_path = os.path.join(directory, f"{name}.stl")
        print(f"[ycb-assets] downloading {name}: {url}")
        urllib.request.urlretrieve(url, archive_path)
        with tarfile.open(archive_path, "r:gz") as archive:
            source = archive.extractfile(member)
            if source is None:
                raise FileNotFoundError(f"{member!r} not found in {archive_path}")
            with open(stl_path, "wb") as destination:
                shutil.copyfileobj(source, destination)
        extracted[name] = stl_path
    return extracted


async def convert_asset(input_path: str, output_path: str) -> None:
    import omni.kit.asset_converter

    context = omni.kit.asset_converter.AssetConverterContext()
    context.ignore_materials = True
    context.single_mesh = True
    context.merge_all_meshes = True
    context.smooth_normals = True
    context.use_meter_as_world_unit = True
    context.create_world_as_default_root_prim = False

    task = omni.kit.asset_converter.get_instance().create_converter_task(
        input_path,
        output_path,
        None,
        context,
    )
    if not await task.wait_until_finished():
        raise RuntimeError(f"asset conversion failed: {input_path} -> {output_path}")


def normalize_usd(path: str) -> None:
    """Make the generated layer referenceable with meter/Z-up metadata."""
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(path)
    if stage is None:
        raise RuntimeError(f"failed to open generated USD: {path}")
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    if not stage.GetDefaultPrim().IsValid():
        roots = stage.GetPseudoRoot().GetChildren()
        if len(roots) != 1:
            raise RuntimeError(f"expected one generated root prim in {path}, got {len(roots)}")
        stage.SetDefaultPrim(roots[0])
    stage.GetRootLayer().Save()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--force", action="store_true", help="overwrite existing generated USD files")
    args, _ = parser.parse_known_args()

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="isaac-indy7-ycb-") as temporary_dir:
        sources = download_sources(temporary_dir)
        app = SimulationApp({"headless": True})
        try:
            from isaacsim.core.experimental.utils.app import enable_extension

            enable_extension("omni.kit.asset_converter")
            loop = asyncio.get_event_loop()
            for name, source_path in sources.items():
                output_path = os.path.join(OUTPUT_DIR, f"{name}.usd")
                if os.path.exists(output_path) and not args.force:
                    print(f"[ycb-assets] exists, skipping: {output_path}")
                    continue
                print(f"[ycb-assets] converting {name} -> {output_path}")
                loop.run_until_complete(convert_asset(source_path, output_path))
                normalize_usd(output_path)
        finally:
            app.close()


if __name__ == "__main__":
    main()
