"""Scene configuration: YCB objects, wrist camera, timing.

Robot-specific settings (USD, prim path, base pose, kinematics) live in
``robots/<name>``, not here — this module describes the world the arm works in.
"""

from __future__ import annotations

import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

YCB_CONFIG = {
    "objects": [
        {
            "name": "001_chips_can",
            "usd": os.path.join(PROJECT_ROOT, "source", "assets", "ycb_overlap", "001_chips_can.usd"),
            "mass": 0.205,
        },
        {
            "name": "002_master_chef_can",
            "usd": "/Isaac/Props/YCB/Axis_Aligned/002_master_chef_can.usd",
            "mass": 0.414,
        },
        {
            "name": "005_tomato_soup_can",
            "usd": "/Isaac/Props/YCB/Axis_Aligned/005_tomato_soup_can.usd",
            "mass": 0.349,
        },
        {
            "name": "007_tuna_fish_can",
            "usd": "/Isaac/Props/YCB/Axis_Aligned/007_tuna_fish_can.usd",
            "mass": 0.171,
        },
        {
            "name": "010_potted_meat_can",
            "usd": "/Isaac/Props/YCB/Axis_Aligned/010_potted_meat_can.usd",
            "mass": 0.370,
        },
        {
            "name": "006_mustard_bottle",
            "usd": "/Isaac/Props/YCB/Axis_Aligned/006_mustard_bottle.usd",
            "mass": 0.404,
        },
        {
            "name": "021_bleach_cleanser",
            "usd": "/Isaac/Props/YCB/Axis_Aligned/021_bleach_cleanser.usd",
            "mass": 0.479,
        },
        {
            "name": "022_windex_bottle",
            "usd": os.path.join(PROJECT_ROOT, "source", "assets", "ycb_overlap", "022_windex_bottle.usd"),
            "mass": 1.022,
        },
        {
            "name": "024_bowl",
            "usd": "/Isaac/Props/YCB/Axis_Aligned/024_bowl.usd",
            "mass": 0.147,
        },
        {
            "name": "025_mug",
            "usd": "/Isaac/Props/YCB/Axis_Aligned/025_mug.usd",
            "mass": 0.118,
        },
        {
            "name": "032_knife",
            "usd": os.path.join(PROJECT_ROOT, "source", "assets", "ycb_overlap", "032_knife.usd"),
            "mass": 0.031,
        },
    ],
    "spawn": {
        "radius": 0.70,
        "angle_min": -1.15,
        "angle_max": 1.15,
        "support_z": 0.0,
        "clearance": 0.0,
        "scale": 1.0,
    },
    # A convex hull seals concave geometry shut — the bowl becomes a solid dome
    # and rolls onto its side. Measured over 300 dynamic steps, hull vs
    # decomposition (both with the up-axis correction applied) is 3 objects
    # tipped vs 0. NVIDIA's own Axis_Aligned_Physics mustard bottle ships
    # convexDecomposition for the same reason.
    "collision": "convexDecomposition",
    "seed": 0,
}

CAMERA_CONFIG = {
    # The d455 mount and its rsd455.usd reference are already baked into the
    # robot USD under ``spec.wrist_camera_link`` — WristCamera only re-poses
    # the mount and wraps it.
    "prim_name": "d455",
    "asset_root_name": "RSD455",
    "color_camera": "Camera_OmniVision_OV9782_Color",
    "depth_camera": "Camera_Pseudo_Depth",
    "resolution": [640, 480],
    "mount_translation": [0.06031178594972571, 0.002541999360932995, 0.03876863710717515],
    "mount_orientation": [0.7071067811865476, 0.0, -0.7071067811865475, 0.0],
    "clipping_range": [0.01, 5.0],
    "capture_depth": True,
    "capture_every": 60,
    "output_dir": "output/camera",
}

PHYSICS_HZ = 240
RENDERING_HZ = 60
CONTROL_HZ = RENDERING_HZ

# World.step(render=True) advances four PhysX substeps, renders once, and then
# the entrypoint applies one IK/control update. This keeps contact at 240 Hz
# without driving the camera and controller faster than 60 Hz.
PHYSICS_DT = 1.0 / PHYSICS_HZ
RENDERING_DT = 1.0 / RENDERING_HZ
