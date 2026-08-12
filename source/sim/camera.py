"""Indy7 wrist camera helper."""

from __future__ import annotations

import os

import numpy as np
from isaacsim.sensors.camera import Camera

try:
    from PIL import Image

    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False


class WristCamera:
    """Wrap the D455 camera already referenced under the Indy7 wrist (``link6/d455``
    in the spawned USD) and save RGB/depth frames.

    The rsd455.usd reference and its mount transform are baked into the USD
    asset itself, so this only re-poses the mount and wraps the resulting
    color/depth camera prims — it never authors a new reference.
    """

    def __init__(self, cfg: dict, parent_prim: str, workdir: str = ".") -> None:
        from pxr import UsdPhysics

        from isaacsim.core.utils.stage import get_current_stage

        self._cfg = cfg
        self._every = int(cfg.get("capture_every", 60))
        self._out_dir = os.path.join(workdir, cfg.get("output_dir", "output/camera"))
        self._rgb_dir = os.path.join(self._out_dir, "rgb")
        self._depth_dir = os.path.join(self._out_dir, "depth")
        os.makedirs(self._rgb_dir, exist_ok=True)

        self._capture_depth = bool(cfg.get("capture_depth", True))
        if self._capture_depth:
            os.makedirs(self._depth_dir, exist_ok=True)

        prim_name = cfg.get("prim_name", "d455")
        self._mount_path = f"{parent_prim.rstrip('/')}/{prim_name}"

        asset_root = cfg.get("asset_root_name", "RSD455")

        # The RealSense asset ships with RigidBodyAPI enabled (free-falls under
        # gravity by default). It isn't welded to the TCP via a physics joint,
        # so it must be disabled to follow the parent transform kinematically.
        rb_prim = get_current_stage().GetPrimAtPath(f"{self._mount_path}/{asset_root}")
        if rb_prim.IsValid() and rb_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(rb_prim).GetRigidBodyEnabledAttr().Set(False)
        color_cam = cfg.get("color_camera", "Camera_OmniVision_OV9782_Color")
        depth_cam = cfg.get("depth_camera", "Camera_Pseudo_Depth")
        self._prim_path = f"{self._mount_path}/{asset_root}/{color_cam}"
        self._depth_prim_path = f"{self._mount_path}/{asset_root}/{depth_cam}"
        self._camera = Camera(prim_path=self._prim_path, resolution=tuple(cfg.get("resolution", [640, 480])))
        self._depth_camera = (
            Camera(prim_path=self._depth_prim_path, resolution=tuple(cfg.get("resolution", [640, 480])))
            if self._capture_depth
            else None
        )

        self._saved = 0
        self._viewport_window = None

    def initialize(self) -> None:
        """Call after World.reset(), so render products are valid."""
        self._camera.initialize()
        if self._depth_camera is not None:
            self._depth_camera.initialize()

        # Internal camera offsets are baked into the USD, so mount the
        # reference root instead of individual cameras.
        from isaacsim.core.prims import SingleXFormPrim

        SingleXFormPrim(self._mount_path).set_local_pose(
            translation=np.asarray(self._cfg.get("mount_translation", [0.0, 0.0, 0.08]), dtype=float),
            orientation=np.asarray(self._cfg.get("mount_orientation", [0.0, 1.0, 0.0, 0.0]), dtype=float),
        )

        clip = self._cfg.get("clipping_range", [0.01, 5.0])
        self._camera.set_clipping_range(near_distance=float(clip[0]), far_distance=float(clip[1]))
        if self._depth_camera is not None:
            self._depth_camera.set_clipping_range(near_distance=float(clip[0]), far_distance=float(clip[1]))
            self._depth_camera.add_distance_to_image_plane_to_frame()
        elif self._capture_depth:
            self._camera.add_distance_to_image_plane_to_frame()

        if self._cfg.get("enable_pointcloud", False):
            pointcloud_camera = self._depth_camera if self._depth_camera is not None else self._camera
            pointcloud_camera.add_instance_segmentation_to_frame()

        try:
            k_matrix = np.asarray(self._camera.get_intrinsics_matrix())
            np.savetxt(os.path.join(self._out_dir, "cam_K.txt"), k_matrix, fmt="%.18e")
            print(
                f"[camera] mounted {self._prim_path} "
                f"fx={k_matrix[0, 0]:.2f} fy={k_matrix[1, 1]:.2f}"
            )
        except Exception:
            print(f"[camera] mounted {self._prim_path}")

    def maybe_capture(self, step: int) -> str | None:
        if self._every <= 0 or step % self._every != 0:
            return None

        rgba = self._camera.get_rgba()
        if rgba is None or rgba.size == 0:
            return None

        rgb = np.asarray(rgba)[:, :, :3].astype(np.uint8)
        rgb_path = os.path.join(self._rgb_dir, f"{self._saved:07d}.png")
        if _HAS_PIL:
            Image.fromarray(rgb).save(rgb_path)
        else:
            rgb_path = rgb_path.replace(".png", ".npy")
            np.save(rgb_path, rgb)

        if self._capture_depth:
            depth_camera = self._depth_camera if self._depth_camera is not None else self._camera
            depth_m = depth_camera.get_depth()
            if depth_m is not None:
                depth_mm = np.nan_to_num(np.asarray(depth_m) * 1000.0, posinf=0.0, neginf=0.0)
                depth_mm = np.clip(depth_mm, 0, 65535).astype(np.uint16)
                depth_path = os.path.join(self._depth_dir, f"{self._saved:07d}.png")
                if _HAS_PIL:
                    Image.fromarray(depth_mm, mode="I;16").save(depth_path)
                else:
                    np.save(depth_path.replace(".png", ".npy"), depth_mm)

        self._saved += 1
        print(f"[camera] saved {rgb_path}" + (" (+depth)" if self._capture_depth else ""))
        return rgb_path

    @property
    def prim_path(self) -> str:
        return self._prim_path

    def get_pointcloud(self, *, world_frame: bool = True) -> np.ndarray:
        """Deproject the current aligned depth image into finite XYZ points.

        Point-cloud capture is opt-in through ``enable_pointcloud`` because its
        annotator has a non-trivial render cost.
        """
        if not self._cfg.get("enable_pointcloud", False):
            raise RuntimeError("camera point cloud is disabled; set enable_pointcloud=True")
        pointcloud_camera = self._depth_camera if self._depth_camera is not None else self._camera
        depth = pointcloud_camera.get_depth()
        if depth is None:
            return np.empty((0, 3), dtype=np.float32)
        depth = np.asarray(depth, dtype=np.float32)
        return self._deproject_mask(pointcloud_camera, depth, np.isfinite(depth) & (depth > 0.0), world_frame)

    def get_object_pointcloud(self, label: str, *, world_frame: bool = True) -> np.ndarray:
        """Return points selected by Isaac's oracle instance mask."""
        if not self._cfg.get("enable_pointcloud", False):
            raise RuntimeError("camera point cloud is disabled; set enable_pointcloud=True")

        pointcloud_camera = self._depth_camera if self._depth_camera is not None else self._camera
        segmentation = pointcloud_camera.get_current_frame().get("instance_segmentation")
        if not isinstance(segmentation, dict):
            raise RuntimeError("instance segmentation is not ready")

        data = np.asarray(segmentation.get("data")).squeeze()
        id_to_semantics = segmentation.get("info", {}).get("idToSemantics", {})
        target_ids = []
        for instance_id, semantics in id_to_semantics.items():
            class_labels = str(semantics.get("class", "")).split(",")
            if label in class_labels:
                target_ids.append(int(instance_id))
        if not target_ids:
            available = sorted(str(value.get("class", "")) for value in id_to_semantics.values())
            raise RuntimeError(f"instance label '{label}' is not visible; visible labels={available}")

        depth = pointcloud_camera.get_depth()
        if depth is None:
            raise RuntimeError("depth image is not ready")
        depth = np.asarray(depth, dtype=np.float32)
        if data.shape != depth.shape:
            raise RuntimeError(
                f"depth/instance-mask shape mismatch: depth={depth.shape}, mask={data.shape}"
            )
        mask = np.isin(data, target_ids) & np.isfinite(depth) & (depth > 0.0)
        return self._deproject_mask(pointcloud_camera, depth, mask, world_frame)

    @staticmethod
    def _deproject_mask(camera, depth: np.ndarray, mask: np.ndarray, world_frame: bool) -> np.ndarray:
        rows, cols = np.nonzero(mask)
        if len(rows) == 0:
            return np.empty((0, 3), dtype=np.float32)
        image_coords = np.column_stack((cols.astype(np.float32) + 0.5, rows.astype(np.float32) + 0.5))
        depths = depth[rows, cols]
        if world_frame:
            points = camera.get_world_points_from_image_coords(image_coords, depths, device="cpu")
        else:
            points = camera.get_camera_points_from_image_coords(image_coords, depths, device="cpu")
        xyz = np.asarray(points, dtype=np.float32)
        return np.ascontiguousarray(xyz[np.isfinite(xyz).all(axis=1)])

    def set_as_active_viewport_camera(self) -> bool:
        """Show this camera in the active Isaac Sim viewport."""
        try:
            from omni.kit.viewport.utility import get_active_viewport

            viewport = get_active_viewport()
            if viewport is None:
                return False
            viewport.camera_path = self._prim_path
            print(f"[camera] active viewport camera -> {self._prim_path}")
            return True
        except Exception as exc:
            print(f"[camera] active viewport camera failed: {exc}")
            return False

    def open_secondary_viewport(self, name: str = "Wrist Camera", width: int = 640, height: int = 480) -> bool:
        """Open a second, docked viewport window showing this camera, leaving the main
        perspective viewport untouched."""
        if self._viewport_window is not None:
            return True
        try:
            from omni.kit.viewport.utility import create_viewport_window

            self._viewport_window = create_viewport_window(
                name=name,
                width=width,
                height=height,
                camera_path=self._prim_path,
            )
            if self._viewport_window is None:
                return False
            print(f"[camera] opened secondary viewport '{name}' -> {self._prim_path}")
            return True
        except Exception as exc:
            print(f"[camera] secondary viewport failed: {exc}")
            return False
