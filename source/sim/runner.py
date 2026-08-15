"""Runtime orchestration for the standalone GraspGen scene.

``grasp_scene.py`` owns only the process boundary: parse arguments, start
``SimulationApp``, then construct this runner. Keeping the mutable simulation
state here makes reset, manual IK, inference, and grasp execution independent
steps instead of one entrypoint-sized control loop.

Import this module only after ``SimulationApp`` has started.
"""

from __future__ import annotations

import os

import numpy as np

from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.viewports import set_camera_view

from control.grasp_execution import GraspExecutor, grasp_pose_reachable, report_finger_straddle
from graspgen.pointcloud import sample_fixed
from graspgen.selection import report_candidate_centering, select_executable_grasp
from graspgen.visualization import draw_grasps, draw_pointcloud
from robots import get_robot
from sim.camera import WristCamera, add_dome_light, set_friction_correlation_distance
from sim.config import (
    CAMERA_CONFIG,
    CONTROL_HZ,
    FRICTION_CORRELATION_DISTANCE,
    PHYSICS_DT,
    PHYSICS_HZ,
    RENDERING_DT,
    RENDERING_HZ,
    YCB_CONFIG,
)
from sim.ros2 import (
    CLOCK_TOPIC,
    JOINT_COMMAND_TOPIC,
    JOINT_STATES_TOPIC,
    TF_TOPIC,
    create_ros2_action_graph,
)
from sim.ycb import (
    get_world_bounds,
    print_ycb_centers,
    report_ycb_drift,
    set_ycb_kinematic,
    settle_ycb,
    spawn_ycb,
    ycb_pose_snapshot,
)


class GraspSceneRunner:
    """Own one scene lifecycle, including reset and the one-shot grasp flow."""

    def __init__(self, args, simulation_app, project_root: str) -> None:
        self.args = args
        self.simulation_app = simulation_app
        self.project_root = project_root
        self.spec = get_robot(args.robot)
        self.base_position = np.asarray(self.spec.position, dtype=float)
        self.control_dt = 1.0 / CONTROL_HZ

        self.world = None
        self.robot = None
        self.ik = None
        self.gripper = None
        self.wrist_camera = None
        self.graspgen_client = None
        self.ycb_paths: list[str] = []
        self.ycb_config = self._build_ycb_config()
        self.ycb_baseline = {}

        self.target_position = (
            None
            if args.target_position is None
            else np.asarray(args.target_position, dtype=float)
        )
        self.target_orientation = np.asarray(args.target_orientation, dtype=float)
        self.gripper_hold_target = 0.0

        self.step_count = 0
        self.reset_needed = False
        self.graspgen_called = False
        self.grasp_executor = None
        self.executed_grasp_pose = None
        self.grasp_phase = None
        self.grasp_target_path = None

    def run(self) -> None:
        """Set up the scene and run until Kit closes or ``--max-steps`` fires."""
        try:
            self._setup()
            while self.simulation_app.is_running():
                if not self._step():
                    break
        finally:
            if self.graspgen_client is not None:
                self.graspgen_client.close()

    def _build_ycb_config(self) -> dict:
        config = YCB_CONFIG
        if self.args.ycb_only is not None:
            selected = [
                item
                for item in YCB_CONFIG["objects"]
                if str(item["name"]) == self.args.ycb_only
            ]
            config = dict(YCB_CONFIG, objects=selected)
        if self.args.ycb_radius is not None:
            config = dict(
                config,
                spawn=dict(config["spawn"], radius=float(self.args.ycb_radius)),
            )
        return config

    def _setup(self) -> None:
        if self.args.graspgen and self.spec.wrist_camera is None:
            raise ValueError(
                f"--graspgen needs a wrist camera, but robot '{self.spec.name}' has no "
                "wrist_camera. GraspGen is fed from the wrist point cloud."
            )

        if not self.args.no_ros2:
            enable_extension("isaacsim.ros2.bridge")
        enable_extension("isaacsim.robot_motion.pink")
        enable_extension("isaacsim.robot_setup.assembler")
        self.simulation_app.update()

        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=PHYSICS_DT,
            rendering_dt=RENDERING_DT,
        )
        print(
            f"[timing] physics={PHYSICS_HZ}Hz render={RENDERING_HZ}Hz "
            f"control={CONTROL_HZ}Hz substeps={PHYSICS_HZ // RENDERING_HZ}"
        )
        self.world.scene.add_default_ground_plane()
        add_dome_light()
        set_friction_correlation_distance(FRICTION_CORRELATION_DISTANCE)

        self.robot = self.world.scene.add(self.spec.spawn(self.spec))
        print(
            f"[{self.spec.name}] spawned '{self.spec.resolve_usd_path()}' -> "
            f"{self.spec.prim_path} @ {self.base_position.tolist()}"
        )
        # Start dynamic so the solver, rather than placement code, determines
        # the resting pose. ``settle_ycb`` pins the observation scene later.
        self.ycb_paths = spawn_ycb(
            self.ycb_config,
            base_position=self.base_position,
            kinematic=False,
            collision_approximation=self.args.ycb_collision,
        )

        self.world.reset()
        settle_ycb(self.world, self.ycb_paths, pin=not self.args.ycb_dynamic)
        self._setup_ros2()
        set_camera_view(
            eye=[1.2, 1.0, 0.9],
            target=[0.35, 0.0, 0.15],
            camera_prim_path="/OmniverseKit_Persp",
        )

        self.ik = self.spec.make_ik(self.robot, dt=self.control_dt)
        self.gripper = self.spec.make_gripper(self.robot)
        self._setup_wrist_camera()
        print(f"[ik] {self.spec.name} PinkArmIK ready")
        print_ycb_centers(self.ycb_paths)
        self.ycb_baseline = ycb_pose_snapshot(self.ycb_paths)

        self._connect_graspgen()
        self._configure_manual_target()
        self._configure_gripper()

    def _setup_ros2(self) -> None:
        if self.args.no_ros2:
            print("[ros2] disabled (--no-ros2)")
            return
        graph_path = create_ros2_action_graph(self.robot.prim_path)
        self.simulation_app.update()
        print(f"[ros2] Action Graph: {graph_path}")
        print(f"[ros2] Publisher: {JOINT_STATES_TOPIC}, {CLOCK_TOPIC}, {TF_TOPIC}")
        print(f"[ros2] Subscriber: {JOINT_COMMAND_TOPIC}")

    def _setup_wrist_camera(self) -> None:
        camera_spec = self.spec.wrist_camera
        if camera_spec is None:
            print(f"[camera] {self.spec.name} has no wrist camera mount; skipping")
            return

        camera_config = dict(CAMERA_CONFIG)
        camera_config.update({key: value for key, value in camera_spec.items() if key != "link"})
        camera_config["enable_pointcloud"] = self.args.graspgen
        self.wrist_camera = WristCamera(
            camera_config,
            parent_prim=self.ik.link_path(camera_spec["link"]),
            workdir=self.project_root,
        )
        self._initialize_wrist_camera()

    def _initialize_wrist_camera(self) -> None:
        self.wrist_camera.initialize()
        if not self.args.headless and self.args.wrist_viewport:
            self.wrist_camera.open_secondary_viewport()

    def _connect_graspgen(self) -> None:
        if not self.args.graspgen:
            return
        if not 0 <= self.args.grasp_object_index < len(self.ycb_paths):
            raise ValueError(
                f"--grasp-object-index must be in [0, {len(self.ycb_paths) - 1}], "
                f"got {self.args.grasp_object_index}"
            )

        from graspgen.client import GraspGenClient

        self.graspgen_client = GraspGenClient(
            host=self.args.graspgen_host,
            port=self.args.graspgen_port,
            timeout_ms=self.args.graspgen_timeout_ms,
        )
        if not self.graspgen_client.health_check():
            raise RuntimeError(
                f"GraspGen server is not ready at "
                f"{self.args.graspgen_host}:{self.args.graspgen_port}. "
                "Start it with ./scripts/run_graspgen_server.py"
            )
        print(f"[graspgen] connected: {self.graspgen_client.metadata()}")

    def _configure_manual_target(self) -> None:
        if self.target_position is not None:
            print(
                f"[ik] tracking target position={self.target_position.tolist()}, "
                f"orientation={self.target_orientation.tolist()}"
            )

    def _configure_gripper(self) -> None:
        if self.args.gripper == "open":
            self.gripper.open()
            self.gripper_hold_target = self.gripper.open_position
            print("[gripper] open")
        elif self.args.gripper == "close":
            self.gripper.close()
            self.gripper_hold_target = self.gripper.closed_position
            print("[gripper] close")
        else:
            self.gripper_hold_target = self.gripper.hold_position

    def _step(self) -> bool:
        self.world.step(render=True)
        if self.world.is_stopped():
            self.reset_needed = True
        if not self.world.is_playing():
            return True
        if self.reset_needed:
            self._reset()

        self._step_manual_target()
        self._step_grasp_executor()
        self._report_gripper()
        if self.wrist_camera is not None:
            self.wrist_camera.maybe_capture(self.step_count)
        self._maybe_infer_grasp()

        self.step_count += 1
        if self.args.max_steps and self.step_count >= self.args.max_steps:
            self._finish_at_step_limit()
            return False
        return True

    def _reset(self) -> None:
        self.world.reset()
        self.ik = self.spec.make_ik(self.robot, dt=self.control_dt)
        self.gripper = self.spec.make_gripper(self.robot)
        if self.wrist_camera is not None:
            self._initialize_wrist_camera()
        self.graspgen_called = False
        self.grasp_executor = None
        self.executed_grasp_pose = None
        self.grasp_phase = None
        self.grasp_target_path = None
        settle_ycb(self.world, self.ycb_paths, pin=not self.args.ycb_dynamic)
        self.ycb_baseline = ycb_pose_snapshot(self.ycb_paths)
        self.reset_needed = False

    def _step_manual_target(self) -> None:
        if self.target_position is None or self.graspgen_called:
            return
        reachable = self.ik.go_to(self.target_position, self.target_orientation)
        if self.step_count % 60 == 0:
            ee_pos, ee_quat = self.ik.ee_pose()
            arm_q = np.asarray(self.robot.get_joint_positions())[: self.spec.num_arm_dofs]
            print(
                f"[ik] step={self.step_count} reachable={reachable} "
                f"ee_pos={np.asarray(ee_pos).round(4).tolist()} "
                f"ee_quat={np.asarray(ee_quat).round(4).tolist()} "
                f"arm_q={arm_q.round(3).tolist()} "
                f"gripper_q={self.gripper.hold_position:.4f}"
            )

    def _step_grasp_executor(self) -> None:
        executor = self.grasp_executor
        if executor is None:
            self.gripper.hold(self.gripper_hold_target)
            return
        if executor.active:
            phase = executor.step()
            if phase != self.grasp_phase:
                self._on_grasp_phase_change(phase)
            return
        if executor.phase == "done":
            self.gripper.close()
        else:
            self.gripper.hold(self.gripper_hold_target)

    def _on_grasp_phase_change(self, phase: str) -> None:
        print(f"[grasp] phase: {self.grasp_phase} -> {phase}")
        if self.grasp_target_path is not None:
            object_min, object_max = get_world_bounds(self.grasp_target_path)
            object_center = 0.5 * (object_min + object_max)
            print(
                f"[grasp] target center={object_center.round(4).tolist()} "
                f"bottom_z={object_min[2]:.4f}"
            )
            report_finger_straddle(
                self.ik,
                self.spec,
                object_center,
                self.executed_grasp_pose,
            )
        if phase == "approach" and self.grasp_target_path is not None:
            set_ycb_kinematic(self.grasp_target_path, False)
            print(f"[grasp] released dynamic target: {self.grasp_target_path}")
        self.grasp_phase = phase

    def _report_gripper(self) -> None:
        if self.step_count % 60 != 0:
            return
        print(
            f"[gripper] step={self.step_count} q={self.gripper.hold_position:.4f} "
            f"target={self.gripper.target_position:.4f} "
            f"effort={self.gripper.measured_effort:.4f}"
        )

    def _maybe_infer_grasp(self) -> None:
        if (
            self.graspgen_client is None
            or self.graspgen_called
            or self.step_count < self.args.graspgen_step
        ):
            return

        self._report_observation_posture()
        target_path = self.ycb_paths[self.args.grasp_object_index]
        self.grasp_target_path = target_path
        target_label = str(self.ycb_config["objects"][self.args.grasp_object_index]["name"])
        object_cloud = self.wrist_camera.get_object_pointcloud(target_label, world_frame=True)
        if len(object_cloud) < 32:
            raise RuntimeError(
                f"only {len(object_cloud)} instance-masked camera points found for "
                f"{target_path}; move the wrist camera so the object is visible"
            )

        grasp_cloud = sample_fixed(
            object_cloud,
            self.args.grasp_point_count,
            seed=self.args.grasp_seed,
        )
        grasp_dir = os.path.join(self.project_root, "output", "graspgen")
        os.makedirs(grasp_dir, exist_ok=True)
        cloud_path = os.path.join(grasp_dir, "input_world.npy")
        np.save(cloud_path, grasp_cloud)
        print(
            f"[graspgen] target={target_path} instance={len(object_cloud)} "
            f"sent={len(grasp_cloud)}"
        )

        result = self.graspgen_client.infer(
            grasp_cloud,
            num_grasps=self.args.grasp_num_grasps,
            topk_num_grasps=self.args.grasp_topk,
            min_grasps=1,
            max_tries=1,
        )
        draw_pointcloud(grasp_cloud)
        draw_grasps(result.grasps, result.confidences, max_grasps=self.args.grasp_topk)
        if len(result.grasps):
            self._handle_grasps(result, grasp_cloud, grasp_dir)
        else:
            print(f"[graspgen] no grasp returned; input saved={cloud_path}")
        self.graspgen_called = True

    def _report_observation_posture(self) -> None:
        observed_q = np.asarray(self.robot.get_joint_positions())[: self.spec.num_arm_dofs]
        posture_error = float(
            np.max(
                np.abs(observed_q - np.asarray(self.spec.observation_posture, dtype=float))
            )
        )
        print(
            f"[graspgen] observing from arm_q={observed_q.round(3).tolist()} "
            f"(max deviation from observation_posture: {posture_error:.3f} rad)"
        )

    def _handle_grasps(self, result, grasp_cloud: np.ndarray, grasp_dir: str) -> None:
        best = int(np.argmax(result.confidences))
        selection = None
        if self.args.execute_grasp:
            best, selection = select_executable_grasp(
                result.grasps,
                result.confidences,
                grasp_cloud,
                gripper_depth=self.spec.gripper.graspgen_depth,
                support_z=float(self.ycb_config["spawn"].get("support_z", 0.0)),
                is_reachable=lambda pose: grasp_pose_reachable(self.ik, pose),
                midline_override=self._oracle_center(),
            )

        best_pose_path = os.path.join(grasp_dir, "best_grasp_world.npy")
        np.save(best_pose_path, result.grasps[best])
        print(
            f"[graspgen] returned={len(result.grasps)} "
            f"best_conf={result.confidences[best]:.4f} "
            f"infer_ms={result.infer_ms} saved={best_pose_path}"
        )
        if selection is not None:
            print(
                f"[grasp] selected index={best} "
                f"base_clearance={selection['base_clearance']:.4f} "
                f"approach_z={selection['approach_z']:.4f} "
                f"outward_cos={selection['outward_cos']:.4f} "
                f"tool_distance={selection['tool_distance']:.4f} "
                f"closing_offset={selection['closing_offset'] * 1000:+.1f}mm"
            )
        self._report_centering(result, grasp_cloud, best, selection is not None)

        if self.args.execute_grasp:
            self.grasp_executor = GraspExecutor(self.ik, self.gripper)
            self.executed_grasp_pose = result.grasps[best]
            self.grasp_executor.start(self.executed_grasp_pose)
            self.grasp_phase = self.grasp_executor.phase
            print(f"[grasp] executing best grasp, phase: {self.grasp_phase}")

    def _oracle_center(self):
        if not self.args.grasp_oracle_centering or self.grasp_target_path is None:
            return None
        object_min, object_max = get_world_bounds(self.grasp_target_path)
        return 0.5 * (object_min + object_max)

    def _report_centering(
        self,
        result,
        grasp_cloud: np.ndarray,
        selected: int,
        report_candidates: bool,
    ) -> None:
        if self.grasp_target_path is None:
            return
        cloud_centroid = np.asarray(grasp_cloud, dtype=float).reshape(-1, 3).mean(axis=0)
        true_min, true_max = get_world_bounds(self.grasp_target_path)
        true_center = 0.5 * (true_min + true_max)
        print(
            f"[grasp] observed centroid={cloud_centroid.round(4).tolist()} "
            f"true center={true_center.round(4).tolist()} "
            f"gap={np.round((cloud_centroid - true_center) * 1000, 1).tolist()}mm"
        )
        if report_candidates:
            report_candidate_centering(
                result.grasps,
                result.confidences,
                true_center,
                gripper_depth=self.spec.gripper.graspgen_depth,
                selected=selected,
            )

    def _finish_at_step_limit(self) -> None:
        print_ycb_centers(self.ycb_paths)
        report_ycb_drift(
            self.ycb_baseline,
            ycb_pose_snapshot(self.ycb_paths),
            label=f"{self.step_count} steps, kinematic={not self.args.ycb_dynamic}",
        )
        print(f"[main] max-steps({self.args.max_steps}) 도달 - 종료")
