"""Grasp scene entrypoint.

The arm is selected with ``--robot``; everything else in this file is scene
logic that does not know which arm it is driving. See
``source/robots/__init__.py`` for the registry.

Examples:
    ~/isaacsim/python.sh grasp_scene.py
    ~/isaacsim/python.sh grasp_scene.py --robot indy7
    ~/isaacsim/python.sh grasp_scene.py --target-position 0.45 0.0 0.35
    ~/isaacsim/python.sh grasp_scene.py --target-position 0.45 0.0 0.35 --target-orientation 0 0 1 0
    ~/isaacsim/python.sh grasp_scene.py --gripper close
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
SOURCE_DIR = os.path.join(ROOT_DIR, "source")
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

from graspgen import config as graspgen_config
from robots import DEFAULT_ROBOT, available_robots, get_robot
from sim.config import (
    CAMERA_CONFIG,
    FRICTION_CORRELATION_DISTANCE,
    CONTROL_HZ,
    PHYSICS_HZ,
    PHYSICS_DT,
    RENDERING_HZ,
    RENDERING_DT,
    YCB_CONFIG,
)

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument(
    "--robot",
    default=DEFAULT_ROBOT,
    choices=available_robots(),
    help=f"구동할 로봇 (source/robots 레지스트리, 기본 {DEFAULT_ROBOT})",
)
parser.add_argument("--headless", action="store_true", help="GUI 없이 실행")
parser.add_argument(
    "--ycb-dynamic",
    action="store_true",
    help="세틀 후에도 YCB를 고정하지 않고 dynamic으로 둔다. 스폰 자세가 물리적으로 "
    "안정한지 검사하는 용도 — 종료 시 물체별 이동/회전량을 출력한다.",
)
parser.add_argument(
    "--ycb-only",
    default=None,
    help="지정한 이름의 YCB 물체 하나만 스폰한다(예: 005_tomato_soup_can). "
    "grasp 파이프라인을 단일 물체로 좁혀 볼 때 쓴다.",
)
parser.add_argument(
    "--ycb-radius",
    type=float,
    default=None,
    help="YCB 스폰 반경(m)을 덮어쓴다. 기본 0.70은 Indy7 리치 기준으로 정한 값이라 "
    "다른 팔에서는 워크스페이스 밖일 수 있다.",
)
parser.add_argument(
    "--ycb-collision",
    choices=["convexHull", "convexDecomposition", "sdf", "boundingCube"],
    default=None,
    help="모든 YCB의 충돌 근사를 강제한다(단일 변수 A/B용). 기본은 물체별 설정.",
)
parser.add_argument("--max-steps", type=int, default=0, help="N 스텝 후 자동 종료 (0=무한)")
parser.add_argument(
    "--wrist-viewport",
    action="store_true",
    help="별도의 wrist-camera viewport를 연다(기본 off: 렌더 부하 절감)",
)
parser.add_argument("--target-position", type=float, nargs=3, metavar=("X", "Y", "Z"))
parser.add_argument(
    "--target-orientation",
    type=float,
    nargs=4,
    default=[0.0, 0.0, 1.0, 0.0],
    metavar=("W", "X", "Y", "Z"),
    help="목표 TCP orientation, Isaac wxyz quaternion",
)
parser.add_argument(
    "--gripper",
    choices=["open", "close", "hold"],
    default="open",
    help="그리퍼 초기/유지 명령(기본 open; hold은 USD 초기 자세 유지)",
)
parser.add_argument(
    "--graspgen",
    action="store_true",
    help="wrist point cloud를 GraspGen ZMQ 서버로 보내고 grasp pose를 표시",
)
parser.add_argument("--graspgen-host", default=graspgen_config.HOST)
parser.add_argument("--graspgen-port", type=int, default=graspgen_config.PORT)
parser.add_argument("--graspgen-timeout-ms", type=int, default=graspgen_config.TIMEOUT_MS)
parser.add_argument(
    "--graspgen-step",
    type=int,
    default=graspgen_config.TRIGGER_STEP,
    help="GraspGen을 한 번 호출할 simulation step",
)
parser.add_argument("--grasp-object-index", type=int, default=0, help="ycb_paths에서 선택할 object index")
parser.add_argument("--grasp-point-count", type=int, default=graspgen_config.POINT_COUNT)
parser.add_argument("--grasp-num-grasps", type=int, default=graspgen_config.NUM_GRASPS)
parser.add_argument("--grasp-topk", type=int, default=graspgen_config.TOPK)
parser.add_argument("--grasp-seed", type=int, default=graspgen_config.SEED)
parser.add_argument(
    "--execute-grasp",
    action="store_true",
    help="best grasp pose를 pregrasp→approach→close→lift 순서로 IK 실행 (--graspgen 필요)",
)
args, _ = parser.parse_known_args()

cpu_threads = int(os.environ.get("ISAAC_GRASPGEN_CPU_THREADS", "8"))
if cpu_threads < 1:
    raise ValueError("ISAAC_GRASPGEN_CPU_THREADS must be >= 1")
simulation_app = SimulationApp(
    {
        "headless": args.headless,
        "limit_cpu_threads": cpu_threads,
        "multi_gpu": False,
    }
)

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402

from control.grasp_execution import GraspExecutor  # noqa: E402
from graspgen.pointcloud import sample_fixed  # noqa: E402
from graspgen.selection import select_executable_grasp  # noqa: E402
from graspgen.visualization import draw_grasps, draw_pointcloud  # noqa: E402
from sim.camera import WristCamera, add_dome_light, set_friction_correlation_distance  # noqa: E402
from sim.ros2 import (  # noqa: E402
    CLOCK_TOPIC,
    JOINT_COMMAND_TOPIC,
    JOINT_STATES_TOPIC,
    TF_TOPIC,
    create_ros2_action_graph,
)
from sim.ycb import (  # noqa: E402
    get_world_bounds,
    print_ycb_centers,
    report_ycb_drift,
    set_ycb_kinematic,
    settle_ycb,
    spawn_ycb,
    ycb_pose_snapshot,
)


def main() -> None:
    spec = get_robot(args.robot)
    base_position = np.asarray(spec.position, dtype=float)

    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.robot_motion.pink")
    enable_extension("isaacsim.robot_setup.assembler")
    simulation_app.update()

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=PHYSICS_DT,
        rendering_dt=RENDERING_DT,
    )
    print(
        f"[timing] physics={PHYSICS_HZ}Hz render={RENDERING_HZ}Hz "
        f"control={CONTROL_HZ}Hz substeps={PHYSICS_HZ // RENDERING_HZ}"
    )
    world.scene.add_default_ground_plane()
    add_dome_light()
    set_friction_correlation_distance(FRICTION_CORRELATION_DISTANCE)

    robot = world.scene.add(spec.spawn(spec))
    print(
        f"[{spec.name}] spawned '{spec.resolve_usd_path()}' -> "
        f"{spec.prim_path} @ {base_position.tolist()}"
    )

    ycb_config = YCB_CONFIG
    if args.ycb_only is not None:
        selected = [o for o in YCB_CONFIG["objects"] if str(o["name"]) == args.ycb_only]
        if not selected:
            available = [str(o["name"]) for o in YCB_CONFIG["objects"]]
            raise ValueError(f"--ycb-only '{args.ycb_only}' not in {available}")
        ycb_config = dict(YCB_CONFIG, objects=selected)
    if args.ycb_radius is not None:
        ycb_config = dict(
            ycb_config, spawn=dict(ycb_config["spawn"], radius=float(args.ycb_radius))
        )

    # Spawned dynamic so the solver, not the placement code, decides the
    # resting pose; settle_ycb pins them afterwards.
    ycb_paths = spawn_ycb(
        ycb_config,
        base_position=base_position,
        kinematic=False,
        collision_approximation=args.ycb_collision,
    )

    world.reset()
    settle_ycb(world, ycb_paths, pin=not args.ycb_dynamic)
    graph_path = create_ros2_action_graph(robot.prim_path)
    simulation_app.update()
    print(f"[ros2] Action Graph: {graph_path}")
    print(f"[ros2] Publisher: {JOINT_STATES_TOPIC}, {CLOCK_TOPIC}, {TF_TOPIC}")
    print(f"[ros2] Subscriber: {JOINT_COMMAND_TOPIC}")
    set_camera_view(
        eye=[1.2, 1.0, 0.9],
        target=[0.35, 0.0, 0.15],
        camera_prim_path="/OmniverseKit_Persp",
    )

    control_dt = 1.0 / CONTROL_HZ
    ik = spec.make_ik(robot, dt=control_dt)
    gripper = spec.make_gripper(robot)
    wrist_camera = None
    if spec.wrist_camera is not None:
        camera_config = dict(CAMERA_CONFIG)
        camera_config.update({k: v for k, v in spec.wrist_camera.items() if k != "link"})
        camera_config["enable_pointcloud"] = args.graspgen
        wrist_camera = WristCamera(
            camera_config,
            parent_prim=ik.link_path(spec.wrist_camera["link"]),
            workdir=ROOT_DIR,
        )
        wrist_camera.initialize()
        if not args.headless and args.wrist_viewport:
            wrist_camera.open_secondary_viewport()
    else:
        print(f"[camera] {spec.name} has no wrist camera mount; skipping")
    print(f"[ik] {spec.name} PinkArmIK ready")
    print_ycb_centers(ycb_paths)
    ycb_baseline = ycb_pose_snapshot(ycb_paths)

    if args.execute_grasp and not args.graspgen:
        raise ValueError("--execute-grasp requires --graspgen")
    if args.graspgen and wrist_camera is None:
        raise ValueError(
            f"--graspgen needs a wrist camera, but robot '{spec.name}' has no "
            "wrist_camera. GraspGen is fed from the wrist point cloud."
        )

    graspgen_client = None
    if args.graspgen:
        from graspgen.client import GraspGenClient

        if not 0 <= args.grasp_object_index < len(ycb_paths):
            raise ValueError(
                f"--grasp-object-index must be in [0, {len(ycb_paths) - 1}], "
                f"got {args.grasp_object_index}"
            )
        graspgen_client = GraspGenClient(
            host=args.graspgen_host,
            port=args.graspgen_port,
            timeout_ms=args.graspgen_timeout_ms,
        )
        if not graspgen_client.health_check():
            raise RuntimeError(
                f"GraspGen server is not ready at "
                f"{args.graspgen_host}:{args.graspgen_port}. "
                "Start it with ./scripts/run_graspgen_server.py"
            )
        print(f"[graspgen] connected: {graspgen_client.metadata()}")

    target_position = None if args.target_position is None else np.asarray(args.target_position, dtype=float)
    target_orientation = np.asarray(args.target_orientation, dtype=float)
    if target_position is not None:
        print(
            f"[ik] tracking target position={target_position.tolist()}, "
            f"orientation={target_orientation.tolist()}"
        )

    if args.gripper == "open":
        gripper.open()
        gripper_hold_target = gripper.open_position
        print("[gripper] open")
    elif args.gripper == "close":
        gripper.close()
        gripper_hold_target = gripper.closed_position
        print("[gripper] close")
    else:  # hold: preserve the joint state authored in the USD.
        gripper_hold_target = gripper.hold_position

    step = 0
    reset_needed = False
    graspgen_called = False
    grasp_executor = None
    grasp_phase = None
    grasp_target_path = None
    try:
        while simulation_app.is_running():
            world.step(render=True)

            if world.is_stopped():
                reset_needed = True
            if not world.is_playing():
                continue
            if reset_needed:
                world.reset()
                ik = spec.make_ik(robot, dt=control_dt)
                gripper = spec.make_gripper(robot)
                if wrist_camera is not None:
                    wrist_camera.initialize()
                    if not args.headless and args.wrist_viewport:
                        wrist_camera.open_secondary_viewport()
                graspgen_called = False
                grasp_executor = None
                grasp_phase = None
                grasp_target_path = None
                settle_ycb(world, ycb_paths, pin=not args.ycb_dynamic)
                ycb_baseline = ycb_pose_snapshot(ycb_paths)
                reset_needed = False

            if target_position is not None and not graspgen_called:
                reachable = ik.go_to(target_position, target_orientation)
                if step % 60 == 0:
                    ee_pos, ee_quat = ik.ee_pose()
                    print(
                        f"[ik] step={step} reachable={reachable} "
                        f"ee_pos={np.asarray(ee_pos).round(4).tolist()} "
                        f"ee_quat={np.asarray(ee_quat).round(4).tolist()} "
                        f"arm_q={np.asarray(robot.get_joint_positions())[:spec.num_arm_dofs].round(3).tolist()} "
                        f"gripper_q={gripper.hold_position:.4f}"
                    )

            if grasp_executor is not None:
                if grasp_executor.active:
                    phase = grasp_executor.step()
                    if phase != grasp_phase:
                        print(f"[grasp] phase: {grasp_phase} -> {phase}")
                        if grasp_target_path is not None:
                            object_min, object_max = get_world_bounds(grasp_target_path)
                            object_center = 0.5 * (object_min + object_max)
                            print(
                                f"[grasp] target center={object_center.round(4).tolist()} "
                                f"bottom_z={object_min[2]:.4f}"
                            )
                        if phase == "approach" and grasp_target_path is not None:
                            set_ycb_kinematic(grasp_target_path, False)
                            print(f"[grasp] released dynamic target: {grasp_target_path}")
                        grasp_phase = phase
                elif grasp_executor.phase == "done":
                    gripper.close()
                else:
                    gripper.hold(gripper_hold_target)
            else:
                gripper.hold(gripper_hold_target)
            if step % 60 == 0:
                print(
                    f"[gripper] step={step} q={gripper.hold_position:.4f} "
                    f"target={gripper.target_position:.4f} "
                    f"effort={gripper.measured_effort:.4f}"
                )
            if wrist_camera is not None:
                wrist_camera.maybe_capture(step)

            if graspgen_client is not None and not graspgen_called and step >= args.graspgen_step:
                # The point cloud is only meaningful from the viewpoint the
                # camera mount was validated at. Report the deviation rather
                # than assuming the arm drifted nowhere since startup.
                observed_q = np.asarray(robot.get_joint_positions())[: spec.num_arm_dofs]
                posture_error = float(
                    np.max(np.abs(observed_q - np.asarray(spec.observation_posture, dtype=float)))
                )
                print(
                    f"[graspgen] observing from arm_q={observed_q.round(3).tolist()} "
                    f"(max deviation from observation_posture: {posture_error:.3f} rad)"
                )

                target_path = ycb_paths[args.grasp_object_index]
                grasp_target_path = target_path
                target_label = str(ycb_config["objects"][args.grasp_object_index]["name"])
                object_cloud = wrist_camera.get_object_pointcloud(
                    target_label,
                    world_frame=True,
                )
                if len(object_cloud) < 32:
                    raise RuntimeError(
                        f"only {len(object_cloud)} instance-masked camera points found for {target_path}; "
                        "move the wrist camera so the object is visible"
                    )
                grasp_cloud = sample_fixed(
                    object_cloud,
                    args.grasp_point_count,
                    seed=args.grasp_seed,
                )
                grasp_dir = os.path.join(ROOT_DIR, "output", "graspgen")
                os.makedirs(grasp_dir, exist_ok=True)
                cloud_path = os.path.join(grasp_dir, "input_world.npy")
                np.save(cloud_path, grasp_cloud)
                print(
                    f"[graspgen] target={target_path} instance={len(object_cloud)} "
                    f"sent={len(grasp_cloud)}"
                )
                result = graspgen_client.infer(
                    grasp_cloud,
                    num_grasps=args.grasp_num_grasps,
                    topk_num_grasps=args.grasp_topk,
                    min_grasps=1,
                    max_tries=1,
                )
                draw_pointcloud(grasp_cloud)
                draw_grasps(
                    result.grasps,
                    result.confidences,
                    max_grasps=args.grasp_topk,
                )
                if len(result.grasps):
                    best = int(np.argmax(result.confidences))
                    selection = None
                    if args.execute_grasp:
                        best, selection = select_executable_grasp(
                            result.grasps,
                            result.confidences,
                            grasp_cloud,
                            gripper_depth=spec.gripper.graspgen_depth,
                            support_z=float(ycb_config["spawn"].get("support_z", 0.0)),
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
                            f"outward={selection['outward_approach']:.4f} "
                            f"tool_distance={selection['tool_distance']:.4f}"
                        )
                    if args.execute_grasp:
                        grasp_executor = GraspExecutor(ik, gripper)
                        grasp_executor.start(result.grasps[best])
                        grasp_phase = grasp_executor.phase
                        print(f"[grasp] executing best grasp, phase: {grasp_phase}")
                else:
                    print(f"[graspgen] no grasp returned; input saved={cloud_path}")
                graspgen_called = True

            step += 1
            if args.max_steps and step >= args.max_steps:
                print_ycb_centers(ycb_paths)
                report_ycb_drift(
                    ycb_baseline,
                    ycb_pose_snapshot(ycb_paths),
                    label=f"{step} steps, kinematic={not args.ycb_dynamic}",
                )
                print(f"[main] max-steps({args.max_steps}) 도달 - 종료")
                break
    finally:
        if graspgen_client is not None:
            graspgen_client.close()

    simulation_app.close()


if __name__ == "__main__":
    main()
