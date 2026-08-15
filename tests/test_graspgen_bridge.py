from __future__ import annotations

import os
import sys
import threading

import msgpack
import msgpack_numpy
import numpy as np
import zmq


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "source"))

from graspgen.client import GraspGenClient
from graspgen.pointcloud import crop_aabb, sample_fixed
from graspgen.selection import select_executable_grasp
from graspgen.visualization import grasp_axis_lines
from control.grasp_execution import GraspExecutor, GraspPlan, grasp_pose_reachable
from robots import DEFAULT_ROBOT, available_robots, get_robot
from robots.gripper import ROBOTIQ_2F_140, ParallelFingerGripper, SingleJointGripper
from sim.cli import parse_args
from sim.config import CONTROL_HZ, PHYSICS_HZ, RENDERING_HZ


msgpack_numpy.patch()


class _ImmediateIK:
    def __init__(self) -> None:
        self.position = np.zeros(3)

    def go_to(self, position, orientation) -> bool:
        self.position = np.asarray(position, dtype=float)
        return True

    def ee_pose(self):
        return self.position, np.array([1.0, 0.0, 0.0, 0.0])


class _RecordingGripper:
    def __init__(self) -> None:
        self.commands: list[str] = []

    def open(self) -> None:
        self.commands.append("open")

    def close(self) -> None:
        self.commands.append("close")


class _RecordingReachability:
    def __init__(self) -> None:
        self.positions: list[np.ndarray] = []

    def reachable(self, position, orientation) -> bool:
        self.positions.append(np.asarray(position, dtype=float))
        return True


def test_timing_contract() -> None:
    assert PHYSICS_HZ == 240
    assert RENDERING_HZ == CONTROL_HZ == 60
    assert PHYSICS_HZ % RENDERING_HZ == 0


def test_grasp_executor_reaches_lift_done() -> None:
    ik = _ImmediateIK()
    gripper = _RecordingGripper()
    executor = GraspExecutor(ik, gripper)
    pose = np.eye(4)
    pose[:3, 3] = [0.4, 0.0, 0.2]
    executor.start(pose)

    phases = [executor.phase]
    for _ in range(1_000):
        phase = executor.step()
        if phase != phases[-1]:
            phases.append(phase)
        if phase in ("done", "failed"):
            break

    assert phases == ["pregrasp", "approach", "close", "lift", "done"]
    assert gripper.commands[-1] == "close"


def test_reachability_and_execution_share_one_grasp_plan() -> None:
    pose = np.eye(4)
    pose[:3, 3] = [0.4, -0.1, 0.2]
    plan = GraspPlan.from_pose(pose)
    ik = _RecordingReachability()

    assert grasp_pose_reachable(ik, pose)
    np.testing.assert_allclose(
        ik.positions,
        [plan.positions["pregrasp"], plan.positions["approach"]],
    )


def test_execution_selection_rejects_table_colliding_best_grasp() -> None:
    depth = ROBOTIQ_2F_140.graspgen_depth
    points = np.array([[0.40, 0.0, 0.02], [0.41, 0.0, 0.02]], dtype=np.float32)
    poses = np.repeat(np.eye(4)[None], 2, axis=0)
    # Higher confidence, but the gripper base is below the clearance gate.
    poses[0, :3, 2] = [0.0, 0.0, -1.0]
    poses[0, :3, 3] = [0.405, 0.0, 0.05 + depth]
    # Lower confidence and executable from above.
    approach = np.array([0.6, 0.0, -0.8])
    poses[1, :3, 2] = approach
    poses[1, :3, 3] = points.mean(axis=0) - depth * approach
    # Make candidate 0 genuinely invalid after constructing its tool point.
    poses[0, 2, 3] = 0.05

    index, metrics = select_executable_grasp(
        poses,
        np.array([0.99, 0.80]),
        points,
        gripper_depth=depth,
    )

    assert index == 1
    assert metrics["approach_z"] == -0.8
    assert metrics["outward_cos"] == 1.0


def test_scene_cli_validates_before_isaac_starts() -> None:
    args = parse_args(
        [
            "--robot",
            "panda",
            "--graspgen",
            "--execute-grasp",
            "--ycb-only",
            "005_tomato_soup_can",
            "--/renderer/multiGpu/enabled=false",
        ]
    )
    assert args.robot == "panda"
    assert args.execute_grasp

    try:
        parse_args(["--execute-grasp"])
    except ValueError as error:
        assert "requires --graspgen" in str(error)
    else:
        raise AssertionError("--execute-grasp without --graspgen must fail before Isaac starts")


def test_robot_registry_exposes_indy7_spec() -> None:
    assert "indy7" in available_robots()
    assert DEFAULT_ROBOT in available_robots()

    spec = get_robot("indy7")
    assert spec.name == "indy7"
    assert len(spec.reach_posture) == spec.num_arm_dofs
    assert spec.gripper is ROBOTIQ_2F_140
    # The specs carry paths, so a moved asset shows up here rather than three
    # minutes into an Isaac Sim launch. Indy7's kinematics is a repo-authored
    # URDF, so kinematics_source is a real path for this arm.
    assert os.path.isfile(spec.usd_path), spec.usd_path
    assert os.path.isfile(spec.kinematics_source), spec.kinematics_source


def test_panda_spec_uses_isaacs_own_assets() -> None:
    spec = get_robot("panda")
    assert spec.name == "panda"
    assert spec.num_arm_dofs == 7
    assert len(spec.reach_posture) == spec.num_arm_dofs
    # Nothing for the Panda is authored in this repo: the USD is assets-root
    # relative and the kinematics come from the PINK extension's bundle, so
    # neither is a path on disk to check here.
    assert spec.usd_path.startswith("/Isaac/")
    assert spec.kinematics_source == "bundled:franka"
    # Two independently actuated fingers — the Robotiq's single-DOF controller
    # would close one and leave the other open.
    assert len(spec.gripper.joint_names) == 2
    assert spec.gripper.make is ParallelFingerGripper
    # franka.usd ships no camera, so the wrist camera is created rather than
    # wrapped from the asset.
    assert spec.wrist_camera["mode"] == "pinhole"
    assert spec.wrist_camera["link"] == "panda_hand"
    assert get_robot("indy7").wrist_camera["mode"] == "asset"


def test_single_joint_gripper_rejects_a_two_finger_spec() -> None:
    panda_hand = get_robot("panda").gripper
    try:
        SingleJointGripper(object(), panda_hand)
    except ValueError as error:
        assert "ParallelFingerGripper" in str(error)
    else:
        raise AssertionError("SingleJointGripper must refuse a two-DOF gripper spec")


def test_unknown_robot_names_the_available_ones() -> None:
    try:
        get_robot("ur10")
    except KeyError as error:
        assert "indy7" in str(error) and "panda" in str(error)
    else:
        raise AssertionError("get_robot('ur10') should raise; ur10 is not registered")


def test_pointcloud_preparation() -> None:
    points = np.array(
        [[0, 0, 0], [1, 1, 1], [0.5, 0.5, 0.5], [np.nan, 0, 0]],
        dtype=np.float32,
    )
    cropped = crop_aabb(points, [0.4, 0.4, 0.4], [1.0, 1.0, 1.0])
    assert cropped.shape == (2, 3)
    sampled = sample_fixed(cropped, 8, seed=7)
    assert sampled.shape == (8, 3)
    assert np.isfinite(sampled).all()


def test_grasp_axis_lines() -> None:
    pose = np.eye(4, dtype=np.float32)
    pose[:3, 3] = [1.0, 2.0, 3.0]
    starts, ends, colors = grasp_axis_lines(pose[None], np.array([0.9]), axis_length=0.1)
    assert len(starts) == len(ends) == len(colors) == 3
    np.testing.assert_allclose(ends[0], [1.1, 2.0, 3.0])
    np.testing.assert_allclose(ends[1], [1.0, 2.1, 3.0])
    np.testing.assert_allclose(ends[2], [1.0, 2.0, 3.1])


def test_graspgen_wire_protocol() -> None:
    context = zmq.Context()
    server = context.socket(zmq.REP)
    port = server.bind_to_random_port("tcp://127.0.0.1")

    def serve() -> None:
        for _ in range(2):
            request = msgpack.unpackb(server.recv(), raw=False)
            if request["action"] == "health":
                response = {"status": "ok"}
            else:
                points = np.asarray(request["point_cloud"])
                grasp = np.eye(4, dtype=np.float32)
                grasp[:3, 3] = points.mean(axis=0)
                response = {
                    "grasps": grasp[None],
                    "confidences": np.array([0.75], dtype=np.float32),
                    "timing": {"infer_ms": 12.5},
                }
            server.send(msgpack.packb(response, use_bin_type=True))

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    client = GraspGenClient(port=port, timeout_ms=2_000)
    try:
        assert client.health_check()
        points = np.array([[0.0, 0.0, 0.0], [0.2, 0.4, 0.6]], dtype=np.float32)
        result = client.infer(points)
        assert result.grasps.shape == (1, 4, 4)
        np.testing.assert_allclose(result.grasps[0, :3, 3], points.mean(axis=0))
        np.testing.assert_allclose(result.confidences, [0.75])
        assert result.infer_ms == 12.5
    finally:
        client.close()
        thread.join(timeout=2.0)
        server.close()
        context.term()
