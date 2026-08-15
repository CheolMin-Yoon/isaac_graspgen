"""Official Isaac PINK IK, driven by a ``RobotSpec``.

The controller itself is arm-agnostic — everything that differs per arm (the
kinematics model, the tool frame, the arm DOF count, the seed posture) comes in
through the spec. The two startup checks below are the reason this stays one
class instead of per-robot copies: they reject a kinematics model that has
drifted from the USD before it can command the plant.
"""

from __future__ import annotations

import numpy as np


def _to_numpy(value) -> np.ndarray:
    """Flatten a warp array, torch tensor, or sequence to a 1-D numpy array."""
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value).reshape(-1)


class PinkArmIK:
    """Official Isaac Sim PINK controller backed by a checked URDF."""

    def __init__(self, articulation, spec, dt: float = 1.0 / 60.0) -> None:
        import isaacsim.robot_motion.experimental.motion_generation as mg
        import pinocchio as pin
        import warp as wp
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.robot_motion.pink import PinkIKController

        ee_link_name = spec.ee_link_name
        num_arm_dofs = spec.num_arm_dofs

        exp_robot = Articulation(articulation.prim_path)
        link_names = list(exp_robot.link_names)
        ee_link_index = link_names.index(ee_link_name) if ee_link_name in link_names else 0
        link_paths = getattr(exp_robot, "link_paths", None)
        ee_path = (
            str(link_paths[0][ee_link_index])
            if link_paths is not None
            else f"{articulation.prim_path}/{ee_link_name}"
        )

        pink_robot = spec.make_pink_robot()
        kinematics_joints = list(pink_robot.controlled_joint_names)
        usd_joints = list(articulation.dof_names)
        # Isaac's bundled Franka model includes the hand, so the kinematics
        # joints are a superset of the arm's rather than exactly the arm's.
        # What has to hold is that every controlled joint is a real DOF, and
        # that the arm joints lead both lists in the same order — that is what
        # makes the DOF indices the controller writes line up with the plant.
        absent = [j for j in kinematics_joints if j not in usd_joints]
        if absent:
            raise RuntimeError(
                f"{spec.name} kinematics joints absent from the USD articulation: {absent}"
            )
        if kinematics_joints[:num_arm_dofs] != usd_joints[:num_arm_dofs]:
            raise RuntimeError(
                f"{spec.name} arm joint order mismatch: "
                f"kinematics={kinematics_joints[:num_arm_dofs]}, "
                f"usd={usd_joints[:num_arm_dofs]}"
            )

        self._mg = mg
        self._wp = wp
        self._pin = pin
        self._spec = spec
        self._pink_robot = pink_robot
        self._kinematics_joints = kinematics_joints
        self._last_command: tuple[np.ndarray, np.ndarray] | None = None
        self._solver_failures = 0

        # PINK asserts the measured configuration lies inside the model's joint
        # limits and raises out of solve_ik when it does not. PhysX routinely
        # overshoots a limit by ~3e-6 rad, and the Panda hand makes that certain
        # because its open position IS the finger joint's upper limit. The
        # controller then returns no command at all and the arm freezes in
        # place, silently, mid-phase. Clamping the estimate is the honest fix:
        # a plant measurement epsilon outside a URDF limit is still that joint's
        # limit, and a kinematics model must not be handed an infeasible q.
        self._clamp_indices: np.ndarray | None = None
        if pink_robot.model.nq == len(kinematics_joints):
            margin = 1e-4
            self._clamp_indices = np.array(
                [usd_joints.index(name) for name in kinematics_joints], dtype=int
            )
            self._clamp_lower = (
                np.asarray(pink_robot.model.lowerPositionLimit, dtype=float) + margin
            )
            self._clamp_upper = (
                np.asarray(pink_robot.model.upperPositionLimit, dtype=float) - margin
            )
        else:
            print(
                f"[ik] {spec.name}: nq={pink_robot.model.nq} does not match "
                f"{len(kinematics_joints)} controlled joints; joint-limit clamping disabled"
            )
        self._articulation = articulation
        self._exp_robot = exp_robot
        self._num_arm_dofs = num_arm_dofs
        self._ee_path = ee_path
        self._link_names = link_names
        self._link_paths = link_paths
        self._robot_prim_path = articulation.prim_path
        self._robot_joint_space = list(articulation.dof_names)
        self._robot_site_space = [ee_link_name]
        self._dt = float(dt)
        self._sim_time = 0.0
        self._controller_reset = False
        self._seed_applied = False
        # Same pattern as Isaac Sim's official Franka PINK example: start from
        # a bent, nonsingular reaching posture before reactive pose tracking.
        self._reach_posture = np.asarray(spec.reach_posture, dtype=np.float32)
        if self._reach_posture.shape != (num_arm_dofs,):
            raise ValueError(
                f"{spec.name} reach_posture must have {num_arm_dofs} entries, "
                f"got {self._reach_posture.shape}"
            )

        self._controller = PinkIKController(
            pink_robot=pink_robot,
            robot_joint_space=self._robot_joint_space,
            robot_site_space=self._robot_site_space,
            tool_frame=ee_link_name,
            position_cost=5.0,
            orientation_cost=0.05,
            posture_cost=5e-3,
            solver="osqp",
            dt=self._dt,
        )

        # Reject a stale kinematics model before it can command the plant, by
        # cross-checking its FK against the live pose. The comparison is made
        # at whatever configuration the plant is currently in, not at the
        # model's neutral one: an asset is free to spawn wherever it likes, and
        # the Panda spawns at Franka's bent home pose, where neutral-pose FK is
        # half a metre away and proves nothing.
        if pink_robot.model.nq != len(kinematics_joints):
            print(
                f"[ik] {spec.name}: model nq={pink_robot.model.nq} does not match "
                f"{len(kinematics_joints)} controlled joints; skipping the FK cross-check"
            )
        else:
            live_positions = np.asarray(articulation.get_joint_positions(), dtype=float)
            q_model = np.array(
                [live_positions[usd_joints.index(name)] for name in kinematics_joints],
                dtype=float,
            )
            pin.forwardKinematics(pink_robot.model, pink_robot.data, q_model)
            pin.updateFramePlacements(pink_robot.model, pink_robot.data)
            model_tcp = pink_robot.data.oMf[pink_robot.model.getFrameId(ee_link_name)]
            live_tcp_position, live_tcp_orientation = self.ee_pose()
            # Model FK is in the robot's base frame; the live pose is in world.
            model_tcp_world = np.asarray(model_tcp.translation) + np.asarray(
                spec.position, dtype=float
            )
            model_tcp_quat = pin.Quaternion(model_tcp.rotation)
            model_tcp_wxyz = np.array(
                [model_tcp_quat.w, model_tcp_quat.x, model_tcp_quat.y, model_tcp_quat.z]
            )
            position_error = float(
                np.linalg.norm(model_tcp_world - np.asarray(live_tcp_position))
            )
            orientation_error = 1.0 - abs(
                float(np.dot(model_tcp_wxyz, np.asarray(live_tcp_orientation)))
            )
            if position_error > 1e-4 or orientation_error > 1e-4:
                raise RuntimeError(
                    f"{spec.name} kinematics/USD FK mismatch at the live pose: "
                    f"model_p={model_tcp_world}, usd_p={live_tcp_position}, "
                    f"position_error={position_error:.6f}, "
                    f"orientation_error={orientation_error:.6f}"
                )
        print(
            f"[ik] backend=isaac-pink robot={spec.name} ee={ee_link_name} "
            f"dt={self._dt:.6f} kinematics={spec.kinematics_source}"
        )

    @property
    def ee_path(self) -> str:
        return self._ee_path

    def link_path(self, link_name: str) -> str:
        """Resolve any articulation link's prim path by name (e.g. "link6")."""
        if link_name in self._link_names:
            index = self._link_names.index(link_name)
            if self._link_paths is not None:
                return str(self._link_paths[0][index])
        return f"{self._robot_prim_path}/{link_name}"

    def go_to(self, position, orientation) -> bool:
        """Apply one official PINK step toward a world-frame pose.

        `position` is xyz in meters. `orientation` is Isaac wxyz quaternion.
        """
        target_position = np.asarray(position, dtype=float)
        target_orientation = np.asarray(orientation, dtype=float)
        if not self._seed_applied:
            self._exp_robot.set_dof_positions(
                self._wp.from_numpy(self._reach_posture),
                dof_indices=list(range(self._num_arm_dofs)),
            )
            self._exp_robot.set_dof_position_targets(
                self._wp.from_numpy(self._reach_posture),
                dof_indices=list(range(self._num_arm_dofs)),
            )
            self._seed_applied = True
            return False

        measured = _to_numpy(self._exp_robot.get_dof_positions()).astype(np.float32)
        if self._clamp_indices is not None:
            measured[self._clamp_indices] = np.clip(
                measured[self._clamp_indices], self._clamp_lower, self._clamp_upper
            )
        estimated_state = self._mg.RobotState(
            joints=self._mg.JointState.from_name(
                robot_joint_space=self._robot_joint_space,
                positions=(self._robot_joint_space, self._wp.from_numpy(measured)),
                velocities=(self._robot_joint_space, self._exp_robot.get_dof_velocities()),
            )
        )
        setpoint_state = self._mg.RobotState(
            sites=self._mg.SpatialState.from_name(
                spatial_space=self._robot_site_space,
                positions=(
                    self._robot_site_space,
                    self._wp.array([target_position.astype(np.float32)]),
                ),
                orientations=(
                    self._robot_site_space,
                    self._wp.array([target_orientation.astype(np.float32)]),
                ),
            )
        )
        if not self._controller_reset:
            self._controller_reset = self._controller.reset(
                estimated_state,
                setpoint_state,
                self._sim_time,
            )
        desired_state = self._controller.forward(estimated_state, setpoint_state, self._sim_time)
        if desired_state is not None and desired_state.joints.positions is not None:
            solution = _to_numpy(desired_state.joints.positions).astype(np.float32)
            indices = _to_numpy(desired_state.joints.position_indices).astype(int)
            # Write only the arm. Isaac's bundled Franka model includes the two
            # finger joints, so the QP solves for them and PINK's posture task
            # pulls them back to whatever they were at reset — i.e. open. Left
            # unfiltered, the arm controller and the gripper controller command
            # the same DOFs every step and the gripper cannot close. The fingers
            # sit past the tool frame, so dropping them costs the arm nothing.
            # The constructor already checked the arm leads both joint lists in
            # the same order, which is what makes this index test valid.
            arm = indices < self._num_arm_dofs
            self._exp_robot.set_dof_position_targets(
                positions=self._wp.from_numpy(solution[arm]),
                dof_indices=indices[arm].tolist(),
            )
            self._last_command = (solution[arm], indices[arm])
            self._solver_failures = 0
        else:
            # A controller that stops commanding must never do it quietly: PINK
            # logs its own failure through carb at warning level, which is
            # indistinguishable from the thousands of lines Kit already prints.
            self._solver_failures += 1
            if self._solver_failures in (1, 30) or self._solver_failures % 300 == 0:
                print(
                    f"[ik] solver returned no command for {self._solver_failures} "
                    "consecutive step(s); the arm is NOT being driven"
                )
        self._sim_time += self._dt

        current_position, current_orientation = self.ee_pose()
        position_error = float(np.linalg.norm(target_position - np.asarray(current_position)))
        orientation_error = 2.0 * np.arccos(
            np.clip(abs(float(np.dot(target_orientation, current_orientation))), 0.0, 1.0)
        )
        return bool(position_error < 0.02 and orientation_error < 0.15)

    def diagnose(self, target_position, target_orientation) -> None:
        """Report why tracking plateaued: solver, plant, or joint limits.

        Three outcomes are possible and they need different fixes, so the dump
        separates them instead of leaving another guess to make. FK of the
        commanded configuration says where the solver *intended* to put the
        tool; the live pose says where the plant actually went; the per-joint
        limit margins say whether the solver had anywhere left to go.
        """
        print(f"[ik] consecutive solver failures at this point: {self._solver_failures}")
        if self._last_command is None:
            print("[ik] diagnose: no command issued yet")
            return

        pin = self._pin
        model = self._pink_robot.model
        data = self._pink_robot.data
        usd_joints = self._robot_joint_space
        target_position = np.asarray(target_position, dtype=float)

        actual = np.asarray(self._articulation.get_joint_positions(), dtype=float).reshape(-1)
        commanded_positions, commanded_indices = self._last_command
        commanded_full = actual.copy()
        commanded_full[commanded_indices] = commanded_positions

        lower = np.asarray(model.lowerPositionLimit, dtype=float)
        upper = np.asarray(model.upperPositionLimit, dtype=float)
        print("[ik] joint | actual | commanded | lower | upper | margin")
        for model_index, name in enumerate(self._kinematics_joints):
            usd_index = usd_joints.index(name)
            command = commanded_full[usd_index]
            margin = min(command - lower[model_index], upper[model_index] - command)
            flag = "  <-- AT LIMIT" if margin < 0.05 else ""
            print(
                f"[ik]   {name}: {actual[usd_index]:+.4f} | {command:+.4f} | "
                f"{lower[model_index]:+.4f} | {upper[model_index]:+.4f} | {margin:.4f}{flag}"
            )

        tracking = float(np.linalg.norm(commanded_full - actual))
        if model.nq == len(self._kinematics_joints):
            q_model = np.array(
                [commanded_full[usd_joints.index(name)] for name in self._kinematics_joints],
                dtype=float,
            )
            pin.forwardKinematics(model, data, q_model)
            pin.updateFramePlacements(model, data)
            frame = data.oMf[model.getFrameId(self._spec.ee_link_name)]
            commanded_tcp = np.asarray(frame.translation) + np.asarray(
                self._spec.position, dtype=float
            )
            solver_error = float(np.linalg.norm(commanded_tcp - target_position))
            live_position, _ = self.ee_pose()
            plant_error = float(np.linalg.norm(np.asarray(live_position) - commanded_tcp))
            print(
                f"[ik] solver: FK(commanded) is {solver_error * 1000:.1f}mm from the target; "
                f"plant: live TCP is {plant_error * 1000:.1f}mm from FK(commanded); "
                f"joint tracking error={tracking:.4f}rad"
            )
            print(
                f"[ik]   target={np.round(target_position, 4).tolist()} "
                f"FK(commanded)={np.round(commanded_tcp, 4).tolist()} "
                f"live={np.round(np.asarray(live_position), 4).tolist()}"
            )
        else:
            print(f"[ik] joint tracking error={tracking:.4f}rad (FK skipped: nq mismatch)")

    def reset(self) -> None:
        self._sim_time = 0.0
        self._controller_reset = False
        self._seed_applied = False

    def ee_pose(self):
        from isaacsim.core.prims import SingleXFormPrim

        return SingleXFormPrim(self._ee_path).get_world_pose()
