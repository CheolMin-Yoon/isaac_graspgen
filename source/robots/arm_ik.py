"""Official Isaac PINK IK, driven by a ``RobotSpec``.

The controller itself is arm-agnostic — everything that differs per arm (the
kinematics model, the tool frame, the arm DOF count, the seed posture) comes in
through the spec. The two startup checks below are the reason this stays one
class instead of per-robot copies: they reject a kinematics model that has
drifted from the USD before it can command the plant.
"""

from __future__ import annotations

import numpy as np


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

        estimated_state = self._mg.RobotState(
            joints=self._mg.JointState.from_name(
                robot_joint_space=self._robot_joint_space,
                positions=(self._robot_joint_space, self._exp_robot.get_dof_positions()),
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
            self._exp_robot.set_dof_position_targets(
                positions=desired_state.joints.positions,
                dof_indices=desired_state.joints.position_indices,
            )
        self._sim_time += self._dt

        current_position, current_orientation = self.ee_pose()
        position_error = float(np.linalg.norm(target_position - np.asarray(current_position)))
        orientation_error = 2.0 * np.arccos(
            np.clip(abs(float(np.dot(target_orientation, current_orientation))), 0.0, 1.0)
        )
        return bool(position_error < 0.02 and orientation_error < 0.15)

    def reset(self) -> None:
        self._sim_time = 0.0
        self._controller_reset = False
        self._seed_applied = False

    def ee_pose(self):
        from isaacsim.core.prims import SingleXFormPrim

        return SingleXFormPrim(self._ee_path).get_world_pose()
