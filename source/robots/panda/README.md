# Robot integration contract

Panda is the default implementation and Indy7 is the second registered arm.
The scene accesses both only through `RobotSpec`; robot-specific asset,
kinematics, camera, and gripper facts stay under `source/robots/`.

## Panda

| fact | value |
|---|---|
| USD | Isaac asset `/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd` |
| kinematics | `load_pink_supported_robot("franka")` |
| tool frame | `panda_hand` |
| arm DOFs | first 7 articulation joints |
| gripper | two independently driven `panda_finger_joint*` joints |
| camera | calibrated pinhole prim created under `panda_hand` |
| GraspGen model | external `graspgen_franka_panda` checkpoints |

The tested single-can scene uses `--ycb-radius 0.55`; the global 0.70 m
default is retained for Indy7. `franka.usd` authors a drive only on finger 1,
so `ParallelFingerGripper` copies its gains and force limit to finger 2 during
startup.

Three Panda invariants are part of the working pick/lift path:

1. `PinkArmIK` clamps measured joints just inside the kinematic limits before
   solving. PhysX can exceed a limit by a few microradians.
2. IK writes only the first seven arm DOFs. The gripper controller exclusively
   owns both finger DOFs.
3. The selected YCB target becomes dynamic on entry to `approach`, before
   finger contact; observation distractors remain kinematic.

Do not fold these back into scene heuristics. They are controller/plant
ownership boundaries, and regressions can look like an IK stall or a false
grasp while all commands appear valid.

## Add another robot

1. Add `source/robots/<name>/__init__.py` exporting one `RobotSpec` and a
   `spawn(spec)` implementation. Use a checked URDF via `load_pink_robot`, or a
   supported Isaac model via `load_pink_supported_robot`.
2. Reuse a `GripperSpec` when hardware matches. Otherwise add the gripper spec
   and the smallest controller matching its actuation layout; GraspGen model
   paths belong to the gripper, not the arm.
3. Put the camera mount in `RobotSpec.wrist_camera`: `mode="asset"` wraps a
   camera already present in the USD, while `mode="pinhole"` creates one.
4. Register the module in `_ROBOT_MODULES` in `source/robots/__init__.py`, then
   extend the plain-Python registry tests.

At startup `PinkArmIK` verifies that kinematic joints exist in the articulation,
that the arm prefix has the same order, and that live USD and model FK agree.
Keep Isaac imports inside factories so registry and CLI tests remain runnable
without Isaac Sim.

## Runtime ownership

- `grasp_scene.py`: starts and closes `SimulationApp` only.
- `sim/cli.py`: Isaac-free argument contract and early validation.
- `sim/runner.py`: scene lifecycle, reset, inference, and phase integration.
- `control/grasp_execution.py`: robot-agnostic grasp plan and phase machine.
- `robots/arm_ik.py`, `robots/gripper.py`: plant-specific command boundaries.
