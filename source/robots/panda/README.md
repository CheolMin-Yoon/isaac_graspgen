# Franka Panda

**Implemented** — `SPEC` in `__init__.py`, spawn in `spawn.py`, registered as
the default robot. This file records what it took, and what is still missing.

Adding the arm moved the `RobotSpec` seam twice, which is the intended
response when a second case does not fit:

- `kinematics_urdf: str` became `make_pink_robot: Callable`, because the Panda
  loads Isaac's bundled model rather than a path in this repo.
- `usd_path` gained `resolve_usd_path()`, because `franka.usd` lives under the
  Isaac assets root rather than on local disk.
- `wrist_camera_link` became optional, because `franka.usd` has no camera mount.
- `GripperSpec.joint_name` became `joint_names`, because the Panda hand drives
  two fingers where the Robotiq drives one.

## Still missing

1. **No wrist camera.** `franka.usd` carries no D455 mount, and `WristCamera`
   re-poses an existing mount rather than creating one, so `wrist_camera_link`
   is None and `--graspgen` is rejected for this arm. Either author a USD
   variant with a mount (the way `indy7_v2_with_2f-140_d455.usd` does) or teach
   `WristCamera` to create one.
2. **No GraspGen checkpoint.** GraspGen publishes a Franka Panda model but
   `/home/frlab/GraspGenModels/checkpoints/` only holds the Robotiq pair.
   `scripts/run_graspgen_server.py --gripper panda_hand` will exit naming the
   missing file. Fetch from HuggingFace `adithyamurali/GraspGenModels`.
3. **Scene geometry is still Indy7-shaped.** `YCB_CONFIG["spawn"]` places
   objects on a 0.70 m arc tuned to the Indy7's reach, and the
   `source/graspgen/config.py` prefilter gates encode a table-mounted arm
   reaching outward. Neither has been re-checked against the Panda's workspace.

## Where the values came from

Everything is Isaac's, not ours — see
`exts/isaacsim.robot_motion.pink.examples/.../ik_controller/scenario.py`
(`FrankaPinkIKExample`) and `.../multi_task/scenario.py`.

| fact | value |
|---|---|
| controller gains | `position_cost=5.0`, `orientation_cost=0.05`, `posture_cost=5e-3`, `solver="osqp"`, `dt=1/60` — already what `PinkArmIK` hardcodes |
| tool frame | `panda_hand` |
| hand → fingertip midpoint | `[0.0, 0.0, 0.1034]`, used as `graspgen_depth` |
| reach posture | j1 `0.012`, j2 `-0.568`, j3 `0.0`, j4 `-2.811`, j5 `0.0`, j6 `3.037`, j7 `0.741` |
| finger joints | `panda_finger_joint1` / `panda_finger_joint2`, both `0.035` open |
| kinematics | `load_pink_supported_robot("franka")` — bundled under `robot_configurations/`, which also has `ur10` |
| USD | `{assets_root}/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd` |

## Adding a third arm

1. Assets: a USD, and either a checked URDF (like Indy7) or a bundled name
   (like Panda). `PinkArmIK` refuses to run unless the controlled joint names
   match the USD DOF order **and** the zero-pose FK of the tool frame agrees
   with the live USD to 1e-4.
2. `spawn(spec) -> SingleArticulation`. Indy7's does real surgery because its
   legacy asset shipped a broken mimic graph; the Panda's is 20 lines. If yours
   needs surgery, that is a problem with the asset.
3. Gripper: reuse a `GripperSpec` if the hardware matches, otherwise add one
   plus a controller class. GraspGen is trained per gripper — its checkpoints
   travel with the `GripperSpec`, not with the arm.
4. Export `SPEC` and add it to `_ROBOT_MODULES` in `source/robots/__init__.py`.

## 0. What Isaac already gives you

Isaac Sim 6.0.1 ships two official Franka Panda PINK examples. Read them before
writing anything — most of this arm is already solved there:

- `exts/isaacsim.robot_motion.pink.examples/.../ik_controller/scenario.py`
  (`FrankaPinkIKExample`) — the basic controller.
- `.../multi_task/scenario.py` (`FrankaMultiTaskExample`) — FrameTask +
  PostureTask + DampingTask.

Concrete values they establish:

| fact | value |
|---|---|
| controller gains | `position_cost=5.0`, `orientation_cost=0.05`, `posture_cost=5e-3`, `solver="osqp"`, `dt=1/60` |
| tool frame | `panda_hand` |
| hand → fingertip midpoint | `[0.0, 0.0, 0.1034]` |
| reach posture | j1 `0.012`, j2 `-0.568`, j3 `0.0`, j4 `-2.811`, j5 `0.0`, j6 `3.037`, j7 `0.741` |
| finger joints | `panda_finger_joint1` / `panda_finger_joint2`, both `0.035` open |

The gains are already what `PinkArmIK` hardcodes — Indy7 inherited them from
this same example — so there is nothing to retune.

## 1. Assets

- **Kinematics: do not author a URDF.** Isaac bundles one at
  `exts/isaacsim.robot_motion.pink/robot_configurations/franka/robot.urdf`,
  reachable as `load_pink_supported_robot("franka")`, which also wires up the
  SRDF and package dirs. `SPEC.make_pink_robot` exists precisely so this arm
  can call that instead of `load_pink_robot(path)`. Set
  `kinematics_source="bundled:franka"`.
  (Only `franka` and `ur10` are bundled; Indy7 is not, which is why it carries
  its own URDF.)
- **USD**: the official asset is
  `{assets_root}/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd`. Note it
  has no wrist camera mount — `WristCamera` re-poses an existing mount, it does
  not create one, so either author a variant that carries a D455 the way
  `indy7_v2_with_2f-140_d455.usd` does, or extend `WristCamera` first.

Whichever kinematics you use, `PinkArmIK` refuses to run unless the controlled
joint names match the USD DOF order **and** the zero-pose FK of the tool frame
agrees with the live USD to 1e-4. Both checks fire at startup.

## 2. `source/robots/panda/spawn.py`

Export `spawn(spec) -> SingleArticulation`. The Indy7 version does real surgery
— it grafts Isaac's official Robotiq subtree onto the arm's TCP because the
legacy combined asset shipped a broken mimic graph. A Panda carrying its own
hand should need far less than that; if it does not, that is a problem with the
asset, not something to paper over in `spawn`.

## 3. Gripper

If the Panda keeps its own hand, it is **not** a `SingleJointGripper` — that
class drives exactly one DOF and lets mimic joints follow. The Panda hand's two
prismatic finger joints are independently actuated, so it needs its own
controller class plus a `GripperSpec` in `source/robots/gripper.py`, including
the matching GraspGen checkpoint pair (`graspgen_*`). GraspGen is trained per
gripper; the Robotiq checkpoints will not transfer.

If instead the Panda carries the same Robotiq 2F-140, reuse `ROBOTIQ_2F_140`
unchanged — that reuse is the reason the gripper spec is separate from the arm.

## 4. `source/robots/panda/__init__.py`

Export `SPEC = RobotSpec(...)` with

```python
def _make_pink_robot():
    from isaacsim.robot_motion.pink import load_pink_supported_robot
    return load_pink_supported_robot("franka")
```

`reach_posture` must have exactly `num_arm_dofs` entries (7 for the Panda, vs
Indy7's 6) and should be a bent, nonsingular posture — it is the seed applied
before reactive tracking starts. Use the example's values above, arm joints
only; the finger entries belong to the gripper.

## 5. Register it

Add `"panda": "robots.panda"` to `_ROBOT_MODULES` in `source/robots/__init__.py`.
Then `--robot panda` works everywhere.

## Not yet abstracted

These are still Indy7-shaped and will need attention:

- `source/sim/config.py` `YCB_CONFIG["spawn"]` places objects on a radius/arc
  tuned to the Indy7's reach.
- `source/graspgen/config.py` execution prefilter thresholds
  (`MIN_GRIPPER_BASE_Z`, `MAX_APPROACH_Z`, `MIN_OUTWARD_APPROACH`) encode a
  table-mounted arm reaching outward.
