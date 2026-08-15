# isaac_graspgen — Isaac Sim grasp workspace

Isaac Sim standalone 6.0.1에서 매니퓰레이터 + 그리퍼, YCB object spawn,
Isaac Sim 공식 PINK IK, wrist camera capture, GraspGen 연동을 실행하는 최소
워크스페이스다. 로봇은 `--robot`으로 고른다.

| 로봇 | 그리퍼 | kinematics | wrist camera | GraspGen |
|---|---|---|---|---|
| `panda` (기본) | Panda hand | Isaac 번들 `franka` | pinhole | 사용 가능 |
| `indy7` | Robotiq 2F-140 | 자체 URDF | D455 | 사용 가능 |

Panda 단일 캔 pick/lift 기준 설정과 새 로봇 추가 계약은
[`source/robots/panda/README.md`](source/robots/panda/README.md)를 본다.

Repo: https://github.com/CheolMin-Yoon/isaac_graspgen

> 새 세션에서 이어받을 때는 [`docs/handoff.md`](docs/handoff.md)를 먼저 확인한다.
> 자산/IK 계약은 [`docs/design/`](docs/design/)가 정본이다.

## Environment

- **필요한 건 Isaac Sim standalone 6.0.1(`~/isaacsim`)뿐이다. Isaac Lab은 필요 없다** —
  이 워크스페이스는 IsaacLab의 manager-based task/gym 등록 없이 `isaacsim.core`
  API를 직접 써서 스폰/IK/카메라를 조립한다.
- 반드시 Isaac Sim 번들 python으로 실행한다. 시스템 python에는 `isaacsim`/`omni`/`pxr`
  모듈이 없다.
- 단, `tests/`는 Isaac 없이 도는 순수 pytest이고 실행 python이 또 다르다.
  Isaac 번들 python에는 `numpy`가, 시스템 python에는 `msgpack-numpy`가 없다.
  현재 세 의존성(`pyzmq`/`msgpack`/`msgpack-numpy`)이 모두 있는 건 graspgen
  conda 환경뿐이다.

  ```bash
  /home/frlab/anaconda3/envs/graspgen/bin/python -m pytest tests/ -q
  ```

PINK는 `isaacsim.robot_motion.pink` 공개 API와 로봇별 kinematics URDF
(Indy7은 `source/robots/indy7/assets/indy7_kinematics.urdf`)를 사용하며, 실행 전
URDF/USD 관절 이름과 q=0 TCP FK 일치를 검사한다.

기본 그리퍼는 열린 상태다. YCB 물체는 dynamic으로 스폰해 물리로 정지시킨 뒤
그 자세에서 kinematic으로 고정하고, `--execute-grasp`에서 선택된 target만
approach 진입 시 다시 dynamic으로 해제한다.

> 스폰 자세를 물리로 검증한 뒤 고정하는 이유가 있다. Isaac의
> `/Isaac/Props/YCB/Axis_Aligned/` 자산은 메타데이터로 `upAxis=Z`를 선언하지만
> **지오메트리는 Y축이 높이**다(토마토 수프 캔이 X·Z 0.068 m, Y 0.102 m).
> 참조만으로는 회전되지 않으므로 보정 없이 놓으면 전부 옆으로 눕고, 그 상태로
> 고정하면 누운 씬이 멀쩡해 보인다. `source/sim/ycb.py`의
> `up_axis_correction()`이 이를 세운다. 오목 형상(bowl/mug)은 convex hull이
> 속을 막아버려 굴러가므로 `convexDecomposition`을 쓴다 — 이 둘은 **함께**
> 적용해야 하며, 하나만으로는 다른 물체가 무너진다.
>
> 스폰을 건드렸다면 `--ycb-dynamic`으로 검사한다. 고정을 생략하고 종료 시
> 물체별 이동/회전량을 출력하므로, 배치가 실제로 안정한지 바로 보인다.

```bash
cd ~/isaac_graspgen
```

## Structure

로봇 종속 사실은 전부 `source/robots/<name>/`에 모여 있고, `sim/`·`control/`·
`graspgen/`은 어떤 팔인지 모른 채 동작한다. 팔을 추가하는 절차는
[`source/robots/panda/README.md`](source/robots/panda/README.md)에 있다.

```
~/isaac_graspgen/
├── README.md
├── docs/
│   ├── handoff.md
│   └── design/
│       ├── indy7.md
│       └── indy7-ik.md
├── grasp_scene.py    # SimulationApp bootstrap
├── scripts/          # python launchers (run_scene.py, run_graspgen_server.py)
├── source/
│   ├── robots/       # 로봇 레지스트리
│   │   ├── base.py       # RobotSpec, GripperSpec
│   │   ├── arm_ik.py     # PinkArmIK (spec 기반, 팔 무관)
│   │   ├── gripper.py    # SingleJointGripper, ROBOTIQ_2F_140
│   │   ├── indy7/        # SPEC, spawn.py, assets/
│   │   └── panda/        # SPEC, spawn.py, 로봇 추가 계약
│   ├── assets/       # 씬 자산 (ycb_overlap)
│   ├── sim/          # CLI/runtime, ycb, camera, ros2, config
│   ├── control/      # grasp_execution.py, config.py
│   └── graspgen/     # client.py, pointcloud.py, visualization.py, config.py
└── output/camera/
```

## Run

`grasp_scene.py`가 `--robot`으로 고른 로봇, YCB 물체, wrist camera를 스폰하고
Isaac Sim 내부에 ROS 2 Action Graph를 생성한다. 목표 pose를 주면 TCP가 해당
월드 pose를 PINK differential IK로 추종하고, 목표를 주지 않으면 스폰 상태로 둔다.

```bash
./scripts/run_scene.py
./scripts/run_scene.py --robot indy7
./scripts/run_scene.py --target-position 0.45 0.0 0.35
./scripts/run_scene.py --target-position 0.45 0.5 0.35 --target-orientation 0 0 1 0
./scripts/run_scene.py --gripper close
./scripts/run_scene.py --headless --max-steps 240
```

기본 실행 타이밍은 physics/contact 240 Hz, PINK IK/control 60 Hz, render/camera
60 Hz다. 실행 스크립트는 app loop를 60 Hz로 제한하고 CPU worker를 8개로
제한하며 single-GPU mode를 사용한다. 별도 wrist-camera viewport는 부하를 줄이기
위해 기본으로 열지 않으며 필요할 때만 `--wrist-viewport`를 붙인다. CPU worker
수는 예를 들어 `ISAAC_GRASPGEN_CPU_THREADS=4 ./scripts/run_scene.py`로 바꿀 수 있다.

### GraspGen 연결

GraspGen 모델은 별도 Conda 환경/ZMQ 서버에 유지하고 Isaac Sim은 point cloud만
보낸다. `better_pcd`와 `GraspGen` 소스는 수정하지 않는다.

두 Python 런타임의 경로, 의존성 경계, gripper별 실행 조합과 연결 점검은
[`docs/config/runtime-environments.md`](docs/config/runtime-environments.md)에 정리했다.

> 경로 주의: 업스트림 NVIDIA GraspGen 체크아웃은 `/home/frlab/GraspGen`이고,
> 이 워크스페이스는 `/home/frlab/isaac_graspgen`이다. 이름이 비슷하지만 서로
> 다른 저장소이며, 둘은 ZMQ로만 통신한다. 서버 경로는
> `source/graspgen/config.py`의 `SERVER_ROOT`에 있다.

최초 한 번 Isaac Sim 번들 Python에 가벼운 client 의존성을 설치한다.

```bash
./scripts/install_graspgen_client_deps.py
```

터미널 1에서 기존 GraspGen checkpoint 서버를 실행한다. GraspGen은 팔이 아니라
그리퍼 단위로 학습되므로 체크포인트는 `--gripper`로 고른다(기본
`robotiq_2f_140`, 경로는 `source/robots/gripper.py`의 GripperSpec에 있다).

```bash
./scripts/run_graspgen_server.py
./scripts/run_graspgen_server.py --gripper robotiq_2f_140
./scripts/run_graspgen_server.py --gripper panda_hand
```

터미널 2에서 wrist-camera cloud를 Isaac instance mask로 선택한 뒤 2048점으로
맞춰 GraspGen에 한 번 전송한다.

```bash
./scripts/run_scene.py --graspgen --grasp-object-index 0 --graspgen-step 180
```

반환된 grasp 중 상위 pose는 viewport에 RGB 축으로 표시한다. 동일한 입력과 최고
grasp pose는 각각 `output/graspgen/input_world.npy`,
`output/graspgen/best_grasp_world.npy`에 저장된다. 기본값은 좌표계 검증을
위해 **시각화까지만 하며 로봇은 자동으로 움직이지 않는다.** `--execute-grasp`를
붙이면 best grasp를 pregrasp→approach→close→lift 상태기계로 실행한다
(`source/control/grasp_execution.py`).
현재 `done`은 TCP trajectory 완료를 뜻하며, 실제 물체 lift 성공은 아직
별도 gate로 확인해야 한다.

구현 범위, 실제 smoke-test 수치 및 현재 알려진 안전상 제한은
[`docs/graspgen-integration.md`](docs/graspgen-integration.md)에 기록되어 있다.

현재 object 선택은 YOLO가 아니라 Isaac Sim 정답 instance mask를 사용하는
oracle-perception baseline이다.

Timeline이 재생되면 Action Graph에서 다음 ROS 2 interface가 활성화된다.

| 구분 | Topic | 메시지 타입 | 설명 |
|---|---|---|---|
| Publisher | `/indy7/joint_states` | `sensor_msgs/msg/JointState` | 현재 관절 상태 |
| Publisher | `/clock` | `rosgraph_msgs/msg/Clock` | 시뮬레이션 시간 |
| Publisher | `/tf` | `tf2_msgs/msg/TFMessage` | Indy7 transform tree |
| Subscriber | `/indy7/joint_command` | `sensor_msgs/msg/JointState` | 관절 명령 입력 |

외부 ROS 2 Jazzy terminal에서 확인한다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list -t
ros2 topic echo /indy7/joint_states --once
```

카메라는 `SPEC.wrist_camera` 계약을 사용한다. Indy7은 `link6/d455`
자산을 감싸고 Panda는 `panda_hand` 아래 pinhole prim을 만든다.

주요 옵션:

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--robot` | `panda` | 구동할 로봇, `source/robots` 레지스트리에 등록된 이름 |
| `--headless` | off | GUI 없이 실행 |
| `--max-steps` | `0` | N 스텝 후 자동 종료, 0은 무한 |
| `--wrist-viewport` | off | 별도의 wrist-camera viewport를 열기 |
| `--target-position X Y Z` | 없음 | 지정하면 TCP가 해당 월드 좌표로 이동 |
| `--target-orientation W X Y Z` | `0 0 1 0` | 목표 TCP orientation, Isaac wxyz quaternion |
| `--gripper` | `open` | `open`, `close`, `hold`; `hold`는 USD 초기 관절 자세 유지 |
| `--graspgen` | off | GraspGen server 연결 및 1회 inference/시각화 |
| `--grasp-object-index` | `0` | GraspGen 입력으로 사용할 YCB object index |
| `--graspgen-step` | `180` | inference를 실행할 simulation step |
| `--grasp-point-count` | `2048` | GraspGen에 전송할 point 수 |
| `--grasp-num-grasps` | `200` | diffusion grasp sample 수 |
| `--grasp-topk` | `60` | 반환 및 표시할 상위 grasp 수 |
| `--execute-grasp` | off | best grasp를 pregrasp→approach→close→lift로 IK 실행 (`--graspgen` 필요) |

GraspGen 연결/실행 기본값은 `source/graspgen/config.py`, grasp 실행 파라미터는
`source/control/config.py`, 씬 구성(YCB, 카메라, dt)은 `source/sim/config.py`에
있다. 로봇 USD·prim path·base pose·kinematics URDF·TCP·wrist camera link 등
로봇 종속 사실은 `source/robots/<name>/`의 `SPEC`이 정본이다.

USD 자산 구조/articulation 계약: [`docs/design/indy7.md`](docs/design/indy7.md).
IK 설계 계약: [`docs/design/indy7-ik.md`](docs/design/indy7-ik.md).
depth 노이즈(RTX stereo depth sensor, Replicator augmentation):
[`docs/depth-sensor-noise.md`](docs/depth-sensor-noise.md).

## Notes

- 실행 시 `fabric::IStageReaderWriter` 버전 불일치 경고는 무시 가능.
- 시스템 `rclpy` 없음 경고는 내부 ROS2(jazzy) 자동 로드 경로라 보통 무시 가능.
- YCB가 안 불러와지면 인터넷 연결 또는 Nucleus 접근을 확인한다.
- GUI가 안 뜨면 `echo $DISPLAY` 값을 확인한다.
