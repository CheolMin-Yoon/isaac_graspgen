# isaac_indy7 — Indy7 Isaac Sim workspace

Isaac Sim standalone 6.0.1에서 Indy7 + Robotiq 2F-140, YCB object spawn,
Isaac Sim 공식 PINK IK, wrist camera capture를 실행하는 최소 워크스페이스다.

Repo: https://github.com/CheolMin-Yoon/isaac_indy7

> 새 세션에서 이어받을 때는 [`docs/handoff.md`](docs/handoff.md)를 먼저 확인한다.
> 자산/IK 계약은 [`docs/design/`](docs/design/)가 정본이다.

## Environment

- **필요한 건 Isaac Sim standalone 6.0.1(`~/isaacsim`)뿐이다. Isaac Lab은 필요 없다** —
  이 워크스페이스는 IsaacLab의 manager-based task/gym 등록 없이 `isaacsim.core`
  API를 직접 써서 스폰/IK/카메라를 조립한다.
- 반드시 Isaac Sim 번들 python으로 실행한다. 시스템 python에는 `isaacsim`/`omni`/`pxr`
  모듈이 없다.

PINK는 `isaacsim.robot_motion.pink` 공개 API와
`source/assets/indy7_v2/indy7_kinematics.urdf`를 사용하며, 실행 전
URDF/USD 관절 이름과 q=0 TCP FK 일치를 검사한다.

기본 그리퍼는 열린 상태다. YCB 물체는 관찰 중 kinematic으로 고정하고,
`--execute-grasp`에서 선택된 target만 approach 진입 시 dynamic으로 해제한다.

```bash
cd ~/isaac_indy7
```

## Structure

```
~/isaac_indy7/
├── README.md
├── docs/
│   ├── handoff.md
│   └── design/
│       ├── indy7.md
│       └── indy7-ik.md
├── indy7.py
├── scripts/          # python launchers (run_indy7.py, run_graspgen_server.py)
├── source/
│   ├── assets/
│   ├── sim/          # spawn.py, ycb.py, camera.py, ros2.py, config.py
│   ├── control/      # ik.py, grasp_execution.py, config.py
│   └── graspgen/     # client.py, pointcloud.py, visualization.py, config.py
└── output/camera/
```

## Run

`indy7.py`가 로봇, YCB 물체, TCP 하위 wrist camera를 스폰하고 Isaac Sim 내부에
ROS 2 Action Graph를 생성한다. 목표 pose를 주면 TCP가 해당 월드 pose를 PINK
differential IK로 추종하고, 목표를 주지 않으면 스폰 상태로 둔다.

```bash
./scripts/run_indy7.py
./scripts/run_indy7.py --target-position 0.45 0.0 0.35
./scripts/run_indy7.py --target-position 0.45 0.5 0.35 --target-orientation 0 0 1 0
./scripts/run_indy7.py --gripper close
./scripts/run_indy7.py --headless --max-steps 240
```

기본 실행 타이밍은 physics/contact 240 Hz, PINK IK/control 60 Hz, render/camera
60 Hz다. 실행 스크립트는 app loop를 60 Hz로 제한하고 CPU worker를 8개로
제한하며 single-GPU mode를 사용한다. 별도 wrist-camera viewport는 부하를 줄이기
위해 기본으로 열지 않으며 필요할 때만 `--wrist-viewport`를 붙인다. CPU worker
수는 예를 들어 `ISAAC_INDY7_CPU_THREADS=4 ./scripts/run_indy7.py`로 바꿀 수 있다.

### GraspGen 연결

GraspGen 모델은 별도 Conda 환경/ZMQ 서버에 유지하고 Isaac Sim은 point cloud만
보낸다. `better_pcd`와 `GraspGen` 소스는 수정하지 않는다.

최초 한 번 Isaac Sim 번들 Python에 가벼운 client 의존성을 설치한다.

```bash
./scripts/install_graspgen_client_deps.py
```

터미널 1에서 기존 GraspGen checkpoint 서버를 실행한다.

```bash
./scripts/run_graspgen_server.py
```

터미널 2에서 wrist-camera cloud를 Isaac instance mask로 선택한 뒤 2048점으로
맞춰 GraspGen에 한 번 전송한다.

```bash
./scripts/run_indy7.py --graspgen --grasp-object-index 0 --graspgen-step 180
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

카메라는 기본으로 `link6/d455` 아래 RealSense D455 prim을 사용하고, RGB/depth를 60스텝마다
`output/camera/`에 저장한다.

주요 옵션:

| 옵션 | 기본값 | 설명 |
|---|---|---|
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
| `--grasp-topk` | `20` | 반환 및 표시할 상위 grasp 수 |
| `--execute-grasp` | off | best grasp를 pregrasp→approach→close→lift로 IK 실행 (`--graspgen` 필요) |

GraspGen 연결/실행 기본값은 `source/graspgen/config.py`, grasp 실행 파라미터는
`source/control/config.py`, 씬 구성(로봇 USD, YCB, 카메라, dt)은
`source/sim/config.py`에 있다.

USD 자산 구조/articulation 계약: [`docs/design/indy7.md`](docs/design/indy7.md).
IK 설계 계약: [`docs/design/indy7-ik.md`](docs/design/indy7-ik.md).
depth 노이즈(RTX stereo depth sensor, Replicator augmentation):
[`docs/depth-sensor-noise.md`](docs/depth-sensor-noise.md).

## Notes

- 실행 시 `fabric::IStageReaderWriter` 버전 불일치 경고는 무시 가능.
- 시스템 `rclpy` 없음 경고는 내부 ROS2(jazzy) 자동 로드 경로라 보통 무시 가능.
- YCB가 안 불러와지면 인터넷 연결 또는 Nucleus 접근을 확인한다.
- GUI가 안 뜨면 `echo $DISPLAY` 값을 확인한다.
