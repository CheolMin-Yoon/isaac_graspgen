# Indy7 IK Design

`source/control/ik.py`의 `Indy7IK`가 Indy7 IK 계약의 owner다. 로봇
USD/articulation facts는 [`indy7.md`](indy7.md)를 본다.

## 결정: Isaac Sim 공식 PINK 경로

Indy7은 Isaac Sim에 번들된 Lula robot descriptor가 없으므로, 현재 IK backend는
Isaac Sim 6.0.1의 공개 PINK API다.

```python
from isaacsim.robot_motion.pink import PinkIKController, load_pink_robot
```

구현은 Isaac Sim 공식 Franka PINK 예제의 계약을 그대로 따른다.

1. 검증된 URDF를 `load_pink_robot()`으로 읽는다.
2. 완전히 편 영점 자세 대신 굽힌 reach posture를 articulation position/target에
   먼저 넣는다.
3. 매 60 Hz control step마다 `RobotState`와 목표 `SpatialState`를 만들고
   `PinkIKController.forward()`를 호출한다.
4. solver가 반환한 arm joint position target만 live USD articulation에 적용한다.

공식 예제의 controller 설정과 같은 값을 사용한다.

| 항목 | 값 |
|---|---:|
| solver | `osqp` |
| control dt | `1/60 s` |
| position cost | `5.0` |
| orientation cost | `0.05` |
| posture cost | `5e-3` |

## 운동학 URDF와 live plant의 경계

[`source/assets/indy7_v2/indy7_kinematics.urdf`](../../source/assets/indy7_v2/indy7_kinematics.urdf)는
PINK/Pinocchio가 읽는 **운동학 전용 파일**이다. 링크 visual, collision, inertia와
Robotiq 물리는 live combined USD가 소유한다.

- URDF: `joint0`~`joint5`, `tcp` frame의 kinematic chain
- USD: articulation, drive, collision, mass, Robotiq gripper, simulation state
- PINK 출력: 앞 6개 arm DOF의 position target
- gripper 출력: `Indy7Gripper`가 `finger_joint`를 별도로 제어

URDF는 Isaac Sim 6.0.1의 공식 `isaacsim.asset.exporter.urdf` 결과에서 정확한
joint origin/axis/limit을 가져왔다. Pinocchio parsing을 위해 effort limit을
명시하고, link `tcp`와 중복되지 않도록 fixed joint 이름만 `tcp_joint`로 정규화했다.

`Indy7IK` 초기화는 다음 두 검사를 통과하지 못하면 명령을 내리지 않는다.

- URDF controlled joint names와 live USD 앞 6개 DOF 이름이 같은가
- q=0에서 URDF `tcp` FK와 live USD `tcp` world pose가 일치하는가

현재 q=0 TCP 기준값은 position `[-0.0, -0.2025, 1.2115] m`, orientation
identity(wxyz)다.

## 기존 구현이 실패한 원인

PINK solver 자체의 문제가 아니었다. 이전 구현은
`isaacsim.replicator.teleop` 내부의 임시 USD→URDF exporter와 내부 controller를
사용했다. 그 exporter가 이 커스텀 Indy7 계층을 잘못 변환하여 q=0 TCP FK부터
live USD와 크게 달랐다.

```text
live USD q=0 TCP      ≈ [ 0.0000, -0.2025,  1.2115]
old temporary URDF   ≈ [-0.4900, -0.1300, -0.0270]
official-export URDF ≈ [ 0.0000, -0.2025,  1.2115]
```

따라서 당시의 비수렴과 joint-limit 포화는 PINK의 한계가 아니라 서로 다른
kinematic model로 계산한 명령을 plant에 적용한 결과다. teleop 내부 exporter는
다시 사용하지 않는다.

## Interface

```python
from control.ik import Indy7IK

ik = Indy7IK(indy7)
ik.go_to(position, orientation)  # position xyz[m], orientation Isaac wxyz
ik.ee_pose()                     # live TCP world pose
ik.ee_path                       # live TCP prim path
ik.reset()                       # PINK state와 reach-posture seed 초기화
```

`go_to()`는 one-shot IK가 아니라 reactive differential IK 한 step이다. 반환값은
현재 live TCP가 목표 position 2 cm, orientation 0.15 rad 이내에 들어왔는지를
뜻한다. 실제 실행 loop에서 매 control step 반복 호출해야 한다.

## 검증

2026-08-12, 다음 live PhysX command를 실행했다.

```bash
./scripts/run_indy7.py --headless --max-steps 500 \
  --target-position 0.45 0.0 0.45 \
  --target-orientation 0 0 1 0
```

- step 120: TCP position `[0.4500, 0.0000, 0.4499]`, `reachable=True`
- step 180: orientation이 목표 quaternion에 수렴
- process exit code: `0`

같은 backend로 GraspGen 실행 상태 머신도 pregrasp→approach→close→lift target까지
추종했다. 다만 첫 tomato-can 시험에서는 물체 높이가 증가하지 않아 실제 grasp/lift는
실패했다. 이는 현재 grasp 후보의 접촉 적합성 문제이며 IK 성공과 별도로 판정한다.
