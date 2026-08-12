# Current Handoff

이 파일은 새 agent가 현재 작업 상태를 빠르게 이어받기 위한 짧은 상태표다.
장기 계약은 `docs/design/`이 정본이며, milestone이 바뀌면 이 파일만 현재형으로
갱신한다.

GraspGen ZMQ 연결의 구현 범위, 실행법, 실제 검증 결과와 현재 안전상 제한은
[`graspgen-integration.md`](graspgen-integration.md)를 참조한다.

## 현재 기준

- Runtime: Isaac Sim standalone 6.0.1 (`~/isaacsim`)뿐이다. **Isaac Lab은
  필요 없다** — manager-based task/gym 등록 없이 `isaacsim.core` API를
  직접 조립해서 쓴다.
- 워크스페이스명은 `isaac_indy7`로 확정(이전 `isaac_gnn`에서 리네임, 2026-07-12).
  동명의 내부 파이썬 패키지는 2026-08-12에 `sim/`·`control/`·`graspgen/`으로 분해됐다.
  현재 워크스페이스는 Indy7 단일 경로 — 스폰/IK/그리퍼/YCB/카메라 단계다.
- 루트 실행 파일은 `indy7.py` 하나, 실행은 `scripts/*.py` 파이썬 런처로 한다.
  재사용 코드는 `source/` 밑 주제별 패키지(`sim/`=스폰·카메라·ros2,
  `control/`=IK·grasp 실행, `graspgen/`=클라이언트·전처리·시각화 —
  mj_rl/mj_deploy와 같은 구조), 각 패키지 설정은 자기 `config.py`,
  USD 자산은 `source/assets/` 아래에 둔다.
- Repo: https://github.com/CheolMin-Yoon/isaac_indy7
- FFW(AI Worker BG2/SG2/SH5) USD들은 이 워크스페이스 소속이 아니다 —
  `~/gs_rl/source/assets/ffw/`로 옮겨졌다(Genesis 기반 워크스페이스, 2026-07-12).
  isaac_indy7는 Indy7 전용으로 유지한다.

## 완료된 기반

- **Indy7 + Robotiq 2F-140 결합**: binary 결합본의 혼합된 Z-axis/단일-mimic
  subtree를 비활성화하고 공식 UR10e 예제의 X-axis/5-mimic configured subtree를
  runtime에 조립한다. `tcp` 정렬과 fixed-joint frame은 공식 Robot Assembler
  계산을 쓰며, 최종 articulation은 self-collision OFF와 solver `16/1`을 강제한다.
  `finger_joint`는 `open=0`, `close=runtime upper limit=0.7 rad`다. open은 step
  0/60에서 `q=0`, close는 step 60에서 `q=0.7`임을 확인했다. 계약/자산 facts:
  `docs/design/indy7.md`.
- **Indy7 + YCB + IK 엔트리포인트**: 루트 `indy7.py`가 로봇/YCB 스폰,
  선택적 목표 pose 추종(`--target-position`, `--target-orientation`),
  선택적 그리퍼 명령(`--gripper open|close|hold`)을 담당한다. 기본은 open이다.
  YCB object는 관찰 중 kinematic으로 고정되고, 선택된 target 하나만 approach 진입
  시 dynamic으로 해제된다. no-action 180-step에서 초기 pose가 유지됐다.
- **공식 PINK 경로 검증**: teleop 내부 USD→URDF exporter를 제거하고
  `isaacsim.robot_motion.pink` 공개 API + 검증된
  `source/assets/indy7_v2/indy7_kinematics.urdf`로 교체했다. q=0 URDF/USD
  TCP FK가 일치하며 `[0.45, 0.0, 0.45]`, quaternion `[0, 0, 1, 0]`에
  live PhysX TCP가 수렴했다.
- **GraspGen 실행 경계**: instance mask→GraspGen→geometry gate→PINK
  pregrasp/approach/close/lift target은 연결됐다. tomato can 첫 시험은 TCP
  trajectory는 완료했지만 물체를 옆으로 밀어 실제 lift는 실패했다.
- **Indy7 wrist camera**: `source/sim/camera.py`의 `WristCamera`가
  TCP 하위 `zed_cam` Camera prim을 만들고, RGB/depth를 `output/camera/`에
  저장한다. 현재는 별도 ZED USD asset을 참조하지 않고 Isaac Sim Camera prim을
  프로그램으로 생성한다.

## 알려진 미해결 이슈

- `indy7_v2.usd`의 관절 `maxForce=100`은 실제 Indy7 축별 스펙 기준 검증값이
  아니다. 다만 이번 그리퍼 폭발은 self-collision 단일변수 A/B로 원인을
  분리했으므로 `maxForce` 문제로 해석하지 않는다.
- `indy7_v2.usd`에 레거시 `hand`/`MPLM1630` 페이로드가 남아 있다.
- Indy7 IK 목표 pose는 월드 좌표 기준이다. 자세/좌표계 튜닝은 GUI에서 확인하며
  잡아야 한다.
- Grasp executor의 `done`은 현재 TCP trajectory 완료이며 pick 성공을 보장하지
  않는다. target을 close가 아니라 approach에서 dynamic으로 해제하도록 교정했지만,
  이 변경 뒤 실제 lift 성공은 아직 재검증하지 않았다. object z/contact 기반
  lift-success gate가 다음 제어 작업이다.
- 카메라 mount pose는 Indy7 TCP 기준 초기값이다. 실제 ZED 형상/시야와 맞추려면
  mount offset, orientation, intrinsics를 조정해야 한다.

## 먼저 읽을 정본

- 실행 명령/옵션: `README.md`
- indy7 USD/articulation 계약: `docs/design/indy7.md`
- indy7 PINK IK 계약: `docs/design/indy7-ik.md`
- Isaac Sim 툴링 관련 재발 방지 노트:
  research-wiki `AI-Sessions/wiki/harness/errors/isaacsim-errors.md`
