# Indy7 Asset Design

`source/robots/indy7/assets/`가 indy7 로봇 USD 계약의 owner다. articulation/솔버 파라미터
facts는 이 문서가 정본이며, IK 쪽 계약은 [`indy7-ik.md`](indy7-ik.md)를 본다.

## Files

### `source/robots/indy7/assets/indy7_v2.usd`

indy7 6축 팔 단독 USD. Owner of:

- Articulation root: `/indy7_v2` (top-level Xform) — `PhysicsArticulationRootAPI` +
  `PhysxArticulationAPI` 적용.
- Joint chain: `root_joint`(FixedJoint, world → `link0`) → `joint0..joint5`
  (RevoluteJoint, `link0→link1→...→link6`) → `tcp`(FixedJoint, `link6→tcp`).
  `dof_names`는 `["joint0", ..., "joint5"]` 6개.
- 소스 USD의 authored articulation 값은 `self-collision=true`, solver `32/4`다.
  이 값은 폐루프 Robotiq에 안전하지 않으므로 실행 계약이 아니다.
- 실제 plant 계약은 `source/robots/indy7/spawn.py`가 최종 composed articulation root에
  `self-collision=false`, solver `16/1`로 override한다. child/base USD 중 하나만
  보고 런타임 값을 추론하지 않는다.
  - `drive:angular:physics:stiffness = 10000`, `damping = 100`,
    `maxForce = 100` (6개 관절 전부 동일). 이 값은 실제 Indy7 축별 사양으로
    검증된 값이 아니므로 별도 plant qualification 대상이지만, 아래 Play 시
    그리퍼 폭발의 원인은 아니었다.
- 레거시 `hand`/`MPLM1630` 페이로드도 이 파일 안에 남아 있다(원 authoring
  시 딸려온 제네릭 그리퍼, prismatic joint 2개). 지금은 아래
  `indy7_v2_with_2f-140_d455.usd` 쪽 Robotiq 결합이 실사용 경로이고,
  이 페이로드는 정리 대상이지만 아직 제거하지 않았다.
- **로봇 툴링 인식**: `IsaacRobotAPI`(`isaacsim.robot.schema`)가 articulation
  root에 별도로 붙어 있어야 Robot Assembler/Robot Inspector가 이 prim을
  로봇으로 인식한다. `PhysicsArticulationRootAPI`만으로는 물리 시뮬레이션은
  되지만 툴링에는 안 잡힌다 — 전체 근거는 research-wiki
  `AI-Sessions/wiki/harness/errors/isaacsim-errors.md` 참고.

### Robotiq 실행 조립

현재 팔·D455 base는 `indy7_v2_with_2f-140_d455.usd`를 사용하고,
`indy7_v2_with_robotiq_2f_140.usd`는 카메라 없는 이전 결합본이다.

Robot Assembler로 만든 결합 스테이지. **`indy7_v2.usd`를 상대 payload로
참조**하므로(`@./indy7_v2.usd@`) base와 결합 USD들은 항상 같은 폴더에 같이
있어야 한다 — 결합본 하나만 옮기면 깨진다.

- `/World/indy7_v2` — 위 `indy7_v2.usd`를 payload, `IsaacRobotAPI` 등 로봇
  스키마 override 추가.
- 바이너리 결합본의 기존 `/Robotiq_2F_140_config`는 Z-axis 관절에
  `rotX` mimic 관계 하나만 남은 혼합 구성이다. open target 0을 주어도
  master joint가 약 `0.7854 rad`에 머물러 실행 시 비활성화한다.
- `source/robots/indy7/spawn.py`가 Isaac Sim 6.0.1 공식 UR10e gripper 예제의
  `/ur/ee_link` subtree를 참조한다. 이 configured graph는 X-axis 관절과
  `finger_joint`를 참조하는 5개 `PhysxMimicJointAPI:rotX`를 갖는다.
- 공식 subtree의 기존 UR10 fixed joint를 비활성화하고, 공식 Robot
  Assembler의 `set_opposite_body_transform()`으로 Indy7 `tcp` ↔
  `robotiq_arg2f_base_link`를 용접한다. 정렬 오차는 `0`이다.
- 최종 articulation `dof_names`는 `joint0..joint5` 뒤에 mimic master
  `finger_joint`가 이어진 7개다.
- 런타임 그리퍼 명령은 `source/robots/gripper.py::SingleJointGripper`가 소유한다.
  `open=0`, `close=runtime upper limit=0.7 rad`이다.

#### Play 시 폭발과 self-collision 계약

공식 Robotiq asset을 썼더라도 fixed joint로 Indy7에 붙이면 그리퍼는 상위 Indy7
articulation 설정을 상속한다. 2F-140은 폐루프/인접 링크 collider를 포함하므로
상위 root의 `enabledSelfCollisions=true`가 적용되면 내부 collider 침투 해소가
첫 simulation step부터 큰 속도를 만들 수 있다.

공식 production UR10e의 `Robotiq_2f_140` variant는 같은 Robotiq 조인트 graph,
limit와 `excludeFromArticulation` 구성을 사용하지만 articulation root의
self-collision을 명시적으로 끈다. 별도 collision group이나 filtered-pair로
우회하지 않는다.

현재 Indy7 + 공식 configured 2F-140에서 나머지 조건을 고정한
A/B 결과:

- self-collision ON: open target 0에서 `q≈0.115 rad`, mimic/drive effort 약
  `187–200`으로 jam.
- self-collision OFF, solver 16/1: open은 step 0/60 모두 `q≈0`, close는
  step 60에 upper limit `0.7 rad`에 도달.

자산 graph 혼합은 그리퍼가 안 열리는 첫 원인이고, 새 공식 graph을
jam하게 한 즉시 원인은 최종 composed articulation의 self-collision이었다.
이를 끄더라도 로봇과 YCB/ShapeNet 같은 외부 물체 사이 collision은 유지된다.

## General Rules

- USD를 직접 authoring/수정할 때는 `PhysicsArticulationRootAPI`(물리용)와
  `IsaacRobotAPI`(툴링용)를 별개로 취급한다. 하나만 붙이고 끝내지 않는다.
- Robot Assembler로 재조립하면 drive target만 보지 말고 joint axis, 적용된
  mimic schema 수, mimic relationship target/gearing까지 공식 configured asset과 비교한다.
- 폐루프 Robotiq을 결합한 articulation은 공식 UR10e 구성처럼
  `enabledSelfCollisions=false`를 유지한다. 소스 USD나 문서가 아니라 스폰 후
  최종 composed articulation root 값을 확인한다.
- `indy7_v2.usd`와 이를 참조하는 Robotiq 결합 USD들은 항상 같은 폴더에서
  같이 이동/커밋한다.
