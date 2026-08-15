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
- 워크스페이스명은 `isaac_graspgen`으로 확정(2026-08-15 리네임; 이전 이름
  `isaac_indy7`, 그 전 `isaac_gnn`). 동명의 내부 파이썬 패키지는 2026-08-12에
  `sim/`·`control/`·`graspgen/`으로 분해됐고, 2026-08-15에 로봇 종속 코드가
  `robots/`로 다시 분리됐다.
- **방향 전환(2026-08-15): 이 워크스페이스는 더 이상 Indy7 전용이 아니다.**
  이전 handoff는 "isaac_indy7는 Indy7 전용으로 유지한다"고 적었으나, Panda를
  같은 워크스페이스에서 다루기로 하면서 뒤집혔다. 팔은 `--robot`으로 고르고,
  로봇 종속 사실은 `source/robots/<name>/`의 `SPEC`(`RobotSpec`)에 모은다.
  등록된 로봇은 `panda`(기본값)와 `indy7`이다. Panda는 스폰/IK/카메라/GraspGen
  추론에 더해 **pregrasp → approach → close → lift → done 전 구간을 완주**한다.
  남은 것은 파지 자체다(폐쇄축 기준 14 mm 어긋남) — 아래 진행 상황 절을 볼 것.
- 루트 실행 파일은 `grasp_scene.py` 하나, 실행은 `scripts/*.py` 파이썬 런처로
  한다. 재사용 코드는 `source/` 밑 주제별 패키지(`robots/`=로봇 레지스트리·IK·
  그리퍼·로봇 자산, `sim/`=YCB·카메라·ros2, `control/`=grasp 실행,
  `graspgen/`=클라이언트·전처리·시각화 — mj_rl/mj_deploy와 같은 구조),
  각 패키지 설정은 자기 `config.py`, 씬 USD 자산은 `source/assets/`,
  로봇 USD 자산은 `source/robots/<name>/assets/` 아래에 둔다.
- Repo: https://github.com/CheolMin-Yoon/isaac_graspgen
- 업스트림 NVIDIA GraspGen 체크아웃(`/home/frlab/GraspGen`)은 이 워크스페이스와
  다른 저장소다. 이름이 비슷해졌으니 경로를 혼동하지 않는다 — 둘은 ZMQ로만
  통신하고, 서버 경로는 `source/graspgen/config.py`의 `SERVER_ROOT`에 있다.
- FFW(AI Worker BG2/SG2/SH5) USD들은 이 워크스페이스 소속이 아니다 —
  `~/gs_rl/source/assets/ffw/`로 옮겨졌다(Genesis 기반 워크스페이스, 2026-07-12).

## 완료된 기반

- **Indy7 + Robotiq 2F-140 결합**: binary 결합본의 혼합된 Z-axis/단일-mimic
  subtree를 비활성화하고 공식 UR10e 예제의 X-axis/5-mimic configured subtree를
  runtime에 조립한다. `tcp` 정렬과 fixed-joint frame은 공식 Robot Assembler
  계산을 쓰며, 최종 articulation은 self-collision OFF와 solver `16/1`을 강제한다.
  `finger_joint`는 `open=0`, `close=runtime upper limit=0.7 rad`다. open은 step
  0/60에서 `q=0`, close는 step 60에서 `q=0.7`임을 확인했다. 계약/자산 facts:
  `docs/design/indy7.md`.
- **로봇 레지스트리 분리(2026-08-15)**: 로봇 종속 사실을 `RobotSpec`
  (`source/robots/base.py`)으로 모으고, `sim/`·`control/`·`graspgen/`은 어떤
  팔인지 모르게 만들었다. `Indy7IK`→`PinkArmIK`(spec 주입), `Indy7Gripper`→
  `SingleJointGripper`(GripperSpec 주입), `spawn_indy7`→`robots.indy7.spawn`.
  GraspGen 체크포인트는 팔이 아니라 그리퍼에 붙는다(`ROBOTIQ_2F_140`).
  Isaac 없이 도는 pytest 8개가 통과하지만 **Isaac Sim 실행 재검증은 아직이다.**
- **Indy7 + YCB + IK 엔트리포인트**: 루트 `grasp_scene.py`가 로봇/YCB 스폰,
  선택적 목표 pose 추종(`--target-position`, `--target-orientation`),
  선택적 그리퍼 명령(`--gripper open|close|hold`)을 담당한다. 기본은 open이다.
  YCB object는 관찰 중 kinematic으로 고정되고, 선택된 target 하나만 approach 진입
  시 dynamic으로 해제된다. no-action 180-step에서 초기 pose가 유지됐다.
- **공식 PINK 경로 검증**: teleop 내부 USD→URDF exporter를 제거하고
  `isaacsim.robot_motion.pink` 공개 API + 검증된
  `source/robots/indy7/assets/indy7_kinematics.urdf`로 교체했다. q=0 URDF/USD
  TCP FK가 일치하며 `[0.45, 0.0, 0.45]`, quaternion `[0, 0, 1, 0]`에
  live PhysX TCP가 수렴했다.
- **GraspGen 실행 경계**: instance mask→GraspGen→geometry gate→PINK
  pregrasp/approach/close/lift target은 연결됐다. tomato can 첫 시험은 TCP
  trajectory는 완료했지만 물체를 옆으로 밀어 실제 lift는 실패했다.
- **Indy7 wrist camera**: `source/sim/camera.py`의 `WristCamera`가
  TCP 하위 `zed_cam` Camera prim을 만들고, RGB/depth를 `output/camera/`에
  저장한다. 현재는 별도 ZED USD asset을 참조하지 않고 Isaac Sim Camera prim을
  프로그램으로 생성한다.

## Panda GraspGen pick 진행 상황 (2026-08-15)

캔 하나(`005_tomato_soup_can`)를 Panda + GraspGen으로 집는 경로를 뚫는 중이다.
`--ycb-only`, `--ycb-radius`로 씬을 단일 물체로 좁혀서 실험한다.

동작 확인된 것:

- `graspgen_franka_panda` 체크포인트 3종 다운로드 완료, 서버는
  `./scripts/run_graspgen_server.py --gripper panda_hand`로 뜬다.
- 손목 카메라(pinhole)가 캔을 10k 픽셀 규모로 잡고, GraspGen이 conf 0.95,
  78 ms로 후보 20개를 반환한다. 기하 게이트도 통과한다.
- `--ycb-radius 0.55`에서 pregrasp 자세에 도달한다. 기본 0.70은 Indy7 리치
  기준 값이라 Panda에서는 pregrasp 자세부터 실패한다.

### 근본 원인 확인: PINK가 관절 한계 위반으로 침묵한다 (2026-08-15)

**찾았다.** 정체의 원인은 파지 기하도 접촉 물리도 아니었다.

```
PINK solve_ik failed: Joint 7 violates configuration limits 0.0 <= 0.04000313580036163 <= 0.04
```

`PANDA_HAND.open_position = 0.04`는 `panda_finger_joint`의 **상한과 정확히 같다**.
PhysX는 관절 한계를 3e-6 rad 정도 일상적으로 넘기는데, PINK는 계측된 configuration이
모델 한계 안에 있다고 **단언**하므로 `solve_ik`가 예외를 던진다. Isaac의
`PinkIKController.forward()`는 이를 `carb.log_warn`으로만 남기고 `None`을 반환하고,
우리 `go_to`는 `None`이면 아무 목표도 쓰지 않는다. 결과적으로 **팔에 명령이 전혀
나가지 않고 그 자리에 얼어붙는다.** Kit이 쏟아내는 수천 줄 경고에 묻혀 보이지 않았다.

측정 근거 (`scratchpad/ikdiag1.log`):

- lift 단계 241 스텝, `solve_ik` 실패 **241회** — 한 번도 명령이 나가지 않았다.
- `FK(commanded)`는 target에서 150.0 mm, `live TCP`는 `FK(commanded)`에서 0.7 mm.
  즉 **플랜트는 명령을 정확히 추종하고 있었고, 명령 자체가 갱신되지 않았다.**
  `LIFT_HEIGHT=0.15`와 150.0 mm가 일치한다 — 팔이 lift에서 한 톨도 안 움직였다.
- 관절 한계 여유: 팔 7축 모두 0.09 rad 이상, `panda_finger_joint1/2`만 **여유 0.0000**.

**이것이 일곱 번의 반증을 전부 설명한다.** 그리퍼 폭·마찰·솔버 반복·스폰 반경을
바꿔도 오차가 23.9 / 26.1 / 27.7 / 30.0 mm로 "상수처럼" 남았던 이유는, 실패가 씬의
어떤 물리량과도 무관하게 **손가락이 상한을 부동소수점 오차만큼 넘는 순간 IK 전체가
멈추는** 데 있었기 때문이다. 멈춘 위치가 매번 조금씩 달랐던 것은 침묵이 시작된
시점이 달랐던 것뿐이다. `open_position`을 0.035에서 0.04로 올린 것은 효과가 없던
게 아니라 **상황을 확정적으로 악화시켰다** — 여유 5 mm를 0으로 만들었다.

적용한 수정 (`source/robots/arm_ik.py`):

1. PINK에 넘기기 전에 계측 관절값을 모델 한계 안으로 클램프한다(여유 1e-4).
   플랜트 계측이 URDF 한계를 epsilon 벗어나는 것은 정상이며, 그것을 하드 제약
   QP에 그대로 넣은 배선이 버그였다.
2. `desired_state`가 `None`이면 연속 실패를 세어 `[ik] solver returned no command
   ... the arm is NOT being driven`를 찍는다. 명령을 멈춘 컨트롤러가 조용한 상황을
   다시 만들지 않는다.

부수 관측: 실패 시점에 손가락이 0.0400(완전 개방)이었다 — close 단계를 90스텝
지났는데도. 위상 전이마다 `target / measured / effort`를 찍게 해서 확인한 결과,
**331 스텝 동안 닫으라고 명령했는데 손가락이 0.0400에서 전혀 움직이지 않았다**
(effort 5.6). 원인은 같은 배선이었다: `PinkArmIK`가 `set_dof_position_targets`에
**9개 DOF 전부**를 썼다. Isaac의 번들 Franka 모델은 손가락 관절을 포함하므로 QP가
그것까지 풀고, posture task가 컨트롤러 reset 시점 값으로 끌어당긴다. 팔 컨트롤러와
그리퍼 컨트롤러가 매 스텝 같은 DOF를 놓고 싸웠다. IK 쓰기를 팔 DOF로 제한하자
그리퍼가 닫힌다. 손가락은 tool frame 바깥에 있어서 팔 추종에는 손실이 없다.

### 여기까지의 결과: 전 구간 완주

세 수정(클램프 · 팔 전용 쓰기 · orientation_cost) 후 위상 기계가 완주한다:

```
pregrasp -> approach -> close -> lift -> done     (solve_ik 실패 0회)
```

`orientation_cost`를 0.05에서 1.0으로 올린 것이 lift를 통과시켰다. 측정된 실패는
`position=2.1mm(통과) / orientation=0.220rad(초과, tol 0.150)`이었다 — QP가 위치를
100배 무겁게 잡고 있어서 **게이트가 재는 바로 그 양을 솔버가 버리고 있었다**.
공식 예제(`FrankaPinkIKExample`)의 5.0/0.05를 그대로 쓴 것이 화근인데, 그 예제는
목표를 텔레오퍼레이션할 뿐 도착 여부를 묻는 수렴 판정이 아예 없어서 이 대가를
치르지 않는다. 5:1로 낮추자 lift가 done까지 간다.

### 남은 문제: 파지가 캔을 비껴간다

캔은 여전히 테이블 위다(`bottom_z=-0.0000`). 규약에 의존하지 않는 계측을 넣었다 —
두 손가락 링크의 world pose를 직접 읽어서 **둘을 잇는 선을 폐쇄축으로 정의**한다
(`report_finger_straddle`). URDF가 그 축을 뭐라 부르든 상관없다.

```
straddle: lateral miss=-14.1mm, finger span=80.0mm,
          object depth from hand=147.3mm (fingertip reach 103.4mm)
```

캔 반경 33 mm, 손가락 반폭 40 mm — 편측 여유가 7 mm뿐인데 폐쇄축 기준으로
**14 mm 어긋나 있다**. 한쪽 손가락은 캔에 박히고 반대쪽은 헛돈다. 실제로 close에서
`measured=0.0356 effort=3.66`으로 뭔가에 걸리지만 캔을 물지는 못한다.

이 14 mm의 출처를 가르는 중이다. 위상 전이마다 추종 오차를 찍게 했다 — approach가
2 mm로 추종했는데 14 mm가 어긋났다면 GraspGen 자세 쪽이고, 10 mm로 추종했다면
우리 `POSITION_TOL=0.01`이 캔의 7 mm 여유보다 헐거운 것이 절반을 설명한다.

미확인 관측 하나: `panda_finger_joint2`의 drive가 `stiffness=0, damping=0,
max_effort=0`이다(joint1은 400/80/7.2). mimic joint라 drive가 없는 것인지 자산이
authoring을 빠뜨린 것인지 확인하지 않았다. 두 손가락이 항상 함께 움직이는 것으로
보아 mimic 쪽이 유력하나, `ParallelFingerGripper`는 둘 다 직접 명령하고 있으므로
mimic이 맞다면 그 전제부터 틀린 것이다.

### 이전 조사 기록 (위 원인으로 대체됨)

막혀 있던 지점: **approach 단계**. 당시 측정은

```
best position error   = 26.1mm (tol 10mm)   실패
best orientation error= 0.122rad (tol 0.150) 통과
캔 회전 = 19.98도
```

자세는 맞고 위치가 26 mm 부족한데, 캔이 20도 돌아간 것은 손가락이 캔에 닿고
있다는 뜻이다. 즉 팔이 못 뻗는 게 아니라 **물체에 막힌 것**이다. 캔 지름
66 mm에 그리퍼 개구가 70 mm(`open_position=0.035`)라 편측 여유 2 mm뿐이었고,
이는 approach의 자세 허용 오차(7도)보다 작다. `open_position`을 관절 한계인
0.04(개구 80 mm)로 올렸으나 **효과가 없었다** — 오차가 26.1 mm에서 30.0 mm로
사실상 그대로였다. 그리퍼 개구 폭은 원인이 아니다.

IsaacLab 실측 대조로 반증된 가설:

- **"Panda를 바닥에 놓은 배치가 문제"** — 아니다. IsaacLab도 Franka 베이스가
  Z=0이고 물체는 Z=0.0203이다(`stack_env_cfg.py:46-57`, `stack_joint_pos_env_cfg.py:118`).
  테이블 상판이 Z=0이고 ground plane을 Z=-1.05로 내려서 로봇이 바닥 충돌
  지오메트리를 보지 않게 할 뿐, 물체는 베이스와 같은 평면에 있다. 우리 Z 배치는
  틀리지 않았다. 다만 우리는 로봇 바로 아래에 실제 ground plane 충돌체가 있다.

`frictionCorrelationDistance` 0.00625과 YCB solver 16/1을 적용했으나 **효과가
없었다** — 27.7 mm로 여전하다. 접촉 물리 파라미터는 원인이 아니다.

**TCP 프레임 가설은 GraspGen 소스 확인으로 약해졌다.** 확인한 사실:

- `grasp_gen/models/action_decoder.py:36` — `grasp_translation = contact_pt -
  gripper_depth * approach_dir`. 즉 반환 pose는 **gripper base 프레임**이고 tool
  center는 `base + depth*approach`다. 우리 선택 게이트와 일치한다.
- `graspgen_franka_panda.yml`의 `gripper_depth: 0.1034` — IsaacLab의
  hand→fingertip midpoint와 같은 값.
- `grasp_gen/robot.py:385-392` — `transform_from_base_link_to_tool_tcp`는 gripper
  모듈이 오버라이드하지 않으면 **순수 z 평행이동 `[0,0,depth]`**다.

따라서 `TCP_OFFSET=0`으로 `panda_hand`를 구동하는 것은 GraspGen의 base link가
`panda_hand`인 한 맞다. 남은 확인거리는 그 base link가 `panda_hand`인지
`panda_link8`인지다 — 후자라면 둘은 z축 45도 회전만 차이나므로 **27 mm 위치
오차는 설명하지 못한다**(자세 오차도 0.122 rad로 45도와 맞지 않는다).

즉 상수 오차의 출처는 아직 미상이다. 다음에 볼 것: 실행 중 실제 `panda_hand`
world pose와 명령된 target을 나란히 찍어 **어느 축으로 27 mm가 벌어지는지** 본다.
접근축 방향이면 depth 계열, 수직이면 다른 원인이다. 이전 가설: 오차가 그리퍼 폭·마찰·반경을 바꿔도
23.9 / 26.1 / 27.7 / 30.0 mm로 일정하게 유지되는 것은 접촉 현상이 아니라 **상수
오프셋의 서명**이다. 우리는 `ee_link_name="panda_hand"`로 panda_hand를 직접 구동하고
`TCP_OFFSET=0`이다. 반면 IsaacLab은 IK가 panda_hand +0.107 프레임을 구동하고 파지
판정은 +0.1034를 쓴다. GraspGen의 Franka 규약이 어느 프레임 기준인지 확인해야 한다
— `/home/frlab/GraspGenModels/checkpoints/graspgen_franka_panda.yml`과 GraspGen
저장소의 gripper 정의를 볼 것. 목표가 물체 안쪽으로 파고들면 팔이 캔에 막혀
정확히 지금 같은 정체가 생긴다.

그 다음 후보:

- **Franka USD가 다르다**: IsaacLab은 `{ISAACLAB_NUCLEUS_DIR}/Robots/FrankaEmika/
  panda_instanceable.usd`를 쓴다(`franka.py:28`). 우리가 쓰는
  `/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd`는 `Gripper`/`Mesh`
  variant set이 있고, 선택하지 않으면 기본 variant의 손가락 충돌 메시가 적용된다
  (`franka_pick_up.py:57-58`은 `AlternateFinger`/`Quality`를 명시 선택한다).
- **High-PD 변형**: IK 태스크는 `FRANKA_PANDA_HIGH_PD_CFG`(`disable_gravity=True`,
  어깨/전완 stiffness 400 / damping 80)를 쓴다. IK 추종 정확도에 직접 영향.
- **TCP 프레임 불일치**: IsaacLab 내부에서도 IK가 구동하는 프레임(panda_hand +0.107)과
  파지 판정 프레임(+0.1034)이 3.6 mm 다르다. 우리는 `graspgen_depth=0.1034`,
  `TCP_OFFSET=0`으로 panda_hand를 직접 구동한다. 상수 standoff의 원인이 될 수 있다.

크래시 A/B (실행 간격 10초 고정):

- `--graspgen` 없이 120스텝: **6/6 성공**
- `--graspgen` 포함 4000스텝: **3/4 크래시**

간격은 요인이 아니다. GraspGen 서버(GPU 상주 약 844 MB)와의 경합이 유력하나
서버를 분리한 대조 실험은 아직 못 했다.

## Isaac Sim 간헐적 크래시 (미해결, 최대 병목)

시작 직후 SIGSEGV(exit 139)로 죽는다. 실험 자체를 막고 있으므로 다른 무엇보다
먼저 잡아야 한다. **크래시를 수렴 실패로 오독하지 않도록 로그의
`A crash has occurred` 유무를 항상 먼저 확인할 것.**

지금까지 세운 가설과 그 결과 — 셋 다 틀렸거나 불충분했다:

- "카메라 aperture를 살아있는 상태에서 바꾸는 게 원인" — 그 패턴이 크래시 직전
  마지막 동작인 건 맞고 prim에 직접 authoring하도록 고쳤지만, 이후에도 재발했다.
- "단독 실행이면 안 죽는다" — 단독에서도 죽었다.
- "실행 간 8초 간격이면 안 죽는다" — 6/6 통과했으나 그 테스트는 `--graspgen`
  **없는** 120스텝 실행이었다. 실패하는 구성을 검증하지 않은 일반화였다.

다음에 볼 것: 크래시한 실행이 거의 전부 `--graspgen`이었으므로 **GraspGen
서버(GPU 상주 약 844 MB)와의 경합**이 유력하다. 서버 유무로 동일 씬을 반복
실행해 크래시율을 비교하면 갈린다.

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
- 씬에 조명을 authoring하지 않던 문제를 고쳤다(`add_dome_light`). 조명이 없으면
  RGB뿐 아니라 **depth annotator도 유효 픽셀을 반환하지 않아** "카메라가 허공을
  본다"와 증상이 완전히 같다. Indy7 자산이 조명을 품고 있어 로봇이 하나일 때는
  드러나지 않았다.
- IsaacLab에서 값을 가져올 때 세 번 연속 같은 실수를 했다. 값은 맞아도 맥락이
  같이 오지 않는다: PINK 게인은 예제에 수렴 판정 자체가 없었고, 카메라 오프셋
  쿼터니언은 IsaacLab의 ROS 규약이라 USD local pose에 직접 쓰면 하늘을 보며,
  렌즈 파라미터는 spawn 시점에 적용해야 하는 값이었다. **"공식 예제 값"은
  적용 시점과 좌표 규약까지 확인하고 쓸 것.**
- 카메라 mount pose는 Indy7 TCP 기준 초기값이다. 실제 ZED 형상/시야와 맞추려면
  mount offset, orientation, intrinsics를 조정해야 한다.

## 먼저 읽을 정본

- 실행 명령/옵션: `README.md`
- indy7 USD/articulation 계약: `docs/design/indy7.md`
- indy7 PINK IK 계약: `docs/design/indy7-ik.md`
- 팔 추가 절차와 아직 Indy7 모양인 부분: `source/robots/panda/README.md`
- Isaac Sim 툴링 관련 재발 방지 노트:
  research-wiki `AI-Sessions/wiki/harness/errors/isaacsim-errors.md`
