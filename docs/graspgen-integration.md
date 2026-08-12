# GraspGen 연결 작업 기록

기준일: 2026-08-12

## 목표와 경계

이 작업의 목표는 `isaac_indy7`만 수정하여 기존 GraspGen을 inference backend로
연결하는 것이다.

- 수정하는 저장소: `/home/frlab/isaac_indy7`
- 수정하지 않는 저장소: `/home/frlab/GraspGen`, `/home/frlab/better_pcd`,
  `/home/frlab/IsaacLab`
- GraspGen은 별도 Conda/CUDA 환경에서 ZMQ server로 실행한다.
- Isaac Sim은 가벼운 ZMQ client만 포함하며 GraspGen의 PyTorch 코드를 import하지
  않는다.
- 기본 경로는 `카메라 point cloud → GraspGen → grasp 시각화`다.
  `--execute-grasp`를 명시하면 geometry gate를 통과한 후보를
  pregrasp→approach→close→lift target 순서로 실행한다. 아직 실제 물체의
  lift 성공까지 검증된 상태는 아니다.

```text
Isaac Sim 6.0.1 / isaac_indy7
  wrist depth camera
        ↓ world point cloud
  target instance mask + aligned depth deprojection
        ↓ 2048 × XYZ, meter/world frame
  local ZMQ client
        │
        └──────────────► GraspGen server
                          diffusion + discriminator
        ◄─────────────── grasps[M,4,4] + confidence[M]
        ↓
  geometry gate → viewport/NPY
        ↓ opt-in
  official PINK IK → pregrasp/approach/close/lift target
```

## 구현된 파일

### `source/graspgen/client.py`

기존 `/home/frlab/GraspGen/grasp_gen/serving/zmq_server.py`의 msgpack/ZMQ
protocol을 독립적으로 구현한 client다.

- `health`, `metadata`, `infer` action 지원
- 입력 validation: finite `float32[N,3]`
- 출력: `GraspGenResult(grasps, confidences, infer_ms)`
- timeout 후 REQ socket을 폐기해 다음 요청에서 잘못된 REQ/REP state가 재사용되지
  않게 처리
- GraspGen Python package를 Isaac Sim process에서 import하지 않음

### `source/graspgen/pointcloud.py`, `source/graspgen/selection.py`

- NaN/Inf 제거와 `float32 XYZ` 변환
- point-cloud finite/range 검사
- 점이 부족할 때만 replacement를 사용하는 고정 점수 sampling
- seed 기반 deterministic sampling
- tool point 거리, table clearance, approach 방향, outward approach를 사용한
  실행 후보 geometry gate

### `source/graspgen/visualization.py`

- GraspGen confidence 순으로 grasp pose 정렬
- 상위 grasp를 RGB(X/Y/Z) 축으로 viewport에 표시
- GraspGen에 실제 전송한 cloud를 주황색 point로 표시

### `source/sim/camera.py`

기존 `WristCamera`에 opt-in point-cloud capture를 추가했다.

- `enable_pointcloud=True`일 때 depth/instance annotator 연결
- color camera의 aligned depth를 intrinsics로 deproject한 뒤 world frame으로 변환
- `get_object_pointcloud(instance_label, world_frame=True)`로 target semantic instance만 선택
- 기본 실행은 기존과 동일하게 point-cloud annotator를 사용하지 않음

### `source/sim/ycb.py`

현재 pose에서 선택한 YCB prim의 world-aligned render bounds를 구하는
`get_world_bounds()`를 추가했다. 모든 object는 support-aligned pose에서
kinematic으로 스폰되며, 실행 대상으로 선택된 object 하나만 approach phase
진입 시 dynamic으로 해제된다. close까지 kinematic을 유지하면 finger overlap
해소 impulse로 물체가 옆으로 밀릴 수 있어 release 시점을 앞당겼다.

### `indy7.py`

`--graspgen` 실행 경로를 추가했다.

1. GraspGen health/metadata 확인
2. 지정 step에서 wrist-camera world point cloud 획득
3. `--grasp-object-index`에 해당하는 semantic instance mask로 object cloud 선택
4. `--grasp-point-count`만큼 sampling
5. GraspGen inference 1회 요청
6. 상위 grasp를 viewport에 표시
7. 입력 cloud와 최고 grasp pose를 `output/graspgen/`에 저장
8. `--execute-grasp`일 때 geometry gate 후보를 official Isaac PINK로 실행

주요 옵션:

| 옵션 | 기본값 | 의미 |
|---|---:|---|
| `--graspgen` | off | GraspGen 연결 활성화 |
| `--graspgen-host` | `127.0.0.1` | ZMQ server host |
| `--graspgen-port` | `5556` | ZMQ server port |
| `--graspgen-step` | `180` | inference를 1회 실행할 simulation step |
| `--grasp-object-index` | `0` | `ycb_paths`의 target index |
| `--grasp-crop-padding` | `0.01` | world AABB padding, meter |
| `--grasp-point-count` | `2048` | GraspGen 입력 점수 |
| `--grasp-num-grasps` | `200` | diffusion candidate 수 |
| `--grasp-topk` | `20` | 반환·표시할 상위 grasp 수 |
| `--execute-grasp` | off | geometry gate 후 pregrasp→approach→close→lift target 실행 |

### 실행 및 테스트 스크립트

- `scripts/install_graspgen_client_deps.py`: Isaac Sim Python에 client dependency 설치
- `scripts/run_graspgen_server.py`: 기존 GraspGen 환경/checkpoint로 localhost server 실행
- `tests/test_graspgen_bridge.py`: point-cloud 전처리, grasp 축, mock ZMQ protocol 테스트

## 환경 변경

Isaac Sim 번들 Python에 다음 client 전용 package를 설치했다.

```text
pyzmq==27.1.0
msgpack==1.2.1
msgpack-numpy==0.4.8
```

재설치는 다음 명령으로 가능하다.

```bash
cd /home/frlab/isaac_indy7
./scripts/install_graspgen_client_deps.py
```

GraspGen server는 다음 환경과 checkpoint를 사용한다.

```text
Python: /home/frlab/anaconda3/envs/graspgen/bin/python
Source: /home/frlab/GraspGen
Config: /home/frlab/GraspGenModels/checkpoints/graspgen_robotiq_2f_140.yml
Generator: graspgen_robotiq_2f_140_gen.pth
Discriminator: graspgen_robotiq_2f_140_dis.pth
```

## 실행 방법

터미널 1:

```bash
cd /home/frlab/isaac_indy7
./scripts/run_graspgen_server.py
```

터미널 2:

```bash
cd /home/frlab/isaac_indy7
./scripts/run_indy7.py \
  --graspgen \
  --target-position 0.45 0.0 0.45 \
  --grasp-object-index 0 \
  --graspgen-step 120
```

`--target-position`은 현재 wrist camera가 물체를 관측하게 하기 위한 임시 동작이다.
해당 IK target과 orientation은 최종 view-planning 값으로 확정된 것이 아니다.

## 검증 결과

### 단위 및 protocol 테스트

```text
tests/test_graspgen_bridge.py: 6 passed
Isaac Sim Python module import: 성공
py_compile: 성공
git diff --check: 성공
```

### 실제 GraspGen server + 합성 cloud

2048점 box surface cloud를 보내 실제 generator/discriminator를 실행했다.

```text
health: True
model: diffusion-discriminator
gripper: robotiq_2f_140
requested candidates: 20
returned top-k: 5
ZMQ 전체 왕복: 약 332.4 ms
server inference: 약 331.9 ms
best confidence: 약 0.9411
```

같은 PC localhost에서 ZMQ serialization/transport 자체는 전체 시간 중 매우 작고,
대부분이 GraspGen inference 시간임을 확인했다.

### Isaac Sim wrist camera + 실제 GraspGen server

다음 smoke command로 end-to-end 요청까지 성공했다.

```bash
./scripts/run_indy7.py \
  --headless \
  --target-position 0.45 0.0 0.45 \
  --graspgen \
  --graspgen-step 120 \
  --max-steps 130 \
  --grasp-num-grasps 20 \
  --grasp-topk 5
```

결과:

```text
camera world points: 307200
AABB cropped points: 4225
points sent: 2048
returned grasps: 5
server inference: 약 36.5 ms (warm server, 20 candidates)
best confidence: 약 0.9402
```

생성된 진단 파일:

```text
output/graspgen/input_world.npy
output/graspgen/best_grasp_world.npy
```

## 현재 검증 경계

2026-08-12 tomato-soup can(index 2) 실제 headless run에서 다음을 확인했다.

- official PINK: 관찰 pose와 grasp phase target을 모두 추종
- instance mask: target 2,296점 선택, 2,048점 전송
- GraspGen: 100개 반환, server inference 약 55.8 ms
- executor: pregrasp→approach→close→lift target→done 진행
- 실제 물체: center z가 `0.0329 m → 0.0335 m`로 증가하지 않고 옆으로
  밀려남. **lift 실패**

이 실패 run은 선택 target을 close phase에서 dynamic으로 해제하던 버전이다.
이후 모든 YCB를 관찰 중 kinematic으로 고정하고 target을 approach phase에서
해제하도록 교정했다. no-action 180-step에서는 11개 object pose가 유지됐지만,
교정 뒤 실제 grasp/lift는 아직 재실행하지 않았다.

따라서 현재 `done`은 TCP trajectory 완료이지 pick 성공을 의미하지 않는다.
다음 항목은 아직 미해결이다.

1. 그리퍼 collision/closure 후보 검사와 contact-based grasp 성공 판정
2. object center z 증가를 사용한 lift success/failure gate
3. 물체를 옆으로 밀지 않는 접근 경로와 grasp depth 튜닝
4. ShapeNet OBJ/USD와 PoinTr-Evidential completion 연결

## 다음 작업 순서

1. close 직전/직후 물체 pose와 gripper contact를 기록하고 pick 성공 gate를 만든다.
2. tomato can을 기준으로 grasp depth, approach, finger close target을 한 변수씩
   검증한다.
3. 실제 lift 성공 후 `better_pcd`를 수정하지 않는 별도 PoinTr-EDL
   service/client bridge를
   `isaac_indy7`에 추가한다.
4. 최종적으로 partial → completion/uncertainty → GraspGen → uncertainty-aware
   selection → pick/place state machine 순서로 확장한다.
