# Isaac Sim과 GraspGen 런타임 연결

이 워크스페이스는 서로 의존성이 다른 두 Python 런타임을 한 프로세스에 섞지
않는다. Isaac Sim이 wrist-camera point cloud를 만들고, 별도 GraspGen 서버가
추론한 grasp pose를 localhost ZMQ로 돌려준다.

정확히는 **두 Conda 환경이 아니다.** GraspGen만 Conda 환경이고 Isaac 쪽은
standalone Isaac Sim에 포함된 Python이다.

| 역할 | 실행기 | 현재 Python | 포함하는 무거운 의존성 |
|---|---|---:|---|
| 시뮬레이터·ZMQ client | `/home/frlab/isaacsim/python.sh` | 3.12.13 | Isaac Sim 6.0.1, Kit, PhysX |
| Grasp 추론 ZMQ server | `/home/frlab/anaconda3/envs/graspgen/bin/python` | 3.11.15 | PyTorch 2.7.0+cu128, GraspGen |

```text
Isaac Sim bundle Python                       graspgen Conda Python
scripts/run_scene.py                          scripts/run_graspgen_server.py
        │                                                 │
        │ point_cloud float32[N,3]                        │
        └──────────── ZMQ REQ/REP + msgpack ─────────────►│
        │◄──────── grasps[M,4,4] + confidences[M] ────────┘
        │
        └─ 기본 endpoint: tcp://127.0.0.1:5556
```

두 런타임 사이에 `PYTHONPATH`, PyTorch, CUDA library 또는 Isaac 모듈을 공유하지
않는다. 프로세스 사이를 건너가는 것은 msgpack으로 직렬화한 NumPy 배열뿐이다.

## 설정의 정본

| 설정 | 파일 | 현재 값 또는 역할 |
|---|---|---|
| server host·port·timeout | `source/graspgen/config.py` | `127.0.0.1:5556`, 60초 |
| GraspGen checkout | `source/graspgen/config.py` | `/home/frlab/GraspGen` |
| server Python | `source/graspgen/config.py` | `.../envs/graspgen/bin/python` |
| gripper별 YAML·checkpoint | `source/robots/gripper.py` | Robotiq/Panda model 경로 |
| Isaac launcher | `scripts/run_scene.py` | `~/isaacsim/python.sh`로 실행 |
| server launcher | `scripts/run_graspgen_server.py` | 지정된 Conda Python으로 실행 |

실행 스크립트가 절대 경로의 Python으로 자신을 교체하므로 보통 `conda activate`는
필요 없다. 다만 GraspGen 환경에서 활성화된 `PYTHONPATH`나 `LD_LIBRARY_PATH`가
Isaac 프로세스로 상속되지 않도록 두 프로그램은 각각 새 터미널에서 실행한다.

## 최초 한 번 준비

경로와 server 환경을 확인한다.

```bash
test -x /home/frlab/isaacsim/python.sh
test -x /home/frlab/anaconda3/envs/graspgen/bin/python
test -f /home/frlab/GraspGen/client-server/graspgen_server.py

cd /home/frlab/GraspGen
/home/frlab/anaconda3/envs/graspgen/bin/python -c \
  "import torch, zmq, msgpack; print(torch.__version__, torch.version.cuda)"
```

Isaac Sim bundle Python에는 server의 PyTorch stack이 아니라 가벼운 client package
세 개만 설치한다.

```bash
cd /home/frlab/isaac_graspgen
./scripts/install_graspgen_client_deps.py

/home/frlab/isaacsim/python.sh -c \
  "import zmq, msgpack, msgpack_numpy; print('GraspGen client deps: OK')"
```

`ModuleNotFoundError: No module named 'zmq'`가 나오면 위 설치 스크립트를 다시
실행한다. Isaac Sim 설치 디렉터리를 교체하면 bundle Python의 package도 함께
사라질 수 있다.

## 실행

server checkpoint의 gripper와 scene robot의 실제 gripper가 일치해야 한다.

| scene | server 명령의 `--gripper` |
|---|---|
| `--robot panda` | `panda_hand` |
| `--robot indy7` | `robotiq_2f_140` |

주의: server의 `--gripper panda_hand`는 **모델 종류**를 고른다. scene의
`--gripper open|close|hold`는 **초기 관절 명령**이라 이름만 같고 의미가 다르다.

### Panda

터미널 1에서 Panda checkpoint server를 띄운다.

```bash
cd /home/frlab/isaac_graspgen
./scripts/run_graspgen_server.py --gripper panda_hand
```

터미널 2에서 기본 Panda scene을 연결한다.

```bash
cd /home/frlab/isaac_graspgen
./scripts/run_scene.py \
  --robot panda \
  --graspgen \
  --grasp-object-index 0 \
  --graspgen-step 180
```

### Indy7 + Robotiq 2F-140

터미널 1:

```bash
cd /home/frlab/isaac_graspgen
./scripts/run_graspgen_server.py --gripper robotiq_2f_140
```

터미널 2:

```bash
cd /home/frlab/isaac_graspgen
./scripts/run_scene.py \
  --robot indy7 \
  --graspgen \
  --grasp-object-index 0 \
  --graspgen-step 180
```

## 연결 확인

server가 listen 중인지 확인한다.

```bash
ss -ltnp | rg ':5556\b'
```

Isaac bundle Python에서 protocol health check만 실행할 수도 있다.

```bash
cd /home/frlab/isaac_graspgen
PYTHONPATH="$PWD/source" /home/frlab/isaacsim/python.sh -c \
  "from graspgen.client import GraspGenClient; c=GraspGenClient(); print('health:', c.health_check()); print('metadata:', c.metadata()); c.close()"
```

정상 연결이면 `health: True`와 server의 `gripper_name`이 출력된다. 이 이름이 실행할
robot의 gripper와 일치하는지 확인한다. 현재 scene client는 metadata를 출력하지만
불일치를 자동으로 거부하지는 않는다.

최소 end-to-end 검사는 후보 수를 줄여 실행한다.

```bash
./scripts/run_scene.py \
  --headless \
  --no-ros2 \
  --robot panda \
  --ycb-only 005_tomato_soup_can \
  --graspgen \
  --graspgen-step 180 \
  --max-steps 200 \
  --grasp-num-grasps 20 \
  --grasp-topk 5
```

성공 기준은 다음 세 가지다.

1. 시작 로그에 `[graspgen] connected:`가 나온다.
2. inference 로그에 반환된 grasp 수와 `infer_ms`가 나온다.
3. `output/graspgen/input_world.npy`와 `best_grasp_world.npy`가 생성된다.

## 다른 port 또는 host 사용

같은 PC에서 port만 바꾸려면 server와 client 양쪽 값을 함께 바꾼다.

```bash
./scripts/run_graspgen_server.py --gripper panda_hand --port 5557
./scripts/run_scene.py --robot panda --graspgen --graspgen-port 5557
```

다른 PC의 server에 연결할 때는 server가 외부 interface에 bind하도록 `--host`를
지정하고, scene에는 server 주소를 넘긴다.

```bash
# server PC
./scripts/run_graspgen_server.py --gripper panda_hand --host 0.0.0.0 --port 5556

# Isaac PC
./scripts/run_scene.py \
  --robot panda \
  --graspgen \
  --graspgen-host SERVER_IP \
  --graspgen-port 5556
```

외부 bind는 인증이나 암호화를 추가하지 않는다. 신뢰할 수 있는 사설망에서만 쓰고
방화벽은 필요한 client 주소에만 연다.

## 오류를 구분하는 기준

| 증상 | 확인할 위치 |
|---|---|
| Isaac에서 `No module named 'zmq'` | `install_graspgen_client_deps.py` 재실행 |
| `server is not ready at ...` | server 프로세스, host/port, `ss` 결과 |
| launcher가 `Missing GraspGen file`로 종료 | `source/robots/gripper.py`의 외부 model 경로 |
| 연결됐지만 grasp 형상이 맞지 않음 | server metadata와 robot/gripper 조합 |
| Isaac 시작 중 shared-library 충돌 | GraspGen Conda가 활성화되지 않은 새 터미널에서 scene 재실행 |
| 요청이 60초 뒤 실패 | server traceback/OOM 확인 후 client timeout 조정 |

순수 bridge test는 Isaac을 시작하지 않고 현재 GraspGen Conda 환경으로 실행한다.

```bash
cd /home/frlab/isaac_graspgen
/home/frlab/anaconda3/envs/graspgen/bin/python -m pytest tests/test_graspgen_bridge.py -q
```
