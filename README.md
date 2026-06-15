# MacGyvBot Simulation

MacGyvBot simulation 저장소는 저장소 루트의 `macgvbot.usd` 장면을 Isaac Sim에서
열고 RGB 프레임 또는 mp4 영상으로 캡처하기 위한 파일을 담고 있습니다.

현재 저장소는 ROS 2 runtime workspace가 아니라 Isaac Sim 장면 캡처용 저장소입니다.

## 파일 구조

```text
.
├── macgvbot.usd
├── scripts/isaacsim/
│   ├── render_macgvbot_usd.py
│   └── run_render_macgvbot_usd.sh
├── docs/simulation/
│   └── isaacsim_macgvbot_usd.md
├── CONTRIBUTING.md
└── README.md
```

## 실행

```bash
scripts/isaacsim/run_render_macgvbot_usd.sh \
  --tool screwdriver \
  --drawer-id 1 \
  --drawer-prim /drawer/drawer_floor_02 \
  --drawer-control physics 
  --no-headless \
  --frames 180 \
  --no-video
```

## 기여

브랜치, 커밋, PR, 이슈, 리뷰 규칙은 [CONTRIBUTING.md](./CONTRIBUTING.md)를
따릅니다.

## 파일 역할

- `macgvbot.usd`
  - Isaac Sim에서 여는 MacGyvBot simulation 장면입니다.
  - USD crate 형식의 바이너리 USD 파일입니다.

- `.gitignore`
  - Python cache, ROS/colcon 산출물, 대용량 모델/영상 파일, Isaac Sim 캡처 결과를
    Git 추적에서 제외합니다.

- `scripts/isaacsim/run_render_macgvbot_usd.sh`
  - 저장소 루트로 이동한 뒤 `render_macgvbot_usd.py`를 Isaac Sim `python.sh`로
    실행하는 wrapper입니다.
  - `ISAAC_SIM_PYTHON`이 설정되어 있으면 그 경로를 사용합니다.
  - 설정되어 있지 않으면 몇 가지 알려진 Isaac Sim 설치 경로에서 실행 가능한
    `python.sh`를 찾습니다.

- `scripts/isaacsim/render_macgvbot_usd.py`
  - `macgvbot.usd`를 열고 캡처 카메라와 보조 조명을 추가합니다.
  - Replicator `BasicWriter`로 RGB PNG 프레임을 저장합니다.
  - `--video`가 켜져 있으면 `ffmpeg`로 mp4를 생성합니다.
  - `--simulate-twin`이 켜져 있으면 스크립트 내부 joint waypoint를 사용해 로봇과
    그리퍼 target을 갱신합니다.
  - 선택한 서랍의 기존 rigid body와 prismatic joint를 사용하며, 그리퍼와 손잡이의
    collider 접촉력으로 서랍을 움직입니다.

- `docs/simulation/isaacsim_macgvbot_usd.md`
  - Isaac Sim 실행 준비, 기본 캡처 명령, 출력 위치, 자주 쓰는 옵션을 설명합니다.
  - 서랍 prim path를 찾고 `--drawer-prim`, `--drawer-axis`,
    `--drawer-distance`를 사용하는 방법을 설명합니다.

기본 캡처 결과는 아래에 생성됩니다.

```text
outputs/isaacsim/macgvbot_usd_capture/
```

대표 출력 파일:

```text
outputs/isaacsim/macgvbot_usd_capture/macgvbot_usd_capture.mp4
outputs/isaacsim/macgvbot_usd_capture/rgb_*.png
```

`outputs/isaacsim/`은 로컬 생성물로 보고 Git에 포함하지 않습니다.
