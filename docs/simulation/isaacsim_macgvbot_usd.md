# Isaac Sim `macgvbot.usd` Capture

이 문서는 저장소 루트의 `macgvbot.usd` 장면을 Isaac Sim에서 열고 RGB 프레임과
mp4 영상을 만드는 방법을 설명합니다. `macgvbot.usd`에는 로봇과 서랍이 이미 들어
있다고 가정하며, 캡처 스크립트는 ROS 2 런타임 노드와 독립적으로 동작합니다.

## 준비

- NVIDIA Isaac Sim 4.5 이상
- Isaac Sim이 실행 가능한 NVIDIA GPU 환경
- mp4 생성을 위한 `ffmpeg`

```bash
sudo apt update
sudo apt install -y ffmpeg
```

`ffmpeg`가 없어도 `--no-video`를 주면 PNG 프레임 캡처만 수행할 수 있습니다.

## 기본 실행

Isaac Sim 설치 디렉터리의 `python.sh`로 실행합니다. 시스템 `python3`로는
Isaac Sim 모듈을 import할 수 없습니다.

```bash
cd ~/macgyvbot-simulation
scripts/isaacsim/run_render_macgvbot_usd.sh
```

현재 이 컴퓨터에서 확인된 Isaac Sim Python 경로는 아래입니다.

```text
/home/ssu/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh
```

wrapper는 이 경로를 자동으로 사용합니다. 다른 Isaac Sim을 쓰고 싶으면
`ISAAC_SIM_PYTHON`으로 직접 지정할 수 있습니다.

```bash
ISAAC_SIM_PYTHON=/path/to/isaacsim/python.sh scripts/isaacsim/run_render_macgvbot_usd.sh
```

결과는 기본적으로 아래에 저장됩니다.

```text
outputs/isaacsim/macgvbot_usd_capture/
```

생성되는 대표 파일:

```text
outputs/isaacsim/macgvbot_usd_capture/macgvbot_usd_capture.mp4
outputs/isaacsim/macgvbot_usd_capture/rgb_*.png
```

## 자주 쓰는 옵션

고정 카메라로 촬영:

```bash
scripts/isaacsim/run_render_macgvbot_usd.sh \
  --camera-mode fixed \
  --fixed-camera-position 2.6 -2.4 1.8 \
  --camera-target 0.0 0.0 0.75
```

더 긴 1080p 영상:

```bash
scripts/isaacsim/run_render_macgvbot_usd.sh \
  --width 1920 \
  --height 1080 \
  --frames 300 \
  --fps 30
```

GUI 창을 띄워 확인하면서 렌더링:

```bash
scripts/isaacsim/run_render_macgvbot_usd.sh \
  --no-headless
```

PNG 프레임만 저장:

```bash
scripts/isaacsim/run_render_macgvbot_usd.sh \
  --no-video
```

## 서랍 움직임 옵션

서랍의 움직이는 rigid body prim path를 `--drawer-prim`으로 지정하면 기존
prismatic joint의 범위와 damping을 설정하고 그리퍼 접촉력으로 서랍을 움직입니다.

```bash
scripts/isaacsim/run_render_macgvbot_usd.sh \
  --drawer-prim /drawer/drawer_floor_02 \
  --drawer-axis x \
  --drawer-distance 0.26
```

`--drawer-prim`은 실제 USD 안의 prim path와 일치해야 합니다. Isaac Sim GUI에서
`macgvbot.usd`를 열고 Stage 패널에서 움직일 서랍 파트를 선택하면 prim path를 확인할
수 있습니다. 현재 USD에서 로컬 `+X`는 월드 `-X` 방향이므로 양수 거리 `0.26`이
서랍이 로봇 쪽으로 열리는 범위입니다.

## 서랍 제어 모드 (`--drawer-control`)

`--drawer-control`로 서랍이 움직이는 방식을 선택합니다. 기본값은 `animated`입니다.

- `animated`(기본): 기존 동작을 그대로 유지합니다.
- `physics`: 서랍 transform을 직접 조작하지 않고, **그리퍼가 손잡이를 잡는 순간
  생성되는 grasp constraint(FixedJoint)로만** 서랍을 움직입니다. 서랍 이동은 로봇
  움직임 → gripper link → grasp constraint → drawer rigid body → prismatic joint
  의 물리 결과입니다.

```bash
scripts/isaacsim/run_render_macgvbot_usd.sh \
  --tool screwdriver \
  --drawer-id 1 \
  --drawer-prim /drawer/drawer_floor_02 \
  --drawer-control physics \
  --no-headless \
  --frames 180 \
  --no-video
```

`physics` 모드 동작 흐름:

1. 로봇이 `JOINT_BOUNDARIES["HANDLE"]` 하드코딩 pose(손잡이 위치)로 이동 (auto-IK·grasp gate 미사용)
2. 그리퍼 close → `gripper_body`와 선택 drawer 사이에 grasp constraint 생성
3. 로봇이 `JOINT_BOUNDARIES["PULL"]`로 이동 → 서랍이 prismatic 축을 따라 물리적으로 열림
4. 로봇이 다시 HANDLE pose로 복귀 → 서랍이 물리적으로 닫힘
5. 그리퍼 open → grasp constraint 제거

`--grasp-anchor`로 서랍을 묶을 그리퍼 rigid-body link를 바꿀 수 있습니다(기본은
손끝 `/m0609/onrobot_rg2ft/left_inner_finger` — 서랍을 손잡이 근처에 고정).
grasp constraint는 손끝이 실제로 손잡이에 닿았을 때(손끝중점↔손잡이 ≤
`--grasp-center-tolerance`)만 생성되며, 닿지 않은 채 fallback 시점이 지나면
경고와 함께 강제로 결합해 동작이 멈추지 않게 합니다.

`--drawer-open-fraction`(기본 0.95)으로 서랍이 열리는 정도를 조절합니다. 값이 클수록
더 많이 열립니다. 이 값은
HANDLE→PULL 포즈 이동량의 비율이며, 서랍은 손끝 이동량만큼만 물리적으로 열립니다
(과도한 인출 방지). 1.0이면 기존처럼 PULL 포즈까지 완전히 이동합니다.

`--grasp-pose-offset`(기본 `0,0,0,0,0,0`, 비활성)로 grasp/pull 포즈에 관절별 각도(도)
오프셋을 더해 손끝 위치를 미세조정할 수 있습니다. 예: `2,0,0,0,0,0`(joint_1 좌우),
`0,-3,0,0,0,0`(joint_2 상하). 기본은 보정 없이 원래 HANDLE 포즈를 사용합니다.

비선택 서랍은 physics 모드에서 kinematic으로 고정되어, 홈 복귀 중 그리퍼가 스쳐도
열리지 않습니다.

`--gripper-closed-degrees`(기본 50, 범위 ~0..67.6)로 그리퍼가 닫힐 때 `finger_joint`
목표 각도를 조절합니다. 값이 클수록 손가락 간격이 좁아져 더 꽉 쥡니다. 실제 서랍
결합은 grasp constraint가 담당하므로 이 값은 주로 시각적 악력입니다.

physics 모드에서는 손잡이로 가기 전 관찰(탐지/인스펙션) 포즈를 경유하지 않고 HOME에서
곧바로 손잡이 접근 자세로 이동합니다.

검증용 로그(`[PHYSICS]` 접두):

```text
[PHYSICS] drawer setup done: prim=..., joint=..., rest=...
[PHYSICS] grasp constraint created @frame N ...
[PHYSICS] frame N: drawer disp=... m        # 열림에 따라 증가
[PHYSICS] grasp constraint removed @frame N ...
[PHYSICS] RESULT open=True (max .../... m), close=True (final ... m)
```

`RESULT`의 성공 판정 기준: pull 후 서랍이 목표거리의 50% 이상 열렸으면
`open=True`, push 후 서랍이 원위치 ±3cm 이내로 돌아오면 `close=True`.

## 스크립트 위치

```text
scripts/isaacsim/render_macgvbot_usd.py
scripts/isaacsim/run_render_macgvbot_usd.sh
```

이 스크립트는 `macgvbot.usd`를 열고 `/World/MacGyvBotCaptureCamera` 캡처 카메라와
보조 조명을 추가한 뒤 Replicator `BasicWriter`로 RGB 프레임을 저장합니다. 기본
카메라는 로봇/서랍 주변을 짧게 orbit하도록 설정되어 있습니다. `--simulate-twin`
기본값은 켜져 있으며, 스크립트 내부 joint waypoint로 로봇과 그리퍼 target을
갱신합니다.
