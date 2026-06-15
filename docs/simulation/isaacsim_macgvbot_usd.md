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

서랍의 움직이는 파트 prim path를 알고 있다면 `--drawer-prim`을 지정해서 간단한
열림 애니메이션을 같이 캡처할 수 있습니다.

```bash
scripts/isaacsim/run_render_macgvbot_usd.sh \
  --drawer-prim /World/Drawer/SlidingPart \
  --drawer-axis x \
  --drawer-distance 0.26
```

`--drawer-prim`은 실제 USD 안의 prim path와 일치해야 합니다. Isaac Sim GUI에서
`macgvbot.usd`를 열고 Stage 패널에서 움직일 서랍 파트를 선택하면 prim path를 확인할
수 있습니다. 축 방향이 반대라면 `--drawer-distance -0.26`처럼 음수를 사용합니다.

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
