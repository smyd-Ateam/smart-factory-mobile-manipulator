# ogre2 카메라 회색 프레임 진단 (월드 yaw별)

진단 전용 폴더입니다. 프로덕션 코드가 아니고, 레포의 기존 패키지는 수정하지 않습니다.

로봇을 제자리에서 한 바퀴(360°) 천천히 돌리면서 `/camera/image_raw`를 받아, 화면이 한 가지 색으로만 나오는 "회색 프레임"이 어느 월드 yaw에서 생기는지 기록합니다. Mac VM(ogre2 + CPU 렌더링)에서는 yaw가 0°, ±90°, 180°에서 약 10° 이상 벗어나면 카메라가 회색이 됩니다. 다른 환경(WSL2 + GPU 등)에서도 같은 현상이 나는지 확인하려고 만들었습니다.

- Gazebo + 로봇 + 카메라만 띄웁니다 (`mm_description/sim.launch.py` 그대로, RViz 없이 headless). Nav2, YOLO, Align은 실행하지 않습니다.
- 실행 시간: 빌드 몇 초 + Gazebo 시작부터 회전 끝까지 약 30~40초 (Mac VM에서 31~32초)
- 이 브랜치는 `feat/grasp` 5741526 (RGB-D 카메라가 달린 mm_description)에서 만들었습니다. 스크립트가 이 checkout 안의 `src/mm_description`, `src/dofbot_description`을 직접 빌드해서 씁니다.

## 필요한 것

평소 시뮬을 돌리던 환경이면 추가 설치는 필요 없습니다.

- Ubuntu 22.04 + ROS 2 Humble (`/opt/ros/humble`)
- Gazebo Fortress (`ign gazebo`), `ros_gz_bridge`, `ros_gz_sim`, `xacro`, `robot_state_publisher`
- `colcon`, Python 3, `numpy`

확인:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix ros_gz_bridge
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix xacro
ign gazebo --versions
```

## 1. 브랜치 받기

처음 받는 경우:

```bash
git clone https://github.com/smyd-Ateam/smart-factory-mobile-manipulator.git
cd smart-factory-mobile-manipulator
git fetch origin
git checkout diagnostics/ogre2-gray-yaw-bundle
```

이미 레포가 있는 경우 (작업 중인 변경이 있으면 먼저 commit 또는 stash):

```bash
cd smart-factory-mobile-manipulator
git fetch origin
git checkout diagnostics/ogre2-gray-yaw-bundle
git pull
```

## 2. 실행

실행 전에 켜 둔 Gazebo가 있으면 먼저 끕니다 (스크립트는 Gazebo가 이미 떠 있으면 실행하지 않고 멈춥니다):

```bash
pkill -f "ign gazebo"
```

레포 최상위 폴더에서 한 줄로 실행합니다. 평소 쓰는 워크스페이스를 source하지 않아도 됩니다 (스크립트가 `/opt/ros/humble`과 번들 안 빌드 결과를 직접 source합니다).

```bash
tools/diagnostics/ogre2-gray-yaw/run_gray_yaw.sh
```

기본값은 `RENDER_ENGINE=ogre2`, `SOFTWARE_GL=false`(GPU 사용), `WORLD=test_room.sdf`, `SPEED=0.35` rad/s 입니다.

시간이 되면 아래 비교도 한 번씩 부탁드립니다 (각각 약 40초):

```bash
# 같은 ogre2를 CPU 렌더링으로
SOFTWARE_GL=true tools/diagnostics/ogre2-gray-yaw/run_gray_yaw.sh

# ogre(1)로
RENDER_ENGINE=ogre tools/diagnostics/ogre2-gray-yaw/run_gray_yaw.sh
```

옵션:

| 환경변수 | 기본값 | 설명 |
|---|---|---|
| `RENDER_ENGINE` | `ogre2` | `ogre2` 또는 `ogre` |
| `SOFTWARE_GL` | `false` | `true`면 CPU 렌더링 (`LIBGL_ALWAYS_SOFTWARE`) |
| `WORLD` | `test_room.sdf` | `mm_description/worlds` 안의 월드 이름 |
| `SPEED` | `0.35` | 회전 속도 (rad/s). 카메라 10 Hz 기준 프레임당 약 2°. 더 촘촘히 보려면 0.2 |

결과 폴더를 지정하려면 첫 번째 인자로 줍니다: `run_gray_yaw.sh ./my_result`

## 3. 보내 줄 것

실행이 끝나면 콘솔 마지막에 아래 모양의 블록이 나옵니다. `=====` 줄부터 `=====` 줄까지 전체를 복사해서 보내 주세요. 같은 내용이 결과 폴더의 `report.txt`에도 저장됩니다.

```text
===== ogre2 회색 yaw 진단 결과 (아래 전체를 복사해서 보내 주세요) =====
branch/commit : diagnostics/ogre2-gray-yaw-bundle <커밋>
OS            : Ubuntu 22.04.5 LTS | kernel ... | x86_64 | WSL=yes
ROS / Gazebo  : humble / Gazebo 6.x
render engine : 요청 ogre2, 실제 로드 ignition-rendering-ogre2
GPU 가속      : software_gl=false | OpenGL renderer: ... | nvidia-smi: ... | /dev/dxg=yes
world / speed : test_room.sdf / 0.35 rad/s
시간          : 빌드 ..s, Gazebo~회전 끝 ..s
--- 판정 ---
프레임 ...개 (회전 중 ...개), 회전량 ...°, 프레임당 약 ...°, 회색 프레임 ...개
회색 구간 (odom yaw, 월드 기준):   ← 또는 "회색 현상 없음"
  1) yaw +10.8° ~ +77.6°  (...)
결과 폴더     : ...
=====================================================================
```

중간에 `FAIL:`로 끝나면 그 줄과 결과 폴더의 `sim.log` 마지막 30줄을 보내 주세요.

여유가 되면 결과 폴더의 `frames.csv`, `summary.json`도 같이 보내 주시면 프레임별로 비교할 수 있습니다 (이미지 자체는 저장하지 않아 용량이 작습니다).

## 판정 방법

- 프레임마다 4픽셀 간격 표본으로 픽셀 표준편차(std)와 색 개수를 계산합니다. `std < 2.0`이고 색이 4개 이하이면 회색 프레임입니다. Mac VM에서 회색 프레임은 std 0, 색 1개(밝기 218)였고, 정상 프레임은 std 69 이상이었습니다.
- yaw는 `/odom` 기준입니다. 로봇이 원점에서 yaw 0으로 생성되므로 월드 yaw와 같습니다 (odom 회전 오차 0.1% 미만). 각 이미지의 header.stamp와 가장 가까운 odom 값을 붙입니다.
- 같은 stamp의 깊이 이미지(`/camera/depth/image_raw`)에서 유효한 픽셀 비율도 기록합니다. 회색이면서 깊이도 비어 있으면(유효 0) 렌더링 실패로 보고, 깊이가 유효하면 무늬 없는 면을 본 것일 수 있다고 표시합니다.
- 회전 전에 팔을 Far 관측 자세(`src/mm_description/scripts/set_observe_pose_far.sh`)로 바꿉니다.
- 이미지 구독은 RELIABLE입니다 (bridge가 RELIABLE로 발행, BEST_EFFORT로 받으면 큰 이미지가 많이 빠짐).

## Mac VM 기준 결과 (참고)

Ubuntu 22.04 aarch64 (UTM), Gazebo 6.18.0, ogre2 + `SOFTWARE_GL=true` (llvmpipe), 2회 실행 결과가 같았습니다:

```text
회색 구간 (odom yaw, 월드 기준):
  1) yaw +10.8° ~ +77.6°   깊이 유효 0 → 렌더링 실패
  2) yaw +101.5° ~ +167.6° 깊이 유효 0
  3) yaw -168.5° ~ -102.4° 깊이 유효 0
  4) yaw -77.8° ~ -11.7°   깊이 유효 0
```

0°, ±90°, 180° 근처 약 ±11° 안에서만 정상 화면이 나옵니다.

## 결과 폴더 내용

기본 위치: `tools/diagnostics/ogre2-gray-yaw/results/<날짜_시각>/` (git에 올라가지 않음)

| 파일 | 내용 |
|---|---|
| `report.txt` | 보내 줄 요약 (콘솔 출력과 같음) |
| `frames.csv` | 프레임별 stamp, yaw, std, 색 개수, 평균 밝기, 깊이 유효 비율, 회색 판정 |
| `summary.json` | 회색 구간, 프레임 수, 회전량 |
| `sim.log`, `build.log`, `pose.log`, `probe.txt` | 실행 로그 |

빌드 결과는 `tools/diagnostics/ogre2-gray-yaw/.ws/`에 생기며 git에 올라가지 않습니다. 지우고 다시 실행해도 됩니다.
