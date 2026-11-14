# mm_navigation

차동구동 차체의 지도 작성(slam_toolbox)과 자율주행(Nav2) 실행 패키지

## 빌드

레포 폴더에서

```bash
colcon build --packages-select mm_description mm_navigation
source install/setup.bash
```

## 실행

시뮬을 다시 켜기 전에 남은 서버를 정리한다: `pkill -f "ign gazebo"`

### 자율주행

아래는 상태머신 없이 테스트하는 명령이다 (`standalone:=true`).

| 환경 | 명령어 |
|---|---|
| Mac UTM VM | `ros2 launch mm_navigation nav2.launch.py software_gl:=true headless:=true rviz_gpu:=true standalone:=true` |
| Windows + WSL2 | `ros2 launch mm_navigation nav2.launch.py software_gl:=true standalone:=true` |

RViz의 `2D Goal Pose`로 목표를 찍으면 움직인다. 초기 위치는 스폰 위치 (0, 0, 0)으로 자동 설정된다.

### 지도 만들기

| 환경 | 명령어 |
|---|---|
| Mac UTM VM | `ros2 launch mm_navigation slam.launch.py software_gl:=true headless:=true rviz_gpu:=true` |
| Windows + WSL2 | `ros2 launch mm_navigation slam.launch.py software_gl:=true` |

저장 (타임아웃을 안 늘리면 실패한다)

```bash
ros2 run nav2_map_server map_saver_cli -f <경로> --ros-args -p save_map_timeout:=10.0
```

## launch 옵션

`software_gl`, `headless`, `rviz`, `rviz_gpu`, `render_engine`은 `mm_description/README.md`와 같다.

`nav2.launch.py` 전용 옵션

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `standalone` | `false` | `true`면 Nav2 출력을 `/cmd_vel`로 바로 보냄 (상태머신 없이 테스트). `false`면 `/nav2/cmd_vel`로 보내므로 상태머신이 없으면 차체가 움직이지 않음 |
| `map` | `maps/test_room.yaml` | 지도 yaml 경로 |
| `params_file` | `config/nav2_params.yaml` | Nav2 파라미터 파일 경로 |

## 속도 명령 흐름

기준은 `docs/interfaces.md` 3.1, 5.0.

```
controller_server → cmd_vel_nav → velocity_smoother → /nav2/cmd_vel → 상태머신 → /cmd_vel
behavior_server (복구 동작)                         → /nav2/cmd_vel
```

behavior_server는 velocity_smoother를 거치지 않고 `cmd_vel`로 바로 내보낸다. 그대로 두면 복구 동작 때 상태머신을 건너뛰므로 같은 출력으로 리매핑했다.

기본 `navigation_launch.py`로는 이 출력을 바꿀 수 없어서 `nav2.launch.py`에서 노드를 직접 띄운다.

## 기본값에서 바꾼 파라미터

`config/nav2_params.yaml`은 Humble `nav2_bringup` 기본 파일을 복사한 것이고, 바꾼 줄에는 `# 변경` 표시가 있다.

| 항목 | 값 | 이유 |
|---|---|---|
| footprint | 0.40 × 0.40 m | 바퀴 포함 폭 약 0.39 m. 팔을 올리면 다시 조정 |
| 최대 속도 | 직진 0.25 m/s, 회전 0.8 rad/s | 팀 합의값 |
| 가감속 | 직진 ±1.0 m/s², 회전 ±2.0 rad/s² | `mm_description/urdf/mm_base.urdf.xacro`의 DiffDrive 플러그인 제한(`max_linear_acceleration` 등)에 맞춤. 플러그인 값 자체는 측정값이 아니라 임시값이므로, 바꾸면 여기도 같이 바꾼다 |
| 도착 허용 오차 | xy 0.10 m (yaw는 기본값 0.25 rad) | 정렬(D) 시작 오차 범위 ±0.15 m, ±0.35 rad 안에 들어오도록 |
| 충돌 critic | `BaseObstacle` → `ObstacleFootprint` | `BaseObstacle`은 원형 로봇 기준이라 사각형 모서리를 고려하지 못함 |
| AMCL 초기 위치 | `set_initial_pose: true`, (0, 0, 0) | 상태머신이 RViz 조작 없이 시작하도록 |
| AMCL 라이다 범위 | 0.12 ~ 8 m | 라이다 사양 |

## 주의사항

- 가감속(직진 ±1.0 m/s², 회전 ±2.0 rad/s²)은 DiffDrive 플러그인 제한에 맞춘 값이고, 플러그인 값 자체가 측정값이 아니라 임시값이다.
- 라이다 높이가 지면에서 약 16.5 cm라서 `block`(4 cm) 같이 낮은 물체는 `/scan`과 costmap에 잡히지 않는다. Nav2는 이런 물체를 장애물로 피하지 않으므로, 물체를 밟지 않도록 목표 지점을 물체에서 떨어뜨려 정해야 한다.
- 목표 근처 맴돌기: 다가오는 방향과 목표 yaw가 크게 다르면(약 180°) 도착 후 제자리 회전 중 맴돌 수 있다. DWB와 goal checker가 서로 다른 시각의 `map→odom`을 쓰고 1 Hz 재계획마다 둘 다 초기화되기 때문 (Humble 1.1.20 소스로 확인, 2026-11-13). 재계획을 거리 기준으로 바꾸면 두 판정이 더 벌어져 멈춰 버려서 설정은 그대로 두고, 목표 yaw를 다가오는 방향에 가깝게 정한다 (`docs/interfaces.md` 6장). 회귀 테스트: `MM_SIM_TEST=1 PYTHONNOUSERSITE=1 colcon test --packages-select mm_navigation` (`test/sim/test_spin_at_goal.py`, 지금은 맴돌면 실패함)
- 가짜 성공: controller_server가 볼 때 `map→odom`이 오래돼 목표 변환이 실패하면, 변환 실패가 무시되어 목표가 기본값 (0, 0, yaw 0)으로 판정된다. 차체가 odom 원점 근처(허용 오차 안)에 있으면 안 움직여도 성공으로 끝난다. 로그에는 `Transform data too old ... map to odom` 바로 뒤에 `Reached the goal!`이 나온다. Nav2 1.1.20 소스와 AMCL TF 발행을 일부러 끈 실험으로 확인했다. 도착 여부는 성공 응답만 믿지 말고 위치로도 확인한다.
- 위 문제의 계기인 TF 멈춤은 원인 미확인이다 (조사 중단, 2026-11-14). 확인한 것:
  - 재현: RViz 켠 실행 약 27회 중 1회 (2026-11-13). AMCL은 `map→odom`을 계속 정상 발행했고(별도 감시 노드가 받음), controller_server만 멈춘 `map→odom`을 약 12.5 s 동안 봤다 (`Transform data too old`). 보내는 쪽(AMCL)이 아니라 받는 쪽 전달 문제로 보인다
  - 그때 `/clock` 발행자는 1개, 시계 역행 없음, `odom→base_link`·`/odom`·`/scan` 지연 0.1 s 이하
  - Fast DDS 공유메모리(SHM)를 끈 UDP 전용 설정과 비교(RViz 켬, 각 10회)했지만 둘 다 재현 0회라 판단하지 못했다
  - 대응: 상태머신이 `map→odom` 신선도(`tf_stale`)와 도착 위치로 확인하므로 가짜 성공은 걸러진다
- 이전 실행 노드가 남아 있으면 `/cmd_vel` 0이 섞여 `Failed to make progress`가 난다 (2026-11-09 확인). 실행 전 남은 노드를 확인한다 (`ros2 topic info /cmd_vel`의 Publisher 수, `pgrep -fa task_manager`). 상태머신(`mm_task_manager`)은 시작할 때 다른 `/cmd_vel` 발행자가 있으면 `NAV_NOT_READY` / `cmd_vel_conflict`로 시작하지 않는다.
- 남은 Gazebo 서버: launch 프로세스에만 SIGINT를 보내면(launch_testing, 스크립트에서 종료) launch는 끝나도 `ign gazebo` 서버가 남는다 (2026-11-14, 20/20회). 다음 실행과 겹치면 `/clock`이 중복되므로 실행 전 `pkill -f "ign gazebo"`로 정리한다. 위 TF 멈춤 재현 때는 `/clock` 발행자가 1개라 원인이 아니었다
- 시작 단계 실패: AMCL의 `change_state` 응답이 timeout되어(`failed to send response to /amcl/change_state`) `lifecycle_manager_localization`이 `Configuring amcl`에서 멈추고, `map→odom`이 나오지 않아 Nav2가 활성화되지 않은 적이 1회 있다. 원인 미확인이며, 목표를 보내기 전에 Nav2 활성화를 확인한다. 같은 증상이 `controller_server`, `waypoint_follower`에서도 났다 (2026-11-14, 기본 DDS 설정 10회 중 3회, UDP 전용 설정 10회 중 0회. 횟수가 적어 SHM 관련 여부는 판단 못 함). 상태머신은 `NAV_NOT_READY`로 시작하지 않는다
- Mac UTM VM은 Mac 잠자기 등으로 일시정지됐다가 깨어나면 시계가 한 번에 바뀐다 (journal의 `Clock change detected`). 실행 중에 생기면 목표 실패 · 메시지 끊김이 난다 (2026-11-14 2회 확인). 긴 시뮬 실험은 Mac 잠자기를 막고 돌리고, 이상한 결과는 `journalctl | grep "Clock change"` 시각과 대조한다
- RViz에서 벽 안쪽으로 costmap 띠가 한 줄 더 평행하게 보일 때가 있다. 1회 측정에서 costmap 장애물 칸이 -x 벽 지도 칸보다 약 0.05 m 안쪽까지 찍혔고, AMCL 위치 오차는 x 방향 0.054 m, yaw -0.025 rad였다. 크기와 방향은 AMCL 위치 오차와 맞지만 원인으로 확정하지는 않았다. 목표 yaw 오차(0.272 rad)는 목표 방향과의 차이라서 직접 원인으로 보지 않는다.
- AMCL 초기 위치는 차체가 (0, 0, 0)에 스폰된다고 가정한다. 스폰 위치를 바꾸면 `config/nav2_params.yaml`의 `initial_pose`도 바꾼다.
- 지도 원점은 Gazebo 월드 원점과 같다 (지도의 벽 위치로 확인). 다른 위치에서 SLAM을 시작해 지도를 새로 만들면 이 관계가 달라질 수 있다.

## 파일

| 파일 | 내용 |
|---|---|
| `launch/slam.launch.py` | 시뮬 + slam_toolbox + RViz |
| `launch/nav2.launch.py` | 시뮬 + 지도 + AMCL + Nav2 + RViz |
| `config/slam_params.yaml` | slam_toolbox 파라미터 |
| `config/nav2_params.yaml` | Nav2 파라미터 |
| `maps/` | `test_room` 지도 (119×121칸, 0.05 m/칸) |
| `rviz/nav.rviz` | RViz 설정 (지도, costmap, 경로, footprint, 파티클) |
