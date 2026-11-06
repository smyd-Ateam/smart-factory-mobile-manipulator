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
- 목표 근처에서 10초 넘게 맴돌 때가 있음, 원인 미확인
- RViz를 켠 실행에서 차체가 안 움직였는데 성공으로 끝난 적이 있음, 원인 미확인. 당시 로그에 `Transform data too old ... map to odom`이 함께 나왔다. 도착 여부는 성공 응답만 믿지 말고 위치로도 확인한다.
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
