# mm_description

직접 만든 차동구동 차체 + 라이다 시뮬 패키지 (DOFBOT 팔은 나중에 추가)

## 빌드

레포 폴더에서

```bash
colcon build --packages-select mm_description
source install/setup.bash
```

새 터미널을 열 때마다 `source install/setup.bash`를 다시 해야 한다.

## 실행

| 환경 | 명령어 |
|---|---|
| 기본 | `ros2 launch mm_description sim.launch.py` |
| Windows + WSL2 | `ros2 launch mm_description sim.launch.py software_gl:=true` |
| Mac UTM VM | `ros2 launch mm_description sim.launch.py software_gl:=true headless:=true rviz_gpu:=true` |

Gazebo, 차체, 토픽 연결(브리지), RViz가 한 번에 켜진다. 끌 때는 `Ctrl+C`.

## launch 옵션

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `software_gl` | `false` | `true`면 CPU 렌더링 (`LIBGL_ALWAYS_SOFTWARE=1`). WSL2, Mac VM에서 필요 |
| `headless` | `false` | `true`면 Gazebo 화면 없이 계산만 함. 로봇은 RViz로 확인 |
| `rviz` | `true` | `false`면 RViz를 띄우지 않음 |
| `rviz_config` | `rviz/base.rviz` | RViz 설정 파일 경로 |
| `rviz_gpu` | `false` | `true`면 RViz만 GPU로 그림 (Mac VM용). `false`면 RViz도 `software_gl` 설정을 따름 (WSL에서 RViz가 꺼지면 `false` 유지) |
| `render_engine` | `ogre` | Gazebo 렌더링 엔진. ogre2는 Mac VM, WSL2에서 튕겨서 ogre 사용 |

## 동작 확인

새 터미널에서 (`source install/setup.bash` 먼저)

```bash
# 라이다: 벽까지 거리(약 3 m)가 나오면 정상, 전부 0.12면 렌더링 문제 → software_gl:=true
ros2 topic echo /scan --once --field ranges | head -20

# 시뮬 속도 (1에 가까울수록 실시간)
ign topic -e -t /stats -n 3 | grep real_time_factor

# 차체 움직이기 (원을 그리며 돌면 정상, Ctrl+C로 멈춤)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}"
```

## 차체 치수

PR #4(희선님, 로봇팔 통합용 차체 수정) 기준 값. 바꿀 때는 `urdf/mm_base.urdf.xacro` 위쪽 값을 고친다.

| 항목 | 값 |
|---|---|
| 차체 (x × y × z) | 0.38 × 0.28 × 0.06 m, 바닥에서 0.02 m 띄움 |
| 바퀴 | 반지름 0.05 m, 폭 0.04 m, 간격 0.35 m (충돌 모양은 구) |
| 캐스터 | 반지름 0.02 m 구, x ±0.15 m, 바닥에서 3 mm 띄움 (`caster_gap` 0.003)[^gap] |
| 라이다 (`base_scan`) | `base_footprint` 기준 (0.18, 0, 0.125) m = `base_link` 기준 (0.18, 0, 0.075) m |
| 라이다 사양 | `gpu_lidar` 10 Hz, 360개, 0.12 ~ 8.0 m |

- `base_link`는 바퀴 축 높이(바닥에서 0.05 m), `base_footprint`는 바닥.
- 캐스터 간극 때문에 차체가 앞 캐스터가 닿을 때까지 앞으로 기운다: 3 mm면 약 0.020 rad.
- 오도메트리 오차와 캐스터 간극 (scratchpad 시뮬 조사, 2026-11-17, 예전 차체 0.40 × 0.30 × 0.10 m 기준): 회전 · 직진 오차는 간극 0 ~ 3 mm에서 모두 0%로 간극과 무관했다. 예전 회전 오차 13%의 원인은 바퀴 충돌 모양(원통 → 구로 수정)이었다. 지금은 캐스터 마찰이 0이라 간극을 2 mm 이하로 줄여도 오차에는 영향이 없다. 단 캐스터에 마찰이 생기면 0 mm는 위험하다(캐스터가 바닥을 붙잡아 차체가 안 움직이고 바퀴만 헛돎).

[^gap]: 2 mm(앞 기울기 약 0.013 rad)로 줄이는 변경이 `feat/state-machine`(0782f98)에 있으나 아직 승인 안 된 제안값이다.

## 토픽 · 프레임

자세한 기준은 `docs/interfaces.md` 참고.

- 입력: `/cmd_vel`
- 출력: `/odom`, `/scan`, `/tf`, `/joint_states`, `/clock`
- TF: `odom → base_footprint → base_link → base_scan` (+ 바퀴, 캐스터)

## 파일

| 파일 | 내용 |
|---|---|
| `urdf/mm_base.urdf.xacro` | 차체 모델 (치수는 파일 위쪽에 모여 있음, 위 차체 치수 표 참고) |
| `worlds/test_room.sdf` | 6 m × 6 m 테스트 방, 장애물 3개 |
| `launch/sim.launch.py` | 실행 파일 |
| `rviz/base.rviz` | RViz 설정 |
