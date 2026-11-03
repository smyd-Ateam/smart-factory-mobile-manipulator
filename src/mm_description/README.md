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

## 토픽 · 프레임

자세한 기준은 `docs/interfaces.md` 참고.

- 입력: `/cmd_vel`
- 출력: `/odom`, `/scan`, `/tf`, `/joint_states`, `/clock`
- TF: `odom → base_footprint → base_link → base_scan` (+ 바퀴, 캐스터)

## 파일

| 파일 | 내용 |
|---|---|
| `urdf/mm_base.urdf.xacro` | 차체 모델 (치수는 파일 위쪽에 모여 있음, 임시값) |
| `worlds/test_room.sdf` | 6 m × 6 m 테스트 방, 장애물 3개 |
| `launch/sim.launch.py` | 실행 파일 |
| `rviz/base.rviz` | RViz 설정 |
