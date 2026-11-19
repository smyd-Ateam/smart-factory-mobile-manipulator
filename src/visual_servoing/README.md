# visual_servoing

카메라 피드백으로 모바일 베이스를 미세 정렬하는 패키지 (D). 설계는 `docs/visual_servoing_design.md`.

Align 액션 서버 `/servoing/align` (`project_interfaces/action/Align`)를 제공한다.

## 빌드

같은 레포의 `project_interfaces` 패키지(Align 액션)가 필요하다. 둘을 같이 빌드한다.

```bash
cd ~/smart-factory-mobile-manipulator
source /opt/ros/humble/setup.bash
colcon build --packages-select project_interfaces visual_servoing
source install/setup.bash
```

`~/.local`에 setuptools를 따로 설치한 WSL 환경에서 `canonicalize_version()` 오류가 나면 `PYTHONNOUSERSITE=1 colcon build ...`로 빌드한다.

## 모의 입력으로 실행

Gazebo 카메라, YOLO 없이 모의 차체 + 모의 카메라로 정렬을 확인한다.

```bash
ros2 launch visual_servoing mock_align.launch.py
# 지연, 노이즈 추가
ros2 launch visual_servoing mock_align.launch.py delay_sec:=0.5 noise_px:=2.0 noise_depth:=0.005
```

목표 한 번 보내기 (다른 터미널):

```bash
ros2 action send_goal /servoing/align project_interfaces/action/Align \
  "{target_class: pin, standoff_m: 0.25, timeout_sec: 60.0}" --feedback
```

반복 시험 (다른 터미널, launch가 떠 있는 상태):

```bash
ros2 run visual_servoing mock_trials --ros-args -p trials:=20 -p csv:=/tmp/trials.csv
```

시험마다 모의 차체를 Nav2 도착 오차만큼 흐트러진 위치(물체에서 `standoff_m + 0.15` m, 위치 오차 반지름 0.13 m, yaw ±0.15 rad)에 두고 목표를 보낸 뒤, 액션 결과와 모의 차체의 실제 오차를 비교한다.

RViz가 같이 켜진다 (위에서 본 화면). 주황 상자 = 차체, 파란 물체 = 목표 물체, 초록 원 = 목표 거리(`standoff_m`).
차체 중심(`base_link`)이 초록 원 위에 오고 물체를 정면으로 보면 정렬 완료다. RViz 없이 실행하려면 `rviz:=false`.

## Gazebo 카메라로 실행 (시각 피드백 정렬 테스트)

Gazebo Fortress가 렌더링한 깊이 영상과 boundingbox 카메라 정답 박스로 정렬한다 (설계 문서 5.2).
Gazebo는 화면 없이(`-s`, ogre2) 뜬다. 필요한 것: `ros-humble-ros-gz`.

```bash
ros2 launch visual_servoing sim_align.launch.py
```

반복 시험 (다른 터미널):

```bash
ros2 run visual_servoing mock_trials --ros-args -p scene:=sim -p use_sim_time:=true   -p settle_sec:=2.0 -p wall_factor:=5.0 -p trials:=20 -p csv:=/tmp/sim_trials.csv
```

- 월드 `worlds/align_sim.sdf`: 단순 차체(`mm_test_base`, 바퀴 물리 없이 cmd_vel을 그대로 따름) + 카메라 + `pin` 1개 (feat/yolo 모델과 같은 모양, label 2).
- 카메라 위치·각도는 임시값으로 `mock_model.CameraMount`와 같다 (`test_sim.py`가 확인). B의 near 자세가 정해지면 바꾼다.
- YOLO 검출 노드(C)로 시험할 때는 `gt_detections:=false`로 정답 박스를 끄고, 검출 노드가 `/camera/image_raw`를 받아 `/perception/detections`를 내게 한다.
- 정렬 시간은 시뮬레이션 시간이다. WSL 소프트웨어 렌더링에서 RTF는 약 0.65.

## 단위 테스트

```bash
cd src/visual_servoing
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test
```

`~/.local`에 pytest를 따로 설치한 환경에서는 ROS의 `launch_testing` 플러그인과 충돌해서 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`이 필요하다.

## Align 액션 동작

| 상황 | 결과 |
|---|---|
| 정지 후 찍힌 프레임 3장 연속 허용 범위 안 | `success=true`, `error_code=""` |
| `timeout_sec` 초과 | `ALIGN_TIMEOUT` |
| 새 검출을 1초(`lost_timeout`) 동안 못 받음 (시작부터 못 본 경우, 정렬 중 놓친 경우) | `OBJECT_NOT_FOUND` |
| 취소 요청 | 0 속도 발행 후 CANCELED |
| 빈 `target_class`, `standoff_m <= 0`, `timeout_sec <= 0`, 이미 실행 중인 목표가 있음 | 목표 거부 |
| 노드 종료 (Ctrl+C, SIGTERM) | 실행 중인 목표 중단, 0 속도 발행 후 종료 |

`/servoing/cmd_vel`은 목표 실행 중에만 발행하고, 끝나면 0을 한 번 보낸다.

## 노드

| 노드 | 입력 | 출력 |
|---|---|---|
| `align_node` | `/perception/detections`, `/camera/depth/image_raw`, `/camera/camera_info`, TF, 액션 `/servoing/align` | `/servoing/cmd_vel` |
| `mock_scene_node` (시험용) | `/servoing/cmd_vel` | `/perception/detections`, `/camera/depth/image_raw`, `/camera/camera_info`, TF, `/servoing/mock_object_in_base`, 서비스 `/servoing/mock_reset` |
| `mock_trials` (시험용) | 위 노드들 (`scene:=mock` 또는 `sim`) | 시험 결과 요약, CSV |
| `sim_scene_node` (Gazebo 시험용) | `/servoing/sim_boxes`, `/servoing/sim_base_pose` | `/perception/detections` (정답 박스), `/servoing/sim_object_in_base`, 서비스 `/servoing/sim_reset` |

`align_node` 파라미터: 제어 `k_v`, `k_w`, `max_v`, `max_w`, `bearing_gate`, `tol_range`, `tol_bearing`, `stop_ratio` / 판정 `lost_timeout`, `max_age`, `settle`, `required_frames` / 기타 `score_min`, `control_rate`, `feedback_rate`, `center_offsets`.

`center_offsets`: 깊이는 물체 앞면까지 거리라서 물체 중심까지 더할 클래스별 반폭 (`"클래스:미터"` 목록, 기본 block 0.0125, pin 0.010, hex_nut 0.015, bolt 0.006). 모의 입력(`mock_align.launch.py`)은 깊이가 중심까지라 0으로 둔다.

## 파일

| 파일 | 내용 |
|---|---|
| `visual_servoing/geometry.py` | 오차 계산: 깊이 처리, 역투영, 좌표 변환, 거리·방위 오차 |
| `visual_servoing/control.py` | P제어 법칙 |
| `visual_servoing/session.py` | 목표 하나의 진행 판단: 정지 후 완료 판정, 시간 초과, 물체 놓침 |
| `visual_servoing/align_node.py` | Align 액션 서버 |
| `visual_servoing/mock_model.py`, `mock_scene_node.py` | 모의 차체·카메라 |
| `visual_servoing/mock_trials.py` | 반복 시험 (모의 입력, Gazebo) |
| `visual_servoing/sim_model.py`, `sim_scene_node.py` | Gazebo 시험 보조 (label → 클래스, 차체 위치 리셋) |
| `launch/sim_align.launch.py`, `worlds/align_sim.sdf` | Gazebo 시험 |
| `test/` | 단위 테스트, 모의 폐루프 테스트 |
