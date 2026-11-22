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
ros2 launch visual_servoing sim_align.launch.py                     # 카메라 임시 위치
ros2 launch visual_servoing sim_align.launch.py camera_pose:=near   # 팀 관측 자세 (near, pregrasp)
ros2 launch visual_servoing sim_align.launch.py camera_pose:=near distractor:=true   # 같은 pin 하나 더
```

반복 시험 (다른 터미널):

```bash
ros2 run visual_servoing mock_trials --ros-args -p scene:=sim -p use_sim_time:=true \
  -p settle_sec:=2.0 -p wall_factor:=5.0 -p trials:=20 -p csv:=/tmp/sim_trials.csv
# near에서 standoff 0.35, Nav2 도착 0.50 m ± 0.07
ros2 run visual_servoing mock_trials --ros-args -p scene:=sim -p use_sim_time:=true \
  -p settle_sec:=2.0 -p wall_factor:=5.0 -p standoff_m:=0.35 -p nav_margin:=0.15 -p nav_error_m:=0.07
```

- 월드 `worlds/align_sim.sdf`: 단순 차체(`mm_test_base`, 바퀴 물리 없이 cmd_vel을 그대로 따름) + 카메라 + `pin` 1개 (feat/yolo 모델과 같은 모양, label 2).
- 카메라 위치·각도: `camera_pose` 인자, 값은 `sim_model.CAMERA_POSES`. `test`는 임시값(`mock_model.CameraMount`와 같음), `near`·`pregrasp`는 feat/grasp URDF에 관측 자세 관절값을 넣어 계산한 값 (near는 B 실측 TF와 같음, pregrasp는 11/21 69.3° 자세). launch가 월드의 카메라 pose와 static TF를 같은 값으로 바꿔 띄운다.
- `distractor:=true`: 목표 옆(y -0.18 m)에 같은 pin을 하나 더 둔다. 정렬이 처음 고른 물체를 계속 따라가는지 확인용.
- YOLO 검출 노드(C)로 시험할 때는 `gt_detections:=false`로 정답 박스를 끄고, 검출 노드가 `/camera/image_raw`를 받아 `/perception/detections`를 내게 한다.
- 정렬 시간은 시뮬레이션 시간이다. WSL 소프트웨어 렌더링에서 RTF는 약 0.65.

### YOLO 검출로 정렬 (C 검출 노드)

```bash
ros2 launch visual_servoing sim_align.launch.py camera_pose:=near gt_detections:=false
# 다른 터미널: feat/yolo의 src/mm_perception/mm_perception/yolo_detector.py
python3 yolo_detector.py --ros-args -p use_sim_time:=true -p model:=<모델 .pt 또는 _openvino_model 폴더> \
  -p publish_image:=false
```

- 검출 노드는 `/camera/image_raw`를 받아 `/perception/detections`를 낸다. stamp는 원본 이미지 stamp라 깊이와 맞는다.
- 추론이 느린 PC(정렬 기준 3 Hz 미만)에서는 `rtf:=0.1`로 시뮬레이션을 느리게 해서 검출 품질만 본다. 모든 노드가 시뮬레이션 시간을 써서 느린 추론도 시뮬레이션 시간으로는 빨라진다. 이때 `mock_trials`에 `-p settle_sec:=10.0 -p wall_factor:=12.0`을 준다.

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

`/camera/depth/image_raw`, `/camera/camera_info`는 RELIABLE(KEEP_LAST 5)로 받는다. 약 1 MB 이미지를 BEST_EFFORT(`sensor_data`)로 받으면 ros_gz_bridge에서 오는 프레임이 40~60 %만 도착했다 (WSL, PR #1 RGB-D 브리지 진단 2026-11-22). 발행 쪽이 BEST_EFFORT면 연결되지 않으니 RELIABLE로 발행해야 한다.

## 노드

| 노드 | 입력 | 출력 |
|---|---|---|
| `align_node` | `/perception/detections`, `/camera/depth/image_raw`, `/camera/camera_info`, TF, 액션 `/servoing/align` | `/servoing/cmd_vel` |
| `mock_scene_node` (시험용) | `/servoing/cmd_vel` | `/perception/detections`, `/camera/depth/image_raw`, `/camera/camera_info`, TF, `/servoing/mock_object_in_base`, 서비스 `/servoing/mock_reset` |
| `mock_trials` (시험용) | 위 노드들 (`scene:=mock` 또는 `sim`) | 시험 결과 요약, CSV |
| `sim_scene_node` (Gazebo 시험용) | `/servoing/sim_boxes`, `/servoing/sim_base_pose` | `/perception/detections` (정답 박스), `/servoing/sim_object_in_base`, 서비스 `/servoing/sim_reset` |

`align_node` 파라미터: 제어 `k_v`, `k_w`, `max_v`, `max_w`, `bearing_gate`, `tol_range`, `tol_bearing`, `stop_ratio` / 판정 `lost_timeout`, `max_age`, `settle`, `required_frames` / 기타 `score_min`, `control_rate`, `feedback_rate`, `object_shapes`, `floor_z`, `track_gate_px`.

- `object_shapes`: 깊이는 bbox 중심이 닿은 물체 표면까지라서, 물체를 세워진 원기둥(`"클래스:반지름:높이"`, m)으로 보고 중심축 위치를 계산한다 (`object_model.py`). 옆에서 보면 반지름만큼, 위에서 보면 거의 0만큼 보정된다. 기본 block 0.0125/0.04, pin 0.010/0.05, hex_nut 0.015/0.015, bolt 0.006/0.06 (feat/yolo 치수). 모의 입력(`mock_align.launch.py`)은 깊이가 중심까지라 반지름 0으로 둔다. `floor_z`는 `base_link` 기준 바닥 높이 (-0.05).
- 같은 클래스가 여러 개면: 첫 프레임은 화면 가운데 세로줄에 가장 가까운 검출, 그 뒤로는 직전 박스에 가장 가까운 검출을 따라간다. 직전 박스에서 `track_gate_px`(150 px)보다 멀면 갈아타지 않고 놓친 것으로 본다 (`tracking.py`).

## 파일

| 파일 | 내용 |
|---|---|
| `visual_servoing/geometry.py` | 오차 계산: 깊이 처리, 역투영, 좌표 변환, 거리·방위 오차 |
| `visual_servoing/control.py` | P제어 법칙 |
| `visual_servoing/session.py` | 목표 하나의 진행 판단: 정지 후 완료 판정, 시간 초과, 물체 놓침 |
| `visual_servoing/align_node.py` | Align 액션 서버 |
| `visual_servoing/mock_model.py`, `mock_scene_node.py` | 모의 차체·카메라 |
| `visual_servoing/mock_trials.py` | 반복 시험 (모의 입력, Gazebo) |
| `visual_servoing/object_model.py` | 깊이로 잰 표면 점 → 물체 중심축 (원기둥 근사) |
| `visual_servoing/tracking.py` | 같은 클래스 여러 개일 때 목표 하나 따라가기 |
| `visual_servoing/sim_model.py`, `sim_scene_node.py` | Gazebo 시험 보조 (label → 클래스, 차체 위치 리셋) |
| `launch/sim_align.launch.py`, `worlds/align_sim.sdf` | Gazebo 시험 |
| `test/` | 단위 테스트, 모의 폐루프 테스트 |
