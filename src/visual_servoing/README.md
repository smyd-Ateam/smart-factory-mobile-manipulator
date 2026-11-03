# visual_servoing

카메라 피드백으로 모바일 베이스를 미세 정렬하는 패키지 (D). 설계는 `docs/visual_servoing_design.md`.

## 빌드

레포 폴더에서

```bash
colcon build --packages-select visual_servoing
source install/setup.bash
```

## 모의 입력으로 실행

Gazebo 카메라, YOLO 없이 모의 차체 + 모의 카메라로 정렬을 확인한다.

```bash
ros2 launch visual_servoing mock_align.launch.py
# 노이즈, 지연 추가
ros2 launch visual_servoing mock_align.launch.py delay_sec:=0.5 noise_px:=2.0 noise_depth:=0.005
```

`[실제]` 로그가 모의 차체의 실제 오차, `servoing_align` 로그가 정렬 노드가 계산한 오차다. 둘 다 0 근처에서 멈추면 정상.

## 단위 테스트

```bash
cd src/visual_servoing
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test
```

`~/.local`에 pytest를 따로 설치한 환경에서는 ROS의 `launch_testing` 플러그인과 충돌해서 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`이 필요하다.

## 노드

| 노드 | 입력 | 출력 |
|---|---|---|
| `align_node` | `/perception/detections`, `/camera/depth/image_raw`, `/camera/camera_info`, TF | `/servoing/cmd_vel` |
| `mock_scene_node` (시험용) | `/servoing/cmd_vel` | `/perception/detections`, `/camera/depth/image_raw`, `/camera/camera_info`, TF `base_link → camera_optical_frame` |

`align_node` 주요 파라미터: `target_class`, `standoff_m`, `enabled`, `k_v`, `k_w`, `max_v`, `max_w`, `tol_range`, `tol_bearing`, `stop_ratio`, `max_measurement_age`.
Align 액션 서버(5주차)가 생기면 `target_class`, `standoff_m`은 Goal로 받는다.

## 파일

| 파일 | 내용 |
|---|---|
| `visual_servoing/geometry.py` | 오차 계산: 깊이 처리, 역투영, 좌표 변환, 거리·방위 오차 (2주차) |
| `visual_servoing/control.py` | P제어 법칙 (3주차) |
| `visual_servoing/align_node.py` | 정렬 노드 |
| `visual_servoing/mock_model.py`, `mock_scene_node.py` | 모의 차체·카메라 |
| `test/` | 단위 테스트, 모의 폐루프 테스트 |
