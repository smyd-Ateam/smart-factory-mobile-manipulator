# mm_task_manager

작업 상태머신: Nav2 이동 → 도착 확인 → 정렬(D) → 파지(B), 단계별 `/cmd_vel` 선택 (`docs/interfaces.md` 5.0)

> 작성 중. 지금은 `nav_only`(이동 + 도착 확인)까지 동작한다. Align/Grasp는 `project_interfaces` 생성 후 추가.

## 빌드

```bash
PYTHONNOUSERSITE=1 colcon build --packages-select mm_task_manager
```

`~/.local`에 setuptools를 따로 설치한 환경(예: ultralytics 설치)에서는 시스템 packaging과 충돌해 빌드가 실패하므로 `PYTHONNOUSERSITE=1`을 붙인다.

## 실행

```bash
# 실제 Nav2와 함께 (Nav2는 standalone:=false로 따로 실행)
ros2 launch mm_task_manager task_manager.launch.py mission:=<미션 YAML>

# mock 시나리오 (Gazebo · Nav2 없이)
ros2 launch mm_task_manager mock_nav_only.launch.py scenario:=s1_normal out_dir:=<결과 폴더>
```

mock 결과: `<out_dir>/<scenario>.jsonl` (상태머신 결과), `<out_dir>/<scenario>.csv` (`/cmd_vel` 기록)

| 시나리오 | 내용 | 기대 결과 |
|---|---|---|
| `s1_normal` | 정상 | SUCCEEDED, NAVIGATE 동안만 `/cmd_vel` 0.11 |
| `s2_fake_success` | SUCCEEDED인데 위치가 목표에서 1 m | `NAV_FAILED` / `arrival_check` |
| `s3_tf_stale` | 이동 중 `map→odom` 발행 중단 | `NAV_FAILED` / `tf_stale` |
| `s4_not_ready` | lifecycle 비활성 | `NAV_NOT_READY` / `lifecycle_inactive`, 목표 안 보냄 |
| `s5_timeout` | Nav 끝나지 않음 | 취소 → `NAV_FAILED` / `timeout` |
| `s6_cancel_no_response` | 취소 응답 없음 | `NAV_FAILED` / `timeout`, `aborted: true`, 이후 `/cmd_vel` 0 |
| `s7_cmd_stop` | 이동 중 `/nav2/cmd_vel` 끊김 | 0.5 s 안에 `/cmd_vel` 0, 이후 SUCCEEDED |
| `s8a_reject`, `s8b_aborted` | 목표 거부, ABORTED | `NAV_FAILED` / `rejected`, `aborted`, 재시도 없음 |
| `s9_clock_stop` | `/clock` 정지 | `UNKNOWN` / `clock_stopped`, `aborted: true`, 취소 요청 |

mock 노드: `mock_clock`(`/clock`), `mock_lifecycle`(`is_active`), `mock_tf`(TF), `mock_nav_server`(`/navigate_to_pose`), `cmd_recorder`(`/cmd_vel` CSV)

## 노드 `task_manager`

| 구분 | 이름 |
|---|---|
| 구독 | `/nav2/cmd_vel`, `/servoing/cmd_vel`, `/clock`, TF (`map→odom`, `map→base_link`) |
| 발행 | `/cmd_vel` (10 Hz, 선택한 입력 또는 0), `/perception/target_class` (`transient_local`) |
| 서비스 클라이언트 | `/lifecycle_manager_localization/is_active`, `/lifecycle_manager_navigation/is_active` |
| 액션 클라이언트 | `/navigate_to_pose` |

- 취소 요청이 거부(REJECTED)되면 취소 완료로 보지 않고 `cancel_timeout_sec`까지 기다린다 (목표가 계속 돌 수 있으므로).
- 종료(Ctrl-C) 시 `/cmd_vel` 0을 한 번 보낸다.

## 구조

| 파일 | 내용 |
|---|---|
| `mm_task_manager/task_manager_node.py` | ROS 노드 (액션 · 서비스 · TF · 타이머 · `/cmd_vel` 발행) |
| `mm_task_manager/run.py` | 노드 실행 공통 함수 (SIGINT 처리) |
| `mm_task_manager/mocks/` | 시험용 mock 노드 |
| `config/mock/` | mock 시나리오 파라미터 |
| `mm_task_manager/machine.py` | 상태머신 로직 (rclpy 없음). 이벤트를 받아 명령 목록을 돌려준다 |
| `mm_task_manager/geometry.py` | 도착 오차, TF 신선도 판정 |
| `mm_task_manager/cmd_select.py` | 상태별 `/cmd_vel` 입력 선택, 입력 끊김 처리 |
| `mm_task_manager/retry.py` | IK 실패 후 재정렬 판정, 시도별 `standoff_m` |
| `mm_task_manager/mission.py` | 미션 파라미터 읽기 · 검증 |
| `mm_task_manager/result_log.py` | 결과 기록 (JSON 한 줄씩) |
| `mm_task_manager/codes.py` | 실패 코드, 세부 원인 |
| `config/mission_test_room.yaml` | 미션 파라미터 (임시값, TODO 표시) |

## 테스트

```bash
cd src/mm_task_manager
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test
```

`~/.local`에 pytest를 따로 설치한 환경에서는 ROS의 `launch_testing` 플러그인과 충돌하므로 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`을 붙인다 (`visual_servoing` README 참고).

## 실패 코드

`docs/interfaces.md` 5.3 코드에 아래를 임시로 추가했다 (나중에 5장에 반영).

| 코드 | 의미 |
|---|---|
| `NAV_NOT_READY` | 제한 시간 안에 Nav2 활성화 · `map→odom` · `/navigate_to_pose` 서버가 준비되지 않음. 목표를 보내지 않음 |

`NAV_FAILED`의 세부 원인은 결과 기록의 `detail`에 남긴다: `arrival_check`, `tf_stale`, `timeout`, `aborted`, `rejected`, `canceled`

## 도착 확인과 가짜 성공

Nav2가 SUCCEEDED를 돌려줘도 `map→base_link`를 목표와 비교한다 (±0.15 m, ±0.35 rad). `/amcl_pose`는 움직여야 갱신되므로 쓰지 않는다.

- 가짜 성공을 실제로 잡는 것은 `arrival_check`(위치 비교)다.
- 지금까지 본 가짜 성공(`mm_navigation/README.md`)은 controller_server가 본 `map→odom`이 오래돼서 생긴 것으로 추정된다. 그때 상태머신이 받는 TF는 멀쩡할 수 있으므로 `tf_stale`(상태머신 쪽 TF 신선도 확인)만으로는 잡지 못할 수 있다. `tf_stale`은 보조 확인이다.
- AMCL은 `map→odom` 시각을 `transform_tolerance`(1.0 s)만큼 미래로 찍으므로, 미래 시각은 신선한 것으로 본다.

## 재시도

- 시작 실패, Nav 실패는 같은 실행에서 재시도하지 않는다.
- IK 실패(`IK_FAILED`)만 재정렬한다: 최대 2회, 시도마다 `standoff_list`의 다음 값 사용 (5.2)
- 시간 초과로 취소했는데 취소 응답이 `cancel_timeout_sec` 안에 안 오면 실행을 중단한다 (`aborted: true`). 그동안 `/cmd_vel`은 0이다.

## /clock 정지 감시

시간 제한은 sim time으로 잰다. Gazebo가 멈추면 sim time 타이머도 멈추므로, steady clock 타이머(wall time)로 `/clock` 수신을 감시하고 `clock_watchdog_sec` 넘게 안 오면 `UNKNOWN` / `clock_stopped`로 끝낸다. (Humble rclpy `create_timer(..., clock=Clock(clock_type=ClockType.STEADY_TIME))`가 `/clock` 없이도 동작하는 것 확인)

## TODO (팀 답변 대기, 지금은 파라미터로 둠)

| 항목 | 파라미터 | 담당 |
|---|---|---|
| 상태머신 쪽 Align 시간 초과 코드 | `align_sm_timeout_code` | 팀 |
| Align `timeout_sec` 값 | `align_timeout_sec` | D |
| Grasp 제한 시간 · 코드 | `grasp_timeout_sec`, `grasp_timeout_code` | B |
| `target_pose` 신선도 | `target_pose_max_age_sec` | C, B |
| 시도별 `standoff_m` | `standoff_list` (0.22, 0.28은 임시 자리값) | B |
| 관측 자세 담당 | `observe_pose_required` | B, C |
| Align/Grasp 액션 타입 | `project_interfaces` 생성 전에는 `nav_only: true` | B, D |
