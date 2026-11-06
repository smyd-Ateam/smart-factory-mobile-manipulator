# 공통 인터페이스 정의 (Interfaces)

> 이 문서는 팀 전체가 공유하는 토픽 이름, 메시지 타입, 좌표계, 단위의 기준이다.
> 코드와 이 문서가 다르면 이 문서를 먼저 고치고(PR + 리뷰) 코드를 맞춘다.

- 담당: A (공통 인터페이스 · 내비게이션 · 상태머신)
- 상태: 초안 (v0.11)
- `TBD` = 아직 미정. 남은 항목은 6장 체크리스트 참고.
- 본 프로젝트는 시뮬레이션(소프트웨어)만 다룬다.

### 담당 구분 (문서 안의 A~D 표기)

| 표기 | 담당 | 브랜치 |
|---|---|---|
| A | 공통 인터페이스 · 내비게이션(Nav2) · 상태머신 | `docs/interfaces`, `feat/navigation` |
| B | 파지 (MoveIt2 · 그리퍼) | `feat/grasp` |
| C | 인식 (YOLO) | `feat/yolo` |
| D | Visual Servoing (베이스 미세 정렬) | `feat/visual-servoing` |

---

## 0. 개발 환경

| 항목 | 버전 |
|---|---|
| OS | Ubuntu 22.04 LTS |
| ROS2 | Humble |
| 시뮬레이터 | Gazebo Fortress (`ign gazebo`) |
| Python | 3.10 |
| 로봇팔 모델 | DOFBOT (확정) — `dofbot_original_pos.urdf` (1자 기본 자세 버전), 회전 관절 5개 + 그리퍼 서보 1개. 팀원 실물 구현 경험 있음 |
| 모바일 베이스 모델 | 직접 제작한 차동구동 차체 (확정). 패키지 `mm_description` (`feat/navigation` 브랜치), 차체 40×30×10 cm · 바퀴 반지름 5 cm · 바퀴 간격 35 cm (임시값, 팔 장착 시 조정). TurtleBot3는 Humble에서 Gazebo Classic 전용이라 사용 불가. TurtleBot4는 센서가 많아 무거움: RTF Mac VM 약 10~20%, Windows PC 12.5% (PR #1 댓글 참고) |

> 참고: 이전에 받은 `dofbot_initialposition.urdf`(ㄷ자 자세 버전)는 링크 간 간섭이 생기는 각도 범위가 설정돼 있어 사용하지 않는다.

> 렌더링 참고 (환경별)
> - Mac UTM VM: 기본 엔진(ogre2)이 튕겨서 `--render-engine ogre` 사용
>   - GPU 가속(virgl) 상태에서는 라이다 거리값이 전부 최소값(0.12 m)으로 나옴 → 라이다를 쓰려면 CPU 렌더링(`software_gl:=true`) 필요 (2026-11-03 확인). 이때 Gazebo 화면을 켜면 RTF 약 71~76%, 화면 없이(`headless:=true`) 실행하고 RViz로 보면 약 99.7%
> - Windows + WSL2 (C 노트북): ogre2로 실행하면 Segmentation fault → `LIBGL_ALWAYS_SOFTWARE=1 ign gazebo ... --render-engine ogre`로 실행. 이 경우 카메라도 CPU로 렌더링돼서 RTF가 더 낮을 수 있음
>   - `mm_description` 테스트 (2026-11-03, C): `software_gl:=true`로 Gazebo 정상, RTF 평균 약 73% (Gazebo 화면 켠 상태), 라이다 값 정상. RViz는 실행 직후 꺼짐(exit code -11). 원인은 RViz를 GPU로 그렸기 때문으로 추정 → 기본값을 RViz도 CPU 렌더링으로 변경 (`rviz_gpu` 옵션 참고)
>   - 재테스트 (2026-11-05, C): RViz CPU 렌더링으로 바꾼 뒤 꺼지지 않고 정상 동작 확인. 화면 없이(`headless:=true`) 실행하면 RTF 평균 약 99% (5회, 98.7~99.7%). WSL 소프트웨어 렌더링에서는 Gazebo 화면이 속도를 가장 많이 잡아먹음
>   - 카메라 센서를 추가하면 화면을 꺼도 카메라 영상은 렌더링되므로, 카메라 추가 후 RTF 다시 측정 예정 (C)
> - Windows + WSL2 (B 노트북): MoveIt RViz도 `LIBGL_ALWAYS_SOFTWARE=1`을 붙여서 실행 (예: `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch dofbot_moveit_config demo.launch.py`, 2026-11-03 B 노션 기록)
> - 공용 launch 파일(`mm_description/launch/sim.launch.py`) 옵션: `software_gl:=true`(CPU 렌더링), `headless:=true`(Gazebo 화면 없이), `rviz:=false`(RViz 끄기), `rviz_gpu:=true`(RViz만 GPU로 그리기, Mac VM용), `render_engine`(기본 ogre)
>   - Mac UTM VM: `ros2 launch mm_description sim.launch.py software_gl:=true headless:=true rviz_gpu:=true`
>   - WSL2: `ros2 launch mm_description sim.launch.py software_gl:=true`

---

## 1. 공통 규칙

### 1.1 단위 (REP 103 준수)

| 물리량 | 단위 |
|---|---|
| 길이 · 위치 | m |
| 각도 | rad |
| 시간 | s |
| 선속도 / 각속도 | m/s, rad/s |
| 이미지 좌표 | px (좌상단 원점, u: 오른쪽, v: 아래) |
| 깊이 이미지 | m (`32FC1`) — TBD: 카메라 플러그인 출력 형식 확인 (C) |

- 깊이 이미지가 `32FC1`이면, 측정 범위를 벗어난 픽셀은 `inf`/`NaN`으로 나오므로 이 값은 계산에서 무시한다 (C 제안).

- 메시지 안의 회전은 쿼터니언으로 표현한다. roll/pitch/yaw는 설정 파일·문서에서만 쓴다.

### 1.2 좌표축 방향

| 구분 | x | y | z |
|---|---|---|---|
| 로봇 본체 프레임 (REP 103) | 앞 | 왼쪽 | 위 |
| 카메라 optical 프레임 | 오른쪽 | 아래 | 앞(시선 방향) |

> ⚠️ 카메라 이미지 기반 계산(검출, visual servoing)은 optical 프레임 기준이다. 일반 카메라 프레임과 혼동 주의.

### 1.3 시간

- 모든 노드는 시뮬레이션에서 `use_sim_time: true` 사용.
- 좌표가 있는 메시지는 `header.stamp`와 `header.frame_id`를 반드시 채운다.

### 1.4 이름 규칙

- 토픽 · 노드 이름: `snake_case`
- 모듈별 토픽은 네임스페이스로 구분: `/perception/...`, `/servoing/...`, `/grasp/...`
- 시뮬레이터 · Nav2 · MoveIt2 표준 토픽은 기본 이름 유지

---

## 2. 좌표계 (TF)

### 2.1 TF 트리 (REP 105 기반)

```
map
 └─ odom
     └─ base_footprint        (차체 바닥 기준)
         └─ base_link         (직접 제작 차체)
             ├─ base_scan     (라이다)
             └─ arm_base_link            ← DOFBOT 원본 URDF의 base_link
                 └─ link1     (joint1)
                     └─ link2     (joint2)
                         └─ link3     (joint3)
                             └─ link4     (joint4)
                                 └─ link5     (joint5, 손목)
                                     ├─ camera_link   (link5 위쪽에 장착, eye-in-hand)
                                     │   └─ camera_optical_frame
                                     └─ gripper_link  (그리퍼, 6번 서보)
                                         └─ ee_link   (그리퍼 끝점, TCP)
```

> ⚠️ DOFBOT 원본 URDF는 시작 링크 이름이 `base_link`다. 모바일 베이스의 `base_link`와 충돌하므로 `arm_base_link`로 이름을 바꿔서 사용한다.
> `base_link → arm_base_link` 연결(팔 장착 위치)은 고정 관절(fixed)로 추가하며, 위치 값은 TBD.

### 2.2 프레임 목록

| 역할 | 이름 | 상태 | 발행 주체 |
|---|---|---|---|
| 전역 지도 | `map` | 확정 | Nav2 (AMCL) |
| 오도메트리 | `odom` | 확정 | 시뮬레이터 / 구동 컨트롤러 |
| 바닥 기준 | `base_footprint` | 확정 (직접 제작 차체에서 이 이름 사용) | robot_state_publisher |
| 로봇 기준 | `base_link` | 확정 (직접 제작 차체에서 이 이름 사용) | robot_state_publisher |
| 라이다 | `base_scan` | 확정 (직접 제작 차체에서 이 이름 사용) | robot_state_publisher |
| 바퀴 · 캐스터 | `left_wheel_link`, `right_wheel_link`, `front_caster_link`, `rear_caster_link` | 확정 (`mm_description`) | robot_state_publisher |
| 카메라 | `camera_link` | 위치 확정 (`link5` 위쪽), 장착 오프셋 값 TBD | robot_state_publisher |
| 카메라 optical | `camera_optical_frame` | 이름 확정 | robot_state_publisher |
| 팔 베이스 | `arm_base_link` | 확정 (원본 `base_link`에서 변경) | robot_state_publisher |
| 팔 링크 | `link1` ~ `link5` | 확정 (DOFBOT URDF) | robot_state_publisher |
| 그리퍼 | `gripper_link` | 이름 제안, 모델 TBD (URDF에 없음, 추가 필요). 원본 URDF는 그리퍼 모양이 `link5` mesh(STL 1개)에 붙어 있어 따로 움직이지 않음 → STL 분리 또는 새 그리퍼 모델 필요 (B 확인, 2026-11-03) | robot_state_publisher |
| 그리퍼 끝점 | `ee_link` | 이름 제안, 위치 TBD (URDF에 없음, 추가 필요) | robot_state_publisher |

> 실제 이름 확인 방법: 시뮬 실행 후 `ros2 run tf2_tools view_frames`

> `map → odom` 발행 주체: SLAM 단계(지도 작성)에서는 slam_toolbox, 자율주행 단계에서는 AMCL이 발행한다.

### 2.3 팔 관절 (`dofbot_original_pos.urdf` 기준)

모두 회전 관절(revolute), 회전축은 각 관절 프레임의 z축. 실물 서보 번호와 1:1로 대응한다.

| 관절 | 서보 | 부모 → 자식 | 하한 (rad) | 상한 (rad) | 비고 |
|---|---|---|---|---|---|
| `joint1` | 1번 | `arm_base_link` → `link1` | -1.5708 | 1.5708 | 베이스 회전 |
| `joint2` | 2번 | `link1` → `link2` | -1.5708 | 1.5708 | |
| `joint3` | 3번 | `link2` → `link3` | -1.5708 | 1.5708 | |
| `joint4` | 4번 | `link3` → `link4` | -1.5708 | 1.5708 | |
| `joint5` | 5번 | `link4` → `link5` | -1.5708 | 3.1416 | 손목 회전 |
| `gripper_joint` (제안) | 6번 | `link5` → `gripper_link` | TBD | TBD | 원본 URDF에 없음. 서보 각도 범위 180° (0~180°로 추정, 확인 필요) |

- `/joint_states`의 관절 이름은 위 표의 이름을 그대로 사용한다.
- 실물 서보 각도(°)와 URDF 관절 각도(rad)는 기준점이 다를 수 있다. 실물 코드는 joint2에 -90°, joint4에 +90° 보정을 하고 있었으므로(ㄷ자 버전 기준), 시뮬 결과를 실물로 옮길 때는 변환식을 따로 확인한다.

### 2.4 시뮬레이션용 URDF 수정 필요 사항

원본 DOFBOT URDF를 Gazebo에서 쓰려면 아래 작업이 필요하다.

- [ ] `base_link` → `arm_base_link` 이름 변경
  - 현재 MoveIt 설정 패키지(`dofbot_moveit_config`)는 원본 이름 `base_link`로 생성돼 있음 (Self-Collisions 표 등). 이름을 바꾼 뒤 설정 패키지를 다시 생성해야 함
- [ ] 각 링크에 `<inertial>` (질량 · 관성) 추가
- [x] `base_link.STL` 확보 (팀원 제공, 크기 약 12 × 26 × 8 cm, 단위 m 확인)
- [ ] `ros2_control` / `<gazebo>` 태그 추가
  - 진행 (B, 2026-11-03): MoveIt Setup Assistant로 joint1~5 ros2_control 설정 생성 (명령 position, 상태 position · velocity). 현재는 fake_components(가짜 하드웨어)로 동작하고 Gazebo 연결은 아직
- [ ] 그리퍼 링크 · 관절, `ee_link` 추가
  - 원본 URDF는 그리퍼가 `link5` mesh에 포함돼 있어서 STL 분리 또는 새 그리퍼 모델이 먼저 필요. 이 작업이 끝나야 `gripper_joint`, `gripper_controller`, 벌림 폭 측정이 가능
- [ ] 카메라 링크 · 센서 플러그인 추가
- [ ] mesh 경로(`package://robot/...`)를 실제 패키지 이름에 맞게 수정

---

## 3. 토픽

| 토픽 | 메시지 타입 | 발행 | 구독 | frame_id | 비고 |
|---|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | 시뮬 | 전체 | — | 시뮬 시간 (`use_sim_time: true`) |
| `/scan` | `sensor_msgs/msg/LaserScan` | 시뮬 | A | `base_scan` | 라이다. 10 Hz, 360개(1° 간격), 0.12~8 m |
| `/odom` | `nav_msgs/msg/Odometry` | 시뮬 | A | `odom` | 차동구동 플러그인이 발행. `odom → base_footprint` TF도 함께 발행 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 상태머신(A)이 선택한 쪽 | 시뮬 (차체) | — | 차체 속도. 이동 중엔 Nav2, 정렬 중엔 D의 명령을 전달 (5.0 참고) |
| `/nav2/cmd_vel` | `geometry_msgs/msg/Twist` | A (Nav2: velocity_smoother, behavior_server) | 상태머신 | — | Nav2 최종 출력. velocity_smoother 출력(`cmd_vel_smoothed`)과 behavior_server 출력(`cmd_vel`)을 이 이름으로 리매핑 (3.1 참고) |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 시뮬 | C, D | `camera_optical_frame` | RGB 이미지. 시뮬 플러그인에서 이 이름으로 설정 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 시뮬 | C, D | `camera_optical_frame` | 깊이 이미지 (RGB-D 카메라). D는 정렬 거리 계산에 사용 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 시뮬 | C, D | `camera_optical_frame` | 카메라 내부 파라미터 |
| `/perception/detections` | `vision_msgs/msg/Detection2DArray` | C | D, A | `camera_optical_frame` | YOLO 결과 (px) |
| `/perception/target_pose` | `geometry_msgs/msg/PoseStamped` | C | A, D, B | `base_link` | 목표 물체 3D 위치 (3.3 참고) |
| `/joint_states` | `sensor_msgs/msg/JointState` | 시뮬 / ros2_control | B, D | — | |
| `/servoing/cmd_vel` | `geometry_msgs/msg/Twist` | D | 상태머신 | — | 베이스 미세 정렬용 차체 속도 명령 (팔은 고정) |
| `/perception/target_class` | `std_msgs/msg/String` | 상태머신(A) | C | — | 이번에 잡을 물체 클래스 이름. QoS `transient_local` (나중에 켜진 노드도 마지막 값 수신) |

### 3.1 Nav2 속도 명령 흐름 (Humble `nav2_bringup` 확인)

```
controller_server → cmd_vel_nav → velocity_smoother → (기본) /cmd_vel
                                                   → (우리) /nav2/cmd_vel → 상태머신 → /cmd_vel
behavior_server (복구 동작)                        → (기본) /cmd_vel
                                                   → (우리) /nav2/cmd_vel → 상태머신 → /cmd_vel
```

- 기본 `navigation_launch.py`는 controller_server 출력을 `cmd_vel_nav`로, velocity_smoother 출력(`cmd_vel_smoothed`)을 `cmd_vel`로 리매핑한다.
- 우리 launch 파일에서는 velocity_smoother와 behavior_server 출력을 `/nav2/cmd_vel`로 바꾼다. controller_server 쪽 리매핑은 그대로 둔다.
- behavior_server를 같이 바꾸는 이유: 기본 설정에서는 복구 동작(spin, backup 등)의 속도 명령이 velocity_smoother를 거치지 않고 `/cmd_vel`로 바로 나가는데, 이렇게 상태머신을 건너뛰면 5.0 규칙이 깨진다.
- `standalone:=true` 옵션 (`mm_navigation/launch/nav2.launch.py`, 기본 `false`): 위 두 출력을 `/nav2/cmd_vel` 대신 `/cmd_vel`로 바로 보낸다. 상태머신 없이 테스트할 때만 쓴다. 기본값에서는 상태머신이 없으면 차체가 움직이지 않는다.
- 확인 (2026-11-03): velocity_smoother를 단독 실행하고 `-r cmd_vel:=cmd_vel_nav -r cmd_vel_smoothed:=/nav2/cmd_vel`로 리매핑한 뒤 configure → `/nav2/cmd_vel` Publisher 1, `/cmd_vel_nav` Subscription 1 확인

### 3.2 `/perception/detections` 필드 규칙 (C 제안)

| 필드 | 규칙 |
|---|---|
| `results[0].hypothesis.class_id` | 클래스 이름 문자열 (예: `bolt`) |
| `results[0].hypothesis.score` | 0 ~ 1 |
| `bbox.center.position.x/y`, `bbox.size_x/size_y` | px |
| `header.stamp` | 처리 시각이 아니라 원본 이미지의 stamp를 그대로 복사 |

- 물체가 안 보이는 프레임에도 빈 배열을 발행한다. 받는 쪽이 "검출 없음"과 "노드 멈춤"을 구분할 수 있게 하기 위함.

### 3.3 `/perception/target_pose` 규칙 (C 제안)

- orientation은 당분간 위치만 채우고 단위 쿼터니언(회전 없음)으로 둔다. B가 물체 방향(yaw)이 필요하면 C에게 요청.
- 카메라가 팔(`link5`)에 달려 있으므로, `camera_optical_frame` → `base_link` 변환은 이미지 stamp 시점의 TF로 계산한다.
- C는 `/perception/target_class`로 받은 클래스의 물체만 `target_pose`로 발행한다. 클래스 이름은 `Align`의 `target_class`와 같은 문자열을 쓴다.

---

## 4. 액션 · 서비스

| 이름 | 타입 | 서버 | 클라이언트 | 비고 |
|---|---|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | A (Nav2) | 상태머신 | Nav2 표준 |
| `/servoing/align` | `project_interfaces/action/Align` (예정) | D | 상태머신 | 베이스 미세 정렬 (팔 고정). 필드는 4.1 참고 |
| `/grasp/execute` | `project_interfaces/action/Grasp` (예정) | B | 상태머신 | IK 검증 및 로봇팔 이동·파지. 필드는 4.2 참고 |
| `/dofbot_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 팔 컨트롤러 | B (MoveIt) | joint1~5, `joint_trajectory_controller/JointTrajectoryController`. 액션 이름은 컨트롤러 기본 규칙으로 적음 (B 환경에서 `ros2 action list`로 확인 필요) |
| `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | 그리퍼 컨트롤러 | B | ros2_control 그리퍼 컨트롤러 이름을 `gripper_controller`로 설정 |

> 표준 메시지로 표현 가능한 건 표준을 쓴다. `project_interfaces` 패키지는 위 커스텀 액션이 확정될 때 만든다.

### 4.1 `Align` 액션 (D 제안)

```
# Goal
string target_class        # 정렬할 물체 종류 (YOLO 클래스 이름)
float64 standoff_m         # 물체 앞에서 멈출 거리 (m)
float64 timeout_sec        # 제한 시간 (s)
---
# Result
bool success
string error_code          # 실패 시 5.3 실패 코드 이름, 성공 시 빈 문자열
float64 final_range_error_m
float64 final_bearing_error_rad
---
# Feedback
bool target_visible
float64 range_error_m
float64 bearing_error_rad
```

- 거리·방위 오차의 기준 좌표계: `base_link` (D 확정)
- `standoff_m`: `base_link` 원점에서 물체까지의 거리
- `error_code`: 5.3 표의 이름을 그대로 사용. 정렬 중 물체를 놓친 경우에도 `OBJECT_NOT_FOUND` 반환

### 4.2 `Grasp` 액션 (B 확정)

```
# Goal
string target_class                      # 잡을 물체 종류 (YOLO 클래스 이름, Align과 같은 이름)
geometry_msgs/PoseStamped target_pose    # 물체 위치 (C의 /perception/target_pose, base_link 기준)
---
# Result
bool success
string error_code                        # 실패 시 5.3 실패 코드 이름 (IK_FAILED, GRASP_MISS, GRASP_SLIP 등)
---
# Feedback
string phase                             # 현재 단계 (IK_CHECK, APPROACH, GRIP 등)
```

- 파지 목표 자세는 B가 `target_pose`(C 제공)를 바탕으로 계산하고, MoveIt2로 IK와 경로 계획을 수행한다.
- 처음 제안의 `target_id`는 `target_class`로 바꿨다. 같은 클래스 물체가 여러 개일 때 개별 물체를 구분하는 `target_id`를 추가할지는 추후 논의.
- 처음에는 고정된 접근 방향으로 파지한다. 물체 방향(yaw)이 필요한지는 물체 형상·크기 확정 후 B가 검토.
- IK 실패 시 B가 `IK_FAILED`를 반환하고, 상태머신이 `/servoing/align`을 다시 호출한다

---

## 5. 모듈 간 합의 사항

### 5.0 작업 흐름과 차체 명령 선택

정렬은 팔을 고정하고 차체만 움직이는 베이스 미세 정렬 방식이다. MoveIt Servo는 사용하지 않는다.

```
Nav2로 물체 근처까지 이동 → 베이스 미세 정렬(D) → IK 확인(B) → 파지(B)
                                  ↑                    │
                                  └── IK 실패 시 재정렬 (최대 2회, 5.2) ┘
```

- 차체 속도 명령은 Nav2와 정렬(D) 두 곳에서 나오므로, 상태머신(A)이 현재 단계에 맞는 명령만 `/cmd_vel`로 전달한다.
- Nav2와 정렬(D)이 동시에 차체를 제어하지 않는다 (B 의견).

| 상태머신 단계 | `/cmd_vel`로 전달하는 명령 |
|---|---|
| 이동 (Nav2) | `/nav2/cmd_vel` |
| 정렬 (D) | `/servoing/cmd_vel` |
| 그 외 (IK 확인, 파지, 대기 등) | 속도 0 |

- twist_mux 같은 별도 패키지는 쓰지 않고 상태머신 노드 안에서 직접 선택한다.

### 5.1 정렬(D) → 파지(B) 전환 조건 (D 제안 초안)

- 방위 오차: 0.05 rad 이하
- 거리 오차: 0.02 m 이하
- 차체 정지 후 촬영한 프레임 3장 연속으로 위 조건을 만족하면 정렬 완료
- 이후 B의 IK 결과로 실제로 잡을 수 있는 자세인지 확인하고, IK가 풀리지 않으면 다시 정렬한다 (B, D 의견 반영)
- 임계값은 그리퍼 크기 확정 후 B, D가 함께 조정
- 거리 오차 조건은 C의 bbox 흔들림과 깊이 추정 오차에 영향을 받는다. C가 첫 학습 후 정지 상태에서 위치 추정값의 흔들림 폭을 측정해 공유하고, 그 값을 보고 함께 조정한다

### 5.2 IK 실패 후 재정렬 (D 제안 반영)

- 같은 `standoff_m`으로 재정렬하면 같은 위치로 돌아와 반복 실패할 수 있으므로, 재시도할 때마다 `standoff_m`을 바꾼다.
- 재시도는 최대 2회 (처음 시도 포함 총 3회).
- 3회 모두 실패하면 해당 물체는 `IK_FAILED`로 최종 실패 처리한다.
- 재정렬은 상태머신(A)이 `Align` Goal의 `standoff_m`만 바꿔서 `/servoing/align`을 다시 호출하는 방식이다.
- 시도별 `standoff_m` 값은 TBD: DOFBOT 팔이 닿는 범위를 B가 확인한 뒤 정한다.

### 5.3 실패 코드

실패 원인을 같은 형식으로 기록해 두면 나중에 성능 분석과 LLM 실패 원인 추정에 그대로 쓸 수 있다.

| 코드 | 의미 | 발생 모듈 |
|---|---|---|
| `NAV_FAILED` | 목표 지점 도착 실패 | A |
| `OBJECT_NOT_FOUND` | 물체 미검출 (정렬 중 물체를 놓친 경우 포함) | C, D |
| `ALIGN_TIMEOUT` | 제한 시간 내 정렬 실패 | D |
| `IK_FAILED` | 정렬 후 IK가 풀리지 않음 (재정렬 대상) | B |
| `GRASP_MISS` | 그리퍼가 물체를 못 잡음 | B |
| `GRASP_SLIP` | 잡은 뒤 놓침 | B |
| `UNKNOWN` | 기타 | — |

- `OBJECT_NOT_FOUND` 판정 기준 (C 제안): score 0.5 이상인 목표 클래스 검출이 연속 N프레임(예: 10프레임) 동안 없으면 발생. 수치는 첫 모델 학습 후 조정
- B는 `IK_FAILED`, `GRASP_MISS`, `GRASP_SLIP`을 사용한다 (`GRASP_FAILED`는 따로 만들지 않음, B 확인)

### 5.4 파지 대상 물체 (2026-11-03 회의)

단계별로 진행한다. 앞 단계가 끝까지 성공하면 다음 단계 물체를 추가한다.

| 단계 | 클래스 이름 | 물체 | 크기 | 모델 |
|---|---|---|---|---|
| 1단계 | `block` | 사각 블록 (직육면체) | 2.5 × 2.5 × 4 cm | SDF 기본 도형 (box) |
| 1단계 | `pin` | 원통형 핀 (롤러) | 지름 2 cm, 길이 5 cm | SDF 기본 도형 (cylinder) |
| 2단계 | `hex_nut` | 육각 너트 | 폭 3 cm, 두께 1.5 cm | CAD로 제작 |
| 3단계 | `bolt` | 육각 볼트 | 머리 폭 2.5 cm, 길이 6 cm | CAD 또는 무료 모델 |

- 크기는 그리퍼 최대 벌림 폭(B 확인 중)보다 작아야 한다. hex_nut은 꼭짓점끼리 약 3.5 cm라 잡는 방향에 주의.
- 학습 데이터는 Gazebo 합성 데이터를 기본으로 하고, 실제 사진 데이터셋(Roboflow, CC BY 4.0)을 보조로 섞는다 (C). 클래스 이름은 위 이름으로 통일.

---

## 6. 미정 항목 체크리스트

- [x] 로봇팔 모델 → DOFBOT 확정 (2.3 반영)
- [x] 모바일 베이스 모델 → 직접 제작 차동구동 차체
- [x] `base_scan` 프레임 이름 → 직접 제작 차체에서 이 이름 사용
- [ ] 팔 장착 위치 (`base_link → arm_base_link` 고정 관절 값)
- [x] 카메라 위치 → `link5` 위쪽 (eye-in-hand)
- [ ] 카메라 장착 오프셋 (`link5 → camera_link` xyz, rpy) — 실물 사진 기준으로 추정
- [x] 6번 서보(그리퍼) 각도 범위 → 180°
- [ ] 그리퍼 실제 벌림 폭(cm) · 그리퍼 모델 형태
- [x] `base_link.STL` → 팀원 제공
- [x] joint2, joint4 범위 → 1자 버전 URDF로 교체해서 해결
- [x] 시뮬 카메라 토픽 이름 → `/camera/...`로 확정
- [ ] 깊이 이미지 형식 (`32FC1` 여부) — 카메라 플러그인 실행 후 확인 (C)
- [x] visual servoing 제어 방식 → 베이스 미세 정렬, MoveIt Servo 미사용
- [x] `Align` 액션 필드 → D 제안 반영 (4.1)
- [x] `Align` 오차 기준 좌표계 → `base_link` (D)
- [x] 파지 목표 자세 생성 → B가 C의 `target_pose`로 계산 (4.2)
- [x] 5.1 전환 조건 → D 제안 초안 반영, 그리퍼 확정 후 조정
- [x] 차체 명령 선택 방식 → 상태머신이 단계별로 직접 선택 (5.0)
- [x] IK 실패 후 재정렬 방식 → 최대 2회 재시도, 시도마다 `standoff_m` 변경 (5.2)
- [ ] 재시도별 `standoff_m` 값 (B) — 팔이 닿는 범위 확인 후
- [x] `Grasp` 필드 타입 · `target_class` 사용 · 실패 코드 → B 확정 (4.2, 5.3)
- [ ] 같은 클래스 물체가 여러 개일 때 구분 방법 (`target_id` 추가 여부, A·B·C)
- [ ] 물체 방향(yaw)이 파지에 필요한지 (B) — 처음엔 고정 접근 방향, 물체 확정 후 검토. 필요하면 C가 `target_pose` orientation 채움
- [x] 그리퍼 컨트롤러 이름 → `gripper_controller`
- [x] 파지 대상 물체 · YOLO 클래스 이름 → `block`, `pin`, `hex_nut`, `bolt` (5.4). `Align` · `Grasp`의 `target_class`와 같은 이름 사용
- [ ] DOFBOT 그리퍼 최대 벌림 폭 · 들 수 있는 무게 (B) — 물체 크기 확정 전 확인
- [ ] 물체 놓는 방법 (세움/눕힘, 바닥/작업대 높이) (A, B, D)
- [x] 목표 클래스 지정 방식 → `/perception/target_class` (`std_msgs/msg/String`, `transient_local`)
- [ ] 관측 자세(observe pose) 관절 값 (B, C) — 이동·정렬 중 팔 고정 자세. YOLO 학습용 합성 데이터 시점 기준. MoveIt 자세 이름 `observe_pose`로 추가 예정 (B)
- [x] MoveIt 계획 그룹 → `dofbot_arm` (joint1~5, KDL) (B)
- [x] MoveIt 기본 자세 → `home` (관절 전부 0, 1자 자세) (B)
- [ ] 검출 발행 주기 Hz (C) — WSL 환경 CPU 추론 속도 측정 후 결정
- [x] 공용 launch 파일에 렌더러 옵션 인자 추가 (A) → `software_gl`, `headless`, `rviz_gpu`, `render_engine` (0장)
- [ ] 카메라 해상도 (C) — 320×240(현재 테스트 값)과 640×480을 첫 학습 때 비교, RTF 영향도 함께 확인

---

## 7. 변경 규칙

1. 이 문서 수정은 PR로만 한다.
2. 토픽 이름 · 메시지 타입 · 프레임을 바꾸는 PR은 해당 토픽을 쓰는 담당자 전원의 리뷰를 받는다.
3. 아래 이력에 기록한다.

## 8. 변경 이력

| 날짜 | 버전 | 내용 | 작성자 |
|---|---|---|---|
| 2026-10-31 | v0.1 | 초안 작성 | A |
| 2026-10-31 | v0.2 | DOFBOT 팔 프레임 · 관절 반영 (2.1~2.4), 체크리스트 갱신 | A |
| 2026-10-31 | v0.3 | 팔 모델 DOFBOT 확정, 팀원 확인 항목 정리 | A |
| 2026-11-01 | v0.4 | 1자 버전 URDF로 교체, 카메라 위치(link5, eye-in-hand) · 그리퍼 서보 범위 반영 | A |
| 2026-11-01 | v0.5 | 모바일 베이스(TurtleBot3 Waffle) 제안, 카메라 · 정렬 · 그리퍼 토픽/액션 이름 확정 | A |
| 2026-11-02 | v0.6 | PR #1 리뷰 반영: 정렬을 베이스 미세 정렬로 변경, `/servoing/cmd_vel`, `Align` 액션 필드, 5.1 조건, IK 확인 단계, 차체 직접 제작(B, D 동의, Windows RTF 근거 추가) | A |
| 2026-11-03 | v0.7 | PR #1 2차 리뷰 반영: `Align` 기준 좌표계(`base_link`) · `standoff_m` · `error_code` 규칙(D), `Grasp` 필드 초안과 액션 비고(B), 검출 · 목표 위치 · 깊이 규칙과 `OBJECT_NOT_FOUND` 기준(C), 렌더링 참고(Mac VM 라이다는 CPU 렌더링 필요), launch 옵션, 차체 패키지 정보(0장 · 2.2 · 3장), 체크리스트 추가 / 차체 명령 선택 방식, IK 실패 후 재정렬 규칙(5.2), `/perception/target_class` 토픽 결정, Nav2 속도 토픽 흐름 확인 및 리매핑 테스트(3.1) | A |
| 2026-11-03 | v0.8 | Grasp 액션 필드 확정(B), `target_id` → `target_class`, 실패 코드 정리, 파지 대상 물체 · 클래스 이름(5.4), WSL 테스트 결과와 RViz 렌더링 옵션(`rviz_gpu`), 체크리스트 갱신 | A |
| 2026-11-04 | v0.9 | B의 MoveIt 설정 기록 반영: 그리퍼가 `link5` mesh에 포함된 문제(2.2 표, 2.4), MoveIt 설정의 `base_link` 이름 재생성 필요(2.4), ros2_control 진행 상황(2.4), WSL MoveIt RViz 렌더링(0장), 팔 컨트롤러 액션(4장), MoveIt 그룹 · 자세 이름(6장) | A |
| 2026-11-07 | v0.10 | C의 WSL 재테스트 결과 반영(0장): RViz CPU 렌더링 후 정상 동작, headless RTF 약 99%, 카메라 추가 후 재측정 예정 | A |
| 2026-11-07 | v0.11 | `mm_navigation` 구현 반영: behavior_server 출력도 `/nav2/cmd_vel`로 리매핑(3장 표, 3.1), `standalone:=true` 옵션 설명(3.1), 단계별 `map → odom` 발행 주체(2장) | A |
