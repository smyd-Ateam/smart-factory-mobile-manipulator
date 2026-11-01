# 공통 인터페이스 정의 (Interfaces)

> 이 문서는 팀 전체가 공유하는 **토픽 이름, 메시지 타입, 좌표계, 단위**의 기준이다.
> 코드와 이 문서가 다르면 **이 문서를 먼저 고치고(PR + 리뷰) 코드를 맞춘다.**

- 담당: A (공통 인터페이스 · 내비게이션)
- 상태: 초안 (v0.5)
- `TBD` = 아직 미정. 남은 항목은 6장 체크리스트 참고.
- 본 프로젝트는 **시뮬레이션(소프트웨어)만** 다룬다.

### 담당 구분 (문서 안의 A~D 표기)

| 표기 | 담당 | 브랜치 |
|---|---|---|
| A | 공통 인터페이스 · 내비게이션(Nav2) · 상태머신 | `docs/interfaces`, `feat/navigation` |
| B | 파지 (MoveIt2 · 그리퍼) | `feat/grasp` |
| C | 인식 (YOLO) | `feat/yolo` |
| D | Visual Servoing | `feat/visual-servoing` |

---

## 0. 개발 환경

| 항목 | 버전 |
|---|---|
| OS | Ubuntu 22.04 LTS |
| ROS2 | Humble |
| 시뮬레이터 | Gazebo Fortress (`ign gazebo`) |
| Python | 3.10 |
| 로봇팔 모델 | **DOFBOT (확정)** — `dofbot_original_pos.urdf` (1자 기본 자세 버전), 회전 관절 5개 + 그리퍼 서보 1개. 팀원 실물 구현 경험 있음 |
| 모바일 베이스 모델 | **TurtleBot3 Waffle (제안, 팀 확인 필요)**. 실물 작업은 책상 고정이었으나 본 프로젝트는 Nav2 이동이 필요 |

> 참고: 이전에 받은 `dofbot_initialposition.urdf`(ㄷ자 자세 버전)는 링크 간 간섭이 생기는 각도 범위가 설정돼 있어 **사용하지 않는다.**

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
| 깊이 이미지 | m (`32FC1`) — TBD: 카메라 플러그인 출력 형식 확인 |

- 메시지 안의 회전은 **쿼터니언**으로 표현한다. roll/pitch/yaw는 설정 파일·문서에서만 쓴다.

### 1.2 좌표축 방향

| 구분 | x | y | z |
|---|---|---|---|
| 로봇 본체 프레임 (REP 103) | 앞 | 왼쪽 | 위 |
| 카메라 optical 프레임 | 오른쪽 | 아래 | 앞(시선 방향) |

> ⚠️ 카메라 이미지 기반 계산(검출, visual servoing)은 **optical 프레임** 기준이다. 일반 카메라 프레임과 혼동 주의.

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
     └─ base_footprint        (TurtleBot3)
         └─ base_link         (TurtleBot3 차체)
             ├─ base_scan     (라이다, TurtleBot3 기본 이름 — 설치 후 확인)
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

> ⚠️ DOFBOT 원본 URDF는 시작 링크 이름이 `base_link`다. 모바일 베이스의 `base_link`와 충돌하므로 **`arm_base_link`로 이름을 바꿔서 사용한다.**
> `base_link → arm_base_link` 연결(팔 장착 위치)은 고정 관절(fixed)로 추가하며, 위치 값은 TBD.

### 2.2 프레임 목록

| 역할 | 이름 | 상태 | 발행 주체 |
|---|---|---|---|
| 전역 지도 | `map` | 확정 | Nav2 (AMCL) |
| 오도메트리 | `odom` | 확정 | 시뮬레이터 / 구동 컨트롤러 |
| 바닥 기준 | `base_footprint` | 확정 (TurtleBot3 기본) | robot_state_publisher |
| 로봇 기준 | `base_link` | 확정 (TurtleBot3 기본) | robot_state_publisher |
| 라이다 | `base_scan` | 제안 (TurtleBot3 기본 이름, 설치 후 확인) | robot_state_publisher |
| 카메라 | `camera_link` | 위치 확정 (`link5` 위쪽), 장착 오프셋 값 TBD | robot_state_publisher |
| 카메라 optical | `camera_optical_frame` | 이름 확정 | robot_state_publisher |
| 팔 베이스 | `arm_base_link` | 확정 (원본 `base_link`에서 변경) | robot_state_publisher |
| 팔 링크 | `link1` ~ `link5` | 확정 (DOFBOT URDF) | robot_state_publisher |
| 그리퍼 | `gripper_link` | 이름 제안, 모델 TBD (URDF에 없음, 추가 필요) | robot_state_publisher |
| 그리퍼 끝점 | `ee_link` | 이름 제안, 위치 TBD (URDF에 없음, 추가 필요) | robot_state_publisher |

> 실제 이름 확인 방법: 시뮬 실행 후 `ros2 run tf2_tools view_frames`

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
- [ ] 각 링크에 `<inertial>` (질량 · 관성) 추가
- [x] `base_link.STL` 확보 (팀원 제공, 크기 약 12 × 26 × 8 cm, 단위 m 확인)
- [ ] `ros2_control` / `<gazebo>` 태그 추가
- [ ] 그리퍼 링크 · 관절, `ee_link` 추가
- [ ] 카메라 링크 · 센서 플러그인 추가
- [ ] mesh 경로(`package://robot/...`)를 실제 패키지 이름에 맞게 수정

---

## 3. 토픽

| 토픽 | 메시지 타입 | 발행 | 구독 | frame_id | 비고 |
|---|---|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | 시뮬 | A | `base_scan` | 라이다 |
| `/odom` | `nav_msgs/msg/Odometry` | 시뮬 | A | `odom` | |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | A (Nav2) | 시뮬 | — | 모바일 베이스 속도 |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 시뮬 | C, D | `camera_optical_frame` | RGB 이미지. 시뮬 플러그인에서 이 이름으로 설정 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 시뮬 | C | `camera_optical_frame` | 깊이 이미지 (RGB-D 카메라) |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 시뮬 | C, D | `camera_optical_frame` | 카메라 내부 파라미터 |
| `/perception/detections` | `vision_msgs/msg/Detection2DArray` | C | D, A | `camera_optical_frame` | YOLO 결과 (px) |
| `/perception/target_pose` | `geometry_msgs/msg/PoseStamped` | C | A, D, B | `base_link` | 목표 물체 3D 위치 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 시뮬 / ros2_control | B, D | — | |
| `/servoing/cmd_twist` | `geometry_msgs/msg/TwistStamped` | D | 팔 제어 | `ee_link` 또는 `camera_optical_frame` (D 결정) | visual servoing 속도 명령. MoveIt Servo 사용 시 입력 토픽에 맞춰 리매핑 |

---

## 4. 액션 · 서비스

| 이름 | 타입 | 서버 | 클라이언트 | 비고 |
|---|---|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | A (Nav2) | 상태머신 | Nav2 표준 |
| `/servoing/align` | `project_interfaces/action/Align` (예정) | D | 상태머신 | 커스텀, 정의 TBD |
| `/grasp/execute` | `project_interfaces/action/Grasp` (예정) | B | 상태머신 | 커스텀, 정의 TBD |
| `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | 그리퍼 컨트롤러 | B | ros2_control 그리퍼 컨트롤러 이름을 `gripper_controller`로 설정 |

> 표준 메시지로 표현 가능한 건 표준을 쓴다. `project_interfaces` 패키지는 위 커스텀 액션이 확정될 때 만든다.

---

## 5. 모듈 간 합의 사항

### 5.1 정렬(D) → 파지(B) 전환 조건

- 이미지 중심 오차: `TBD` px 이하
- 물체까지 거리: `TBD` m 이하
- 위 조건이 `TBD` 초 유지되면 정렬 완료로 판단

### 5.2 실패 코드

실패 원인을 같은 형식으로 기록해 두면 나중에 성능 분석과 LLM 실패 원인 추정에 그대로 쓸 수 있다.

| 코드 | 의미 | 발생 모듈 |
|---|---|---|
| `NAV_FAILED` | 목표 지점 도착 실패 | A |
| `OBJECT_NOT_FOUND` | 물체 미검출 | C |
| `ALIGN_TIMEOUT` | 제한 시간 내 정렬 실패 | D |
| `GRASP_MISS` | 그리퍼가 물체를 못 잡음 | B |
| `GRASP_SLIP` | 잡은 뒤 놓침 | B |
| `UNKNOWN` | 기타 | — |

---

## 6. 미정 항목 체크리스트

- [x] 로봇팔 모델 → DOFBOT 확정 (2.3 반영)
- [ ] 모바일 베이스 모델 → TurtleBot3 Waffle 제안, 팀 확인 필요
- [ ] `base_scan` 프레임 이름 — TurtleBot3 설치 후 확인
- [ ] 팔 장착 위치 (`base_link → arm_base_link` 고정 관절 값)
- [x] 카메라 위치 → `link5` 위쪽 (eye-in-hand)
- [ ] 카메라 장착 오프셋 (`link5 → camera_link` xyz, rpy) — 실물 사진 기준으로 추정
- [x] 6번 서보(그리퍼) 각도 범위 → 180°
- [ ] 그리퍼 실제 벌림 폭(cm) · 그리퍼 모델 형태
- [x] `base_link.STL` → 팀원 제공
- [x] joint2, joint4 범위 → 1자 버전 URDF로 교체해서 해결
- [x] 시뮬 카메라 토픽 이름 → `/camera/...`로 확정
- [ ] 깊이 이미지 형식 (`32FC1` 여부) — 카메라 플러그인 실행 후 확인
- [ ] visual servoing 제어 방식 (MoveIt Servo 사용 여부)
- [ ] 커스텀 액션 `Align`, `Grasp`의 goal / feedback / result 정의
- [ ] 5.1 전환 조건 수치 (B, D 합의)
- [x] 그리퍼 컨트롤러 이름 → `gripper_controller`

---

## 7. 변경 규칙

1. 이 문서 수정은 PR로만 한다.
2. 토픽 이름 · 메시지 타입 · 프레임을 바꾸는 PR은 **해당 토픽을 쓰는 담당자 전원**의 리뷰를 받는다.
3. 아래 이력에 기록한다.

## 8. 변경 이력

| 날짜 | 버전 | 내용 | 작성자 |
|---|---|---|---|
| 2026-10-31 | v0.1 | 초안 작성 | A |
| 2026-10-31 | v0.2 | DOFBOT 팔 프레임 · 관절 반영 (2.1~2.4), 체크리스트 갱신 | A |
| 2026-10-31 | v0.3 | 팔 모델 DOFBOT 확정, 팀원 확인 항목 정리 | A |
| 2026-11-01 | v0.4 | 1자 버전 URDF로 교체, 카메라 위치(link5, eye-in-hand) · 그리퍼 서보 범위 반영 | A |
| 2026-11-01 | v0.5 | 모바일 베이스(TurtleBot3 Waffle) 제안, 카메라 · 정렬 · 그리퍼 토픽/액션 이름 확정 | A |
