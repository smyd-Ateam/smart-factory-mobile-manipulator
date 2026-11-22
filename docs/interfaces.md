# 공통 인터페이스 정의 (Interfaces)

> 이 문서는 팀 전체가 공유하는 토픽 이름, 메시지 타입, 좌표계, 단위의 기준이다.
> 코드와 이 문서가 다르면 이 문서를 먼저 고치고(PR + 리뷰) 코드를 맞춘다.

- 담당: A (공통 인터페이스 · 내비게이션 · 상태머신)
- 상태: 초안 (v0.27)
- `TBD` = 아직 미정. 남은 항목은 6장 체크리스트 참고.
- 본 프로젝트는 시뮬레이션(소프트웨어)만 다룬다.

### 담당 구분 (문서 안의 A~D 표기)

| 표기 | 담당 | 브랜치 |
|---|---|---|
| A | 공통 인터페이스 · 내비게이션(Nav2) · 상태머신 | `docs/interfaces`, `feat/navigation`, `feat/state-machine` (`feat/navigation`에서 분기) |
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
| 로봇팔 모델 | DOFBOT (확정) — 그리퍼 분리형 모델 (패키지 `dofbot_description`, `feat/grasp`, 2026-11-08 공식 모델로 확정). 회전 관절 5개 + 그리퍼 관절 1개(좌우 집게 mimic). 출처: GitHub DofBot-PickNPlace 프로젝트 (MIT License). 팀원 실물 구현 경험 있음 |
| 모바일 베이스 모델 | 직접 제작한 차동구동 차체 (확정). 패키지 `mm_description` (`feat/navigation` 브랜치), 차체 40×30×10 cm · 바퀴 반지름 5 cm · 바퀴 간격 35 cm (임시값, 팔 장착 시 조정) (PR #4에서 0.38×0.28×0.06m로 변경 제안 중, 아직 머지 전 — 머지되면 이 값도 갱신 필요). TurtleBot3는 Humble에서 Gazebo Classic 전용이라 사용 불가. TurtleBot4는 센서가 많아 무거움: RTF Mac VM 약 10~20%, Windows PC 12.5% (PR #1 댓글 참고) |

> 참고: 이전에 받은 `dofbot_initialposition.urdf`(ㄷ자 자세 버전)는 링크 간 간섭이 생기는 각도 범위가 설정돼 있어 사용하지 않는다.

> 진행 (2026-11-07, B): 그리퍼가 분리된 DOFBOT 모델(GitHub DofBot-PickNPlace 프로젝트)로 전환. 패키지 `dofbot_description`, `dofbot_moveit_config` (`feat/grasp`). Xacro 여러 파일 구조이고, 링크 · 관절 이름은 팀 규칙(`arm_base_link`, `link1`~`link5`, `joint1`~`joint5`)으로 변경. STL 파일 이름은 원본 유지. 기존 `dofbot_original_pos.urdf`와 관절 좌표계 · 회전축이 달라서, 2026-11-08 이 모델을 공식 모델로 확정하고 TF 트리(2.1) · 관절 표(2.3)를 이 모델 기준으로 갱신. 기존 `dofbot_original_pos.urdf`는 더 이상 쓰지 않는다

> 렌더링 참고 (환경별)
> - Mac UTM VM: 기본 엔진(ogre2)이 튕겨서 `--render-engine ogre` 사용
>   - GPU 가속(virgl) 상태에서는 라이다 거리값이 전부 최소값(0.12 m)으로 나옴 → 라이다를 쓰려면 CPU 렌더링(`software_gl:=true`) 필요 (2026-11-03 확인). 이때 Gazebo 화면을 켜면 RTF 약 71~76%, 화면 없이(`headless:=true`) 실행하고 RViz로 보면 약 99.7%
> - Windows + WSL2 (C 노트북): ogre2로 실행하면 Segmentation fault → `LIBGL_ALWAYS_SOFTWARE=1 ign gazebo ... --render-engine ogre`로 실행. 이 경우 카메라도 CPU로 렌더링돼서 RTF가 더 낮을 수 있음
>   - `mm_description` 테스트 (2026-11-03, C): `software_gl:=true`로 Gazebo 정상, RTF 평균 약 73% (Gazebo 화면 켠 상태), 라이다 값 정상. RViz는 실행 직후 꺼짐(exit code -11). 원인은 RViz를 GPU로 그렸기 때문으로 추정 → 기본값을 RViz도 CPU 렌더링으로 변경 (`rviz_gpu` 옵션 참고)
>   - 재테스트 (2026-11-05, C): RViz CPU 렌더링으로 바꾼 뒤 꺼지지 않고 정상 동작 확인. 화면 없이(`headless:=true`) 실행하면 RTF 평균 약 99% (5회, 98.7~99.7%). WSL 소프트웨어 렌더링에서는 Gazebo 화면이 속도를 가장 많이 잡아먹음
>   - 카메라 센서를 추가하면 화면을 꺼도 카메라 영상은 렌더링되므로, 카메라 추가 후 RTF 다시 측정 예정 (C)
> - Windows + WSL2 (B 노트북): MoveIt RViz도 `LIBGL_ALWAYS_SOFTWARE=1`을 붙여서 실행 (예: `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch dofbot_moveit_config demo.launch.py`, 2026-11-03 B 노션 기록)
> - RGB-D 카메라 렌더링 (B, 2026-11-21, `feat/grasp` 3161f10·b9e8a55): `ogre`에서는 RGB-D 컬러 영상이 회색으로 나오고, `render_engine:=ogre2`로 실행하면 컬러·깊이 모두 정상 (B WSL 환경에서 확인). launch 인자 `render_engine` 기본값은 환경별 선택을 위해 `ogre`로 유지
> - Mac UTM VM 재시험 (A, 2026-11-21, `feat/grasp` b9e8a55): `render_engine:=ogre2 software_gl:=true headless:=true`로 실행하면 `test_room`·`yolo_test_room` 둘 다 깊이·RGB·YOLO 정상 (부품 깊이 오차 약 3~10 mm, 라이다도 정상). `yolo_test_room.sdf`의 센서 plugin 설정은 아직 `ogre`지만, 실행 인자가 이 설정을 덮어씀 — 인자를 생략하면(기본값 ogre) 두 월드 모두 깊이 100% inf · RGB 회색. 단 ogre2+`software_gl:=true` 조합에서도 로그에 `libEGL warning: Not allowed to force software rendering when API explicitly selects a hardware device`가 뜨고 실제로는 GPU로 렌더링되는 것으로 보이며, RTF 하위 5%가 기존 ogre+software 조합(0.839)보다 낮음(0.68~0.75) — 성능 저하는 확인됐으나 원인은 추정 단계
> - 공용 launch 파일(`mm_description/launch/sim.launch.py`) 옵션: `software_gl:=true`(CPU 렌더링), `headless:=true`(Gazebo 화면 없이), `rviz:=false`(RViz 끄기), `rviz_gpu:=true`(RViz만 GPU로 그리기, Mac VM용), `render_engine`(기본 ogre)
>   - Mac UTM VM: `ros2 launch mm_description sim.launch.py software_gl:=true headless:=true rviz_gpu:=true`
>   - WSL2: `ros2 launch mm_description sim.launch.py software_gl:=true`
> - Windows + WSL2: project_interfaces 빌드 시 (C, 2026-11-14): `PYTHONNOUSERSITE=1` 없이 `colcon build`하면 `TypeError: canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'` 발생. ultralytics/PyTorch를 `pip install --user`로 설치하면 최신 setuptools가 `~/.local`에 깔리는데, 이게 Ubuntu 22.04 기본(구버전) packaging과 충돌하기 때문. pip로 AI 패키지를 설치한 경우 전부 해당될 수 있음. 빌드 시 `PYTHONNOUSERSITE=1 colcon build`로 실행.

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
| 깊이 이미지 | m (`32FC1`) — D 확인(2026-11-19), B 통합 모델 RGB-D 카메라에서 확인(2026-11-21, WSL) |

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
             └─ arm_base_link            ← 분리형 DOFBOT 모델의 팔 베이스
                 └─ link1     (joint1)
                     └─ link2     (joint2)
                         └─ link3     (joint3)
                             └─ link4     (joint4)
                                 ├─ DaBai_DCW2_Link   (카메라 본체. link4에 고정, 2026-11-09 확정)
                                 │   └─ camera_link   (DaBai_DCW2_Link와 같은 위치 · 방향)
                                 │       └─ camera_optical_frame
                                 └─ link5     (joint5, 손목)
                                     ├─ gripper_link  (그리퍼 고정 장착 기준 링크)
                                     ├─ 좌우 집게 링크 (gripper_joint + mimic 관절)
                                     └─ ee_link       (그리퍼 끝점, TCP. link5 기준 (0, 0.09, 0) m)
```

> 카메라 장착 링크: `link4`로 확정 (2026-11-09, C · D 찬성, PR #1). 문서는 원래 `link5` 위쪽(eye-in-hand)이었는데, 새 분리형 모델이 `link4`에 카메라(`DaBai_DCW2_Link`)를 고정하고 있다. `link4`에 달면 손목(joint5)이 돌아도 영상이 같이 회전하지 않아서 YOLO 학습 데이터 시점이 일정해진다 (C). 정렬은 이미지 stamp 시점 TF로 계산하고 정렬 중 팔이 고정이라 영향 없음 (D).
> 카메라 장착 오프셋: `link4 → DaBai_DCW2_Link` xyz (-0.0002, 0.0689, 0.0506) m, rpy (0.0087, 0, 0) rad, `DaBai_DCW2_Link → camera_link`는 오프셋 0. **통합 모델(차체+팔) 기준 공식 확정값** (B, PR #1 답변 id 7048233384, 2026-11-20). 참고: `link4 → camera_link`는 고정 관절이라 팔 자세와 상관없이 같은 값이고, 차체 · 바닥 기준 카메라 위치는 팔 장착 위치와 관측 자세로 정해진다.
> 합성 데이터(C)의 카메라 시점은 카메라 오프셋, 관측 자세 관절 값(6장), 팔 장착 위치(`base_link → arm_base_link` xyz, 아래)가 모두 정해져야 확정된다. 이 값이 바뀌면 데이터를 다시 만들어야 한다 (C).
> xacro 반영 완료 (B, 2026-11-08, `feat/grasp`): `Gripping_point_Link` → `ee_link`, `grip_joint` → `gripper_joint`(mimic · transmission · ros2_control · SRDF 포함), `gripper_link`(`link5`에 고정), `camera_link`, `camera_optical_frame` 추가. MoveIt 체인 끝은 `ee_link`. 보조 고정 링크(`dummy_mimic_*`, `vj1_link`, `vj2_link` 등)는 생략.
> 카메라 축: 이 모델의 `camera_link`는 일반 카메라 프레임(x 앞)과 달리 +Y가 시선 방향이다. 그래서 `camera_link → camera_optical_frame`은 rpy (-90°, 0, 0)이고, 결과는 1.2 표대로 x 오른쪽, y 아래, z 시선 방향이다. RViz에서 축 방향 확인(B), Gazebo 센서 설정(`DaBai_DCW2_Link` 기준 pose `0 0 0 3.14 3.14 -1.57`, 시선 +Y)과도 맞음(A 계산). 실제 영상 · 깊이와 맞는지는 센서 연동 후 확인.

> ⚠️ DOFBOT 원본 URDF는 시작 링크 이름이 `base_link`다. 모바일 베이스의 `base_link`와 충돌하므로 `arm_base_link`로 이름을 바꿔서 사용한다.
> `base_link → arm_base_link` 연결(팔 장착 위치)은 고정 관절(fixed)로 추가한다. 위치(xyz)는 C의 합성 데이터 시점에 영향을 주므로 먼저 정한다 (A, B).
> 팔 장착 방향 (2026-11-09 결정, PR #1 B 질문): DOFBOT 모델은 `arm_base_link` 기준 +Y가 팔의 앞쪽이다. 팔 내부 좌표계 · 관절축은 원본 그대로 두고, 장착 관절에서 yaw -90°(`rpy="0 0 -1.5708"`)를 줘서 팔 앞쪽(+Y)을 차체 앞쪽(`base_link` +X)에 맞춘다. 팔 좌표계를 바꾸면 관절축 · mesh 원점 · MoveIt 설정을 모두 다시 고쳐야 하기 때문. 두 좌표계 사이 변환은 TF가 처리한다. 장착 위치(xyz)는 (0.100, 0, 0.010) m로 확정 — `base_link → arm_base_link` 고정 관절값 전체: xyz (0.100, 0, 0.010) m, rpy (0, 0, -1.5708) rad (B, PR #1 답변 id 7048233384, 2026-11-20, 통합 모델 기준).

### 2.2 프레임 목록

| 역할 | 이름 | 상태 | 발행 주체 |
|---|---|---|---|
| 전역 지도 | `map` | 확정 | Nav2 (AMCL) |
| 오도메트리 | `odom` | 확정 | 시뮬레이터 / 구동 컨트롤러 |
| 바닥 기준 | `base_footprint` | 확정 (직접 제작 차체에서 이 이름 사용) | robot_state_publisher |
| 로봇 기준 | `base_link` | 확정 (직접 제작 차체에서 이 이름 사용) | robot_state_publisher |
| 라이다 | `base_scan` | 확정 (직접 제작 차체에서 이 이름 사용) | robot_state_publisher |
| 바퀴 · 캐스터 | `left_wheel_link`, `right_wheel_link`, `front_caster_link`, `rear_caster_link` | 확정 (`mm_description`) | robot_state_publisher |
| 카메라 | `camera_link` | 확정: `link4 → DaBai_DCW2_Link → camera_link`(오프셋 0), +Y가 시선 방향. 장착 링크 `link4` 확정 (C, D). 오프셋 값은 2.1 참고 (B 확정, 2026-11-20, 통합 모델 기준) | robot_state_publisher |
| 카메라 optical | `camera_optical_frame` | 확정: `camera_link` 기준 rpy (-90°, 0, 0), 축은 1.2 표 (xacro 추가 완료, B) | robot_state_publisher |
| 팔 베이스 | `arm_base_link` | 확정 (원본 `base_link`에서 변경). +Y가 팔 앞쪽이라 장착 시 yaw -90° (2.1) | robot_state_publisher |
| 팔 링크 | `link1` ~ `link5` | 확정 (분리형 DOFBOT 모델) | robot_state_publisher |
| 그리퍼 | `gripper_link` | 확정: `link5`에 고정된 그리퍼 장착 기준 링크 (xacro 추가 완료, B). 좌우 집게 링크(`rlink*`, `llink*`)는 `gripper_joint`와 mimic 관절로 움직임 | robot_state_publisher |
| 그리퍼 끝점 | `ee_link` | 확정: 원본 `Gripping_point_Link`에서 이름 변경 (xacro 반영 완료, B). `link5` 기준 (0, 0.09, 0) m에 고정. MoveIt IK 대상 링크 | robot_state_publisher |

> 실제 이름 확인 방법: 시뮬 실행 후 `ros2 run tf2_tools view_frames`

> `map → odom` 발행 주체: SLAM 단계(지도 작성)에서는 slam_toolbox, 자율주행 단계에서는 AMCL이 발행한다.

### 2.3 팔 관절 (분리형 DOFBOT 모델 `dofbot_joints.xacro` 기준)

모두 회전 관절(revolute). 회전축은 관절마다 다르다(비고 참고). 실물 서보 번호와 1:1로 대응한다.

| 관절 | 서보 | 부모 → 자식 | 하한 (rad) | 상한 (rad) | 비고 |
|---|---|---|---|---|---|
| `joint1` | 1번 | `arm_base_link` → `link1` | -1.5708 | 1.5708 | 베이스 회전, 회전축 z |
| `joint2` | 2번 | `link1` → `link2` | -1.5708 | 1.5708 | 회전축 x |
| `joint3` | 3번 | `link2` → `link3` | -1.5708 | 1.5708 | 회전축 x |
| `joint4` | 4번 | `link3` → `link4` | -1.5708 | 1.5708 | 회전축 x |
| `joint5` | 5번 | `link4` → `link5` | -1.5708 | 3.1416 | 손목 회전, 회전축 y |
| `gripper_joint` | 6번 | `link5` → 오른쪽 집게 `rlink1` (나머지 집게 링크는 mimic) | -1.57 | 0 | 약 90°. 좌우 집게가 mimic 관절로 함께 움직임. 원본 이름 `grip_joint`에서 변경 (xacro 반영 완료, B). 실물 서보 각도 범위 180°(0~180°로 추정)와는 별개 |

- `/joint_states`의 관절 이름은 위 표의 이름을 그대로 사용한다.
- 실물 서보 각도(°)와 URDF 관절 각도(rad)는 기준점이 다를 수 있다. 실물 코드는 joint2에 -90°, joint4에 +90° 보정을 하고 있었으므로(ㄷ자 버전 기준), 시뮬 결과를 실물로 옮길 때는 변환식을 따로 확인한다.

### 2.4 시뮬레이션용 URDF 수정 필요 사항

원본 DOFBOT URDF를 Gazebo에서 쓰려면 아래 작업이 필요하다.

- [x] `base_link` → `arm_base_link` 이름 변경
  - 완료 (B, 2026-11-07): 새 모델 Xacro에서 `arm_base_link`, `link1`~`link5`, `joint1`~`joint5`로 변경하고, MoveIt 설정 패키지도 새 모델 기준으로 다시 생성
- [x] 각 링크에 `<inertial>` (질량 · 관성) 추가
  - 완료 (B, PR #1 답변, 2026-11-21): 팔 · 그리퍼 링크에 inertial 값 추가 완료
- [x] `base_link.STL` 확보 (팀원 제공, 크기 약 12 × 26 × 8 cm, 단위 m 확인)
- [ ] `ros2_control` / `<gazebo>` 태그 추가
  - 진행 (B, 2026-11-07): 새 분리형 모델 기준으로 MoveIt2 설정 재생성 (`dofbot_arm` 그룹, `home` 자세, `dofbot_arm_controller`), RViz에서 Plan → Execute 성공. 현재는 FakeSystem(가짜 하드웨어) 기준이고 Gazebo · ros2_control 연동은 아직 (`dofbot_gazebo.xacro`, `dofbot_ros2control.xacro`는 비활성화 상태)
- [x] 그리퍼 링크 · 관절, `ee_link` 추가
  - 완료 (B, 2026-11-07~08): 그리퍼가 분리된 새 모델로 해결. 좌우 집게가 `grip_joint`와 mimic 관절로 함께 움직이고 RViz에서 개폐 확인. 이름 확정(2026-11-08): `Gripping_point_Link` → `ee_link`, `grip_joint` → `gripper_joint`, `gripper_link`는 고정 장착 기준 링크. xacro 반영 완료(B, 2026-11-08). `gripper_controller` 구성은 B 작업 예정
  - [ ] Gazebo에서 그리퍼 mimic 관절 연동 구동 · 검증 (B) — RViz에서는 개폐 확인됐으나 Gazebo 쪽 mimic 관절 연동은 아직 해결 중 (B, PR #1 답변, 2026-11-21)
- [x] 카메라 링크 · 센서 플러그인 추가
  - 진행 (B, 2026-11-08): `camera_link`, `camera_optical_frame` 추가. `dofbot_gazebo.xacro`에 RGB-D 센서(`ignition_frame_id`: `camera_optical_frame`)가 있으나 아직 비활성화 상태
  - 연동할 때 주의: 센서 토픽 이름이 `rgbd_camera/image`, `rgbd_camera/depth_image`, `rgbd_camera/camera_info` 등이라 3장 이름(`/camera/...`)과 다르다. bridge에서 3장 이름으로 맞춘다
  - 완료 (B, 2026-11-21, `feat/grasp` 35e9331·2052c89): 통합 모델(`mm_robot`) `mm_camera.gazebo.xacro`의 RGB-D 센서를 `sim.launch.py` bridge에서 3장 이름으로 연결 (`/rgbd_camera/image` → `/camera/image_raw`, `/rgbd_camera/depth_image` → `/camera/depth/image_raw`, `/rgbd_camera/camera_info` → `/camera/camera_info`). 시험용 일반 RGB 카메라 `arm_rgb_test`는 삭제해서 카메라는 RGB-D 하나만 쓴다. B WSL에서 세 토픽 발행자 1개 · 수신 확인, 초록 큐브 깊이 약 0.603 m 확인
- [x] mesh 경로를 실제 패키지 이름에 맞게 수정 → 새 모델 패키지 `dofbot_description`에서 RViz 표시 확인 (B)

---

## 3. 토픽

| 토픽 | 메시지 타입 | 발행 | 구독 | frame_id | 비고 |
|---|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | 시뮬 | 전체 | — | 시뮬 시간 (`use_sim_time: true`) |
| `/scan` | `sensor_msgs/msg/LaserScan` | 시뮬 | A | `base_scan` | 라이다. 10 Hz, 360개(1° 간격), 0.12~8 m |
| `/odom` | `nav_msgs/msg/Odometry` | 시뮬 | A | `odom` | 차동구동 플러그인이 발행. `odom → base_footprint` TF도 함께 발행 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 상태머신(A)이 선택한 쪽 | 시뮬 (차체) | — | 차체 속도. 이동 중엔 Nav2, 정렬 중엔 D의 명령을 전달 (5.0 참고) |
| `/nav2/cmd_vel` | `geometry_msgs/msg/Twist` | A (Nav2: velocity_smoother, behavior_server) | 상태머신 | — | Nav2 최종 출력. velocity_smoother 출력(`cmd_vel_smoothed`)과 behavior_server 출력(`cmd_vel`)을 이 이름으로 리매핑 (3.1 참고) |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 시뮬 | C, D | `camera_optical_frame` | RGB 이미지 (`rgb8`). RGB-D 카메라 컬러 영상을 bridge에서 이 이름으로 연결 (2.4) |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 시뮬 | C, D | `camera_optical_frame` | 깊이 이미지 (RGB-D 카메라, `32FC1`, m). D는 정렬 거리 계산에 사용 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 시뮬 | C, D | `camera_optical_frame` | 카메라 내부 파라미터 |
| `/perception/detections` | `vision_msgs/msg/Detection2DArray` | C | D, A | `camera_optical_frame` | YOLO 결과 (px) |
| `/perception/target_pose` | `geometry_msgs/msg/PoseStamped` | C | A, D, B | `base_link` | 목표 물체 3D 위치. 최소 3 Hz, 목표 10 Hz. 물체가 안 보이면 발행 안 함 (3.3 참고) |
| `/joint_states` | `sensor_msgs/msg/JointState` | 시뮬 / ros2_control | B, D | — | |
| `/servoing/cmd_vel` | `geometry_msgs/msg/Twist` | D | 상태머신 | — | 베이스 미세 정렬용 차체 속도 명령 (팔은 고정) |
| `/perception/target_class` | `std_msgs/msg/String` | 상태머신(A) | C | — | 이번에 잡을 물체 클래스 이름. QoS `transient_local` (나중에 켜진 노드도 마지막 값 수신) |
| `/task_manager/state` | `std_msgs/msg/String` | 상태머신(A) | 누구나 (모니터링 · 시험) | — | 현재 단계 이름 (`IDLE`, `WAIT_READY`, `NAVIGATE`, `VERIFY_ARRIVAL`, `ALIGN`, `GRASP`, `CANCELING`, `SUCCEEDED`, `FAILED`). 단계가 바뀔 때 발행, QoS `transient_local`. QoS depth 17(11/19, 재정렬 최대+취소 경로의 최대 12개 + 여유 5. 구독 측도 17 이상 권장). 구현 완료 (A) |

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
- 카메라가 팔(`link4`)에 달려 있으므로, `camera_optical_frame` → `base_link` 변환은 이미지 stamp 시점의 TF로 계산한다.
- C는 `/perception/target_class`로 받은 클래스의 물체만 `target_pose`로 발행한다. 클래스 이름은 `Align`의 `target_class`와 같은 문자열을 쓴다.
- 발행 주기 (2026-11-19 갱신, PR #3): 최소 3 Hz, 목표 10 Hz. 평균이 아니라 순간 최소 주기가 3 Hz 아래로 떨어지지 않아야 한다.
  - 최소 3 Hz 근거 (D, 2026-11-19): 검출 주기별 정렬 모의 시험 15회씩. 3 Hz(처리 지연 0.33 s) 15/15 성공, 1.5 Hz 7/15, 1 Hz 3/15 (검출이 늦게 오는 동안 차체가 너무 돌아 물체가 화면 밖으로 나감). D는 정렬 쪽 지연 보정도 추가 예정.
  - C 실측 (2026-11-19, Gazebo와 함께 실행, Far 자세, WSL CPU): OpenVINO 적용 후 평균 약 4.4 Hz · 최소 3.8 Hz (PyTorch는 평균 약 3.0 Hz · 최소 1.4 Hz, 추론 약 165 ms → 100 ms). 최소 3 Hz 기준 충족, 목표 10 Hz에는 아직 못 미침.
  - 이전 기준 최소 5 Hz (C, 2026-11-09)는 Gazebo 연동 전 C 노트북(WSL, i5-1240P CPU)의 YOLOv8n 추론 시간(640 입력 약 128 ms(약 7.8 Hz), 320 입력 약 73 ms(약 13.6 Hz))으로 정한 값이었다.
- 수신 측 안전장치 (A): `/perception/target_pose` 수신 측(A)은 `frame_id`가 `'base_link'`와 정확히 일치하지 않거나, x/y/z에 NaN/inf가 있거나, x≤0이면 해당 메시지를 무시한다(구현: f523b1f, 원준님 PR #1 제안 기반).
  - 추가 필터 (A, C 제안 PR #1 2026-11-20, `feat/state-machine` 구현): x가 상한 `target_pose_max_x_m`(기본 2.0 m)보다 크면 무시한다(경계값 2.0 m는 통과). 직전에 통과한 x와 차이가 `target_pose_max_jump_m`(기본 0.1 m) 이상이면(앞뒤 양방향) 무시한다. 걸러진 메시지는 직전 값으로 쓰지 않고, 마지막으로 통과한 메시지 뒤 stamp 기준 `target_pose_max_age_sec`(기본 1.0 s) 넘게 지나면 직전 값을 지우고 처음 메시지처럼 본다(물체를 놓쳤다가 다시 잡았을 때 계속 걸리지 않게). 상한 2.0 m는 far 관측 범위 최대값 기준 1차값이고, C의 실제 검출 가능 거리 측정 후 조정한다.
- `header.stamp`는 처리 시각이 아니라 이미지를 찍은 시각이다. 물체가 안 보이면 발행하지 않으므로, 받는 쪽은 stamp 기준으로 1 s 넘게 새 값이 없으면 미검출로 처리한다 (C).

---

## 4. 액션 · 서비스

| 이름 | 타입 | 서버 | 클라이언트 | 비고 |
|---|---|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | A (Nav2) | 상태머신 | Nav2 표준 |
| `/servoing/align` | `project_interfaces/action/Align` | D | 상태머신 | 베이스 미세 정렬 (팔 고정). 필드는 4.1 참고 |
| `/grasp/execute` | `project_interfaces/action/Grasp` | B | 상태머신 | IK 검증 및 로봇팔 이동·파지. 필드는 4.2 참고 |
| (이름 미정) | `project_interfaces` 액션 (예정) | B | 상태머신 | 팔을 관측 자세(`observe_pose`)로 이동. B가 MoveIt2로 경로 계획 · 실행 후 성공 여부 반환. 이름 · 필드는 B가 개발하면서 초안 공유 후 확정 (B, 2026-11-09) |
| `/dofbot_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 팔 컨트롤러 | B (MoveIt) | joint1~5, `joint_trajectory_controller/JointTrajectoryController`. `ros2 action list -t`로 확인 (B, 2026-11-07, 새 모델 MoveIt 설정 기준) |
| `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | 그리퍼 컨트롤러 | B | ros2_control 그리퍼 컨트롤러 이름을 `gripper_controller`로 설정 |

> 표준 메시지로 표현 가능한 건 표준을 쓴다. `project_interfaces` 패키지는 위 커스텀 액션이 확정될 때 만든다.

### 4.1 `Align` 액션 (D 확정, 2026-11-09)

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
- 상태머신이 보내는 `timeout_sec` 기본값: 30 s (D 확인, PR #1 2026-11-19. YOLO 3 Hz 조건 모의 시험에서 정렬 최대 약 9.7 s). 파라미터 `align_timeout_sec`로 바꿀 수 있다.
- 시간 제한 (D 확인, 2026-11-09): 서버는 `timeout_sec`이 되면 직접 `ALIGN_TIMEOUT`을 반환한다. 상태머신은 `timeout_sec`보다 몇 초(임시 5 s) 더 기다려도 결과가 없으면 목표를 취소하고 `ALIGN_TIMEOUT`으로 기록한다. 취소되면 정렬 쪽은 속도 0을 보내고 끝낸다.
- 정렬 완료 판정(CHECKING) 중 검출 끊김 허용 시간 (D 확정, 2026-11-22): 차체가 멈춘 뒤 판정하는 동안은 최대 3 s(`lost_timeout_stopped` 파라미터)까지 검출이 끊겨도 기다리고, 그 사이 다시 검출되면 판정을 계속한다. 차체가 움직이는 동안은 기존대로 1 s. 허용 시간을 넘기면 `OBJECT_NOT_FOUND`로 처리(5.3).

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
- `target_pose`는 `base_link` 기준 그대로 받는다. IK 계산 때 B가 TF로 `arm_base_link` 기준으로 변환해서 쓴다 (B 확인, 2026-11-09)
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
- 발행 규칙 (A, 2026-11-09, `mm_task_manager`): `/cmd_vel`을 10 Hz로 계속 발행하고, 실행 중에는 속도 0도 계속 발행한다. 선택한 입력이 0.5 s 넘게 안 오면 0을 보낸다. 단계가 바뀌기 전에 받은 명령은 전달하지 않는다.
- 근거: Ignition DiffDrive는 마지막 `/cmd_vel`을 계속 유지한다 (0.1 m/s를 한 번만 보냈을 때 5.7 s 동안 유지, 2026-11-09 측정). 그래서 0을 한 번만 보내면 안 된다. 이 규칙 덕분에 Nav2가 취소에 응답하지 않아도 상태머신이 0을 보내서 차체가 멈춘다.

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
- 시도별 `standoff_m` 값: 구체적인 순서 · 조합은 아직 TBD이지만, 하한은 `0.32 m 이상`으로 정한다 (D 제안 PR #1 id 7066535320, C 동의 id 7075009411, 2026-11-21~22. 근거: 0.29/0.32/0.35 m 모두 IK 성공 + 접촉 없음이 확인됐으나, pre-grasp 69.3° 화면 가시 범위가 약 0.31 m 아래로는 물체가 화면 밖으로 벗어남을 D·C 양쪽이 독립적으로 확인. 0.29 m는 재시도 후보에서 제외).

### 5.3 실패 코드

실패 원인을 같은 형식으로 기록해 두면 나중에 성능 분석과 LLM 실패 원인 추정에 그대로 쓸 수 있다.

| 코드 | 의미 | 발생 모듈 |
|---|---|---|
| `NAV_NOT_READY` | 시작 확인 실패 (Nav2 비활성, `map → odom` 없음 등). 목표를 보내지 않음 (5.5) | A |
| `NAV_FAILED` | 목표 지점 도착 실패 (도착 확인 실패 포함, 5.5) | A |
| `OBJECT_NOT_FOUND` | 물체 미검출 (정렬 중 물체를 놓친 경우 포함) | C, D |
| `ALIGN_TIMEOUT` | 제한 시간 내 정렬 실패 | D |
| `IK_FAILED` | 정렬 후 IK가 풀리지 않음 (재정렬 대상) | B |
| `GRASP_MISS` | 그리퍼가 물체를 못 잡음 | B |
| `GRASP_SLIP` | 잡은 뒤 놓침 | B |
| `UNKNOWN` | 기타 | — |

- `OBJECT_NOT_FOUND` 판정 기준 (C 제안): score 0.5 이상인 목표 클래스 검출이 연속 N프레임(예: 10프레임) 동안 없으면 발생. 수치는 첫 모델 학습 후 조정
  - 정렬 완료 판정(CHECKING) 중에는 별도 시간 기준을 쓴다 — 멈춘 뒤 최대 3 s, 움직이는 중엔 1 s 끊기면 `OBJECT_NOT_FOUND` (D 확정, 4.1)
- B는 `IK_FAILED`, `GRASP_MISS`, `GRASP_SLIP`을 사용한다 (`GRASP_FAILED`는 따로 만들지 않음, B 확인)
- 세부 원인(detail): A는 같은 코드 안에서도 원인을 나눠 결과 기록(5.5)에 남긴다. `NAV_NOT_READY`: `lifecycle_inactive`, `cmd_vel_conflict` 등 / `NAV_FAILED`: `rejected`, `aborted`, `timeout`, `arrival_check`, `tf_stale` / `UNKNOWN`: `clock_stopped` / `OBJECT_NOT_FOUND`: `target_pose_stale` (정렬 뒤 1.0 s 안에 쓸 `target_pose`가 없음, 3.3 미검출 규칙)
- 상태머신이 `Align` 결과를 `timeout_sec` + 여유 안에 못 받으면 취소하고 `ALIGN_TIMEOUT`으로 기록한다 (D 확인, 4.1). `Grasp` 제한 시간은 임시 60 s (B 확인), 넘었을 때 쓸 코드는 미정

### 5.4 파지 대상 물체 (2026-11-03 회의)

단계별로 진행한다. 앞 단계가 끝까지 성공하면 다음 단계 물체를 추가한다.

| 단계 | 클래스 이름 | 물체 | 크기 | 모델 |
|---|---|---|---|---|
| 1단계 | `block` | 사각 블록 (직육면체) | 2.5 × 2.5 × 4 cm | SDF 기본 도형 (box) |
| 1단계 | `pin` | 원통형 핀 (롤러) | 지름 2 cm, 길이 5 cm | SDF 기본 도형 (cylinder) |
| 2단계 | `hex_nut` | 육각 너트 | 폭 3 cm, 두께 1.5 cm | CAD로 제작 |
| 3단계 | `bolt` | 육각 볼트 | 머리 폭 2.5 cm, 길이 6 cm | CAD 또는 무료 모델 |

- 크기는 그리퍼 최대 벌림 폭(약 6 cm, B 확인)보다 작아야 한다. hex_nut은 꼭짓점끼리 약 3.5 cm라 잡는 방향에 주의.
- 학습 데이터는 Gazebo 합성 데이터를 기본으로 하고, 실제 사진 데이터셋(Roboflow, CC BY 4.0)을 보조로 섞는다 (C). 클래스 이름은 위 이름으로 통일.

### 5.5 이동 단계 확인 · 시간 제한 · 재시도 (A, 2026-11-09)

`mm_task_manager`(`feat/state-machine`) 구현과 시뮬 측정을 바탕으로 정한 규칙이다. 숫자는 모두 임시값이고 파라미터로 바꿀 수 있다.

- 시작 확인: 이동 목표를 보내기 전에 아래를 모두 확인한다. 30 s 안에 안 되면 `NAV_NOT_READY`로 끝내고 목표는 보내지 않는다.
  - `/lifecycle_manager_localization/is_active`, `/lifecycle_manager_navigation/is_active` (`std_srvs/srv/Trigger`)가 true
  - `map → odom` TF가 신선함 (아래 기준)
  - `/navigate_to_pose` 액션 서버가 있음
  - 상태머신 말고 다른 `/cmd_vel` 발행자가 없음. 있으면 `NAV_NOT_READY` (`cmd_vel_conflict`). 이전 실행의 노드나 teleop이 남아 있으면 `/cmd_vel` 0이 섞여 차체가 제대로 못 움직인다 (2026-11-09 확인: 남은 노드 수에 비례해 느려지고 Failed to make progress 발생). Gazebo bridge는 `/cmd_vel`을 구독만 해서 걸리지 않는다
  - 노드 시작 후 1 s 동안은 판정하지 않는다 (다른 노드를 발견하기 전에 시작하는 것을 막기 위함)
  - 근거: AMCL 활성화 단계에서 멈춰 Nav2가 켜지지 않은 적이 있다 (`mm_navigation` README)
- 도착 확인: Nav2가 SUCCEEDED를 돌려줘도, 0.5 s 뒤 `map → base_link`를 목표와 비교한다. 위치 0.15 m, yaw 0.35 rad를 넘으면 `NAV_FAILED` (`arrival_check`).
  - 근거: `map → odom` 변환이 실패하면 Nav2가 차체를 움직이지 않고 성공을 돌려주는 경우가 있다 (`mm_navigation` README "가짜 성공")
  - 기준값은 D의 정렬 시작 범위와 같은 값으로 잡았다
  - `/amcl_pose`는 쓰지 않는다 (차체가 일정 거리 · 각도를 움직여야 갱신됨)
- TF 신선도: `map → odom`의 stamp가 현재 시각 − 0.5 s보다 오래되면 오래된 것으로 본다 (`tf_stale`). AMCL은 stamp를 `transform_tolerance`(1.0 s)만큼 미래로 찍고, 차체가 정지해 있어도 10 Hz로 다시 보낸다 (2026-11-09 측정). 미래 시각은 정상으로 본다.
- 도착 오차 측정 (2026-11-09, 남은 노드 없는 정상 주행 11회): 상태머신 기준 0.08~0.11 m (Nav2 `xy_goal_tolerance` 0.10), Gazebo 실제 위치 기준 0.02~0.126 m (AMCL 오차 0.02~0.12 m가 대부분), yaw 약 0.21~0.26 rad (Nav2 yaw 허용치 0.25 근처). Nav2 허용 오차를 줄여도 AMCL 오차는 남아서 효과가 작으므로 Nav2 설정은 그대로 둔다. D가 모의 모델로 도착 위치 오차 0.13 m 안에서 400번씩 돌려본 결과 정렬 제어는 모두 수렴했고, 문제는 시작할 때 물체가 화면에 보이는지였다 (아래 이동 목표 거리). yaw는 정렬 시작 범위(0.35 rad) 안이라 여유 있음
- 이동 목표 거리 (D 제안, 2026-11-09, 2026-11-21 near standoff 변경에 맞춰 기준 갱신 A): Nav2 목표는 near 1차 Align 기준 `standoff_m`(=0.40 m, 5.6)보다 멀리 둔다. 목표값은 약 0.50 m (D 11/21 재제안, PR #1 id 7066535320: near 화면 가시 범위 약 0.30~0.59 m에 들어오도록 +0.10 m margin). 기존에는 당시 기준 `standoff_m` 0.35 m + 0.15 m margin(실측 도착 오차 기반, D 11/10 제안) = 0.50 m였고 숫자는 그대로 0.50 m로 맞아떨어지지만, margin 근거가 "도착 오차 기반 0.15 m"에서 "near 화면 가시 범위 기반 0.10 m"로 바뀐 것이라 두 근거가 완전히 같은 계산은 아니다 — 상태머신 구현 시 0.50 m를 그대로 쓰되, margin 근거가 달라졌다는 점은 확인 필요로 남겨둔다. 상태머신 코드는 아직 반영 전
  - 근거(과거 0.25/0.35 m 비교 시험, D 모의 시험 2026-11-09): 0.25 m에 두면 약 40 %가 처음부터 물체가 화면 밖, 0.35 m 이상이면 거의 100 % 보임. 차체 앞 끝(라이다 포함)이 `base_link` 기준 x +0.215 m라 0.25 m에 세우면 물체 중심까지 여유가 약 3.5 cm뿐이고, 도착 오차만큼 물체에 부딪힐 수 있음
  - 차체 앞 끝 값 (A, 2026-11-15): PR #4(B, `feat/navigation-chassis-update`, 리뷰 중)에서 차체가 0.40×0.30×0.10 → 0.38×0.28×0.06 m로 바뀌어 차체 앞면은 x +0.19 m로 줄었지만, 라이다(`base_scan`, x 0.18 m)가 앞으로 튀어나와 반경 0.035 m를 포함한 실제 앞 끝은 x +0.215 m. PR #4 리뷰 중 URDF 값 계산과 TF(`base_link → base_scan` = (0.18, 0, 0.075))로 확인하고 RViz로 모양 확인. 이전 값 0.20 m는 예전 차체(길이 0.40 m) 기준
  - 카메라 위치 임시값으로 계산한 결과라 관측 자세가 정해지면 D가 다시 확인
- 시간 제한
  - 단계마다 sim time 기준 제한을 둔다 (이동 90 s). 넘으면 목표를 취소하고 `NAV_FAILED` (`timeout`)
  - 취소 응답은 5 s까지 기다리고, 응답이 없으면 이번 실행을 중단한다
  - 취소 뒤에 늦게 도착한 이전 목표의 결과는 무시한다
  - `/clock`이 실제 시간(wall time)으로 3 s 넘게 멈추면 `UNKNOWN` (`clock_stopped`)으로 중단한다
- 종료: 끝나면(성공, 실패, 실행 중단 모두) `/cmd_vel` 0을 1 s 보낸 뒤 노드를 종료한다. DiffDrive가 마지막 명령(0)을 유지해서 차체는 계속 서 있다 (종료 후 5 s 동안 이동 0 m 확인). 노드가 남아 있지 않으므로 다음 실행의 발행자 충돌 확인에 걸리지 않는다. 한 번 실행에 물체 1개.
- 재시도: 시작 실패 · 이동 실패 · 도착 확인 실패는 같은 실행에서 다시 시도하지 않는다 (가짜 성공이 반복될 수 있음). 재시도는 5.2 재정렬만 한다.
- 결과 기록: 실행마다 JSON 한 줄로 기록한다 (기본 `~/.ros/mm_task_manager/results.jsonl`). 항목은 최종 상태, `error_code`, 세부 원인, 실패 단계, 재정렬 횟수, 사용한 `standoff_m`, 단계별 걸린 시간, 도착 오차. 나중에 LLM 실패 원인 추정에 쓴다.
- **접근 방향(yaw) 고정: `approach_yaw=0` (A, 2026-11-23, block·bolt 각 N=10 반복 검증)**: object 모드로 이동할 때 Nav2 목표 yaw를 물체를 정면으로 바라보는 방향(0 rad, `approach_yaw=0`)으로 강제한다.
  - 배경: 이 개발 VM(Mac+UTM, ogre2 + CPU/software 렌더링)에서는 카메라가 world yaw 기준 `|yaw| 8~11°`를 넘는 특정 방향을 볼 때 항상 회색 화면이 되는 렌더링 버그가 있다(원인: 엔진/소프트웨어 렌더링 조합 자체로 추정, 수정하지 않기로 결정). 안전 구간은 world yaw 0/90/180/270° 각각 중심 `±11°`뿐이라, 물체를 정면(0°)으로 보고 접근하면 이 안전 구간 안에 들어간다.
  - 검증 결과 (block·bolt 각 N=10, Near standoff 0.40 m, 7차 모델, RELIABLE QoS): 둘 다 Nav2 도착 10/10, Align 1차 10/10 성공. 도착 yaw는 물체마다 한쪽으로 치우침(무작위 아님) — block −2.26°~−5.36°(평균 −4.06°), bolt +6.59°~+7.44°(평균 +6.97°). 안전 구간(11°)까지 남은 최소 여유는 block 5.6°, bolt 3.6°. 카메라 회색 감지·충돌 징후 0/10.
  - **도착 yaw가 치우치는 이유 및 `yaw_goal_tolerance` 의존성**: Nav2가 `yaw_goal_tolerance` 안에 들어오는 즉시 멈추기 때문에, 접근 방향에서 돌아오는 쪽 끝에서 서는 경향이 있다(무작위 분산이 아님). 도착 yaw의 상한은 사실상 `yaw_goal_tolerance` + 위치 추정 오차로 정해진다. 지금 `src/mm_navigation/config/nav2_params.yaml`의 `yaw_goal_tolerance: 0.125`(7.16°, 2026-11-17 정차 오차 실험으로 0.25→0.125 변경, 5.5 참고)를 전제로 측정한 값이다.
  - **조건부 확정 (A, 이 VM 기준)**: 위 결과로 block·bolt는 `approach_yaw=0` 설계를 확정한다. 단:
    1. `yaw_goal_tolerance`는 0.125 rad 이하로 유지한다 — 되돌리면(예: 0.25로 복귀) 안전 구간(11°)을 넘어갈 수 있다.
    2. 카메라 회색 자동 감지 가드(`tools/diagnostics/camera_health.py`)는 계속 켜 둔다 — 안전 구간 `±11°`는 이 VM(ogre2+CPU 렌더링) 기준값이고, 팀원 PC(GPU 렌더링 등) 결과는 아직 다 모이지 않아 경계가 다를 수 있다.
    3. pin·hex_nut은 아직 반복 검증 안 함(위치가 block·bolt 사이라 도착 yaw도 그 사이일 가능성이 높지만 미확인) — 확정 전 각 N=5 정도 권장.
  - 참고로 이전 단일 실행에서 추정했던 여유값(pin 1.2°/hex_nut 0.7°/bolt 0.5°)은 계산 기준이 다른 것으로 보이며, 이번 N=10 반복 측정값(block 5.6°, bolt 3.6°)이 더 신뢰도가 높다.

### 5.6 카메라 관측 자세: far / near (C 제안, D 동의, 2026-11-15)

카메라가 팔(`link4`)에 달려 있어 관측 자세에 따라 시점이 바뀐다. 용도별로 관측 자세를 두 가지로 나눈다. 파지 단계에서 쓰는 자세(pre-grasp, grasp)도 함께 적는다.

값 출처: (B 측정, PR #3 댓글·feat/grasp observations/, 2026-11-15). far 관절 값은 PR #1 댓글(2026-11-11)에 공유된 값과 같다.
물체 거리: 로봇 중심 ~ 물체 중심 수평거리.

| 자세 | 용도 | 물체 거리 (m) | 식별자(파일명) | 관절 값 (rad) | 상태 |
|---|---|---|---|---|---|
| far | 이동 중 물체 찾기 · 시맨틱 맵 (원거리 탐색) | 0.55~2.00 (후보) | `observe_pose_01` | joint1 0, joint2 0, joint3 -1.1464, joint4 -0.8237, joint5 0 | 확정 |
| near | 정렬(D) · 파지(B) (접근 · 초기 정렬) | 0.30~0.60 (후보) | `observe_pose_near` | joint1 0, joint2 -0.008482, joint3 -1.282087, joint4 -1.299052, joint5 -0.000157 | 확정 (B 선정) |
| pre-grasp | 최종 관측 · 파지 준비 | 0.25~0.35 | `pre_grasp_pose_01` | joint1 0, joint2 -0.636802, joint3 -0.959445, joint4 -1.190737, joint5 -0.000157 | 변경 적용 완료, YOLO 검증 진행 중 |
| grasp | 그리퍼를 물체 위치로 이동 | 0.265 (참고) | `grasp_pose_01` | joint1 0, joint2 -1.282087, joint3 -0.602873, joint4 -1.231193, joint5 -0.000157 | 참고: 도달 위치 검증용. Pre-grasp → Grasp 이동 중 충돌 · 재현 문제가 있어 MoveIt2로 해결 예정 (B) |

※ 아직 MoveIt SRDF에 등록된 자세 아님, observations/ 파일명을 참고용으로 적음

카메라 TF (`base_link` → `camera_optical_frame`, B 실측):

| 자세 | xyz (m) | rpy (rad) | 광축 방향 |
|---|---|---|---|
| far | (0.259, 0.001, 0.269) | (-1.961, 0, -1.571) | 약 22° 아래 |
| near | (0.260, 0.001, 0.206) | (-2.581, 0, -1.571) | 약 58° 아래 |
| pre-grasp | (0.303, 0.001, 0.150) | (-2.778, 0, -1.571) | 약 69.3° 아래 |
| grasp | — (기재 안 함) | — | — |

- pre-grasp 자세 변경 (B, 2026-11-21, `feat/grasp` 5741526): YOLO 근거리 검출을 위해 카메라 숙임각을 약 84°에서 69.3°(A 실측 69.2°)로 낮춤. joint4 -1.435083 → -1.190737 rad, 나머지 관절은 그대로. 이전 자세는 `set_pre_grasp_pose_84deg.sh`로 보존. 위 표 · 카메라 TF에 반영 완료 (B 확인, PR #1 답변, 2026-11-21). YOLO 쪽 검증은 진행 중(6장 체크리스트 참고)

- 자세 전환 순서: Far → Near(약 0.60m) → Pre-grasp(약 0.35m) → Grasp (B 제안, 희선님 개인 노션, 2026-11-15)

- Near→Pre-grasp 전환 방식 (D 확정, 2026-11-19):

정렬 도중 팔 자세를 바꾸면 영상이 흔들려 검출이 끊기고(OBJECT_NOT_FOUND 유발 가능) 거리·방위 계산이 부정확해질 수 있다. 따라서 Near→Pre-grasp 전환은 정렬을 계속 돌리면서 팔을 바꾸는 방식이 아니라, Align 호출을 두 단계로 나누고 그 사이에 팔을 전환하는 방식으로 한다.

1. near 관측 자세에서 /servoing/align 1차 호출 — standoff_m=0.40 (D 제안, PR #1 댓글 id 7066535320, 2026-11-21 / C 동의, id 7075009411, 2026-11-22. 근거: near 0.35 m에서는 pin 검출 score가 0.40 정도로 OBJECT_NOT_FOUND 임계값 0.5에 못 미치는 경우가 있었음. near 화면 가시 범위(약 0.30~0.59 m)와 pre-grasp 69.3° 화면 가시 범위(약 0.31~0.46 m) 양쪽에 0.40 m가 들어와 전환 전후로 물체가 계속 보임. 기존 0.35 m는 더 이상 near 단계 값으로 쓰지 않음)
2. 1차 완료 후, 정렬(차체 이동)은 멈춘 상태로 B의 관측 자세 전환 액션(observe_pose, 4장)을 호출해 팔을 pre-grasp 자세로 이동
3. 전환 완료 후 pre-grasp 관측 자세에서 /servoing/align 2차 호출 — standoff_m=0.35 m (기본값 확정, A, 2026-11-21. 근거: B의 IK 검증 범위 0.29~0.35 m 전부 성공 + VM 접촉 경계 정밀 시험에서 0.29~0.35 m 전부 주행 접촉 없음 확인, 단 Grasp 자세 기준 pin·bolt는 0.29 m 여유가 타이트해 가장 안전한 0.35 m 채택). near(1차) 0.40 m → pre-grasp(2차) 0.35 m로 2단계 값이 달라지며, 재시도별 값은 5.2 참고

- Near→Pre-grasp 전환 중에는 차체를 정지시키고, 팔만 Pre-grasp 자세로 이동한다. 팔의 자세 전환이 완료된 후 `/servoing/align`을 다시 호출한다. 정렬과 팔의 자세 전환은 동시에 수행하지 않는다 (B 확인, PR #1 답변, 2026-11-21).
- 이 흐름은 5.2(IK 실패 후 재정렬, 최대 2회)와 별개다 — 재정렬은 같은 pre-grasp 자세 안에서 standoff_m만 바꿔 다시 호출하는 것이고, 여기서 말하는 2단계는 재정렬과 무관하게 매번 거치는 고정 순서다.
- 의존성: 이 방식은 B의 observe_pose 액션(PR #5)이 확정·구현돼야 실제로 상태머신에 반영할 수 있다. PR #5 리뷰 전까지는 설계만 문서화하고 mm_task_manager 코드는 그대로 둔다.
- 알아둘 점: observe_pose 액션 초안(PR #5, 7b9920f)은 pre-grasp 자세 이름을 추가하는 작업을 이번에 같이 진행했다(efe20a7, pose_name "pre_grasp").
- 3장 /task_manager/state 단계 목록에 이 전환 구간을 위한 서브상태가 필요한지는 추후 구현 때 결정.

깊이 이미지 형식도 D가 재확인: 32FC1(m), NaN/inf는 정렬 쪽 계산에서 제외(원준님은 11/18에 32FC1 가능성이 높다고 답변, 아직 확정 아님). B도 통합 모델 RGB-D 카메라에서 32FC1 확인 (2026-11-21, WSL).

- near 자세 조건 (D): near 자세에서 정렬 시작 시점(Nav2 도착 기준 약 0.27~0.53 m)과 정렬 끝 시점(standoff 기준 약 0.2~0.55 m) 범위의 물체가 계속 화면 안에 보여야 한다
- near 자세 관절 값은 B가 far 자세 가시성을 확인한 뒤 선정했다 (2026-11-15, 위 표)

### 5.7 카메라 내부 파라미터 (B 측정, 2026-11-15)

| 항목 | 값 |
|---|---|
| 해상도 | 640×480 |
| 주기 | 10 Hz (RGB-D 카메라 `update_rate` 현재 설정값) |
| 화각 (FOV) | 수평 1.047 rad (약 60°), 수직 약 46.8° |
| 초점 거리 · 중심 | fx = fy = 554.3827, cx = 320.0, cy = 240.0 |
| 왜곡 | `plumb_bob`, [0, 0, 0, 0, 0] |

- Gazebo가 해상도 · FOV로 생성한 값이고, 별도 캘리브레이션은 없다
- 관측 자세가 바뀌어도 내부 파라미터는 같다
- 처음 측정(2026-11-15)은 일반 RGB 카메라 `arm_rgb_test`(10 Hz) 기준. 2026-11-21 B가 시험용 RGB 카메라를 삭제하고 RGB-D 카메라(`rgbd_camera`) 하나로 정리함 (2.4). 이후 `update_rate` 설정값을 10 Hz로 변경함(B 확인, PR #1 답변, 2026-11-21). 두 센서의 해상도(640×480) · 수평 FOV(1.047 rad) · 장착 pose 설정이 같아서 내부 파라미터도 같을 것으로 보지만, `rgbd_camera` 기준 `/camera/camera_info` 실측 재확인은 아직
- 실제 발행 주기 실측 (A, 2026-11-21, Mac VM, `render_engine:=ogre2 software_gl:=true headless:=true`, 30 s): 이 측정은 `update_rate` 30 Hz 기준이었음(설정값 30 Hz와 달리 `/camera/image_raw` 약 24.5 Hz, `/camera/depth/image_raw` 약 21.8 Hz, `/camera/camera_info` 약 28 Hz로 측정됨. CPU(소프트웨어) 렌더링 부하 때문으로 추정). 이후 `update_rate`가 10 Hz로 바뀌었으므로 이 실측치는 참고용이고, 10 Hz 기준 재실측 필요

---

## 6. 미정 항목 체크리스트

- [x] 로봇팔 모델 → DOFBOT 확정, 그리퍼 분리형 모델로 확정 (2026-11-08, 2.1~2.3 반영)
- [x] 모바일 베이스 모델 → 직접 제작 차동구동 차체
- [x] `base_scan` 프레임 이름 → 직접 제작 차체에서 이 이름 사용
- [x] 팔 장착 위치 (`base_link → arm_base_link` 고정 관절 값) → xyz (0.100, 0, 0.010) m, rpy (0, 0, -1.5708) rad로 확정 (B, PR #1 답변, 2026-11-20, 2.1)
- [x] 카메라 장착 링크 → `link4` (2026-11-09, C · D 찬성, 2.1)
- [x] 카메라 장착 오프셋 (B) → `link4 → DaBai_DCW2_Link` xyz (-0.0002, 0.0689, 0.0506) m, rpy (0.0087, 0, 0) rad로 확정 (통합 모델 기준, PR #1 답변, 2026-11-20, 2.1)
- [x] 6번 서보(그리퍼) 각도 범위 → 180°
- [x] 그리퍼 실제 벌림 폭(cm) · 그리퍼 모델 형태 → 최대 약 6 cm, 좌우 집게 분리형 (`gripper_joint` + mimic, 원본 이름 `grip_joint`) (B)
- [x] `base_link.STL` → 팀원 제공
- [x] joint2, joint4 범위 → 분리형 DOFBOT 모델로 교체해서 해결 (2.3 관절 표 기준. 처음엔 1자 버전 URDF로 해결했으나 이 URDF는 더 이상 쓰지 않음)
- [x] 시뮬 카메라 토픽 이름 → `/camera/...`로 확정
- [x] 깊이 이미지 형식 → `32FC1`(m) 확정 (D가 Gazebo Fortress 깊이 카메라에서 확인, PR #1 댓글 2026-11-19. C도 32FC1 예상, PR #1 댓글 2026-11-18)
- [x] visual servoing 제어 방식 → 베이스 미세 정렬, MoveIt Servo 미사용
- [x] `Align` 액션 필드 → D 확정 (4.1, 2026-11-09)
- [x] `Align` 오차 기준 좌표계 → `base_link` (D)
- [x] 파지 목표 자세 생성 → B가 C의 `target_pose`로 계산 (4.2)
- [x] 5.1 전환 조건 → D 제안 초안 반영, 그리퍼 확정 후 조정
- [x] 차체 명령 선택 방식 → 상태머신이 단계별로 직접 선택 (5.0)
- [x] IK 실패 후 재정렬 방식 → 최대 2회 재시도, 시도마다 `standoff_m` 변경 (5.2)
- [x] 기본 `standoff_m` 값 → near(1차) 0.40 m / pre-grasp(2차·최종) 0.35 m, 2단계로 분리 확정 (A, 2026-11-21~22, 근거는 5.6 참고. 초기 시험값 0.25 m → 단일 0.35 m → 지금의 near/pre-grasp 2단계 값으로 변경)
- [ ] 재시도별 `standoff_m` 값 (A) — 하한 0.32 m 이상으로 정함(0.29 m는 pre-grasp 69.3° 화면 가시 범위 밖이라 제외, D·C 확인, 5.2). 0.32/0.35 m 중 구체적인 순서 · 조합은 아직 미정
- [x] `Grasp` 필드 타입 · `target_class` 사용 · 실패 코드 → B 확정 (4.2, 5.3)
- [ ] 같은 클래스 물체가 여러 개일 때 구분 방법 (`target_id` 추가 여부, A·B·C) — C가 pre-grasp 단계에서 "화면 중심에 가장 가까운 검출"을 고르는 방식을 제안(PR #3). D 확인 후 결정
- [ ] 물체 방향(yaw)이 파지에 필요한지 (B) — 처음엔 고정 접근 방향, 물체 확정 후 검토. 필요하면 C가 `target_pose` orientation 채움. 파지는 위에서 수직으로 접근하는 방식을 우선 검토 (B). 5축이라 IK가 어려우면 자세 제약 완화나 측면 접근 검토
- [x] 그리퍼 컨트롤러 이름 → `gripper_controller`
- [x] 파지 대상 물체 · YOLO 클래스 이름 → `block`, `pin`, `hex_nut`, `bolt` (5.4). `Align` · `Grasp`의 `target_class`와 같은 이름 사용
- [ ] DOFBOT 그리퍼가 들 수 있는 무게 (B) — 벌림 폭은 위 항목에서 확정(약 6 cm). 무게는 Gazebo에서 시험용 질량으로 마찰 · 파지 안정성을 검증한 뒤 결정
- [ ] 물체 놓는 방법 (세움/눕힘, 바닥/작업대 높이) (A, B, D)
- [ ] 파지 후 물체를 지정된 위치에 내려놓고 관측 자세로 복귀하는 단계 (A, B) — B 언급 (2026-11-09). 내려놓을 위치, 상태머신 단계, 인터페이스 미정
- [x] 목표 클래스 지정 방식 → `/perception/target_class` (`std_msgs/msg/String`, `transient_local`)
- [x] 관측 자세(observe pose) 관절 값 (B, C) → far · near · pre-grasp 관절 값 확정 (5.6, 2026-11-15). 카메라가 `link4`라 joint1~4 값에 따라 시점이 바뀌므로 C의 합성 데이터 시점 기준이 됨
  - [ ] MoveIt SRDF에 자세 이름 등록 (B) — 지금은 `observations/` 파일명만 참고용
  - [ ] 팔을 관측 자세로 보내는 액션 (B) — 상태머신이 요청, B가 MoveIt2로 실행 후 성공 여부 반환 (4장). A 초안 PR #5 리뷰 대기
- [x] pre-grasp 숙임각 69.3° 반영 (A, B) — B가 `feat/grasp`에서 약 84° → 69.3°로 변경(5.6 참고). 5.6 표 · 카메라 TF 반영 완료(B 확인, PR #1 답변, 2026-11-21). 황동 pin 재학습·검출 재시험은 별도 진행 중(인수인계 문서 참고)
- [ ] grasp 자세 카메라 TF (B) — 5.6 표에 아직 없음
- [x] 정렬 완료 거리를 조정 (D) — C 요청(PR #3, 당시 약 0.22~0.28 m)에서 시작해, near 0.40 m / pre-grasp 0.35 m 2단계 값으로 확정(5.6, D 제안 PR #1 id 7066535320 · C 동의 id 7075009411, 2026-11-21~22). 재시도별 값(5.2)과 Nav2 목표 거리(5.5)도 함께 갱신
- [ ] 차체 · 라이다 · 팔 베이스 치수 정식 반영 (A, B) — PR #4 머지 후. caster_gap 2 mm는 `feat/grasp` · PR #4 둘 다 반영됨
- [x] MoveIt 계획 그룹 → `dofbot_arm` (joint1~5, KDL) (B)
- [x] MoveIt 기본 자세 → `home` (관절 전부 0, 1자 자세) (B)
- [x] 검출 발행 주기 Hz → `target_pose` 최소 3 Hz, 목표 10 Hz (3.3). D 정렬 모의 시험 · C OpenVINO 실측으로 5 Hz에서 변경 (PR #3, 2026-11-19)
- [x] 공용 launch 파일에 렌더러 옵션 인자 추가 (A) → `software_gl`, `headless`, `rviz_gpu`, `render_engine` (0장)
- [ ] 카메라 해상도 (C) — 320×240(현재 테스트 값)과 640×480을 첫 학습 때 비교, RTF 영향도 함께 확인. YOLOv8n CPU 추론은 640 입력 약 7.8 Hz, 320 입력 약 13.6 Hz (C 측정)
- [x] 상태머신 구현 방식 → 직접 만든 Python 노드 (`mm_task_manager`, `feat/state-machine`). 이동 단계 규칙은 5.5
- [x] `project_interfaces` 패키지 (Align, Grasp 액션) → develop에 머지 완료 (PR #2)
  - [ ] 관측 자세 액션 추가 — PR #5 초안 확정 후
- [x] 상태머신이 Align 응답을 못 받았을 때 → `timeout_sec` + 여유 뒤 취소, `ALIGN_TIMEOUT` (D 확인, 4.1)
- [ ] Grasp 최대 소요 시간 (B) — 상태머신 제한 시간 임시 60 s (B: 60 s보다 적게 걸릴 것 같음). 그리퍼 제어 구현 후 실제 시간 보고 조정
- [ ] Grasp에 넘길 `target_pose`의 최대 나이 (B) — 정렬 완료 후 받은 값 중 stamp 기준 1.0 s 이내. C 발행 주기(최소 3 Hz, 약 0.33 s 간격)와 맞음. B 확인 필요
  - Grasp 목표는 정렬 완료 직후가 아니라, 정렬 뒤 찍힌 target_pose가 올 때까지 최대 1.0s 기다렸다 보냄 (구현 메모 — B 확인 필요, 확정 스펙 아님)
- [x] Nav2 도착 허용 오차 → 그대로 둠. 대신 Nav2 목표를 near 1차 `standoff_m`(0.40 m) 기준으로 margin을 둬 약 0.50 m로 둠 (D, 5.5. margin 근거가 2026-11-21에 갱신됨 — 5.5 참고)
- [x] 목표 근처 맴돌기 → Nav2 설정은 그대로 둠. 다가오는 방향과 목표 yaw가 크게 다르면(약 180°) 도착 후 제자리 회전 중 맴돌 수 있음 (DWB와 goal checker가 서로 다른 시각의 map→odom을 쓰고, 1 Hz 재계획마다 둘 다 초기화됨. Humble 1.1.20 소스로 확인, 2026-11-13). object 모드는 목표 yaw가 물체를 바라보는 방향(= 다가오는 방향)이라 큰 회전이 거의 없고, 남는 yaw 오차는 D 정렬(시작 범위 ±0.35 rad)이 받는다. goal 모드에서 목표 yaw를 정할 때는 다가오는 방향에 가깝게 정한다 (A)
- [ ] 팀 코드 라이선스 (전원) — MIT 또는 Apache-2.0 중 선택. D는 둘 다 괜찮음
- [x] 정렬 완료 판정(CHECKING) 중 검출 끊김 허용 시간 → 멈춘 뒤 최대 3 s(`lost_timeout_stopped`), 움직이는 중엔 1 s (D 확정, 2026-11-22, 4.1 · 5.3)
- [ ] 같은 클래스 다수 물체 구분: Align Goal에 `expected_position`(PointStamped) · `expected_radius_m` 필드 추가 제안 (D, `project_interfaces` 변경 필요) — A 조건부 동의, 좌표계 일치 · 반경 안 미검출 시 처리 · 기존 클래스 필터와의 관계 3가지 명확화 요청, PR #1 댓글, 답변 대기

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
| 2026-11-08 | v0.12 | B 진행 상황 반영: 분리형 그리퍼 DOFBOT 모델을 공식 모델로 확정(0장), TF 트리 · 그리퍼 · 끝점 · 관절 표 갱신(2.1~2.3), 카메라 장착 링크는 논의 중, `arm_base_link` 이름 변경 완료 · MoveIt 재생성 · 그리퍼 분리(2.4), `grip_joint` 범위(2.3), 팔 컨트롤러 액션 확인(4장), 그리퍼 벌림 폭 약 6 cm · `standoff_m` 초기 시험값 0.25 m · 파지 접근 방향 · 관측 자세 방향(5.4, 6장) | A |
| 2026-11-09 | v0.13 | B xacro 반영 확인: 그리퍼 · 끝점 이름 변경 완료, `camera_link` · `camera_optical_frame` 추가(rpy (-90°, 0, 0), 이 모델 카메라는 +Y가 시선 방향), TF 트리를 현재 모델(카메라 `link4`) 기준으로 갱신(2.1, 2.2), 팔 장착 방향 yaw -90° 결정(2.1), 카메라 센서 토픽 이름 차이 메모(2.4), 체크리스트 갱신 | A |
| 2026-11-09 | v0.14 | 상태머신 설계 반영: `/cmd_vel` 발행 규칙(10 Hz, 0.5 s, DiffDrive 측정 근거, 5.0), 실패 코드 `NAV_NOT_READY`와 세부 원인(5.3), 이동 단계 시작 확인 · 도착 확인 · TF 신선도 · 시간 제한 · 재시도 · 결과 기록(5.5, 신규), 상태 토픽 `/task_manager/state`(3장), A 브랜치 `feat/state-machine`. D 의견 반영: v0.11 확인, 관측 자세 시야 조건(6장). 체크리스트 추가(6장) | A |
| 2026-11-09 | v0.15 | C 의견 반영: 카메라 장착 링크 `link4` 확정(C · D 찬성), 카메라 장착 오프셋은 B가 차체 · 팔 합친 뒤 11/11까지 확정, 합성 데이터 시점에 필요한 값 정리(2.1, 2.2), `target_pose` 발행 주기 최소 5 Hz · 목표 10 Hz, stamp는 이미지 촬영 시각, 미검출 처리(3장, 3.3). 상태머신: 시작 확인에 `/cmd_vel` 발행자 충돌(`cmd_vel_conflict`)과 시작 후 1 s 대기, 종료 시 0 발행 후 노드 종료, 남은 노드 없는 재실행 측정값으로 도착 오차 갱신(5.3, 5.5). 체크리스트 갱신(6장). B 답변: `target_pose`는 `base_link` 기준(4.2), Grasp 제한 시간 임시 60 s. D 답변: Align 필드 확정, 응답 없을 때 `ALIGN_TIMEOUT` 처리(4.1), 이동 목표는 물체에서 `standoff_m` + 0.10 m 이상(5.5). B 답변: 관측 자세 이동은 상태머신이 B에 액션으로 요청(4장, B 초안 예정), 내려놓기 단계 언급(6장) | A |
| 2026-11-13 | v0.16 | 5.5 Nav2 목표 margin 0.10→0.15 m 반영 | A |
| 2026-11-15 | v0.17 | far/near 관측 자세 방식 추가 (C 제안, D 동의, 2026-11-15) | A |
| 2026-11-15 | v0.18 | 5.5절 차체 앞면 수치 0.20m → 0.215m 갱신 (PR #4 반영, 라이다 포함) | A |
| 2026-11-16 | v0.19 | 5.6 관측 · 파지 자세 4종(far, near, pre-grasp, grasp) 관절 값 · 카메라 TF · 물체 거리 추가, 5.7 카메라 내부 파라미터 추가, 6장 체크리스트 near 선정 완료 반영 (B 측정, PR #3 댓글 · feat/grasp observations/, 2026-11-15) | A |
| 2026-11-16 | v0.20 | 5.6 관측 자세 전환 순서 추가 (Far → Near 약 0.60 m → Pre-grasp 약 0.35 m → Grasp, B 제안, 희선님 개인 노션, 2026-11-15). 정렬 중 Near→Pre-grasp 전환 요청 방식은 D 확인 중 | A |
| 2026-11-17 | v0.20 | 3장 `/task_manager/state` QoS depth 10 비고 추가 (늦게 붙는 구독자도 과거 상태 이력을 받을 수 있게 늘림. 버전 유지) | A |
| 2026-11-18 | v0.20 | 3.3 `/perception/target_pose` 수신 측(A) 안전장치 필터 비고 추가: `frame_id`가 `base_link`가 아니거나, x/y/z에 NaN/inf가 있거나, x≤0이면 무시 (구현 f523b1f, C 제안 PR #1. 버전 유지) | A |
| 2026-11-19 | v0.21 | 5.6 Near→Pre-grasp 전환 방식 확정: Align 2단계 호출(near 1차 standoff 0.35 m → 정지 상태에서 observe_pose로 pre-grasp 전환 → 2차 최종 standoff), 5.2 재정렬과 별개, PR #5 확정 전까지 코드 미반영. 깊이 이미지 32FC1(m) · NaN/inf 제외 D 재확인 (D 답변, 2026-11-19) | A |
| 2026-11-19 | v0.21 | 3장 `/task_manager/state` QoS depth 10→17 비고 수정 (재정렬 최대 + 취소 경로 최대 12개 + 여유 5, 구독 측도 17 이상 권장. 버전 유지) | A |
| 2026-11-19 | v0.22 | 3장 · 3.3 `/perception/target_pose` 최소 발행 주기 5 Hz → 3 Hz: D 정렬 모의 시험(3 Hz 15/15, 1.5 Hz 7/15, 1 Hz 3/15), C OpenVINO 적용 후 평균 약 4.4 Hz · 최소 3.8 Hz로 충족 (PR #3 댓글, 2026-11-19). 이전 5 Hz는 C의 Gazebo 연동 전 추론 시간 기준값(2026-11-09). 6장 체크리스트 갱신 | A |
| 2026-11-21 | v0.22 | B 통합 모델 RGB-D 카메라 연결 반영: bridge로 3장 `/camera/*` 토픽 연결, 일반 RGB 카메라 `arm_rgb_test` 삭제(2.4, 3장), 카메라 주기 10 Hz → 30 Hz와 내부 파라미터 비고(5.7), 깊이 32FC1 B 확인(1.1, 5.6), RGB-D는 `render_engine:=ogre2` 필요(B WSL 확인, 0장). 토픽 이름 · 메시지 타입 · 프레임은 그대로라 버전 유지 (PR #3 댓글, 2026-11-21) | A |
| 2026-11-21 | v0.22 | Mac VM 깊이 카메라 재시험 반영(0장): `render_engine:=ogre2 software_gl:=true headless:=true`면 이 VM도 `test_room`·`yolo_test_room` 둘 다 깊이·RGB·YOLO 정상(실행 인자가 SDF 센서 설정을 덮어씀), 기본값 ogre는 여전히 비정상, RTF 저하 트레이드오프 명시. 5.7 카메라 실제 발행 주기 실측치(RGB 약 24.5 Hz · 깊이 약 21.8 Hz · camera_info 약 28 Hz, 설정값 30 Hz보다 낮음) 추가. 토픽 이름 · 메시지 타입 · 프레임은 그대로라 버전 유지 | A |
| 2026-11-21 | v0.23 | 5.6 pre-grasp 최종 `standoff_m` 기본값 0.35 m로 확정(6장 체크리스트 반영). 근거: B의 MoveIt2 IK 검증(World X 0.265/0.29/0.32/0.35 m 전부 16/16 성공, PR #1)과 VM 접촉 경계 정밀 시험(물체 4종 × Pre-grasp/Grasp 2자세 격리 시험, 0.29~0.35 m 전부 주행 접촉 없음 확인, 단 Grasp 자세 기준 pin · bolt는 0.29 m 여유가 0.9~1.3 cm로 타이트) 종합. 다른 모듈(B 파지 접근, D 정렬 목표)이 따라야 할 기준값이라 버전 상향 | A |
| 2026-11-21 | v0.24 | 2.1 `base_link → arm_base_link` 장착 위치 xyz (0.100, 0, 0.010) m 확정, 카메라 장착 오프셋(`link4 → DaBai_DCW2_Link`)을 통합 모델 기준 공식값으로 확정(B, PR #1 답변 id 7048233384, 2026-11-20). 6장 체크리스트 두 항목 완료 반영. 다른 모듈(C 합성 데이터, D 정렬 목표 TF)이 참고하는 기준값이라 버전 상향 | A |
| 2026-11-21 | v0.25 | 6장 체크리스트를 현재 상태와 맞춤: 관절 이름 `gripper_joint`, joint2 · joint4 해결 근거, 관측 자세 관절 값 완료(SRDF · 액션은 하위 항목), `project_interfaces` 머지 완료, 그리퍼 항목 중복 정리, 새 미정 항목 추가(pre-grasp 숙임각 69.3°, grasp 카메라 TF, 정렬 완료 거리, 차체 치수, 같은 클래스 구분에 C 제안 추가). 2.2 카메라 오프셋 상태, 3.3 x 상한 · 점프 필터, 4.1 `timeout_sec` 기본 30 s, 5.5 이동 목표 거리 예시를 0.35 m 기준으로, 5.6 pre-grasp 자세 변경 메모 추가 | A |
| 2026-11-21 | v0.26 | B의 v0.25 리뷰(PR #1) 반영: 5.6 pre-grasp 관절 값 joint4 -1.435083 → -1.190737 표에 반영, 카메라 TF도 69.3° 기준 실측값으로 갱신(xyz (0.303, 0.001, 0.150), rpy (-2.778, 0, -1.571)), Near→Pre-grasp 전환 설명을 "차체 정지·팔만 이동, 완료 후 align 재호출" 기준으로 정정. 5.7 카메라 주기 30 Hz → 10 Hz(실제 설정값)로 정정, 관련 실측치는 참고용으로 표시(10 Hz 기준 재실측 필요). 2.4 팔 · 그리퍼 inertial, 그리퍼 링크 · 관절 · `ee_link` 항목 완료로 변경, Gazebo 그리퍼 mimic 관절 연동 검증 항목 신설. 6장 체크리스트: pre-grasp 숙임각 69.3° 반영 항목 완료 처리 | A |
| 2026-11-22 | v0.27 | 정렬 완료 거리를 near(1차 Align)/pre-grasp(2차 Align) 2단계 값으로 분리: near `standoff_m` 0.35→0.40 m, pre-grasp는 0.35 m 유지(5.6). 근거: near 0.35 m에서는 pin 검출 score가 0.5 임계값에 못 미치는 경우가 있었고, 0.40 m는 near·pre-grasp 69.3° 화면 가시 범위 양쪽에 들어옴 (D 제안 PR #1 id 7066535320, C 동의 id 7075009411, 2026-11-21~22). 5.2 재시도별 `standoff_m` 하한을 0.32 m 이상으로 명시(0.29 m는 pre-grasp 화면 가시 범위 밖이라 제외, D·C 확인). 5.5 Nav2 목표 거리 기준을 near `standoff_m`(0.40 m) 기준으로 재정리, 목표값 약 0.50 m는 유지되나 margin 근거가 "도착 오차 0.15 m"에서 "near 화면 가시 범위 0.10 m"로 바뀌어 두 근거가 완전히 일치하진 않음(확인 필요로 남김). 6장 체크리스트: 기본 `standoff_m`, 재시도별 `standoff_m`, 정렬 완료 거리, Nav2 도착 허용 오차 항목 갱신. 같은 클래스 다수 물체 구분 방법은 B 의견 대기 중이라 미반영(그대로 둠) | A |
| 2026-11-23 | v0.28 | 정렬 완료 판정(CHECKING) 중 검출 끊김 허용 시간 추가: 차체 정지 중 최대 3 s(`lost_timeout_stopped`), 움직이는 중엔 1 s, 넘으면 `OBJECT_NOT_FOUND`(D 확정, 2026-11-22, 4.1 · 5.3). 같은 클래스 다수 물체 구분용 Align Goal `expected_position`·`expected_radius_m` 필드 추가 제안(D)은 A가 방향엔 조건부 동의했으나 좌표계 일치·반경 안 미검출 처리·기존 클래스 필터와의 관계 3가지 명확화를 요청한 상태라 본문에는 아직 반영하지 않고 6장 체크리스트에만 진행 상황 기록(`project_interfaces` 변경 필요, D 답변 대기) | A |
