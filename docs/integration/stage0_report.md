# Nav2-Servo 통합 0단계 조사 결과 (2026-11-19)

대상: feat/grasp 30cf477 (`mm_robot.urdf.xacro`, `worlds/yolo_test_room.sdf`), 지도·Nav2 설정은 PR #4(feat/navigation-chassis-update 7e2a47c).
환경: Mac UTM VM(aarch64). 결과물은 scratchpad에만 있고, 커밋·push 없음. 다른 브랜치는 읽기만 함.
준비: feat/grasp를 detached worktree로 열고, 부품 모델(feat/yolo)과 PR #4 mm_navigation은 `git archive`로 꺼냄. 빌드는 scratchpad 별도 워크스페이스(우리 build/install 미사용).

## 요약

| 단계 | 결과 | 핵심 숫자 |
|---|---|---|
| 0-1 통합 모델 실행 · RTF | 됨 | RTF 중앙값 1.000, 평균 0.976, p5 0.839, 0.9 미만 11.7% (ogre + software_gl + headless) |
| 0-2 /scan에 팔이 잡히는지 | 됨 (팔에 맞는 빔 없음) | 기본 · Near 둘 다 최소 1.313 m (팔 없는 차체 기준 1.31 m와 같음), 뒤쪽 0.35 m 미만 0%, inf/NaN 0%, 0.12 근처 0% |
| 0-3 RGB-D 깊이 | 안 됨 (11/20, 30cf477) → 됨 (11/21 갱신, b9e8a55 + `render_engine:=ogre2`) | 11/20: 모든 렌더러 조합에서 픽셀 100% inf, RGB-D 컬러 회색. 11/21: ogre2 + software_gl + headless에서 깊이 100% 유효, 부품 오차 Far 3~4 mm · Near/Pre-grasp 핀 윗면 1 mm 이내. 기본값 ogre는 여전히 회색 · inf (아래 "0-3 재시험") |
| 0-4 Nav2 도착 | 안 됨 (11/19, +90° 목표) → 원인 확인됨 (11/20 갱신) | 11/19: Gazebo 실제 위치는 목표에서 3.4 cm · 1.0° 안. recovery 17회, "Failed to make progress" 10회. 11/20: 목표 근처 큰 회전 + progress checker가 원인. object 모드 목표(-8.5°)는 통합 모델 3/3 성공, 3.6~5.2 s, recovery 0 (아래 "0-4 원인 확인") |
| 0-4 costmap 자기 몸 | 됨 (안 잡힘) | footprint 안 · 바깥 0.1 m · 뒤쪽 0.4 m 모두 장애물 칸 0 (기본 · Near) |
| 0-4 부품 밀림 | 검증 안 됨 (11/21 정정) | 부품이 static 모델이라 밀림 여부 자체가 검증되지 않았음 (block · pin · hex_nut · bolt 모두 `<static>true</static>`, 원래 움직일 수 없음). 부품은 costmap에도 안 잡힘 |
| 접촉 경계 · standoff 후보 (11/21 추가, caster_gap 2mm 재검증 11/21 갱신) | 됨. 2mm에서도 경계 차이 1 mm 미만, standoff 0.35 m 안전 (최소 여유 Grasp pin 7.0 cm) | Pre-grasp 경계 0.186~0.203 m (차체 · 캐스터), Grasp 경계 pin 0.281 · bolt 0.277 m (팔 · 그리퍼), block · hex_nut 0.203 · 0.186 m. 후보 0.35/0.32/0.29 m는 8개 조합 모두 접촉 없음, 단 Grasp의 pin · bolt는 0.29 m 여유 0.9 · 1.3 cm (아래 "접촉 경계 · standoff 후보") |

## 0-1. 통합 모델 실행, RTF

- 명령: `sim.launch.py software_gl:=true headless:=true rviz:=false world:=yolo_test_room.sdf` (렌더러 기본 ogre)
- 로드됨: mm_robot, 부품 4개(test_block, test_pin, test_hex_nut, test_bolt), 벽 · 장애물 3개. 생성까지 약 3 s
- RTF (/stats 30 s, 마지막 20000개): 중앙값 1.000, 평균 0.976, p5 0.839, p1 0.772, 최소 0.436, 0.9 미만 11.7%
- 토픽 주기(wall): /scan 9.6 Hz, /odom 28.5 Hz, /camera/image_raw 8.7 Hz, /camera/camera_info 9.7 Hz, /joint_states 약 897 Hz, /clock 약 900 Hz
- 참고
  - /joint_states가 물리 주기(약 900 Hz)로 브리지됨. 지금은 문제없지만 노드가 늘면 부담이 될 수 있음
  - Gazebo 기준 로봇이 pitch 0.020 rad(약 1.15°) 기울어 있음 (원점 정지 상태). DiffDrive odom과 TF는 이 기울기를 모름
  - 라이다 TF: base_link 기준 (0.180, 0, 0.075), base_link는 base_footprint 위 0.050 → 지면에서 약 0.125 m

## 0-2. /scan에 팔이 잡히는지

라이다 360개, range_min 0.12, range_max 8.0. 자세마다 20개씩 모음. 뒤쪽 = |각도| > 90°.

| 자세 | 전체 최소 | 뒤쪽 최소 | 뒤쪽 0.35 m 미만 | 뒤쪽 inf/-inf/NaN | 뒤쪽 0.12 근처 | 팔 때문에 안 보이는 각도 |
|---|---|---|---|---|---|---|
| 기본 (관절 0) | 1.313 m | 1.572 m | 0% | 0% | 0% | 없음 |
| Near (원점) | 1.313 m | 1.572 m | 0% | 0% | 0% | 없음 |
| Near (0-4 목표 위치) | 1.173 m | 1.732 m | 0% | 0% | 0% | 없음 |

- 비교 기준(11/15 팔 없는 차체, 최소 1.31 m)과 같은 값. 앞쪽 최소 1.313 m는 벽 · 장애물이고, 0.8 m 앞 부품은 안 잡힘
- 예상(팔 밑판 앞끝 x 0.1788이 라이다 x 0.18에 붙어 있어 뒤쪽 빔이 inf/0.12로 나옴)과 다르게 팔이 전혀 안 보임
  - 추정 원인 (확인 필요): gpu_lidar는 렌더링 방식이라 range_min(0.12 m)보다 가까운 면은 잘려 나가고, 밑판 안쪽에서 보는 뒷면은 그려지지 않아 빔이 팔을 통과하는 것으로 보임
  - 시뮬에서는 costmap에 자기 몸이 안 잡혀 유리하지만, 실물 라이다라면 뒤쪽이 가려질 구조라는 점은 기억해 둘 것
- 부품(높이 1.5~6 cm)은 라이다 높이(약 0.125 m)보다 낮아서 /scan에 안 잡힘

## 0-3. RGB-D 깊이 (scratchpad 전용 브리지)

브리지: `/rgbd_camera/depth_image → /camera/depth/image_raw`, `/rgbd_camera/camera_info → /camera/depth/camera_info` (sim.launch.py는 수정 안 함).

| 조합 | Gazebo 실행 | 깊이 (32FC1) | RGB-D 컬러 | 일반 RGB(arm_rgb_test) | 라이다 |
|---|---|---|---|---|---|
| (a) ogre + software_gl + headless (기본) | 됨 | inf 100% | 한 가지 색(178), std 0 | 정상 (색 20가지, std 72) | 정상 1.313 m |
| (b) ogre2 + LIBGL_ALWAYS_SOFTWARE=1 + headless | 됨 | inf 100% | 한 가지 색(218) | 한 가지 색(218) | 정상 1.31 m |
| (c) ogre2 + GPU + headless | 시작 직후 종료 (`GLXBadFBConfig`) | 판단 불가 | — | — | (판단 안 함) |
| (d) ogre + GPU + headless | 됨 | inf 100% | 한 가지 색(178) | 정상 (색 23가지) | 0.12 고정 (기존 문제) |

- (b) · (c)는 월드 Sensors 플러그인의 `<render_engine>ogre</render_engine>`를 ogre2로 바꾼 scratchpad 복사본으로 실행 (원본 월드는 ogre 고정)
- encoding · 크기 · frame_id는 맞음: 32FC1, 640×480, `camera_optical_frame`, K = fx 554.38, fy 554.38, cx 320, cy 240
- 센서 방향은 맞음: `rgbd_camera`(DaBai_DCW2_Link + rpy 3.14, 3.14, -1.57)의 시선이 camera_optical_frame과 같음 (Far에서 둘 다 아래로 22.4~22.5°)
- 원인 범위
  - ogre(1)에서는 일반 카메라는 되고 rgbd_camera만 회색 · inf → software/GPU와 상관없이 ogre + rgbd_camera 조합 문제로 보임
  - ogre2는 이 VM에서 software면 모든 카메라가 회색, GPU면 실행 자체가 안 됨
  - 결론: 이 VM에서는 깊이가 나오는 조합을 못 찾음. 다은님 월드(ogre2)에서는 깊이가 잘 나온다는 원준님 댓글(PR #3, 11/19)과도 맞음
- 정답 비교 (측정값이 없어 오차 계산은 못 함, 화면 안에 들어오는지와 기대 깊이만)
  - 원점 + Far(아래로 22.4°): 4개 모두 화면 안
    - Gazebo 실제 자세(pitch 0.020) 반영: block (320.7, 274.9) 0.600 m, pin (432.4, 266.6) 0.596 m, hex_nut (211.6, 295.2) 0.610 m, bolt (95.9, 258.2) 0.592 m
    - odom 기준(기울기 무시)으로 계산하면 같은 점이 약 13 px 아래, 기대 깊이 약 7 mm 차이 → pitch 0.020이 TF에 없다는 점이 투영에 바로 보임
  - 0-4 도착 위치 + Near(아래로 57.9°): 로봇에서 핀까지 0.396 m(Gazebo 기준). 핀 윗면 (318.2, 218.0) 0.240 m, 핀 바닥 (318.4, 271.8) 0.283 m로 화면 중앙. block 윗면도 (319.0, 48.4)로 화면 위쪽 끝에 들어옴
  - 원점에서 Near 자세는 이번에 투영하지 않음 (요청대로 Far와 도착 후 Near만 봄)

## 0-3 재시험 (11/21 갱신, feat/grasp b9e8a55)

희선님 수정(3161f10 test_room.sdf 센서 render_engine ogre2, 35e9331 sim.launch.py에 RGB-D 브리지 추가, 2052c89 일반 RGB 카메라 arm_rgb_test 삭제) 기준. sim.launch.py가 `/rgbd_camera/*`를 `/camera/image_raw`, `/camera/depth/image_raw`, `/camera/camera_info`로 리매핑하므로 별도 브리지 없이 측정. launch 인자 이름은 `--show-args`로 확인 (`world`, `render_engine` 기본 ogre, `software_gl`, `headless`, `rviz`). 월드 SDF의 센서 render_engine은 test_room ogre2, yolo_test_room은 아직 ogre.

원점 + Far 자세 (30 s 주기, 60 s RTF):

| 조합 | 실행 | RGB | 깊이 유효 비율 | 부품 깊이 (기대 vs 측정) | 카메라 Hz (wall) | RTF 중앙값 / 하위 5% | YOLO (OpenVINO f32, 20 s) |
|---|---|---|---|---|---|---|---|
| A. test_room + ogre2 + software_gl + headless | 됨 | 정상 (색 44가지, std 70) | 100% (최소 0.394, 중앙 0.784, 최대 2.587 m) | 부품 없음 | RGB 24.4, 깊이 21.4 | 0.996 / 0.748 | 안 함 |
| B. yolo_test_room + ogre2 + software_gl + headless | 됨 | 정상 (색 61가지, std 70) | 100% | block -3.0 mm, pin -3.5 mm, hex_nut +9.5 mm, bolt -3.2 mm (기대 0.59~0.61 m) | RGB 24.5, 깊이 21.8 | 0.994 / 0.676 | 4종 모두 217/217 (block 0.948, hex_nut 0.904, bolt 0.883, pin 0.862), 10.8 Hz |
| C. yolo_test_room + ogre (기본) + software_gl + headless | 됨 | 회색 한 가지 색 (178, std 0) | 0% (inf 100%) | 측정 불가 | RGB 22.8, 깊이 22.8 | 0.999 / 0.568 | 안 함 (RGB 비정상) |
| D. GPU 조합 | 생략 (A, B에서 깊이 정상) | — | — | — | — | — | — |

- 세 조합 모두 encoding RGB `rgb8` · 깊이 `32FC1`, 640×480, `frame_id` `camera_optical_frame`, K fx 554.38, cx 320, cy 240
- B에서 SDF는 ogre인데 실행 인자 ogre2로 `ignition-rendering-ogre2`가 로드됨 → 실행 인자가 SDF 센서 설정을 덮어쓰는 것으로 보임. 단 launch 기본값이 ogre라 인자를 빠뜨리면 C처럼 됨
- ogre2 로그에 `libEGL warning: Not allowed to force software rendering when API explicitly selects a hardware device` → software_gl을 줘도 실제로는 GPU로 그리는 것으로 보임. 그래도 B에서 라이다는 정상 (최소 1.31 m, 0.12 근처 0%)
- hex_nut +9.5 mm는 너트 가운데 구멍으로 아래가 보여서일 가능성 (추정)
- 일반 RGB 카메라가 삭제되어, ogre 조합에서는 `/camera/image_raw`(RGB-D 컬러)가 회색이라 YOLO 입력이 망가짐
- B에서 측정 시작 2 s 시점 발행자 수가 0으로 나옴 (메시지는 정상 수신, 확인 필요)

Near · Pre-grasp 자세 (B 조합, Nav2 없이 원점에서 핀 정면으로 회전 후 `/cmd_vel` 직진, 5×5 중앙값):

| 자세 | 핀까지 거리 | 핀 윗면 기대 vs 측정 | 핀 중간 높이 기대 vs 측정 |
|---|---|---|---|
| Near (아래로 57.9°) | 0.449 m | 0.2671 vs 0.2664 (-0.7 mm) | 0.2885 vs 0.2739 (-14.6 mm) |
| | 0.399 m | 0.2412 vs 0.2407 (-0.5 mm) | 0.2626 vs 0.2443 (-18.3 mm) |
| | 0.349 m | 0.2154 vs 0.2152 (-0.2 mm) | 0.2369 vs 0.2108 (-26.1 mm) |
| | 0.298 m | 0.1895 vs 0.1894 (-0.1 mm) | 0.2110 vs 0.1875 (-23.5 mm) |
| Pre-grasp (아래로 83.2°) | 0.399 m | 화면 밖 | 화면 밖 |
| | 0.348 m | 0.1330 vs 0.1329 (-0.1 mm) | 0.1579 vs 0.1320 (-25.9 mm) |
| | 0.299 m | 0.1282 vs 0.1282 (0.0 mm) | 0.1530 vs 0.1281 (-24.9 mm) |
| | 0.248 m | 화면 밖 (아래) | 0.1480 vs 0.1239 (-24.1 mm) |
| | 0.213 m (0.20 목표, 핀에 막힘) | 화면 밖 | 화면 밖 |

- 모든 지점에서 깊이 100% 유효. 보이는 면(핀 윗면, Near의 block 윗면 -0.3~-0.4 mm)은 1 mm 이내로 정확
- 중간 높이 · 바닥이 15~50 mm 가깝게 나오는 것은 깊이 오류가 아니라, 그 픽셀이 원기둥 중심이 아닌 앞쪽 옆면 · 윗면을 재기 때문 (D의 "깊이가 앞면까지" 현상). 정렬 쪽 `center_offsets`(pin 10 mm) 보정으로 충분한지는 쓰는 픽셀에 따라 다름 (확인 필요)
- Pre-grasp에서 핀이 화면에 들어오는 구간은 약 0.35~0.25 m (0단계 계산과 일치)

## 0-4. Nav2 도착, costmap, 부품

- 명령: PR #4 `nav2.launch.py software_gl:=true headless:=true rviz:=false standalone:=true world:=yolo_test_room.sdf`
  - CLI의 `world:=`가 안쪽 sim.launch.py까지 전달됨 (부품이 로드됨). launch 복사본은 필요 없었음
  - 지도: feat/navigation과 PR #4의 test_room.pgm이 같음(md5). yolo 월드와 벽 · 장애물 일치(test_room.sdf에만 있는 3 cm 큐브 1개 차이)
  - AMCL 초기 위치 (0, 0, 0) 파라미터로 설정됨, map→odom 정상
- footprint: 설정 x -0.20~+0.24, y ±0.24. published_footprint는 (0.25, ±0.25), (-0.21, ±0.25) (패딩 0.01로 보임)
- 목표 (0.80, -0.55, +90°) (핀에서 남쪽 약 0.43 m, 핀을 바라봄)
  - 결과: 120 s 시간 제한까지 성공 판정 없음 → 취소. 마지막 남은 거리 0.058 m, recovery 17회
  - 로그: "Failed to make progress" 10회, behavior_server backup 2, spin 2, wait 2
  - Gazebo 실제 위치 (0.8023, -0.5163, yaw 1.588) → 목표 대비 3.4 cm, 1.0°. AMCL 위치 (0.7746, -0.5166, 1.582) → Gazebo와 x 2.8 cm 차이
  - 바퀴 관절 누적 회전: 왼쪽 +278 rad, 오른쪽 -239 rad (목표 근처에서 오래 돌았다는 뜻)
  - 원인 후보 (확인 필요)
    1. progress_checker `required_movement_radius` 0.5 m / 10 s에 비해 목표 근처 미세 조정 거리가 짧음 (xy 허용 0.05 m)
    2. 다가오는 방향(약 -34°)과 목표 yaw(+90°)가 약 124° 차이 → interfaces.md 6장 "목표 근처 맴돌기" 조건과 비슷
    3. 통합 모델 영향(무게 · pitch 0.020 기울기)일 가능성도 배제 못 함. 팔 없는 차체로 같은 목표를 돌려 비교해야 구분됨
- 0-4 원인 확인 (11/20 갱신). 시행마다 시뮬 새로 실행, 원점 출발, 목표 120 s 제한. 설정 변경 없음
  - 팔 없는 차체는 scratchpad 별도 워크스페이스(ws_base: PR #4의 mm_description + mm_navigation, test_room 월드)

| 조건 | 결과 | 걸린 시간 | recovery | 실제 오차 (Gazebo xy / yaw) |
|---|---|---|---|---|
| 팔 없는 차체, (0.80, -0.55, +90°) | 3/3 성공 | 17.9~20.8 s | 1회씩 ("Failed to make progress" 1회씩) | 2.6~5.4 cm / 7.0~7.9° |
| 통합 모델, (0.80, -0.55, -34.5° = 다가오는 방향) | 3/3 성공 (가짜 성공 1회는 제외하고 1회 추가) | 7.9~9.7 s | 0 | 4.2~7.6 cm / 6.2~6.8° |
| 통합 모델, object 모드 목표 (0.404, -0.061, -8.53°, 핀에서 0.40 m) | 3/3 성공 | 3.6~5.2 s | 0 | 4.5~5.2 cm / 4.4~7.3° |
| (참고) 통합 모델, (0.80, -0.55, +90°) | 0/1 (11/19) | 120 s 초과 | 17 | 3.4 cm / 1.0° |

  - 판정: 후보 1(progress checker)이 실패가 일어나는 방식, 후보 2(다가오는 방향과 목표 yaw 약 124° 차이)가 실패 조건. 후보 3(통합 모델)은 혼자서는 원인이 아니고, +90°에서 회복을 못 하게 만드는 쪽으로 영향이 있어 보임 (통합 모델 +90°는 1회뿐이라 확정 아님)
  - object 모드 목표(상태머신이 실제로 쓰는 방식)는 그 뒤 같은 목표로 9회 더 돌려도 모두 성공 (가짜 성공 1회 제외)
  - 성공 판정은 AMCL 기준(xy 0.05 m, yaw 0.125 rad)이라 실제 오차는 xy 4.5~5 cm로 허용치 경계, 목표보다 약 5 cm 덜 감. 핀까지 실제 거리는 약 0.45 m (설계 0.40 m)
- Nav2 가짜 성공 (11/20 갱신): 2회 발생 (통합 모델 -34.5° 2회차, Pre-grasp 전진 시험 1회)
  - 목표를 받자마자 controller_server가 `Transform data too old when converting from map to odom` (데이터 시각 7.0~7.5 s, TF 시각 2.7~4.4 s) 뒤 `Reached the goal!` → 약 60 ms 만에 SUCCEEDED, 로봇은 원점 그대로
  - 원인: 시작 직후 map→odom이 오래된 상태에서 목표를 보냄. 시험 스크립트가 map→odom이 "있는지"만 보고 "최신인지"는 안 봤음. 이후 준비 확인 뒤 8 s 더 기다리게 바꾼 뒤로는 0회
  - 상태머신은 도착 확인(`arrival_check`)과 TF 신선도(`tf_stale`) 검사가 있어 이 경우 NAV_FAILED로 잡을 것으로 보임 (확인 필요)
- costmap 자기 몸 (local_costmap, 0.05 m 칸)

| 위치 · 자세 | footprint 안 (장애물/전체) | 바깥 0.1 m 띠 | 뒤쪽 사각지대 x -0.20~-0.60, |y|≤0.24 | 1 m 안 장애물 칸 |
|---|---|---|---|---|
| 원점, 기본 | 0/90 | 0/92 | 0/80, unknown 0 | 없음 |
| 목표, 기본 | 0/81 | 0/90 | 0/80, unknown 0 | 없음 |
| 목표, Near | 0/81 | 0/90 | 0/80, unknown 0 | 없음 |

  - 자기 몸은 안 잡힘 (0-2에서 라이다가 팔을 못 보는 것과 일치)
  - 뒤쪽 사각지대는 전부 free로 표시됨. 라이다가 팔을 통과해서 보기 때문에 지금은 사각지대가 없음 (실물이면 다를 수 있음)
  - 핀이 0.40 m 앞에 있어도 1 m 안 장애물 칸 0 → 부품은 costmap에 안 잡힘
- 부품 밀림: 부품이 static 모델이라 밀림 여부 자체가 검증되지 않았음 (11/21 정정. Nav2 전후 Gazebo 위치는 block 0.8, 0 / pin 0.8, -0.12 / hex_nut 0.8, 0.12 / bolt 0.8, 0.24, z 0으로 같았지만 static이라 당연한 결과)
  - 정정 (11/21): yolo_test_room.sdf에서 부품 4개가 모두 `<static>true</static>`로 들어가 있어 Gazebo에서 움직일 수 없는 고정 모델임. 따라서 "위치 변화 없음"은 밀리지 않았다는 근거가 아님 (이후 시행들의 "부품 0.0 mm"도 같음)
  - 대신 접촉은 로봇 쪽에서 드러남: 11/21 Pre-grasp 깊이 시험에서 핀 쪽으로 직진할 때 로봇이 핀 중심에서 0.213 m 지점에 멈춤 (odom은 이동했다고 보고, 실제는 헛바퀴, yaw -4.5°로 틀어짐). 차체 앞면인지 팔·그리퍼인지는 확인 못 함
    - 갱신 (11/21): 핀만 둔 월드로 다시 재니 Pre-grasp 핀 접촉 경계는 0.2006 m (차체 앞면). 0.213 m는 핀 옆 0.12 m에 있던 block에 폭 0.28 m 차체가 먼저 닿은 것으로 보임 (추정, "접촉 경계 · standoff 후보")
  - costmap에 부품이 안 잡히므로, 실제(움직이는) 부품이면 경로가 부품 위를 지나갈 때 밀거나 부딪힐 수 있음. 밀림을 확인하려면 static을 끈 월드 복사본이 필요함

## 접촉 경계 · standoff 후보 (11/21 추가)

- 조건: feat/grasp b9e8a55 통합 모델, scratchpad 월드 복사본 4개(worlds_contact/contact_*.sdf, yolo_test_room에서 부품 include를 빼고 물체 하나만 (0.6, 0)에 static으로 둠). 렌더러 기본(ogre) + software_gl + headless. Nav2 없이 원점에서 +x로 `/cmd_vel` 직진
- 거리 기준: 로봇 중심(base_footprint)에서 물체 중심까지 = 희선님 기록의 World X (로봇 원점, 물체를 World X에 둠. grasp_pose_01 기준 큐브 0.265)
- 방법: 0.02 m/s로 0.35 · 0.32 · 0.29(후보) · 0.25 · 0.23 · 0.21 m에서 멈추며 헛바퀴(Gazebo 이동 / odom 이동), yaw, pitch 확인 → 접촉이 처음 보이면 직전 지점으로 물러났다가(없으면 0.21 m부터) 0.003 m/s로 기어가며 경계 측정 (샘플 간격 약 0.8 mm). 판정 기준: 헛바퀴 비율 < 0.7(기어갈 때 1 s 창 < 0.5), |yaw 변화| > 0.3°, |pitch 변화| > 0.0015 rad
- 8개 조합 모두 경계에서 pitch가 0에서 -0.003~-0.012 rad로 갑자기 바뀜 (차체 앞이 물체에 걸려 들림). 그 전까지 pitch 변화는 정확히 0
- 차체 충돌 상자 0.38 × 0.28 × 0.06 m (바닥에서 0.02~0.08 m, 앞면 x 0.19), 앞 캐스터 구 반지름 0.02 (x 0.15, 앞끝 0.17, 바닥 0.3~4.3 cm)

접촉 경계:

| 물체 (높이) | 차체 · 캐스터만 닿는다고 본 계산값 | Pre-grasp 경계 | Grasp 경계 | 닿는 부위 (추정) |
|---|---|---|---|---|
| pin (5 cm) | 0.200 m | 0.2006 m | 0.2811 m | Pre-grasp 차체 앞면 / Grasp 팔 · 그리퍼 |
| block (4 cm) | 0.2025 m | 0.2031 m | 0.2030 m | 두 자세 모두 차체 앞면 |
| hex_nut (1.5 cm) | 0.1865 m | 0.1858 m | 0.1855 m | 두 자세 모두 앞 캐스터 (차체 밑으로 들어감) |
| bolt (6 cm) | 0.196 m | 0.1968 m | 0.2774 m | Pre-grasp 차체 앞면 / Grasp 팔 · 그리퍼 |

- Pre-grasp: 지정 지점 0.25 · 0.23 · 0.21 m에서 4종 모두 접촉 없음. 경계가 차체 · 캐스터 계산값과 1 mm 안으로 맞음 → 이 자세에서는 팔이 물체에 안 닿고 차체가 먼저 닿음
- Grasp: pin · bolt는 0.25 m에서 이미 접촉 (bolt 헛바퀴 비율 0.33) → 0.29 m로 물러나 경계 측정. 키 큰 물체(5 · 6 cm)만 약 0.28 m에서 그리퍼 쪽에 닿고, block(4 cm) · hex_nut은 그리퍼 밑으로 지나감 → 그리퍼 가장 낮은 곳은 바닥에서 4~5 cm 사이 (추정)

후보별 여유 (후보 - 접촉 경계). 실제 멈춘 위치는 후보보다 약 1 mm 덜 간 0.348~0.349 / 0.319~0.320 / 0.288~0.290 m. 8개 조합 모두 세 후보에서 접촉 없음:

| 자세 | 물체 | 0.35 m | 0.32 m | 0.29 m |
|---|---|---|---|---|
| Pre-grasp | pin | 14.9 cm | 11.9 cm | 8.9 cm |
| | block | 14.7 cm | 11.7 cm | 8.7 cm |
| | hex_nut | 16.4 cm | 13.4 cm | 10.4 cm |
| | bolt | 15.3 cm | 12.3 cm | 9.3 cm |
| Grasp | pin | 6.9 cm | 3.9 cm | 0.9 cm |
| | block | 14.7 cm | 11.7 cm | 8.7 cm |
| | hex_nut | 16.5 cm | 13.5 cm | 10.5 cm |
| | bolt | 7.3 cm | 4.3 cm | 1.3 cm |

- 물체별 차이: Pre-grasp는 1.7 cm 차이(hex_nut 0.186 ~ block 0.203)로 차체 · 캐스터와 물체 반지름만으로 정해짐. Grasp는 pin · bolt가 약 8 cm 더 멀리서 닿음 → Grasp 자세로 주행한다면 0.29 m는 pin · bolt 여유 1 cm 안팎이라 정렬 오차(지난 시행 xy 4~5 cm)를 생각하면 부족, 0.32 m(여유 3.9~4.3 cm) 이상이 안전
- hex_nut vs 노션 기록(0.265~0.32 m, 위에서 집는 자세 조건): 주행 접촉 경계 0.186 m라 기록 범위 전체에서 주행 중 접촉 없음 (0.265 m에서도 여유 약 7.9 cm). 시뮬 충돌 모델은 구멍 없는 원기둥(반지름 1.65 cm)이고 구멍은 겉모양(mesh)에만 있어서, 구멍 때문에 접촉 지점이 달라지는 효과는 이 시험에서 안 나타남. 위에서 실제로 집을 수 있는지는 범위 밖
- 확인 못 한 것: 조합마다 1회, 정면 접근만 (옆으로 치우친 물체 없음). 물체 static이라 밀리는 경우 미반영. 팔을 그 자세로 둔 채 주행한 경우만 봄 → standoff에 멈춘 뒤 Pre-grasp에서 Grasp로 팔을 내리는 동안의 접촉(희선님이 말한 충돌 문제)은 범위 밖

### caster_gap 2mm 재검증 (11/21 갱신, feat/grasp 5741526)

- 대상: 11b577d caster_gap 3mm → 2mm, 5741526 Pre-grasp 숙임각 69.3°(joint4 -1.435083 → -1.190737, 예전 자세는 set_pre_grasp_pose_84deg.sh) · ogre2 · RGB-D 10 Hz. 위 3mm 결과의 Pre-grasp는 84° 자세 기준이라 비교용
- 방법 · 판정 기준은 위와 같음. 시험 월드는 새 yolo_test_room.sdf로 다시 만듦 (worlds_contact2/). 8개 조합 모두 목표 자세 첫 시도에 도달. 카메라 숙임각 Pre-grasp 69.2°, Grasp 88.0°
- 정지 상태 pitch 0.0200 → 0.0134 rad (2mm 반영 확인)

접촉 경계 비교 (World X, m):

| 물체 | Pre-grasp 3mm (84°) | Pre-grasp 2mm (69.3°) | 차이 | Grasp 3mm | Grasp 2mm | 차이 |
|---|---|---|---|---|---|---|
| pin | 0.2006 | 0.2004 | -0.2 mm | 0.2811 | 0.2804 | -0.7 mm |
| block | 0.2031 | 0.2028 | -0.3 mm | 0.2030 | 0.2022 | -0.8 mm |
| hex_nut | 0.1858 | 0.1857 | -0.1 mm | 0.1855 | 0.1858 | +0.3 mm |
| bolt | 0.1968 | 0.1966 | -0.2 mm | 0.2774 | 0.2770 | -0.4 mm |

- 차이 -0.8~+0.3 mm로 모두 1 mm 미만 (측정 간격 약 0.8 mm 수준). 차체가 덜 들린 영향은 사실상 없음
- 닿는 부위도 같음: Pre-grasp(69.3°)는 4종 모두 차체 앞면 · 앞 캐스터 (팔 안 닿음), Grasp는 pin · bolt만 약 0.28 m에서 팔 · 그리퍼, block · hex_nut은 차체 · 캐스터

후보별 여유 (2mm, 후보 - 경계). 8개 조합 모두 세 후보에서 접촉 없음:

| 자세 | 물체 | 0.35 m | 0.32 m | 0.29 m | (참고) 3mm 0.29 m |
|---|---|---|---|---|---|
| Pre-grasp | pin | 15.0 cm | 12.0 cm | 9.0 cm | 8.9 cm |
| | block | 14.7 cm | 11.7 cm | 8.7 cm | 8.7 cm |
| | hex_nut | 16.4 cm | 13.4 cm | 10.4 cm | 10.4 cm |
| | bolt | 15.3 cm | 12.3 cm | 9.3 cm | 9.3 cm |
| Grasp | pin | 7.0 cm | 4.0 cm | 1.0 cm | 0.9 cm |
| | block | 14.8 cm | 11.8 cm | 8.8 cm | 8.7 cm |
| | hex_nut | 16.4 cm | 13.4 cm | 10.4 cm | 10.5 cm |
| | bolt | 7.3 cm | 4.3 cm | 1.3 cm | 1.3 cm |

- 결론: standoff_m 최종값 0.35 m는 caster_gap 2mm에서도 안전. 최소 여유는 Grasp pin 7.0 cm, Pre-grasp는 14.7 cm(block) 이상. 지난 시행의 실제 도착 오차(xy 4~5 cm)를 감안해도 남음
- Grasp 자세 pin · bolt의 0.29 m 여유는 1.0 · 1.3 cm로 3mm 때와 사실상 같아 여전히 빠듯함
- 확인 못 한 것은 위와 같음 (조합마다 1회, 정면 접근만, 부품 static, 팔을 내리는 동안의 접촉은 범위 밖)
- 파일: contact2_results.txt, c2_{pregrasp,grasp}_{pin,block,hex_nut,bolt}.json · .log, worlds_contact2/, mm_robot_5741526.urdf

## 다음 단계를 막는 것, 물어볼 사람

1. 깊이가 VM에서 안 나옴 (0-3, 정렬 · 파지 연동 시험을 막음)
   - 갱신 (11/21): feat/grasp b9e8a55 + `render_engine:=ogre2`(software_gl, headless)로 해결됨 ("0-3 재시험"). 남은 것: launch 기본값과 yolo_test_room.sdf가 아직 ogre라 인자를 빠뜨리면 다시 회색 · inf가 됨. 기본값을 ogre2로 바꿀지는 희선님(B)과 정할 일. 아래 원래 항목은 11/20 기준
   - 희선님(B): 희선님 환경(WSL)에서 `rgbd_camera` 깊이가 실제로 나오는지?? 회색이었다면 어떤 렌더러였는지?? Sensors 플러그인이 ogre로 고정된 이유가 있는지??
   - 다은님(D): 정렬 시험 월드(ogre2)에서 깊이가 나오는 환경(OS, GPU, software_gl 여부)이 어떤지?? 같은 rgbd 설정을 통합 모델에 써도 되는지??
   - 결과에 따라: 깊이가 필요한 시험은 팀원 PC(WSL)에서 돌리고 VM은 깊이 없는 시험만 하는 식으로 나눌지 정해야 함 (A)
2. Nav2가 목표 근처에서 성공 판정을 못 받음 (0-4, A 자체 작업)
   - 갱신 (11/20): 원인 확인됨 ("0-4 원인 확인"). object 모드 목표는 문제없이 도착하므로 다음 단계를 막지 않음. 목표 yaw를 다가오는 방향에 가깝게 잡는 규칙(interfaces.md 6장)을 지키면 됨. progress checker 값은 그대로 둠. 아래 원래 항목은 11/19 기준
   - 같은 목표를 팔 없는 차체(feat/navigation)로 돌려 통합 모델 영향인지 먼저 구분. 목표 yaw를 다가오는 방향에 맞춘 목표로 한 번 더 시험
   - progress_checker 값은 PR #4 범위라 바꾸기 전에 PR #4에서 논의
3. 로봇 pitch 0.020 rad 기울기 (TF에 없음)
   - 희선님(B): 팔 무게나 캐스터 때문에 생긴 의도하지 않은 기울기인지?? (Far 기준 투영 약 13 px, 깊이 약 7 mm 차이를 만듦. 원준님 target_pose 계산에도 영향)
4. 부품이 라이다 · costmap에 안 잡힘
   - A가 정할 것: object 모드 목표(물체에서 0.40 m)는 괜찮지만, 경로가 다른 부품 위를 지나갈 수 있음. 부품 영역을 keepout으로 둘지 등은 나중에 논의
   - 막는 문제는 아님
5. 참고: 라이다가 팔을 통과해서 보는 동작은 시뮬에서만 그런 것일 수 있음 (gpu_lidar 렌더링 방식 추정, 확인 필요). 지금 단계를 막지는 않음

## scratchpad 파일

- 로그: sim_a.log, sim_b.log, sim_c.log, sim_d.log, nav_e.log
- 데이터: rtf_a.txt, scan_default.json, scan_near.json, scan_near_goal.json, depth_far_odom.json, depth_far_gz.json, depth_near_goal.json, parts_before.txt, parts_after.txt
- 도구: tools/ (scan_stats.py, depth_check.py, img_stats.py, look_dir.py, costmap_self.py, send_goal.py, tf_once.py, joints.py, world_to_base.py, combo_check.sh)
- 남아 있는 것 (11/19 기준): worktree wt-grasp(30cf477, detached), ws_int(빌드), yolo-models, pr4, yolo_test_room_ogre2.sdf
- 갱신 (11/21)
  - worktree: wt-grasp는 지금 5741526 (detached, 11/21 caster_gap 2mm 재검증 때 옮김), wt-yolo e2ce2af (detached). 둘 다 커밋 안 된 변경 없음
  - 빌드: ws_int(통합 모델 + mm_perception + PR #4 mm_navigation), ws_base(팔 없는 차체, PR #4 mm_description + mm_navigation)
  - 0-4 원인 확인: nav_trials.jsonl, nav_trials_obj.jsonl, trial_base_*.log, trial_arm_*.log
  - 근거리 YOLO · 전진 시험: nav_trial_yolo.jsonl, nav_trial_approach.jsonl, nav_*_approach2.jsonl, nav_conf01_*.jsonl, nav_rep_near_*.jsonl, trial_*.approach.json, trial_*.det.log
  - 0-3 재시험: d3_results.txt, d3_A/B/C.cam.json, d3_*.log, d3_*.rtf.txt, d3_B.det.json, d3_B_scan.json, d3n_near.json, d3p_pregrasp.json
  - 접촉 경계: worlds_contact/contact_{block,pin,hex_nut,bolt}.sdf, contact_results.txt, contact_{pregrasp,grasp}_{pin,block,hex_nut,bolt}.json · .log, mm_robot_b9e8a55.urdf, tools/contact_probe.py · contact_run.sh · contact_all.sh
  - 추가 도구: tools/ (nav_trial.sh, approach_steps*.py, approach_depth.py, cam_check.py, proj_check.py, det_collect.py, det_run.sh, hook_*.sh, depth_combo.sh, depth_all.sh, run_det_f32.py, ov_f32_patch.py)
