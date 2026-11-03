# Visual Servoing 설계 (D)

- 담당: D (윤다은), 브랜치 `feat/visual-servoing`
- 상태: 1주차 초안
- 기준: 회의록 역할 정의, `docs/interfaces.md` v0.7 (PR #1)
- 공통 인터페이스는 `interfaces.md`가 기준이다.

## 1. 범위

Nav2 정차 후 남은 위치 오차를 카메라 피드백으로 보정해, 물체를 팔이 잡을 수 있는 위치에 둔다.

```
Nav2 정차 → 관측 자세 고정 → Align (D) → IK 확인 (B) → Grasp (B)
                                  ↑              │
                                  └─ IK 실패 시 재정렬 ┘
```

- 제어 대상: **모바일 베이스** (`vx`, `wz`). 로봇팔은 정렬 중 움직이지 않는다.
- 팔 접근과 파지는 B 담당이다.
- MoveIt Servo는 팔 제어용이므로 사용하지 않는다.

## 2. 제어 방식

직접 제작하는 차체는 차동구동이라 옆으로 이동할 수 없다. 따라서 x/y 오차 대신 **거리 오차와 방위 오차**를 제어한다. 제어 입력(`vx`, `wz`) 2개로 오차 2개를 줄일 수 있다.

오차 기준 좌표계는 `base_link`다. `standoff`는 `base_link` 원점에서 물체까지의 xy 평면 거리다. `mm_description` 차체에서 `base_link` 원점은 두 바퀴 축의 중앙이라 차체 회전 중심과 같다.

### 2.1 오차 계산

1. 검출 결과에서 물체 픽셀 좌표 `(u, v)`를 얻는다.
2. 깊이 이미지의 bbox 중심 주변 값 중앙값을 `Z`로 쓴다. NaN/inf는 제외한다.
3. 카메라 좌표로 변환한다. `fx, fy, cx, cy`는 `/camera/camera_info` 값이다.

   ```
   X = (u - cx) * Z / fx
   Y = (v - cy) * Z / fy
   ```

4. 측정 시각(`header.stamp`)의 TF로 `base_link` 좌표 `(x_b, y_b)`로 변환한다.
5. 오차를 계산한다.

   ```
   e_r = sqrt(x_b^2 + y_b^2) - standoff
   e_b = atan2(y_b, x_b)
   ```

### 2.2 제어 법칙

```
wz = clamp(k_w * e_b, ±0.30 rad/s)
vx = clamp(k_v * e_r, ±0.05 m/s)
|e_b| > 0.20 rad 이면 vx = 0
```

초기 이득: `k_v = 1.0`, `k_w = 1.0`.

정지는 허용 범위의 절반(`stop_ratio = 0.5`) 안에 들어왔을 때 한다. P제어는 경계에 닿자마자 멈추므로, 허용 범위 그대로 쓰면 최종 오차가 경계(예: 0.020 m)에 걸려 노이즈에 따라 판정이 흔들린다.

### 2.3 완료 판정

오래된 프레임으로 완료를 판정하면 안 된다. 움직이는 동안 들어온 값이 허용 범위 안이어도 실제 위치는 이미 다를 수 있다.

1. 허용 범위 안이면 정지한다.
2. **정지 시각 이후 촬영된 프레임**만 사용한다 (`header.stamp` 비교).
3. 3프레임 연속 허용 범위 안이면 성공이다.
4. 벗어나면 제어를 재개한다.

### 2.4 지연이 클 때: 반복 보정

측정 지연이 약 1초를 넘으면 연속 제어 대신 아래를 반복한다. 지연은 메시지 `header.stamp`와 현재 sim time의 차이로 측정한다.

```
측정 → 0.5초 이동 → 정지 → 새 프레임 대기 → 측정
```

### 2.5 실패 처리

| 상황 | 동작 | 결과 코드 |
|---|---|---|
| 제한 시간 초과 | 정지 | `ALIGN_TIMEOUT` |
| 시작 시 물체 없음 | 정지 | `OBJECT_NOT_FOUND` |
| 정렬 중 물체 놓침 | 정지 후 대기, 복구 안 되면 실패 | `OBJECT_NOT_FOUND` |

물체 없음 판정은 `interfaces.md` 5.3을 따른다: score 0.5 이상인 목표 클래스 검출이 연속 N프레임(예: 10프레임) 동안 없으면 실패다.

성공 시 `error_code`는 빈 문자열이다. 속도 명령 노드가 끝나거나 취소되면 반드시 0 속도를 발행한다.

IK 실패 후 재정렬은 상태머신이 `standoff_m`만 바꿔 Align을 다시 호출한다 (최대 2회, `interfaces.md` 5.2). D는 Goal의 값을 그대로 쓴다.

## 3. 인터페이스 제안

### 3.1 토픽

| 토픽 | 타입 | 방향 | 용도 |
|---|---|---|---|
| `/perception/detections` | `vision_msgs/msg/Detection2DArray` | 구독 | `results[0].hypothesis.class_id`, `results[0].hypothesis.score`, `bbox.center.position` (`interfaces.md` 3.2) |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 구독 | 거리 `Z` |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 구독 | 내부 파라미터 |
| `/servoing/cmd_vel` | `geometry_msgs/msg/Twist` | 발행 | 베이스 미세 정렬 속도 |

`/perception/target_pose`는 C의 3D 추정이 나오면 입력으로 바꿔 비교한다 (6장).

### 3.2 제어권

D는 `/servoing/cmd_vel`만 발행하고 `/cmd_vel`에 직접 발행하지 않는다. 상태머신(A)이 현재 단계에 맞는 명령만 `/cmd_vel`로 전달한다 (`interfaces.md` 5.0). 구현 방식은 A가 정한다.

### 3.3 Align 액션

```text
# Goal
string target_class
float64 standoff_m
float64 timeout_sec
---
# Result
bool success
string error_code
float64 final_range_error_m
float64 final_bearing_error_rad
---
# Feedback
bool target_visible
float64 range_error_m
float64 bearing_error_rad
```

허용 오차는 goal에 넣지 않고 5.1장 값을 파라미터로 관리한다.

### 3.4 전환 조건 후보 (`interfaces.md` 5.1)

- 방위 오차 0.05 rad 이하
- 거리 오차 0.02 m 이하
- 정지 후 촬영한 3프레임 연속 만족

이미지 중심 오차(px) 기준은 쓰지 않는다. 카메라가 베이스 중심축에 있지 않으면 물체가 화면 중앙에 있어도 베이스 기준 오차가 남기 때문이다. 수치는 후보이며 B의 작업 공간과 그리퍼 크기로 확정한다.

## 4. 다른 담당에게 필요한 것

| 담당 | 필요 항목 | 필요 시점 |
|---|---|---|
| A | 카메라 TF, 해상도, FPS, 깊이 형식 | 6주 전, 가능하면 2주 |
| A | Nav2 정차 오차 측정값 (Align 시작 범위 확인) | 5주 |
| A | `/cmd_vel` 제어권 연동 | 7주 |
| B | 관측 자세 관절값 (물체가 화면에 들어오는 자세) | 6주 전 |
| B | 시도별 `standoff_m`: 팔이 잡을 수 있는 베이스-물체 거리. 차체 앞면(현재 임시값 0.20 m)보다 커야 함 | 6주 전 |
| B | 팔 장착 위치. 본 설계는 차체 중심축 장착을 가정 | 6주 전 |
| C | 클래스 이름, 검출 FPS, 지연 | 7주 전 |

## 5. 수치 검증

`docs/visual_servoing_check.py`로 확인했다. 시작 오차 거리 ±0.15 m, 방위 ±0.35 rad, 시나리오별 200회, 카메라 노이즈 2 px / 5 mm, `k_v = k_w = 1`. 결과의 오차는 측정값이 아니라 실제 위치 기준 p95다.

| 지연 / 주기 | 단순 판정 | 정지 후 판정 (2.3) | 반복 보정 (2.4) |
|---|---|---|---|
| 0.2 s / 0.1 s | 100%, 9 mm, 0.018 rad | 100%, 18 mm, 0.040 rad | 100%, 10 mm, 0.025 rad |
| 0.5 s / 0.2 s | 100%, 8 mm, 0.025 rad | 100%, 8 mm, 0.029 rad | 100%, 14 mm, 0.035 rad |
| 1.0 s / 0.5 s | 100%, **40 mm, 0.175 rad** | 100%, 18 mm, 0.045 rad | 100%, 19 mm, 0.046 rad |
| 2.0 s / 1.0 s | 61%, **97 mm, 0.605 rad** | 3% | **100%**, 17 mm, 0.038 rad |
| 4.0 s / 2.0 s | 8% | 2% | 44% |

해석:

- 지연 0.5초까지는 연속 P제어로 충분하다.
- 지연 1초부터 단순 판정은 성공으로 보고해도 실제 오차가 허용 범위를 넘는다.
- 지연 2초에서는 반복 보정만 안정적이다.
- 지연 0.2초 기준 이득은 `k_v` 0.5~2, `k_w` 0.5~4 전부 수렴했다.

한계: 등속 unicycle 모델 계산이다. 바퀴 동역학, 미끄럼, 마찰, 오검출은 없다. Gazebo와 실제 YOLO로 다시 측정해야 한다. 초점거리와 standoff는 임시값이다.

## 6. 평가 계획

비교 대상:

1. 보정 없음 (Nav2 정차만)
2. 연속 P제어 + 정지 후 판정
3. 반복 보정
4. 입력 비교: 픽셀+깊이 직접 계산 vs `/perception/target_pose`

측정 항목: 최종 거리 오차, 최종 방위 오차, 정렬 시간, 성공률, 이후 파지 성공률. 조건별 같은 시작 위치를 반복한다.

## 7. 주차별 산출물

| 주차 | 산출물 |
|---|---|
| 2주 | 2.1 오차 계산 함수, 모의 좌표 단위 테스트 (완료, `src/visual_servoing`) |
| 3주 | 2.2 P제어 노드, 모의 입력 베이스 정렬 (2주차에 앞당겨 완료) |
| 4주 | 모의 폐루프 검증, 2.3 완료 판정 |
| 5주 | `/servoing/align` 액션 서버 |
| 6~7주 | Gazebo 카메라, 실제 YOLO 입력 연동 |
| 8주 | 2.4 반복 보정 안정화 |
| 9주 이후 | 6장 비교 실험, 통계 분석, 발표 자료 |
