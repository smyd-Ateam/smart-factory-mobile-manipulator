# 통합 1단계 시험 도구 (Nav2 → Near → Align)

진단·시험 전용 폴더입니다. 프로덕션 코드가 아니고, 상태머신·Align 노드 등 레포의 기존 패키지는 수정하지 않습니다.

상태머신의 2단계 호출(Near → Pre-grasp)이 아직 없어서, 이 스크립트가 순서를 대신 진행합니다:
Gazebo·Nav2 시작 → 카메라 확인 → 팔 Far 자세 → Nav2 이동 → 팔 Near 자세 → YOLO·align_node 시작 → Align 1차(Near) → 팔 Pre-grasp 자세 → Align 2차.
`/servoing/cmd_vel` → `/cmd_vel` 중계도 이 스크립트가 합니다.

## 파일

| 파일 | 내용 |
|---|---|
| `stage1_run.py` | 시험 1회 본체. 단계별 결과를 JSON 하나(`s1_result_<TAG>.json`)로 저장 |
| `repeat_yaw0.sh` | approach_yaw=0 강제 접근 반복 시험 (대상별 N회, 회차마다 Gazebo부터 새로 시작) |
| `yaw0rep_summary.py` | 반복 시험 집계: 도착 yaw, Align 1차 결과, 카메라 회색, costmap, 부품 여유, 실패 원인 분류 |
| `stage1_all.sh` | 시험 1회 실행: 남은 프로세스 정리 → Gazebo·Nav2 시작 → 카메라 확인(실패 시 최대 3번 재시작) → `stage1_run.py` → 정리 |
| `cam_gate.sh` | 카메라 정상 확인: 팔 Far 자세 → 3 s 측정 → RGB 색 50가지 이상, 깊이 유효 90% 이상이면 통과 |
| `env_s1.sh` | ROS Humble, 팀 워크스페이스, 시험 폴더 install을 source하고 부품 모델 경로 추가 |
| `joints.py` | `/joint_states`에서 팔 관절값 읽기 (자세 도달 확인용) |
| `world_to_base.py` | 월드 좌표 → base_link 좌표 변환 (카메라 투영 확인용) |
| `cam_check.py` | RGB·깊이·camera_info 수신 주기, 색 개수, 깊이 유효 비율, 지정 점의 투영·깊이 측정 |
| `costmap_self.py` | costmap에서 footprint 안·바깥 띠·뒤쪽(-x) 칸 통계 |
| `../camera_health.py` | 카메라 회색 프레임 판정 공용 모듈 (`ogre2-gray-yaw` 진단 번들과 같은 기준) |

## 실행 준비 (시험 폴더)

스크립트는 시험 폴더(`STAGE1_DIR`, 기본 `/tmp/sfmm-brass-v7`) 안에서 돕니다. 시험 폴더 구성:

- `env_s1.sh`: 이 폴더의 `env_s1.sh`를 복사 (`TEAM_WS` 기본값은 `$HOME/smart-factory-mobile-manipulator`, mm_navigation·mm_task_manager가 빌드된 팀 워크스페이스)
- `tools/`: 이 폴더의 `.py`·`.sh` 파일과 `../camera_health.py`를 복사
- `models/`: feat/yolo 6724510의 `perception/synthetic_data/models` (황동 pin)
- `src/` + 빌드 결과: `mm_description`·`dofbot_description` (feat/grasp 5741526), `mm_perception` (feat/yolo 6724510, 황동 pin 모델 포함)
- visual_servoing 90df7d2 빌드 (Align depth·camera_info RELIABLE). 시험 때는 `/tmp/sfmm-align-reliable/install`을 source
- `yolo_weights/synth_v7_640_openvino_model` (7차 모델을 OpenVINO로 변환한 것)

시험 폴더 만들기 예 (이 브랜치를 checkout한 레포 최상위에서):

```bash
S=/tmp/sfmm-brass-v7; mkdir -p $S/tools $S/src $S/yolo_weights
cp tools/diagnostics/stage1/*.py tools/diagnostics/stage1/*.sh tools/diagnostics/camera_health.py $S/tools/
mv $S/tools/env_s1.sh $S/
git archive 5741526 src/mm_description src/dofbot_description | tar -x -C $S
git archive 6724510 src/mm_perception | tar -x -C $S
T=$(mktemp -d); git archive 6724510 perception/synthetic_data/models | tar -x -C $T && mv $T/perception/synthetic_data/models $S/models && rm -rf $T
(cd $S && source /opt/ros/humble/setup.bash && PYTHONNOUSERSITE=1 colcon build --symlink-install)
# 7차 모델 OpenVINO 폴더를 $S/yolo_weights/synth_v7_640_openvino_model 에 두기
```

## 실행

시험 1회 (예: pin, Near 0.40 / Pre-grasp 0.35):

```bash
cd $STAGE1_DIR
TAG=my_pin_run GOAL=0.3,-0.12,0.0 NEAR_STANDOFF=0.40 timeout 900 bash tools/stage1_all.sh pin
```

approach_yaw=0 반복 시험 (대상별 N회, Align 1차까지):

```bash
bash tools/repeat_yaw0.sh 10 "block bolt"
python3 tools/yaw0rep_summary.py $STAGE1_DIR
```

환경변수:

| 이름 | 기본값 | 설명 |
|---|---|---|
| `STAGE1_DIR` | `/tmp/sfmm-brass-v7` | 시험 폴더 |
| `TEAM_WS` | `$HOME/smart-factory-mobile-manipulator` | mm_navigation·mm_task_manager가 빌드된 팀 워크스페이스 (`env_s1.sh`) |
| `ALIGN_SETUP` | `/tmp/sfmm-align-reliable/install/setup.bash` | visual_servoing 90df7d2 빌드의 setup (`stage1_all.sh`) |
| `TAG` | 대상 이름 | 결과·로그 파일 이름 |
| `GOAL` | 없음 (approach_goal 계산) | Nav2 목표 `x,y,yaw` (map) |
| `NEAR_STANDOFF` | `0.35` | Align 1차 목표 거리 (m). Pre-grasp(2차)는 0.35 고정 |
| `STOP_AFTER_ALIGN1` | 없음 | `1`이면 Align 1차 뒤에 멈춤 |

## 결과 파일 (`s1_result_<TAG>.json`) 주요 항목

- `steps.nav`: Nav2 상태, 도착 위치(Gazebo 실제 값), 목표 대비 오차
- `steps.costmap_after_nav`: 도착 직후 local costmap에서 footprint 안, 바깥 0.1 m 띠, 뒤쪽(-x, base 기준 x -0.20 ~ -0.60 m) 칸 통계
- `steps.align1_near`, `steps.align2_pregrasp`: Align 결과, 끝 오차, 실제 거리 오차, 검출 집계, 카메라 회색 판정
- 검출 집계는 메시지 header.stamp(sim 시간) 기준으로 구간을 나눕니다 (구간 경계는 `/clock`).
- 카메라 회색 가드: 정렬 직전 3초와 Align 구간에서 `/camera/image_raw`를 표본(BEST_EFFORT)으로 받아 `camera_health.py`로 판정합니다. 회색이면 콘솔에 `[CAMERA_GRAY_WARNING]`을 남기고, 결과에 `camera_gray_detected`, `camera_gray_windows`, 구간별 `camera`/`camera_3s`(프레임 수, 회색 프레임 수, yaw 범위)를 기록합니다. 시험은 중단하지 않습니다.

## approach_yaw=0 반복 시험 결과 (Mac VM, ogre2 + CPU 렌더링)

조건: 목표 x 0.3, y = 물체 y, yaw 0. Near standoff 0.40, 7차 모델, YOLO RELIABLE, `yaw_goal_tolerance` 0.125 rad. 카메라 안전 구간은 |yaw| ≤ 11° (`ogre2-gray-yaw` 진단 결과).

| 대상 | Align 1차 성공 | 도착 yaw | 안전 구간까지 최소 여유 | 카메라 회색 | 충돌 징후 |
|---|---|---|---|---|---|
| block (10회) | 10/10 | -2.26° ~ -5.36° | 5.6° | 0회 | 없음 |
| bolt (10회) | 10/10 | +6.59° ~ +7.44° | 3.6° | 0회 | 없음 |
| pin (5회) | 5/5 | -6.22° ~ -7.30° | 3.7° | 0회 | 없음 |
| hex_nut (5회) | 5/5 | +5.65° ~ +6.82° | 4.2° | 0회 | 없음 |

도착 yaw의 상한은 Nav2 `yaw_goal_tolerance`(0.125 rad = 7.16°)와 위치 추정 오차로 정해집니다. 이 값을 키우면 안전 구간을 넘을 수 있습니다.
