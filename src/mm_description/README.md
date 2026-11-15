# mm_description

차동구동 차체, LiDAR, DOFBOT 로봇팔 및 카메라를 통합한 모바일 매니퓰레이터 시뮬레이션 패키지.

Gazebo에서 로봇팔 관절 제어 및 카메라 관측을 수행하며, 객체 인식·정렬·파지 기능 연동을 위한 시뮬레이션 환경을 제공한다.

## 개발 환경

| 항목 | 환경 |
| --- | --- |
| OS | Ubuntu 22.04 (WSL2) |
| ROS | ROS2 Humble |
| Simulator | Gazebo Fortress (Ignition Gazebo 6) |
| Visualization | RViz2 |
| Branch | `feat/grasp` |

## 빌드

레포지토리 폴더에서 실행한다.

```bash
git fetch origin
git switch feat/grasp
git pull --ff-only origin feat/grasp

source /opt/ros/humble/setup.bash

colcon build --packages-select dofbot_description mm_description

source install/setup.bash
```

`mm_description`은 `dofbot_description`의 로봇팔 링크, 관절 및 STL 메시를 사용하므로 두 패키지를 함께 빌드한다.

## 실행

### 1. 통합 시뮬레이션

WSL2 환경에서는 Gazebo를 headless로 실행하고 RViz를 별도로 실행하는 방식을 사용한다.

```bash
ros2 launch mm_description sim.launch.py \
  software_gl:=true \
  render_engine:=ogre \
  headless:=true \
  rviz:=false
```

실행 시 다음 기능이 활성화된다.

- 차체 및 DOFBOT 로봇팔 통합 모델
- 차동구동 차체 및 LiDAR
- 로봇팔 joint1~joint5 위치 제어기
- RGB 카메라 및 CameraInfo
- Robot State Publisher
- ROS-Gazebo Bridge

기존 Gazebo 서버가 남아 있으면 중복 실행될 수 있으므로, 이전 시뮬레이션을 종료한 후 실행한다.

### 2. RViz 실행

새 터미널에서 실행한다.

```bash
cd ~/smart-factory-mobile-manipulator

source /opt/ros/humble/setup.bash
source install/setup.bash

LIBGL_ALWAYS_SOFTWARE=1 rviz2 \
  --ros-args -p use_sim_time:=true
```

RViz 설정:

| 항목 | 설정 |
| --- | --- |
| Fixed Frame | `base_footprint` |
| RobotModel | `/robot_description` |
| Image | `/camera/image_raw` |

Displays에서 RobotModel과 Image를 추가하면 로봇팔 자세와 카메라 영상을 확인할 수 있다.

### 3. Far 관측 자세 실행

Gazebo 실행 및 로봇 생성 완료 후 새 터미널에서 실행한다.

```bash
cd ~/smart-factory-mobile-manipulator

source /opt/ros/humble/setup.bash
source install/setup.bash

bash src/mm_description/scripts/set_observe_pose_far.sh
```

기존 `observe_pose_01`의 목표 관절각을 Gazebo에 전달한다.

실제 관절 상태는 다음 명령어로 확인한다.

```bash
ros2 topic echo /joint_states --once
```

목표 관절각에 정상적으로 도달하고 정지하는 것을 확인했다.

## Launch 옵션

| 옵션 | 기본값 | 설명 |
| --- | --- | --- |
| `software_gl` | `false` | CPU 렌더링 사용 여부. WSL2에서는 `true` 사용 |
| `headless` | `false` | `true`이면 Gazebo GUI 없이 서버만 실행 |
| `render_engine` | `ogre` | 렌더링 엔진 선택 (`ogre`, `ogre2`) |
| `rviz` | `true` | RViz 동시 실행 여부 |
| `rviz_gpu` | `false` | `true`이면 RViz만 GPU 렌더링 사용 |
| `rviz_config` | `rviz/base.rviz` | RViz 설정 파일 경로 |

## 로봇팔 제어

현재 Gazebo의 JointPositionController를 이용해 joint1~joint5의 목표 관절 위치를 제어한다.

### 관절 제어 토픽

```text
/mm_robot/joint1/cmd_pos
/mm_robot/joint2/cmd_pos
/mm_robot/joint3/cmd_pos
/mm_robot/joint4/cmd_pos
/mm_robot/joint5/cmd_pos
```

개별 관절 제어 예시:

```bash
ign topic -t /mm_robot/joint3/cmd_pos \
  -m ignition.msgs.Double \
  -p "data: -1.1464"
```

관절각의 단위는 rad이다.

### 제어기 설정

| 항목 | 설정값 |
| --- | --- |
| P gain | 5.0 |
| I gain | 0.0 |
| D gain | 0.1 |
| initial_position | 0.0 |
| use_velocity_commands | true |
| cmd_max | 0.4 |

현재 Gazebo 모델은 관측 시험을 위해 `use_vj:=false`, `fix_gripper:=true` 옵션으로 실행한다.

따라서 joint1~joint5의 제어는 가능하지만 그리퍼는 고정되어 있다. 그리퍼 개폐 및 MoveIt2를 통한 파지 경로 계획은 추가 구현 예정이다.

## 관측 자세

### Far (`observe_pose_01`)

이동 중 물체 탐색 및 시맨틱 맵 생성을 위한 관측 자세.

| 관절 | 목표 관절각 (rad) |
| --- | ---: |
| joint1 | 0.0000 |
| joint2 | 0.0000 |
| joint3 | -1.1464 |
| joint4 | -0.8237 |
| joint5 | 0.0000 |

카메라 장착 링크의 기존 측정 좌표:

```text
base_link → DaBai_DCW2_Link

XYZ (m):   (0.259, 0.001, 0.269)
RPY (rad): (-0.391, 0.000, -1.571)
```

카메라 optical frame의 기존 측정 좌표:

```text
base_link → camera_optical_frame

XYZ (m):   (0.259, 0.001, 0.269)
RPY (rad): (-1.961, 0.000, -1.571)
```

위 값은 기존 Far 관측 시험에서 측정한 `base_link` 기준 상대 좌표이다.

실제 카메라 광축은 전방에서 약 22° 아래를 향한다. 로봇팔 관절각에 따라 카메라 외부 파라미터가 변경되므로, 실제 좌표 변환 시에는 해당 시점의 TF를 사용한다.

카메라 TF 확인:

```bash
ros2 run tf2_ros tf2_echo \
  base_link camera_optical_frame
```

기존 관측 자세의 측정 결과는 레포지토리의 `observations/`에 저장되어 있다.

### Near (개발 예정)

정렬 및 파지 단계에서 사용할 근거리 관측 자세.

| 항목 | 목표 조건 |
| --- | --- |
| Nav2 도착 목표 거리 | 약 0.40 m |
| 정렬 시작 거리 범위 | 약 0.27~0.53 m |
| 정렬 완료 거리 | 약 0.22~0.28 m |
| 목표 관측 범위 | 약 0.20~0.55 m |

Near 자세에서는 정렬 과정 동안 로봇팔 자세를 고정하고 물체를 지속적으로 관측하는 것을 목표로 한다.

관절각 및 카메라 TF는 관측 범위 검증 후 추가할 예정이다.

## 카메라 인터페이스

`docs/interfaces.md`에 맞춰 RGB 이미지 및 CameraInfo를 발행한다.

카메라는 로봇팔 `link4`에 연결된 `DaBai_DCW2_Link`에 장착되어 있다.

| 항목 | 설정 |
| --- | --- |
| RGB 이미지 | `/camera/image_raw` |
| CameraInfo | `/camera/camera_info` |
| Image frame_id | `camera_optical_frame` |
| CameraInfo frame_id | `camera_optical_frame` |
| 해상도 | 640 × 480 |
| 업데이트 설정 | 10 Hz |
| 수평 FOV | 1.047 rad (약 60°) |
| 수직 FOV | 약 46.8° |
| Near clip | 0.05 m |
| Far clip | 10.0 m |

카메라 브리지는 `sim.launch.py`에 포함되어 있으므로 별도로 실행할 필요가 없다.

### 카메라 토픽 확인

```bash
ros2 topic list | grep camera
```

Image의 frame_id 확인:

```bash
ros2 topic echo /camera/image_raw \
  --once --field header \
  --qos-reliability best_effort
```

CameraInfo 확인:

```bash
ros2 topic echo /camera/camera_info \
  --once --qos-reliability best_effort
```

Image와 CameraInfo 메시지가 정상적으로 발행되고, 두 메시지의 frame_id가 `camera_optical_frame`으로 일치하는 것을 확인했다.

### 카메라 내부 파라미터

`/camera/camera_info`에서 확인한 값이다.

| 파라미터 | 값 |
| --- | ---: |
| fx | 554.3827 px |
| fy | 554.3827 px |
| cx | 320.0 px |
| cy | 240.0 px |

Intrinsic Matrix (K):

```text
[554.3827,   0.0000, 320.0]
[  0.0000, 554.3827, 240.0]
[  0.0000,   0.0000,   1.0]
```

- Distortion model: `plumb_bob`
- Distortion coefficients: [0, 0, 0, 0, 0]

위 파라미터는 Gazebo가 카메라 해상도와 FOV를 바탕으로 생성한 값이며, 별도의 캘리브레이션을 수행하지 않았다.

Far/Near 관측 자세에 따라 카메라의 외부 파라미터는 변경되지만, 해상도 및 FOV 설정이 동일하면 내부 파라미터는 유지된다.

## 차체 및 로봇팔 배치

### 차체

| 항목 | 설정값 |
| --- | --- |
| 차체 크기 | 0.38 × 0.28 × 0.06 m |
| 바퀴 반경 | 0.05 m |
| 바퀴 간 거리 | 0.35 m |
| Ground clearance | 0.02 m |
| LiDAR 위치 (base_link) | (0.18, 0, 0.075) m |
| LiDAR 스캔 높이 (base_footprint) | 0.125 m |

LiDAR 높이는 기존 Xacro 계산식을 유지하며, LiDAR 원통 아랫면과 차체 윗면이 일치한다.

LiDAR 전방 끝은 X=0.215 m로 차체 전면보다 25 mm 돌출된다.

### 로봇팔 장착 위치

```text
Joint: arm_mount_joint (fixed)

Parent: base_link
Child:  arm_base_link

XYZ (m):   (0.100, 0, 0.010)
RPY (rad): (0, 0, -1.5708)
```

팔 밑판 STL의 장착 후 범위 (`base_link` 기준):

| 축 | 범위 (m) |
| --- | --- |
| X | -0.1217 ~ +0.1788 |
| Y | -0.0927 ~ +0.0928 |
| Z | -0.0050 ~ +0.1079 |

팔 밑판은 차체의 X/Y 외곽을 벗어나지 않는다.

바닥 물체까지의 로봇팔 도달 범위를 확보하기 위해 팔 장착 높이를 낮췄으며, 이에 따라 팔 밑판 메시의 최저점은 차체 윗면보다 35 mm 낮다.

현재 프로젝트는 시뮬레이션 환경에서만 진행하므로 해당 배치를 유지한다. 향후 통합 충돌 검사에서는 차체와 고정된 팔 밑판 사이의 충돌 제외를 검토한다.

## 테스트 월드

기본 월드는 `worlds/test_room.sdf`를 사용한다.

현재 카메라 관측 시험용 정육면체가 배치되어 있다.

| 항목 | 설정 |
| --- | --- |
| 모델명 | `grasp_cube_3cm` |
| 크기 | 0.03 × 0.03 × 0.03 m |
| 중심 위치 (World) | (0.8, 0, 0.015) m |
| Static | true |

기존 RGB 카메라를 이용해 해당 정육면체를 관측하는 데 성공했다.

### YOLO 부품 모델 활용

YOLO 학습용 부품 모델은 `feat/yolo` 브랜치의 다음 경로에 있다.

```text
perception/synthetic_data/models/
```

모델을 사용할 때는 Gazebo 리소스 경로에 해당 폴더를 추가한다.

```bash
export IGN_GAZEBO_RESOURCE_PATH="/path/to/models:${IGN_GAZEBO_RESOURCE_PATH}"
```

`test_room.sdf`에 모델을 추가하는 예시:

```xml
<include>
  <uri>model://block</uri>
  <name>test_block</name>
  <pose>0.8 0 0 0 0 0</pose>
</include>
```

부품 모델의 원점은 바닥 중심이므로 Z=0으로 배치한다.

현재 기본 월드에는 YOLO 학습용 부품 4종을 아직 추가하지 않았다.

### 카메라 영상 녹화

객체 인식 시험용 RGB 이미지 및 CameraInfo를 녹화할 수 있다.

```bash
ros2 bag record \
  /camera/image_raw \
  /camera/camera_info \
  -o arm_camera_test
```

녹화를 종료하려면 Ctrl+C를 누른다.

## 주의사항

- Gazebo 서버가 중복 실행되면 관절 제어 및 카메라 출력에 문제가 발생할 수 있다. 기존 시뮬레이션을 종료한 후 재실행한다.
- 일반 RGB 카메라 영상 및 CameraInfo는 정상적으로 전달된다. 기존 RGB-D 센서의 RGB 영상은 회색으로 출력되며, 원인은 아직 확인되지 않았다.
- Depth 정보는 아직 검증하지 않았다. 필요시 일반 RGB 센서와 Depth 센서를 동일한 위치 및 방향에 별도로 구성하는 방식을 검토한다.
- Image 및 CameraInfo의 frame_id는 `camera_optical_frame`으로 설정했으나, 실제 영상과 TF의 투영 정합은 추가 검증이 필요하다.
- 현재 그리퍼는 관측 시험을 위해 고정되어 있다.
- 현재 테스트용 정육면체는 static 모델이므로 실제 파지 물리 시험에는 동적 물체 모델이 필요하다.
- LiDAR 전방 끝은 X=0.215 m이며, 바퀴 collision 외곽은 Y=±0.225 m이다. Nav2 footprint 설정 시 해당 범위를 고려해야 한다.
- `mm_robot.urdf.xacro`는 `dofbot_description`에 의존한다. Navigation 브랜치와 통합 시 패키지 의존성 및 launch 파일 변경사항을 함께 검토해야 한다.

## 파일

| 파일 | 내용 |
| --- | --- |
| `urdf/mm_base.urdf.xacro` | 차동구동 차체 및 LiDAR |
| `urdf/mm_robot.urdf.xacro` | 차체·DOFBOT 통합 모델 및 관절 제어기 |
| `urdf/mm_camera.gazebo.xacro` | RGB 및 RGB-D 카메라 설정 |
| `launch/sim.launch.py` | Gazebo 및 ROS-Gazebo Bridge 실행 |
| `scripts/set_observe_pose_far.sh` | Far 관측 자세 명령 |
| `worlds/test_room.sdf` | 테스트 월드 및 관측 물체 |
| `rviz/base.rviz` | RViz 설정 |
| `package.xml` | 패키지 의존성 |
| `../../observations/` | Far 자세의 관절각 및 카메라 TF 측정 결과 |
| `../dofbot_description/` | DOFBOT 로봇팔 링크, 관절 및 STL 메시 |

## Observation & Grasp Poses

로봇팔에 장착된 카메라를 이용하여 물체와의 거리에 따라 관측 자세를 변경하도록 구성하였다.

### 기준 관절값 (rad)

| Joint | Far | Near | Pre-grasp | Grasp |
|---|---:|---:|---:|---:|
| joint1 | 0 | 0 | 0 | 0 |
| joint2 | 0 | -0.008482 | -0.636802 | -1.282087 |
| joint3 | -1.146400 | -1.282087 | -0.959445 | -0.602873 |
| joint4 | -0.823700 | -1.299052 | -1.435083 | -1.231193 |
| joint5 | 0 | -0.000157 | -0.000157 | -0.000157 |

| Pose | Distance | Purpose |
|---|---|---|
| Far | 0.55~2.00m | 원거리 물체 탐색 |
| Near | 0.30~0.60m | 접근 및 초기 정렬 |
| Pre-grasp | 0.25~0.35m | 최종 물체 관측 및 파지 준비 |
| Grasp | 0.265m | 물체 위치까지 End Effector 이동 |

거리 기준: 로봇 중심에서 물체 중심까지의 수평 거리.

Far 및 Near의 전체 관측 범위는 운용 후보 범위이며, 추가 검증이 필요하다.

### 자세 재현

프로젝트 루트 디렉터리에서 실행한다.

```bash
# Far (기존 observe_pose_01)
bash src/mm_description/scripts/set_observe_pose_far.sh

# Near
bash src/mm_description/scripts/set_observe_pose_near.sh

# Pre-grasp
bash src/mm_description/scripts/set_pre_grasp_pose_01.sh

# Grasp
bash src/mm_description/scripts/set_grasp_pose_01.sh
```

실제 관절각 확인:

```bash
ros2 topic echo /joint_states --once
```

관절각 및 TF 측정 결과는 프로젝트 루트의 `observations/`에 저장하였다.

## YOLO Test World

`feat/yolo` 브랜치의 부품 모델 4종을 로봇팔 카메라에서 관측할 수 있도록 테스트 월드를 추가하였다.

- block
- pin
- hex_nut
- bolt

### 모델 가져오기

```bash
cd ~/smart-factory-mobile-manipulator

git fetch origin

git worktree add --detach ~/mm-yolo-models origin/feat/yolo
```

이미 해당 worktree가 존재한다면 생성 과정은 생략한다.

### 빌드 및 실행

```bash
source /opt/ros/humble/setup.bash

colcon build --packages-select dofbot_description mm_description

source install/setup.bash

export IGN_GAZEBO_RESOURCE_PATH="$HOME/mm-yolo-models/perception/synthetic_data/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"

ros2 launch mm_description sim.launch.py \
  world:=yolo_test_room.sdf \
  software_gl:=true \
  render_engine:=ogre \
  headless:=true \
  rviz:=false
```

### Gazebo GUI 실행

별도의 터미널에서 실행한다.

```bash
export IGN_GAZEBO_RESOURCE_PATH="$HOME/mm-yolo-models/perception/synthetic_data/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"

LIBGL_ALWAYS_SOFTWARE=1 ign gazebo -g
```

### Camera Interface

- RGB Image: `/camera/image_raw`
- CameraInfo: `/camera/camera_info`
- Frame ID: `camera_optical_frame`
- Resolution: 640 × 480
- RGB Update Rate: 10 Hz (시뮬레이션 설정값)

Far 자세에서 부품 4종이 카메라 영상에 들어오는 것을 확인하였다.

### Known Issues & Next Steps

- Pre-grasp와 Grasp 사이의 이동 과정에서 간헐적으로 충돌 및 자세 재현 문제가 발생함.
- 현재 관절 위치 명령 방식은 충돌 회피 경로를 고려하지 않음.
- 추후 MoveIt2를 활용한 충돌 회피 경로 계획 및 목표 자세 도달 확인 로직을 구현할 예정임.
- Grasp 자세는 물체 위치에 대한 도달 후보 자세이며, 실제 그리퍼 개폐 및 파지는 추가 구현이 필요함.
- YOLO 검출 성능, Depth 카메라 및 카메라 TF 투영 정확도는 추가 검증 예정임.
- 현재 YOLO 테스트 월드의 부품들은 관측 실험을 위해 static 모델로 설정함.