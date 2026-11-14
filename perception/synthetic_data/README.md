# 부품 모델 4종 (C · 인식)

YOLO 합성 데이터용 Gazebo Fortress 부품 모델과 확인용 월드입니다.

| 클래스 | YOLO class id | Gazebo label | 크기 | 비고 |
|---|---|---|---|---|
| `block` | 0 | 1 | 2.5 × 2.5 × 4 cm | 기본 도형 (box) |
| `pin` | 1 | 2 | 지름 2 cm, 길이 5 cm | 기본 도형 (cylinder) |
| `hex_nut` | 2 | 3 | 맞변 3 cm, 두께 1.5 cm, 구멍 1.6 cm | STL 메쉬 |
| `bolt` | 3 | 4 | 머리 맞변 2.5 cm, 전체 길이 6 cm | STL 메쉬 (머리가 아래로 선 자세) |

- 모든 모델의 원점은 **바닥 중심**이라 `z=0`에 두면 바닥 위에 놓입니다.
- Gazebo label은 0이 배경이라 1부터 씁니다. **YOLO class id = label − 1** 입니다.
- 충돌 모양은 계산이 가볍도록 단순 도형(원기둥)으로 근사했습니다.

## 폴더 구조

```
parts_models/
├── models/        block, pin, hex_nut, bolt (model.sdf, model.config, meshes/)
├── worlds/        parts_test.sdf  (부품 4개 + boundingbox 카메라)
└── tools/         make_meshes.py  (너트·볼트 STL 생성기, 치수 수정용)
```

## 실행 (WSL 터미널)

```bash
cd ~/parts_models
export IGN_GAZEBO_RESOURCE_PATH=$PWD/models:$IGN_GAZEBO_RESOURCE_PATH
LIBGL_ALWAYS_SOFTWARE=1 ign gazebo -r worlds/parts_test.sdf --render-engine ogre
```

## 확인 (새 터미널)

```bash
ign topic -l | grep boxes                # /boxes, /boxes_image 가 보여야 함
ign topic -e -t /boxes -n 1              # 박스 4개, label 1~4
```

GUI 오른쪽 위 `⋮` 메뉴 → **Image Display** 추가 → 토픽 `/boxes_image` 선택하면 카메라 화면을 볼 수 있습니다.

> ⚠️ boundingbox 카메라는 **ogre2 전용**입니다. WSL에서는 화면 없이(`-s`) 서버만 실행하세요.
> `parts_test_ogre2.sdf` = 박스 확인용, `parts_test_rgb.sdf` = 일반 카메라(ogre) 버전.

## 합성 데이터 수집

```bash
cd ~/parts_models
bash tools/run_collect.sh 200 ~/yolo_dataset      # 200장 수집 (Gazebo·브리지 자동 실행/종료)
python3 tools/draw_labels.py --data ~/yolo_dataset # 검수용 모음 이미지 → ~/yolo_dataset/check_labels.jpg
```

- 매 장면마다 부품 종류·개수(1~6개)·위치·자세(서 있기/눕기), 방해물(0~2개),
  카메라 각도(아래로 20~70°)·거리(0.25~1.2 m), 조명 방향·밝기가 무작위로 바뀝니다.
- 같은 폴더로 다시 실행하면 **이어서** 모읍니다 (번호가 이어짐).
- 결과 폴더: `images/{train,val}/*.jpg`, `labels/{train,val}/*.txt` (YOLO 형식), `data.yaml`, `meta.csv`
- 월드: `worlds/collect.sdf` (부품 4종 × 2개 + 라벨 없는 방해물 2개)

## 치수 바꾸기

`tools/make_meshes.py` 맨 위 상수를 고치고 `python3 tools/make_meshes.py`를 실행하면 STL이 다시 만들어집니다.
(그리퍼 벌림 폭이 확정되면 B와 맞춰서 조정)
