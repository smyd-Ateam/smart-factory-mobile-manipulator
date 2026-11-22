#!/usr/bin/env bash
# approach_yaw=0 강제 접근 반복 시험 (시험 도구, production 아님)
# 목표 = mm_task_manager approach_goal(물체, (0,0,0), 0.50, approach_yaw=0) 와 같은 값 (x 0.3, y = 물체 y, yaw 0)
# 회차마다 stage1_all.sh 한 번 = Gazebo·Nav2 새로 시작 → 카메라 확인 → Nav2 → Near → Align 1차(standoff 0.40) 후 종료
# 사용: repeat_yaw0.sh [반복 수, 기본 10] [대상 목록, 기본 "block bolt"]
# 이미 결과 파일이 있는 회차는 건너뜀 (중간에 끊겨도 이어서 실행 가능)
S=${STAGE1_DIR:-/tmp/sfmm-brass-v7}   # 시험 폴더 (tools/stage1_all.sh 등이 있는 곳)
N=${1:-10}; TARGETS=${2:-block bolt}
OUT=$S/yaw0rep; mkdir -p "$OUT"
declare -A GOALS=([block]=0.3,0.0,0.0 [pin]=0.3,-0.12,0.0 [hex_nut]=0.3,0.12,0.0 [bolt]=0.3,0.24,0.0)
for i in $(seq -w 1 "$N"); do
  for t in $TARGETS; do          # 대상끼리 번갈아 실행 (시간에 따른 VM 상태 차이가 한쪽에 몰리지 않게)
    TAG=yaw0rep_${t}_$i
    if [ -f "$S/s1_result_$TAG.json" ]; then echo "[skip] $TAG"; continue; fi
    echo "[$(date -u +%T)] start $TAG goal ${GOALS[$t]}"
    TAG=$TAG GOAL=${GOALS[$t]} NEAR_STANDOFF=0.40 STOP_AFTER_ALIGN1=1 timeout 900 bash "$S/tools/stage1_all.sh" "$t" > "$OUT/$TAG.log" 2>&1
    rc=$?
    echo "[$(date -u +%T)] end   $TAG rc=$rc | $(grep -E 'align1_near:|stopped:' "$OUT/$TAG.log" | cut -c1-150 | tr '\n' ' ')"
  done
done
echo REPEAT_DONE
