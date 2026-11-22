#!/usr/bin/env bash
# 통합 1단계 실제 시험 1회 (카메라 확인 후 진행). 사용: TAG=<이름> stage1_all.sh <target_class>
S=${STAGE1_DIR:-/tmp/sfmm-brass-v7}   # 시험 폴더
ALIGN_SETUP=${ALIGN_SETUP:-/tmp/sfmm-align-reliable/install/setup.bash}   # visual_servoing 90df7d2 빌드
TARGET=${1:-block}; TAG=${TAG:-$TARGET}; export TAG; source $S/env_s1.sh; _sfmm_stage_opts=$-; set +u; source "$ALIGN_SETUP"; case $_sfmm_stage_opts in *u*) set -u;; esac; unset _sfmm_stage_opts; cd $S
clean() { pkill -f "[i]gn gazebo"; pkill -f "[p]arameter_bridge"; pkill -f "[r]obot_state_publisher"; pkill -f "[o]pt/ros/humble/lib/nav2_"
          pkill -f "[y]olo_detector --ros-args"; pkill -f "[a]lign_node --ros-args"; sleep 3
          for p in "[i]gn gazebo" "[p]arameter_bridge" "[r]obot_state_publisher" "[o]pt/ros/humble/lib/nav2_"; do pkill -KILL -f "$p"; done; sleep 1; }
RENDER_FAIL=0; CAM_OK=0
for attempt in 1 2 3 4; do      # 처음 1번 + 재시작 최대 3번
  clean >/dev/null 2>&1
  echo "[$TAG attempt $attempt pre-check $(date -u +%T)] gazebo $(pgrep -fc '[i]gn gazebo') nav2 $(pgrep -fc '[o]pt/ros/humble/lib/nav2_')"
  timeout 300 ros2 launch mm_navigation nav2.launch.py world:=yolo_test_room.sdf render_engine:=ogre2 software_gl:=true headless:=true rviz:=false standalone:=true > $S/s1_nav_${TAG}.log 2>&1 &
  LPID=$!
  t0=$(date +%s); SPAWN=1
  until timeout 5 ign model --list 2>/dev/null | grep -q mm_robot; do [ $(( $(date +%s)-t0 )) -ge 90 ] && { SPAWN=0; break; }; sleep 2; done
  if [ $SPAWN = 0 ]; then echo "[$TAG attempt $attempt] SPAWN TIMEOUT"; kill $LPID 2>/dev/null; continue; fi
  echo "[$TAG attempt $attempt] spawned; renderer: $(grep -m1 -o 'ignition-rendering-ogre[0-9]*' $S/s1_nav_${TAG}.log)"
  sleep 5
  G=$(timeout 120 bash tools/cam_gate.sh $S/s1_camgate_${TAG}_$attempt.json); GS=$?
  echo "[$TAG attempt $attempt] camera gate: $G"
  if [ $GS = 0 ]; then CAM_OK=1; break; fi
  RENDER_FAIL=$((RENDER_FAIL+1)); echo "[$TAG attempt $attempt] RENDER_FAIL"
  kill $LPID 2>/dev/null; sleep 3
done
echo "[$TAG] render_fail_count=$RENDER_FAIL cam_ok=$CAM_OK"
echo "{\"tag\": \"$TAG\", \"render_fail_count\": $RENDER_FAIL, \"cam_ok\": $CAM_OK}" > $S/s1_gate_${TAG}.json
if [ $CAM_OK = 1 ]; then
  echo "[$TAG] load before run: $(cut -d' ' -f1-3 /proc/loadavg)"
  timeout 280 python3 tools/stage1_run.py $S/s1_result_${TAG}.json $TARGET "${GOAL:-}"
  echo "[$TAG] yolo infer: $(grep -o '추론 평균 [0-9]* ms' $S/s1_yolo_${TAG}.log | tr '\n' ' ')"
fi
kill $LPID 2>/dev/null; sleep 4; clean >/dev/null 2>&1
echo "[$TAG cleaned $(date -u +%T)] gazebo $(pgrep -fc '[i]gn gazebo') nav2 $(pgrep -fc '[o]pt/ros/humble/lib/nav2_')"
