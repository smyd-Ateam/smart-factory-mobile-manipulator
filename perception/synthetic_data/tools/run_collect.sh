#!/usr/bin/env bash
# YOLO 합성 데이터 수집을 한 번에 실행한다.
#   1) Gazebo 서버 (화면 없이, ogre2, 소프트웨어 렌더링)
#   2) ros_gz_bridge (/boxes_image → ROS2)
#   3) collect_data.py
# 끝나거나 Ctrl+C를 누르면 Gazebo와 브리지도 같이 종료된다.
#
# 사용법: bash tools/run_collect.sh [장수=200] [저장 폴더=~/yolo_dataset]
set -e
cd "$(dirname "$0")/.."

NUM=${1:-200}
OUT=${2:-$HOME/yolo_dataset}
mkdir -p "$OUT/logs"

source /opt/ros/humble/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$PWD/models:${IGN_GAZEBO_RESOURCE_PATH}

echo "[1/3] Gazebo 서버 시작 (로그: $OUT/logs/gazebo.log)"
LIBGL_ALWAYS_SOFTWARE=1 setsid ign gazebo -s -r worlds/collect.sdf > "$OUT/logs/gazebo.log" 2>&1 &
GZ_PID=$!

echo "[2/3] 브리지 시작 (로그: $OUT/logs/bridge.log)"
setsid ros2 run ros_gz_bridge parameter_bridge \
  "/boxes_image@sensor_msgs/msg/Image[ignition.msgs.Image" > "$OUT/logs/bridge.log" 2>&1 &
BR_PID=$!

cleanup() {
  echo "[정리] Gazebo와 브리지 종료"
  kill -INT -- -"$BR_PID" 2>/dev/null || true
  kill -INT -- -"$GZ_PID" 2>/dev/null || true
  sleep 2
  kill -KILL -- -"$BR_PID" 2>/dev/null || true
  kill -KILL -- -"$GZ_PID" 2>/dev/null || true
}
trap cleanup EXIT

sleep 8
echo "[3/3] 수집 시작: ${NUM}장 → $OUT"
python3 tools/collect_data.py --num "$NUM" --out "$OUT"
