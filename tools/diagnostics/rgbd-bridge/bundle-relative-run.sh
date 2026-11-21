#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env_s1.sh"
OUT_DIR="${1:-$SCRIPT_DIR/results}"
mkdir -p "$OUT_DIR"
for model in block pin hex_nut bolt; do test -d "$SCRIPT_DIR/models/$model"; done
command -v ros2 >/dev/null
command -v ign >/dev/null
pkg-config --exists ignition-transport11 ignition-msgs8 ignition-math6
g++ -std=c++17 -O2 "$SCRIPT_DIR/collectors/gz_stamp_collect.cpp" -o "$OUT_DIR/gz_stamp_collect" $(pkg-config --cflags --libs ignition-transport11 ignition-msgs8 ignition-math6)
LAUNCH_PID=''; GZ_PID=''; ROS_PID=''
cleanup() {
  [[ -n "$GZ_PID" ]] && kill -TERM "$GZ_PID" 2>/dev/null || true
  [[ -n "$ROS_PID" ]] && kill -TERM "$ROS_PID" 2>/dev/null || true
  [[ -n "$LAUNCH_PID" ]] && kill -TERM "$LAUNCH_PID" 2>/dev/null || true
  [[ -n "$GZ_PID" ]] && wait "$GZ_PID" 2>/dev/null || true
  [[ -n "$ROS_PID" ]] && wait "$ROS_PID" 2>/dev/null || true
  [[ -n "$LAUNCH_PID" ]] && wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
ros2 launch "$SCRIPT_DIR/min_split_bridge.launch.py" world:="$SCRIPT_DIR/worlds/yolo_test_room.sdf" render_engine:=ogre2 software_gl:=true headless:=true >"$OUT_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!
for _ in $(seq 1 90); do
  ros2 topic info /camera/image_raw 2>/dev/null | grep -q 'Publisher count: 1' &&
  ros2 topic info /camera/depth/image_raw 2>/dev/null | grep -q 'Publisher count: 1' && break
  sleep 1
done
ros2 topic info /camera/image_raw -v >"$OUT_DIR/ros_rgb_topic.txt"
ros2 topic info /camera/depth/image_raw -v >"$OUT_DIR/ros_depth_topic.txt"
grep -q 'Publisher count: 1' "$OUT_DIR/ros_rgb_topic.txt"
grep -q 'Publisher count: 1' "$OUT_DIR/ros_depth_topic.txt"
grep -q 'rgb_parameter_bridge' "$OUT_DIR/ros_rgb_topic.txt"
grep -q 'depth_parameter_bridge' "$OUT_DIR/ros_depth_topic.txt"
ign topic -i -t /rgbd_camera/image >"$OUT_DIR/gz_rgb_topic.txt"
ign topic -i -t /rgbd_camera/depth_image >"$OUT_DIR/gz_depth_topic.txt"
grep -q 'ignition.msgs.Image' "$OUT_DIR/gz_rgb_topic.txt"
grep -q 'ignition.msgs.Image' "$OUT_DIR/gz_depth_topic.txt"
"$OUT_DIR/gz_stamp_collect" "$OUT_DIR/gz.jsonl" "$OUT_DIR/gz.ready" 40 >"$OUT_DIR/gz_collector.log" 2>&1 & GZ_PID=$!
python3 "$SCRIPT_DIR/collectors/ros_stamp_collect.py" "$OUT_DIR/ros.jsonl" "$OUT_DIR/ros.ready" 40 >"$OUT_DIR/ros_collector.log" 2>&1 & ROS_PID=$!
for _ in $(seq 1 30); do test -s "$OUT_DIR/gz.ready" && test -s "$OUT_DIR/ros.ready" && break; sleep 1; done
kill -0 "$GZ_PID"; kill -0 "$ROS_PID"; kill -0 "$LAUNCH_PID"
test "$(wc -l < "$OUT_DIR/gz.jsonl")" -ge 1; test "$(wc -l < "$OUT_DIR/ros.jsonl")" -ge 1
START_NS=$(date +%s%N); END_NS=$((START_NS+30000000000))
printf 'start_wall_ns=%s\nend_wall_ns=%s\n' "$START_NS" "$END_NS" >"$OUT_DIR/window.txt"
while (( $(date +%s%N) < END_NS )); do sleep 0.1; done
cleanup
trap - EXIT INT TERM
echo "30-second metadata capture complete: $OUT_DIR"
