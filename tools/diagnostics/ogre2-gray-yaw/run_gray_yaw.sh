#!/usr/bin/env bash
# ogre2 카메라 회색 프레임 진단 (진단 전용, 프로덕션 코드 아님)
# Gazebo + 카메라만 띄우고 로봇을 제자리에서 한 바퀴 돌리면서 회색 프레임이 나오는 월드 yaw를 찾는다.
# Nav2 · YOLO · Align은 실행하지 않는다.
#
# 사용: tools/diagnostics/ogre2-gray-yaw/run_gray_yaw.sh [결과 폴더]
# 환경변수 (선택):
#   RENDER_ENGINE=ogre2|ogre   (기본 ogre2)
#   SOFTWARE_GL=false|true     (기본 false = GPU 사용. true면 CPU 렌더링)
#   WORLD=test_room.sdf        (mm_description/worlds 안의 월드 이름)
#   SPEED=0.35                 (회전 속도 rad/s. 작을수록 프레임 간 각도가 촘촘함)
set -uo pipefail

BUNDLE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd "$BUNDLE/../../.." && pwd)
ENGINE=${RENDER_ENGINE:-ogre2}
SOFTWARE_GL=${SOFTWARE_GL:-false}
WORLD=${WORLD:-test_room.sdf}
SPEED=${SPEED:-0.35}
OUT=${1:-$BUNDLE/results/$(date +%Y%m%d_%H%M%S)}
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)

fail() { echo "FAIL: $*" >&2; exit "${2:-1}"; }

for d in mm_description dofbot_description; do
  test -d "$ROOT/src/$d" || fail "$ROOT/src/$d 가 없습니다. 이 브랜치를 checkout한 레포 안에서 실행하세요." 10
done
test -f /opt/ros/humble/setup.bash || fail "/opt/ros/humble/setup.bash 가 없습니다 (ROS 2 Humble 필요)." 11

set +u; source /opt/ros/humble/setup.bash; set -u

if pgrep -f '[i]gn gazebo' >/dev/null; then
  fail "이미 실행 중인 Gazebo가 있습니다. 끄고 다시 실행하세요:  pkill -f 'ign gazebo'" 12
fi

# 1) mm_description, dofbot_description 빌드 (번들 안 .ws, 두 번째부터는 변경분만)
WS=$BUNDLE/.ws
echo "[1/4] mm_description, dofbot_description 빌드 중..."
B0=$(date +%s)
if ! PYTHONNOUSERSITE=1 colcon --log-base "$WS/log" build --symlink-install \
     --base-paths "$ROOT/src/mm_description" "$ROOT/src/dofbot_description" \
     --build-base "$WS/build" --install-base "$WS/install" >"$OUT/build.log" 2>&1; then
  tail -20 "$OUT/build.log"; fail "빌드 실패 (전체 로그: $OUT/build.log)" 13
fi
BUILD_S=$(( $(date +%s) - B0 ))
set +u; source "$WS/install/setup.bash"; set -u

# 2) Gazebo + 카메라 (sim.launch.py 그대로, RViz 없이 headless)
T0=$(date +%s)
echo "[2/4] Gazebo 시작: world=$WORLD render_engine=$ENGINE software_gl=$SOFTWARE_GL"
setsid ros2 launch mm_description sim.launch.py world:="$WORLD" render_engine:="$ENGINE" \
  software_gl:="$SOFTWARE_GL" headless:=true rviz:=false >"$OUT/sim.log" 2>&1 </dev/null &
LPID=$!
cleanup() {
  if kill -0 "$LPID" 2>/dev/null; then
    kill -INT -- "-$LPID" 2>/dev/null; sleep 3
    kill -KILL -- "-$LPID" 2>/dev/null
  fi
}
trap cleanup EXIT INT TERM

pubs() { local out; out=$(ros2 topic info "$1" 2>/dev/null) || true; awk -F': ' '/^Publisher count:/ {print $2; exit}' <<<"$out"; }
READY=0
for _ in $(seq 1 40); do
  kill -0 "$LPID" 2>/dev/null || break
  [[ "$(pubs /camera/image_raw)" -ge 1 && "$(pubs /odom)" -ge 1 ]] 2>/dev/null && { READY=1; break; }
  sleep 1
done
[ "$READY" = 1 ] || { tail -20 "$OUT/sim.log"; fail "/camera/image_raw 또는 /odom 이 40 s 안에 나타나지 않음 (로그: $OUT/sim.log)" 20; }

# 3) 팔을 Far 관측 자세로 (기존 팀 스크립트 그대로)
echo "[3/4] 팔 Far 관측 자세"
bash "$ROOT/src/mm_description/scripts/set_observe_pose_far.sh" >"$OUT/pose.log" 2>&1 || echo "주의: Far 자세 명령 실패 (로그: $OUT/pose.log)"
sleep 3

# 4) 제자리 한 바퀴 회전 + 회색 프레임 판정
echo "[4/4] 제자리 회전 (${SPEED} rad/s, 360°) + 회색 프레임 판정"
timeout 45 python3 "$BUNDLE/gray_yaw_probe.py" "$OUT" "$SPEED" | tee "$OUT/probe.txt"
PROBE_RC=${PIPESTATUS[0]}
RUN_S=$(( $(date +%s) - T0 ))
cleanup; trap - EXIT INT TERM

# 환경 정보
RENDERER=$(grep -m1 -o 'ignition-rendering-ogre[0-9]*' "$OUT/sim.log" || echo "확인 안 됨")
GL=$( (command -v glxinfo >/dev/null && { if [ "$SOFTWARE_GL" = true ]; then LIBGL_ALWAYS_SOFTWARE=1 glxinfo -B; else glxinfo -B; fi; } 2>/dev/null | awk -F': ' '/OpenGL renderer string/ {print $2; exit}') || true)
NVIDIA=$( (command -v nvidia-smi >/dev/null && nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null | head -1) || true)
WSL=no; grep -qi microsoft /proc/version 2>/dev/null && WSL=yes
DXG=no; test -e /dev/dxg && DXG=yes
. /etc/os-release 2>/dev/null
GZV=$(ign gazebo --versions 2>/dev/null | head -1)

{
echo "===== ogre2 회색 yaw 진단 결과 (아래 전체를 복사해서 보내 주세요) ====="
echo "branch/commit : $(git -C "$ROOT" rev-parse --abbrev-ref HEAD 2>/dev/null) $(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null)"
echo "OS            : ${PRETTY_NAME:-?} | kernel $(uname -r) | $(uname -m) | WSL=$WSL"
echo "ROS / Gazebo  : ${ROS_DISTRO:-?} / Gazebo $GZV"
echo "render engine : 요청 $ENGINE, 실제 로드 $RENDERER"
echo "GPU 가속      : software_gl=$SOFTWARE_GL | OpenGL renderer: ${GL:-glxinfo 없음} | nvidia-smi: ${NVIDIA:-없음} | /dev/dxg=$DXG"
echo "world / speed : $WORLD / $SPEED rad/s"
echo "시간          : 빌드 ${BUILD_S}s, Gazebo~회전 끝 ${RUN_S}s"
echo "--- 판정 ---"
cat "$OUT/probe.txt"
echo "결과 폴더     : $OUT"
echo "====================================================================="
} | tee "$OUT/report.txt"

exit "$PROBE_RC"
