#!/usr/bin/env bash
set -euo pipefail

# Usage:
# bash safe_joint_cmd.sh <joint_number> <angle>
# bash safe_joint_cmd.sh <joint_number> <angle> --dry-run

if [[ $# -lt 2 || $# -gt 3 ]]; then
    echo "Usage: $0 <joint 1-5> <angle rad> [--dry-run]"
    exit 1
fi

JOINT="$1"
ANGLE="$2"
MODE="${3:-}"

# 관절 번호 검사
if [[ ! "$JOINT" =~ ^[1-5]$ ]]; then
    echo "[ERROR] Joint must be 1-5"
    exit 1
fi

if [[ -n "$MODE" && "$MODE" != "--dry-run" ]]; then
    echo "[ERROR] Invalid option"
    exit 1
fi

# 모든 관절의 명령 제한: -1.57 ~ +1.57 rad
SAFE_ANGLE=$(python3 - "$ANGLE" <<'PY'
import sys
import math

angle = float(sys.argv[1])

if not math.isfinite(angle):
    raise ValueError("Angle must be finite")

LIMIT = 1.57

safe_angle = max(-LIMIT, min(LIMIT, angle))

if angle != safe_angle:
    print(
        f"[LIMIT] {angle} -> {safe_angle} rad",
        file=sys.stderr
    )

print(f"{safe_angle:.6f}")
PY
)

echo "[JOINT${JOINT}] target: ${SAFE_ANGLE} rad"

# 테스트 모드: 명령을 전송하지 않음
if [[ "$MODE" == "--dry-run" ]]; then
    echo "[DRY RUN] No command sent"
    exit 0
fi

# Gazebo에 목표 관절각 전송
ign topic -t "/mm_robot/joint${JOINT}/cmd_pos" \
    -m ignition.msgs.Double \
    -p "data: ${SAFE_ANGLE}"

