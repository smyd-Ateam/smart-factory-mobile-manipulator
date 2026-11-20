#!/usr/bin/env bash
set -e

# Far observation pose (rad)
# Run after spawning mm_robot in Gazebo Fortress.

"$(dirname "${BASH_SOURCE[0]}")/safe_joint_cmd.sh" 1 "0.0"

"$(dirname "${BASH_SOURCE[0]}")/safe_joint_cmd.sh" 2 "0.0"

"$(dirname "${BASH_SOURCE[0]}")/safe_joint_cmd.sh" 3 "-1.1464"

"$(dirname "${BASH_SOURCE[0]}")/safe_joint_cmd.sh" 4 "-0.8237"

"$(dirname "${BASH_SOURCE[0]}")/safe_joint_cmd.sh" 5 "0.0"

echo "Far observation pose commands published."
