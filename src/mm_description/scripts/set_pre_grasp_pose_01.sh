#!/usr/bin/env bash
set -e

"$(dirname "${BASH_SOURCE[0]}")/safe_joint_cmd.sh" 1 "0.0"

"$(dirname "${BASH_SOURCE[0]}")/safe_joint_cmd.sh" 2 "-0.636802"

"$(dirname "${BASH_SOURCE[0]}")/safe_joint_cmd.sh" 3 "-0.959445"

"$(dirname "${BASH_SOURCE[0]}")/safe_joint_cmd.sh" 4 "-1.435083"

"$(dirname "${BASH_SOURCE[0]}")/safe_joint_cmd.sh" 5 "-0.000157"

echo "Pre-grasp pose commands published."
