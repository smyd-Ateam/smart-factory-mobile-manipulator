#!/usr/bin/env bash
set -e

"$(dirname "${BASH_SOURCE[0]}")/safe_joint_cmd.sh" 1 "0.0"

"$(dirname "${BASH_SOURCE[0]}")/safe_joint_cmd.sh" 2 "-0.008482"

"$(dirname "${BASH_SOURCE[0]}")/safe_joint_cmd.sh" 3 "-1.282087"

"$(dirname "${BASH_SOURCE[0]}")/safe_joint_cmd.sh" 4 "-1.299052"

"$(dirname "${BASH_SOURCE[0]}")/safe_joint_cmd.sh" 5 "-0.000157"

echo "Near observation pose commands published."
