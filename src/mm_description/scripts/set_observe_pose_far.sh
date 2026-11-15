#!/usr/bin/env bash
set -e

# Far observation pose (rad)
# Run after spawning mm_robot in Gazebo Fortress.

ign topic -t /mm_robot/joint1/cmd_pos \
  -m ignition.msgs.Double -p "data: 0.0"

ign topic -t /mm_robot/joint2/cmd_pos \
  -m ignition.msgs.Double -p "data: 0.0"

ign topic -t /mm_robot/joint3/cmd_pos \
  -m ignition.msgs.Double -p "data: -1.1464"

ign topic -t /mm_robot/joint4/cmd_pos \
  -m ignition.msgs.Double -p "data: -0.8237"

ign topic -t /mm_robot/joint5/cmd_pos \
  -m ignition.msgs.Double -p "data: 0.0"

echo "Far observation pose commands published."
