#!/usr/bin/env bash
set -e

ign topic -t /mm_robot/joint1/cmd_pos \
  -m ignition.msgs.Double -p "data: 0.0"

ign topic -t /mm_robot/joint2/cmd_pos \
  -m ignition.msgs.Double -p "data: -1.282087"

ign topic -t /mm_robot/joint3/cmd_pos \
  -m ignition.msgs.Double -p "data: -0.602873"

ign topic -t /mm_robot/joint4/cmd_pos \
  -m ignition.msgs.Double -p "data: -1.231193"

ign topic -t /mm_robot/joint5/cmd_pos \
  -m ignition.msgs.Double -p "data: -0.000157"

echo "Grasp pose commands published."
