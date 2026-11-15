#!/usr/bin/env bash
set -e

ign topic -t /mm_robot/joint1/cmd_pos \
  -m ignition.msgs.Double -p "data: 0.0"

ign topic -t /mm_robot/joint2/cmd_pos \
  -m ignition.msgs.Double -p "data: -0.636802"

ign topic -t /mm_robot/joint3/cmd_pos \
  -m ignition.msgs.Double -p "data: -0.959445"

ign topic -t /mm_robot/joint4/cmd_pos \
  -m ignition.msgs.Double -p "data: -1.435083"

ign topic -t /mm_robot/joint5/cmd_pos \
  -m ignition.msgs.Double -p "data: -0.000157"

echo "Pre-grasp pose commands published."
