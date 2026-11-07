# DOFBOT Description

ROS 2 Humble robot description for the smart factory mobile manipulator.

## Original source
https://github.com/God-Official/DofBot-PickNPlace

Based on the original DOFBOT description and gripper model.
Original source is distributed under the MIT License.
See LICENSE.

## Modifications
- Renamed arm base_link to arm_base_link.
- Renamed arm links to link1 through link5.
- Renamed arm joints to joint1 through joint5.

## Verification
- ROS 2 Humble colcon build: passed
- Xacro to URDF conversion: passed
- RViz model display and TF: checked
- Gripper motion in RViz: checked
- MoveIt 2 and Gazebo integration: not yet verified with this model
