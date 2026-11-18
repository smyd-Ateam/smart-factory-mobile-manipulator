"""/perception/target_pose → 상태머신 TargetDistance 이벤트 (rclpy 없음, 노드에서 사용).

정렬(ALIGN) 중에만 물체 수평거리를 넘긴다. 전환 판단은 상태머신이 한다 (pose_transition.py).
frame_id가 base_link가 아니거나, x · y · z에 NaN · inf가 있거나, x <= 0이면 이벤트를 만들지 않는다
(target_pose.is_valid_target_pose, C 제안 안전장치).
"""

from mm_task_manager.machine import TargetDistance
from mm_task_manager.pose_transition import horizontal_distance
from mm_task_manager.states import State
from mm_task_manager.target_pose import is_valid_target_pose


def target_distance_event(state, frame_id, x, y, z, now):
    """state가 ALIGN이고 target_pose가 안전장치를 통과하면 (x, y) 수평거리의 TargetDistance, 아니면 None."""
    if state != State.ALIGN:
        return None
    if not is_valid_target_pose(frame_id, x, y, z):
        return None
    return TargetDistance(now=now, distance_m=horizontal_distance(x, y))
