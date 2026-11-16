"""/perception/target_pose → 상태머신 TargetDistance 이벤트 (rclpy 없음, 노드에서 사용).

정렬(ALIGN) 중에만 물체 수평거리를 넘긴다. 전환 판단은 상태머신이 한다 (pose_transition.py).
NaN · inf도 그대로 넘기고, 상태머신이 기준과 비교할 때 걸러진다.
"""

from mm_task_manager.machine import TargetDistance
from mm_task_manager.pose_transition import horizontal_distance
from mm_task_manager.states import State


def target_distance_event(state, x, y, now):
    """state가 ALIGN이면 base_link 기준 (x, y)의 TargetDistance, 아니면 None."""
    if state != State.ALIGN:
        return None
    return TargetDistance(now=now, distance_m=horizontal_distance(x, y))
