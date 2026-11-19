"""/task_manager/state 발행 · 구독 QoS (interfaces.md 3장: transient_local).

depth만큼 지난 단계를 들고 있다가 늦게 붙은 구독자에게 보낸다. depth 1이면 IDLE이 WAIT_READY에 덮여
늦은 구독자가 IDLE을 못 받았다 (mock 시험). 구독하는 쪽도 같은 depth로 받아야 한 번에 온 이력을
버리지 않는다.

depth는 한 실행의 최대 상태 메시지 수 + 여유 5. 되돌아가는 반복은 GRASP → ALIGN(재정렬)뿐이라
최대 = IDLE, WAIT_READY, NAVIGATE, VERIFY_ARRIVAL + (ALIGN, GRASP) × (max_realign + 1)
+ CANCELING + FAILED = 2 * max_realign + 8개. 기본 max_realign 2 → 12개 → depth 17.
(예전 10은 재정렬 최대 + 시간 제한 취소 경로에서 IDLE · WAIT_READY를 잃었다.)
max_realign을 5 이상으로 올리면 최대 개수(18개 이상)가 depth를 넘으므로 이 값도 같이 올린다.
"""

from rclpy.qos import DurabilityPolicy, QoSProfile

STATE_QOS_DEPTH = 17   # 2 * max_realign(2) + 8 = 12, 여유 5


def state_qos():
    return QoSProfile(depth=STATE_QOS_DEPTH, durability=DurabilityPolicy.TRANSIENT_LOCAL)
