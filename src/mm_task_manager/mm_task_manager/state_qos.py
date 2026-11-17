"""/task_manager/state 발행 · 구독 QoS (interfaces.md 3장: transient_local).

depth만큼 지난 단계를 들고 있다가 늦게 붙은 구독자에게 보낸다. depth 1이면 IDLE이 WAIT_READY에 덮여
늦은 구독자가 IDLE을 못 받았다 (mock 시험). 새 노드 discovery가 약 3 s까지 걸린 측정 기준으로
그 사이 단계(IDLE · WAIT_READY · NAVIGATE)에 충분한 여유, 정상 실행 전체(7단계)도 들어가는 값.
구독하는 쪽도 같은 depth로 받아야 한 번에 온 이력을 버리지 않는다.
"""

from rclpy.qos import DurabilityPolicy, QoSProfile

STATE_QOS_DEPTH = 10


def state_qos():
    return QoSProfile(depth=STATE_QOS_DEPTH, durability=DurabilityPolicy.TRANSIENT_LOCAL)
