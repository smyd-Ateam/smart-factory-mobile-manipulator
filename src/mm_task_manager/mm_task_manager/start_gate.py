"""task_manager 출발(IDLE → WAIT_READY) 조건 (rclpy 없음).

required_subscribers > 0이면 /task_manager/state 구독자가 그 수만큼 연결된 뒤에 출발한다.
mock 시험에서 recorder가 늦게 연결돼(새 노드 discovery 지연, 2~3 s) WAIT_READY를 놓치지 않게
하려는 것. 지금은 상태 토픽 depth 17(state_qos.py)으로 늦게 붙어도 지난 단계를 받으므로 추가 안전장치.
기본값 0이면 예전과 같다 (autostart이고 /clock을 받으면 바로 출발).
"""


def can_start(autostart, now, state_subscribers, required_subscribers):
    return autostart and now > 0.0 and state_subscribers >= required_subscribers
