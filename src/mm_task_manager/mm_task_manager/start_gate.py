"""task_manager 출발(IDLE → WAIT_READY) 조건 (rclpy 없음).

required_subscribers > 0이면 /task_manager/state 구독자가 그 수만큼 연결된 뒤에 출발한다.
mock 시험에서 recorder가 늦게 연결돼(새 노드 discovery 지연, 2~3 s) WAIT_READY를 놓치지 않게
하려는 것. 지금은 상태 토픽 depth 17(state_qos.py)으로 늦게 붙어도 지난 단계를 받으므로 추가 안전장치.
기본값 0이면 예전과 같다 (autostart이고 /clock을 받으면 바로 출발).
"""

import math


def can_start(autostart, now, state_subscribers, required_subscribers):
    return autostart and now > 0.0 and state_subscribers >= required_subscribers


def waiting_for_subscribers(autostart, now, state_subscribers, required_subscribers):
    """구독자 수 때문에만 출발을 못 하고 있으면 True (/clock 대기 · autostart 꺼짐은 아님)."""
    return autostart and now > 0.0 and state_subscribers < required_subscribers


class SubscriberWait:
    """구독자 대기 경고 시점 (#43). 시간 제한은 두지 않고 기다리는 중이라는 것만 알린다.

    계속 기다린 지 period_sec가 되면 처음, 그 뒤로 period_sec마다 기다린 시간(s)을 돌려준다.
    그 밖에는 None. 기다림이 끝나면(waiting False) 처음부터 다시 잰다. now는 단조 증가 시각(s).
    """

    def __init__(self, period_sec=5.0):
        if not math.isfinite(period_sec) or period_sec <= 0.0:
            raise ValueError(f'period_sec must be finite and > 0 (got {period_sec})')
        self._period = period_sec
        self._since = None
        self._next = None

    def update(self, now, waiting):
        if not waiting:
            self._since = None
            return None
        if self._since is None:
            self._since = now
            self._next = now + self._period
            return None
        if now < self._next:
            return None
        while self._next <= now:
            self._next += self._period
        return now - self._since
