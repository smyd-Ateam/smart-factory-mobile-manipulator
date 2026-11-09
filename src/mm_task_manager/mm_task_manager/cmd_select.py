"""단계별 /cmd_vel 입력 선택 (docs/interfaces.md 5.0).

명령 값은 그대로 전달만 하므로 타입을 따지지 않는다 (노드에서는 Twist).
output()이 None이면 노드가 속도 0을 발행한다.
"""

from mm_task_manager.states import State

NAV = 'nav'        # /nav2/cmd_vel
SERVO = 'servo'    # /servoing/cmd_vel
ZERO = 'zero'      # 속도 0

_INPUTS = (NAV, SERVO)
_SOURCES = (NAV, SERVO, ZERO)


def source_for(state):
    if state == State.NAVIGATE:
        return NAV
    if state == State.ALIGN:
        return SERVO
    return ZERO


class CmdSelector:

    def __init__(self, input_timeout_sec):
        self._timeout = input_timeout_sec
        self._latest = {}            # 입력 이름 → (명령, 수신 시각)
        self._selected = ZERO
        self._switched_at = None

    @property
    def selected(self):
        return self._selected

    def select(self, source, now):
        if source not in _SOURCES:
            raise ValueError(f'unknown source: {source}')
        if source != self._selected:
            self._selected = source
            self._switched_at = now

    def update(self, source, cmd, now):
        if source not in _INPUTS:
            raise ValueError(f'unknown input: {source}')
        self._latest[source] = (cmd, now)

    def output(self, now):
        if self._selected == ZERO:
            return None
        latest = self._latest.get(self._selected)
        if latest is None:
            return None
        cmd, stamp = latest
        if self._switched_at is not None and stamp < self._switched_at:
            return None              # 단계가 바뀌기 전에 받은 명령
        if now - stamp > self._timeout:
            return None              # 입력 끊김
        return cmd
