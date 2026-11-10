from enum import Enum


class State(Enum):
    IDLE = 'IDLE'
    WAIT_READY = 'WAIT_READY'
    NAVIGATE = 'NAVIGATE'
    VERIFY_ARRIVAL = 'VERIFY_ARRIVAL'
    ALIGN = 'ALIGN'
    GRASP = 'GRASP'
    CANCELING = 'CANCELING'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'


TERMINAL_STATES = frozenset({State.SUCCEEDED, State.FAILED})


class StateReporter:
    """/task_manager/state 발행 판단: 단계가 바뀔 때만 이름을 돌려준다 (interfaces.md 3장)."""

    def __init__(self):
        self._last = None

    def update(self, state):
        if state == self._last:
            return None
        self._last = state
        return state.value
