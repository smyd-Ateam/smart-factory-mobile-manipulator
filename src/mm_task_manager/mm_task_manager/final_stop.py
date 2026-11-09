"""종료 처리: 끝난 뒤 /cmd_vel 0을 duration_sec 동안 보내고 노드를 종료한다.

DiffDrive는 마지막 명령을 유지하므로 마지막 명령을 0으로 남기고 끝낸다.
노드가 계속 0을 보내면 다음 실행의 명령과 섞이므로(2026-11-09 확인) 종료한다.
시각은 wall time (/clock이 멈춰도 종료되도록).
"""


class FinalStop:

    def __init__(self, duration_sec):
        self.duration = duration_sec
        self._start = None

    @property
    def started(self):
        return self._start is not None

    def start(self, now):
        if self._start is None:
            self._start = now

    def should_exit(self, now):
        return self._start is not None and now - self._start >= self.duration
