"""Align 목표 하나의 진행 판단 (설계 문서 2.3, 2.5).

ROS에 의존하지 않는다. 액션 서버는 제어 주기마다 tick()을 부르고,
돌려받은 속도 명령을 보내고, 끝났으면 결과를 반환한다.

판단 순서
1. 제한 시간 초과 → ALIGN_TIMEOUT
2. 새 측정을 lost_timeout 동안 못 받음 → OBJECT_NOT_FOUND
   (시작 시점부터 한 번도 못 본 경우, 정렬 중 놓친 경우 모두)
3. 측정값이 max_age보다 오래됨 → 정지하고 기다림
4. 이동 중: P제어. 허용 범위 * stop_ratio 안이면 정지
5. 정지 후: 정지 시각 + settle 이후에 찍힌 새 측정만 센다.
   허용 범위 안이 required_frames번 연속이면 성공, 벗어나면 다시 이동
"""
from dataclasses import dataclass

from visual_servoing.control import ControlParams, compute_cmd, within_tolerance
from visual_servoing.geometry import range_bearing_error

RUNNING = 'RUNNING'
SUCCEEDED = 'SUCCEEDED'
FAILED = 'FAILED'

ALIGN_TIMEOUT = 'ALIGN_TIMEOUT'
OBJECT_NOT_FOUND = 'OBJECT_NOT_FOUND'


@dataclass
class SessionParams:
    lost_timeout: float = 1.0      # s, 새 측정이 이만큼 안 오면 놓침. C 기준(10프레임)과 맞춘 값
    max_age: float = 1.0           # s, 이보다 오래 지연된 측정으로는 움직이지 않음 (2.4 기준)
    settle: float = 0.1            # s, 정지 명령 후 차체가 멈출 때까지 기다리는 시간
    required_frames: int = 3       # 정지 후 연속으로 허용 범위 안이어야 하는 프레임 수


@dataclass
class TickResult:
    status: str
    vx: float = 0.0
    wz: float = 0.0
    error_code: str = ''
    target_visible: bool = False
    e_r: float = float('nan')
    e_b: float = float('nan')
    phase: str = ''


class AlignSession:

    def __init__(self, standoff, timeout, start_time, control=None, params=None):
        self.standoff = standoff
        self.timeout = timeout
        self.start = start_time
        self.cp = control or ControlParams()
        self.sp = params or SessionParams()
        self.stopped_at = None       # 정지 명령을 보낸 시각
        self.good_frames = 0
        self.last_counted = None     # 마지막으로 센 측정의 stamp
        self.last_error = (float('nan'), float('nan'))
        self.last_stamp = None       # 마지막으로 받은 측정의 stamp
        self.last_received = start_time   # 새 측정을 마지막으로 받은 시각

    def tick(self, now, measurement):
        """measurement: (stamp, x_b, y_b) 또는 None. 시각은 모두 초 단위 같은 시계."""
        if now - self.start > self.timeout:
            return self._end(FAILED, ALIGN_TIMEOUT)

        # 물체 놓침은 "새 측정을 마지막으로 받은 시각" 기준으로 판단한다.
        # stamp 기준으로 하면 측정 지연만큼 항상 늦어 보여서 지연이 크면 바로 실패한다.
        fresh_enough = measurement is not None and measurement[0] >= self.start
        if fresh_enough and measurement[0] != self.last_stamp:
            self.last_stamp = measurement[0]
            self.last_received = now
        if now - self.last_received > self.sp.lost_timeout:
            return self._end(FAILED, OBJECT_NOT_FOUND)
        if not fresh_enough:
            return TickResult(RUNNING, phase='WAIT_DETECTION')

        stamp, x_b, y_b = measurement
        e_r, e_b = range_bearing_error(x_b, y_b, self.standoff)
        self.last_error = (e_r, e_b)

        if now - stamp > self.sp.max_age:
            return TickResult(RUNNING, target_visible=False, e_r=e_r, e_b=e_b,
                              phase='WAIT_FRESH')

        if self.stopped_at is None:
            vx, wz = compute_cmd(e_r, e_b, self.cp)
            if (vx, wz) == (0.0, 0.0):
                self.stopped_at = now
                self.good_frames = 0
                self.last_counted = None
                return TickResult(RUNNING, target_visible=True, e_r=e_r, e_b=e_b,
                                  phase='STOPPING')
            return TickResult(RUNNING, vx, wz, target_visible=True, e_r=e_r, e_b=e_b,
                              phase='MOVING')

        # 정지 후 판정: 멈춘 뒤 찍힌 새 프레임만 센다
        if stamp < self.stopped_at + self.sp.settle or stamp == self.last_counted:
            return TickResult(RUNNING, target_visible=True, e_r=e_r, e_b=e_b, phase='CHECKING')
        self.last_counted = stamp
        if within_tolerance(e_r, e_b, self.cp):
            self.good_frames += 1
            if self.good_frames >= self.sp.required_frames:
                return self._end(SUCCEEDED, '', e_r, e_b)
            return TickResult(RUNNING, target_visible=True, e_r=e_r, e_b=e_b, phase='CHECKING')

        # 멈춰서 보니 허용 범위 밖 → 다시 이동
        self.stopped_at = None
        vx, wz = compute_cmd(e_r, e_b, self.cp)
        return TickResult(RUNNING, vx, wz, target_visible=True, e_r=e_r, e_b=e_b, phase='MOVING')

    def _end(self, status, code, e_r=None, e_b=None):
        if e_r is None:
            e_r, e_b = self.last_error
        return TickResult(status, 0.0, 0.0, error_code=code, e_r=e_r, e_b=e_b,
                          target_visible=status == SUCCEEDED, phase=status)
