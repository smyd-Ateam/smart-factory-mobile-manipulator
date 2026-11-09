"""작업 상태머신 로직 (rclpy 없음).

노드가 이벤트(결과, 시각, TF 샘플)를 handle()에 넣으면, 해야 할 명령 목록을 돌려준다.
시각은 모두 sim time 초(float). /cmd_vel 선택은 cmd_select.source_for(machine.state)로 정한다.
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple

from mm_task_manager import codes
from mm_task_manager.geometry import arrival_error, arrival_ok, tf_fresh
from mm_task_manager.retry import REALIGN, after_grasp_failure, standoff_for
from mm_task_manager.states import TERMINAL_STATES, State

# NavResult.status
NAV_SUCCEEDED = 'succeeded'
NAV_ABORTED = 'aborted'
NAV_CANCELED = 'canceled'
NAV_REJECTED = 'rejected'

_NAV_FAIL_DETAIL = {
    NAV_ABORTED: codes.ABORTED,
    NAV_CANCELED: codes.CANCELED,
    NAV_REJECTED: codes.REJECTED,
}

# 목표가 진행 중인 상태 (/clock 정지 시 취소 대상). CANCELING은 이미 취소를 보냈으므로 제외
_GOAL_STATES = (State.NAVIGATE, State.ALIGN, State.GRASP)


# ---- 이벤트 ----

@dataclass(frozen=True)
class Start:
    now: float


@dataclass(frozen=True)
class Tick:
    now: float


@dataclass(frozen=True)
class ReadyStatus:
    now: float
    localization_active: Optional[bool]     # None = 아직 응답 없음
    navigation_active: Optional[bool]
    nav_server_ready: bool
    map_odom_stamp: Optional[float]         # None = 조회 실패


@dataclass(frozen=True)
class NavResult:
    now: float
    token: int
    status: str


@dataclass(frozen=True)
class PoseSample:
    now: float
    pose: Optional[Tuple[float, float, float]]   # map→base_link (x, y, yaw), None = 조회 실패
    map_odom_stamp: Optional[float]


@dataclass(frozen=True)
class AlignResult:
    now: float
    token: int
    success: bool
    error_code: str = ''
    detail: str = ''


@dataclass(frozen=True)
class GraspResult:
    now: float
    token: int
    success: bool
    error_code: str = ''
    detail: str = ''


@dataclass(frozen=True)
class CancelDone:
    now: float
    token: int


@dataclass(frozen=True)
class ClockStopped:
    now: float      # 마지막으로 받은 sim time


# ---- 명령 ----

@dataclass(frozen=True)
class PublishTargetClass:
    target_class: str


@dataclass(frozen=True)
class SendNavGoal:
    token: int
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class SendAlignGoal:
    token: int
    target_class: str
    standoff_m: float
    timeout_sec: float


@dataclass(frozen=True)
class SendGraspGoal:
    token: int
    target_class: str
    max_pose_age_sec: float     # 노드가 /perception/target_pose 신선도 확인에 사용


@dataclass(frozen=True)
class CancelGoal:
    token: int


@dataclass(frozen=True)
class WriteRecord:
    record: dict = field(hash=False)


@dataclass
class _PendingFailure:
    error_code: str
    detail: str
    stage: State


class TaskMachine:

    def __init__(self, mission, run_id):
        self.m = mission
        self.run_id = run_id
        self.state = State.IDLE

        self._token = 0
        self._active_token = None
        self._deadline = None
        self._settle_until = None
        self._pending = None
        self._ready_reason = codes.NO_READY_STATUS

        self._realign_count = 0
        self._standoffs_used = []
        self._arrival = None
        self._start_time = None
        self._state_entered = None
        self._durations = {}

    # ---- 진입점 ----

    def handle(self, event):
        if self.state in TERMINAL_STATES:
            return []
        if self.state == State.IDLE:
            return self._on_start(event) if isinstance(event, Start) else []
        if isinstance(event, ClockStopped):
            return self._on_clock_stopped(event)

        handler = {
            State.WAIT_READY: self._wait_ready,
            State.NAVIGATE: self._navigate,
            State.VERIFY_ARRIVAL: self._verify_arrival,
            State.ALIGN: self._align,
            State.GRASP: self._grasp,
            State.CANCELING: self._canceling,
        }[self.state]
        return handler(event)

    # ---- 상태별 처리 ----

    def _on_start(self, ev):
        self._start_time = ev.now
        self._enter(State.WAIT_READY, ev.now)
        self._deadline = ev.now + self.m.ready_timeout_sec
        return [PublishTargetClass(target_class=self.m.target_class)]

    def _wait_ready(self, ev):
        if isinstance(ev, ReadyStatus):
            reason = self._not_ready_reason(ev)
            if reason is None:
                return self._send_nav(ev.now)
            self._ready_reason = reason
            return []
        if isinstance(ev, Tick) and ev.now > self._deadline:
            return self._finish(State.FAILED, ev.now, codes.NAV_NOT_READY, self._ready_reason,
                                failed_stage=State.WAIT_READY)
        return []

    def _navigate(self, ev):
        if isinstance(ev, NavResult) and ev.token == self._active_token:
            if ev.status == NAV_SUCCEEDED:
                self._enter(State.VERIFY_ARRIVAL, ev.now)
                self._active_token = None
                self._settle_until = ev.now + self.m.settle_sec
                return []
            detail = _NAV_FAIL_DETAIL.get(ev.status, codes.UNKNOWN)
            return self._finish(State.FAILED, ev.now, codes.NAV_FAILED, detail,
                                failed_stage=State.NAVIGATE)
        if isinstance(ev, Tick) and ev.now > self._deadline:
            return self._start_cancel(ev.now, codes.NAV_FAILED, State.NAVIGATE)
        return []

    def _verify_arrival(self, ev):
        if not isinstance(ev, PoseSample) or ev.now < self._settle_until:
            return []
        if ev.pose is None or not tf_fresh(ev.map_odom_stamp, ev.now, self.m.tf_max_age_sec):
            return self._finish(State.FAILED, ev.now, codes.NAV_FAILED, codes.TF_STALE,
                                failed_stage=State.VERIFY_ARRIVAL)
        goal = (self.m.goal_x, self.m.goal_y, self.m.goal_yaw)
        dist, yaw_err = arrival_error(goal, ev.pose)
        self._arrival = {'dist': dist, 'yaw_err': yaw_err}
        if not arrival_ok(dist, yaw_err, self.m.arrival_xy_tol, self.m.arrival_yaw_tol):
            return self._finish(State.FAILED, ev.now, codes.NAV_FAILED, codes.ARRIVAL_CHECK,
                                failed_stage=State.VERIFY_ARRIVAL)
        if self.m.nav_only:
            return self._finish(State.SUCCEEDED, ev.now)
        return self._send_align(ev.now)

    def _align(self, ev):
        if isinstance(ev, AlignResult) and ev.token == self._active_token:
            if ev.success:
                return self._send_grasp(ev.now)
            return self._finish(State.FAILED, ev.now, ev.error_code or codes.UNKNOWN, ev.detail,
                                failed_stage=State.ALIGN)
        if isinstance(ev, Tick) and ev.now > self._deadline:
            return self._start_cancel(ev.now, self.m.align_sm_timeout_code, State.ALIGN)
        return []

    def _grasp(self, ev):
        if isinstance(ev, GraspResult) and ev.token == self._active_token:
            if ev.success:
                return self._finish(State.SUCCEEDED, ev.now)
            code = ev.error_code or codes.UNKNOWN
            if after_grasp_failure(code, self._realign_count, self.m.max_realign) == REALIGN:
                self._realign_count += 1
                return self._send_align(ev.now)
            return self._finish(State.FAILED, ev.now, code, ev.detail, failed_stage=State.GRASP)
        if isinstance(ev, Tick) and ev.now > self._deadline:
            return self._start_cancel(ev.now, self.m.grasp_timeout_code, State.GRASP)
        return []

    def _canceling(self, ev):
        p = self._pending
        token_done = (isinstance(ev, CancelDone)
                      or isinstance(ev, (NavResult, AlignResult, GraspResult)))
        if token_done and ev.token == self._active_token:
            return self._finish(State.FAILED, ev.now, p.error_code, p.detail,
                                failed_stage=p.stage)
        if isinstance(ev, Tick) and ev.now > self._deadline:
            return self._finish(State.FAILED, ev.now, p.error_code, p.detail,
                                failed_stage=p.stage, aborted=True, cancel_no_response=True)
        return []

    def _on_clock_stopped(self, ev):
        cmds = []
        if self.state in _GOAL_STATES and self._active_token is not None:
            cmds.append(CancelGoal(token=self._active_token))
        stage = self._pending.stage if self.state == State.CANCELING else self.state
        cmds += self._finish(State.FAILED, ev.now, codes.UNKNOWN, codes.CLOCK_STOPPED,
                             failed_stage=stage, aborted=True)
        return cmds

    # ---- 도우미 ----

    def _not_ready_reason(self, ev):
        if ev.localization_active is not True or ev.navigation_active is not True:
            return codes.LIFECYCLE_INACTIVE
        if not ev.nav_server_ready:
            return codes.NAV_SERVER_UNAVAILABLE
        if not tf_fresh(ev.map_odom_stamp, ev.now, self.m.tf_max_age_sec):
            return codes.TF_STALE
        return None

    def _new_token(self):
        self._token += 1
        self._active_token = self._token
        return self._token

    def _send_nav(self, now):
        self._enter(State.NAVIGATE, now)
        self._deadline = now + self.m.nav_timeout_sec
        return [SendNavGoal(token=self._new_token(), x=self.m.goal_x, y=self.m.goal_y,
                            yaw=self.m.goal_yaw)]

    def _send_align(self, now):
        self._enter(State.ALIGN, now)
        self._deadline = now + self.m.align_timeout_sec + self.m.align_margin_sec
        standoff = standoff_for(self._realign_count, self.m.standoff_list)
        self._standoffs_used.append(standoff)
        return [SendAlignGoal(token=self._new_token(), target_class=self.m.target_class,
                              standoff_m=standoff, timeout_sec=self.m.align_timeout_sec)]

    def _send_grasp(self, now):
        self._enter(State.GRASP, now)
        self._deadline = now + self.m.grasp_timeout_sec
        return [SendGraspGoal(token=self._new_token(), target_class=self.m.target_class,
                              max_pose_age_sec=self.m.target_pose_max_age_sec)]

    def _start_cancel(self, now, error_code, stage):
        self._pending = _PendingFailure(error_code, codes.TIMEOUT, stage)
        self._enter(State.CANCELING, now)
        self._deadline = now + self.m.cancel_timeout_sec
        return [CancelGoal(token=self._active_token)]

    def _enter(self, state, now):
        if self._state_entered is not None:
            name = self.state.value
            self._durations[name] = self._durations.get(name, 0.0) + now - self._state_entered
        self.state = state
        self._state_entered = now

    def _finish(self, final_state, now, error_code='', detail='', failed_stage=None,
                aborted=False, cancel_no_response=False):
        self._enter(final_state, now)
        self._active_token = None
        record = {
            'run_id': self.run_id,
            'target_class': self.m.target_class,
            'goal': [self.m.goal_x, self.m.goal_y, self.m.goal_yaw],
            'nav_only': self.m.nav_only,
            'final_state': final_state.value,
            'error_code': error_code,
            'detail': detail,
            'failed_stage': failed_stage.value if failed_stage else '',
            'aborted': aborted,
            'cancel_no_response': cancel_no_response,
            'realign_count': self._realign_count,
            'standoffs_used': list(self._standoffs_used),
            'arrival': self._arrival,
            'stage_durations': dict(self._durations),
            'start_time': self._start_time,
            'end_time': now,
        }
        return [WriteRecord(record=record)]
