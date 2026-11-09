import math

import pytest

from mm_task_manager import codes
from mm_task_manager.machine import (NAV_ABORTED, NAV_CANCELED, NAV_REJECTED, NAV_SUCCEEDED,
                                     AlignResult, CancelDone, CancelGoal, ClockStopped,
                                     GraspResult, NavResult, PoseSample, PublishTargetClass,
                                     ReadyStatus, SendAlignGoal, SendGraspGoal, SendNavGoal,
                                     Start, TaskMachine, Tick, WriteRecord)
from mm_task_manager.mission import Mission
from mm_task_manager.states import State

GOAL = (1.0, 2.0, 0.5)


def make(**overrides):
    params = dict(goal_x=GOAL[0], goal_y=GOAL[1], goal_yaw=GOAL[2], target_class='block',
                  ready_timeout_sec=30.0, tf_max_age_sec=0.5, nav_timeout_sec=90.0,
                  cancel_timeout_sec=5.0, settle_sec=0.5, align_timeout_sec=30.0,
                  align_margin_sec=5.0, grasp_timeout_sec=60.0)
    params.update(overrides)
    return TaskMachine(Mission(**params), run_id='run-1')


def of_type(cmds, cls):
    return [c for c in cmds if isinstance(c, cls)]


def ready(now, **overrides):
    kw = dict(localization_active=True, navigation_active=True, nav_server_ready=True,
              map_odom_stamp=now)
    kw.update(overrides)
    return ReadyStatus(now=now, **kw)


def record_of(cmds):
    records = of_type(cmds, WriteRecord)
    assert len(records) == 1
    return records[0].record


# ---- 상태까지 진행시키는 도우미 ----

def to_navigate(m, now=0.0):
    m.handle(Start(now=now))
    cmds = m.handle(ready(now + 1.0))
    assert m.state == State.NAVIGATE
    return of_type(cmds, SendNavGoal)[0].token


def to_verify(m):
    token = to_navigate(m)
    m.handle(NavResult(now=10.0, token=token, status=NAV_SUCCEEDED))
    assert m.state == State.VERIFY_ARRIVAL


def to_align(m):
    to_verify(m)
    cmds = m.handle(PoseSample(now=10.5, pose=GOAL, map_odom_stamp=10.5))
    assert m.state == State.ALIGN
    return of_type(cmds, SendAlignGoal)[0]


def to_grasp(m):
    goal = to_align(m)
    cmds = m.handle(AlignResult(now=20.0, token=goal.token, success=True))
    assert m.state == State.GRASP
    return of_type(cmds, SendGraspGoal)[0]


# ---- 시작 ----

def test_initial_state_idle():
    assert make().state == State.IDLE


def test_events_before_start_are_ignored():
    m = make()
    assert m.handle(ready(1.0)) == []
    assert m.handle(Tick(now=100.0)) == []
    assert m.state == State.IDLE


def test_start_publishes_target_class():
    m = make()
    cmds = m.handle(Start(now=0.0))
    assert m.state == State.WAIT_READY
    assert of_type(cmds, PublishTargetClass) == [PublishTargetClass(target_class='block')]


def test_ready_sends_nav_goal():
    m = make()
    m.handle(Start(now=0.0))
    cmds = m.handle(ready(1.0))
    assert m.state == State.NAVIGATE
    goal = of_type(cmds, SendNavGoal)[0]
    assert (goal.x, goal.y, goal.yaw) == GOAL


@pytest.mark.parametrize('overrides, detail', [
    (dict(localization_active=False), codes.LIFECYCLE_INACTIVE),
    (dict(navigation_active=None), codes.LIFECYCLE_INACTIVE),      # 응답 아직 없음
    (dict(nav_server_ready=False), codes.NAV_SERVER_UNAVAILABLE),
    (dict(map_odom_stamp=None), codes.TF_STALE),
    (dict(map_odom_stamp=0.0), codes.TF_STALE),                   # now=1.0, 1 s 지남
])
def test_not_ready_then_timeout(overrides, detail):
    m = make()
    m.handle(Start(now=0.0))
    assert m.handle(ready(1.0, **overrides)) == []
    assert m.state == State.WAIT_READY
    assert m.handle(Tick(now=30.0)) == []                          # 경계값은 아직 아님
    cmds = m.handle(Tick(now=30.1))
    assert m.state == State.FAILED
    rec = record_of(cmds)
    assert (rec['error_code'], rec['detail'], rec['failed_stage']) == \
        (codes.NAV_NOT_READY, detail, 'WAIT_READY')
    assert of_type(cmds, SendNavGoal) == []


def test_ready_timeout_without_any_status():
    m = make()
    m.handle(Start(now=0.0))
    rec = record_of(m.handle(Tick(now=31.0)))
    assert (rec['error_code'], rec['detail']) == (codes.NAV_NOT_READY, codes.NO_READY_STATUS)


# ---- 이동 ----

@pytest.mark.parametrize('status, detail', [
    (NAV_ABORTED, codes.ABORTED),
    (NAV_REJECTED, codes.REJECTED),
    (NAV_CANCELED, codes.CANCELED),
])
def test_nav_failure_no_retry(status, detail):
    m = make()
    token = to_navigate(m)
    cmds = m.handle(NavResult(now=5.0, token=token, status=status))
    assert m.state == State.FAILED
    rec = record_of(cmds)
    assert (rec['error_code'], rec['detail'], rec['failed_stage']) == \
        (codes.NAV_FAILED, detail, 'NAVIGATE')
    assert of_type(cmds, SendNavGoal) == []                        # 재시도 0회


def test_result_with_old_token_is_ignored():
    m = make()
    token = to_navigate(m)
    assert m.handle(NavResult(now=5.0, token=token + 100, status=NAV_ABORTED)) == []
    assert m.state == State.NAVIGATE


def test_nav_timeout_cancels_then_fails():
    m = make()
    token = to_navigate(m)                       # 목표 보낸 시각 1.0 → 마감 91.0
    assert m.handle(Tick(now=91.0)) == []
    cmds = m.handle(Tick(now=91.1))
    assert m.state == State.CANCELING
    assert of_type(cmds, CancelGoal) == [CancelGoal(token=token)]
    cmds = m.handle(CancelDone(now=92.0, token=token))
    assert m.state == State.FAILED
    rec = record_of(cmds)
    assert (rec['error_code'], rec['detail'], rec['failed_stage']) == \
        (codes.NAV_FAILED, codes.TIMEOUT, 'NAVIGATE')
    assert rec['aborted'] is False
    assert rec['cancel_no_response'] is False


def test_cancel_no_response_aborts_run():
    m = make()
    to_navigate(m)
    m.handle(Tick(now=91.1))
    assert m.handle(Tick(now=96.1)) == []        # 취소 마감 96.1, 경계값은 아직 아님
    cmds = m.handle(Tick(now=96.2))
    assert m.state == State.FAILED
    rec = record_of(cmds)
    assert (rec['error_code'], rec['detail']) == (codes.NAV_FAILED, codes.TIMEOUT)
    assert rec['aborted'] is True
    assert rec['cancel_no_response'] is True


def test_result_of_canceled_goal_also_ends_canceling():
    m = make()
    token = to_navigate(m)
    m.handle(Tick(now=91.1))
    cmds = m.handle(NavResult(now=92.0, token=token, status=NAV_CANCELED))
    assert m.state == State.FAILED
    rec = record_of(cmds)
    assert (rec['detail'], rec['aborted']) == (codes.TIMEOUT, False)


def test_late_result_after_failure_is_ignored():
    m = make()
    token = to_navigate(m)
    m.handle(Tick(now=91.1))
    m.handle(Tick(now=96.2))                     # 중단
    assert m.handle(NavResult(now=100.0, token=token, status=NAV_SUCCEEDED)) == []
    assert m.state == State.FAILED


# ---- 도착 확인 ----

def test_pose_before_settle_is_ignored():
    m = make()
    to_verify(m)                                 # SUCCEEDED 시각 10.0 → 10.5부터 확인
    assert m.handle(PoseSample(now=10.4, pose=(5.0, 5.0, 0.0), map_odom_stamp=10.4)) == []
    assert m.state == State.VERIFY_ARRIVAL


def test_arrival_ok_nav_only_succeeds():
    m = make(nav_only=True)
    to_verify(m)
    pose = (GOAL[0] + 0.1, GOAL[1], GOAL[2] - 0.3)
    cmds = m.handle(PoseSample(now=10.5, pose=pose, map_odom_stamp=10.5))
    assert m.state == State.SUCCEEDED
    rec = record_of(cmds)
    assert rec['error_code'] == ''
    assert rec['arrival']['dist'] == pytest.approx(0.1)
    assert rec['arrival']['yaw_err'] == pytest.approx(-0.3)
    assert rec['arrival']['pose'] == pytest.approx(list(pose))   # Gazebo 실제 위치와 비교용


def test_fake_success_caught_by_arrival_check():
    # S2: Nav2는 SUCCEEDED, TF는 신선한데 위치가 목표 밖
    m = make()
    to_verify(m)
    cmds = m.handle(PoseSample(now=10.5, pose=(0.0, 0.0, 0.0), map_odom_stamp=10.5))
    assert m.state == State.FAILED
    rec = record_of(cmds)
    assert (rec['error_code'], rec['detail'], rec['failed_stage']) == \
        (codes.NAV_FAILED, codes.ARRIVAL_CHECK, 'VERIFY_ARRIVAL')
    assert rec['arrival']['dist'] == pytest.approx(math.hypot(1.0, 2.0))


def test_yaw_out_of_range_fails_arrival():
    m = make()
    to_verify(m)
    cmds = m.handle(PoseSample(now=10.5, pose=(GOAL[0], GOAL[1], GOAL[2] + 0.4),
                               map_odom_stamp=10.5))
    assert record_of(cmds)['detail'] == codes.ARRIVAL_CHECK


@pytest.mark.parametrize('pose, stamp', [
    (GOAL, 9.9),         # map→odom 0.6 s 지남
    (GOAL, None),
    (None, 10.5),        # map→base_link 조회 실패
])
def test_stale_tf_fails(pose, stamp):
    m = make()
    to_verify(m)
    cmds = m.handle(PoseSample(now=10.5, pose=pose, map_odom_stamp=stamp))
    assert m.state == State.FAILED
    assert (record_of(cmds)['error_code'], record_of(cmds)['detail']) == \
        (codes.NAV_FAILED, codes.TF_STALE)


def test_future_map_odom_stamp_is_fresh():
    m = make()
    to_verify(m)
    m.handle(PoseSample(now=10.5, pose=GOAL, map_odom_stamp=11.5))
    assert m.state == State.SUCCEEDED


# ---- 정렬 ----

def test_arrival_ok_sends_align_with_first_standoff():
    m = make(nav_only=False)
    goal = to_align(m)
    assert (goal.target_class, goal.standoff_m, goal.timeout_sec) == ('block', 0.25, 30.0)


@pytest.mark.parametrize('code, expected', [
    (codes.OBJECT_NOT_FOUND, codes.OBJECT_NOT_FOUND),
    (codes.ALIGN_TIMEOUT, codes.ALIGN_TIMEOUT),
    ('', codes.UNKNOWN),
])
def test_align_failure(code, expected):
    m = make(nav_only=False)
    goal = to_align(m)
    cmds = m.handle(AlignResult(now=15.0, token=goal.token, success=False, error_code=code))
    assert m.state == State.FAILED
    rec = record_of(cmds)
    assert (rec['error_code'], rec['failed_stage']) == (expected, 'ALIGN')


def test_align_timeout_uses_parameter_code():
    m = make(nav_only=False)
    goal = to_align(m)                           # 시각 10.5 + 30 + 5 = 45.5
    assert m.handle(Tick(now=45.5)) == []
    cmds = m.handle(Tick(now=45.6))
    assert m.state == State.CANCELING
    assert of_type(cmds, CancelGoal) == [CancelGoal(token=goal.token)]
    rec = record_of(m.handle(CancelDone(now=46.0, token=goal.token)))
    assert (rec['error_code'], rec['detail'], rec['failed_stage']) == \
        (codes.ALIGN_TIMEOUT, codes.TIMEOUT, 'ALIGN')


# ---- 파지 · 재정렬 ----

def test_grasp_success():
    m = make(nav_only=False)
    goal = to_grasp(m)
    assert (goal.target_class, goal.max_pose_age_sec) == ('block', 1.0)
    cmds = m.handle(GraspResult(now=30.0, token=goal.token, success=True))
    assert m.state == State.SUCCEEDED
    rec = record_of(cmds)
    assert rec['realign_count'] == 0
    assert rec['standoffs_used'] == [0.25]


def test_realign_twice_then_success():
    m = make(nav_only=False)
    grasp = to_grasp(m)
    standoffs = []
    now = 30.0
    for _ in range(2):
        cmds = m.handle(GraspResult(now=now, token=grasp.token, success=False,
                                    error_code=codes.IK_FAILED))
        assert m.state == State.ALIGN
        align = of_type(cmds, SendAlignGoal)[0]
        standoffs.append(align.standoff_m)
        cmds = m.handle(AlignResult(now=now + 5, token=align.token, success=True))
        grasp = of_type(cmds, SendGraspGoal)[0]
        now += 10
    assert standoffs == [0.22, 0.28]
    rec = record_of(m.handle(GraspResult(now=now, token=grasp.token, success=True)))
    assert m.state == State.SUCCEEDED
    assert rec['realign_count'] == 2
    assert rec['standoffs_used'] == [0.25, 0.22, 0.28]


def test_ik_failed_three_times_fails():
    m = make(nav_only=False)
    grasp = to_grasp(m)
    now = 30.0
    for _ in range(2):
        cmds = m.handle(GraspResult(now=now, token=grasp.token, success=False,
                                    error_code=codes.IK_FAILED))
        align = of_type(cmds, SendAlignGoal)[0]
        cmds = m.handle(AlignResult(now=now + 5, token=align.token, success=True))
        grasp = of_type(cmds, SendGraspGoal)[0]
        now += 10
    cmds = m.handle(GraspResult(now=now, token=grasp.token, success=False,
                                error_code=codes.IK_FAILED))
    assert m.state == State.FAILED
    rec = record_of(cmds)
    assert (rec['error_code'], rec['failed_stage'], rec['realign_count']) == \
        (codes.IK_FAILED, 'GRASP', 2)
    assert of_type(cmds, SendAlignGoal) == []


@pytest.mark.parametrize('code', [codes.GRASP_MISS, codes.GRASP_SLIP])
def test_other_grasp_failure_no_realign(code):
    m = make(nav_only=False)
    grasp = to_grasp(m)
    cmds = m.handle(GraspResult(now=30.0, token=grasp.token, success=False, error_code=code))
    assert m.state == State.FAILED
    assert record_of(cmds)['error_code'] == code


def test_grasp_failure_detail_is_recorded():
    m = make(nav_only=False)
    grasp = to_grasp(m)
    cmds = m.handle(GraspResult(now=30.0, token=grasp.token, success=False,
                                error_code=codes.UNKNOWN, detail='target_pose_stale'))
    assert record_of(cmds)['detail'] == 'target_pose_stale'


def test_grasp_timeout_uses_parameter_code():
    m = make(nav_only=False)
    grasp = to_grasp(m)                          # 시각 20.0 + 60 = 80.0
    m.handle(Tick(now=80.1))
    assert m.state == State.CANCELING
    rec = record_of(m.handle(CancelDone(now=81.0, token=grasp.token)))
    assert (rec['error_code'], rec['detail'], rec['failed_stage']) == \
        (codes.UNKNOWN, codes.TIMEOUT, 'GRASP')


# ---- /clock 정지 ----

def test_clock_stopped_cancels_active_goal_and_aborts():
    m = make()
    token = to_navigate(m)
    cmds = m.handle(ClockStopped(now=5.0))
    assert m.state == State.FAILED
    assert of_type(cmds, CancelGoal) == [CancelGoal(token=token)]
    rec = record_of(cmds)
    assert (rec['error_code'], rec['detail'], rec['failed_stage']) == \
        (codes.UNKNOWN, codes.CLOCK_STOPPED, 'NAVIGATE')
    assert rec['aborted'] is True


def test_clock_stopped_while_waiting_ready():
    m = make()
    m.handle(Start(now=0.0))
    cmds = m.handle(ClockStopped(now=1.0))
    assert of_type(cmds, CancelGoal) == []
    assert record_of(cmds)['failed_stage'] == 'WAIT_READY'


def test_clock_stopped_while_canceling_keeps_stage_and_no_second_cancel():
    m = make()
    to_navigate(m)
    m.handle(Tick(now=91.1))                     # 시간 초과 → CANCELING
    cmds = m.handle(ClockStopped(now=92.0))
    assert of_type(cmds, CancelGoal) == []
    rec = record_of(cmds)
    assert (rec['detail'], rec['failed_stage'], rec['aborted']) == \
        (codes.CLOCK_STOPPED, 'NAVIGATE', True)


def test_clock_stopped_ignored_in_idle_and_terminal():
    m = make()
    assert m.handle(ClockStopped(now=0.0)) == []
    to_verify(m)
    m.handle(PoseSample(now=10.5, pose=GOAL, map_odom_stamp=10.5))
    assert m.handle(ClockStopped(now=20.0)) == []


# ---- 결과 기록 ----

def test_record_contents():
    m = make()
    to_verify(m)
    rec = record_of(m.handle(PoseSample(now=10.5, pose=GOAL, map_odom_stamp=10.5)))
    assert rec['run_id'] == 'run-1'
    assert rec['target_class'] == 'block'
    assert rec['goal'] == list(GOAL)
    assert rec['nav_only'] is True
    assert rec['final_state'] == 'SUCCEEDED'
    assert rec['failed_stage'] == ''
    assert (rec['start_time'], rec['end_time']) == (0.0, 10.5)
    assert rec['stage_durations'] == pytest.approx(
        {'WAIT_READY': 1.0, 'NAVIGATE': 9.0, 'VERIFY_ARRIVAL': 0.5})


def test_stage_durations_accumulate_over_realign():
    m = make(nav_only=False)
    grasp = to_grasp(m)                          # ALIGN 10.5~20.0
    cmds = m.handle(GraspResult(now=25.0, token=grasp.token, success=False,
                                error_code=codes.IK_FAILED))
    align = of_type(cmds, SendAlignGoal)[0]
    cmds = m.handle(AlignResult(now=28.0, token=align.token, success=True))
    grasp = of_type(cmds, SendGraspGoal)[0]
    rec = record_of(m.handle(GraspResult(now=30.0, token=grasp.token, success=True)))
    assert rec['stage_durations']['ALIGN'] == pytest.approx(9.5 + 3.0)
    assert rec['stage_durations']['GRASP'] == pytest.approx(5.0 + 2.0)


def test_record_written_once_and_terminal_ignores_events():
    m = make()
    to_navigate(m)
    m.handle(Tick(now=91.1))
    m.handle(Tick(now=96.2))
    assert m.handle(Tick(now=200.0)) == []
    assert m.handle(Start(now=201.0)) == []


def test_tokens_are_unique_per_goal():
    m = make(nav_only=False)
    nav_token = to_navigate(m)
    m.handle(NavResult(now=10.0, token=nav_token, status=NAV_SUCCEEDED))
    cmds = m.handle(PoseSample(now=10.5, pose=GOAL, map_odom_stamp=10.5))
    align_token = of_type(cmds, SendAlignGoal)[0].token
    cmds = m.handle(AlignResult(now=20.0, token=align_token, success=True))
    grasp_token = of_type(cmds, SendGraspGoal)[0].token
    assert len({nav_token, align_token, grasp_token}) == 3
