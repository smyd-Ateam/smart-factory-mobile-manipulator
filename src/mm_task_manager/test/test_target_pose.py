"""Grasp에 넘길 /perception/target_pose 선택 (interfaces.md 3.3, 6장).

- 정렬이 끝난 뒤(since) 찍힌 값만 쓴다 (stamp >= since)
- stamp 기준 max_age 이내여야 한다
- 요청 후 max_age가 지나도 쓸 값이 없으면 미검출 (stale)
"""

from mm_task_manager.target_pose import SEND, STALE, WAIT, GraspPoseRequest


def req(since=10.0, max_age=1.0):
    return GraspPoseRequest(token=3, since=since, max_age_sec=max_age)


def test_no_pose_yet_waits():
    assert req().check(latest_stamp=None, now=10.2) == WAIT


def test_fresh_pose_after_align_is_sent():
    assert req().check(latest_stamp=10.1, now=10.2) == SEND


def test_pose_captured_before_align_done_is_not_used():
    # 정렬 중(차체가 움직이는 동안) 찍힌 값은 쓰지 않음
    assert req().check(latest_stamp=9.95, now=10.0) == WAIT


def test_pose_at_exactly_since_is_used():
    assert req().check(latest_stamp=10.0, now=10.0) == SEND


def test_old_pose_is_not_sent():
    # since 이후 값이라도 지금 기준 max_age보다 오래되면 안 씀
    assert req(since=10.0).check(latest_stamp=10.1, now=11.2) == STALE


def test_stale_after_max_age_without_pose():
    r = req()
    assert r.check(latest_stamp=None, now=10.99) == WAIT
    assert r.check(latest_stamp=None, now=11.01) == STALE


def test_stale_when_only_pre_align_pose_until_deadline():
    assert req().check(latest_stamp=9.9, now=11.01) == STALE


def test_pose_arriving_just_before_deadline_is_sent():
    assert req().check(latest_stamp=10.95, now=11.0) == SEND
