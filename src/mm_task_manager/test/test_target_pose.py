"""Grasp에 넘길 /perception/target_pose 선택 (interfaces.md 3.3, 6장).

- 정렬이 끝난 뒤(since) 찍힌 값만 쓴다 (stamp >= since)
- stamp 기준 max_age 이내여야 한다
- 요청 후 max_age가 지나도 쓸 값이 없으면 미검출 (stale)
"""

import math

import pytest

from mm_task_manager.target_pose import SEND, STALE, WAIT, GraspPoseRequest, is_valid_target_pose


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


# ---- 받는 쪽 안전장치 (C 제안, PR #1 댓글 2026-11-18) ----
# 걸러진 메시지는 아예 무시: Grasp 목표 후보(latest)에도, Near→Pre-grasp 거리 판단에도 안 씀

def test_valid_pose_passes():
    assert is_valid_target_pose('base_link', 0.25, -0.05, 0.02)


def test_tiny_positive_x_passes():
    assert is_valid_target_pose('base_link', 1e-9, 0.0, 0.0)


@pytest.mark.parametrize('frame_id', ['map', 'odom', 'camera_optical_frame', '', '/base_link'])
def test_frame_other_than_base_link_is_rejected(frame_id):
    # ① base_link가 아니면 무시 (변환하지 않음, 문자열 그대로 비교)
    assert not is_valid_target_pose(frame_id, 0.25, 0.0, 0.02)


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize('axis', [0, 1, 2])
def test_non_finite_xyz_is_rejected(axis, bad):
    # ② x, y, z 중 하나라도 NaN · inf면 무시 (z도 본다)
    xyz = [0.25, 0.0, 0.02]
    xyz[axis] = bad
    assert not is_valid_target_pose('base_link', *xyz)


@pytest.mark.parametrize('x', [0.0, -0.0, -1e-9, -0.20])
def test_x_not_in_front_is_rejected(x):
    # ③ x <= 0 무시: 카메라가 항상 앞을 봐서 차체 뒤쪽 물체는 나올 수 없음
    assert not is_valid_target_pose('base_link', x, 0.0, 0.02)
