"""Grasp에 넘길 /perception/target_pose 선택 (interfaces.md 3.3, 6장).

- 정렬이 끝난 뒤(since) 찍힌 값만 쓴다 (stamp >= since)
- stamp 기준 max_age 이내여야 한다
- 요청 후 max_age가 지나도 쓸 값이 없으면 미검출 (stale)
"""

import math

import pytest

from mm_task_manager.target_pose import (SEND, STALE, WAIT, GraspPoseRequest, TargetPoseFilter,
                                         is_valid_target_pose, target_pose_reject_reason)


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


@pytest.mark.parametrize('frame_id', [
    'Base_Link', 'BASE_LINK',                       # 대소문자만 다름
    'base_link ', ' base_link', ' base_link ',      # 앞뒤 공백
    'base_link\n', 'base_link\t', '\nbase_link',    # 개행 · 탭
])
def test_frame_id_requires_exact_match(frame_id):
    # ① 정확히 'base_link'만 통과: 대소문자 무시 · strip() 비교로 바꾸면 이 테스트가 깨져야 함
    assert not is_valid_target_pose(frame_id, 0.25, 0.0, 0.02)


# ---- x 상한 · 점프 필터 (C 제안, PR #1 댓글 2026-11-20) ----
# 상한 2.0 m: 희선님 Far 자세 관측 범위 최대값(1차값, 원준님 실측 후 조정). 점프 0.1 m: 직전에 통과한 x 대비

MAX_X, MAX_JUMP = 2.0, 0.1


def reason(x, prev_x=None, frame_id='base_link', y=0.0, z=0.02):
    return target_pose_reject_reason(frame_id, x, y, z, max_x=MAX_X, prev_x=prev_x,
                                     max_jump=MAX_JUMP)


@pytest.mark.parametrize('x', [2.0001, 5.0, 100.0])
def test_x_above_max_is_rejected(x):
    assert reason(x) == 'x_above_max'


def test_x_at_max_passes():
    assert reason(2.0) is None                     # "넘으면" 거름: 2.0 자체는 통과


@pytest.mark.parametrize('x', [0.625, 0.375])     # 직전 0.5에서 +0.125 / -0.125 (양방향)
def test_x_jump_is_rejected_both_directions(x):
    assert reason(x, prev_x=0.5) == 'x_jump'


def test_x_jump_at_threshold_is_rejected():
    # "0.1 m 이상" 튀면 거름 (경계 포함). 이진수로 정확한 값으로 경계 확인
    assert target_pose_reject_reason('base_link', 0.625, 0.0, 0.0, max_x=MAX_X, prev_x=0.5,
                                     max_jump=0.125) == 'x_jump'


@pytest.mark.parametrize('x', [0.45, 0.5, 0.55])
def test_small_change_passes(x):
    assert reason(x, prev_x=0.5) is None


def test_first_message_without_previous_passes():
    assert reason(1.9) is None                     # 직전 값이 없으면 점프 판단 안 함


@pytest.mark.parametrize('kwargs, expected', [
    ({'frame_id': 'map'}, 'frame_id'),
    ({'y': math.nan}, 'non_finite'),
    ({'x': -0.1}, 'x_not_positive'),
    ({'x': 3.0}, 'x_above_max'),
    ({'x': 0.9, 'prev_x': 0.5}, 'x_jump'),
])
def test_reject_reason_names_each_filter(kwargs, expected):
    # 로그에 어떤 필터에 걸렸는지 구분해서 남기기 위함
    args = dict({'x': 0.25}, **kwargs)
    assert reason(**args) == expected


def test_is_valid_target_pose_without_new_limits_is_unchanged():
    # 기존 호출(상한 · 직전 값 없이)은 예전과 같이 동작: 먼 값도 통과
    assert is_valid_target_pose('base_link', 100.0, 0.0, 0.0)


def test_filter_tracks_last_accepted_x():
    f = TargetPoseFilter(max_x=MAX_X, max_jump=MAX_JUMP)
    assert f.check('base_link', 0.5, 0.0, 0.02) is None        # 첫 값: 통과
    assert f.check('base_link', 0.7, 0.0, 0.02) == 'x_jump'    # 0.5 대비 +0.2
    assert f.prev_x == 0.5                                     # 걸러진 값으로 기준을 바꾸지 않음
    assert f.check('base_link', 0.55, 0.0, 0.02) is None       # 0.5 대비 +0.05
    assert f.prev_x == 0.55


def test_filter_rejected_by_other_filters_keeps_previous():
    f = TargetPoseFilter(max_x=MAX_X, max_jump=MAX_JUMP)
    f.check('base_link', 0.5, 0.0, 0.02)
    assert f.check('map', 0.5, 0.0, 0.02) == 'frame_id'
    assert f.check('base_link', 2.5, 0.0, 0.02) == 'x_above_max'
    assert f.prev_x == 0.5


def test_filter_passes_normal_approach():
    # 정렬 중 물체에 다가가는 정상 흐름 (0.60 → 0.25, 한 번에 0.03 m씩): 전부 통과
    f = TargetPoseFilter(max_x=MAX_X, max_jump=MAX_JUMP)
    xs = [0.60 - 0.03 * i for i in range(12)]
    assert [f.check('base_link', x, 0.01, 0.02) for x in xs] == [None] * len(xs)
