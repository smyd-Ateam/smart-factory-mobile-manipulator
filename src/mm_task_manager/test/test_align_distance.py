"""노드의 /perception/target_pose → TargetDistance 전달 경로 (task_manager_node._on_target_pose).

전환 기준 0.35 m는 희선님 확정(PR #1, 2026-11-17). 여기서는 미션 값을 그대로 쓰는지만 본다.
"""

import math

import pytest

from mm_task_manager.align_distance import target_distance_event
from mm_task_manager.machine import RequestObservePose, TargetDistance
from mm_task_manager.mission import Mission
from mm_task_manager.states import State

from test_machine import make, to_align, to_grasp, to_navigate

THRESHOLD = Mission('x').pregrasp_switch_distance_m


# ---- 정상: ALIGN 중이면 수평거리로 이벤트를 만든다 ----

def test_event_in_align_uses_horizontal_distance():
    ev = target_distance_event(State.ALIGN, 'base_link', 0.3, 0.4, 0.02, now=12.5)
    assert ev == TargetDistance(now=12.5, distance_m=pytest.approx(0.5))


def test_event_ignores_height():
    # z는 넘기지 않는다 (interfaces.md 5.6: 수평거리)
    ev = target_distance_event(State.ALIGN, 'base_link', 0.25, 0.0, 0.02, now=1.0)
    assert ev.distance_m == 0.25


@pytest.mark.parametrize('state', [s for s in State if s != State.ALIGN])
def test_no_event_outside_align(state):
    assert target_distance_event(state, 'base_link', 0.1, 0.0, 0.02, now=1.0) is None


# ---- 경계 · 이상값: 노드 경로 → 상태머신까지 ----

def _requests(x, y, z=0.02, frame_id='base_link'):
    """ALIGN 중 target_pose 한 개를 넣었을 때 나온 전환 요청 목록 (걸러진 값이면 빈 목록)."""
    m = make(nav_only=False)
    to_align(m)
    ev = target_distance_event(m.state, frame_id, x, y, z, now=11.0)
    if ev is None:
        return []
    return [c for c in m.handle(ev) if isinstance(c, RequestObservePose)]


@pytest.mark.parametrize('x, y, requested', [
    (THRESHOLD, 0.0, False),            # 기준과 같으면 아직 아님
    (THRESHOLD + 1e-6, 0.0, False),
    (THRESHOLD - 1e-6, 0.0, True),
    (0.30, 0.20, False),                # x만 보면 기준 안쪽이지만 수평거리 0.361 m
    (0.05, -0.30, True),                # 옆쪽, 음수 y: 거리는 y 부호와 무관 (x > 0이어야 함)
])
def test_threshold_boundary_through_node_path(x, y, requested):
    assert bool(_requests(x, y)) is requested


@pytest.mark.parametrize('x, y', [
    (math.nan, 0.0),
    (0.1, math.nan),
    (math.inf, 0.0),
    (-math.inf, 0.0),
])
def test_non_finite_pose_never_requests(x, y):
    assert _requests(x, y) == []


def test_requested_distance_is_passed_through():
    (req,) = _requests(0.05, -0.30)
    assert req.distance_m == pytest.approx(math.hypot(0.05, 0.30))


# ---- 안전장치 (C 제안, PR #1 댓글 2026-11-18): 걸러진 값은 이벤트 자체를 안 만든다 ----

def test_zero_and_negative_x_never_request():
    # (0, 0)이나 차체 뒤쪽(x <= 0)은 카메라 시야상 나올 수 없어 무시 (수평거리가 기준 안쪽이어도)
    assert _requests(0.0, 0.0) == []
    assert _requests(-0.20, 0.0) == []
    assert target_distance_event(State.ALIGN, 'base_link', -0.20, 0.0, 0.02, now=1.0) is None


@pytest.mark.parametrize('frame_id', ['map', 'camera_optical_frame', ''])
def test_wrong_frame_never_requests(frame_id):
    assert _requests(0.10, 0.0, frame_id=frame_id) == []


@pytest.mark.parametrize('z', [math.nan, math.inf])
def test_non_finite_z_never_requests(z):
    # z는 거리 계산엔 안 쓰지만, NaN · inf면 메시지 전체를 무시
    assert _requests(0.10, 0.0, z=z) == []


def test_ignored_pose_does_not_block_later_valid_pose():
    # 걸러진 값 뒤에 정상 값이 오면 그대로 전환 요청
    m = make(nav_only=False)
    to_align(m)
    assert target_distance_event(m.state, 'base_link', -0.10, 0.0, 0.02, now=11.0) is None
    ev = target_distance_event(m.state, 'base_link', 0.30, 0.0, 0.02, now=11.1)
    assert [c.distance_m for c in m.handle(ev) if isinstance(c, RequestObservePose)] == [0.30]


# ---- 경고 로그 조건: 노드는 RequestObservePose 명령에서만 "not sent" 경고를 찍는다 ----

def test_warning_command_only_once_per_run_and_only_in_align():
    m = make(nav_only=False)
    to_navigate(m)
    sent = []

    def feed(x, y, now):
        ev = target_distance_event(m.state, 'base_link', x, y, 0.02, now)
        if ev is not None:
            sent.extend(c for c in m.handle(ev) if isinstance(c, RequestObservePose))

    feed(0.10, 0.0, 5.0)                    # NAVIGATE: 이벤트 자체가 안 만들어짐
    assert sent == []

    m2 = make(nav_only=False)
    to_grasp(m2)
    # GRASP
    assert target_distance_event(m2.state, 'base_link', 0.10, 0.0, 0.02, now=21.0) is None

    m = make(nav_only=False)
    to_align(m)
    for i, d in enumerate([0.50, 0.40, 0.30, 0.25, 0.20]):
        feed(d, 0.0, 11.0 + 0.1 * i)
    assert [c.distance_m for c in sent] == [0.30]
