"""같은 클래스 물체가 여러 개일 때 목표 하나를 따라가는 선택 규칙."""
import math

import numpy as np
import pytest

from visual_servoing.geometry import matrix_to_quat, optical_rotation_in_base, quat_to_matrix
from visual_servoing.sim_model import CAMERA_POSES
from visual_servoing.tracking import (
    Candidate, base_bearing, base_plane_point, expected_goal_is_valid, pick_near_expected,
    pick_target)

K = (554.38, 554.38, 320.0, 240.0)


def test_first_frame_picks_smallest_base_bearing():
    cands = [Candidate(100.0, 200.0, 0.95), Candidate(330.0, 150.0, 0.6),
             Candidate(500.0, 300.0, 0.9)]
    assert pick_target(cands, None, gate_px=100.0, first_cost=[0.4, 0.05, 0.2]) == 1


def test_follows_last_position_not_highest_score():
    cands = [Candidate(100.0, 200.0, 0.95), Candidate(300.0, 260.0, 0.6)]
    assert pick_target(cands, (310.0, 250.0), gate_px=100.0, first_cost=[0.0, 1.0]) == 1


def test_returns_none_when_tracked_object_disappeared():
    # 따라가던 물체가 안 보이고 다른 물체만 멀리 있으면 갈아타지 않는다
    cands = [Candidate(100.0, 200.0, 0.95)]
    assert pick_target(cands, (320.0, 250.0), gate_px=100.0, first_cost=[0.0]) is None


def test_empty_candidates():
    assert pick_target([], None, gate_px=100.0, first_cost=[]) is None


@pytest.mark.parametrize('bearing', [0.0, 0.2, -0.3])
def test_base_bearing_of_pixel(bearing):
    # near 카메라에서 base_link 기준 (0.45 m, bearing) 물체 중간 높이를 화면에 투영한 뒤 되돌린다
    pose = CAMERA_POSES['near']
    q = matrix_to_quat(optical_rotation_in_base(pose.pitch))
    t = (pose.x, 0.0, pose.z)
    p = (0.45 * math.cos(bearing), 0.45 * math.sin(bearing), -0.025)
    o = quat_to_matrix(*q).T @ (np.array(p) - np.array(t))
    u, v = K[0] * o[0] / o[2] + K[2], K[1] * o[1] / o[2] + K[3]
    assert base_bearing(u, v, K, t, q, -0.025) == pytest.approx(bearing, abs=1e-6)


# ---------- 예상 위치로 첫 목표 고르기 (설계 문서 8장) ----------


def project_near(p):
    """near 카메라에서 base_link 점 p를 화면 픽셀 (u, v)로 투영한다."""
    pose = CAMERA_POSES['near']
    q = matrix_to_quat(optical_rotation_in_base(pose.pitch))
    t = (pose.x, 0.0, pose.z)
    o = quat_to_matrix(*q).T @ (np.array(p) - np.array(t))
    return (K[0] * o[0] / o[2] + K[2], K[1] * o[1] / o[2] + K[3]), t, q


def test_expected_picks_closest_candidate_inside_radius():
    # 부품 간격 0.12 m, 예상 위치는 가운데 pin 근처
    points = [(0.45, -0.12), (0.46, 0.01), (0.45, 0.12)]
    assert pick_near_expected(points, (0.45, 0.0), radius_m=0.07, ambiguity_m=0.02) == 1


def test_expected_returns_none_when_nothing_inside_radius():
    points = [(0.45, -0.12), (0.45, 0.12)]
    assert pick_near_expected(points, (0.45, 0.0), radius_m=0.07, ambiguity_m=0.02) is None


def test_expected_returns_none_when_two_candidates_are_ambiguous():
    # 두 후보가 모두 반경 안이고 예상 위치까지 거리 차이가 ambiguity_m보다 작으면 고르지 않는다
    points = [(0.45, -0.055), (0.45, 0.06)]
    assert pick_near_expected(points, (0.45, 0.0), radius_m=0.08, ambiguity_m=0.02) is None


def test_expected_second_candidate_outside_radius_is_not_ambiguous():
    points = [(0.45, 0.065), (0.45, -0.075)]
    assert pick_near_expected(points, (0.45, 0.0), radius_m=0.07, ambiguity_m=0.02) == 0


def test_expected_skips_candidates_without_position():
    # 광선이 바닥 높이와 만나지 않아 위치를 못 구한 후보(None)는 건너뛴다
    points = [None, (0.45, 0.01)]
    assert pick_near_expected(points, (0.45, 0.0), radius_m=0.07, ambiguity_m=0.02) == 1


def test_expected_empty_candidates():
    assert pick_near_expected([], (0.45, 0.0), radius_m=0.07, ambiguity_m=0.02) is None


@pytest.mark.parametrize('x, y', [(0.45, 0.0), (0.40, 0.12), (0.50, -0.10)])
def test_base_plane_point_recovers_xy(x, y):
    (u, v), t, q = project_near((x, y, -0.025))
    assert base_plane_point(u, v, K, t, q, -0.025) == pytest.approx((x, y), abs=1e-6)


@pytest.mark.parametrize('frame_id, radius, ok', [
    ('', 0.0, True),        # 예상 위치 없음: 지금과 똑같이 동작
    ('', -1.0, True),       # frame_id가 비면 반경은 보지 않는다
    ('map', 0.07, True),
    ('map', 0.0, False),    # 예상 위치는 있는데 반경이 없으면 거부
    ('map', -0.07, False),
])
def test_expected_goal_is_valid(frame_id, radius, ok):
    assert expected_goal_is_valid(frame_id, radius) is ok


def test_base_plane_point_none_when_ray_points_up():
    # 수평으로 놓인 카메라의 화면 위쪽 픽셀은 아래 평면과 만나지 않는다
    q = matrix_to_quat(optical_rotation_in_base(0.0))
    assert base_plane_point(320.0, 0.0, K, (0.0, 0.0, 0.2), q, -0.025) is None
