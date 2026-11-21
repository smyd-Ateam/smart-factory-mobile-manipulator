"""같은 클래스 물체가 여러 개일 때 목표 하나를 따라가는 선택 규칙."""
import math

import numpy as np
import pytest

from visual_servoing.geometry import matrix_to_quat, optical_rotation_in_base, quat_to_matrix
from visual_servoing.sim_model import CAMERA_POSES
from visual_servoing.tracking import Candidate, base_bearing, pick_target

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
