import math

import pytest

from mm_task_manager.geometry import (arrival_error, arrival_ok, tf_fresh, wrap_angle,
                                      yaw_from_quaternion)


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


@pytest.mark.parametrize('yaw', [0.0, 0.5, -1.2, math.pi - 0.01, -math.pi + 0.01])
def test_yaw_from_quaternion(yaw):
    assert yaw_from_quaternion(*quat_from_yaw(yaw)) == pytest.approx(yaw)


@pytest.mark.parametrize('angle, expected', [
    (0.0, 0.0),
    (math.pi / 2, math.pi / 2),
    (3 * math.pi / 2, -math.pi / 2),
    (-3 * math.pi / 2, math.pi / 2),
    (2 * math.pi + 0.1, 0.1),
])
def test_wrap_angle(angle, expected):
    assert wrap_angle(angle) == pytest.approx(expected)


def test_arrival_error_distance_and_yaw():
    dist, yaw_err = arrival_error((1.0, 2.0, 0.0), (1.3, 2.4, 0.2))
    assert dist == pytest.approx(0.5)
    assert yaw_err == pytest.approx(0.2)


def test_arrival_error_yaw_wraps_across_pi():
    # 목표 179°, 현재 -179° → 실제 차이는 2°
    _, yaw_err = arrival_error((0.0, 0.0, math.radians(179)), (0.0, 0.0, math.radians(-179)))
    assert yaw_err == pytest.approx(math.radians(2))


@pytest.mark.parametrize('dist, yaw_err, ok', [
    (0.0, 0.0, True),
    (0.15, 0.35, True),      # 경계값 포함
    (0.151, 0.0, False),
    (0.0, 0.351, False),
    (0.0, -0.351, False),    # yaw는 절댓값으로 비교
    (0.1, -0.3, True),
])
def test_arrival_ok(dist, yaw_err, ok):
    assert arrival_ok(dist, yaw_err, xy_tol=0.15, yaw_tol=0.35) is ok


@pytest.mark.parametrize('stamp, now, fresh', [
    (10.0, 10.0, True),
    (9.6, 10.0, True),
    (9.5, 10.0, True),       # 경계값 포함
    (9.4, 10.0, False),
    (11.0, 10.0, True),      # AMCL은 transform_tolerance만큼 미래 시각을 찍음
])
def test_tf_fresh(stamp, now, fresh):
    assert tf_fresh(stamp, now, max_age_sec=0.5) is fresh


def test_tf_fresh_none_stamp_is_not_fresh():
    assert tf_fresh(None, 10.0, max_age_sec=0.5) is False
