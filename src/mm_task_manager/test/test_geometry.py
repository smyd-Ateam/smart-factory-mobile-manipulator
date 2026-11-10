import math

import pytest

from mm_task_manager.geometry import (approach_goal, arrival_error, arrival_ok, tf_fresh,
                                      wrap_angle, yaw_from_quaternion)


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


# ---- approach_goal: 물체에서 distance만큼 떨어진, 물체를 바라보는 목표 ----

def _check_goal_faces_object(goal, obj, distance):
    gx, gy, gyaw = goal
    assert math.hypot(obj[0] - gx, obj[1] - gy) == pytest.approx(distance)
    assert wrap_angle(math.atan2(obj[1] - gy, obj[0] - gx) - gyaw) == pytest.approx(0.0, abs=1e-9)


def test_approach_straight_ahead():
    goal = approach_goal((1.0, 0.0), (0.0, 0.0, 0.0), 0.35)
    assert goal == pytest.approx((0.65, 0.0, 0.0))


@pytest.mark.parametrize('obj, robot', [
    ((2.0, 1.0), (1.0, 0.0, 0.0)),          # 대각선
    ((0.0, -0.5), (1.0, 0.0, 0.0)),         # 뒤쪽 (yaw가 -pi 근처)
    ((-1.0, 0.001), (0.0, 0.0, 0.0)),       # 거의 정반대 (yaw가 +pi 근처)
    ((1.2, 0.0), (1.0, 0.0, 0.0)),          # 차체가 distance보다 가까움 → 뒤로 물러남
    ((-2.5, 2.0), (1.5, -1.0, 2.0)),
])
def test_approach_distance_and_heading(obj, robot):
    goal = approach_goal(obj, robot, 0.35)
    _check_goal_faces_object(goal, obj, 0.35)
    # 출발 위치 → 물체 방향의 선 위에 있음
    ux, uy = obj[0] - robot[0], obj[1] - robot[1]
    assert (goal[0] - robot[0]) * uy - (goal[1] - robot[1]) * ux == pytest.approx(0.0, abs=1e-9)


def test_approach_behind_object_yaw():
    goal = approach_goal((0.0, -0.5), (1.0, 0.0, 0.0), 0.35)
    assert goal[2] == pytest.approx(math.atan2(-0.5, -1.0))     # 약 -2.678
    assert goal[0] == pytest.approx(0.0 + 0.35 * 1.0 / math.hypot(1.0, 0.5))
    assert goal[1] == pytest.approx(-0.5 + 0.35 * 0.5 / math.hypot(1.0, 0.5))


def test_approach_close_object_backs_up():
    goal = approach_goal((1.2, 0.0), (1.0, 0.0, 0.0), 0.35)
    assert goal == pytest.approx((0.85, 0.0, 0.0))


def test_approach_ignores_robot_yaw():
    a = approach_goal((2.0, 1.0), (1.0, 0.0, 0.0), 0.35)
    b = approach_goal((2.0, 1.0), (1.0, 0.0, 2.5), 0.35)
    assert a == pytest.approx(b)


def test_approach_robot_on_object_uses_robot_heading():
    # 방향을 정할 수 없으면 차체가 바라보는 방향으로 접근
    goal = approach_goal((1.0, 1.0), (1.0, 1.0, math.pi / 2), 0.35)
    assert goal == pytest.approx((1.0, 0.65, math.pi / 2))


def test_approach_yaw_overrides_direction():
    # 확장용: 접근 방향을 직접 주면 출발 위치와 상관없이 그 방향
    goal = approach_goal((2.0, 1.0), (1.0, 0.0, 0.0), 0.35, approach_yaw=math.pi)
    assert goal[:2] == pytest.approx((2.35, 1.0))
    assert wrap_angle(goal[2] - math.pi) == pytest.approx(0.0)   # pi와 -pi는 같은 방향
    _check_goal_faces_object(goal, (2.0, 1.0), 0.35)
