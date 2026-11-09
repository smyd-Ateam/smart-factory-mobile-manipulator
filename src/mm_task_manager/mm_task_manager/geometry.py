"""도착 오차 계산과 TF 신선도 판정. 시각은 모두 초(float)."""

import math


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(angle):
    """각도를 [-pi, pi) 범위로."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def arrival_error(goal, pose):
    """goal, pose = (x, y, yaw). (거리 오차, yaw 오차(pose - goal, wrap)) 반환."""
    dist = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
    yaw_err = wrap_angle(pose[2] - goal[2])
    return dist, yaw_err


def arrival_ok(dist, yaw_err, xy_tol, yaw_tol):
    return dist <= xy_tol and abs(yaw_err) <= yaw_tol


def tf_fresh(stamp, now, max_age_sec):
    """stamp가 now보다 max_age_sec 넘게 오래됐으면 False.

    AMCL은 map→odom 시각을 transform_tolerance만큼 미래로 찍으므로 미래 시각은 신선한 것으로 본다.
    """
    if stamp is None:
        return False
    return now - stamp <= max_age_sec
