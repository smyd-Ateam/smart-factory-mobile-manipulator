"""박스 중심 광선이 닿는 점(깊이로 잰 점) → 물체 중심축 위치 복원."""
import math

import numpy as np
import pytest

from visual_servoing.geometry import matrix_to_quat, optical_rotation_in_base
from visual_servoing.object_model import (Cylinder, axis_from_hit, bbox_center_hit,
                                          parse_object_shapes)
from visual_servoing.sim_model import CAMERA_POSES

K = (554.38, 554.38, 320.0, 240.0)   # 팀 카메라 fx, fy, cx, cy (640×480)
PIN = Cylinder(radius=0.010, height=0.05)
FLOOR_Z = -0.05


def camera(name):
    pose = CAMERA_POSES[name]
    return np.array([pose.x, 0.0, pose.z]), matrix_to_quat(optical_rotation_in_base(pose.pitch))


@pytest.mark.parametrize('name, d, y', [
    ('near', 0.35, 0.0), ('near', 0.40, 0.03), ('near', 0.50, -0.04),
    ('pregrasp', 0.32, 0.0), ('pregrasp', 0.35, 0.01), ('pregrasp', 0.40, -0.02),
    ('test', 0.30, 0.05),
])
def test_axis_recovered_from_bbox_center_hit(name, d, y):
    cam_t, cam_q = camera(name)
    hit = bbox_center_hit((d, y), PIN, FLOOR_Z, cam_t, cam_q, K)
    assert hit is not None
    x, yy = axis_from_hit(hit, PIN, FLOOR_Z, cam_t, cam_q, K)
    assert (x, yy) == pytest.approx((d, y), abs=0.001)


def test_side_view_hit_is_one_radius_in_front():
    # 낮은 각도(test 카메라)에서는 옆면에 닿아 수평으로 반지름만큼 앞이다
    cam_t, cam_q = camera('test')
    hit = bbox_center_hit((0.50, 0.0), PIN, FLOOR_Z, cam_t, cam_q, K)
    assert 0.50 - hit[0] == pytest.approx(PIN.radius, abs=0.0005)


def test_top_view_hit_is_near_axis():
    # pre-grasp 0.32 m는 거의 위에서 봐서 윗면에 닿는다 (반지름보다 훨씬 덜 앞)
    cam_t, cam_q = camera('pregrasp')
    hit = bbox_center_hit((0.32, 0.0), PIN, FLOOR_Z, cam_t, cam_q, K)
    assert abs(0.32 - hit[0]) < 0.003


def test_zero_radius_returns_hit():
    cam_t, cam_q = camera('near')
    point = np.array([0.4, 0.02, -0.03])
    assert axis_from_hit(point, Cylinder(0.0, 0.05), FLOOR_Z, cam_t, cam_q, K) == \
        pytest.approx((0.4, 0.02))


def test_parse_object_shapes():
    assert parse_object_shapes(['pin:0.01:0.05']) == {'pin': Cylinder(0.01, 0.05)}
    for bad in (['pin:0.01'], ['pin:a:0.05'], ['pin:-0.01:0.05'], ['pin:0.01:0'], [':0.01:0.05']):
        with pytest.raises(ValueError):
            parse_object_shapes(bad)


def test_math_sanity_quaternion_unit():
    _, q = camera('pregrasp')
    assert math.isclose(sum(v * v for v in q), 1.0, rel_tol=1e-9)
