import math

import numpy as np
import pytest

from visual_servoing.geometry import (decode_depth, depth_at, matrix_to_quat,
                                      optical_rotation_in_base, pixel_to_camera,
                                      quat_to_matrix, range_bearing_error,
                                      transform_point)
from visual_servoing.mock_model import CameraMount, observe

FX = FY = 277.0
CX, CY = 160.0, 120.0


def test_pixel_center_is_optical_axis():
    assert pixel_to_camera(CX, CY, 0.5, FX, FY, CX, CY) == pytest.approx((0.0, 0.0, 0.5))


def test_pixel_right_and_down_are_positive():
    x, y, z = pixel_to_camera(CX + FX, CY + FY / 2, 2.0, FX, FY, CX, CY)
    assert (x, y, z) == pytest.approx((2.0, 1.0, 2.0))


def test_depth_median_ignores_invalid():
    d = np.full((10, 10), 0.4, dtype=np.float32)
    d[5, 5] = np.nan
    d[4, 5] = np.inf
    d[5, 4] = 0.0
    d[6, 6] = 9.0   # 튀는 값 하나는 중앙값이 무시
    assert depth_at(d, 5, 5, 1) == pytest.approx(0.4)


def test_depth_none_when_all_invalid_or_outside():
    d = np.full((10, 10), np.nan, dtype=np.float32)
    assert depth_at(d, 5, 5, 2) is None
    assert depth_at(np.ones((10, 10)), 20, 5, 2) is None


def test_decode_depth_32fc1_and_16uc1():
    a = np.array([[0.5, 1.0], [np.inf, 2.0]], dtype='<f4')
    out = decode_depth('32FC1', a.tobytes(), 2, 2, 8)
    assert out[0, 1] == pytest.approx(1.0) and np.isinf(out[1, 0])

    b = np.array([[500, 0]], dtype='<u2')
    out = decode_depth('16UC1', b.tobytes(), 1, 2, 4)
    assert out[0, 0] == pytest.approx(0.5) and np.isnan(out[0, 1])

    with pytest.raises(ValueError):
        decode_depth('rgb8', b'', 0, 0, 0)


def test_decode_depth_row_padding():
    a = np.array([[1.0, 2.0, 99.0], [3.0, 4.0, 99.0]], dtype='<f4')   # step에 패딩 1칸
    out = decode_depth('32FC1', a.tobytes(), 2, 2, 12)
    assert out.shape == (2, 2) and out[1, 1] == pytest.approx(4.0)


def test_range_bearing_signs():
    e_r, e_b = range_bearing_error(0.5, 0.0, 0.3)
    assert (e_r, e_b) == pytest.approx((0.2, 0.0))
    _, e_b = range_bearing_error(0.5, 0.1, 0.3)
    assert e_b > 0     # 왼쪽 물체 → 양수 (REP 103)
    _, e_b = range_bearing_error(0.5, -0.1, 0.3)
    assert e_b < 0


def test_quaternion_round_trip():
    for pitch in (0.0, 0.3, 0.75, 1.2):
        r = optical_rotation_in_base(pitch)
        assert quat_to_matrix(*matrix_to_quat(r)) == pytest.approx(r, abs=1e-9)


def test_optical_axis_points_forward_when_level():
    r = optical_rotation_in_base(0.0)
    assert r @ np.array([0, 0, 1]) == pytest.approx([1, 0, 0])    # optical z = 앞
    assert r @ np.array([1, 0, 0]) == pytest.approx([0, -1, 0])   # optical x = 오른쪽
    assert r @ np.array([0, 1, 0]) == pytest.approx([0, 0, -1])   # optical y = 아래


@pytest.mark.parametrize('obj', [(0.6, 0.08, -0.04), (0.35, -0.05, -0.04), (0.45, 0.0, 0.02)])
def test_full_chain_recovers_true_error(obj):
    """모의 카메라로 찍은 픽셀+깊이 → 역투영 → TF → 오차가 실제 오차와 같아야 한다."""
    m = CameraMount()
    u, v, z, _, _ = observe(obj, m, 0.05, 0.02)
    p_cam = pixel_to_camera(u, v, z, m.fx, m.fy, m.cx, m.cy)
    q = matrix_to_quat(optical_rotation_in_base(m.pitch))
    x_b, y_b, z_b = transform_point(p_cam, (m.x, 0.0, m.z), q)
    assert (x_b, y_b, z_b) == pytest.approx(obj, abs=1e-9)

    e_r, e_b = range_bearing_error(x_b, y_b, 0.3)
    assert e_r == pytest.approx(math.hypot(obj[0], obj[1]) - 0.3, abs=1e-9)
    assert e_b == pytest.approx(math.atan2(obj[1], obj[0]), abs=1e-9)
