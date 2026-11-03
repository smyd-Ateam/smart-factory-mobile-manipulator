import math

import pytest

from visual_servoing.control import ControlParams, compute_cmd, within_tolerance
from visual_servoing.geometry import (matrix_to_quat, optical_rotation_in_base,
                                      pixel_to_camera, range_bearing_error,
                                      transform_point)
from visual_servoing.mock_model import (CameraMount, object_in_base, observe,
                                        step_unicycle, true_error)

P = ControlParams()


def test_stop_inside_tolerance():
    assert compute_cmd(0.008, 0.02, P) == (0.0, 0.0)
    assert within_tolerance(-0.02, -0.05, P)


def test_keep_moving_near_tolerance_edge():
    # 허용 범위 안이지만 stop_ratio 밖이면 계속 줄인다
    vx, _ = compute_cmd(0.018, 0.0, P)
    assert vx > 0


def test_speed_limits():
    vx, wz = compute_cmd(1.0, 0.1, P)
    assert vx == pytest.approx(P.max_v)
    assert wz == pytest.approx(0.1)
    vx, wz = compute_cmd(-1.0, 0.0, P)
    assert vx == pytest.approx(-P.max_v)
    _, wz = compute_cmd(0.0, -1.0, P)
    assert wz == pytest.approx(-P.max_w)


def test_turn_only_when_bearing_large():
    vx, wz = compute_cmd(0.2, 0.3, P)
    assert vx == 0.0 and wz > 0


def test_turn_direction_matches_bearing():
    assert compute_cmd(0.0, 0.1, P)[1] > 0
    assert compute_cmd(0.0, -0.1, P)[1] < 0


@pytest.mark.parametrize('start', [
    (0.0, 0.0, 0.25),     # 왼쪽으로 틀어짐, 멀다
    (0.0, 0.0, -0.10),    # 오른쪽으로 틀어짐
    (0.25, 0.08, 0.20),   # 너무 가깝다 → 후진
])
def test_closed_loop_converges(start):
    """모의 카메라 + P제어 폐루프 (지연 0). 실제 위치 기준으로 허용 범위 안에 들어와야 한다."""
    m = CameraMount()
    q = matrix_to_quat(optical_rotation_in_base(m.pitch))
    obj, standoff, dt = (0.60, 0.08, 0.01), 0.30, 0.01
    x, y, yaw = start
    cmd = (0.0, 0.0)
    for k in range(int(30.0 / dt)):
        if k % 10 == 0:   # 10 Hz 측정
            p = object_in_base(x, y, yaw, *obj, base_height=0.05)
            obs = observe(p, m, 0.05, 0.02)
            assert obs is not None, f'물체가 화면 밖으로 나감: {p}'
            u, v, z, _, _ = obs
            p_cam = pixel_to_camera(u, v, z, m.fx, m.fy, m.cx, m.cy)
            x_b, y_b, _ = transform_point(p_cam, (m.x, 0.0, m.z), q)
            cmd = compute_cmd(*range_bearing_error(x_b, y_b, standoff), P)
        x, y, yaw = step_unicycle(x, y, yaw, *cmd, dt)

    e_r, e_b = true_error(object_in_base(x, y, yaw, *obj, base_height=0.05), standoff)
    assert abs(e_r) <= P.tol_range and abs(e_b) <= P.tol_bearing
    assert cmd == (0.0, 0.0)
    assert not math.isnan(e_r)
