import math
from collections import deque

import numpy as np
import pytest

from visual_servoing.control import ControlParams
from visual_servoing.geometry import (matrix_to_quat, optical_rotation_in_base,
                                      pixel_to_camera, transform_point)
from visual_servoing.mock_model import (CameraMount, object_in_base, observe,
                                        step_unicycle, true_error)
from visual_servoing.session import (ALIGN_TIMEOUT, FAILED, OBJECT_NOT_FOUND, RUNNING,
                                     SUCCEEDED, AlignSession)

CP = ControlParams()
STANDOFF = 0.25


def simulate(start, delay=0.0, noise_px=0.0, noise_depth=0.0, timeout=60.0,
             hide_after=None, obj=(0.0, 0.0, 0.01), seed=0):
    """모의 차체 + 모의 카메라(10 Hz, 지연) + AlignSession(20 Hz) 폐루프."""
    rng = np.random.default_rng(seed)
    m = CameraMount()
    q = matrix_to_quat(optical_rotation_in_base(m.pitch))
    x, y, yaw = start
    dt, t = 0.01, 0.0
    sess = AlignSession(STANDOFF, timeout, 0.0)
    pending, latest = deque(), None
    cmd = (0.0, 0.0)
    while t < timeout + 5:
        k = round(t / dt)
        if k % 10 == 0:   # 10 Hz 촬영
            p = object_in_base(x, y, yaw, *obj, base_height=0.05)
            o = observe(p, m, 0.05, 0.02)
            if o is not None and not (hide_after is not None and t >= hide_after):
                u = o[0] + (rng.normal(0, noise_px) if noise_px else 0)
                v = o[1] + (rng.normal(0, noise_px) if noise_px else 0)
                z = o[2] + (rng.normal(0, noise_depth) if noise_depth else 0)
                pc = pixel_to_camera(u, v, z, m.fx, m.fy, m.cx, m.cy)
                xb, yb, _ = transform_point(pc, (m.x, 0.0, m.z), q)
                pending.append((t + delay, (t, xb, yb)))
        while pending and pending[0][0] <= t + 1e-9:
            latest = pending.popleft()[1]
        if k % 5 == 0:    # 20 Hz 제어
            r = sess.tick(t, latest)
            cmd = (r.vx, r.wz)
            if r.status != RUNNING:
                p = object_in_base(x, y, yaw, *obj, base_height=0.05)
                return r, true_error(p, STANDOFF), t
        x, y, yaw = step_unicycle(x, y, yaw, *cmd, dt)
        t += dt
    raise AssertionError('세션이 끝나지 않음')


@pytest.mark.parametrize('delay,noise_px,noise_depth', [
    (0.0, 0.0, 0.0), (0.2, 2.0, 0.005), (0.5, 2.0, 0.005), (0.8, 2.0, 0.005)])
def test_success_true_error_within_tolerance(delay, noise_px, noise_depth):
    for i, start in enumerate([(-0.40, 0.08, 0.10), (-0.38, -0.10, -0.15), (-0.45, 0.0, 0.0)]):
        r, (e_r, e_b), _ = simulate(start, delay, noise_px, noise_depth, seed=i)
        assert r.status == SUCCEEDED, (start, r)
        assert r.error_code == ''
        assert abs(e_r) <= CP.tol_range and abs(e_b) <= CP.tol_bearing, (start, e_r, e_b)


def test_success_only_after_stop_frames():
    """성공은 정지 후 찍힌 프레임 3장으로 판정한다 → 성공 시점의 실제 오차가 허용 범위 안."""
    r, (e_r, e_b), t = simulate((-0.40, 0.08, 0.10), delay=0.5)
    assert r.status == SUCCEEDED
    assert abs(e_r) <= CP.tol_range


def test_timeout():
    r, _, t = simulate((-0.45, 0.10, 0.15), timeout=2.0)
    assert r.status == FAILED and r.error_code == ALIGN_TIMEOUT
    assert (r.vx, r.wz) == (0.0, 0.0)


def test_not_found_at_start():
    # 물체가 뒤쪽에 있어 처음부터 안 보임
    r, _, t = simulate((0.40, 0.0, 0.0))
    assert r.status == FAILED and r.error_code == OBJECT_NOT_FOUND
    assert t == pytest.approx(1.05, abs=0.06)


def test_delay_alone_is_not_lost():
    """지연 0.8 s여도 측정이 계속 들어오면 놓침으로 보지 않는다."""
    r, _, _ = simulate((-0.40, 0.05, 0.05), delay=0.8)
    assert r.status == SUCCEEDED


def test_lost_during_alignment():
    r, _, t = simulate((-0.45, 0.05, 0.10), hide_after=2.0)
    assert r.status == FAILED and r.error_code == OBJECT_NOT_FOUND
    assert 2.9 < t < 3.3
    assert (r.vx, r.wz) == (0.0, 0.0)


def test_stale_measurement_stops_robot():
    s = AlignSession(STANDOFF, 30.0, 0.0)
    r = s.tick(1.5, (0.3, 0.6, 0.0))   # 1.2 s 지연된 측정 (max_age 1.0 초과)
    assert r.status == RUNNING and (r.vx, r.wz) == (0.0, 0.0) and r.phase == 'WAIT_FRESH'


def test_old_measurement_from_before_goal_is_ignored():
    s = AlignSession(STANDOFF, 30.0, 5.0)
    r = s.tick(5.2, (4.9, 0.6, 0.0))
    assert r.status == RUNNING and r.phase == 'WAIT_DETECTION' and (r.vx, r.wz) == (0.0, 0.0)


def test_resume_when_off_after_stop():
    s = AlignSession(STANDOFF, 30.0, 0.0)
    r = s.tick(0.0, (0.0, STANDOFF + 0.005, 0.0))     # 정지 범위 안 → 정지
    assert r.phase == 'STOPPING'
    r = s.tick(0.3, (0.25, STANDOFF + 0.05, 0.0))     # 정지 후 보니 5 cm 멀다 → 재개
    assert r.phase == 'MOVING' and r.vx > 0


def test_same_frame_not_counted_twice():
    s = AlignSession(STANDOFF, 30.0, 0.0)
    s.tick(0.0, (0.0, STANDOFF, 0.0))
    for now in (0.20, 0.25, 0.30, 0.35):              # 같은 측정을 여러 tick에서 받음
        r = s.tick(now, (0.15, STANDOFF, 0.0))
    assert r.status == RUNNING and s.good_frames == 1
    s.tick(0.40, (0.25, STANDOFF, 0.0))
    r = s.tick(0.50, (0.35, STANDOFF, 0.0))
    assert r.status == SUCCEEDED
    assert math.isclose(r.e_r, 0.0, abs_tol=1e-9)
