"""베이스 정렬 P제어 (설계 문서 2.2).

ROS에 의존하지 않는 순수 함수만 둔다.
"""
from dataclasses import dataclass


@dataclass
class ControlParams:
    k_v: float = 1.0             # 거리 오차 이득
    k_w: float = 1.0             # 방위 오차 이득
    max_v: float = 0.05          # m/s
    max_w: float = 0.30          # rad/s
    bearing_gate: float = 0.20   # rad, 이보다 방위가 틀어지면 전진하지 않고 회전만
    tol_range: float = 0.02      # m (interfaces.md 5.1)
    tol_bearing: float = 0.05    # rad (interfaces.md 5.1)
    # 허용 범위의 이 비율 안으로 들어와야 정지한다. P제어는 경계에 닿자마자 멈추므로
    # 1.0이면 최종 오차가 허용 범위 끝에 걸려 노이즈에 판정이 흔들린다.
    stop_ratio: float = 0.5


def clamp(x, limit):
    return max(-limit, min(limit, x))


def within_tolerance(e_r, e_b, p, ratio=1.0):
    return abs(e_r) <= p.tol_range * ratio and abs(e_b) <= p.tol_bearing * ratio


def compute_cmd(e_r, e_b, p):
    """거리 오차, 방위 오차 → (vx, wz).

    허용 범위 * stop_ratio 안이면 정지한다.
    정지 후 프레임으로 완료를 판정하는 것(2.3)은 4주차.
    """
    if within_tolerance(e_r, e_b, p, p.stop_ratio):
        return 0.0, 0.0
    wz = clamp(p.k_w * e_b, p.max_w)
    vx = 0.0 if abs(e_b) > p.bearing_gate else clamp(p.k_v * e_r, p.max_v)
    return vx, wz
