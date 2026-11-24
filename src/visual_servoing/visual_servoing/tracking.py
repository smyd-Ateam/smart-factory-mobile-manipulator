"""같은 클래스 물체가 여러 개일 때 정렬 목표 하나를 따라가는 규칙. ROS에 의존하지 않는다.

- 첫 프레임: base_link 기준 방위가 가장 작은 검출 (차체 정면에 가장 가까운 물체).
  Nav2가 차체를 물체 쪽으로 세우므로 정렬할 물체가 차체 정면 근처에 있다.
  화면 가운데로 고르면 안 된다: 카메라가 차체보다 앞(약 0.26 m)에 있어서 화면 가운데와
  차체 정면이 다르다 (Gazebo 방해물 시험에서 10회 중 1회 옆 물체를 고름, 설계 문서 5.3).
- 이후: 직전에 고른 박스 중심에 가장 가까운 검출. 점수가 더 높은 다른 물체로 갈아타지 않는다.
- 직전 위치에서 gate_px보다 멀면 놓친 것으로 본다 (다른 물체로 갈아타지 않음).
- 상태머신이 목표의 예상 위치를 주면 첫 프레임은 방위 대신 예상 위치 반경 안의 가장 가까운
  검출을 고른다. 반경 안에 없거나 두 후보가 구분되지 않으면 고르지 않는다 (설계 문서 8장).
"""
import math
from typing import NamedTuple, Optional, Sequence, Tuple

import numpy as np

from visual_servoing.geometry import quat_to_matrix


class Candidate(NamedTuple):
    u: float
    v: float
    score: float


def base_bearing(u: float, v: float, k: Tuple[float, float, float, float], cam_t, cam_q,
                 z_plane: float) -> float:
    """픽셀 (u, v) 광선이 높이 z_plane(base_link)에 닿는 점의 방위 (rad). 깊이 없이 쓴다.

    광선이 그 높이로 내려가지 않으면 (위를 봄) 광선 수평 방향의 방위를 쓴다.
    """
    p = base_plane_point(u, v, k, cam_t, cam_q, z_plane)
    if p is not None:
        return math.atan2(p[1], p[0])
    fx, fy, cx, cy = k
    ray = quat_to_matrix(*cam_q) @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    return math.atan2(ray[1], ray[0])


def base_plane_point(u: float, v: float, k: Tuple[float, float, float, float], cam_t, cam_q,
                     z_plane: float) -> Optional[Tuple[float, float]]:
    """픽셀 (u, v) 광선이 높이 z_plane(base_link)에 닿는 점의 (x, y). 위를 보는 광선이면 None."""
    fx, fy, cx, cy = k
    ray = quat_to_matrix(*cam_q) @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    t = np.asarray(cam_t, dtype=float)
    if ray[2] >= -1e-9:
        return None
    p = t + (z_plane - t[2]) / ray[2] * ray
    return float(p[0]), float(p[1])


def expected_goal_is_valid(frame_id: str, radius_m: float) -> bool:
    """Align 목표의 예상 위치 필드 검사. frame_id가 비면 예상 위치를 쓰지 않으므로 항상 유효."""
    return not frame_id or radius_m > 0.0


def pick_near_expected(points: Sequence[Optional[Tuple[float, float]]],
                       expected_xy: Tuple[float, float], radius_m: float,
                       ambiguity_m: float) -> Optional[int]:
    """예상 위치 반경 안에서 가장 가까운 후보의 인덱스. 고를 수 없으면 None.

    points: 후보별 base_link (x, y). 위치를 못 구한 후보는 None.
    반경 안 두 번째 후보와의 거리 차이가 ambiguity_m보다 작으면 어느 쪽인지 확신할 수 없어
    고르지 않는다 (엉뚱한 물체에 정렬하는 것보다 실패가 낫다).
    """
    inside = sorted(
        (math.hypot(p[0] - expected_xy[0], p[1] - expected_xy[1]), i)
        for i, p in enumerate(points) if p is not None)
    inside = [(d, i) for d, i in inside if d <= radius_m]
    if not inside:
        return None
    if len(inside) > 1 and inside[1][0] - inside[0][0] < ambiguity_m:
        return None
    return inside[0][1]


def pick_target(candidates: Sequence[Candidate], last_uv: Optional[Tuple[float, float]],
                gate_px: float, first_cost: Sequence[float]) -> Optional[int]:
    """고른 검출의 인덱스. 고를 것이 없으면 None.

    first_cost: 첫 프레임(last_uv 없음)에서 쓸 후보별 비용 (작을수록 우선, 보통 |방위|).
    """
    if not candidates:
        return None
    if last_uv is None:
        return min(range(len(candidates)), key=lambda i: first_cost[i])
    dist = [math.hypot(c.u - last_uv[0], c.v - last_uv[1]) for c in candidates]
    best = min(range(len(candidates)), key=dist.__getitem__)
    return best if dist[best] <= gate_px else None
