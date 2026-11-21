"""깊이로 잰 점 → 물체 중심축 위치. ROS에 의존하지 않는다.

깊이는 bbox 중심 픽셀이 닿은 물체 표면까지 거리다. 옆에서 보면 옆면(중심보다 반지름만큼 앞),
위에서 보면 윗면(중심 근처)에 닿는다. 그래서 고정값을 더하지 않고, 물체를 세워진 원기둥
(반지름, 높이)으로 보고 "중심축이 여기 있으면 bbox 중심 광선이 어디에 닿는지"를 계산해
잰 점과 맞는 중심축 위치를 찾는다 (설계 문서 5.3).

좌표: base_link. floor_z = 바닥 높이 (base_link가 바닥에서 0.05 m 위면 -0.05).
"""
import math
from typing import NamedTuple, Optional, Sequence, Tuple

import numpy as np

from visual_servoing.geometry import quat_to_matrix

_RING = 48            # 둘레 표본 수
_LEVELS = 9           # 높이 표본 수
_ITERATIONS = 4       # 중심축 위치 반복 보정 횟수


class Cylinder(NamedTuple):
    radius: float
    height: float


def parse_object_shapes(items: Sequence[str]) -> dict:
    """['pin:0.010:0.05', ...] → {'pin': Cylinder(0.010, 0.05)}. 틀리면 ValueError."""
    out = {}
    for item in items:
        parts = [p.strip() for p in item.split(':')]
        if len(parts) != 3 or not parts[0]:
            raise ValueError(f'object_shapes 형식은 "클래스:반지름:높이" (m): {item!r}')
        radius, height = float(parts[1]), float(parts[2])
        if radius < 0.0 or height <= 0.0:
            raise ValueError(f'object_shapes 반지름은 0 이상, 높이는 0 초과: {item!r}')
        out[parts[0]] = Cylinder(radius, height)
    return out


def _surface(axis_xy, shape, floor_z):
    a = np.linspace(0.0, 2.0 * math.pi, _RING, endpoint=False)
    z = np.linspace(floor_z, floor_z + shape.height, _LEVELS)
    aa, zz = np.meshgrid(a, z)
    return np.stack([axis_xy[0] + shape.radius * np.cos(aa).ravel(),
                     axis_xy[1] + shape.radius * np.sin(aa).ravel(), zz.ravel()], axis=1)


def _ray_hit(origin, ray, axis_xy, shape, floor_z):
    """광선이 원기둥(옆면, 윗면)에 처음 닿는 점. 안 닿으면 None."""
    hits = []
    ox, oy = origin[0] - axis_xy[0], origin[1] - axis_xy[1]
    a = ray[0] ** 2 + ray[1] ** 2
    if a > 1e-12:
        b = 2.0 * (ox * ray[0] + oy * ray[1])
        c = ox * ox + oy * oy - shape.radius ** 2
        disc = b * b - 4.0 * a * c
        if disc >= 0.0:
            for t in ((-b - math.sqrt(disc)) / (2 * a), (-b + math.sqrt(disc)) / (2 * a)):
                z = origin[2] + t * ray[2]
                if t > 0.0 and floor_z <= z <= floor_z + shape.height:
                    hits.append(t)
    if abs(ray[2]) > 1e-12:
        t = (floor_z + shape.height - origin[2]) / ray[2]
        p = origin + t * ray
        if t > 0.0 and (p[0] - axis_xy[0]) ** 2 + (p[1] - axis_xy[1]) ** 2 <= shape.radius ** 2:
            hits.append(t)
    return origin + min(hits) * ray if hits else None


def bbox_center_hit(axis_xy, shape: Cylinder, floor_z: float, cam_t, cam_q,
                    k: Tuple[float, float, float, float]) -> Optional[np.ndarray]:
    """중심축이 axis_xy인 원기둥의 bbox 중심 픽셀 광선이 닿는 점 (base_link). 화면 뒤면 None."""
    fx, fy, cx, cy = k
    r = quat_to_matrix(*cam_q)
    cam_t = np.asarray(cam_t, dtype=float)
    q = (_surface(axis_xy, shape, floor_z) - cam_t) @ r     # 광학 좌표 (행 = 점)
    if np.any(q[:, 2] <= 1e-6):
        return None
    u, v = fx * q[:, 0] / q[:, 2] + cx, fy * q[:, 1] / q[:, 2] + cy
    uc, vc = (u.min() + u.max()) / 2.0, (v.min() + v.max()) / 2.0
    ray = r @ np.array([(uc - cx) / fx, (vc - cy) / fy, 1.0])
    return _ray_hit(cam_t, ray, axis_xy, shape, floor_z)


def axis_from_hit(hit_b, shape: Cylinder, floor_z: float, cam_t, cam_q,
                  k: Tuple[float, float, float, float]) -> Tuple[float, float]:
    """잰 점(bbox 중심 깊이, base_link) → 원기둥 중심축 (x, y).

    중심축을 가정 → 그때 닿을 점을 계산 → 잰 점과의 차이만큼 중심축을 옮기기를 반복한다.
    """
    hit = np.asarray(hit_b, dtype=float)
    if shape.radius <= 0.0:
        return float(hit[0]), float(hit[1])
    d = hit[:2] - np.asarray(cam_t, dtype=float)[:2]
    n = float(np.hypot(*d))
    axis = hit[:2] + (shape.radius * d / n if n > 1e-9 else 0.0)   # 첫 추정: 옆면에 닿았다고 봄
    for _ in range(_ITERATIONS):
        predicted = bbox_center_hit(axis, shape, floor_z, cam_t, cam_q, k)
        if predicted is None:
            break
        axis = axis + (hit[:2] - predicted[:2])
    return float(axis[0]), float(axis[1])
