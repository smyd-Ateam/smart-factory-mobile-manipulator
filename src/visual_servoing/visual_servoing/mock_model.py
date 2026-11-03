"""모의 입력용 차체·카메라 모델. Gazebo 카메라가 붙기 전까지 정렬 노드 시험에 쓴다.

ROS에 의존하지 않는 순수 함수만 둔다.
"""
import math
from dataclasses import dataclass

import numpy as np

from visual_servoing.geometry import camera_to_pixel, optical_rotation_in_base


@dataclass
class CameraMount:
    """base_link 기준 카메라 장착 위치 (임시값, 실제 값은 A·B 확정 후)."""
    x: float = 0.10
    z: float = 0.25
    pitch: float = 0.75          # rad, 양수 = 아래를 봄. 0.6 m ~ 0.3 m 거리 물체가 모두 화면에 들어오는 값
    width: int = 320
    height: int = 240
    fx: float = 277.0            # 수평 화각 약 60°
    fy: float = 277.0

    @property
    def cx(self):
        return self.width / 2.0

    @property
    def cy(self):
        return self.height / 2.0


def step_unicycle(x, y, yaw, vx, wz, dt):
    """차동구동 차체 위치를 dt만큼 적분한다."""
    yaw_new = yaw + wz * dt
    mid = yaw + 0.5 * wz * dt
    return x + vx * math.cos(mid) * dt, y + vx * math.sin(mid) * dt, yaw_new


def object_in_base(robot_x, robot_y, robot_yaw, obj_x, obj_y, obj_z, base_height):
    """odom 좌표 물체 위치 → base_link 좌표."""
    dx, dy = obj_x - robot_x, obj_y - robot_y
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    return (c * dx + s * dy, -s * dx + c * dy, obj_z - base_height)


def observe(p_base, mount, obj_w, obj_h):
    """base_link 좌표 물체 → (u, v, depth, bbox_w, bbox_h). 화면 밖이면 None."""
    r = optical_rotation_in_base(mount.pitch)
    t = np.array([mount.x, 0.0, mount.z])
    xo, yo, zo = r.T @ (np.asarray(p_base, dtype=float) - t)
    if zo < 0.05:
        return None
    u, v = camera_to_pixel(xo, yo, zo, mount.fx, mount.fy, mount.cx, mount.cy)
    if not (0 <= u < mount.width and 0 <= v < mount.height):
        return None
    return u, v, zo, mount.fx * obj_w / zo, mount.fy * obj_h / zo


def true_error(p_base, standoff):
    """실제 위치 기준 거리·방위 오차."""
    return math.hypot(p_base[0], p_base[1]) - standoff, math.atan2(p_base[1], p_base[0])
