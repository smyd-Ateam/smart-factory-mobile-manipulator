"""정렬 오차 계산 (설계 문서 2.1).

ROS에 의존하지 않는 순수 함수만 둔다. 노드와 단위 테스트가 같이 쓴다.
"""
import math

import numpy as np


def decode_depth(encoding, data, height, width, step, is_bigendian=False):
    """sensor_msgs/Image 깊이 데이터를 m 단위 float32 배열로 바꾼다.

    지원 형식: 32FC1 (m), 16UC1 (mm, 0은 측정 없음 → NaN).
    """
    if encoding == '32FC1':
        dtype = np.dtype('>f4' if is_bigendian else '<f4')
    elif encoding == '16UC1':
        dtype = np.dtype('>u2' if is_bigendian else '<u2')
    else:
        raise ValueError(f'지원하지 않는 깊이 형식: {encoding}')

    row_items = step // dtype.itemsize
    arr = np.frombuffer(bytes(data), dtype=dtype).reshape(height, row_items)[:, :width]

    if encoding == '16UC1':
        out = arr.astype(np.float32) / 1000.0
        out[arr == 0] = np.nan
        return out
    return arr.astype(np.float32)


def depth_at(depth, u, v, half_window):
    """(u, v) 주변 (2*half_window+1)^2 영역의 유효 깊이 중앙값. 없으면 None.

    NaN, inf, 0 이하 값은 제외한다 (interfaces.md 1.1).
    """
    h, w = depth.shape
    ui, vi = int(round(u)), int(round(v))
    if not (0 <= ui < w and 0 <= vi < h):
        return None
    u0, u1 = max(0, ui - half_window), min(w, ui + half_window + 1)
    v0, v1 = max(0, vi - half_window), min(h, vi + half_window + 1)
    patch = depth[v0:v1, u0:u1]
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def pixel_to_camera(u, v, z, fx, fy, cx, cy):
    """픽셀 + 깊이 → 카메라 optical 프레임 좌표 (x 오른쪽, y 아래, z 앞)."""
    return ((u - cx) * z / fx, (v - cy) * z / fy, z)


def camera_to_pixel(x, y, z, fx, fy, cx, cy):
    """pixel_to_camera의 역변환."""
    return (fx * x / z + cx, fy * y / z + cy)


def quat_to_matrix(qx, qy, qz, qw):
    """쿼터니언 → 3x3 회전 행렬."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def matrix_to_quat(m):
    """3x3 회전 행렬 → 쿼터니언 (x, y, z, w)."""
    m = np.asarray(m, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    return (qx, qy, qz, qw)


def transform_point(point, translation, quaternion):
    """부모 ← 자식 변환 (translation, quaternion xyzw)으로 자식 좌표 점을 부모 좌표로 바꾼다."""
    r = quat_to_matrix(*quaternion)
    return tuple(r @ np.asarray(point, dtype=float) + np.asarray(translation, dtype=float))


def range_bearing_error(x_b, y_b, standoff):
    """base_link 좌표 (x_b, y_b)의 물체에 대한 거리 오차, 방위 오차.

    e_r > 0: 물체가 standoff보다 멀다 (전진 필요).
    e_b > 0: 물체가 왼쪽에 있다 (반시계 회전 필요, REP 103).
    """
    e_r = math.hypot(x_b, y_b) - standoff
    e_b = math.atan2(y_b, x_b)
    return e_r, e_b


def optical_rotation_in_base(pitch):
    """base_link 기준 camera_optical_frame 회전 행렬.

    camera_link는 base_link에서 y축으로 pitch만큼 숙인 자세 (양수 = 아래를 봄),
    optical 프레임은 camera_link 기준 x 오른쪽 = -y, y 아래 = -z, z 앞 = x.
    """
    c, s = math.cos(pitch), math.sin(pitch)
    r_base_cam = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    r_cam_opt = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    return r_base_cam @ r_cam_opt
