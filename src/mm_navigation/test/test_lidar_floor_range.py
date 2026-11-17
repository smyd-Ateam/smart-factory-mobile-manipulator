"""SLAM max_laser_range가 차체 앞 기울기로 라이다 빔이 바닥에 닿는 거리보다 충분히 짧은지.

차동구동 차체는 바퀴(x=0)를 축으로 캐스터 간극만큼 앞으로 기운다: tilt = atan(간극 / 캐스터 x).
라이다 빔은 그만큼 아래를 향해 바닥에 닿는다: 거리 = 기운 상태의 라이다 높이 / tan(tilt).
치수는 mm_description xacro를 직접 펼쳐서 읽는다 (차체 치수가 바뀌면 이 테스트가 다시 확인).
2026-11-17 Gazebo 측정(chassis_z 0.10, caster_gap 0.002): pitch 0.01335 rad, 라이다 base_footprint 기준
(0.130, 0.165) m → 계산값(0.01333 rad, 바닥 12.2 m)과 일치.
"""

import math
import os
import xml.etree.ElementTree as ET

import xacro
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
XACRO = os.path.join(HERE, '..', '..', 'mm_description', 'urdf', 'mm_base.urdf.xacro')
SLAM_PARAMS = os.path.join(HERE, '..', 'config', 'slam_params.yaml')

# max_laser_range와 바닥 접촉 거리 사이에 남길 최소 여유 (m)
MARGIN_M = 1.0


def _origin(robot, joint_name):
    o = robot.find(f"joint[@name='{joint_name}']/origin").get('xyz').split()
    return [float(v) for v in o]


def floor_hit_distance(xacro_path=XACRO):
    """(기울기 rad, 기운 상태 라이다 높이 m, 라이다 ~ 바닥 접촉 수평거리 m)."""
    robot = ET.fromstring(xacro.process_file(xacro_path).toxml())
    base_z = _origin(robot, 'base_joint')[2]                  # base_footprint → base_link
    wheel_x = _origin(robot, 'left_wheel_joint')[0]
    caster_x, _, caster_z = _origin(robot, 'front_caster_joint')
    caster_r = float(robot.find(
        "link[@name='front_caster_link']/collision/geometry/sphere").get('radius'))
    lidar_x, _, lidar_z = _origin(robot, 'base_scan_joint')

    gap = base_z + caster_z - caster_r                       # 수평일 때 앞 캐스터 ~ 바닥
    tilt = math.atan2(gap, caster_x - wheel_x)
    height = (base_z + lidar_z) * math.cos(tilt) - (lidar_x - wheel_x) * math.sin(tilt)
    return tilt, height, height / math.tan(tilt)


def max_laser_range(path=SLAM_PARAMS):
    with open(path) as f:
        return float(yaml.safe_load(f)['slam_toolbox']['ros__parameters']['max_laser_range'])


def test_current_dimensions_match_gazebo_measurement():
    tilt, height, _ = floor_hit_distance()
    assert abs(tilt - 0.01335) < 0.0005
    assert abs(height - 0.1632) < 0.002


def test_max_laser_range_stays_short_of_floor_hit():
    _, _, floor = floor_hit_distance()
    assert max_laser_range() + MARGIN_M <= floor, (
        f'max_laser_range {max_laser_range()} m + {MARGIN_M} m > floor hit {floor:.2f} m')
