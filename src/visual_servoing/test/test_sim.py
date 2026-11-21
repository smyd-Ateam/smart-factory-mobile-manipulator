"""Gazebo 시험 월드·보조 함수 테스트 (Gazebo 없이 실행)."""
import math
import pathlib
import xml.etree.ElementTree as ET

import pytest

from visual_servoing.mock_model import CameraMount
from visual_servoing.sim_model import (CAMERA_POSES, label_to_class, sdf_pose,
                                       set_pose_request, yaw_from_quat)

WORLD = pathlib.Path(__file__).resolve().parents[1] / 'worlds' / 'align_sim.sdf'


def sensor_poses():
    root = ET.parse(WORLD).getroot()
    link = root.find(".//model[@name='mm_test_base']/link[@name='base_link']")
    return {s.get('name'): [float(v) for v in s.find('pose').text.split()]
            for s in link.findall('sensor')}


def test_world_camera_matches_test_pose():
    """launch가 바꿔 끼우는 월드 카메라 pose가 test 값과 같아야 한다 (static TF와 일치)."""
    m = CAMERA_POSES['test']
    poses = sensor_poses()
    assert set(poses) == {'rgbd', 'bbox'}
    for pose in poses.values():
        assert pose == pytest.approx([m.x, 0.0, m.z, 0.0, m.pitch, 0.0])
    assert WORLD.read_text(encoding='utf-8').count(sdf_pose(m)) == 2


def test_test_pose_matches_mock_mount():
    m, t = CameraMount(), CAMERA_POSES['test']
    assert (t.x, t.z, t.pitch) == pytest.approx((m.x, m.z, m.pitch))


def test_team_camera_poses_look_down():
    # B 실측 near TF (0.260, 0.001, 0.206), 약 58° 아래
    near = CAMERA_POSES['near']
    assert (near.x, near.z) == pytest.approx((0.260, 0.206), abs=0.002)
    assert math.degrees(near.pitch) == pytest.approx(57.9, abs=0.2)
    assert math.degrees(CAMERA_POSES['pregrasp'].pitch) == pytest.approx(69.3, abs=0.2)


def test_world_target_label_is_pin():
    root = ET.parse(WORLD).getroot()
    label = root.find(".//model[@name='pin_target']/plugin/label")
    assert label_to_class(label.text) == 'pin'


@pytest.mark.parametrize('cid, name', [('1', 'block'), ('2', 'pin'), ('3', 'hex_nut'),
                                       ('4', 'bolt'), ('0', None), ('x', None), (None, None)])
def test_label_to_class(cid, name):
    assert label_to_class(cid) == name


@pytest.mark.parametrize('yaw', [0.0, 0.25, -1.2, 3.0])
def test_yaw_from_quat(yaw):
    assert yaw_from_quat(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) == pytest.approx(yaw)


def test_set_pose_request():
    req = set_pose_request('mm_test_base', 0.1, -0.2, 0.05, math.pi / 2)
    assert req.startswith('name: "mm_test_base"')
    assert 'x: 0.100000, y: -0.200000, z: 0.050000' in req
    assert f'z: {math.sin(math.pi / 4):.9f}' in req
