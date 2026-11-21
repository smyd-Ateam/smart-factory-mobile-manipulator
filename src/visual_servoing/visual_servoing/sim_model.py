"""Gazebo 시험(align_sim.sdf)용 순수 함수. ROS에 의존하지 않는다."""
import math
from dataclasses import dataclass

# feat/yolo perception/synthetic_data/README.md: Gazebo label = YOLO class id + 1 (0은 배경)
LABEL_TO_CLASS = {1: 'block', 2: 'pin', 3: 'hex_nut', 4: 'bolt'}


@dataclass(frozen=True)
class CameraPose:
    """base_link 기준 카메라 위치 (x, z m)와 숙임각 pitch (rad, 양수 = 아래)."""
    x: float
    z: float
    pitch: float


# test: 임시값 (mock_model.CameraMount와 같음)
# near, pregrasp: feat/grasp 5741526 URDF에 관측 자세 관절값을 넣어 계산한 값.
#   near는 B 실측 TF (0.260, 0.001, 0.206), 약 58° 아래와 같다.
#   pregrasp는 11/21에 바뀐 69.3° 자세 (joint4 -1.190737). y 0.0008 m는 무시
CAMERA_POSES = {
    'test': CameraPose(0.10, 0.25, 0.75),
    'near': CameraPose(0.2595, 0.2061, math.radians(57.87)),
    'pregrasp': CameraPose(0.3035, 0.1500, math.radians(69.18)),
}


def sdf_pose(pose):
    """SDF <pose> 문자열 (카메라 링크: x 앞, pitch 양수 = 아래)."""
    return f'{pose.x:.4f} 0 {pose.z:.4f} 0 {pose.pitch:.4f} 0'


def label_to_class(class_id):
    """boundingbox 카메라 label 문자열('2') → 클래스 이름('pin'). 모르는 label이면 None."""
    try:
        return LABEL_TO_CLASS.get(int(class_id))
    except (TypeError, ValueError):
        return None


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def set_pose_request(model, x, y, z, yaw):
    """ign service /world/<w>/set_pose 에 넘길 ignition.msgs.Pose 문자열."""
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return (f'name: "{model}", position: {{x: {x:.6f}, y: {y:.6f}, z: {z:.6f}}}, '
            f'orientation: {{x: 0, y: 0, z: {qz:.9f}, w: {qw:.9f}}}')
