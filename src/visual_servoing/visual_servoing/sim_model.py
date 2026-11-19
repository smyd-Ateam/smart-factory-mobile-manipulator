"""Gazebo 시험(align_sim.sdf)용 순수 함수. ROS에 의존하지 않는다."""
import math

# feat/yolo perception/synthetic_data/README.md: Gazebo label = YOLO class id + 1 (0은 배경)
LABEL_TO_CLASS = {1: 'block', 2: 'pin', 3: 'hex_nut', 4: 'bolt'}


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
