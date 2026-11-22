"""카메라 프레임 상태 판정 (진단·시험 스크립트 공용, ROS 의존 없음).

ogre2 렌더링 버그로 카메라 화면 전체가 한 가지 색(회색)으로 나오는 프레임을 찾는다.
판정: 4픽셀 간격 표본에서 픽셀 표준편차 < 2.0 이고 색 개수 <= 4 이면 회색 프레임.
  (Mac VM 실측: 회색 프레임은 std 0, 색 1개, 밝기 218. 정상 프레임은 std 69 이상)
같은 stamp 깊이 이미지의 유효 비율이 0이면 렌더링 실패, 유효하면 무늬 없는 면일 수 있다.

sensor_msgs/Image 메시지는 height, width, step, encoding, data 속성만 쓰므로
rclpy 없이도 (numpy 배열이나 같은 속성을 가진 객체로) 시험할 수 있다.

사용 예:
    import camera_health as ch
    r = ch.check_image_msg(msg)          # {'std', 'unique_colors', 'mean', 'gray'} 또는 None
    v = ch.depth_valid_ratio(depth_msg)  # 0~1 또는 None
"""
import numpy as np

STD_MAX = 2.0      # 이 값 미만이고
UNIQUE_MAX = 4     # 색 개수가 이 값 이하이면 회색 프레임
STEP = 4           # 표본 간격 (px)
DEPTH_EMPTY_MAX = 0.05   # 깊이 유효 비율이 이 값 미만이면 "깊이도 비어 있음"

CHANNELS = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
RULE = f'std < {STD_MAX} and unique_colors <= {UNIQUE_MAX} ({STEP}px 간격 표본)'


def image_msg_to_sample(msg, step=STEP):
    """Image 메시지 → (H/step, W/step, C) uint8 표본 배열. 지원하지 않는 인코딩이면 None."""
    ch = CHANNELS.get(msg.encoding)
    if ch is None:
        return None
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch]
    return a.reshape(msg.height, msg.width, ch)[::step, ::step, :min(ch, 3)]


def frame_stats(sample):
    """표본 배열 → (std, 색 개수, 평균 밝기)."""
    std = float(sample.std())
    flat = sample.reshape(-1, sample.shape[2]).astype(np.uint32)
    packed = flat[:, 0] << 16
    if flat.shape[1] > 1:
        packed = packed | flat[:, 1] << 8
    if flat.shape[1] > 2:
        packed = packed | flat[:, 2]
    return std, int(np.unique(packed).size), float(sample.mean())


def is_gray(std, unique_colors, std_max=STD_MAX, unique_max=UNIQUE_MAX):
    return std < std_max and unique_colors <= unique_max


def check_image_msg(msg, step=STEP):
    """Image 메시지 하나 판정. 지원하지 않는 인코딩이면 None."""
    s = image_msg_to_sample(msg, step)
    if s is None:
        return None
    std, uniq, mean = frame_stats(s)
    return {'std': round(std, 2), 'unique_colors': uniq, 'mean': round(mean, 1), 'gray': is_gray(std, uniq)}


def depth_valid_ratio(msg, step=STEP):
    """32FC1 깊이 메시지의 유효(유한하고 0보다 큰) 픽셀 비율. 다른 인코딩이면 None."""
    if msg.encoding != '32FC1':
        return None
    d = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width][::step, ::step]
    return float((np.isfinite(d) & (d > 0)).mean())


def gray_kind(depth_valid):
    """회색 프레임의 종류 설명 (같은 stamp 깊이 유효 비율 기준)."""
    if depth_valid is None:
        return '깊이 정보 없음'
    return '깊이도 비어 있음 → 렌더링 실패로 보임' if depth_valid < DEPTH_EMPTY_MAX else '깊이는 유효 → 무늬 없는 면일 수 있음'
