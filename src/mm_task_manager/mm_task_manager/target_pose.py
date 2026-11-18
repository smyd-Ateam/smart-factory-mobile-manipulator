"""Grasp에 넘길 /perception/target_pose 선택 (rclpy 없음).

interfaces.md 6장: 정렬 완료 후 받은 값 중 stamp 기준 max_age(1.0 s) 이내.
interfaces.md 3.3: stamp 기준 1 s 넘게 새 값이 없으면 미검출 → 상태머신은 OBJECT_NOT_FOUND / target_pose_stale.
시각은 모두 sim time 초. stamp는 C가 이미지를 찍은 시각.

받는 쪽 안전장치 (C 제안, PR #1 댓글 2026-11-18): is_valid_target_pose를 통과 못 한 메시지는
노드가 아예 무시한다 (Grasp 목표 후보에도, Near→Pre-grasp 거리 판단에도 안 씀).
"""

import math
from dataclasses import dataclass

TARGET_POSE_FRAME = 'base_link'   # interfaces.md 3장: /perception/target_pose는 base_link 기준

SEND = 'send'      # 이 값으로 Grasp 목표를 보냄
WAIT = 'wait'      # 아직 쓸 값이 없음, 더 기다림
STALE = 'stale'    # max_age가 지나도록 쓸 값이 없음 → 미검출


@dataclass(frozen=True)
class GraspPoseRequest:
    token: int
    since: float          # 정렬이 끝난 시각 (이보다 먼저 찍힌 값은 정렬 중 값이라 안 씀)
    max_age_sec: float

    def check(self, latest_stamp, now):
        if (latest_stamp is not None and latest_stamp >= self.since
                and now - latest_stamp <= self.max_age_sec):
            return SEND
        if now - self.since > self.max_age_sec:
            return STALE
        return WAIT


def is_valid_target_pose(frame_id, x, y, z):
    """/perception/target_pose를 써도 되면 True.

    - frame_id가 정확히 'base_link'가 아니면 거름 (변환하지 않음)
    - x, y, z 중 NaN · inf가 있으면 거름
    - x <= 0이면 거름: 카메라가 항상 앞을 봐서 차체 뒤쪽 물체는 나올 수 없음
    """
    if frame_id != TARGET_POSE_FRAME:
        return False
    if not all(math.isfinite(v) for v in (x, y, z)):
        return False
    return x > 0.0
